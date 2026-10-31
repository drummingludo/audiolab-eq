#include "EqDisplay.h"

using namespace audiolab;
using namespace audiolab::eq;
namespace P = audiolab::params;
using juce::String;

static String U (const char* s) { return String::fromUTF8 (s); }

EqDisplay::EqDisplay (Theme& t, AudioLabEqProcessor& p) : theme (t), proc (p)
{
    setWantsKeyboardFocus (true);
    setTitle (U ("Courbe d'égalisation"));
    fftBuf.assign ((size_t) fftSize * 2, 0.0f);
    window.resize ((size_t) fftSize);
    for (int i = 0; i < fftSize; ++i)
        window[(size_t) i] = (float) (0.5 - 0.5 * std::cos (2.0 * pi * i / (fftSize - 1)));
    startTimerHz (30);
}

EqDisplay::~EqDisplay() { stopTimer(); endEdit(); if (soloHeld) proc.setSolo (-1); }

// ── Géométrie ───────────────────────────────────────────────────────────
juce::Rectangle<float> EqDisplay::plot() const
{
    return { 0.0f, 0.0f, (float) getWidth() - 50.0f, (float) getHeight() - 24.0f };
}
juce::Rectangle<float> EqDisplay::pianoArea() const
{
    auto r = plot();
    return { r.getX(), r.getBottom() + 5.0f, r.getWidth(), 17.0f };
}
juce::Rectangle<float> EqDisplay::rangeButton() const
{
    auto r = plot();
    return { r.getRight() + 6.0f, r.getY() + 2.0f, 40.0f, 18.0f };
}
double EqDisplay::range() const { return (double) (int) proc.ui.getProperty ("dbRange", 12); }
float EqDisplay::xOf (double f) const
{
    auto r = plot();
    return r.getX() + r.getWidth() * (float) (std::log (f / fMin) / std::log (fMax / fMin));
}
double EqDisplay::fOf (float x) const
{
    auto r = plot();
    return fMin * std::pow (fMax / fMin, (double) ((x - r.getX()) / r.getWidth()));
}
float EqDisplay::yOf (double db) const
{
    auto r = plot().reduced (0, 12);
    return r.getCentreY() - (float) (db / range()) * r.getHeight() * 0.5f;
}
double EqDisplay::dbOf (float y) const
{
    auto r = plot().reduced (0, 12);
    return (r.getCentreY() - y) / (r.getHeight() * 0.5f) * range();
}
float EqDisplay::yAn (double db) const
{
    auto r = plot();
    return r.getY() + (float) ((6.0 - db) / 96.0) * r.getHeight();
}

void EqDisplay::resized()
{
    numCols = std::max (2, (int) plot().getWidth());
    colFreq.resize ((size_t) numCols);
    for (int i = 0; i < numCols; ++i) colFreq[(size_t) i] = fOf (plot().getX() + i + 0.5f);
    for (auto& c : bandCurve) c.assign ((size_t) numCols, 0.0f);
    totalCurve.assign ((size_t) numCols, 0.0f);
    curveB.assign ((size_t) numCols, 0.0f);
    anPre.assign ((size_t) numCols, -120.0f);
    anPost.assign ((size_t) numCols, -120.0f);
    sketch.assign ((size_t) numCols, std::numeric_limits<float>::quiet_NaN());
    dirty = true;
}

// ── Mise à jour ─────────────────────────────────────────────────────────
void EqDisplay::timerCallback()
{
    // retire de la sélection les bandes supprimées (automation, annuler…)
    bool selChanged = false;
    for (int i = selection.size(); --i >= 0;)
        if (! proc.isUsed (selection[i])) { selection.remove (i); selChanged = true; }
    if (selChanged && onSelectionChanged) onSelectionChanged();
    if (hover >= 0 && ! proc.isUsed (hover)) hover = -1;

    // état : paramètres + décalages dynamiques
    std::vector<float> key;
    key.reserve (eq::maxBands * 8 + 4);
    for (int b = 0; b < maxBands; ++b)
    {
        const auto s = proc.getBandSpec (b, true);
        key.push_back (proc.isUsed (b) ? 1.0f : 0.0f);
        key.push_back (s.active ? 1.0f : 0.0f);
        if (! proc.isUsed (b)) continue;
        key.insert (key.end(), { (float) s.freq, (float) s.gainDb, (float) s.q, (float) s.shape, (float) s.slope, (float) s.place,
                                 std::round (proc.getDynOffset (b) * 20.0f) / 20.0f });
    }
    key.push_back (proc.isLinearPhase() ? 1.0f : 0.0f);
    key.push_back ((float) range());
    if (key != lastKey) { lastKey = std::move (key); dirty = true; }
    if (dirty) { updateCurves(); dirty = false; repaint(); }

    updateAnalyzer();

    // crêtes de sortie (décroissance ≈ 26 dB/s)
    const bool active = juce::Time::getMillisecondCounter() - proc.lastBlockMs.load() < 300;
    const float pl = proc.peakL.exchange (0.0f), pr = proc.peakR.exchange (0.0f);
    const float dl = juce::Decibels::gainToDecibels (pl, -90.0f), dr = juce::Decibels::gainToDecibels (pr, -90.0f);
    const float nl = std::max (dl, meterL - 0.9f), nr = std::max (dr, meterR - 0.9f);
    if (std::abs (nl - meterL) > 0.01f || std::abs (nr - meterR) > 0.01f || analyzerLive || active)
    {
        meterL = nl; meterR = nr;
        repaint();
    }
}

void EqDisplay::updateCurves()
{
    // Courbe principale = ce que reçoit la voie gauche / le Mid ; seconde courbe (pointillés)
    // pour la voie droite / le Side quand des bandes ne touchent qu'une partie du signal.
    const double fs = proc.getSampleRateSafe();
    const bool analog = proc.isLinearPhase();
    std::fill (totalCurve.begin(), totalCurve.end(), 0.0f);
    std::fill (curveB.begin(), curveB.end(), 0.0f);
    bool anyLR = false, anyMS = false;
    for (int b = 0; b < maxBands; ++b)
        if (proc.isUsed (b) && proc.bandValue (b, P::fOn) > 0.5f)
        {
            const auto pl = (Place) juce::roundToInt (proc.bandValue (b, P::fPlace));
            anyLR = anyLR || pl == Place::left || pl == Place::right;
            anyMS = anyMS || pl == Place::mid || pl == Place::side;
        }
    curveMode = anyMS ? 2 : anyLR ? 1 : 0;
    Design d;
    for (int b = 0; b < maxBands; ++b)
    {
        bandShown[(size_t) b] = proc.isUsed (b);
        if (! bandShown[(size_t) b]) continue;
        auto s = proc.getBandSpec (b, true);
        const bool on = s.active;
        s.active = true;
        if (hasGain (s.shape)) s.gainDb += proc.getDynOffset (b);
        design (s, fs, d, analog);
        const bool toA = s.place == Place::stereo || s.place == Place::left || s.place == Place::mid;
        const bool toB = s.place == Place::stereo || s.place == Place::right || s.place == Place::side;
        auto& c = bandCurve[(size_t) b];
        for (int i = 0; i < numCols; ++i)
        {
            const double f = colFreq[(size_t) i];
            const float v = f < 0.5 * fs || analog ? (float) magnitudeDb (d, std::min (f, 0.4999 * fs), fs, analog) : c[(size_t) std::max (0, i - 1)];
            c[(size_t) i] = v;
            if (! on) continue;
            if (toA) totalCurve[(size_t) i] += v;
            if (toB) curveB[(size_t) i] += v;
        }
    }
}

void EqDisplay::analyse (const AudioLabEqProcessor::Tap& tap, std::vector<float>& col)
{
    std::fill (fftBuf.begin(), fftBuf.end(), 0.0f);
    tap.readLast (fftBuf.data(), fftSize);
    double wsum = 0;
    for (int i = 0; i < fftSize; ++i) { fftBuf[(size_t) i] *= window[(size_t) i]; wsum += window[(size_t) i]; }
    fft.performFrequencyOnlyForwardTransform (fftBuf.data(), true);
    const double fs = proc.getSampleRateSafe();
    const float norm = (float) (2.0 / wsum);
    auto binDb = [&] (double k)
    {
        const int k0 = juce::jlimit (0, fftSize / 2 - 1, (int) k);
        const double t = k - k0;
        const double a = fftBuf[(size_t) k0] * norm, b = fftBuf[(size_t) (k0 + 1)] * norm;
        return 20.0 * std::log10 (std::max (1e-9, a + (b - a) * t));
    };
    for (int i = 0; i < numCols; ++i)
    {
        const double f = colFreq[(size_t) i];
        const double fa = i > 0 ? std::sqrt (f * colFreq[(size_t) i - 1]) : f;
        const double fb = i + 1 < numCols ? std::sqrt (f * colFreq[(size_t) i + 1]) : f;
        double v;
        const double ka = fa * fftSize / fs, kb = fb * fftSize / fs;
        if (f >= 0.5 * fs) v = -120;
        else if (kb - ka >= 1.0)
        {
            double m = 0;
            for (int k = (int) std::ceil (ka); k <= (int) std::floor (kb) && k <= fftSize / 2; ++k) m = std::max (m, (double) fftBuf[(size_t) k] * norm);
            v = 20.0 * std::log10 (std::max (1e-9, m));
        }
        else v = binDb (f * fftSize / fs);
        v += 4.5 * std::log2 (f / 1000.0);   // pente d'affichage : un bruit rose paraît plat
        auto& s = col[(size_t) i];
        if (v > s) s += (float) (0.65 * (v - s));
        else s -= (float) std::min (s - v, 1.4);
    }
}

void EqDisplay::updateAnalyzer()
{
    const auto mode = proc.ui.getProperty ("analyzer", "prepost").toString();
    const bool active = juce::Time::getMillisecondCounter() - proc.lastBlockMs.load() < 300;
    analyzerLive = active && mode != "off";
    if (! analyzerLive)
    {
        // retombée douce quand le son s'arrête
        bool any = false;
        for (auto* v : { &anPre, &anPost })
            for (auto& x : *v) if (x > -120) { x -= 2.0f; any = true; }
        if (any) repaint();
        return;
    }
    if (mode == "prepost" || mode == "pre") analyse (proc.tapPre, anPre);
    else std::fill (anPre.begin(), anPre.end(), -120.0f);
    if (mode == "prepost" || mode == "post") analyse (proc.tapPost, anPost);
    else std::fill (anPost.begin(), anPost.end(), -120.0f);
}

// ── Dessin ──────────────────────────────────────────────────────────────
void EqDisplay::paint (juce::Graphics& g)
{
    const auto& p = theme.pal();
    auto r = plot();
    const double fs = proc.getSampleRateSafe();

    juce::Path bgPath; bgPath.addRoundedRectangle (r, 10);
    g.setColour (p.cv); g.fillPath (bgPath);
    g.saveState();
    g.reduceClipRegion (bgPath);

    // zone au-delà de Nyquist
    if (0.5 * fs < fMax)
    {
        const float xn = xOf (0.5 * fs);
        g.setColour (p.cv2); g.fillRect (xn, r.getY(), r.getRight() - xn, r.getHeight());
    }

    // grille
    const double freqs[] = { 20, 30, 40, 50, 60, 70, 80, 90, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,
                             2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 20000 };
    for (double f : freqs)
    {
        const bool major = f == 100 || f == 1000 || f == 10000;
        g.setColour (major ? p.grid2 : p.grid.withAlpha (0.7f));
        g.drawVerticalLine (juce::roundToInt (xOf (f)), r.getY(), r.getBottom());
    }
    const double rg = range();
    const double stepDb = rg <= 3 ? 1 : rg <= 6 ? 2 : rg <= 12 ? 3 : 6;
    for (double db = -rg; db <= rg + 1e-6; db += stepDb)
    {
        g.setColour (std::abs (db) < 1e-6 ? p.grid2 : p.grid.withAlpha (0.7f));
        g.drawHorizontalLine (juce::roundToInt (yOf (db)), r.getX(), r.getRight());
    }

    // analyseur
    if (proc.ui.getProperty ("analyzer", "prepost").toString() != "off")
    {
        paintAnalyzer (g, anPre, p.muted, false);
        paintAnalyzer (g, anPost, p.accent, true);
    }

    // libellés de fréquence
    g.setFont (theme.ui (10));
    g.setColour (p.txt);
    for (double f : { 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0 })
    {
        const float x = xOf (f);
        g.drawText (f >= 1000 ? String ((int) (f / 1000)) + "k" : String ((int) f), juce::Rectangle<float> (x + 3, r.getBottom() - 16, 40, 14),
                    juce::Justification::centredLeft, false);
    }

    const float y0 = yOf (0);
    // courbes des bandes sélectionnées / survolées
    for (int b = 0; b < maxBands; ++b)
    {
        if (! bandShown[(size_t) b]) continue;
        const bool hl = selection.contains (b) || b == hover || b == dragBand;
        if (! hl) continue;
        juce::Path fill; fill.startNewSubPath (r.getX(), y0);
        for (int i = 0; i < numCols; ++i)
            fill.lineTo (r.getX() + i + 0.5f, juce::jlimit (r.getY() - 20, r.getBottom() + 20, yOf (bandCurve[(size_t) b][(size_t) i])));
        fill.lineTo (r.getRight(), y0); fill.closeSubPath();
        const auto c = bandColour (b, p);
        g.setColour (c.withAlpha (0.16f)); g.fillPath (fill);
        juce::Path line;
        for (int i = 0; i < numCols; ++i)
        {
            const float y = juce::jlimit (r.getY() - 20, r.getBottom() + 20, yOf (bandCurve[(size_t) b][(size_t) i]));
            if (i == 0) line.startNewSubPath (r.getX() + 0.5f, y); else line.lineTo (r.getX() + i + 0.5f, y);
        }
        g.setColour (c.withAlpha (0.55f)); g.strokePath (line, juce::PathStrokeType (1.2f));
    }

    // courbe(s) totale(s)
    auto curvePath = [&] (const std::vector<float>& c, juce::Path& line, juce::Path* fill)
    {
        if (fill != nullptr) fill->startNewSubPath (r.getX(), y0);
        for (int i = 0; i < numCols; ++i)
        {
            const float y = juce::jlimit (r.getY() - 40, r.getBottom() + 40, yOf (c[(size_t) i]));
            if (i == 0) line.startNewSubPath (r.getX() + 0.5f, y); else line.lineTo (r.getX() + i + 0.5f, y);
            if (fill != nullptr) fill->lineTo (r.getX() + i + 0.5f, y);
        }
        if (fill != nullptr) { fill->lineTo (r.getRight(), y0); fill->closeSubPath(); }
    };
    const auto curveColour = p.dark ? juce::Colour (0xFFF5F5F7) : p.text.withAlpha (0.88f);
    if (curveMode != 0)
    {
        juce::Path lineB, dashed;
        curvePath (curveB, lineB, nullptr);
        const float dash[] = { 5.0f, 4.0f };
        juce::PathStrokeType (1.8f).createDashedStroke (dashed, lineB, dash, 2);
        g.setColour (p.orange.withAlpha (0.9f)); g.fillPath (dashed);
    }
    {
        juce::Path line, fill;
        curvePath (totalCurve, line, &fill);
        g.setColour (p.accent.withAlpha (p.dark ? 0.14f : 0.10f)); g.fillPath (fill);
        g.setColour (curveColour);
        g.strokePath (line, juce::PathStrokeType (2.2f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    }
    if (curveMode != 0)
    {
        // légende des deux courbes
        const String a = curveMode == 2 ? "Mid" : U ("Gauche"), b = curveMode == 2 ? "Side" : U ("Droite");
        auto lg = juce::Rectangle<float> (r.getX() + 12, r.getY() + 10, 150, 16);
        g.setColour (curveColour); g.fillRect (lg.getX(), lg.getCentreY() - 1, 18.0f, 2.2f);
        g.setFont (theme.ui (11, 600)); g.drawText (a, lg.withTrimmedLeft (24), juce::Justification::centredLeft, false);
        lg.translate (0, 18);
        g.setColour (p.orange); 
        for (int k = 0; k < 3; ++k) g.fillRect (lg.getX() + k * 7.0f, lg.getCentreY() - 1, 4.0f, 2.0f);
        g.drawText (b, lg.withTrimmedLeft (24), juce::Justification::centredLeft, false);
    }

    // poignées
    for (int b = 0; b < maxBands; ++b)
    {
        if (! bandShown[(size_t) b]) continue;
        const auto s = proc.getBandSpec (b, true);
        const auto c = bandColour (b, p);
        const auto pos = nodePos (b);
        const bool sel = selection.contains (b), hv = b == hover;
        // plage dynamique
        if (proc.bandValue (b, P::fDyn) > 0.5f && hasGain (s.shape))
        {
            const float yr = yOf (s.gainDb + proc.bandValue (b, P::fRange) * proc.gainScale());
            const float yd = yOf (s.gainDb + proc.getDynOffset (b));
            g.setColour (c.withAlpha (0.55f));
            g.drawLine (pos.x, pos.y, pos.x, yr, 2.0f);
            g.drawLine (pos.x - 5, yr, pos.x + 5, yr, 2.0f);
            g.setColour (c); g.drawEllipse (pos.x - 4.5f, yd - 4.5f, 9, 9, 1.8f);
        }
        const float rad = sel ? 9.5f : hv ? 9.0f : 8.0f;
        if (sel)
        {
            g.setColour (c.withAlpha (0.25f));
            g.fillEllipse (pos.x - rad - 5, pos.y - rad - 5, (rad + 5) * 2, (rad + 5) * 2);
        }
        if (s.active)
        {
            g.setColour (c); g.fillEllipse (pos.x - rad, pos.y - rad, rad * 2, rad * 2);
            g.setColour (juce::Colours::white);
        }
        else
        {
            g.setColour (p.cv); g.fillEllipse (pos.x - rad, pos.y - rad, rad * 2, rad * 2);
            g.setColour (c.withAlpha (0.7f)); g.drawEllipse (pos.x - rad, pos.y - rad, rad * 2, rad * 2, 1.6f);
        }
        if (sel) { g.setColour (p.dark ? juce::Colours::white : p.text); g.drawEllipse (pos.x - rad - 1, pos.y - rad - 1, rad * 2 + 2, rad * 2 + 2, 1.5f); }
        g.setColour (s.active ? juce::Colours::white : c);
        g.setFont (theme.ui (9.5f, 700));
        g.drawText (String (b + 1), juce::Rectangle<float> (pos.x - 10, pos.y - 6, 20, 12), juce::Justification::centred, false);
        if (s.place != Place::stereo)
        {
            g.setColour (c); g.setFont (theme.ui (9, 700));
            g.drawText (P::placeShort()[(int) s.place], juce::Rectangle<float> (pos.x + rad - 1, pos.y - rad - 11, 14, 11), juce::Justification::centredLeft, false);
        }
        if (proc.getSolo() == b)
        {
            g.setColour (c); g.drawEllipse (pos.x - rad - 4, pos.y - rad - 4, rad * 2 + 8, rad * 2 + 8, 1.2f);
        }
    }

    // lasso
    if (drag == Drag::lasso)
    {
        g.setColour (p.accent.withAlpha (0.12f)); g.fillRect (lasso);
        g.setColour (p.accent.withAlpha (0.6f)); g.drawRect (lasso, 1.0f);
    }

    // dessin à main levée
    if (sketchMode)
    {
        juce::Path sp; bool started = false;
        for (int i = 0; i < numCols; ++i)
        {
            const float v = sketch[(size_t) i];
            if (std::isnan (v)) { started = false; continue; }
            const float x = r.getX() + i + 0.5f, y = yOf (v);
            if (! started) { sp.startNewSubPath (x, y); started = true; } else sp.lineTo (x, y);
        }
        g.setColour (p.orange);
        g.strokePath (sp, juce::PathStrokeType (2.5f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        if (drag != Drag::sketch)
        {
            g.setColour (p.orange); g.setFont (theme.ui (12, 600));
            g.drawText (U ("Dessinez la courbe voulue : les bandes sont ajustées au relâchement"),
                        r.withHeight (28).translated (0, 8), juce::Justification::centred, false);
        }
    }
    else
    {
        bool any = false;
        for (auto v : bandShown) any = any || v;
        if (! any)
        {
            g.setColour (p.muted); g.setFont (theme.ui (13));
            g.drawText (U ("Cliquez pour ajouter une bande · Glissez pour sélectionner · Alt+clic sur une bande : écouter"),
                        r.withSizeKeepingCentre (r.getWidth(), 24).translated (0, -40), juce::Justification::centred, false);
        }
    }

    // valeurs de la bande survolée / déplacée
    const int rb = drag == Drag::node ? dragBand : hover;
    if (rb >= 0 && bandShown[(size_t) rb] && ! sketchMode) paintReadout (g, rb);

    g.restoreState();

    // échelle dB + bouton de plage
    g.setFont (theme.ui (10));
    for (double db = -rg; db <= rg + 1e-6; db += stepDb * (rg >= 30 ? 1 : 2))
    {
        if (std::abs (db - rg) < 1e-6) continue;
        g.setColour (std::abs (db) < 1e-6 ? p.dim : p.txt);
        g.drawText ((db > 0 ? "+" : "") + String ((int) db), juce::Rectangle<float> (r.getRight() + 4, yOf (db) - 7, 26, 14),
                    juce::Justification::centredLeft, false);
    }
    auto rbx = rangeButton();
    g.setColour (p.bg3); g.fillRoundedRectangle (rbx, 6);
    g.setColour (p.text); g.setFont (theme.ui (10.5f, 600));
    g.drawText (U ("±") + String ((int) rg), rbx, juce::Justification::centred, false);

    paintMeters (g);
    paintPiano (g);
}

void EqDisplay::paintAnalyzer (juce::Graphics& g, const std::vector<float>& col, juce::Colour c, bool fill)
{
    auto r = plot();
    juce::Path line, area;
    area.startNewSubPath (r.getX(), r.getBottom());
    bool any = false;
    for (int i = 0; i < numCols; ++i)
    {
        const float y = juce::jlimit (r.getY(), r.getBottom(), yAn (col[(size_t) i]));
        if (y < r.getBottom() - 0.5f) any = true;
        if (i == 0) line.startNewSubPath (r.getX(), y); else line.lineTo (r.getX() + i + 0.5f, y);
        area.lineTo (r.getX() + i + 0.5f, y);
    }
    if (! any) return;
    area.lineTo (r.getRight(), r.getBottom()); area.closeSubPath();
    const auto& p = theme.pal();
    if (fill)
    {
        g.setGradientFill (juce::ColourGradient (c.withAlpha (p.dark ? 0.30f : 0.22f), 0, r.getY() + r.getHeight() * 0.3f,
                                                 c.withAlpha (0.03f), 0, r.getBottom(), false));
        g.fillPath (area);
        g.setColour (c.withAlpha (0.45f)); g.strokePath (line, juce::PathStrokeType (1.0f));
    }
    else
    {
        g.setColour (c.withAlpha (p.dark ? 0.45f : 0.38f)); g.strokePath (line, juce::PathStrokeType (1.0f));
    }
}

void EqDisplay::paintPiano (juce::Graphics& g)
{
    const auto& p = theme.pal();
    auto pa = pianoArea();
    juce::Path clip; clip.addRoundedRectangle (pa, 4);
    g.saveState(); g.reduceClipRegion (clip);
    g.setColour (p.dark ? juce::Colour (0xFFD1D1D6) : juce::Colours::white); g.fillRect (pa);
    // touches blanches (A0 = 21 → C8 = 108)
    for (int m = 21; m <= 108; ++m)
    {
        const int pc = m % 12;
        const bool black = pc == 1 || pc == 3 || pc == 6 || pc == 8 || pc == 10;
        const double fl = 440.0 * std::pow (2.0, (m - 69 - 0.5) / 12.0), fh = 440.0 * std::pow (2.0, (m - 69 + 0.5) / 12.0);
        const float x0 = xOf (fl), x1 = xOf (fh);
        if (black)
        {
            g.setColour (p.dark ? juce::Colour (0xFF1C1C1E) : juce::Colour (0xFF2C2C2E));
            g.fillRect (x0, pa.getY(), x1 - x0, pa.getHeight() * 0.62f);
        }
        else if (pc == 0 || pc == 5)
        {
            g.setColour (p.grid2); g.drawVerticalLine (juce::roundToInt (x0), pa.getY(), pa.getBottom());
            if (pc == 0)
            {
                g.setColour (p.muted); g.setFont (theme.ui (7.5f));
                g.drawText ("C" + String (m / 12 - 1), juce::Rectangle<float> (x0 + 1, pa.getBottom() - 9, 16, 9), juce::Justification::centredLeft, false);
            }
        }
    }
    // fréquences des bandes sur le clavier
    for (int b = 0; b < maxBands; ++b)
    {
        if (! bandShown[(size_t) b]) continue;
        const float x = xOf (proc.bandValue (b, P::fFreq));
        g.setColour (bandColour (b, p).withAlpha (selection.contains (b) ? 1.0f : 0.6f));
        g.fillRect (x - 1.0f, pa.getY(), 2.0f, pa.getHeight());
    }
    g.restoreState();
    g.setColour (p.border); g.drawRoundedRectangle (pa, 4, 1);
}

void EqDisplay::paintMeters (juce::Graphics& g)
{
    const auto& p = theme.pal();
    auto r = plot();
    const float x = r.getRight() + 34, w = 4, top = r.getY() + 26, bot = r.getBottom();
    auto yM = [&] (float db) { return bot - (bot - top) * juce::jlimit (0.0f, 1.0f, (db + 60.0f) / 66.0f); };
    for (int ch = 0; ch < 2; ++ch)
    {
        const float xx = x + ch * (w + 2), v = ch == 0 ? meterL : meterR;
        g.setColour (p.bg3); g.fillRoundedRectangle (xx, top, w, bot - top, 2);
        if (v > -60)
        {
            const float yv = yM (v);
            g.setGradientFill (juce::ColourGradient (p.red, 0, yM (3), p.green, 0, yM (-24), false));
            g.fillRoundedRectangle (xx, yv, w, bot - yv, 2);
        }
    }
    g.setColour (p.txt); g.setFont (theme.ui (8));
    g.drawText ("0", juce::Rectangle<float> (x - 12, yM (0) - 5, 10, 10), juce::Justification::centredRight, false);
}

void EqDisplay::paintReadout (juce::Graphics& g, int b)
{
    const auto& p = theme.pal();
    const auto s = proc.getBandSpec (b, true);
    const auto pos = nodePos (b);
    int midi; double cents;
    nearestNote (s.freq, midi, cents);
    String l1 = U ("Bande ") + String (b + 1) + U (" · ") + P::shapeNames()[(int) s.shape];
    if (s.place != Place::stereo) l1 << U (" · ") << P::placeNames()[(int) s.place];
    if (! s.active) l1 << U (" · désactivée");
    String l2 = P::fmtHz ((float) s.freq);
    if (hasGain (s.shape)) l2 << U ("  ") << P::fmtDb ((float) s.gainDb) << " dB";
    if (usesQ (s.shape, s.slope)) l2 << "  Q " << P::fmtQ ((float) s.q);
    if (usesSlope (s.shape) && ! hasGain (s.shape)) l2 << U ("  ") << String (s.slope) << " dB/oct";
    l2 << U ("  ·  ") << noteName (midi) << String (noteOctave (midi)) << " " << (cents >= 0 ? "+" : "") << String (juce::roundToInt (cents)) << " ct";
    juce::GlyphArrangement ga; ga.addLineOfText (theme.ui (11, 600), l1, 0, 0);
    juce::GlyphArrangement gb; gb.addLineOfText (theme.ui (11), l2, 0, 0);
    const float w = std::max (ga.getBoundingBox (0, -1, true).getWidth(), gb.getBoundingBox (0, -1, true).getWidth()) + 20, h = 40;
    auto r = plot();
    float x = juce::jlimit (r.getX() + 4, r.getRight() - w - 4, pos.x - w / 2);
    float y = pos.y - h - 18;
    if (y < r.getY() + 4) y = pos.y + 18;
    auto box = juce::Rectangle<float> (x, y, w, h);
    juce::Path bp; bp.addRoundedRectangle (box, 8);
    juce::DropShadow (juce::Colours::black.withAlpha (0.18f), 10, { 0, 3 }).drawForPath (g, bp);
    g.setColour (p.bg2.withAlpha (0.97f)); g.fillPath (bp);
    g.setColour (bandColour (b, p)); g.fillRoundedRectangle (box.getX() + 6, box.getY() + 8, 3, h - 16, 1.5f);
    g.setColour (p.text); g.setFont (theme.ui (11, 600));
    g.drawText (l1, box.withTrimmedLeft (14).withHeight (20).translated (0, 2), juce::Justification::centredLeft, false);
    g.setColour (p.dim); g.setFont (theme.ui (11));
    g.drawText (l2, box.withTrimmedLeft (14).withTrimmedTop (19).withHeight (17), juce::Justification::centredLeft, false);
}

// ── Poignées ────────────────────────────────────────────────────────────
juce::Point<float> EqDisplay::nodePos (int b) const
{
    const auto s = proc.getBandSpec (b, true);
    const float x = xOf (s.freq);
    const float y = hasGain (s.shape) ? yOf (s.gainDb) : yOf (0);
    auto r = plot();
    return { x, juce::jlimit (r.getY() + 9, r.getBottom() - 9, y) };
}

int EqDisplay::hitBand (juce::Point<float> pt) const
{
    int best = -1; float bd = 13.0f * 13.0f;
    for (int b = maxBands; --b >= 0;)
    {
        if (! bandShown[(size_t) b]) continue;
        const float d = nodePos (b).getDistanceSquaredFrom (pt);
        if (d < bd || (d <= bd + 1 && selection.contains (b))) { bd = d; best = b; }
    }
    return best;
}

// ── Sélection ───────────────────────────────────────────────────────────
void EqDisplay::select (int band, bool add)
{
    if (! add) selection.clearQuick();
    if (band >= 0 && ! selection.contains (band)) selection.add (band);
    if (onSelectionChanged) onSelectionChanged();
    repaint();
}

void EqDisplay::clearSelection()
{
    selection.clearQuick();
    if (onSelectionChanged) onSelectionChanged();
    repaint();
}

void EqDisplay::selectAll()
{
    selection.clearQuick();
    for (int b = 0; b < maxBands; ++b) if (proc.isUsed (b)) selection.add (b);
    if (onSelectionChanged) onSelectionChanged();
    repaint();
}

void EqDisplay::deleteSelected()
{
    if (selection.isEmpty()) return;
    proc.pushUndo();
    for (int b : selection) proc.deleteBand (b);
    if (onMessage) onMessage (selection.size() > 1 ? String (selection.size()) + U (" bandes supprimées") : U ("Bande supprimée"));
    selection.clearQuick();
    if (onSelectionChanged) onSelectionChanged();
    refresh();
}

// ── Gestes (automation des logiciels) ───────────────────────────────────
void EqDisplay::beginEdit()
{
    endEdit();
    for (auto& st : starts)
        for (auto* f : { P::fFreq, P::fGain, P::fQ })
        {
            auto* prm = proc.bandParam (st.band, f);
            prm->beginChangeGesture();
            gestureParams.push_back (prm);
        }
}

void EqDisplay::endEdit()
{
    for (auto* prm : gestureParams) prm->endChangeGesture();
    gestureParams.clear();
}

// ── Souris ──────────────────────────────────────────────────────────────
void EqDisplay::mouseMove (const juce::MouseEvent& e)
{
    const int h = sketchMode ? -1 : hitBand (e.position);
    if (h != hover) { hover = h; repaint(); }
    if (sketchMode) setMouseCursor (juce::MouseCursor::CrosshairCursor);
    else if (rangeButton().contains (e.position)) setMouseCursor (juce::MouseCursor::PointingHandCursor);
    else setMouseCursor (h >= 0 ? juce::MouseCursor::DraggingHandCursor
                                : plot().contains (e.position) ? juce::MouseCursor::CrosshairCursor : juce::MouseCursor::NormalCursor);
}

void EqDisplay::mouseExit (const juce::MouseEvent&)
{
    if (hover >= 0) { hover = -1; repaint(); }
}

void EqDisplay::mouseDown (const juce::MouseEvent& e)
{
    grabKeyboardFocus();
    downPos = lastPos = e.position;
    drag = Drag::none;

    if (rangeButton().contains (e.position))
    {
        const int cur = (int) proc.ui.getProperty ("dbRange", 12);
        const int next = cur == 3 ? 6 : cur == 6 ? 12 : cur == 12 ? 30 : 3;
        proc.ui.setProperty ("dbRange", next, nullptr);
        refresh();
        return;
    }
    if (! plot().contains (e.position)) return;

    if (sketchMode)
    {
        std::fill (sketch.begin(), sketch.end(), std::numeric_limits<float>::quiet_NaN());
        drag = Drag::sketch;
        lastSketchCol = -1;
        mouseDrag (e);
        return;
    }

    const int b = hitBand (e.position);
    if (e.mods.isPopupMenu())
    {
        if (b >= 0) { if (! selection.contains (b)) select (b); showBandMenu (b, e.position); }
        else showEmptyMenu (e.position);
        return;
    }

    if (b >= 0)
    {
        if (e.mods.isCommandDown()) { if (selection.contains (b)) selection.removeFirstMatchingValue (b); else selection.add (b); if (onSelectionChanged) onSelectionChanged(); repaint(); return; }
        if (! selection.contains (b)) select (b);
        if (e.mods.isAltDown()) { proc.setSolo (b); soloHeld = true; }   // écoute tant que le bouton est enfoncé
        proc.pushUndo();
        drag = Drag::node; dragBand = b;
        starts.clear();
        for (int sb : selection)
            starts.push_back ({ sb, (double) proc.bandValue (sb, P::fFreq), (double) proc.bandValue (sb, P::fGain), (double) proc.bandValue (sb, P::fQ) });
        beginEdit();
        repaint();
        return;
    }
    drag = Drag::empty;
}

void EqDisplay::mouseDrag (const juce::MouseEvent& e)
{
    auto r = plot();
    if (drag == Drag::sketch)
    {
        const int col = juce::jlimit (0, numCols - 1, (int) (e.position.x - r.getX()));
        const float v = (float) juce::jlimit (-range(), range(), dbOf (e.position.y));
        if (lastSketchCol >= 0 && lastSketchCol != col)
        {
            const float v0 = sketch[(size_t) lastSketchCol];
            const int dir = col > lastSketchCol ? 1 : -1;
            for (int c = lastSketchCol + dir; c != col; c += dir)
                sketch[(size_t) c] = v0 + (v - v0) * (float) (c - lastSketchCol) / (float) (col - lastSketchCol);
        }
        sketch[(size_t) col] = v;
        lastSketchCol = col;
        repaint();
        return;
    }
    if (drag == Drag::empty && e.position.getDistanceFrom (downPos) > 4) drag = Drag::lasso;
    if (drag == Drag::lasso)
    {
        lasso = juce::Rectangle<float> (downPos, e.position);
        juce::Array<int> sel;
        for (int b = 0; b < maxBands; ++b) if (bandShown[(size_t) b] && lasso.contains (nodePos (b))) sel.add (b);
        if (sel != selection) { selection = sel; if (onSelectionChanged) onSelectionChanged(); }
        repaint();
        return;
    }
    if (drag != Drag::node) return;

    const float fine = e.mods.isShiftDown() ? 0.15f : 1.0f;
    const float dx = (e.position.x - downPos.x) * fine, dy = (e.position.y - downPos.y) * fine;
    const double octPerPx = std::log (fMax / fMin) / r.getWidth();
    const double dbPerPx = 2.0 * range() / (r.getHeight() - 24.0);
    const double sc = std::max (0.01, proc.gainScale());
    const bool lockF = e.mods.isCommandDown() && ! e.mods.isAltDown();   // Cmd : verticale seulement
    for (const auto& st : starts)
    {
        const auto shape = (Shape) juce::roundToInt (proc.bandValue (st.band, P::fShape));
        if (! lockF)
        {
            const double f = juce::jlimit (10.0, 30000.0, st.f * std::exp (dx * octPerPx));
            proc.bandParam (st.band, P::fFreq)->setValueNotifyingHost (proc.bandParam (st.band, P::fFreq)->convertTo0to1 ((float) f));
        }
        if (hasGain (shape))
        {
            const double g = juce::jlimit (-30.0, 30.0, st.g - dy * dbPerPx / sc);
            proc.bandParam (st.band, P::fGain)->setValueNotifyingHost (proc.bandParam (st.band, P::fGain)->convertTo0to1 ((float) g));
        }
        else
        {
            // coupe-bas / coupe-haut / notch / passe-bande : la hauteur règle le Q
            const double q = juce::jlimit (0.025, 40.0, st.q * std::pow (2.0, -dy / 45.0));
            proc.bandParam (st.band, P::fQ)->setValueNotifyingHost (proc.bandParam (st.band, P::fQ)->convertTo0to1 ((float) q));
        }
    }
    lastPos = e.position;
    refresh();
}

void EqDisplay::mouseUp (const juce::MouseEvent& e)
{
    if (soloHeld) { proc.setSolo (-1); soloHeld = false; }
    endEdit();
    const auto d = drag;
    drag = Drag::none; dragBand = -1;
    if (d == Drag::sketch) { fitSketch(); return; }
    if (d == Drag::lasso) { lasso = {}; repaint(); return; }
    if (d == Drag::empty && ! e.mods.isPopupMenu())
    {
        // clic dans le vide : nouvelle bande (cloche ; coupe-bas / coupe-haut aux extrémités)
        auto r = plot();
        const double f = fOf (e.position.x);
        const float rel = (e.position.x - r.getX()) / r.getWidth();
        Shape shape = Shape::bell;
        if (rel < 0.035f) shape = Shape::lowCut;
        else if (rel > 0.965f) shape = Shape::highCut;
        const double sc = std::max (0.01, proc.gainScale());
        const float g = shape == Shape::bell ? (float) juce::jlimit (-30.0, 30.0, dbOf (e.position.y) / sc) : 0.0f;
        proc.pushUndo();
        const int b = proc.createBand (shape, (float) juce::jlimit (10.0, 30000.0, shape == Shape::lowCut ? std::max (f, 20.0) : shape == Shape::highCut ? std::min (f, 20000.0) : f),
                                       std::abs (g) < 0.15f ? 0.0f : g, shape == Shape::bell ? 1.0f : 0.71f, shape == Shape::bell ? 12 : 24);
        if (b < 0) { if (onMessage) onMessage (U ("Les 24 bandes sont utilisées")); return; }
        createdBand = b; createdAt = juce::Time::getMillisecondCounter();
        select (b);
        refresh();
    }
    repaint();
}

void EqDisplay::mouseDoubleClick (const juce::MouseEvent& e)
{
    if (sketchMode) return;
    const int b = hitBand (e.position);
    if (b < 0) return;
    if (b == createdBand && juce::Time::getMillisecondCounter() - createdAt < 600) return;   // double-clic qui vient de créer la bande
    proc.pushUndo();
    const bool on = proc.bandValue (b, P::fOn) > 0.5f;
    proc.setBandValue (b, P::fOn, on ? 0.0f : 1.0f);
    if (onMessage) onMessage (U ("Bande ") + String (b + 1) + (on ? U (" désactivée") : U (" activée")));
    refresh();
}

void EqDisplay::mouseWheelMove (const juce::MouseEvent& e, const juce::MouseWheelDetails& w)
{
    if (sketchMode) return;
    int b = hitBand (e.position);
    if (b < 0 && selection.size() == 1 && plot().contains (e.position)) b = -1;
    if (b < 0) return;
    const float dv = (std::abs (w.deltaY) > std::abs (w.deltaX) ? w.deltaY : -w.deltaX) * (w.isReversed ? -1.0f : 1.0f);
    if (std::abs (dv) < 1e-6f) return;
    if (juce::Time::getMillisecondCounter() - lastWheel > 600) proc.pushUndo();
    lastWheel = juce::Time::getMillisecondCounter();
    const auto shape = (Shape) juce::roundToInt (proc.bandValue (b, P::fShape));
    if (e.mods.isAltDown() && canBeDynamic (shape))
    {
        // Alt + molette : plage dynamique (active la dynamique)
        if (proc.bandValue (b, P::fDyn) < 0.5f) proc.setBandValue (b, P::fDyn, 1);
        proc.setBandValue (b, P::fRange, juce::jlimit (-30.0f, 30.0f, proc.bandValue (b, P::fRange) + dv * 6.0f));
    }
    else
    {
        const float q = proc.bandValue (b, P::fQ) * std::pow (2.0f, dv * (e.mods.isShiftDown() ? 0.25f : 1.2f));
        proc.setBandValue (b, P::fQ, juce::jlimit (0.025f, 40.0f, q));
    }
    if (! selection.contains (b)) select (b);
    hover = b;
    refresh();
}

bool EqDisplay::keyPressed (const juce::KeyPress& k)
{
    if (k.isKeyCode (juce::KeyPress::deleteKey) || k.isKeyCode (juce::KeyPress::backspaceKey)) { deleteSelected(); return true; }
    if (k.isKeyCode (juce::KeyPress::escapeKey)) { if (sketchMode) setSketchMode (false); else clearSelection(); return true; }
    if (k.getModifiers().isCommandDown())
    {
        const auto c = juce::CharacterFunctions::toLowerCase (k.getTextCharacter() != 0 ? k.getTextCharacter() : (juce::juce_wchar) k.getKeyCode());
        if (c == 'z' && ! k.getModifiers().isShiftDown()) { if (proc.undo() && onUndoRedo) onUndoRedo(); refresh(); return true; }
        if ((c == 'z' && k.getModifiers().isShiftDown()) || c == 'y') { if (proc.redo() && onUndoRedo) onUndoRedo(); refresh(); return true; }
        if (c == 'a') { selectAll(); return true; }
    }
    return false;
}

// ── Menus ───────────────────────────────────────────────────────────────
void EqDisplay::showBandMenu (int b, juce::Point<float>)
{
    juce::PopupMenu m, shapes, slopes, places;
    const auto shape = (Shape) juce::roundToInt (proc.bandValue (b, P::fShape));
    const int slopeIdx = juce::roundToInt (proc.bandValue (b, P::fSlope));
    const auto place = juce::roundToInt (proc.bandValue (b, P::fPlace));
    const juce::Array<int> targets = selection.contains (b) ? selection : juce::Array<int> { b };
    juce::Component::SafePointer<EqDisplay> safe (this);
    auto apply = [safe, targets] (const char* field, float v)
    {
        if (safe == nullptr) return;
        safe->proc.pushUndo();
        for (int t : targets) safe->proc.setBandValue (t, field, v);
        safe->refresh();
    };
    const auto names = P::shapeNames();
    for (int i = 0; i < numShapes; ++i) shapes.addItem (names[i], true, i == (int) shape, [apply, i] { apply (P::fShape, (float) i); });
    const auto sn = P::slopeNames();
    for (int i = 0; i < numSlopes; ++i) slopes.addItem (sn[i], true, i == slopeIdx, [apply, i] { apply (P::fSlope, (float) i); });
    const auto pn = P::placeNames();
    for (int i = 0; i < numPlaces; ++i) places.addItem (pn[i], true, i == place, [apply, i] { apply (P::fPlace, (float) i); });
    m.addSectionHeader (U ("Bande ") + String (b + 1) + (targets.size() > 1 ? U (" (+") + String (targets.size() - 1) + ")" : String()));
    m.addSubMenu (U ("Forme"), shapes);
    m.addSubMenu (U ("Pente"), slopes, usesSlope (shape));
    m.addSubMenu (U ("Placement stéréo"), places);
    m.addSeparator();
    const bool on = proc.bandValue (b, P::fOn) > 0.5f;
    m.addItem (on ? U ("Désactiver") : U ("Activer"), [apply, on] { apply (P::fOn, on ? 0.0f : 1.0f); });
    const bool dyn = proc.bandValue (b, P::fDyn) > 0.5f;
    m.addItem (U ("EQ dynamique"), canBeDynamic (shape), dyn, [apply, dyn] { apply (P::fDyn, dyn ? 0.0f : 1.0f); });
    m.addItem (U ("Écouter cette bande"), true, proc.getSolo() == b, [safe, b]
    {
        if (safe != nullptr) { safe->proc.setSolo (safe->proc.getSolo() == b ? -1 : b); safe->refresh(); if (safe->onSelectionChanged) safe->onSelectionChanged(); }
    });
    m.addItem (U ("Réinitialiser le gain"), hasGain (shape), false, [apply] { apply (P::fGain, 0.0f); });
    m.addSeparator();
    m.addItem (targets.size() > 1 ? U ("Supprimer les bandes") : U ("Supprimer"), [safe] { if (safe != nullptr) safe->deleteSelected(); });
    m.showMenuAsync (juce::PopupMenu::Options().withTargetComponent (this).withMousePosition());
}

void EqDisplay::showEmptyMenu (juce::Point<float> at)
{
    juce::PopupMenu m;
    juce::Component::SafePointer<EqDisplay> safe (this);
    const double f = fOf (at.x);
    const double g = dbOf (at.y) / std::max (0.01, proc.gainScale());
    auto add = [safe, f, g] (Shape s, float q, int slope)
    {
        if (safe == nullptr) return;
        safe->proc.pushUndo();
        const int b = safe->proc.createBand (s, (float) f, hasGain (s) ? (float) juce::jlimit (-30.0, 30.0, g) : 0.0f, q, slope);
        if (b >= 0) safe->select (b);
        safe->refresh();
    };
    m.addSectionHeader (U ("Ajouter une bande à ") + P::fmtHz ((float) f));
    m.addItem (U ("Cloche"), [add] { add (Shape::bell, 1.0f, 12); });
    m.addItem (U ("Low shelf"), [add] { add (Shape::lowShelf, 0.71f, 12); });
    m.addItem (U ("High shelf"), [add] { add (Shape::highShelf, 0.71f, 12); });
    m.addItem (U ("Coupe-bas"), [add] { add (Shape::lowCut, 0.71f, 24); });
    m.addItem (U ("Coupe-haut"), [add] { add (Shape::highCut, 0.71f, 24); });
    m.addItem (U ("Notch"), [add] { add (Shape::notch, 8.0f, 12); });
    m.addSeparator();
    m.addItem (U ("Tout sélectionner"), [safe] { if (safe != nullptr) safe->selectAll(); });
    m.addItem (U ("Supprimer toutes les bandes"), [safe]
    {
        if (safe == nullptr) return;
        safe->proc.pushUndo(); safe->proc.clearAllBands(); safe->clearSelection(); safe->refresh();
    });
    m.showMenuAsync (juce::PopupMenu::Options().withTargetComponent (this).withMousePosition());
}

// ── Dessin à main levée ─────────────────────────────────────────────────
void EqDisplay::setSketchMode (bool on)
{
    if (sketchMode == on) return;
    sketchMode = on;
    std::fill (sketch.begin(), sketch.end(), std::numeric_limits<float>::quiet_NaN());
    hover = -1;
    if (onSketchModeChanged) onSketchModeChanged (on);
    repaint();
}

void EqDisplay::fitSketch()
{
    // grille de 160 points de 20 Hz à 20 kHz ; hors du tracé, la cible est plate (0 dB)
    const double fs = proc.getSampleRateSafe();
    constexpr int n = 160;
    std::vector<double> grid (n), target (n), weight (n);
    int drawn = 0;
    for (int i = 0; i < n; ++i)
    {
        const double f = 20.0 * std::pow (1000.0, i / (n - 1.0));
        grid[(size_t) i] = f;
        const int col = juce::jlimit (0, numCols - 1, (int) (xOf (f) - plot().getX()));
        const float v = sketch[(size_t) col];
        if (std::isnan (v)) { target[(size_t) i] = 0; weight[(size_t) i] = 0.35; }
        else { target[(size_t) i] = v / std::max (0.01, proc.gainScale()); weight[(size_t) i] = 1.0; ++drawn; }
    }
    if (drawn < 3) { repaint(); return; }
    SketchFitter fitter;
    const auto bandsOut = fitter.fit (grid, target, weight, fs, maxBands);
    proc.pushUndo();
    proc.clearAllBands();
    selection.clearQuick();
    for (const auto& fb : bandsOut)
    {
        const int b = proc.createBand (fb.shape, (float) fb.freq, (float) fb.gain, (float) fb.q, fb.slope);
        if (b >= 0) selection.add (b);
    }
    std::fill (sketch.begin(), sketch.end(), std::numeric_limits<float>::quiet_NaN());
    if (onSelectionChanged) onSelectionChanged();
    if (onMessage) onMessage (String ((int) bandsOut.size()) + U (" bandes ajustées sur le tracé"));
    refresh();
}
