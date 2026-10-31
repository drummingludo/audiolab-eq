#include "PluginEditor.h"

using namespace audiolab;
using namespace audiolab::eq;
namespace P = audiolab::params;
using juce::String;

static String U (const char* s) { return String::fromUTF8 (s); }

// ── Icônes ──────────────────────────────────────────────────────────────
static void headphonesIcon (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    r = r.withSizeKeepingCentre (13, 12);
    juce::Path p;
    p.addCentredArc (r.getCentreX(), r.getY() + 6.5f, 5.5f, 5.5f, 0, -juce::MathConstants<float>::halfPi * 1.05f, juce::MathConstants<float>::halfPi * 1.05f, true);
    g.setColour (c);
    g.strokePath (p, juce::PathStrokeType (1.4f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    g.fillRoundedRectangle (r.getX(), r.getY() + 6.5f, 3.2f, 5.5f, 1.2f);
    g.fillRoundedRectangle (r.getRight() - 3.2f, r.getY() + 6.5f, 3.2f, 5.5f, 1.2f);
}

static void undoIcon (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c, bool redo)
{
    r = r.withSizeKeepingCentre (14, 12);
    juce::Path p;
    p.addCentredArc (r.getCentreX(), r.getCentreY() + 1, 5.5f, 4.5f, 0, -juce::MathConstants<float>::halfPi, juce::MathConstants<float>::halfPi, true);
    juce::Path head;
    head.addTriangle (r.getCentreX() - 4, r.getCentreY() - 3.5f, r.getCentreX() + 1, r.getCentreY() - 7, r.getCentreX() + 1, r.getCentreY());
    if (! redo)
    {
        auto t = juce::AffineTransform::scale (-1, 1, r.getCentreX(), r.getCentreY());
        p.applyTransform (t);
        head.applyTransform (juce::AffineTransform::translation (-0.5f, 0.0f));
    }
    else head.applyTransform (juce::AffineTransform::scale (-1, 1, r.getCentreX(), r.getCentreY()).translated (0.5f, 0.0f));
    g.setColour (c);
    g.strokePath (p, juce::PathStrokeType (1.6f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    g.fillPath (head);
}

static void pencilIcon (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    r = r.withSizeKeepingCentre (13, 13);
    juce::Path p;
    p.startNewSubPath (r.getX() + 1.5f, r.getBottom() - 1.5f);
    p.lineTo (r.getX() + 2.5f, r.getBottom() - 5);
    p.lineTo (r.getRight() - 4, r.getY() + 1.5f);
    p.lineTo (r.getRight() - 1.5f, r.getY() + 4);
    p.lineTo (r.getX() + 5, r.getBottom() - 2.5f);
    p.closeSubPath();
    g.setColour (c);
    g.strokePath (p, juce::PathStrokeType (1.4f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
}

static void trashIcon (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    r = r.withSizeKeepingCentre (11, 12);
    g.setColour (c);
    g.drawLine (r.getX(), r.getY() + 2.5f, r.getRight(), r.getY() + 2.5f, 1.4f);
    g.drawLine (r.getCentreX() - 2, r.getY() + 0.5f, r.getCentreX() + 2, r.getY() + 0.5f, 1.4f);
    juce::Path b;
    b.startNewSubPath (r.getX() + 1.5f, r.getY() + 4);
    b.lineTo (r.getX() + 2.5f, r.getBottom());
    b.lineTo (r.getRight() - 2.5f, r.getBottom());
    b.lineTo (r.getRight() - 1.5f, r.getY() + 4);
    g.strokePath (b, juce::PathStrokeType (1.4f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
}

static void flaskIcon (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    // erlenmeyer d'Audio Lab, avec une petite fumée en note de musique
    const float x = r.getX(), y = r.getY(), w = r.getWidth(), h = r.getHeight();
    juce::Path p;
    p.startNewSubPath (x + w * 0.40f, y + h * 0.30f);
    p.lineTo (x + w * 0.40f, y + h * 0.50f);
    p.lineTo (x + w * 0.16f, y + h * 0.92f);
    p.lineTo (x + w * 0.84f, y + h * 0.92f);
    p.lineTo (x + w * 0.60f, y + h * 0.50f);
    p.lineTo (x + w * 0.60f, y + h * 0.30f);
    g.setColour (c);
    g.strokePath (p, juce::PathStrokeType (1.8f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    juce::Path liquid;
    liquid.startNewSubPath (x + w * 0.27f, y + h * 0.72f);
    liquid.lineTo (x + w * 0.73f, y + h * 0.72f);
    liquid.lineTo (x + w * 0.84f, y + h * 0.92f);
    liquid.lineTo (x + w * 0.16f, y + h * 0.92f);
    liquid.closeSubPath();
    g.setColour (c.withAlpha (0.35f)); g.fillPath (liquid);
    g.setColour (c);
    g.fillEllipse (x + w * 0.58f, y + h * 0.10f, w * 0.15f, h * 0.11f);
    g.drawLine (x + w * 0.73f, y + h * 0.15f, x + w * 0.73f, y - h * 0.08f, 1.4f);
    g.drawLine (x + w * 0.73f, y - h * 0.08f, x + w * 0.88f, y - h * 0.02f, 1.4f);
}

// ── Construction ────────────────────────────────────────────────────────
AudioLabEqEditor::AudioLabEqEditor (AudioLabEqProcessor& p)
    : AudioProcessorEditor (&p), proc (p),
      display (theme, p),
      undoBtn (theme, PillButton::Style::rst, {}), redoBtn (theme, PillButton::Style::rst, {}),
      abA (theme, PillButton::Style::snap, "A"), abB (theme, PillButton::Style::snap, "B"),
      abCopy (theme, PillButton::Style::rst, U ("Copier")),
      drawBtn (theme, PillButton::Style::ibtn, U ("Dessiner")),
      bypassBtn (theme, PillButton::Style::bp, "Bypass"),
      onBtn (theme, PillButton::Style::ibtn, U ("Active")),
      soloBtn (theme, PillButton::Style::ibtn, U ("Écouter")),
      delBtn (theme, PillButton::Style::rst, {}),
      dynBtn (theme, PillButton::Style::ibtn, U ("Dynamique")),
      autoBtn (theme, PillButton::Style::ibtn, U ("Gain auto"))
{
    setLookAndFeel (&lnf);
    tooltips.setLookAndFeel (&lnf);
    setWantsKeyboardFocus (true);
    addAndMakeVisible (content);
    auto& ap = proc.apvts;
    auto prm = [&] (const char* id) -> juce::RangedAudioParameter& { return *ap.getParameter (id); };

    // Affichage
    display.onSelectionChanged = [this] { rebuildBandPanel(); };
    display.onMessage = [this] (const String& s) { toast.show (s); };
    display.onUndoRedo = [this] { rebuildBandPanel(); toast.show (U ("Annulé / rétabli")); };
    display.onSketchModeChanged = [this] (bool on) { drawBtn.on = on; drawBtn.repaint(); };
    content.addAndMakeVisible (display);

    // Presets
    {
        const auto& list = P::presets();
        for (int i = 0; i < (int) list.size(); ++i) presetBox.addItem (U (list[(size_t) i].name), i + 1);
        presetBox.setTitle ("Presets");
        presetBox.setTooltip (U ("Presets d'usine : chaque preset remplace toutes les bandes"));
        presetBox.setSelectedId ((int) proc.ui.getProperty ("preset", 0) + 1, juce::dontSendNotification);
        presetBox.onChange = [this]
        {
            const int i = presetBox.getSelectedId() - 1;
            if (i < 0) return;
            proc.pushUndo();
            proc.applyPreset (i);
            display.clearSelection();
            display.refresh();
            toast.show (U (P::presets()[(size_t) i].hint));
        };
        content.addAndMakeVisible (presetBox);
    }

    undoBtn.icon = [] (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c) { undoIcon (g, r.translated (5, 0), c, false); };
    redoBtn.icon = [] (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c) { undoIcon (g, r.translated (5, 0), c, true); };
    undoBtn.setTooltip (U ("Annuler (Cmd+Z)"));
    redoBtn.setTooltip (U ("Rétablir (Cmd+Maj+Z)"));
    undoBtn.onClick = [this] { if (proc.undo()) { display.refresh(); rebuildBandPanel(); } };
    redoBtn.onClick = [this] { if (proc.redo()) { display.refresh(); rebuildBandPanel(); } };

    abA.setTooltip (U ("Réglage A")); abB.setTooltip (U ("Réglage B : comparez deux égalisations"));
    abA.onClick = [this] { proc.switchAB ("A"); display.refresh(); rebuildBandPanel(); };
    abB.onClick = [this] { proc.switchAB ("B"); display.refresh(); rebuildBandPanel(); };
    abCopy.setTooltip (U ("Copie le réglage affiché dans l'autre emplacement (A → B ou B → A)"));
    abCopy.onClick = [this]
    {
        proc.copyABToOther();
        toast.show (proc.currentAB() == "A" ? U ("A copié dans B") : U ("B copié dans A"), true);
    };

    drawBtn.icon = pencilIcon;
    drawBtn.setTooltip (U ("Dessinez la courbe voulue : les bandes sont placées automatiquement au relâchement (Échap pour quitter)"));
    drawBtn.onClick = [this] { display.setSketchMode (! display.isSketchMode()); };

    analyzerBox.addItem (U ("Analyseur : avant + après"), 1);
    analyzerBox.addItem (U ("Analyseur : après"), 2);
    analyzerBox.addItem (U ("Analyseur : avant"), 3);
    analyzerBox.addItem (U ("Analyseur : désactivé"), 4);
    analyzerBox.setTitle (U ("Analyseur"));
    {
        const auto m = proc.ui.getProperty ("analyzer", "prepost").toString();
        analyzerBox.setSelectedId (m == "post" ? 2 : m == "pre" ? 3 : m == "off" ? 4 : 1, juce::dontSendNotification);
    }
    analyzerBox.onChange = [this]
    {
        const char* modes[] = { "prepost", "post", "pre", "off" };
        proc.ui.setProperty ("analyzer", modes[juce::jlimit (0, 3, analyzerBox.getSelectedId() - 1)], nullptr);
        display.refresh();
    };

    phaseSeg.attach (prm (P::phase));
    {
        auto f = phaseSeg.onSelect;
        phaseSeg.onSelect = [this, f] (int i)
        {
            f (i);
            toast.show (i == 1 ? U ("Phase linéaire : aucune rotation de phase, latence ") + String (juce::roundToInt (1000.0 * LinearPhase::firLengthFor (proc.getSampleRateSafe()) / 2 / proc.getSampleRateSafe())) + " ms"
                               : U ("Zéro latence : filtres analogiques, aucune latence"));
        };
    }
    phaseSeg.setTitle (U ("Mode de phase"));

    bypassAtt = std::make_unique<juce::ParameterAttachment> (prm (P::bypass), [this] (float v)
    {
        bypassed = v > 0.5f; bypassBtn.on = ! bypassed; bypassBtn.repaint();
        display.setAlpha (bypassed ? 0.45f : 1.0f);
    }, nullptr);
    bypassBtn.onClick = [this] { bypassAtt->setValueAsCompleteGesture (bypassed ? 0.0f : 1.0f); };
    bypassAtt->sendInitialUpdate();

    for (juce::Component* c : { (juce::Component*) &undoBtn, (juce::Component*) &redoBtn, (juce::Component*) &abA, (juce::Component*) &abB,
                                (juce::Component*) &abCopy, (juce::Component*) &drawBtn, (juce::Component*) &analyzerBox,
                                (juce::Component*) &phaseSeg, (juce::Component*) &bypassBtn })
        content.addAndMakeVisible (c);

    // Réglages généraux
    auto acc = [] (const Pal& q) { return q.accent; };
    auto grn = [] (const Pal& q) { return q.green; };
    auto ora = [] (const Pal& q) { return q.orange; };
    kOut = std::make_unique<ArcKnob> (theme, prm (P::outGain), U ("Sortie"), "dB", grn, ArcKnob::Size::xs,
                                      [] (float v) { return P::fmtDb (v); }, 0.0f, U ("Gain de sortie"));
    kScale = std::make_unique<ArcKnob> (theme, prm (P::gainScale), U ("Échelle"), "%", acc, ArcKnob::Size::xs,
                                        [] (float v) { return String (juce::roundToInt (v)); }, 100.0f,
                                        U ("Échelle de gain : multiplie le gain de toutes les bandes (100 % = tel quel, 0 % = courbe plate, 200 % = doublé)"));
    kAtk = std::make_unique<ArcKnob> (theme, prm (P::attack), U ("Attaque"), "ms", ora, ArcKnob::Size::xs,
                                      [] (float v) { return v < 10 ? String (v, 1) : String (juce::roundToInt (v)); }, 0.5f,
                                      U ("Vitesse de réaction de l'EQ dynamique (toutes les bandes)"));
    kRel = std::make_unique<ArcKnob> (theme, prm (P::release), U ("Retour"), "ms", ora, ArcKnob::Size::xs,
                                      [] (float v) { return String (juce::roundToInt (v)); }, 10.0f,
                                      U ("Temps de retour de l'EQ dynamique (toutes les bandes)"));
    for (auto* k : { kOut.get(), kScale.get(), kAtk.get(), kRel.get() })
    {
        k->onGestureStart = [this] { proc.pushUndo(); };
        content.addAndMakeVisible (k);
    }
    autoBtn.setTooltip (U ("Gain auto : compense le volume moyen ajouté ou retiré par l'égalisation, pour comparer à niveau égal"));
    autoBtn.onClick = [this]
    {
        proc.pushUndo();
        proc.setValue (proc.apvts.getParameter (P::autoGain), proc.getParam (P::autoGain) > 0.5f ? 0.0f : 1.0f);
    };
    content.addAndMakeVisible (autoBtn);

    // Panneau de bande
    onBtn.setTooltip (U ("Activer / désactiver la bande (double-clic sur la poignée)"));
    soloBtn.icon = headphonesIcon;
    soloBtn.setTooltip (U ("Écouter seulement la zone du spectre traitée par la bande (ou Alt + clic maintenu sur la poignée)"));
    delBtn.icon = trashIcon;
    delBtn.setTooltip (U ("Supprimer la bande (touche Effacer)"));
    dynBtn.setTooltip (U ("EQ dynamique : le gain de la bande varie selon le niveau du signal dans sa zone de fréquences"));
    for (int i = 0; i < numShapes; ++i) shapeBox.addItem (P::shapeNames()[i], i + 1);
    for (int i = 0; i < numSlopes; ++i) slopeBox.addItem (P::slopeNames()[i], i + 1);
    shapeBox.setTitle (U ("Forme")); slopeBox.setTitle (U ("Pente"));
    placeSeg.setTitle (U ("Placement stéréo"));
    placeSeg.setTooltip (U ("Stéréo, gauche, droite, Mid (centre) ou Side (côtés)"));

    onBtn.onClick = [this]
    {
        proc.pushUndo();
        const auto& sel = display.getSelection();
        const bool anyOn = std::any_of (sel.begin(), sel.end(), [this] (int b) { return proc.bandValue (b, P::fOn) > 0.5f; });
        for (int b : sel) proc.setBandValue (b, P::fOn, anyOn ? 0.0f : 1.0f);
        display.refresh(); syncBandPanel();
    };
    soloBtn.onClick = [this]
    {
        if (panelBand < 0) return;
        proc.setSolo (proc.getSolo() == panelBand ? -1 : panelBand);
        display.refresh(); syncBandPanel();
    };
    delBtn.onClick = [this] { display.deleteSelected(); };
    dynBtn.onClick = [this]
    {
        if (panelBand < 0) return;
        proc.pushUndo();
        proc.setBandValue (panelBand, P::fDyn, proc.bandValue (panelBand, P::fDyn) > 0.5f ? 0.0f : 1.0f);
        display.refresh(); syncBandPanel();
    };
    shapeBox.onChange = [this]
    {
        const int v = shapeBox.getSelectedId() - 1;
        if (v < 0 || panelBand < 0 || v == juce::roundToInt (proc.bandValue (panelBand, P::fShape))) return;
        proc.pushUndo();
        for (int b : display.getSelection()) proc.setBandValue (b, P::fShape, (float) v);
        display.refresh(); syncBandPanel();
    };
    slopeBox.onChange = [this]
    {
        const int v = slopeBox.getSelectedId() - 1;
        if (v < 0 || panelBand < 0 || v == juce::roundToInt (proc.bandValue (panelBand, P::fSlope))) return;
        proc.pushUndo();
        for (int b : display.getSelection()) proc.setBandValue (b, P::fSlope, (float) v);
        display.refresh();
    };
    placeSeg.onSelect = [this] (int v)
    {
        proc.pushUndo();
        for (int b : display.getSelection()) proc.setBandValue (b, P::fPlace, (float) v);
        display.refresh();
    };
    for (juce::Component* c : { (juce::Component*) &onBtn, (juce::Component*) &soloBtn, (juce::Component*) &delBtn,
                                (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox,
                                (juce::Component*) &placeSeg })
        content.addChildComponent (c);
    content.addAndMakeVisible (toast);

    juce::Desktop::getInstance().addDarkModeSettingListener (this);
    applyTheme();
    rebuildBandPanel();

    const double sc = juce::jlimit (0.6, 2.0, (double) proc.ui.getProperty ("scale", 1.0));
    setResizable (true, true);
    getConstrainer()->setFixedAspectRatio ((double) baseW / baseH);
    setResizeLimits (baseW * 6 / 10, baseH * 6 / 10, baseW * 2, baseH * 2);
    setSize (juce::roundToInt (baseW * sc), juce::roundToInt (baseH * sc));
    startTimerHz (10);
}

AudioLabEqEditor::~AudioLabEqEditor()
{
    stopTimer();
    juce::Desktop::getInstance().removeDarkModeSettingListener (this);
    tooltips.setLookAndFeel (nullptr);
    setLookAndFeel (nullptr);
}

void AudioLabEqEditor::applyTheme()
{
    const String t = proc.ui.getProperty ("themeOverride", "auto").toString();
    const bool dark = t == "dark" || (t != "light" && juce::Desktop::getInstance().isDarkModeActive());
    theme.setDark (dark);
    lnf.refresh();
    sendLookAndFeelChange();
    content.repaint();
    for (auto* c : content.getChildren()) c->repaint();
}

bool AudioLabEqEditor::keyPressed (const juce::KeyPress& k) { return display.keyPressed (k); }

// ── Panneau de bande ────────────────────────────────────────────────────
void AudioLabEqEditor::rebuildBandPanel()
{
    const auto& sel = display.getSelection();
    panelCount = sel.size();
    panelBand = sel.size() >= 1 ? sel[0] : -1;
    kFreq.reset(); kGain.reset(); kQ.reset(); kThr.reset(); kRange.reset();

    const bool single = panelCount == 1, any = panelCount >= 1;
    for (auto* c : { (juce::Component*) &onBtn, (juce::Component*) &delBtn, (juce::Component*) &placeSeg })
        c->setVisible (any);
    for (auto* c : { (juce::Component*) &soloBtn, (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox })
        c->setVisible (single);

    if (single)
    {
        const int b = panelBand;
        auto bc = [b] (const Pal& q) { return bandColour (b, q); };
        auto& fq = *proc.bandParam (b, P::fFreq);
        kFreq = std::make_unique<ArcKnob> (theme, fq, U ("Fréquence"), "Hz", bc, ArcKnob::Size::xs,
            [] (float v) { return P::fmtHzShort (v); }, 10.0f, U ("Fréquence centrale ou de coupure"));
        kGain = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fGain), "Gain", "dB", bc, ArcKnob::Size::xs,
            [] (float v) { return P::fmtDb (v); }, 0.0f, U ("Gain de la bande (cloches, shelfs, tilt)"));
        kQ = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fQ), "Q", "", bc, ArcKnob::Size::xs,
            [] (float v) { return P::fmtQ (v); }, 0.025f, U ("Largeur de bande : Q élevé = étroit (molette sur la poignée)"));
        kThr = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fThr), U ("Seuil"), "dB", [] (const Pal& q) { return q.orange; }, ArcKnob::Size::xs,
            [] (float v) { return String (v, 1); }, 0.0f, U ("Niveau à partir duquel la bande dynamique agit"));
        kRange = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fRange), "Plage", "dB", [] (const Pal& q) { return q.orange; }, ArcKnob::Size::xs,
            [] (float v) { return P::fmtDb (v); }, 0.0f, U ("Gain ajouté (+) ou retiré (−) au maximum quand le signal dépasse le seuil (Alt + molette sur la poignée)"));
        for (auto* k : { kFreq.get(), kGain.get(), kQ.get(), kThr.get(), kRange.get() })
        {
            k->onGestureStart = [this] { proc.pushUndo(); };
            content.addAndMakeVisible (k);
        }
        panelTitle = U ("Bande ") + String (b + 1);
    }
    else if (any) panelTitle = String (panelCount) + U (" bandes sélectionnées");
    else panelTitle = {};

    layoutBandPanel();
    syncBandPanel();
    content.repaint();
}

void AudioLabEqEditor::syncBandPanel()
{
    if (panelBand < 0 || panelCount == 0) return;
    const int b = panelBand;
    const auto shape = (Shape) juce::roundToInt (proc.bandValue (b, P::fShape));
    const int slopeIdx = juce::roundToInt (proc.bandValue (b, P::fSlope));
    const auto& sel = display.getSelection();
    const bool anyOn = std::any_of (sel.begin(), sel.end(), [this] (int x) { return proc.bandValue (x, P::fOn) > 0.5f; });
    if (onBtn.on != anyOn) { onBtn.on = anyOn; onBtn.repaint(); }
    const bool solo = proc.getSolo() == b;
    if (soloBtn.on != solo) { soloBtn.on = solo; soloBtn.repaint(); }
    const bool dyn = proc.bandValue (b, P::fDyn) > 0.5f;
    if (dynBtn.on != dyn) { dynBtn.on = dyn; dynBtn.repaint(); }
    shapeBox.setSelectedId ((int) shape + 1, juce::dontSendNotification);
    slopeBox.setSelectedId (slopeIdx + 1, juce::dontSendNotification);
    slopeBox.setEnabled (usesSlope (shape));
    slopeBox.setAlpha (usesSlope (shape) ? 1.0f : 0.4f);
    const int place = juce::roundToInt (proc.bandValue (b, P::fPlace));
    if (placeSeg.getSelected() != place) placeSeg.setSelected (place);
    dynBtn.setEnabled (canBeDynamic (shape));
    dynBtn.setAlpha (canBeDynamic (shape) ? 1.0f : 0.4f);
    auto dim = [] (juce::Component* c, bool en) { if (c == nullptr) return; c->setEnabled (en); c->setAlpha (en ? 1.0f : 0.35f); };
    const int slope = slopeValues[juce::jlimit (0, numSlopes - 1, slopeIdx)];
    dim (kGain.get(), hasGain (shape));
    dim (kQ.get(), usesQ (shape, slope));
    dim (kThr.get(), dyn && canBeDynamic (shape));
    dim (kRange.get(), dyn && canBeDynamic (shape));
}

void AudioLabEqEditor::layoutBandPanel()
{
    auto pa = panelArea();
    const int x0 = pa.getX() + 14, y1 = pa.getY() + 12;
    // ligne 1 : titre · Active · Écouter · Supprimer · Forme · Pente
    int x = x0 + (panelCount > 1 ? 210 : 150);
    onBtn.setBounds (x, y1, 72, 28); x += 80;
    if (panelCount == 1) { soloBtn.setBounds (x, y1, juce::roundToInt (soloBtn.idealWidth()), 28); x += soloBtn.getWidth() + 8; }
    delBtn.setBounds (x, y1, 36, 28); x += 36 + 16;
    if (panelCount == 1)
    {
        shapeBox.setBounds (x, y1, 132, 28); x += 140;
        slopeBox.setBounds (x, y1, 118, 28);
    }

    // ligne 2 : potentiomètres, dynamique, placement stéréo
    const int y2 = pa.getY() + 58, kw = 78, kh = 82;
    int px = x0 + 84;
    if (kFreq != nullptr)
    {
        kFreq->setBounds (x0 - 4, y2, kw, kh);
        kGain->setBounds (x0 - 4 + kw, y2, kw, kh);
        kQ->setBounds (x0 - 4 + 2 * kw, y2, kw, kh);
        dynBtn.setBounds (x0 + 3 * kw + 18, y2 + 16, juce::roundToInt (dynBtn.idealWidth()), 28);
        const int dx = dynBtn.getRight() + 6;
        kThr->setBounds (dx, y2, kw, kh);
        kRange->setBounds (dx + kw, y2, kw, kh);
        px = dx + 2 * kw + 18;
    }
    placeSeg.setBounds (px, y2 + 16, juce::roundToInt (placeSeg.idealWidth()), 28);
}

void AudioLabEqEditor::timerCallback()
{
    // boutons qui dépendent de l'état
    auto setOn = [] (PillButton& b, bool on) { if (b.on != on) { b.on = on; b.repaint(); } };
    undoBtn.setEnabled (proc.canUndo()); undoBtn.setAlpha (proc.canUndo() ? 1.0f : 0.4f);
    redoBtn.setEnabled (proc.canRedo()); redoBtn.setAlpha (proc.canRedo() ? 1.0f : 0.4f);
    const auto ab = proc.currentAB();
    setOn (abA, ab == "A"); setOn (abB, ab == "B");
    const bool hasA = ab == "A" || proc.ui.getProperty ("slotA").toString().isNotEmpty();
    const bool hasB = ab == "B" || proc.ui.getProperty ("slotB").toString().isNotEmpty();
    if (abA.has != hasA) { abA.has = hasA; abA.repaint(); }
    if (abB.has != hasB) { abB.has = hasB; abB.repaint(); }
    setOn (autoBtn, proc.getParam (P::autoGain) > 0.5f);
    if (autoBtn.on)
    {
        const String tip = U ("Gain auto : ") + P::fmtDb (proc.getAutoGainDb()) + " dB";
        if (autoBtn.getTooltip() != tip) autoBtn.setTooltip (tip);
    }
    const int pid = (int) proc.ui.getProperty ("preset", 0) + 1;
    if (presetBox.getSelectedId() != pid) presetBox.setSelectedId (pid, juce::dontSendNotification);
    // le panneau suit les changements faits ailleurs (automation, menu, molette…)
    const auto& sel = display.getSelection();
    if (sel.size() != panelCount || (panelCount > 0 && sel[0] != panelBand)) rebuildBandPanel();
    else syncBandPanel();
}

void AudioLabEqEditor::resized()
{
    const float s = (float) getWidth() / baseW;
    content.setTransform (juce::AffineTransform::scale (s));
    content.setBounds (0, 0, baseW, baseH);
    proc.ui.setProperty ("scale", (double) s, nullptr);
}

void AudioLabEqEditor::Content::paint (juce::Graphics& g)
{
    auto& e = ed;
    const auto& p = e.theme.pal();
    g.fillAll (p.bg2);

    // logo
    flaskIcon (g, { 18, 16, 22, 26 }, p.accent);
    g.setColour (p.text); g.setFont (e.theme.ui (17, 700, -0.01f));
    g.drawText ("Audio Lab", juce::Rectangle<float> (48, 12, 120, 20), juce::Justification::centredLeft, false);
    g.setColour (p.accent); g.setFont (e.theme.ui (17, 700, -0.01f));
    g.drawText ("EQ", juce::Rectangle<float> (134, 12, 40, 20), juce::Justification::centredLeft, false);
    g.setColour (p.muted); g.setFont (e.theme.ui (10));
    g.drawText (U ("Chimiste du son"), juce::Rectangle<float> (48, 31, 120, 13), juce::Justification::centredLeft, false);

    // panneau de bande
    auto pa = e.panelArea().toFloat();
    g.setColour (p.bg); g.fillRoundedRectangle (pa, 12);
    g.setColour (p.border); g.drawRoundedRectangle (pa.reduced (0.5f), 12, 1);

    const float a = e.bypassed ? 0.45f : 1.0f;
    const float x0 = pa.getX() + 14, y1 = pa.getY() + 12;
    if (e.panelCount > 0)
    {
        if (e.panelCount == 1)
        {
            g.setColour (bandColour (e.panelBand, p).withMultipliedAlpha (a));
            g.fillEllipse (x0, y1 + 7, 14, 14);
        }
        g.setColour (p.text.withMultipliedAlpha (a)); g.setFont (e.theme.ui (14, 600));
        g.drawText (e.panelTitle, juce::Rectangle<float> (x0 + (e.panelCount == 1 ? 22 : 0), y1, e.panelCount == 1 ? 120.0f : 200.0f, 28), juce::Justification::centredLeft, false);
        if (e.panelCount > 1)
        {
            g.setColour (p.muted); g.setFont (e.theme.ui (12));
            g.drawText (U ("Glissez une poignée pour déplacer toute la sélection · Maj : réglage fin"),
                        juce::Rectangle<float> (x0, pa.getY() + 112, 560, 20), juce::Justification::centredLeft, false);
            g.setColour (p.dim); g.setFont (e.theme.ui (12));
            g.drawText (U ("Placement"), juce::Rectangle<float> ((float) e.placeSeg.getX() - 78, (float) e.placeSeg.getY(), 72, 28), juce::Justification::centredRight, false);
        }
    }
    else
    {
        g.setColour (p.muted); g.setFont (e.theme.ui (13));
        g.drawText (U ("Aucune bande sélectionnée"), juce::Rectangle<float> (x0, y1, 400, 28), juce::Justification::centredLeft, false);
        g.setFont (e.theme.ui (11.5f));
        const char* tips[] = { "Clic dans la courbe : nouvelle bande (bords : coupe-bas / coupe-haut)",
                               "Glisser : fréquence et gain · Maj : fin · Molette : Q · Alt + molette : plage dynamique",
                               "Alt + clic maintenu : écouter la bande · Double-clic : activer / désactiver · Clic droit : options",
                               "Cmd+Z / Cmd+Maj+Z : annuler / rétablir · Effacer : supprimer · Cmd+A : tout sélectionner" };
        for (int i = 0; i < 4; ++i)
            g.drawText (U (tips[i]), juce::Rectangle<float> (x0, y1 + 34 + i * 20.0f, 640, 18), juce::Justification::centredLeft, false);
    }

    // réglages généraux (à droite)
    const float gx = (float) e.kAtk->getX() - 14;
    g.setColour (p.border); g.drawVerticalLine (juce::roundToInt (gx), pa.getY() + 14, pa.getBottom() - 14);
    g.setColour (p.muted); g.setFont (e.theme.ui (10.5f, 600, 0.06f));
    g.drawText (U ("RÉGLAGES GÉNÉRAUX"), juce::Rectangle<float> (gx + 16, y1, 160, 28), juce::Justification::centredLeft, false);
    g.setFont (e.theme.ui (9.5f));
    g.drawText (U ("dynamique : toutes les bandes"), juce::Rectangle<float> ((float) e.kAtk->getX(), pa.getBottom() - 17, 156, 12), juce::Justification::centred, false);
}

void AudioLabEqEditor::Content::resized()
{
    auto& e = ed;
    // barre du haut : à gauche presets, annuler/rétablir, A/B ; à droite dessin, analyseur, phase, bypass
    const int y = 13, h = 30;
    e.presetBox.setBounds (186, y, 206, h);
    e.undoBtn.setBounds (400, y, 34, h);
    e.redoBtn.setBounds (438, y, 34, h);
    e.abA.setBounds (484, y + 2, 26, h - 4);
    e.abB.setBounds (514, y + 2, 26, h - 4);
    e.abCopy.setBounds (544, y + 2, juce::roundToInt (e.abCopy.idealWidth()), h - 4);
    int rx = baseW - 16;
    const int bw = juce::roundToInt (e.bypassBtn.idealWidth()) + 8;
    e.bypassBtn.setBounds (rx - bw, y + 2, bw, h - 4); rx -= bw + 12;
    const int pw = juce::roundToInt (e.phaseSeg.idealWidth());
    e.phaseSeg.setBounds (rx - pw, y, pw, h); rx -= pw + 10;
    e.analyzerBox.setBounds (rx - 180, y, 180, h); rx -= 180 + 10;
    const int dw = juce::roundToInt (e.drawBtn.idealWidth());
    e.drawBtn.setBounds (rx - dw, y + 1, dw, h - 2);

    e.display.setBounds (16, 56, baseW - 32, 466);

    // réglages généraux (à droite du panneau)
    auto pa = e.panelArea();
    const int kw = 78, kh = 82, y2 = pa.getY() + 40;
    const int gx = pa.getRight() - 14 - 4 * kw - 8;
    e.kAtk->setBounds (gx, y2 + 6, kw, kh);
    e.kRel->setBounds (gx + kw, y2 + 6, kw, kh);
    e.kScale->setBounds (gx + 2 * kw + 8, y2 + 6, kw, kh);
    e.kOut->setBounds (gx + 3 * kw + 8, y2 + 6, kw, kh);
    e.autoBtn.setBounds (pa.getRight() - 14 - juce::roundToInt (e.autoBtn.idealWidth()), pa.getY() + 12, juce::roundToInt (e.autoBtn.idealWidth()), 26);
    e.layoutBandPanel();
    e.toast.setBounds (getLocalBounds());
}
