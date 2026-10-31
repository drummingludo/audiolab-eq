// Composants d'interface d'Audio Lab, reproduits pour le plugin :
// potentiomètres à arc, Dry/Wet, boutons segmentés, curseurs, boutons, menus, toast.
#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "Theme.h"

namespace audiolab
{
// Ombre douce façon CSS box-shadow
inline void softShadow (juce::Graphics& g, const juce::Path& p, juce::Colour c, int radius, juce::Point<int> offset)
{
    juce::DropShadow (c, radius, offset).drawForPath (g, p);
}

// Majuscules, y compris les lettres accentuées du français
inline juce::String upper (const juce::String& s)
{
    juce::String r = s.toUpperCase();
    const char* pairs[][2] = { { "é", "É" }, { "è", "È" }, { "ê", "Ê" }, { "à", "À" }, { "â", "Â" }, { "ç", "Ç" }, { "ô", "Ô" }, { "û", "Û" }, { "î", "Î" } };
    for (auto& pr : pairs) r = r.replace (juce::String::fromUTF8 (pr[0]), juce::String::fromUTF8 (pr[1]));
    return r;
}

// ── Potentiomètre à arc (opKnob) ─────────────────────────────────────────
class ArcKnob : public juce::Component, public juce::SettableTooltipClient
{
public:
    enum class Size { md, sm, xs };

    ArcKnob (Theme& t, juce::RangedAudioParameter& prm, juce::String label, juce::String unit,
             std::function<juce::Colour (const Pal&)> colour, Size size, std::function<juce::String (float)> format,
             float zeroValue, juce::String help = {})
        : theme (t), param (prm), labelText (std::move (label)), unitText (std::move (unit)),
          colourFn (std::move (colour)), sz (size), fmt (std::move (format)),
          zeroNorm (prm.convertTo0to1 (zeroValue)),
          attachment (prm, [this] (float) { repaint(); }, nullptr)
    {
        setWantsKeyboardFocus (true);
        setTitle (labelText);
        setTooltip (help);
        setMouseCursor (juce::MouseCursor::UpDownResizeCursor);
        attachment.sendInitialUpdate();
    }

    float knobDiameter() const { return sz == Size::md ? 112.0f : sz == Size::sm ? 84.0f : 58.0f; }
    std::function<void()> onGestureStart;   // pour « Annuler »

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        const float S = knobDiameter(), s = S / 150.0f;
        const float cx = getWidth() * 0.5f, top = (sz == Size::sm ? 14.0f : sz == Size::xs ? 2.0f : 0.0f);
        const float cy = top + S * 0.5f, r = 66 * s, sw = 7 * s;
        const float A0 = -135.0f * juce::MathConstants<float>::pi / 180, A1 = -A0;
        const float n = param.getValue();
        auto ang = [&] (float t) { return A0 + (A1 - A0) * t; };

        juce::Path trk; trk.addCentredArc (cx, cy, r, r, 0, A0, A1, true);
        g.setColour (p.bg4); g.strokePath (trk, juce::PathStrokeType (sw, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        const float a = ang (n), az = ang (zeroNorm);
        if (std::abs (a - az) >= 0.004f)
        {
            juce::Path arc; arc.addCentredArc (cx, cy, r, r, 0, std::min (a, az), std::max (a, az), true);
            g.setColour (colourFn (p));
            g.strokePath (arc, juce::PathStrokeType (sw, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        }
        // capuchon
        const float cd = sz == Size::md ? 74.0f : sz == Size::sm ? 56.0f : 42.0f;
        juce::Path cap; cap.addEllipse (cx - cd / 2, cy - cd / 2, cd, cd);
        softShadow (g, cap, juce::Colours::black.withAlpha (p.dark ? 0.45f : 0.14f), sz == Size::xs ? 10 : 20, { 0, sz == Size::xs ? 4 : 8 });
        softShadow (g, cap, juce::Colours::black.withAlpha (p.dark ? 0.4f : 0.08f), 2, { 0, 1 });
        g.setColour (p.bg2); g.fillPath (cap);
        if (focused)
        {
            g.setColour (p.accent); g.drawEllipse (cx - S / 2 + 1.5f, cy - S / 2 + 1.5f, S - 3, S - 3, 3.0f);
        }
        // valeur + unité
        const float vs = sz == Size::md ? 18.0f : sz == Size::sm ? 13.0f : 10.5f, us = sz == Size::md ? 11.0f : sz == Size::sm ? 9.0f : 8.0f;
        g.setColour (p.text); g.setFont (theme.ui (vs, 600, -0.02f));
        g.drawText (fmt (param.convertFrom0to1 (n)), juce::Rectangle<float> (cx - cd / 2, cy - vs * 0.78f, cd, vs * 1.2f), juce::Justification::centred, false);
        g.setColour (p.muted); g.setFont (theme.ui (us));
        g.drawText (unitText, juce::Rectangle<float> (cx - cd / 2, cy + vs * 0.38f, cd, us * 1.3f), juce::Justification::centred, false);
        // libellé
        g.setColour (p.text); g.setFont (theme.ui (sz == Size::xs ? 9.5f : 11, 600, 0.06f));
        const float ly = top + S + (sz == Size::sm ? 14.0f : 0.0f) + (sz == Size::xs ? 0.0f : 2.0f);
        g.drawText (upper (labelText), juce::Rectangle<float> (0, ly, (float) getWidth(), 16), juce::Justification::centredTop, false);
    }

    void mouseDown (const juce::MouseEvent& e) override
    {
        if (! isEnabled()) return;
        if (onGestureStart) onGestureStart();
        y0 = e.position.y; v0 = param.getValue();
        attachment.beginGesture(); dragging = true;
    }
    void mouseDrag (const juce::MouseEvent& e) override
    {
        if (! dragging) return;
        const float k = e.mods.isShiftDown() ? 1200.0f : 220.0f;   // Maj = réglage fin
        setNorm (v0 + (y0 - e.position.y) / k, true);
    }
    void mouseUp (const juce::MouseEvent&) override { if (dragging) { attachment.endGesture(); dragging = false; } }
    void mouseDoubleClick (const juce::MouseEvent&) override
    {
        if (onGestureStart) onGestureStart();
        attachment.setValueAsCompleteGesture (param.convertFrom0to1 (param.getDefaultValue()));
    }
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails& w) override
    {
        if (std::abs (w.deltaY) < 1e-6f || ! isEnabled()) return;
        if (onGestureStart && juce::Time::getMillisecondCounter() - lastWheel > 600) onGestureStart();
        lastWheel = juce::Time::getMillisecondCounter();
        setNorm (param.getValue() + (w.deltaY > 0 ? 0.01f : -0.01f), false);
    }
    bool keyPressed (const juce::KeyPress& k) override
    {
        const float st = 1.0f / 50;
        if (k.isKeyCode (juce::KeyPress::upKey) || k.isKeyCode (juce::KeyPress::rightKey)) { setNorm (param.getValue() + st, false); return true; }
        if (k.isKeyCode (juce::KeyPress::downKey) || k.isKeyCode (juce::KeyPress::leftKey)) { setNorm (param.getValue() - st, false); return true; }
        if (k.isKeyCode (juce::KeyPress::homeKey)) { setNorm (0, false); return true; }
        if (k.isKeyCode (juce::KeyPress::endKey)) { setNorm (1, false); return true; }
        return false;
    }
    void focusGained (FocusChangeType c) override { focused = c == focusChangedByTabKey; repaint(); }
    void focusLost (FocusChangeType) override { focused = false; repaint(); }

    std::unique_ptr<juce::AccessibilityHandler> createAccessibilityHandler() override
    {
        struct Value : juce::AccessibilityRangedNumericValueInterface
        {
            explicit Value (ArcKnob& k) : knob (k) {}
            bool isReadOnly() const override { return false; }
            double getCurrentValue() const override { return knob.param.convertFrom0to1 (knob.param.getValue()); }
            void setValue (double v) override { knob.attachment.setValueAsCompleteGesture ((float) v); }
            AccessibleValueRange getRange() const override
            {
                const auto& r = knob.param.getNormalisableRange();
                return { { r.start, r.end }, r.interval > 0 ? r.interval : 0.01 };
            }
            ArcKnob& knob;
        };
        return std::make_unique<juce::AccessibilityHandler> (*this, juce::AccessibilityRole::slider,
                                                              juce::AccessibilityActions(),
                                                              juce::AccessibilityHandler::Interfaces { std::make_unique<Value> (*this) });
    }

private:
    void setNorm (float n, bool partOfGesture)
    {
        n = juce::jlimit (0.0f, 1.0f, n);
        const float v = param.convertFrom0to1 (n);
        if (partOfGesture) attachment.setValueAsPartOfGesture (v);
        else attachment.setValueAsCompleteGesture (v);
    }

    Theme& theme;
    juce::RangedAudioParameter& param;
    juce::String labelText, unitText;
    std::function<juce::Colour (const Pal&)> colourFn;
    Size sz;
    std::function<juce::String (float)> fmt;
    float zeroNorm;
    juce::ParameterAttachment attachment;
    float y0 = 0, v0 = 0;
    bool dragging = false, focused = false;
    juce::uint32 lastWheel = 0;
};

// ── Dry/Wet (dknob) ─────────────────────────────────────────────────────
class MixKnob : public juce::Component, public juce::SettableTooltipClient
{
public:
    MixKnob (Theme& t, juce::RangedAudioParameter& prm)
        : theme (t), param (prm), attachment (prm, [this] (float) { repaint(); }, nullptr)
    {
        setTooltip (fr ("Dry / Wet — glisser verticalement"));
        setMouseCursor (juce::MouseCursor::UpDownResizeCursor);
        setTitle ("Dry/Wet");
        attachment.sendInitialUpdate();
    }
    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        const float v = param.convertFrom0to1 (param.getValue()), n = v / 100;
        const float x = getWidth() * 0.5f, y = 19, r = 15;
        g.setColour (p.cv); g.fillEllipse (x - r, y - r, 2 * r, 2 * r);
        g.setColour (p.grid); g.drawEllipse (x - r, y - r, 2 * r, 2 * r, 1);
        // arc de 135° à 405° (sens horaire depuis 3 h) = -135°…+135° depuis midi
        const float A0 = juce::MathConstants<float>::pi * 1.25f;
        juce::Path bgArc; bgArc.addCentredArc (x, y, r - 2, r - 2, 0, A0, A0 + 1.5f * juce::MathConstants<float>::pi, true);
        g.setColour (p.grid); g.strokePath (bgArc, juce::PathStrokeType (2.5f));
        if (n > 0.001f)
        {
            juce::Path a; a.addCentredArc (x, y, r - 2, r - 2, 0, A0, A0 + n * 1.5f * juce::MathConstants<float>::pi, true);
            g.setColour (p.accent); g.strokePath (a, juce::PathStrokeType (2.5f));
        }
        const float ang = A0 + n * 1.5f * juce::MathConstants<float>::pi;
        g.setColour (p.txt2);
        g.drawLine (x, y, x + std::sin (ang) * (r - 5), y - std::cos (ang) * (r - 5), 1.5f);
        g.setColour (p.muted); g.setFont (theme.mono (9));
        g.drawText (juce::String (juce::roundToInt (v)) + "%", juce::Rectangle<float> (0, 39, (float) getWidth(), 12), juce::Justification::centred, false);
    }
    void mouseDown (const juce::MouseEvent& e) override { sy = e.position.y; sv = param.convertFrom0to1 (param.getValue()); attachment.beginGesture(); }
    void mouseDrag (const juce::MouseEvent& e) override
    {
        const float nv = juce::jlimit (0.0f, 100.0f, sv + (sy - e.position.y) * (e.mods.isShiftDown() ? 0.2f : 0.8f));
        attachment.setValueAsPartOfGesture (nv);
    }
    void mouseUp (const juce::MouseEvent&) override { attachment.endGesture(); }
    void mouseDoubleClick (const juce::MouseEvent&) override { attachment.setValueAsCompleteGesture (100.0f); }
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails& w) override
    {
        if (std::abs (w.deltaY) < 1e-6f) return;
        attachment.setValueAsCompleteGesture (juce::jlimit (0.0f, 100.0f, param.convertFrom0to1 (param.getValue()) + (w.deltaY > 0 ? 1.0f : -1.0f)));
    }

private:
    Theme& theme;
    juce::RangedAudioParameter& param;
    juce::ParameterAttachment attachment;
    float sy = 0, sv = 0;
};

// ── Boutons segmentés (.wbtns .wb) ───────────────────────────────────────
class SegButtons : public juce::Component, public juce::SettableTooltipClient
{
public:
    SegButtons (Theme& t, juce::StringArray opts) : theme (t), options (std::move (opts)) {}

    // Relie à un paramètre « choix » (Peak/RMS…)
    void attach (juce::RangedAudioParameter& prm)
    {
        attachment = std::make_unique<juce::ParameterAttachment> (prm, [this, &prm] (float) {
            selected = juce::roundToInt (prm.convertFrom0to1 (prm.getValue())); repaint(); }, nullptr);
        onSelect = [this] (int i) { attachment->setValueAsCompleteGesture ((float) i); };
        attachment->sendInitialUpdate();
    }

    void setSelected (int i) { selected = i; repaint(); }
    int getSelected() const { return selected; }
    std::function<void (int)> onSelect;

    float idealWidth() const
    {
        float w = 6;
        for (auto& o : options) w += textWidth (o) + 20 + 4;
        return w - 4;
    }

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        auto b = getLocalBounds().toFloat();
        g.setColour (p.bg3); g.fillRoundedRectangle (b, 9);
        for (int i = 0; i < options.size(); ++i)
        {
            auto r = cell (i);
            if (i == selected)
            {
                juce::Path pill; pill.addRoundedRectangle (r, 7);
                softShadow (g, pill, juce::Colours::black.withAlpha (0.12f), 3, { 0, 1 });
                g.setColour (p.bg2); g.fillPath (pill);
            }
            g.setColour (i == selected || i == hover ? p.text : p.dim);
            g.setFont (theme.ui (12));
            g.drawText (options[i], r, juce::Justification::centred, false);
        }
    }
    void mouseMove (const juce::MouseEvent& e) override { const int h = hit (e.position); if (h != hover) { hover = h; repaint(); } }
    void mouseExit (const juce::MouseEvent&) override { hover = -1; repaint(); }
    void mouseDown (const juce::MouseEvent& e) override
    {
        const int h = hit (e.position);
        if (h >= 0) { selected = h; repaint(); if (onSelect) onSelect (h); }
    }

private:
    float textWidth (const juce::String& s) const
    {
        juce::GlyphArrangement ga; ga.addLineOfText (theme.ui (12), s, 0, 0);
        return ga.getBoundingBox (0, -1, true).getWidth();
    }
    juce::Rectangle<float> cell (int i) const
    {
        float x = 3;
        for (int k = 0; k < i; ++k) x += textWidth (options[k]) + 20 + 4;
        return { x, 3.0f, textWidth (options[i]) + 20, (float) getHeight() - 6 };
    }
    int hit (juce::Point<float> pt) const
    {
        for (int i = 0; i < options.size(); ++i) if (cell (i).contains (pt)) return i;
        return -1;
    }
    Theme& theme;
    juce::StringArray options;
    int selected = 0, hover = -1;
    std::unique_ptr<juce::ParameterAttachment> attachment;
};

// ── Curseur horizontal (.slr) : libellé · piste · valeur ─────────────────
class SliderRow : public juce::Component, public juce::SettableTooltipClient
{
public:
    SliderRow (Theme& t, juce::RangedAudioParameter& prm, juce::String label, std::function<juce::String (float)> format)
        : theme (t), param (prm), labelText (std::move (label)), fmt (std::move (format)),
          attachment (prm, [this] (float) { repaint(); }, nullptr)
    {
        setWantsKeyboardFocus (true);
        setTitle (labelText);
        attachment.sendInitialUpdate();
    }
    void setDimmed (bool d) { if (dimmed != d) { dimmed = d; repaint(); } }

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        const float a = dimmed ? 0.4f : 1.0f;
        const float h = (float) getHeight(), cy = h * 0.5f;
        g.setColour (p.dim.withMultipliedAlpha (a)); g.setFont (theme.ui (12));
        g.drawText (labelText, juce::Rectangle<float> (0, 0, 76, h), juce::Justification::centredLeft, false);
        auto tr = track();
        g.setColour (p.bg4.withMultipliedAlpha (a)); g.fillRoundedRectangle (tr.getX(), cy - 2, tr.getWidth(), 4, 2);
        const float x = tr.getX() + 9 + (tr.getWidth() - 18) * param.getValue();
        juce::Path th; th.addEllipse (x - 9, cy - 9, 18, 18);
        softShadow (g, th, juce::Colours::black.withAlpha (0.2f * a), 6, { 0, 2 });
        g.setColour (juce::Colours::white.withMultipliedAlpha (a)); g.fillPath (th);
        g.setColour (juce::Colours::black.withAlpha (0.08f * a)); g.strokePath (th, juce::PathStrokeType (0.5f));
        if (focused) { g.setColour (p.accent.withAlpha (0.3f)); g.drawEllipse (x - 10.5f, cy - 10.5f, 21, 21, 3); }
        g.setColour (p.text.withMultipliedAlpha (a)); g.setFont (theme.mono (12));
        g.drawText (fmt (param.convertFrom0to1 (param.getValue())), juce::Rectangle<float> ((float) getWidth() - 60, 0, 60, h), juce::Justification::centredRight, false);
    }
    void mouseDown (const juce::MouseEvent& e) override { attachment.beginGesture(); drag (e); }
    void mouseDrag (const juce::MouseEvent& e) override { drag (e); }
    void mouseUp (const juce::MouseEvent&) override { attachment.endGesture(); }
    void mouseDoubleClick (const juce::MouseEvent&) override { attachment.setValueAsCompleteGesture (param.convertFrom0to1 (param.getDefaultValue())); }
    bool keyPressed (const juce::KeyPress& k) override
    {
        const float st = param.getNormalisableRange().interval > 0
                           ? param.getNormalisableRange().interval / (param.getNormalisableRange().end - param.getNormalisableRange().start) : 0.01f;
        float n = param.getValue();
        if (k.isKeyCode (juce::KeyPress::rightKey) || k.isKeyCode (juce::KeyPress::upKey)) n += std::max (st, 0.001f);
        else if (k.isKeyCode (juce::KeyPress::leftKey) || k.isKeyCode (juce::KeyPress::downKey)) n -= std::max (st, 0.001f);
        else return false;
        attachment.setValueAsCompleteGesture (param.convertFrom0to1 (juce::jlimit (0.0f, 1.0f, n)));
        return true;
    }
    void focusGained (FocusChangeType c) override { focused = c == focusChangedByTabKey; repaint(); }
    void focusLost (FocusChangeType) override { focused = false; repaint(); }

private:
    juce::Rectangle<float> track() const { return { 86.0f, 0, (float) getWidth() - 86 - 70, (float) getHeight() }; }
    void drag (const juce::MouseEvent& e)
    {
        auto tr = track();
        const float n = juce::jlimit (0.0f, 1.0f, (e.position.x - tr.getX() - 9) / (tr.getWidth() - 18));
        attachment.setValueAsPartOfGesture (param.convertFrom0to1 (n));
    }
    Theme& theme;
    juce::RangedAudioParameter& param;
    juce::String labelText;
    std::function<juce::String (float)> fmt;
    juce::ParameterAttachment attachment;
    bool dimmed = false, focused = false;
};

// ── Valeur compacte réglable : glisser verticalement, molette, double-clic = défaut ──
// stacked : libellé au-dessus de la valeur (Attaque / Retour) ; sinon côte à côte (Échelle 100 %).
class ValueBox : public juce::Component, public juce::SettableTooltipClient
{
public:
    ValueBox (Theme& t, juce::RangedAudioParameter& prm, juce::String label, std::function<juce::String (float)> format, bool stackedLayout)
        : theme (t), param (prm), labelText (std::move (label)), fmt (std::move (format)), stacked (stackedLayout),
          attachment (prm, [this] (float) { repaint(); }, nullptr)
    {
        setWantsKeyboardFocus (true);
        setTitle (labelText);
        setMouseCursor (juce::MouseCursor::UpDownResizeCursor);
        attachment.sendInitialUpdate();
    }
    bool raised = false;
    std::function<void()> onGestureStart;

    float idealWidth() const
    {
        juce::GlyphArrangement a; a.addLineOfText (theme.ui (11), labelText, 0, 0);
        juce::GlyphArrangement b; b.addLineOfText (theme.ui (12, 600), fmt (param.convertFrom0to1 (param.getValue())), 0, 0);
        return a.getBoundingBox (0, -1, true).getWidth() + 6 + std::max (b.getBoundingBox (0, -1, true).getWidth(), 40.0f) + 20;
    }

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        auto b = getLocalBounds().toFloat();
        g.setColour (isMouseOver (true) && isEnabled() ? p.ctlHover (raised) : p.ctl (raised)); g.fillRoundedRectangle (b, 8);
        if (focused) { g.setColour (p.accent.withAlpha (0.35f)); g.drawRoundedRectangle (b.reduced (1), 8, 2); }
        const auto v = fmt (param.convertFrom0to1 (param.getValue()));
        if (stacked)
        {
            g.setColour (p.muted); g.setFont (theme.ui (8.5f, 600, 0.06f));
            g.drawText (upper (labelText), b.withHeight (11).withCentre ({ b.getCentreX(), b.getCentreY() - 6.5f }), juce::Justification::centred, false);
            g.setColour (p.text); g.setFont (theme.ui (11.5f, 600));
            g.drawText (v, b.withHeight (14).withCentre ({ b.getCentreX(), b.getCentreY() + 6.0f }), juce::Justification::centred, false);
        }
        else
        {
            juce::GlyphArrangement a; a.addLineOfText (theme.ui (11), labelText, 0, 0);
            const float lw = a.getBoundingBox (0, -1, true).getWidth();
            auto r = b.reduced (10, 0);
            g.setColour (p.muted); g.setFont (theme.ui (11));
            g.drawText (labelText, r.removeFromLeft (lw + 1), juce::Justification::centredLeft, false);
            r.removeFromLeft (6);
            g.setColour (p.text); g.setFont (theme.ui (12, 600));
            g.drawText (v, r, juce::Justification::centredLeft, false);
        }
    }
    void mouseEnter (const juce::MouseEvent&) override { repaint(); }
    void mouseExit (const juce::MouseEvent&) override { repaint(); }
    void mouseDown (const juce::MouseEvent& e) override
    {
        if (! isEnabled()) return;
        if (onGestureStart) onGestureStart();
        y0 = e.position.y; v0 = param.getValue();
        attachment.beginGesture(); dragging = true;
    }
    void mouseDrag (const juce::MouseEvent& e) override
    {
        if (! dragging) return;
        const float k = e.mods.isShiftDown() ? 1200.0f : 220.0f;
        setNorm (v0 + (y0 - e.position.y) / k, true);
    }
    void mouseUp (const juce::MouseEvent&) override { if (dragging) { attachment.endGesture(); dragging = false; } }
    void mouseDoubleClick (const juce::MouseEvent&) override
    {
        if (! isEnabled()) return;
        if (onGestureStart) onGestureStart();
        attachment.setValueAsCompleteGesture (param.convertFrom0to1 (param.getDefaultValue()));
    }
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails& w) override
    {
        if (std::abs (w.deltaY) < 1e-6f || ! isEnabled()) return;
        if (onGestureStart && juce::Time::getMillisecondCounter() - lastWheel > 600) onGestureStart();
        lastWheel = juce::Time::getMillisecondCounter();
        setNorm (param.getValue() + (w.deltaY > 0 ? 0.01f : -0.01f), false);
    }
    bool keyPressed (const juce::KeyPress& k) override
    {
        if (k.isKeyCode (juce::KeyPress::upKey) || k.isKeyCode (juce::KeyPress::rightKey)) { setNorm (param.getValue() + 0.02f, false); return true; }
        if (k.isKeyCode (juce::KeyPress::downKey) || k.isKeyCode (juce::KeyPress::leftKey)) { setNorm (param.getValue() - 0.02f, false); return true; }
        return false;
    }
    void focusGained (FocusChangeType c) override { focused = c == focusChangedByTabKey; repaint(); }
    void focusLost (FocusChangeType) override { focused = false; repaint(); }

    std::unique_ptr<juce::AccessibilityHandler> createAccessibilityHandler() override
    {
        struct Value : juce::AccessibilityRangedNumericValueInterface
        {
            explicit Value (ValueBox& v) : box (v) {}
            bool isReadOnly() const override { return false; }
            double getCurrentValue() const override { return box.param.convertFrom0to1 (box.param.getValue()); }
            void setValue (double v) override { box.attachment.setValueAsCompleteGesture ((float) v); }
            AccessibleValueRange getRange() const override
            {
                const auto& r = box.param.getNormalisableRange();
                return { { r.start, r.end }, r.interval > 0 ? r.interval : 0.01 };
            }
            ValueBox& box;
        };
        return std::make_unique<juce::AccessibilityHandler> (*this, juce::AccessibilityRole::slider, juce::AccessibilityActions(),
                                                              juce::AccessibilityHandler::Interfaces { std::make_unique<Value> (*this) });
    }

private:
    void setNorm (float n, bool partOfGesture)
    {
        const float v = param.convertFrom0to1 (juce::jlimit (0.0f, 1.0f, n));
        if (partOfGesture) attachment.setValueAsPartOfGesture (v); else attachment.setValueAsCompleteGesture (v);
    }
    Theme& theme;
    juce::RangedAudioParameter& param;
    juce::String labelText;
    std::function<juce::String (float)> fmt;
    bool stacked;
    juce::ParameterAttachment attachment;
    float y0 = 0, v0 = 0;
    bool dragging = false, focused = false;
    juce::uint32 lastWheel = 0;
};

// ── Boutons (ibtn, Reset, On/Bypass, A/B/C) ──────────────────────────────
class PillButton : public juce::Button
{
public:
    enum class Style { ibtn, rst, bp, snap };
    PillButton (Theme& t, Style s, juce::String label) : juce::Button (label), theme (t), style (s) {}

    bool on = false;          // ibtn « actif », bp « On », snap « courant »
    bool has = false;         // snap : contient un réglage
    bool armed = false;       // snap : prêt à enregistrer
    bool iconOnly = false;    // bouton carré avec une icône centrée
    bool raised = false;      // posé sur le panneau flottant
    std::function<void (juce::Graphics&, juce::Rectangle<float>, juce::Colour)> icon;

    void paintButton (juce::Graphics& g, bool over, bool down) override
    {
        const auto& p = theme.pal();
        auto b = getLocalBounds().toFloat().reduced (0.5f);
        juce::Colour bg = p.ctl (raised), fg = p.text, br = juce::Colours::transparentBlack;
        float rad = 8;
        juce::String txt = getButtonText();
        if (style == Style::rst) fg = p.dim;
        if (style == Style::ibtn && on) { bg = p.accent; fg = juce::Colours::white; }
        if (style == Style::bp)
        {
            rad = b.getHeight() / 2;
            if (on) { bg = p.green; br = p.green; fg = juce::Colours::white; txt = "On"; }
            else { bg = p.bg3; br = p.border; fg = p.muted; txt = "Bypass"; }
        }
        if (style == Style::snap)
        {
            rad = 7; bg = juce::Colours::transparentBlack; fg = p.muted;
            if (has) { bg = p.bg3; fg = p.text; }
            if (on) { bg = p.accent; fg = juce::Colours::white; }
        }
        if ((over || down) && style != Style::bp && ! (style == Style::ibtn && on) && ! (style == Style::snap && on)) bg = p.ctlHover (raised);
        g.setColour (bg); g.fillRoundedRectangle (b, rad);
        if (style == Style::snap && ! on)
        {
            juce::Path outline; outline.addRoundedRectangle (b, rad);
            if (has) { g.setColour (p.border); g.strokePath (outline, juce::PathStrokeType (1)); }
            else
            {
                juce::Path dashed; const float d[] = { 3, 3 };
                juce::PathStrokeType (1).createDashedStroke (dashed, outline, d, 2);
                g.setColour (p.dim.withAlpha (0.45f)); g.fillPath (dashed);
            }
        }
        else if (! br.isTransparent()) { g.setColour (br); g.drawRoundedRectangle (b, rad, 1); }

        if (style == Style::snap && armed)
        {
            g.setColour (p.red); g.fillEllipse (b.getCentreX() - 4, b.getCentreY() - 4, 8, 8);
            return;
        }
        if (iconOnly && icon) { icon (g, b.withSizeKeepingCentre (16, 16), fg); return; }
        auto tb = b;
        if (icon) { icon (g, tb.removeFromLeft (b.getHeight()).reduced (5).translated (4, 0), fg); }
        g.setColour (fg);
        g.setFont (style == Style::bp ? theme.ui (11, 600) : style == Style::snap ? theme.ui (11, 600) : theme.ui (12, 400));
        g.drawText (txt, tb, juce::Justification::centred, false);
    }

    float idealWidth() const
    {
        if (style == Style::snap) return 26;
        const juce::String t = style == Style::bp ? juce::String ("Bypass") : getButtonText();
        juce::GlyphArrangement ga; ga.addLineOfText (style == Style::bp ? theme.ui (11, 600) : theme.ui (12), t, 0, 0);
        return ga.getBoundingBox (0, -1, true).getWidth() + (style == Style::bp ? 20 : 22) + (icon ? 18 : 0);
    }

private:
    Theme& theme;
    Style style;
};

// ── Menus déroulants (select) ────────────────────────────────────────────
class ALLookAndFeel : public juce::LookAndFeel_V4
{
public:
    explicit ALLookAndFeel (Theme& t) : theme (t) { refresh(); }
    void refresh()
    {
        const auto& p = theme.pal();
        setColour (juce::ComboBox::textColourId, p.text);
        setColour (juce::PopupMenu::backgroundColourId, p.bg2);
        setColour (juce::PopupMenu::textColourId, p.text);
        setColour (juce::PopupMenu::highlightedBackgroundColourId, p.accent);
        setColour (juce::PopupMenu::highlightedTextColourId, juce::Colours::white);
        setColour (juce::TooltipWindow::backgroundColourId, p.bg2);
        setColour (juce::TooltipWindow::textColourId, p.text);
        setColour (juce::TooltipWindow::outlineColourId, p.border);
    }
    void drawComboBox (juce::Graphics& g, int w, int h, bool, int, int, int, int, juce::ComboBox& box) override
    {
        const auto& p = theme.pal();
        auto b = juce::Rectangle<float> (0, 0, (float) w, (float) h);
        const bool raised = (bool) box.getProperties().getWithDefault ("raised", false);
        g.setColour (box.isMouseOver (true) ? p.ctlHover (raised) : p.ctl (raised)); g.fillRoundedRectangle (b, 8);
        if (box.hasKeyboardFocus (true)) { g.setColour (p.accent.withAlpha (0.3f)); g.drawRoundedRectangle (b.reduced (1), 8, 2); }
        juce::Path chev; const float cx = (float) w - 15, cy = h * 0.5f;
        chev.startNewSubPath (cx - 4, cy - 2); chev.lineTo (cx, cy + 2); chev.lineTo (cx + 4, cy - 2);
        g.setColour (p.dim); g.strokePath (chev, juce::PathStrokeType (1.5f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    }
    juce::Font getComboBoxFont (juce::ComboBox&) override { return theme.ui (12); }
    void positionComboBoxText (juce::ComboBox& box, juce::Label& label) override
    {
        label.setBounds (4, 0, box.getWidth() - 24, box.getHeight());
        label.setFont (getComboBoxFont (box));
    }
    juce::Font getPopupMenuFont() override { return theme.ui (13); }
    void drawPopupMenuBackground (juce::Graphics& g, int w, int h) override
    {
        const auto& p = theme.pal();
        g.fillAll (p.bg2);
        g.setColour (p.border); g.drawRect (0, 0, w, h);
    }
    void drawTooltip (juce::Graphics& g, const juce::String& tip, int w, int h) override
    {
        const auto& p = theme.pal();
        g.setColour (p.bg2); g.fillRoundedRectangle (0, 0, (float) w, (float) h, 6);
        g.setColour (p.border); g.drawRoundedRectangle (0.5f, 0.5f, w - 1.0f, h - 1.0f, 6, 1);
        g.setColour (p.text); g.setFont (theme.ui (12));
        g.drawFittedText (tip, juce::Rectangle<int> (0, 0, w, h).reduced (8, 4), juce::Justification::centredLeft, 4);
    }
    juce::Rectangle<int> getTooltipBounds (const juce::String& tip, juce::Point<int> pos, juce::Rectangle<int> parent) override
    {
        juce::GlyphArrangement ga; ga.addFittedText (theme.ui (12), tip, 0, 0, 260, 200, juce::Justification::topLeft, 4);
        const int w = juce::jmin (276, (int) ga.getBoundingBox (0, -1, true).getWidth() + 18);
        const int h = (int) ga.getBoundingBox (0, -1, true).getHeight() + 10;
        return juce::Rectangle<int> (pos.x > parent.getCentreX() ? pos.x - (w + 12) : pos.x + 12, pos.y + 14, w, h).constrainedWithin (parent);
    }

private:
    Theme& theme;
};

// ── Message bref (toast) ─────────────────────────────────────────────────
class Toast : public juce::Component, private juce::Timer
{
public:
    explicit Toast (Theme& t) : theme (t) { setInterceptsMouseClicks (false, false); setAlpha (0); }
    bool atTop = false;
    void show (const juce::String& s, bool ok = false)
    {
        text = s; good = ok; shownAt = juce::Time::getMillisecondCounter();
        setAlpha (1); setVisible (true); toFront (false); repaint(); startTimerHz (30);
    }
    void paint (juce::Graphics& g) override
    {
        if (text.isEmpty()) return;
        const auto& p = theme.pal();
        juce::GlyphArrangement ga; ga.addLineOfText (theme.ui (13, 600), text, 0, 0);
        const float w = ga.getBoundingBox (0, -1, true).getWidth() + 36, h = 34;
        auto r = juce::Rectangle<float> ((getWidth() - w) / 2, atTop ? 14.0f : (float) getHeight() - h - 14, w, h);
        juce::Path box; box.addRoundedRectangle (r, 10);
        softShadow (g, box, juce::Colours::black.withAlpha (0.25f), 18, { 0, 6 });
        g.setColour (p.dark ? p.bg3 : p.text); g.fillPath (box);
        g.setColour (good ? p.green : (p.dark ? p.text : p.bg2));
        g.drawText (text, r, juce::Justification::centred, false);
    }

private:
    void timerCallback() override
    {
        const auto el = juce::Time::getMillisecondCounter() - shownAt;
        if (el > 2200) { setAlpha (0); stopTimer(); text.clear(); return; }
        setAlpha (el > 1800 ? 1.0f - (el - 1800) / 400.0f : 1.0f);
    }
    Theme& theme;
    juce::String text;
    bool good = false;
    juce::uint32 shownAt = 0;
};
} // namespace audiolab
