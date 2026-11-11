// Composants d'interface des plugins Audio Lab, selon la charte V2 :
// potentiomètres à arc (principal 72 px, petit 28 px), sélecteur segmenté, boutons, menus, toast.
#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "Theme.h"

namespace audiolab
{
// Majuscules, y compris les lettres accentuées du français
inline juce::String upper (const juce::String& s)
{
    juce::String r = s.toUpperCase();
    const char* pairs[][2] = { { "é", "É" }, { "è", "È" }, { "ê", "Ê" }, { "à", "À" }, { "â", "Â" }, { "ç", "Ç" }, { "ô", "Ô" }, { "û", "Û" }, { "î", "Î" } };
    for (auto& pr : pairs) r = r.replace (juce::String::fromUTF8 (pr[0]), juce::String::fromUTF8 (pr[1]));
    return r;
}

inline juce::Point<float> onCircle (juce::Point<float> c, float r, float a) { return { c.x + r * std::sin (a), c.y - r * std::cos (a) }; }

// Arc de 270° (7 h 30 → 4 h 30) : rail, partie colorée du départ à la valeur (dégradé en ligne droite,
// bout mobile coupé droit, pastille de départ), halo léger, capuchon avec liseré et ombres, repère.
inline void drawArcControl (juce::Graphics& g, const Theme& th, juce::Point<float> c, float r, float sw, float capD,
                            float startN, float valueN, Fam fam, bool hover, bool active)
{
    const auto& p = th.pal();
    const float A0 = -0.75f * juce::MathConstants<float>::pi, A1 = -A0;
    auto ang = [&] (float t) { return A0 + (A1 - A0) * juce::jlimit (0.0f, 1.0f, t); };
    const auto cols = th.family (fam, true);

    juce::Path rail; rail.addCentredArc (c.x, c.y, r, r, 0, A0, A1, true);
    g.setColour (hover ? p.rail.brighter (0.25f) : p.rail);
    g.strokePath (rail, juce::PathStrokeType (sw, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));

    if (std::abs (valueN - startN) > 0.004f)
    {
        const float a = ang (std::min (startN, valueN)), b = ang (std::max (startN, valueN));
        juce::Path arc; arc.addCentredArc (c.x, c.y, r, r, 0, a, b, true);
        const auto ps = onCircle (c, r, ang (startN)), pv = onCircle (c, r, ang (valueN));
        juce::ColourGradient grad (cols.first, ps, cols.second, pv, false);
        // halo fin et discret (2 voiles : +1,6 et +3,2 px), à peine renforcé pendant le réglage
        const float k = active ? 1.5f : 1.0f;
        g.setGradientFill (grad); g.setOpacity (0.05f * k);
        g.strokePath (arc, juce::PathStrokeType (sw + 3.2f, juce::PathStrokeType::curved, juce::PathStrokeType::butt));
        g.setOpacity (0.12f * k);
        g.strokePath (arc, juce::PathStrokeType (sw + 1.6f, juce::PathStrokeType::curved, juce::PathStrokeType::butt));
        g.setOpacity (1.0f);
        g.setGradientFill (grad);
        g.strokePath (arc, juce::PathStrokeType (sw, juce::PathStrokeType::curved, juce::PathStrokeType::butt));
        g.setColour (cols.first); g.fillEllipse (ps.x - 1.6f, ps.y - 1.6f, 3.2f, 3.2f);
    }

    // capuchon
    juce::Path cap; cap.addEllipse (c.x - capD / 2, c.y - capD / 2, capD, capD);
    juce::DropShadow (juce::Colours::black.withAlpha (0.5f), capD > 30 ? 20 : 8, { 0, capD > 30 ? 8 : 3 }).drawForPath (g, cap);
    juce::DropShadow (juce::Colours::black.withAlpha (0.45f), 2, { 0, 1 }).drawForPath (g, cap);
    g.setColour (p.cap); g.fillPath (cap);
    juce::Path lip; lip.addCentredArc (c.x, c.y, capD / 2 - 0.5f, capD / 2 - 0.5f, 0, -1.1f, 1.1f, true);
    g.setColour (juce::Colours::white.withAlpha (0.07f)); g.strokePath (lip, juce::PathStrokeType (1.0f));
    // repère
    const float av = ang (valueN);
    const auto i0 = onCircle (c, capD / 2 - (capD > 30 ? 5.5f : 4.5f), av), i1 = onCircle (c, capD / 2 - 1.5f, av);
    g.setColour (cols.second); g.drawLine (i0.x, i0.y, i1.x, i1.y, 1.5f);
}

// ── Potentiomètre (principal 72 px, ou petit 28 px avec nom et valeur à côté) ──
class ArcKnob : public juce::Component, public juce::SettableTooltipClient
{
public:
    enum class Size { main, small };

    ArcKnob (Theme& t, juce::RangedAudioParameter& prm, juce::String label, juce::String unit, Fam family, Size size,
             std::function<juce::String (float)> format, float startValue, juce::String help = {})
        : theme (t), param (prm), labelText (std::move (label)), unitText (std::move (unit)), fam (family), sz (size),
          fmt (std::move (format)), startNorm (prm.convertTo0to1 (startValue)),
          attachment (prm, [this] (float) { repaint(); }, nullptr)
    {
        setWantsKeyboardFocus (true);
        setTitle (labelText);
        setTooltip (help);
        setMouseCursor (juce::MouseCursor::UpDownResizeCursor);
        setRepaintsOnMouseActivity (true);
        attachment.sendInitialUpdate();
    }

    std::function<void()> onGestureStart;   // pour « Annuler »
    std::function<juce::String (float)> unitFmt;   // unité qui dépend de la valeur (Hz / kHz)
    bool labelLeft = false;                 // petit potentiomètre : nom et valeur à gauche, calés à droite

    // Largeur conseillée (petit potentiomètre : 28 px + nom / valeur)
    int idealWidth() const
    {
        if (sz == Size::main) return 80;
        const float a = juce::GlyphArrangement::getStringWidth (theme.ui (10, 600, 0.04f), upper (labelText));
        const float b = juce::GlyphArrangement::getStringWidth (theme.ui (10), fmt (param.convertFrom0to1 (param.getValue())) + "0");
        return 32 + 8 + juce::roundToInt (std::max (a, b)) + 4;
    }

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        const float n = param.getValue();
        const bool hover = isMouseOver (true) && isEnabled();
        const auto value = fmt (param.convertFrom0to1 (n));
        if (sz == Size::main)
        {
            // grille 3.0 : arc de 64 hors tout (rayon d'axe 30,5, trait 3), capuchon 46 (5/7 de l'arc),
            // valeur 12 px remontée de 1 px, unité 10 px, nom 10 px en capitales 52 px sous l'axe
            const juce::Point<float> c ((float) getWidth() * 0.5f, 32.0f);
            drawArcControl (g, theme, c, 30.5f, 3.0f, 46.0f, startNorm, n, fam, hover, dragging);
            g.setColour (p.text);
            theme.drawValue (g, value, { c.x - 23, c.y - 12, 46, 14 }, 12, 600);
            g.setColour (p.soft); g.setFont (theme.ui (10));
            g.drawText (unitFmt ? unitFmt (param.convertFrom0to1 (n)) : unitText, juce::Rectangle<float> (c.x - 23, c.y + 3, 46, 12), juce::Justification::centred, false);
            g.setColour (p.text); g.setFont (theme.ui (10, 600, 0.06f));
            g.drawText (upper (labelText), juce::Rectangle<float> (0.4f, c.y + 40, (float) getWidth(), 16), juce::Justification::centred, false);
        }
        else
        {
            // petit potentiomètre : 32 hors tout (rayon d'axe 14,5, trait 3), capuchon 20
            const float w = (float) getWidth();
            const juce::Point<float> c (labelLeft ? w - 16.0f : 16.0f, (float) getHeight() * 0.5f);
            drawArcControl (g, theme, c, 14.5f, 3.0f, 20.0f, startNorm, n, fam, hover, dragging);
            const juce::Rectangle<float> tx (labelLeft ? 0.0f : 40.0f, c.y - 13, w - 40, 12);
            g.setColour (p.text); g.setFont (theme.ui (10, 600, 0.04f));
            g.drawText (upper (labelText), tx, labelLeft ? juce::Justification::bottomRight : juce::Justification::bottomLeft, false);
            g.setColour (p.text2);
            theme.drawValue (g, value, tx.translated (0, 13.5f).withHeight (13), 10, 400, labelLeft ? juce::Justification::right : juce::Justification::left);
        }
        if (focused) { g.setColour (p.teal2.withAlpha (0.5f)); g.drawRoundedRectangle (getLocalBounds().toFloat().reduced (1), 8, 1.5f); }
    }

    void mouseDown (const juce::MouseEvent& e) override
    {
        if (! isEnabled() || e.mods.isPopupMenu()) return;
        if (e.mods.isAltDown() || e.mods.isCommandDown()) { resetToDefault(); return; }   // Alt ou Cmd + clic : valeur par défaut
        if (onGestureStart) onGestureStart();
        y0 = e.position.y; v0 = param.getValue();
        attachment.beginGesture(); dragging = true; repaint();
    }
    void mouseDrag (const juce::MouseEvent& e) override
    {
        if (! dragging) return;
        const float k = e.mods.isShiftDown() ? 1200.0f : 220.0f;   // Maj = réglage fin
        setNorm (v0 + (y0 - e.position.y) / k, true);
    }
    void mouseUp (const juce::MouseEvent&) override { if (dragging) { attachment.endGesture(); dragging = false; repaint(); } }
    // Double-clic (ou Entrée quand le potentiomètre a le focus) : saisie de la valeur au clavier.
    void mouseDoubleClick (const juce::MouseEvent&) override { if (isEnabled()) showEditor(); }
    void resetToDefault()
    {
        if (onGestureStart) onGestureStart();
        attachment.setValueAsCompleteGesture (param.convertFrom0to1 (param.getDefaultValue()));
    }
    // « 2k », « 2.5 kHz », « -3 », « −3 dB », « 1,5 » ; parseFn (facultatif) passe en premier (notes : « A4 »)
    std::function<bool (const juce::String&, float&)> parseFn;
    std::function<juce::String (float)> editFmt;   // texte de départ de la saisie quand l'affichage change d'unité (Hz / kHz)
    static bool parseNumber (juce::String t, float& out)
    {
        t = t.trim().toLowerCase().replace (juce::String::fromUTF8 ("−"), "-").replaceCharacter (',', '.');
        const auto num = t.retainCharacters ("0123456789.-+");
        if (num.isEmpty() || ! num.containsAnyOf ("0123456789")) return false;
        double v = num.getDoubleValue();
        const auto rest = t.removeCharacters ("0123456789.-+ ");
        if (rest.startsWithChar ('k')) v *= 1000.0;
        out = (float) v; return true;
    }
    void showEditor()
    {
        if (dragging) { attachment.endGesture(); dragging = false; }
        editor = std::make_unique<juce::TextEditor>();
        const auto& p = theme.pal();
        editor->setFont (theme.ui (12, 600));
        editor->setJustification (juce::Justification::centred);
        editor->setColour (juce::TextEditor::backgroundColourId, p.win);
        editor->setColour (juce::TextEditor::textColourId, p.text);
        editor->setColour (juce::TextEditor::highlightColourId, p.select);
        editor->setColour (juce::TextEditor::highlightedTextColourId, p.text);
        editor->setColour (juce::TextEditor::outlineColourId, p.teal2);
        editor->setColour (juce::TextEditor::focusedOutlineColourId, p.teal2);
        editor->setIndents (2, 3);
        const float cur = param.convertFrom0to1 (param.getValue());
        editor->setText (editFmt ? editFmt (cur) : fmt (cur), false);
        if (sz == Size::main) editor->setBounds (getWidth() / 2 - 28, 32 - 11, 56, 22);
        else                  editor->setBounds (labelLeft ? 0 : 40, getHeight() / 2 - 11, getWidth() - 40, 22);
        addAndMakeVisible (*editor);
        editor->selectAll();
        editor->grabKeyboardFocus();
        editor->onReturnKey = [this]
        {
            float v = 0;
            const auto t = editor->getText();
            if ((parseFn && parseFn (t, v)) || parseNumber (t, v))
            {
                if (onGestureStart) onGestureStart();
                const auto& r = param.getNormalisableRange();
                attachment.setValueAsCompleteGesture (juce::jlimit (r.start, r.end, v));
            }
            hideEditor();
        };
        editor->onEscapeKey = [this] { hideEditor(); };
        editor->onFocusLost = [this] { hideEditor(); };
    }
    void hideEditor()
    {
        if (editor == nullptr) return;
        juce::MessageManager::callAsync ([safe = juce::Component::SafePointer<ArcKnob> (this)] { if (safe != nullptr) { safe->editor.reset(); safe->repaint(); } });
    }
    std::unique_ptr<juce::TextEditor> editor;
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
        if (k.isKeyCode (juce::KeyPress::returnKey) && isEnabled()) { showEditor(); return true; }
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
        const float v = param.convertFrom0to1 (juce::jlimit (0.0f, 1.0f, n));
        if (partOfGesture) attachment.setValueAsPartOfGesture (v); else attachment.setValueAsCompleteGesture (v);
    }

    Theme& theme;
    juce::RangedAudioParameter& param;
    juce::String labelText, unitText;
    Fam fam;
    Size sz;
    std::function<juce::String (float)> fmt;
    float startNorm;
    juce::ParameterAttachment attachment;
    float y0 = 0, v0 = 0;
    bool dragging = false, focused = false;
    juce::uint32 lastWheel = 0;
};

// ── Sélecteur segmenté ───────────────────────────────────────────────────
class SegButtons : public juce::Component, public juce::SettableTooltipClient
{
public:
    SegButtons (Theme& t, juce::StringArray opts) : theme (t), options (std::move (opts)) {}

    bool onSurface = false;   // posé sur un écran / panneau : bande « Rails » au lieu de « Surface »
    float fontSize = 12.0f;

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
        float w = 0;
        for (auto& o : options) w = std::max (w, juce::GlyphArrangement::getStringWidth (theme.ui (fontSize, 500), o) + 24);
        return 8 + w * options.size();
    }

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        g.setColour (onSurface ? p.rail : p.surface); g.fillRoundedRectangle (getLocalBounds().toFloat(), 8);
        for (int i = 0; i < options.size(); ++i)
        {
            auto r = cell (i);
            if (i == selected)
            {
                juce::Path pill; pill.addRoundedRectangle (r, 4);
                juce::DropShadow (juce::Colours::black.withAlpha (0.18f), 3, { 0, 1 }).drawForPath (g, pill);
                g.setColour (p.select); g.fillPath (pill);
            }
            else if (i == hover) { g.setColour (p.cap.withAlpha (0.6f)); g.fillRoundedRectangle (r, 4); }
            g.setColour (i == selected ? p.text : p.text2);
            g.setFont (theme.ui (fontSize, i == selected ? 500 : 400));
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
    juce::Rectangle<float> cell (int i) const
    {
        // marge intérieure 4 (rayons concentriques 8 et 4), segments jointifs
        const float w = ((float) getWidth() - 8) / (float) options.size();
        return { 4 + i * w, 4.0f, w, (float) getHeight() - 8 };
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

// ── Boutons : ibtn (bascule, choisi = #005F73), rst (action simple) ──────
class PillButton : public juce::Button
{
public:
    enum class Style { ibtn, rst };
    PillButton (Theme& t, Style s, juce::String label) : juce::Button (label), theme (t), style (s) {}

    bool on = false;          // ibtn « actif »
    bool iconOnly = false;    // bouton carré avec une icône centrée
    bool raised = false;      // (compatibilité)
    std::function<void (juce::Graphics&, juce::Rectangle<float>, juce::Colour)> icon;

    void paintButton (juce::Graphics& g, bool over, bool down) override
    {
        const auto& p = theme.pal();
        auto b = getLocalBounds().toFloat();
        const bool sel = style == Style::ibtn && on;
        juce::Colour bg = sel ? p.select : (over || down) ? p.capHover : p.cap;
        juce::Colour fg = sel ? p.text : (iconOnly || style == Style::rst ? p.text2 : p.text);
        g.setColour (bg); g.fillRoundedRectangle (b, 8);
        if (! sel) { g.setColour (juce::Colours::white.withAlpha (0.05f)); g.drawHorizontalLine (1, b.getX() + 6, b.getRight() - 6); }
        if (iconOnly && icon) { icon (g, b.withSizeKeepingCentre (16, 16), fg); return; }
        auto tb = b;
        if (icon) icon (g, tb.removeFromLeft (b.getHeight()).reduced (5).translated (4, 0), fg);
        g.setColour (fg);
        g.setFont (theme.ui (12, sel ? 500 : 400));
        g.drawText (getButtonText(), tb, juce::Justification::centred, false);
    }

    float idealWidth() const
    {
        return juce::GlyphArrangement::getStringWidth (theme.ui (12, 500), getButtonText()) + 24 + (icon ? 18 : 0);
    }

private:
    Theme& theme;
    Style style;
};

// ── Flèche d'ouverture (« Advanced settings ») : fermé = texte secondaire, ouvert = texte ──
// (même composant que le Compresseur)
class Disclosure : public juce::Button
{
public:
    Disclosure (Theme& t, juce::String label) : juce::Button (label), theme (t) {}
    bool open = false;

    void paintButton (juce::Graphics& g, bool over, bool) override
    {
        const auto& p = theme.pal();
        const juce::Colour c = open || over ? p.text : p.text2;
        const float x = 8, y = getHeight() * 0.5f - 6;   // boîte 12 × 12 du chevron
        juce::Path ch;
        if (open) { ch.startNewSubPath (x + 2.5f, y + 4.5f); ch.lineTo (x + 6, y + 8); ch.lineTo (x + 9.5f, y + 4.5f); }
        else      { ch.startNewSubPath (x + 4.5f, y + 2.5f); ch.lineTo (x + 8, y + 6); ch.lineTo (x + 4.5f, y + 9.5f); }
        g.setColour (c);
        g.strokePath (ch, juce::PathStrokeType (1.6f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        g.setFont (theme.ui (12));
        g.drawText (getButtonText(), juce::Rectangle<float> (26, 0, (float) getWidth() - 26, (float) getHeight()), juce::Justification::centredLeft, false);
    }
    float idealWidth() const { return 26 + juce::GlyphArrangement::getStringWidth (theme.ui (12), getButtonText()) + 8; }

private:
    Theme& theme;
};

// ── Menus déroulants, bulles d'aide ──────────────────────────────────────
class ALLookAndFeel : public juce::LookAndFeel_V4
{
public:
    explicit ALLookAndFeel (Theme& t) : theme (t) { refresh(); }
    void refresh()
    {
        const auto& p = theme.pal();
        setColour (juce::ComboBox::textColourId, p.text);
        setColour (juce::PopupMenu::backgroundColourId, p.surface);
        setColour (juce::PopupMenu::textColourId, p.text);
        setColour (juce::PopupMenu::headerTextColourId, p.soft);
        setColour (juce::PopupMenu::highlightedBackgroundColourId, p.select);
        setColour (juce::PopupMenu::highlightedTextColourId, p.text);
        setColour (juce::TooltipWindow::backgroundColourId, p.cap);
        setColour (juce::TooltipWindow::textColourId, p.text);
        setColour (juce::TooltipWindow::outlineColourId, p.rail);
    }
    void drawComboBox (juce::Graphics& g, int w, int h, bool, int, int, int, int, juce::ComboBox& box) override
    {
        const auto& p = theme.pal();
        auto b = juce::Rectangle<float> (0, 0, (float) w, (float) h);
        g.setColour (box.isMouseOver (true) && box.isEnabled() ? p.capHover : p.cap); g.fillRoundedRectangle (b, 8);
        g.setColour (juce::Colours::white.withAlpha (0.05f)); g.drawHorizontalLine (1, 6.0f, (float) w - 6);
        if (box.hasKeyboardFocus (true)) { g.setColour (p.teal2.withAlpha (0.5f)); g.drawRoundedRectangle (b.reduced (1), 8, 1.5f); }
        juce::Path chev; const float cx = (float) w - 15, cy = h * 0.5f;
        chev.startNewSubPath (cx - 4, cy - 2); chev.lineTo (cx, cy + 2); chev.lineTo (cx + 4, cy - 2);
        g.setColour (p.soft); g.strokePath (chev, juce::PathStrokeType (1.5f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    }
    juce::Font getComboBoxFont (juce::ComboBox&) override { return theme.ui (12); }
    void positionComboBoxText (juce::ComboBox& box, juce::Label& label) override
    {
        label.setBounds (4, 0, box.getWidth() - 24, box.getHeight());
        label.setFont (getComboBoxFont (box));
        label.setColour (juce::Label::textColourId, box.isEnabled() ? theme.pal().text : theme.pal().soft);
    }
    juce::Font getPopupMenuFont() override { return theme.ui (12); }
    void drawPopupMenuBackground (juce::Graphics& g, int w, int h) override
    {
        const auto& p = theme.pal();
        g.fillAll (p.surface);
        g.setColour (p.rail); g.drawRect (0, 0, w, h);
    }
    void drawTooltip (juce::Graphics& g, const juce::String& tip, int w, int h) override
    {
        const auto& p = theme.pal();
        g.setColour (p.cap); g.fillRoundedRectangle (0, 0, (float) w, (float) h, 6);
        g.setColour (p.rail); g.drawRoundedRectangle (0.5f, 0.5f, w - 1.0f, h - 1.0f, 6, 1);
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
        const float w = juce::GlyphArrangement::getStringWidth (theme.ui (12, 500), text) + 36, h = 34;
        auto r = juce::Rectangle<float> ((getWidth() - w) / 2, atTop ? 14.0f : (float) getHeight() - h - 14, w, h);
        juce::Path box; box.addRoundedRectangle (r, 10);
        juce::DropShadow (juce::Colours::black.withAlpha (0.45f), 18, { 0, 6 }).drawForPath (g, box);
        g.setColour (p.cap); g.fillPath (box);
        g.setColour (p.rail); g.strokePath (box, juce::PathStrokeType (1));
        g.setColour (good ? p.teal2 : p.text); g.setFont (theme.ui (12, 500));
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
