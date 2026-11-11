// Lab EQ 3.0 — bandeau fixe de la bande sélectionnée, sous la courbe et le clavier (1072 × 104).
// Une bande : forme, pente, placement, active / écouter / supprimer, fréquence, gain, Q,
// dynamique (seuil, plage, attaque, retour). Plusieurs bandes : placement, active, supprimer.
#pragma once
#include "PluginProcessor.h"
#include "Widgets.h"
#include "EqDisplay.h"

namespace audiolab
{
namespace icons
{
inline void power (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    r = r.withSizeKeepingCentre (13, 13);
    juce::Path p;
    p.addCentredArc (r.getCentreX(), r.getCentreY() + 0.6f, 4.8f, 4.8f, 0, 0.72f, juce::MathConstants<float>::twoPi - 0.72f, true);
    g.setColour (c);
    g.strokePath (p, juce::PathStrokeType (1.5f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    g.drawLine (r.getCentreX(), r.getY() + 0.5f, r.getCentreX(), r.getCentreY() - 0.5f, 1.5f);
}

inline void headphones (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    r = r.withSizeKeepingCentre (14, 13);
    juce::Path p;
    p.startNewSubPath (r.getX() + 1.5f, r.getY() + 8);
    p.lineTo (r.getX() + 1.5f, r.getY() + 7);
    p.addCentredArc (r.getCentreX(), r.getY() + 7, 5.5f, 5.5f, 0, -juce::MathConstants<float>::halfPi, juce::MathConstants<float>::halfPi);
    p.lineTo (r.getRight() - 1.5f, r.getY() + 8);
    g.setColour (c);
    g.strokePath (p, juce::PathStrokeType (1.4f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    g.fillRoundedRectangle (r.getX() + 1, r.getY() + 7.5f, 3.2f, 5, 1.2f);
    g.fillRoundedRectangle (r.getRight() - 4.2f, r.getY() + 7.5f, 3.2f, 5, 1.2f);
}

inline void cross (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    const auto m = r.getCentre();
    g.setColour (c);
    g.drawLine (m.x - 4, m.y - 4, m.x + 4, m.y + 4, 1.5f);
    g.drawLine (m.x + 4, m.y - 4, m.x - 4, m.y + 4, 1.5f);
}

inline void trash (juce::Graphics& g, juce::Rectangle<float> r, juce::Colour c)
{
    r = r.withSizeKeepingCentre (12, 13);
    juce::Path p;
    p.startNewSubPath (r.getX() + 1, r.getY() + 3); p.lineTo (r.getRight() - 1, r.getY() + 3);
    p.startNewSubPath (r.getX() + 4.5f, r.getY() + 1); p.lineTo (r.getX() + 7.5f, r.getY() + 1);
    p.startNewSubPath (r.getX() + 2.5f, r.getY() + 4.5f); p.lineTo (r.getX() + 3.3f, r.getBottom() - 1);
    p.lineTo (r.getX() + 8.7f, r.getBottom() - 1); p.lineTo (r.getX() + 9.5f, r.getY() + 4.5f);
    g.setColour (c);
    g.strokePath (p, juce::PathStrokeType (1.4f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
}
} // namespace icons

class BandPanel : public juce::Component
{
public:
    static constexpr int panelW = 1072, panelH = 104;

    BandPanel (Theme& t, AudioLabEqProcessor& p, EqDisplay& d)
        : theme (t), proc (p), display (d),
          onBtn (t, PillButton::Style::ibtn, {}), soloBtn (t, PillButton::Style::ibtn, {}),
          delBtn (t, PillButton::Style::rst, {}), dynBtn (t, PillButton::Style::ibtn, juce::String ("Dynamic"))
    {
        namespace P = params;
        setInterceptsMouseClicks (true, true);
        onBtn.icon = icons::power;      onBtn.iconOnly = true;
        soloBtn.icon = icons::headphones; soloBtn.iconOnly = true;
        delBtn.icon = icons::trash;     delBtn.iconOnly = true;
        onBtn.setTooltip (fr ("Turn the band on / off (double-click the handle)"));
        soloBtn.setTooltip (fr ("Listen only to the part of the spectrum this band processes (or Alt + click and hold the handle)"));
        delBtn.setTooltip (fr ("Delete the band (Delete key)"));
        dynBtn.setTooltip (fr ("Dynamic EQ: the band gain follows the level of the signal in its frequency range"));

        for (int i = 0; i < eq::numShapes; ++i) shapeBox.addItem (P::shapeNames()[i], i + 1);
        for (int i = 0; i < eq::numSlopes; ++i) slopeBox.addItem (P::slopeNames()[i], i + 1);
        // 3.0, interface minimale : Brickwall (paramètre bN_brick) et pente libre (bN_slopefree) pour les coupes.
        // TODO interface 3.0 : remplacer ces deux entrées et la fenêtre de saisie par les contrôles définitifs
        // (voir setCutSlope() plus bas : tout passe par fSlope, fSlopeFree et fBrick).
        slopeBox.addSeparator();
        slopeBox.addItem (fr ("Brickwall"), idBrick);
        slopeBox.addItem (fr ("Custom…"), idCustom);
        for (int i = 0; i < eq::numPlaces; ++i) placeBox.addItem (P::placeNames()[i], i + 1);
        shapeBox.setTitle (fr ("Shape")); slopeBox.setTitle (fr ("Slope")); placeBox.setTitle (fr ("Stereo placement"));
        shapeBox.setTooltip (fr ("Filter shape"));
        slopeBox.setTooltip (fr ("Slope of the cuts and shelves"));
        placeBox.setTooltip (fr ("Stereo, left, right, Mid (centre) or Side (sides)"));
        for (auto* c : { &shapeBox, &slopeBox, &placeBox }) c->getProperties().set ("raised", true);

        onBtn.onClick = [this]
        {
            proc.pushUndo();
            const auto& sel = display.getSelection();
            const bool anyOn = std::any_of (sel.begin(), sel.end(), [this] (int b) { return proc.bandValue (b, params::fOn) > 0.5f; });
            for (int b : sel) proc.setBandValue (b, params::fOn, anyOn ? 0.0f : 1.0f);
            display.refresh(); sync();
        };
        soloBtn.onClick = [this]
        {
            if (band < 0) return;
            proc.setSolo (proc.getSolo() == band ? -1 : band);
            display.refresh(); sync();
        };
        delBtn.onClick = [this] { display.deleteSelected(); };
        dynBtn.onClick = [this]
        {
            if (band < 0) return;
            proc.pushUndo();
            proc.setBandValue (band, params::fDyn, proc.bandValue (band, params::fDyn) > 0.5f ? 0.0f : 1.0f);
            display.refresh(); sync();
        };
        // source du détecteur dynamique (2.0) : signal de la bande ou side-chain externe
        scBox.addItem (fr ("Internal"), 1);
        scBox.addItem (fr ("External"), 2);
        scBox.setTitle (fr ("Side-chain"));
        scBox.getProperties().set ("raised", true);
        scBox.onChange = [this]
        {
            const int v = scBox.getSelectedId() - 1;
            if (v < 0 || band < 0 || juce::roundToInt (proc.bandValue (band, params::fSc)) == v) return;
            proc.pushUndo();
            proc.setBandValue (band, params::fSc, (float) v);
            sync();
        };
        shapeBox.onChange = [this] { applyChoice (shapeBox, params::fShape); };
        slopeBox.onChange = [this]
        {
            const int id = slopeBox.getSelectedId();
            if (id == idCustom) askCustomSlope();
            else if (id == idBrick) setCutSlope (-1, 0.0f, true);
            else if (id >= 1 && id <= eq::numSlopes) setCutSlope (id - 1, 0.0f, false);
        };
        placeBox.onChange = [this] { applyChoice (placeBox, params::fPlace); };

        for (juce::Component* c : { (juce::Component*) &onBtn, (juce::Component*) &soloBtn, (juce::Component*) &delBtn,
                                    (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox,
                                    (juce::Component*) &placeBox, (juce::Component*) &scBox })
            addChildComponent (c);
        setSize (panelW, panelH);
    }

    void testEditFrequency() { if (kFreq != nullptr) kFreq->showEditor(); }   // tests

    int getBand() const { return band; }
    int getCount() const { return count; }

    // Reconstruit le panneau d'après la sélection de la courbe
    void rebuild()
    {
        namespace P = params;
        const auto& sel = display.getSelection();
        count = sel.size();
        band = count >= 1 ? sel[0] : -1;
        kFreq.reset(); kGain.reset(); kQ.reset(); kThr.reset(); kRange.reset(); kAtt.reset(); kRel.reset();

        const bool single = count == 1, any = count >= 1;
        for (auto* c : { (juce::Component*) &onBtn, (juce::Component*) &delBtn, (juce::Component*) &placeBox }) c->setVisible (any);
        for (auto* c : { (juce::Component*) &soloBtn, (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox, (juce::Component*) &scBox })
            c->setVisible (single);

        if (single)
        {
            const int b = band;
            using K = ArcKnob;
            const auto M = K::Size::main, S = K::Size::small;   // attaque et retour : petits (réglages secondaires)
            // teal : fréquence, gain, Q, seuil (forme de la courbe, niveau) ; orange : plage, attaque, retour (action, temps)
            kFreq = std::make_unique<K> (theme, *proc.bandParam (b, P::fFreq), fr ("Frequency"), "Hz", Fam::teal, M,
                [] (float v) { return v < 1000.0f ? juce::String (juce::roundToInt (v)) : juce::String (v / 1000.0f, 2); }, 20.0f, fr ("Centre or cut-off frequency"));
            // saisie au clavier : « 2k », « 440 », ou une note (« A4 », « C#3 », « Bb2 »)
            kFreq->parseFn = [] (const juce::String& txt, float& out)
            {
                const auto t = txt.trim().toUpperCase();
                if (t.length() < 2 || ! juce::String ("CDEFGAB").containsChar (t[0])) return false;
                static const int semis[] = { 9, 11, 0, 2, 4, 5, 7 };   // A B C D E F G
                int n = semis[t[0] - 'A'], i = 1;
                if (t[i] == '#') { ++n; ++i; } else if (t[i] == 'B' && t.length() > 2) { --n; ++i; }
                const auto oct = t.substring (i);
                if (oct.isEmpty() || ! oct.containsOnly ("-0123456789")) return false;
                const int midi = (oct.getIntValue() + 1) * 12 + n;
                out = (float) (440.0 * std::pow (2.0, (midi - 69) / 12.0));
                return true;
            };
            kFreq->editFmt = [] (float v) { return juce::String (juce::roundToInt (v)); };   // toujours en Hz
            kFreq->unitFmt = [] (float v) { return juce::String (v < 1000.0f ? "Hz" : "kHz"); };
            kGain = std::make_unique<K> (theme, *proc.bandParam (b, P::fGain), "Gain", "dB", Fam::teal, M,
                [] (float v) { return P::fmtDb (v); }, 0.0f, fr ("Band gain (bells, shelves, tilt)"));
            kQ = std::make_unique<K> (theme, *proc.bandParam (b, P::fQ), "Q", "Q", Fam::teal, M,
                [] (float v) { return P::fmtQ (v); }, 0.025f, fr ("Bandwidth: high Q = narrow (mouse wheel on the handle)"));
            kThr = std::make_unique<K> (theme, *proc.bandParam (b, P::fThr), fr ("Threshold"), "dB", Fam::teal, M,
                [] (float v) { return juce::String (v, 1); }, 0.0f, fr ("Level above which the dynamic band acts"));
            kRange = std::make_unique<K> (theme, *proc.bandParam (b, P::fRange), "Range", "dB", Fam::orange, M,
                [] (float v) { return P::fmtDb (v); }, 0.0f, fr ("Maximum gain added (+) or removed (−) when the signal goes above the threshold (Alt + mouse wheel on the handle)"));
            kAtt = std::make_unique<K> (theme, *proc.bandParam (b, P::fAtt), fr ("Attack"), "ms", Fam::orange, S,
                [] (float v) { return P::fmtMs (v); }, 0.5f, fr ("How fast this band's dynamics react (Alt-click: 10 ms)"));
            kRel = std::make_unique<K> (theme, *proc.bandParam (b, P::fRel), fr ("Release"), "ms", Fam::orange, S,
                [] (float v) { return P::fmtMs (v); }, 10.0f, fr ("Release time of this band's dynamics (Alt-click: 150 ms)"));
            for (auto* k : { kFreq.get(), kGain.get(), kQ.get(), kThr.get(), kRange.get(), kAtt.get(), kRel.get() })
            {
                k->onGestureStart = [this] { proc.pushUndo(); };
                addAndMakeVisible (k);
            }
            // 3.0, interface minimale : clic droit sur Threshold = menu « Auto threshold » (paramètre bN_thrauto).
            // TODO interface 3.0 : petit bouton « Auto » à côté du potentiomètre.
            kThr->addMouseListener (this, false);
            title = fr ("Band ") + juce::String (b + 1);
        }
        else if (any) title = juce::String (count) + fr (" bands selected");
        else title = fr ("No band selected");

        setSize (panelW, panelH);
        layout();
        sync();
        repaint();
    }

    // Suit les valeurs (automation, molette, menus…)
    void sync()
    {
        namespace P = params;
        if (band < 0 || count == 0) return;
        const int b = band;
        const auto shape = (eq::Shape) juce::roundToInt (proc.bandValue (b, P::fShape));
        const int slopeIdx = juce::roundToInt (proc.bandValue (b, P::fSlope));
        const auto& sel = display.getSelection();
        const bool anyOn = std::any_of (sel.begin(), sel.end(), [this] (int x) { return proc.bandValue (x, P::fOn) > 0.5f; });
        setOn (onBtn, anyOn);
        setOn (soloBtn, proc.getSolo() == b);
        const bool dyn = proc.bandValue (b, P::fDyn) > 0.5f;
        setOn (dynBtn, dyn);
        shapeBox.setSelectedId ((int) shape + 1, juce::dontSendNotification);
        // 3.0 : Brickwall et pente libre des coupes (la bande telle que le moteur la voit)
        const auto spec = proc.getBandSpec (b, false);
        const bool cut = eq::isCut (shape);
        slopeBox.setItemEnabled (idBrick, cut);
        slopeBox.setItemEnabled (idCustom, cut);
        if (eq::isBrick (spec)) slopeBox.setSelectedId (idBrick, juce::dontSendNotification);
        else if (cut && spec.slopeFree > 0.0) slopeBox.setText (juce::String (eq::cutSlope (spec), 1) + " dB/oct", juce::dontSendNotification);
        else slopeBox.setSelectedId (slopeIdx + 1, juce::dontSendNotification);
        placeBox.setSelectedId (juce::roundToInt (proc.bandValue (b, P::fPlace)) + 1, juce::dontSendNotification);
        dim (&slopeBox, eq::usesSlope (shape), 0.4f);
        dim (&dynBtn, eq::canBeDynamic (shape), 0.4f);
        dim (kGain.get(), eq::hasGain (shape));
        dim (kQ.get(), eq::usesQ (spec));
        // un passe-tout ne change que la phase : il n'a aucun effet en Linear phase
        const auto shapeTip = shape == eq::Shape::allPass && proc.phaseMode() == P::phaseLinear
                                  ? fr ("All pass only rotates the phase: it has no effect in Linear phase mode")
                            : shape == eq::Shape::allPass ? fr ("All pass: flat level, phase rotation only (slope 6 dB/oct = 1st order, otherwise 2nd order with Q)")
                            : shape == eq::Shape::flatTilt ? fr ("Flat tilt: constant slope across the whole spectrum; Gain = level difference between 20 kHz and 20 Hz")
                                                           : fr ("Filter shape");
        if (shapeBox.getTooltip() != shapeTip) shapeBox.setTooltip (shapeTip);
        const bool dynOn = dyn && eq::canBeDynamic (shape);
        for (juce::Component* c : { (juce::Component*) kThr.get(), (juce::Component*) kRange.get(), (juce::Component*) kAtt.get(), (juce::Component*) kRel.get() })
            dim (c, dynOn);
        // seuil automatique : le potentiomètre Threshold n'agit plus (grisé), le seuil suit le niveau moyen de la bande
        const bool thrAuto = proc.bandValue (b, P::fThrAuto) > 0.5f;
        if (kThr != nullptr)
        {
            if (dynOn && thrAuto) kThr->setAlpha (0.35f);
            const auto thrTip = thrAuto ? fr ("Auto threshold is on: the threshold follows the average level of the band (right-click to turn it off)")
                                        : fr ("Level above which the dynamic band acts (right-click: Auto threshold)");
            if (kThr->getTooltip() != thrTip) kThr->setTooltip (thrTip);
        }
        const bool ext = proc.bandValue (b, P::fSc) > 0.5f;
        scBox.setSelectedId (ext ? 2 : 1, juce::dontSendNotification);
        dim (&scBox, dynOn, 0.4f);
        const auto tip = ! ext ? fr ("Side-chain: the dynamic band follows its own signal (Internal) or the plugin's side-chain input (External)")
                       : proc.sideChainConnected() ? fr ("External: the dynamic band follows the side-chain input")
                                                   : fr ("External selected, but no side-chain input is connected: the band follows its own signal");
        if (scBox.getTooltip() != tip) scBox.setTooltip (tip);
    }

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        g.setColour (p.surface); g.fillRoundedRectangle (getLocalBounds().toFloat(), 12);

        float x = 16;
        if (count == 1)
        {
            // pastille numérotée de la bande choisie (24 px, comme la poignée)
            g.setColour (p.select); g.fillEllipse (x, 20, 24, 24);
            g.setColour (p.text); g.setFont (theme.ui (12, 600));
            g.drawText (juce::String (band + 1), juce::Rectangle<float> (x, 20, 24, 24), juce::Justification::centred, false);
            x += 32;
        }
        g.setColour (count == 0 ? p.text2 : p.text); g.setFont (theme.ui (16, 600));
        g.drawText (title, juce::Rectangle<float> (x, 16, count == 1 ? 96.0f : 320.0f, 32), juce::Justification::centredLeft, false);
        g.setColour (p.rail);
        if (count == 1) { g.fillRect (sep1, 16.0f, 1.0f, 72.0f); g.fillRect (sep2, 16.0f, 1.0f, 72.0f); }
        else
        {
            g.setColour (p.text2); g.setFont (theme.ui (12));
            g.drawText (count == 0 ? fr ("Click a handle to edit a band · Click the curve to add one · Drag on the background to select several")
                                   : fr ("Drag a handle to move the whole selection · Shift: fine adjustment"),
                        juce::Rectangle<float> (16, 56, (float) getWidth() - 32, 32), juce::Justification::centredLeft, false);
        }
    }

    // 3.0, interface minimale : clic droit sur le potentiomètre Threshold
    void mouseDown (const juce::MouseEvent& e) override
    {
        if (kThr == nullptr || band < 0 || ! e.mods.isPopupMenu()) return;
        if (e.eventComponent != kThr.get() && ! kThr->isParentOf (e.eventComponent)) return;
        const bool on = proc.bandValue (band, params::fThrAuto) > 0.5f;
        juce::PopupMenu m;
        m.addItem (fr ("Auto threshold"), true, on, [safe = juce::Component::SafePointer<BandPanel> (this), on]
        {
            if (safe == nullptr || safe->band < 0) return;
            safe->proc.pushUndo();
            safe->proc.setBandValue (safe->band, params::fThrAuto, on ? 0.0f : 1.0f);
            safe->display.refresh(); safe->sync();
        });
        m.showMenuAsync (juce::PopupMenu::Options().withTargetComponent (kThr.get()));
    }

private:
    static constexpr int idBrick = 101, idCustom = 102;

    // Pente d'une coupe pour la sélection : entrée de la liste (index ≥ 0), pente libre en dB/oct (> 0) ou Brickwall
    void setCutSlope (int listIndex, float freeDbPerOct, bool brickwall)
    {
        if (band < 0) return;
        namespace P = params;
        proc.pushUndo();
        for (int b : display.getSelection())
        {
            if (listIndex >= 0) proc.setBandValue (b, P::fSlope, (float) listIndex);
            proc.setBandValue (b, P::fSlopeFree, freeDbPerOct);
            proc.setBandValue (b, P::fBrick, brickwall ? 1.0f : 0.0f);
        }
        display.refresh(); sync();
    }

    // Saisie de la pente libre (6 à 96 dB/oct)
    void askCustomSlope()
    {
        if (band < 0) return;
        const auto spec = proc.getBandSpec (band, false);
        auto* w = new juce::AlertWindow (fr ("Custom slope"), fr ("Slope of the cut, from 6 to 96 dB/oct"), juce::MessageBoxIconType::NoIcon, this);
        w->addTextEditor ("slope", juce::String (eq::cutSlope (spec), 1));
        w->addButton ("OK", 1, juce::KeyPress (juce::KeyPress::returnKey));
        w->addButton (fr ("Cancel"), 0, juce::KeyPress (juce::KeyPress::escapeKey));
        w->enterModalState (true, juce::ModalCallbackFunction::create ([safe = juce::Component::SafePointer<BandPanel> (this), w] (int result)
        {
            if (safe == nullptr) return;
            if (result == 1)
            {
                const float v = w->getTextEditorContents ("slope").replaceCharacter (',', '.').getFloatValue();
                if (v > 0.0f) safe->setCutSlope (-1, juce::jlimit (6.0f, 96.0f, v), false);
            }
            safe->sync();
        }), true);
    }

    void layout()
    {
        // grille 3.0 : marge intérieure 16, deux rangées de 32 (y = 16 et 56), écarts de 8 ;
        // identité | fréquence, gain, Q (potentiomètres de 64 au pas de 80) | dynamique
        const int y1 = 16, y2 = 56, h = 32;
        if (count == 1)
        {
            onBtn.setBounds (16, y2, 32, h);
            soloBtn.setBounds (56, y2, 32, h);
            delBtn.setBounds (96, y2, 32, h);
            shapeBox.setBounds (152, y1, 104, h);
            slopeBox.setBounds (264, y1, 104, h);
            placeBox.setBounds (152, y2, 104, h);
            int cx = 440;
            for (auto* k : { kFreq.get(), kGain.get(), kQ.get() }) { k->setBounds (cx - 40, 12, 80, 88); cx += 80; }
            dynBtn.setBounds (672, y1, 88, h);
            scBox.setBounds (672, y2, 88, h);
            kThr->setBounds (816 - 40, 12, 80, 88);
            kRange->setBounds (896 - 40, 12, 80, 88);
            kAtt->setBounds (960, y1, 96, h);
            kRel->setBounds (960, y2, 96, h);
        }
        else if (count > 1)
        {
            delBtn.setBounds (getWidth() - 16 - 32, y1, 32, h);
            onBtn.setBounds (delBtn.getX() - 8 - 32, y1, 32, h);
            placeBox.setBounds (onBtn.getX() - 16 - 104, y1, 104, h);
        }
    }

    void applyChoice (juce::ComboBox& box, const char* field)
    {
        const int v = box.getSelectedId() - 1;
        if (v < 0 || band < 0) return;
        const auto& sel = display.getSelection();
        if (std::all_of (sel.begin(), sel.end(), [&] (int b) { return juce::roundToInt (proc.bandValue (b, field)) == v; })) return;
        proc.pushUndo();
        for (int b : sel) proc.setBandValue (b, field, (float) v);
        display.refresh(); sync();
    }

    static void setOn (PillButton& b, bool on) { if (b.on != on) { b.on = on; b.repaint(); } }
    static void dim (juce::Component* c, bool en, float off = 0.35f)
    {
        if (c == nullptr) return;
        c->setEnabled (en);
        c->setAlpha (en ? 1.0f : off);
    }

    Theme& theme;
    AudioLabEqProcessor& proc;
    EqDisplay& display;
    int band = -1, count = 0;
    juce::String title;
    static constexpr float sep1 = 384.0f, sep2 = 656.0f;
    PillButton onBtn, soloBtn, delBtn, dynBtn;
    juce::ComboBox shapeBox, slopeBox, placeBox, scBox;
    std::unique_ptr<ArcKnob> kFreq, kGain, kQ, kThr, kRange, kAtt, kRel;
};
} // namespace audiolab
