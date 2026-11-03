// Audio Lab EQ — panneau flottant de la bande sélectionnée, ancré sous la courbe.
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
    static constexpr int panelW = 580, singleH = 208, multiH = 88;

    BandPanel (Theme& t, AudioLabEqProcessor& p, EqDisplay& d)
        : theme (t), proc (p), display (d),
          onBtn (t, PillButton::Style::ibtn, {}), soloBtn (t, PillButton::Style::ibtn, {}),
          delBtn (t, PillButton::Style::rst, {}), closeBtn (t, PillButton::Style::rst, {}), dynBtn (t, PillButton::Style::ibtn, juce::String ("Dynamic"))
    {
        namespace P = params;
        setInterceptsMouseClicks (true, true);
        onBtn.icon = icons::power;      onBtn.iconOnly = true;
        soloBtn.icon = icons::headphones; soloBtn.iconOnly = true;
        delBtn.icon = icons::trash;     delBtn.iconOnly = true;
        closeBtn.icon = icons::cross;   closeBtn.iconOnly = true;
        closeBtn.setTooltip (fr ("Close the panel (Esc)"));
        closeBtn.onClick = [this] { display.clearSelection(); };
        onBtn.setTooltip (fr ("Turn the band on / off (double-click the handle)"));
        soloBtn.setTooltip (fr ("Listen only to the part of the spectrum this band processes (or Alt + click and hold the handle)"));
        delBtn.setTooltip (fr ("Delete the band (Delete key)"));
        dynBtn.setTooltip (fr ("Dynamic EQ: the band gain follows the level of the signal in its frequency range"));

        for (int i = 0; i < eq::numShapes; ++i) shapeBox.addItem (P::shapeNames()[i], i + 1);
        for (int i = 0; i < eq::numSlopes; ++i) slopeBox.addItem (P::slopeNames()[i], i + 1);
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
        slopeBox.onChange = [this] { applyChoice (slopeBox, params::fSlope); };
        placeBox.onChange = [this] { applyChoice (placeBox, params::fPlace); };

        for (juce::Component* c : { (juce::Component*) &onBtn, (juce::Component*) &soloBtn, (juce::Component*) &delBtn,
                                    (juce::Component*) &closeBtn, (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox,
                                    (juce::Component*) &placeBox, (juce::Component*) &scBox })
            addChildComponent (c);
        setVisible (false);
    }

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
        for (auto* c : { (juce::Component*) &onBtn, (juce::Component*) &delBtn, (juce::Component*) &closeBtn, (juce::Component*) &placeBox }) c->setVisible (any);
        for (auto* c : { (juce::Component*) &soloBtn, (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox, (juce::Component*) &scBox })
            c->setVisible (single);

        if (single)
        {
            const int b = band;
            using K = ArcKnob;
            const auto M = K::Size::main, S = K::Size::small;
            // teal : fréquence, gain, Q, seuil (forme de la courbe, niveau) ; orange : plage, attaque, retour (action, temps)
            kFreq = std::make_unique<K> (theme, *proc.bandParam (b, P::fFreq), fr ("Frequency"), "Hz", Fam::teal, M,
                [] (float v) { return P::fmtHzShort (v); }, 20.0f, fr ("Centre or cut-off frequency"));
            kGain = std::make_unique<K> (theme, *proc.bandParam (b, P::fGain), "Gain", "dB", Fam::teal, M,
                [] (float v) { return P::fmtDb (v); }, 0.0f, fr ("Band gain (bells, shelves, tilt)"));
            kQ = std::make_unique<K> (theme, *proc.bandParam (b, P::fQ), "Q", "Q", Fam::teal, M,
                [] (float v) { return P::fmtQ (v); }, 0.025f, fr ("Bandwidth: high Q = narrow (mouse wheel on the handle)"));
            kThr = std::make_unique<K> (theme, *proc.bandParam (b, P::fThr), fr ("Threshold"), "dB", Fam::teal, M,
                [] (float v) { return juce::String (v, 1); }, 0.0f, fr ("Level above which the dynamic band acts"));
            kRange = std::make_unique<K> (theme, *proc.bandParam (b, P::fRange), "Range", "dB", Fam::orange, M,
                [] (float v) { return P::fmtDb (v); }, 0.0f, fr ("Maximum gain added (+) or removed (−) when the signal goes above the threshold (Alt + mouse wheel on the handle)"));
            kAtt = std::make_unique<K> (theme, *proc.bandParam (b, P::fAtt), fr ("Attack"), "ms", Fam::orange, S,
                [] (float v) { return P::fmtMs (v); }, 0.5f, fr ("How fast this band's dynamics react (double-click: 10 ms)"));
            kRel = std::make_unique<K> (theme, *proc.bandParam (b, P::fRel), fr ("Release"), "ms", Fam::orange, S,
                [] (float v) { return P::fmtMs (v); }, 10.0f, fr ("Release time of this band's dynamics (double-click: 150 ms)"));
            for (auto* k : { kFreq.get(), kGain.get(), kQ.get(), kThr.get(), kRange.get(), kAtt.get(), kRel.get() })
            {
                k->onGestureStart = [this] { proc.pushUndo(); };
                addAndMakeVisible (k);
            }
            title = fr ("Band ") + juce::String (b + 1);
        }
        else if (any) title = juce::String (count) + fr (" bands selected");
        else title = {};

        setVisible (any);
        setSize (panelW, single ? singleH : multiH);
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
        slopeBox.setSelectedId (slopeIdx + 1, juce::dontSendNotification);
        placeBox.setSelectedId (juce::roundToInt (proc.bandValue (b, P::fPlace)) + 1, juce::dontSendNotification);
        const int slope = eq::slopeValues[juce::jlimit (0, eq::numSlopes - 1, slopeIdx)];
        dim (&slopeBox, eq::usesSlope (shape), 0.4f);
        dim (&dynBtn, eq::canBeDynamic (shape), 0.4f);
        dim (kGain.get(), eq::hasGain (shape));
        dim (kQ.get(), eq::usesQ (shape, slope));
        const bool dynOn = dyn && eq::canBeDynamic (shape);
        for (juce::Component* c : { (juce::Component*) kThr.get(), (juce::Component*) kRange.get(), (juce::Component*) kAtt.get(), (juce::Component*) kRel.get() })
            dim (c, dynOn);
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
        auto b = getLocalBounds().toFloat();
        juce::Path box; box.addRoundedRectangle (b.reduced (0.5f), 12);
        g.setColour (p.surface); g.fillPath (box);
        g.setColour (p.rail); g.strokePath (box, juce::PathStrokeType (1));

        if (count == 0) return;
        float x = 16;
        if (count == 1)
        {
            // pastille numérotée de la bande choisie
            g.setColour (p.select); g.fillEllipse (x, 20, 20, 20);
            g.setColour (p.text); g.setFont (theme.ui (10, 600));
            g.drawText (juce::String (band + 1), juce::Rectangle<float> (x, 20, 20, 20), juce::Justification::centred, false);
            x += 28;
        }
        g.setColour (p.text); g.setFont (theme.ui (14, 600));
        g.drawText (title, juce::Rectangle<float> (x, 16, count == 1 ? 70.0f : 200.0f, 28), juce::Justification::centredLeft, false);
        g.setColour (p.rail);
        g.fillRect (sepCloseX, 21.0f, 1.0f, 18.0f);   // avant la croix de fermeture
        if (count == 1) g.fillRect (sepX, 58.0f, 1.0f, 84.0f);
        else
        {
            g.setColour (p.text2); g.setFont (theme.ui (12));
            g.drawText (fr ("Drag a handle to move the whole selection · Shift: fine adjustment"),
                        juce::Rectangle<float> (16, 56, (float) getWidth() - 32, 18), juce::Justification::centredLeft, false);
        }
    }

private:
    void layout()
    {
        // marge intérieure 16 ; ligne 1 : … · active · écouter · supprimer | fermer
        const int y1 = 16, h1 = 28, right = getWidth() - 16;
        closeBtn.setBounds (right - 28, y1, 28, h1);
        sepCloseX = (float) right - 28 - 9;
        delBtn.setBounds (right - 28 - 18 - 28, y1, 28, h1);
        if (count == 1)
        {
            soloBtn.setBounds (delBtn.getX() - 4 - 28, y1, 28, h1);
            onBtn.setBounds (soloBtn.getX() - 4 - 28, y1, 28, h1);
            shapeBox.setBounds (118, y1, 100, h1);
            slopeBox.setBounds (224, y1, 96, h1);
            placeBox.setBounds (326, y1, juce::jmin (88, onBtn.getX() - 12 - 326), h1);

            // ligne 2 : potentiomètres 72 px (colonnes de 80), dynamique ; ligne 3 : attaque, retour
            const int y2 = 58, kw = 80, kh = 94;
            int x = 16;
            for (auto* k : { kFreq.get(), kGain.get(), kQ.get() }) { k->setBounds (x, y2, kw, kh); x += kw + 6; }
            sepX = (float) x + 4; x += 14;
            dynBtn.setBounds (x, y2 + 22, 88, 28);
            x = dynBtn.getRight() + 8;
            kThr->setBounds (x, y2, kw, kh);
            kRange->setBounds (x + kw + 6, y2, kw, kh);
            kAtt->setBounds (x + 4, y2 + kh + 12, kAtt->idealWidth(), 28);
            kRel->setBounds (x + kw + 10, y2 + kh + 12, kRel->idealWidth(), 28);
            scBox.setBounds (dynBtn.getX(), y2 + kh + 12, dynBtn.getWidth(), 28);   // sous « Dynamic »
        }
        else
        {
            onBtn.setBounds (delBtn.getX() - 4 - 28, y1, 28, h1);
            placeBox.setBounds (224, y1, 104, h1);
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
    float sepX = 246, sepCloseX = 0;
    PillButton onBtn, soloBtn, delBtn, closeBtn, dynBtn;
    juce::ComboBox shapeBox, slopeBox, placeBox, scBox;
    std::unique_ptr<ArcKnob> kFreq, kGain, kQ, kThr, kRange, kAtt, kRel;
};
} // namespace audiolab
