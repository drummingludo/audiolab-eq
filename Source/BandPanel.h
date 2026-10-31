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
    static constexpr int panelW = 540, singleH = 182, multiH = 78;

    BandPanel (Theme& t, AudioLabEqProcessor& p, EqDisplay& d)
        : theme (t), proc (p), display (d),
          onBtn (t, PillButton::Style::ibtn, {}), soloBtn (t, PillButton::Style::ibtn, {}),
          delBtn (t, PillButton::Style::rst, {}), dynBtn (t, PillButton::Style::ibtn, juce::String::fromUTF8 ("Dynamique"))
    {
        namespace P = params;
        setInterceptsMouseClicks (true, true);
        onBtn.icon = icons::power;      onBtn.iconOnly = true;
        soloBtn.icon = icons::headphones; soloBtn.iconOnly = true;
        delBtn.icon = icons::trash;     delBtn.iconOnly = true;
        for (auto* b : { &onBtn, &soloBtn, &delBtn, &dynBtn }) b->raised = true;
        onBtn.setTooltip (fr ("Activer / désactiver la bande (double-clic sur la poignée)"));
        soloBtn.setTooltip (fr ("Écouter seulement la zone du spectre traitée par la bande (ou Alt + clic maintenu sur la poignée)"));
        delBtn.setTooltip (fr ("Supprimer la bande (touche Effacer)"));
        dynBtn.setTooltip (fr ("EQ dynamique : le gain de la bande varie selon le niveau du signal dans sa zone de fréquences"));

        for (int i = 0; i < eq::numShapes; ++i) shapeBox.addItem (P::shapeNames()[i], i + 1);
        for (int i = 0; i < eq::numSlopes; ++i) slopeBox.addItem (P::slopeNames()[i], i + 1);
        for (int i = 0; i < eq::numPlaces; ++i) placeBox.addItem (P::placeNames()[i], i + 1);
        shapeBox.setTitle (fr ("Forme")); slopeBox.setTitle (fr ("Pente")); placeBox.setTitle (fr ("Placement stéréo"));
        shapeBox.setTooltip (fr ("Forme du filtre"));
        slopeBox.setTooltip (fr ("Pente des coupes et des shelfs"));
        placeBox.setTooltip (fr ("Stéréo, gauche, droite, Mid (centre) ou Side (côtés)"));
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
        shapeBox.onChange = [this] { applyChoice (shapeBox, params::fShape); };
        slopeBox.onChange = [this] { applyChoice (slopeBox, params::fSlope); };
        placeBox.onChange = [this] { applyChoice (placeBox, params::fPlace); };

        for (juce::Component* c : { (juce::Component*) &onBtn, (juce::Component*) &soloBtn, (juce::Component*) &delBtn,
                                    (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox,
                                    (juce::Component*) &placeBox })
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
        kFreq.reset(); kGain.reset(); kQ.reset(); kThr.reset(); kRange.reset(); vAtt.reset(); vRel.reset();

        const bool single = count == 1, any = count >= 1;
        for (auto* c : { (juce::Component*) &onBtn, (juce::Component*) &delBtn, (juce::Component*) &placeBox }) c->setVisible (any);
        for (auto* c : { (juce::Component*) &soloBtn, (juce::Component*) &dynBtn, (juce::Component*) &shapeBox, (juce::Component*) &slopeBox })
            c->setVisible (single);

        if (single)
        {
            const int b = band;
            auto bc = [b] (const Pal& q) { return bandColour (b, q); };
            auto ora = [] (const Pal& q) { return q.orange; };
            kFreq = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fFreq), fr ("Fréquence"), "Hz", bc, ArcKnob::Size::xs,
                [] (float v) { return P::fmtHzShort (v); }, 20.0f, fr ("Fréquence centrale ou de coupure"));
            kGain = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fGain), "Gain", "dB", bc, ArcKnob::Size::xs,
                [] (float v) { return P::fmtDb (v); }, 0.0f, fr ("Gain de la bande (cloches, shelfs, tilt)"));
            kQ = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fQ), "Q", "Q", bc, ArcKnob::Size::xs,
                [] (float v) { return P::fmtQ (v); }, 0.025f, fr ("Largeur de bande : Q élevé = étroit (molette sur la poignée)"));
            kThr = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fThr), fr ("Seuil"), "dB", ora, ArcKnob::Size::xs,
                [] (float v) { return juce::String (v, 1); }, 0.0f, fr ("Niveau à partir duquel la bande dynamique agit"));
            kRange = std::make_unique<ArcKnob> (theme, *proc.bandParam (b, P::fRange), "Plage", "dB", ora, ArcKnob::Size::xs,
                [] (float v) { return P::fmtDb (v); }, 0.0f, fr ("Gain ajouté (+) ou retiré (−) au maximum quand le signal dépasse le seuil (Alt + molette sur la poignée)"));
            vAtt = std::make_unique<ValueBox> (theme, *proc.bandParam (b, P::fAtt), fr ("Attaque"), [] (float v) { return P::fmtMs (v); }, true);
            vRel = std::make_unique<ValueBox> (theme, *proc.bandParam (b, P::fRel), fr ("Retour"), [] (float v) { return P::fmtMs (v); }, true);
            vAtt->setTooltip (fr ("Vitesse de réaction de la dynamique de cette bande (glisser verticalement, double-clic : 10 ms)"));
            vRel->setTooltip (fr ("Temps de retour de la dynamique de cette bande (glisser verticalement, double-clic : 150 ms)"));
            for (auto* k : { kFreq.get(), kGain.get(), kQ.get(), kThr.get(), kRange.get() })
            {
                k->onGestureStart = [this] { proc.pushUndo(); };
                addAndMakeVisible (k);
            }
            for (auto* v : { vAtt.get(), vRel.get() })
            {
                v->raised = true;
                v->onGestureStart = [this] { proc.pushUndo(); };
                addAndMakeVisible (v);
            }
            title = fr ("Bande ") + juce::String (b + 1);
        }
        else if (any) title = juce::String (count) + fr (" bandes sélectionnées");
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
        for (juce::Component* c : { (juce::Component*) kThr.get(), (juce::Component*) kRange.get(), (juce::Component*) vAtt.get(), (juce::Component*) vRel.get() })
            dim (c, dynOn);
    }

    void paint (juce::Graphics& g) override
    {
        const auto& p = theme.pal();
        auto b = getLocalBounds().toFloat();
        juce::Path box; box.addRoundedRectangle (b.reduced (0.5f), 14);
        g.setColour (p.panel); g.fillPath (box);
        g.setColour (p.dark ? juce::Colours::white.withAlpha (0.08f) : juce::Colours::black.withAlpha (0.06f));
        g.strokePath (box, juce::PathStrokeType (1));

        if (count == 0) return;
        float x = 14;
        if (count == 1)
        {
            g.setColour (bandColour (band, p)); g.fillEllipse (x, 12 + 8, 12, 12);
            x += 20;
        }
        g.setColour (p.text); g.setFont (theme.ui (14, 600));
        g.drawText (title, juce::Rectangle<float> (x, 12, count == 1 ? 66.0f : 200.0f, 28), juce::Justification::centredLeft, false);
        if (count == 1)
        {
            g.setColour (p.dark ? juce::Colours::white.withAlpha (0.1f) : juce::Colours::black.withAlpha (0.08f));
            g.fillRect (sepX, 50.0f, 1.0f, 70.0f);
        }
        else
        {
            g.setColour (p.muted); g.setFont (theme.ui (11.5f));
            g.drawText (fr ("Glissez une poignée pour déplacer toute la sélection · Maj : réglage fin"),
                        juce::Rectangle<float> (14, 46, (float) getWidth() - 28, 18), juce::Justification::centredLeft, false);
        }
    }

private:
    void layout()
    {
        const int y1 = 12, h1 = 28, right = getWidth() - 14;
        delBtn.setBounds (right - 28, y1, 28, h1);
        if (count == 1)
        {
            soloBtn.setBounds (right - 28 - 6 - 28, y1, 28, h1);
            onBtn.setBounds (right - 3 * 28 - 12, y1, 28, h1);
            shapeBox.setBounds (106, y1, 108, h1);
            slopeBox.setBounds (220, y1, 100, h1);
            placeBox.setBounds (326, y1, 88, h1);

            const int y2 = 50, kw = 70, kh = 76;
            int x = 14;
            for (auto* k : { kFreq.get(), kGain.get(), kQ.get() }) { k->setBounds (x, y2, kw, kh); x += kw + 4; }
            sepX = (float) x + 10; x += 21 + 4;
            dynBtn.setBounds (x, y2 + 22, juce::roundToInt (dynBtn.idealWidth()), 28);
            x = dynBtn.getRight() + 6;
            kThr->setBounds (x, y2, kw, kh);
            kRange->setBounds (x + kw + 4, y2, kw, kh);
            vAtt->setBounds (x, y2 + kh + 2, kw, 32);
            vRel->setBounds (x + kw + 4, y2 + kh + 2, kw, 32);
        }
        else
        {
            onBtn.setBounds (right - 2 * 28 - 6, y1, 28, h1);
            placeBox.setBounds (220, y1, 104, h1);
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
    float sepX = 246;
    PillButton onBtn, soloBtn, delBtn, dynBtn;
    juce::ComboBox shapeBox, slopeBox, placeBox;
    std::unique_ptr<ArcKnob> kFreq, kGain, kQ, kThr, kRange;
    std::unique_ptr<ValueBox> vAtt, vRel;
};
} // namespace audiolab
