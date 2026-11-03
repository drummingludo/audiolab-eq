#pragma once
#include "PluginProcessor.h"
#include "Widgets.h"
#include "EqDisplay.h"
#include "BandPanel.h"

// Fenêtre v2 : la courbe occupe tout le haut, le panneau de la bande flotte sous la courbe,
// les réglages généraux sont dans la barre du bas.
class AudioLabEqEditor : public juce::AudioProcessorEditor,
                         private juce::Timer,
                         private juce::DarkModeSettingListener
{
public:
    explicit AudioLabEqEditor (AudioLabEqProcessor&);
    ~AudioLabEqEditor() override;

    void paint (juce::Graphics&) override {}
    void resized() override;
    bool keyPressed (const juce::KeyPress&) override;

    // 1120 × 640 (validé) ; panneau « Advanced settings » ouvert : 1120 × 714
    static constexpr int baseW = 1120, baseH = 640, baseHOpen = 714;

    audiolab::EqDisplay& getDisplay() { return display; }   // tests

private:
    class Content : public juce::Component
    {
    public:
        explicit Content (AudioLabEqEditor& e) : ed (e) {}
        void paint (juce::Graphics& g) override;
        void resized() override;
        AudioLabEqEditor& ed;
    };

    // Ombre du panneau flottant, peinte au-dessus de la courbe
    class Shadow : public juce::Component
    {
    public:
        explicit Shadow (AudioLabEqEditor& e) : ed (e) { setInterceptsMouseClicks (false, false); }
        void paint (juce::Graphics& g) override;
        AudioLabEqEditor& ed;
    };

    // Poignées sélectionnées redessinées par-dessus le panneau quand elles passent dessous
    // (elles restent visibles et saisissables ; les gestes sont transmis à la courbe).
    class NodeOverlay : public juce::Component
    {
    public:
        explicit NodeOverlay (AudioLabEqEditor& e) : ed (e) { setMouseCursor (juce::MouseCursor::DraggingHandCursor); }
        void update();
        void paint (juce::Graphics& g) override;
        bool hitTest (int x, int y) override;
        void mouseMove (const juce::MouseEvent& e) override        { ed.display.mouseMove (rel (e)); }
        void mouseExit (const juce::MouseEvent& e) override        { ed.display.mouseExit (rel (e)); }
        void mouseDown (const juce::MouseEvent& e) override        { ed.display.mouseDown (rel (e)); }
        void mouseDrag (const juce::MouseEvent& e) override        { ed.display.mouseDrag (rel (e)); }
        void mouseUp (const juce::MouseEvent& e) override          { ed.display.mouseUp (rel (e)); }
        void mouseDoubleClick (const juce::MouseEvent& e) override { ed.display.mouseDoubleClick (rel (e)); }
        void mouseWheelMove (const juce::MouseEvent& e, const juce::MouseWheelDetails& w) override { ed.display.mouseWheelMove (rel (e), w); }
    private:
        juce::MouseEvent rel (const juce::MouseEvent& e) const { return e.getEventRelativeTo (&ed.display); }
        juce::Array<int> shown;
        AudioLabEqEditor& ed;
    };

    void timerCallback() override;
    void darkModeSettingChanged() override { applyTheme(); }
    void applyTheme();
    void rebuildPanel();
    void placePanel();
    void setAdvanced (bool open, bool resizeWindow);
    int currentH() const { return advOpen ? baseHOpen : baseH; }
    juce::String latencyText() const;
    juce::String sideChainText() const;

    AudioLabEqProcessor& proc;
    audiolab::Theme theme;
    audiolab::ALLookAndFeel lnf { theme };
    juce::TooltipWindow tooltips { nullptr, 700 };
    Content content { *this };
    audiolab::EqDisplay display;
    Shadow shadow { *this };
    audiolab::BandPanel panel;
    NodeOverlay overlay { *this };
    audiolab::Toast toast { theme };

    // barre du bas : segments dans l'ordre Zero latency / Natural / Linear (paramètre : 0 / 2 / 1)
    audiolab::SegButtons phaseSeg { theme, { "Zero latency", "Natural", "Linear" } };
    juce::ComboBox analyzerBox;
    std::unique_ptr<audiolab::ArcKnob> vScale, vOut;
    audiolab::PillButton autoBtn;
    audiolab::Disclosure advBtn { theme, "Advanced settings" };

    // panneau « Advanced settings »
    audiolab::SegButtons resSeg { theme, { "Auto", "Low", "Medium", "High", "Max" } };
    bool advOpen = false;
    juce::String latText, scText;

    std::unique_ptr<juce::ParameterAttachment> bypassAtt, phaseAtt;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioLabEqEditor)
};
