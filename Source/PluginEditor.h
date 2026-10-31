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

    static constexpr int baseW = 1120, baseH = 624;

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

    void timerCallback() override;
    void darkModeSettingChanged() override { applyTheme(); }
    void applyTheme();
    void rebuildPanel();
    void placePanel();

    AudioLabEqProcessor& proc;
    audiolab::Theme theme;
    audiolab::ALLookAndFeel lnf { theme };
    juce::TooltipWindow tooltips { nullptr, 700 };
    Content content { *this };
    audiolab::EqDisplay display;
    Shadow shadow { *this };
    audiolab::BandPanel panel;
    audiolab::Toast toast { theme };
    bool panelOnTop = false;

    // barre du bas
    audiolab::SegButtons phaseSeg { theme, { juce::String::fromUTF8 ("Zéro latence"), juce::String::fromUTF8 ("Linéaire") } };
    juce::ComboBox analyzerBox;
    std::unique_ptr<audiolab::ValueBox> vScale, vOut;
    audiolab::PillButton autoBtn;

    std::unique_ptr<juce::ParameterAttachment> bypassAtt;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioLabEqEditor)
};
