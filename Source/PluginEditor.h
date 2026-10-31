#pragma once
#include "PluginProcessor.h"
#include "Widgets.h"
#include "EqDisplay.h"

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

    static constexpr int baseW = 1160, baseH = 700;

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

    void timerCallback() override;
    void darkModeSettingChanged() override { applyTheme(); }
    void applyTheme();
    void rebuildBandPanel();
    void syncBandPanel();
    void layoutBandPanel();
    juce::Rectangle<int> panelArea() const { return { 16, 532, baseW - 32, 154 }; }

    AudioLabEqProcessor& proc;
    audiolab::Theme theme;
    audiolab::ALLookAndFeel lnf { theme };
    juce::TooltipWindow tooltips { nullptr, 700 };
    Content content { *this };
    audiolab::EqDisplay display;
    audiolab::Toast toast { theme };

    // barre du haut
    juce::ComboBox presetBox, analyzerBox;
    audiolab::PillButton undoBtn, redoBtn, abA, abB, abCopy, drawBtn, bypassBtn;
    audiolab::SegButtons phaseSeg { theme, { juce::String::fromUTF8 ("Zéro latence"), juce::String::fromUTF8 ("Linéaire") } };
    std::unique_ptr<juce::ParameterAttachment> bypassAtt;
    bool bypassed = false;

    // panneau de bande
    int panelBand = -1, panelCount = 0;
    juce::String panelTitle;
    audiolab::PillButton onBtn, soloBtn, delBtn, dynBtn;
    juce::ComboBox shapeBox, slopeBox;
    audiolab::SegButtons placeSeg { theme, audiolab::params::placeShort() };
    std::unique_ptr<audiolab::ArcKnob> kFreq, kGain, kQ, kThr, kRange;

    // réglages généraux
    std::unique_ptr<audiolab::ArcKnob> kOut, kScale, kAtk, kRel;
    audiolab::PillButton autoBtn;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioLabEqEditor)
};
