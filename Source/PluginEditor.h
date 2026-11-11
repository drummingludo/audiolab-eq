#pragma once
#include "PluginProcessor.h"
#include "Widgets.h"
#include "EqDisplay.h"
#include "BandPanel.h"

// Fenêtre 3.0 (piste B validée) : écran de la courbe 1072 × 400, clavier, bandeau fixe de la bande
// (1072 × 104), barre du bas. Tout est posé sur le module de 8 px de Lab Comp 3.0.0, marges de 24.
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
    audiolab::BandPanel& getPanel() { return panel; }       // tests

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
    void rebuildPanel();
    void setAdvanced (bool open, bool resizeWindow);
    int currentH() const { return advOpen ? baseHOpen : baseH; }
    juce::String latencyText() const;
    juce::String sideChainText() const;
    // niveaux de sortie : deux barres de 4 px dans la barre du bas, entre « Advanced settings » et « Scale »
    juce::Rectangle<int> meterArea() const { return { 712, 594, 80, 12 }; }

    AudioLabEqProcessor& proc;
    audiolab::Theme theme;
    audiolab::ALLookAndFeel lnf { theme };
    juce::TooltipWindow tooltips { nullptr, 700 };
    Content content { *this };
    audiolab::EqDisplay display;
    audiolab::BandPanel panel;
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
