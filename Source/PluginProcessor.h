#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "Params.h"
#include "EqEngine.h"
#include "LinearPhase.h"

class AudioLabEqProcessor : public juce::AudioProcessor
{
public:
    AudioLabEqProcessor();
    ~AudioLabEqProcessor() override;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    using AudioProcessor::processBlock;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return JucePlugin_Name; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    juce::AudioProcessorParameter* getBypassParameter() const override { return bypassParam; }

    // ── État partagé avec l'interface ────────────────────────────────────
    juce::AudioProcessorValueTreeState apvts;
    juce::ValueTree ui;   // thème, taille, analyseur, échelle dB, A/B, preset

    float getParam (const juce::String& id) const { return apvts.getRawParameterValue (id)->load(); }
    float bandValue (int band, const char* field) const { return bandPtr[(size_t) band][(size_t) fieldIndex (field)]->load(); }
    juce::RangedAudioParameter* bandParam (int band, const char* field) const { return bandParams[(size_t) band][(size_t) fieldIndex (field)]; }

    bool isUsed (int band) const { return bandValue (band, audiolab::params::fUsed) > 0.5f; }
    audiolab::eq::BandSpec getBandSpec (int band, bool withScale = true) const;
    audiolab::eq::DynSettings getDyn (int band) const;
    double gainScale() const { return getParam (audiolab::params::gainScale) / 100.0; }
    bool isLinearPhase() const { return getParam (audiolab::params::phase) > 0.5f; }
    float getDynOffset (int band) const { return engine.getDynOffset (band); }
    double getSampleRateSafe() const { const double s = getSampleRate(); return s > 0 ? s : 48000.0; }
    float getAutoGainDb() const { return autoGainDb.load(); }

    // Modifications venant de l'interface (fil des messages)
    void setValue (juce::RangedAudioParameter* p, float value);
    void setBandValue (int band, const char* field, float value) { setValue (bandParam (band, field), value); }
    int firstFreeBand() const;
    int createBand (audiolab::eq::Shape shape, float freq, float gain, float q, int slope,
                    audiolab::eq::Place place = audiolab::eq::Place::stereo);
    void deleteBand (int band);
    void clearAllBands();
    void applyPreset (int index);
    void setSolo (int band) { soloBand = band; engine.setSolo (band); }
    int getSolo() const { return soloBand; }

    // Annuler / rétablir (états complets avant chaque geste)
    void pushUndo();
    bool undo();
    bool redo();
    bool canUndo() const { return ! undoStack.empty(); }
    bool canRedo() const { return ! redoStack.empty(); }

    // Comparaison A/B
    void switchAB (const juce::String& slot);
    void copyABToOther();
    juce::String currentAB() const { return ui.getProperty ("ab", "A").toString(); }

    // Analyseur et mesures (fil audio → interface)
    struct Tap
    {
        static constexpr int size = 32768;
        std::vector<float> buf = std::vector<float> (size, 0.0f);
        std::atomic<int> w { 0 };
        void push (const float* x, int n)
        {
            int p = w.load (std::memory_order_relaxed);
            for (int i = 0; i < n; ++i) { buf[(size_t) p] = x[i]; p = (p + 1) & (size - 1); }
            w.store (p, std::memory_order_release);
        }
        void readLast (float* out, int n) const
        {
            int p = (w.load (std::memory_order_acquire) - n + size) & (size - 1);
            for (int i = 0; i < n; ++i) { out[i] = buf[(size_t) p]; p = (p + 1) & (size - 1); }
        }
    };
    Tap tapPre, tapPost;
    std::atomic<float> peakL { 0 }, peakR { 0 };
    std::atomic<juce::uint32> lastBlockMs { 0 };

    std::vector<float> getStateVector() const;
    void setStateVector (const std::vector<float>&);

private:
    static int fieldIndex (const char* field);
    void updateEngineTargets (bool stereo);
    audiolab::eq::LinearPhase::Snapshot makeSnapshot (bool stereo) const;
    void computeAutoGain();

    class Designer;
    std::unique_ptr<Designer> designer;

    std::array<std::array<std::atomic<float>*, audiolab::params::numBandFields>, audiolab::eq::maxBands> bandPtr {};
    std::array<std::array<juce::RangedAudioParameter*, audiolab::params::numBandFields>, audiolab::eq::maxBands> bandParams {};
    juce::AudioProcessorParameter* bypassParam = nullptr;

    audiolab::eq::Engine engine;
    audiolab::eq::LinearPhase linear;
    std::atomic<bool> linearActive { false }, stereoLayout { true };
    int currentLatency = 0;
    int soloBand = -1;

    // retard du signal sec (bypass, écoute solo en phase linéaire)
    static constexpr int chunk = 1024;
    std::vector<float> dryRingL, dryRingR;
    int dryW = 0;
    std::vector<float> inL, inR, dlyL, dlyR, mono;
    juce::SmoothedValue<float> outSm, bypassSm;

    std::array<audiolab::eq::BandSpec, audiolab::eq::maxBands> lastSpecs {};
    bool specsValid = false;
    std::atomic<float> autoGainDb { 0.0f };

    std::vector<std::vector<float>> undoStack, redoStack;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioLabEqProcessor)
};
