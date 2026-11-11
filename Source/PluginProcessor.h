#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "Params.h"
#include "EqEngine.h"
#include "LinearPhase.h"
#include "AutoGain.h"

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
    juce::ValueTree ui;   // taille, analyseur, échelle dB, A/B, preset, panneau avancé

    float getParam (const juce::String& id) const { return apvts.getRawParameterValue (id)->load(); }
    float bandValue (int band, const char* field) const { return bandPtr[(size_t) band][(size_t) fieldIndex (field)]->load(); }
    juce::RangedAudioParameter* bandParam (int band, const char* field) const { return bandParams[(size_t) band][(size_t) fieldIndex (field)]; }

    bool isUsed (int band) const { return bandValue (band, audiolab::params::fUsed) > 0.5f; }
    audiolab::eq::BandSpec getBandSpec (int band, bool withScale = true) const;
    audiolab::eq::DynSettings getDyn (int band) const;
    double gainScale() const { return getParam (audiolab::params::gainScale) / 100.0; }
    int phaseMode() const { return juce::jlimit (0, 2, juce::roundToInt (getParam (audiolab::params::phase))); }
    bool isLinearPhase() const { return phaseMode() != audiolab::params::phaseZero; }   // un mode FIR (linéaire ou naturel)
    int linResIndex() const { return juce::jlimit (0, 4, juce::roundToInt (getParam (audiolab::params::linRes))); }
    float getDynOffset (int band) const { return eng().getDynOffset (band); }
    // 3.0 : seuil automatique actuel d'une bande (dBFS), quand bN_thrauto est On (pour l'afficher)
    float getAutoThresholdDb (int band) const { return eng().getAutoThreshold (band); }
    double getSampleRateSafe() const { const double s = getSampleRate(); return s > 0 ? s : 48000.0; }

    // 3.0, Auto gain à sonie égale (ITU-R BS.1770) : gain de compensation actuel en dB (borné à ± 24 dB).
    // Auto gain en service : valeur réellement appliquée (mesure de sonie entrée / sortie) ; coupé : valeur
    // qu'il prendrait d'abord (prévision d'après la courbe et le dernier spectre connu).
    float getAutoGainDb() const { return autoG.gainDb(); }
    // Dernier écart de sonie mesuré entrée − sortie avant compensation, en LU (0 tant que rien n'est mesuré)
    float getAutoGainMeasuredDb() const { return autoG.measuredDb(); }

    // 3.0, pré-écho du FIR en place (calculé par le fil de conception) : durée avant le pic pendant laquelle
    // l'énergie cumulée dépasse −60 / −90 dB du pic, et niveau de ce qui arrive plus de 5 / 20 ms avant
    // le pic (−200 : rien). Valeurs nulles en Zero latency, et tant que le FIR n'est pas conçu.
    using PreEcho = audiolab::eq::fir::PreEcho;
    PreEcho getPreEcho() const;
    // Texte court prêt à afficher, par exemple « Pre-echo: 12 ms above −60 dB » (vide en Zero latency)
    juce::String getPreEchoText() const;
    // Un changement de mode de phase ou de longueur est en cours (attente du FIR, fondu enchaîné)
    bool isSwitchingMode() const { return switching.load(); }

    // Mode réellement actif (après le fondu de changement), longueur du FIR, latence annoncée
    int activePhaseMode() const { return activeMode.load(); }
    int activeFirLength() const { return activeN.load(); }
    int reportedLatency() const { return latencyNow.load(); }
    int latencyForMode (int mode) const;   // latence qu'aurait ce mode avec les réglages actuels
    bool sideChainConnected() const { return scConnected.load(); }
    // Courbe réelle du FIR (modes Linear / Natural phase), pour l'écran
    void getFirDisplay (audiolab::eq::LinearPhase::DisplayData& d) const { linear.getDisplay (d); }
    int firDisplayVersion() const { return linear.displayVersion(); }
    int firDesignCount() const { return linear.designsDone(); }   // nombre de jeux de FIR conçus (tests)
    std::atomic<bool> editorOpen { false };

    // Modifications venant de l'interface (fil des messages)
    void setValue (juce::RangedAudioParameter* p, float value);
    void setBandValue (int band, const char* field, float value) { setValue (bandParam (band, field), value); }
    int firstFreeBand() const;
    int createBand (audiolab::eq::Shape shape, float freq, float gain, float q, int slope,
                    audiolab::eq::Place place = audiolab::eq::Place::stereo);
    void deleteBand (int band);
    void clearAllBands();
    void applyPreset (int index);
    void setSolo (int band) { soloBand = band; for (auto& e : engines) e.setSolo (band); }
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

    // Tests : force l'état « lecture en cours » (l'hôte de test n'a pas de tête de lecture)
    std::atomic<int> testPlaying { -1 };
    // Tests : loi du fondu enchaîné d'un changement de mode. 0 = celle du plugin (puissance constante pour la
    // corrélation mesurée), 1 = puissance constante (sinus / cosinus), 2 = linéaire, 3 = corrélation d'une seule fenêtre
    std::atomic<int> testFadeLaw { 0 };

private:
    static int fieldIndex (const char* field);
    void updateEngineTargets (bool stereo);
    audiolab::eq::LinearPhase::Snapshot makeSnapshot (bool stereo) const;
    int desiredFirLength (int mode, bool playing) const;
    void applySwitch (int mode, int N);
    void renderDual (float* l, float* r, int n, const float* scL, const float* scR, bool stereo, uint32_t dynMask);
    void finishSwitch();
    static uint64_t autoKey (const std::array<audiolab::eq::BandSpec, audiolab::eq::maxBands>& bands,
                             const std::array<double, audiolab::eq::maxBands>& span, bool stereo, double fs);
    void verifyAutoLength (const audiolab::eq::LinearPhase::Snapshot& snap);
    audiolab::eq::Engine& eng() { return engines[(size_t) engIdx.load (std::memory_order_relaxed)]; }
    const audiolab::eq::Engine& eng() const { return engines[(size_t) engIdx.load (std::memory_order_relaxed)]; }

    class Designer;
    std::unique_ptr<Designer> designer;

    std::array<std::array<std::atomic<float>*, audiolab::params::numBandFields>, audiolab::eq::maxBands> bandPtr {};
    std::array<std::array<juce::RangedAudioParameter*, audiolab::params::numBandFields>, audiolab::eq::maxBands> bandParams {};
    juce::AudioProcessorParameter* bypassParam = nullptr;

    // Deux chaînes zéro latence : celle du mode en place, et celle du mode qui arrive pendant un
    // changement de mode (les deux chemins tournent en parallèle le temps du fondu enchaîné)
    std::array<audiolab::eq::Engine, 2> engines;
    std::atomic<int> engIdx { 0 };
    audiolab::eq::LinearPhase linear;
    audiolab::eq::AutoGain autoG;
    std::atomic<bool> stereoLayout { true }, scConnected { false }, switching { false };
    std::atomic<int> activeMode { 0 }, activeN { 0 }, latencyNow { 0 };
    std::atomic<int> designMode { 0 }, designN { 0 };   // réglage que le fil de conception prépare
    int currentLatency = 0, curMode = 0, curN = 0;   // chemin en place (fil audio)
    int soloBand = -1;

    // Résolution Auto vérifiée par la mesure (fil de conception) : clé des réglages et longueur retenue
    std::atomic<uint64_t> autoVerified { 0 };
    uint64_t lastVerifyKey = 0;   // (fil de conception)
    std::mutex verifyMutex;
    audiolab::eq::fir::Work verifyWork;
    audiolab::eq::fir::Result verifyTmp;
    std::vector<audiolab::eq::fir::cd> verifyScratch;

    // Changement de mode / de latence sans silence : attente du FIR (l'ancien chemin continue), mise en
    // route du nouveau chemin en parallèle, fondu enchaîné, puis l'ancien chemin est arrêté
    enum class Sw { idle, waitDesign, preRoll, fade };
    Sw sw = Sw::idle;
    int swCount = 0, swLen = 0, pendMode = 0, pendN = 0, pendLat = 0, swDesigns = 0, corrLen = 0;
    double swRho = 0.0;
    struct Corr { double xx = 0.0, yy = 0.0, xy = 0.0; };
    Corr corr[2];
    std::vector<float> nwL, nwR, dl2L, dl2R, det2L, det2R, sc2L, sc2R, trace2;
    std::array<const float*, audiolab::eq::maxBands> tracePtr2 {};

    // retard du signal sec (bypass, écoute solo en phase linéaire) et du détecteur en mode FIR
    static constexpr int chunk = 1024;
    std::vector<float> dryRingL, dryRingR, scRingL, scRingR;
    int dryW = 0;
    std::vector<float> dlyL, dlyR, detL, detR, scDL, scDR, mono;
    // Natural phase : sortie du FIR sortant et position du fondu quand le partage FIR / moteur change ;
    // Linear phase : gain dynamique de chaque bande à chaque échantillon (mélange de FIR)
    std::vector<float> oldL, oldR, fadeT, traceBuf;
    std::array<const float*, audiolab::eq::maxBands> tracePtr {};
    juce::SmoothedValue<float> outSm, bypassSm;

    std::vector<std::vector<float>> undoStack, redoStack;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioLabEqProcessor)
};
