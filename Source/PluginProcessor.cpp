#include "PluginProcessor.h"
#include "PluginEditor.h"

using namespace audiolab;
using namespace audiolab::eq;
namespace P = audiolab::params;

// ── Fil de conception des FIR (mode phase linéaire) ─────────────────────
class AudioLabEqProcessor::Designer : public juce::Thread
{
public:
    explicit Designer (AudioLabEqProcessor& p) : juce::Thread ("AudioLabEQ FIR"), proc (p) {}
    void poke() { force = true; notify(); }

    void run() override
    {
        while (! threadShouldExit())
        {
            wait (20);
            proc.linear.collectGarbage();
            if (! proc.linearActive.load()) { last.clear(); continue; }
            const auto snap = proc.makeSnapshot (proc.stereoLayout.load());
            std::vector<double> key;
            key.reserve (maxBands * 6 + 1);
            key.push_back (snap.stereo ? 1 : 0);
            for (const auto& b : snap.bands)
            {
                if (! b.active) { key.push_back (-1); continue; }
                key.insert (key.end(), { (double) b.shape, (double) b.slope, (double) b.place,
                                         std::round (std::log (b.freq) * 2000.0), std::round (b.gainDb * 20.0), std::round (std::log (b.q) * 500.0) });
            }
            if (key != last || force.exchange (false))
            {
                last = key;
                proc.linear.design (snap);
            }
        }
    }

private:
    AudioLabEqProcessor& proc;
    std::vector<double> last;
    std::atomic<bool> force { false };
};

// ── Construction ────────────────────────────────────────────────────────
AudioLabEqProcessor::AudioLabEqProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput ("Input", juce::AudioChannelSet::stereo(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "AudioLabEQ", P::createLayout()),
      ui ("ui")
{
    for (int b = 0; b < maxBands; ++b)
        for (int f = 0; f < P::numBandFields; ++f)
        {
            const auto id = P::bandId (b, P::bandFields[f]);
            bandPtr[(size_t) b][(size_t) f] = apvts.getRawParameterValue (id);
            bandParams[(size_t) b][(size_t) f] = apvts.getParameter (id);
        }
    bypassParam = apvts.getParameter (P::bypass);
    ui.setProperty ("analyzer", "prepost", nullptr);
    ui.setProperty ("dbRange", 12, nullptr);
    ui.setProperty ("scale", 1.0, nullptr);
    ui.setProperty ("preset", 0, nullptr);
    ui.setProperty ("ab", "A", nullptr);

    designer = std::make_unique<Designer> (*this);
    designer->startThread (juce::Thread::Priority::low);
}

AudioLabEqProcessor::~AudioLabEqProcessor()
{
    designer->stopThread (2000);
}

bool AudioLabEqProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto out = layouts.getMainOutputChannelSet();
    if (out != juce::AudioChannelSet::mono() && out != juce::AudioChannelSet::stereo()) return false;
    return out == layouts.getMainInputChannelSet();
}

int AudioLabEqProcessor::fieldIndex (const char* field)
{
    for (int i = 0; i < P::numBandFields; ++i)
        if (std::strcmp (P::bandFields[i], field) == 0) return i;
    jassertfalse;
    return 0;
}

BandSpec AudioLabEqProcessor::getBandSpec (int b, bool withScale) const
{
    BandSpec s;
    s.active = bandValue (b, P::fUsed) > 0.5f && bandValue (b, P::fOn) > 0.5f;
    s.shape = (Shape) juce::jlimit (0, numShapes - 1, juce::roundToInt (bandValue (b, P::fShape)));
    s.slope = slopeValues[juce::jlimit (0, numSlopes - 1, juce::roundToInt (bandValue (b, P::fSlope)))];
    s.place = (Place) juce::jlimit (0, numPlaces - 1, juce::roundToInt (bandValue (b, P::fPlace)));
    s.freq = bandValue (b, P::fFreq);
    s.q = bandValue (b, P::fQ);
    s.gainDb = hasGain (s.shape) ? bandValue (b, P::fGain) * (withScale ? gainScale() : 1.0) : 0.0;
    return s;
}

DynSettings AudioLabEqProcessor::getDyn (int b) const
{
    DynSettings d;
    d.on = bandValue (b, P::fDyn) > 0.5f;
    d.thrDb = bandValue (b, P::fThr);
    d.rangeDb = bandValue (b, P::fRange);
    d.attackMs = bandValue (b, P::fAtt);
    d.releaseMs = bandValue (b, P::fRel);
    return d;
}

LinearPhase::Snapshot AudioLabEqProcessor::makeSnapshot (bool stereo) const
{
    LinearPhase::Snapshot s;
    s.stereo = stereo;
    for (int b = 0; b < maxBands; ++b)
    {
        s.bands[(size_t) b] = getBandSpec (b, true);
        if (s.bands[(size_t) b].active && hasGain (s.bands[(size_t) b].shape))
            s.bands[(size_t) b].gainDb += engine.getDynOffset (b);
    }
    return s;
}

// ── Audio ───────────────────────────────────────────────────────────────
void AudioLabEqProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    stereoLayout = getTotalNumInputChannels() > 1;
    engine.prepare (sampleRate);
    linear.prepare (sampleRate, std::min (chunk, std::max (32, samplesPerBlock)));
    const int maxLat = linear.latency();
    dryRingL.assign ((size_t) (maxLat + chunk + 1), 0.0f);
    dryRingR.assign ((size_t) (maxLat + chunk + 1), 0.0f);
    dryW = 0;
    for (auto* v : { &inL, &inR, &dlyL, &dlyR, &mono }) v->assign ((size_t) chunk, 0.0f);
    outSm.reset (sampleRate, 0.03);
    bypassSm.reset (sampleRate, 0.02);
    bypassSm.setCurrentAndTargetValue (getParam (P::bypass) > 0.5f ? 1.0f : 0.0f);
    specsValid = false;
    computeAutoGain();
    const float g = juce::Decibels::decibelsToGain (getParam (P::outGain) + (getParam (P::autoGain) > 0.5f ? autoGainDb.load() : 0.0f));
    outSm.setCurrentAndTargetValue (g);
    linearActive = isLinearPhase();
    currentLatency = linearActive ? linear.latency() : 0;
    setLatencySamples (currentLatency);
    if (designer != nullptr) designer->poke();
}

void AudioLabEqProcessor::updateEngineTargets (bool)
{
    const double sc = gainScale();
    for (int b = 0; b < maxBands; ++b) engine.setBand (b, getBandSpec (b, false), getDyn (b), sc);
}

static bool sameSpec (const BandSpec& a, const BandSpec& b)
{
    if (a.active != b.active) return false;
    if (! a.active) return true;
    return a.shape == b.shape && a.slope == b.slope && a.place == b.place
        && a.freq == b.freq && a.gainDb == b.gainDb && a.q == b.q;
}

void AudioLabEqProcessor::computeAutoGain()
{
    // Compensation = moyenne de la courbe statique (en dB) entre 40 Hz et 16 kHz, pondérée en log.
    // Recalculée seulement quand une bande change (aucune allocation : fil audio).
    std::array<BandSpec, maxBands> specs;
    bool same = specsValid;
    for (int b = 0; b < maxBands; ++b)
    {
        specs[(size_t) b] = getBandSpec (b, true);
        if (same && ! sameSpec (specs[(size_t) b], lastSpecs[(size_t) b])) same = false;
    }
    if (same) return;
    lastSpecs = specs;
    specsValid = true;
    const double fs = getSampleRateSafe();
    constexpr int np = 48;
    double sum = 0;
    Design d;
    for (const auto& s : specs)
    {
        if (! s.active) continue;
        design (s, fs, d, false);
        // Mid, Side, G, D : la bande ne touche qu'une partie du signal stéréo
        const double w = s.place == Place::stereo ? 1.0 : 0.5;
        for (int i = 0; i < np; ++i)
        {
            const double f = 40.0 * std::pow (16000.0 / 40.0, (i + 0.5) / np);
            sum += w * 20.0 * std::log10 (std::max (1e-6, std::abs (response (d, f, fs, false))));
        }
    }
    autoGainDb.store ((float) juce::jlimit (-24.0, 24.0, -sum / np));
}

void AudioLabEqProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    const int total = buffer.getNumSamples();
    const int nIn = getTotalNumInputChannels();
    for (int ch = nIn; ch < getTotalNumOutputChannels(); ++ch) buffer.clear (ch, 0, total);
    if (nIn == 0 || total == 0) return;
    const bool stereo = nIn > 1;
    stereoLayout = stereo;

    // Mode de phase (la latence change avec lui)
    const bool lin = isLinearPhase();
    if (lin != linearActive.load())
    {
        linearActive = lin;
        currentLatency = lin ? linear.latency() : 0;
        setLatencySamples (currentLatency);
        linear.reset();
        std::fill (dryRingL.begin(), dryRingL.end(), 0.0f);
        std::fill (dryRingR.begin(), dryRingR.end(), 0.0f);
        if (designer != nullptr) designer->poke();
    }

    updateEngineTargets (stereo);
    computeAutoGain();
    const float og = getParam (P::outGain) + (getParam (P::autoGain) > 0.5f ? autoGainDb.load() : 0.0f);
    outSm.setTargetValue (juce::Decibels::decibelsToGain (og));
    bypassSm.setTargetValue (getParam (P::bypass) > 0.5f ? 1.0f : 0.0f);

    float* L = buffer.getWritePointer (0);
    float* R = stereo ? buffer.getWritePointer (1) : nullptr;
    const int ringLen = (int) dryRingL.size();
    float pkL = 0, pkR = 0;

    for (int off = 0; off < total; off += chunk)
    {
        const int n = std::min (chunk, total - off);
        float* l = L + off;
        float* r = R != nullptr ? R + off : nullptr;

        // analyseur (entrée) et signal sec retardé de la latence
        for (int i = 0; i < n; ++i)
        {
            const float xl = l[i], xr = r != nullptr ? r[i] : xl;
            mono[(size_t) i] = 0.5f * (xl + xr);
            dryRingL[(size_t) dryW] = xl; dryRingR[(size_t) dryW] = xr;
            const int ri = (dryW - currentLatency + ringLen) % ringLen;
            dlyL[(size_t) i] = dryRingL[(size_t) ri]; dlyR[(size_t) i] = dryRingR[(size_t) ri];
            dryW = (dryW + 1) % ringLen;
        }
        tapPre.push (mono.data(), n);

        if (! lin) engine.process (l, r, n, false);
        else
        {
            engine.process (l, r, n, true);   // détecteurs dynamiques seulement
            linear.process (l, r, n);
            engine.solo (dlyL.data(), stereo ? dlyR.data() : nullptr, l, r, n);
        }

        for (int i = 0; i < n; ++i)
        {
            const float g = outSm.getNextValue(), b = bypassSm.getNextValue();
            float ol = l[i] * g, orr = r != nullptr ? r[i] * g : 0.0f;
            ol = ol * (1.0f - b) + dlyL[(size_t) i] * b;
            l[i] = ol;
            pkL = std::max (pkL, std::abs (ol));
            if (r != nullptr)
            {
                orr = orr * (1.0f - b) + dlyR[(size_t) i] * b;
                r[i] = orr;
                pkR = std::max (pkR, std::abs (orr));
            }
            mono[(size_t) i] = r != nullptr ? 0.5f * (ol + orr) : ol;
        }
        tapPost.push (mono.data(), n);
    }
    if (R == nullptr) pkR = pkL;
    peakL.store (std::max (peakL.load(), pkL));
    peakR.store (std::max (peakR.load(), pkR));
    lastBlockMs.store (juce::Time::getMillisecondCounter());
}

// ── Modifications depuis l'interface ────────────────────────────────────
void AudioLabEqProcessor::setValue (juce::RangedAudioParameter* p, float value)
{
    if (p == nullptr) return;
    const float n = p->convertTo0to1 (value);
    if (std::abs (p->getValue() - n) < 1e-7f) return;
    p->beginChangeGesture();
    p->setValueNotifyingHost (n);
    p->endChangeGesture();
}

int AudioLabEqProcessor::firstFreeBand() const
{
    for (int b = 0; b < maxBands; ++b) if (! isUsed (b)) return b;
    return -1;
}

int AudioLabEqProcessor::createBand (Shape shape, float freq, float gain, float q, int slope, Place place)
{
    const int b = firstFreeBand();
    if (b < 0) return -1;
    setBandValue (b, P::fShape, (float) (int) shape);
    setBandValue (b, P::fFreq, freq);
    setBandValue (b, P::fGain, gain);
    setBandValue (b, P::fQ, q);
    setBandValue (b, P::fSlope, (float) P::slopeIndex (slope));
    setBandValue (b, P::fPlace, (float) (int) place);
    setBandValue (b, P::fDyn, 0);
    setBandValue (b, P::fThr, -30);
    setBandValue (b, P::fRange, -6);
    setBandValue (b, P::fAtt, 10);
    setBandValue (b, P::fRel, 150);
    setBandValue (b, P::fOn, 1);
    setBandValue (b, P::fUsed, 1);
    return b;
}

void AudioLabEqProcessor::deleteBand (int b)
{
    if (b < 0 || b >= maxBands) return;
    if (soloBand == b) setSolo (-1);
    setBandValue (b, P::fUsed, 0);
    setBandValue (b, P::fDyn, 0);
}

void AudioLabEqProcessor::clearAllBands()
{
    for (int b = 0; b < maxBands; ++b) if (isUsed (b)) deleteBand (b);
}

void AudioLabEqProcessor::applyPreset (int index)
{
    const auto& list = P::presets();
    if (index < 0 || index >= (int) list.size()) return;
    clearAllBands();
    for (const auto& pb : list[(size_t) index].bands)
    {
        const int b = createBand (pb.shape, pb.freq, pb.gain, pb.q, pb.slope, pb.place);
        if (b >= 0 && pb.dyn)
        {
            setBandValue (b, P::fThr, pb.thr);
            setBandValue (b, P::fRange, pb.range);
            setBandValue (b, P::fDyn, 1);
        }
    }
    setValue (apvts.getParameter (P::outGain), 0.0f);
    setValue (apvts.getParameter (P::gainScale), 100.0f);
    ui.setProperty ("preset", index, nullptr);
}

// ── État complet (annuler, A/B) ─────────────────────────────────────────
std::vector<float> AudioLabEqProcessor::getStateVector() const
{
    std::vector<float> v;
    v.reserve (maxBands * P::numBandFields + 8);
    for (int b = 0; b < maxBands; ++b)
        for (int f = 0; f < P::numBandFields; ++f) v.push_back (bandParams[(size_t) b][(size_t) f]->getValue());
    for (auto* id : P::globalIds) v.push_back (apvts.getParameter (id)->getValue());
    return v;
}

void AudioLabEqProcessor::setStateVector (const std::vector<float>& v)
{
    if (v.size() != (size_t) (maxBands * P::numBandFields + (int) std::size (P::globalIds))) return;
    size_t k = 0;
    auto set = [&] (juce::RangedAudioParameter* p)
    {
        const float n = v[k++];
        if (std::abs (p->getValue() - n) < 1e-7f) return;
        p->beginChangeGesture(); p->setValueNotifyingHost (n); p->endChangeGesture();
    };
    for (int b = 0; b < maxBands; ++b)
        for (int f = 0; f < P::numBandFields; ++f) set (bandParams[(size_t) b][(size_t) f]);
    for (auto* id : P::globalIds) set (apvts.getParameter (id));
}

void AudioLabEqProcessor::pushUndo()
{
    auto s = getStateVector();
    if (! undoStack.empty() && undoStack.back() == s) return;
    undoStack.push_back (std::move (s));
    if (undoStack.size() > 100) undoStack.erase (undoStack.begin());
    redoStack.clear();
}

bool AudioLabEqProcessor::undo()
{
    auto cur = getStateVector();
    while (! undoStack.empty() && undoStack.back() == cur) undoStack.pop_back();
    if (undoStack.empty()) return false;
    redoStack.push_back (cur);
    setStateVector (undoStack.back());
    undoStack.pop_back();
    return true;
}

bool AudioLabEqProcessor::redo()
{
    if (redoStack.empty()) return false;
    undoStack.push_back (getStateVector());
    setStateVector (redoStack.back());
    redoStack.pop_back();
    return true;
}

static juce::String vecToString (const std::vector<float>& v)
{
    juce::StringArray a;
    for (float x : v) a.add (juce::String (x, 7));
    return a.joinIntoString (" ");
}

static std::vector<float> stringToVec (const juce::String& s)
{
    std::vector<float> v;
    for (auto& t : juce::StringArray::fromTokens (s, " ", "")) if (t.isNotEmpty()) v.push_back (t.getFloatValue());
    return v;
}

void AudioLabEqProcessor::switchAB (const juce::String& slot)
{
    const auto cur = currentAB();
    if (slot == cur) return;
    pushUndo();
    ui.setProperty ("slot" + cur, vecToString (getStateVector()), nullptr);
    const auto stored = ui.getProperty ("slot" + slot).toString();
    if (stored.isNotEmpty()) setStateVector (stringToVec (stored));
    ui.setProperty ("ab", slot, nullptr);
}

void AudioLabEqProcessor::copyABToOther()
{
    const auto other = currentAB() == "A" ? juce::String ("B") : juce::String ("A");
    ui.setProperty ("slot" + other, vecToString (getStateVector()), nullptr);
}

// ── Sauvegarde ──────────────────────────────────────────────────────────
void AudioLabEqProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    auto state = apvts.copyState();
    if (auto old = state.getChildWithName ("ui"); old.isValid()) state.removeChild (old, nullptr);
    state.appendChild (ui.createCopy(), nullptr);
    if (auto xml = state.createXml()) copyXmlToBinary (*xml, destData);
}

void AudioLabEqProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
    {
        auto state = juce::ValueTree::fromXml (*xml);
        if (! state.hasType (apvts.state.getType())) return;
        if (auto u = state.getChildWithName ("ui"); u.isValid())
        {
            ui.copyPropertiesAndChildrenFrom (u, nullptr);
            state.removeChild (u, nullptr);
        }
        // Projet v1.0 : Attaque / Retour étaient communs à toutes les bandes
        auto legacy = [&state] (const char* id) -> juce::var
        {
            for (auto c : state) if (c.getProperty ("id").toString() == id) return c.getProperty ("value");
            return {};
        };
        const auto oldAtt = legacy (P::legacyAttack), oldRel = legacy (P::legacyRelease);
        const bool hasNew = ! legacy (P::bandId (0, P::fAtt).toRawUTF8()).isVoid();
        apvts.replaceState (state);
        if (! hasNew && ! oldAtt.isVoid() && ! oldRel.isVoid())
            for (int b = 0; b < maxBands; ++b)
            {
                setBandValue (b, P::fAtt, (float) oldAtt);
                setBandValue (b, P::fRel, (float) oldRel);
            }
    }
}

juce::AudioProcessorEditor* AudioLabEqProcessor::createEditor() { return new AudioLabEqEditor (*this); }

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() { return new AudioLabEqProcessor(); }
