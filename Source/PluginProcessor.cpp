#include "PluginProcessor.h"
#include "PluginEditor.h"

using namespace audiolab;
using namespace audiolab::eq;
namespace P = audiolab::params;

// ── Fil de conception des FIR (modes Linear / Natural phase) ───────────
class AudioLabEqProcessor::Designer : public juce::Thread
{
public:
    explicit Designer (AudioLabEqProcessor& p) : juce::Thread ("AudioLabEQ FIR"), proc (p) {}
    void poke() { force = true; notify(); }

    void run() override
    {
        while (! threadShouldExit())
        {
            wait (25);
            proc.linear.collectGarbage();
            // (réglage à préparer : celui en place, ou celui qui arrive pendant un changement de mode)
            const int mode = proc.designMode.load();
            if (mode == P::phaseZero) { last.clear(); continue; }
            const int N = proc.designN.load();
            const auto snap = proc.makeSnapshot (proc.stereoLayout.load());
            const auto fm = mode == P::phaseNatural ? fir::Mode::natural : fir::Mode::linear;
            // Bandes dynamiques (3.0) : celles que le moteur traite (Natural) ne demandent aucune
            // reconception ; celles du mélange de FIR (Linear) seulement quand leurs réglages changent ;
            // les bandes en trop suivent leur gain du moment, comme en 2.0.
            const auto plan = proc.linear.planFor (snap, fm, N);
            std::vector<double> key;
            key.reserve (maxBands * 7 + 4);
            key.push_back (snap.stereo ? 1 : 0);
            key.push_back (mode);
            key.push_back (N);
            for (int i = 0; i < maxBands; ++i)
            {
                const auto& b = snap.bands[(size_t) i];
                const uint32_t bit = 1u << i;
                if (! b.active) { key.push_back (-1); continue; }
                if (plan.engineMask & bit) { key.push_back (-2); continue; }
                const double g = b.gainDb + ((plan.fallbackMask & bit) ? snap.dyn.now[(size_t) i] : 0.0);
                key.insert (key.end(), { (double) b.shape, (double) b.slope, (double) b.place, b.slopeFree, b.brick ? 1.0 : 0.0,   // (3.0 : pente libre, Brickwall)
                                         std::round (std::log (b.freq) * 2000.0), std::round (g * 10.0), std::round (std::log (b.q) * 500.0) });
                if (plan.mixMask & bit) { key.push_back (-3); key.push_back (std::round (snap.dyn.full[(size_t) i] * 10.0)); }
            }
            const bool display = proc.editorOpen.load();
            const bool forced = force.exchange (false);
            if (key != last || forced || (display && ! displayDone))
            {
                last = key;
                displayDone = display;
                proc.linear.design (snap, fm, N, display);
            }
            // résolution Auto : longueur vérifiée par la mesure (une fois par réglage)
            if (mode == P::phaseLinear && proc.linResIndex() == 0 && ! threadShouldExit()) proc.verifyAutoLength (snap);
        }
    }

private:
    AudioLabEqProcessor& proc;
    std::vector<double> last;
    std::atomic<bool> force { false };
    bool displayDone = false;
};

// ── Construction ────────────────────────────────────────────────────────
AudioLabEqProcessor::AudioLabEqProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput ("Input", juce::AudioChannelSet::stereo(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                          .withInput ("Side-chain", juce::AudioChannelSet::stereo(), false)),
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
    if (out != layouts.getMainInputChannelSet()) return false;
    // side-chain externe (EQ dynamique) : facultatif, mono ou stéréo
    if (layouts.inputBuses.size() > 1)
    {
        const auto sc = layouts.getChannelSet (true, 1);
        if (! sc.isDisabled() && sc != juce::AudioChannelSet::mono() && sc != juce::AudioChannelSet::stereo()) return false;
    }
    return true;
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
    // 3.0 : pente libre (0 = pente de la liste) et Brickwall, pour les coupes seulement
    s.slopeFree = isCut (s.shape) ? (double) bandValue (b, P::fSlopeFree) : 0.0;
    s.brick = isCut (s.shape) && bandValue (b, P::fBrick) > 0.5f;
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
    d.external = bandValue (b, P::fSc) > 0.5f;
    d.autoThr = bandValue (b, P::fThrAuto) > 0.5f;
    return d;
}

LinearPhase::Snapshot AudioLabEqProcessor::makeSnapshot (bool stereo) const
{
    LinearPhase::Snapshot s;
    s.stereo = stereo;
    // gains de base ; la dynamique est décrite à part (plage utile, décalage du moment) : la
    // conception décide qui la réalise (moteur, mélange de FIR, ou reconception pour les bandes en trop)
    for (int b = 0; b < maxBands; ++b)
    {
        auto& sp = s.bands[(size_t) b];
        sp = getBandSpec (b, true);
        const auto d = getDyn (b);
        if (sp.active && d.on && canBeDynamic (sp.shape))
        {
            s.dyn.full[(size_t) b] = d.rangeDb * gainScale();
            s.dyn.now[(size_t) b] = eng().getDynOffset (b);
        }
    }
    return s;
}

// ── Audio ───────────────────────────────────────────────────────────────
// Longueur du FIR voulue pour un mode. Linear phase en Auto : d'après la bande la plus grave et la
// plus étroite (gain de base + plage dynamique : l'EQ dynamique ne change jamais la latence) ; pendant
// la lecture, la longueur (donc la latence) en place est gardée, et ne change qu'à l'arrêt.
int AudioLabEqProcessor::desiredFirLength (int mode, bool playing) const
{
    if (mode == P::phaseZero) return 0;
    const double fs = getSampleRateSafe();
    std::array<BandSpec, maxBands> bands;
    std::array<double, maxBands> span {};
    for (int b = 0; b < maxBands; ++b)
    {
        bands[(size_t) b] = getBandSpec (b, true);
        const auto d = getDyn (b);
        if (d.on && canBeDynamic (bands[(size_t) b].shape)) span[(size_t) b] = d.rangeDb * gainScale();
    }
    if (mode == P::phaseNatural) return fir::chooseLength (fir::Res::automatic, bands, fs, fir::Mode::natural, &span);
    const auto res = (fir::Res) linResIndex();
    if (res != fir::Res::automatic) return fir::lengthFor (res, fs);
    const int cur = activeMode.load() == P::phaseLinear ? activeN.load() : 0;
    if (playing && cur > 0) return cur;
    double need = 0;
    for (int b = 0; b < maxBands; ++b) need = std::max (need, fir::requiredLength (bands[(size_t) b], fs, fir::Mode::linear, span[(size_t) b]));
    int n = fir::minLength (fs);
    while (n < fir::maxLength (fs) && n < need) n *= 2;
    // 3.0 : la longueur de la règle est vérifiée par la mesure (fil de conception) et allongée tant que
    // l'écart à la cible dépasse 0,1 dB. Tant que la vérification de ces réglages n'est pas arrivée,
    // on ne raccourcit pas (sinon la latence ferait un aller-retour à chaque retouche).
    const uint64_t key = autoKey (bands, span, stereoLayout.load(), fs), ver = autoVerified.load();
    if (ver != 0 && (ver & ~uint64_t (3)) == (key & ~uint64_t (3))) n = std::max (n, fir::minLength (fs) << (int) (ver & 3));
    else if (cur > n) n = cur;
    if (cur > 0 && n < cur && need > 0.4 * cur) n = cur;   // hystérésis : on ne raccourcit que nettement
    return n;
}

// Clé des réglages dont dépend la longueur Auto (bandes actives, plage dynamique, voies, fréquence d'échantillonnage)
uint64_t AudioLabEqProcessor::autoKey (const std::array<BandSpec, maxBands>& bands, const std::array<double, maxBands>& span, bool stereo, double fs)
{
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h] (double v)
    {
        uint64_t u; std::memcpy (&u, &v, sizeof u);
        for (int i = 0; i < 8; ++i) { h ^= (u >> (8 * i)) & 0xffu; h *= 1099511628211ull; }
    };
    mix (fs); mix (stereo ? 1.0 : 0.0);
    for (int b = 0; b < maxBands; ++b)
    {
        const auto& s = bands[(size_t) b];
        if (! s.active) { mix (-1.0); continue; }
        mix ((double) (int) s.shape); mix ((double) s.slope); mix ((double) (int) s.place); mix (s.freq); mix (s.gainDb); mix (s.q);
        mix (s.slopeFree); mix (s.brick ? 1.0 : 0.0); mix (fir::isDynamicSpan (s, span[(size_t) b]) ? span[(size_t) b] : 0.0);
    }
    return h;
}

// Fil de conception (ou prepareToPlay) : longueur Auto vérifiée pour ces réglages, publiée pour le fil audio
void AudioLabEqProcessor::verifyAutoLength (const LinearPhase::Snapshot& snap)
{
    const std::lock_guard<std::mutex> lock (verifyMutex);
    const double fs = getSampleRateSafe();
    const uint64_t key = autoKey (snap.bands, snap.dyn.full, snap.stereo, fs);
    if (key == lastVerifyKey && autoVerified.load() != 0) return;
    const int n = fir::verifiedAutoLength (snap.bands, snap.dyn.full, snap.stereo, fs, verifyWork, verifyTmp, verifyScratch);
    int idx = 0;
    for (int len = fir::minLength (fs); len < n && idx < 3; len *= 2) ++idx;
    autoVerified.store ((key & ~uint64_t (3)) | (uint64_t) idx);
    lastVerifyKey = key;
}

int AudioLabEqProcessor::latencyForMode (int mode) const
{
    if (mode == P::phaseZero) return 0;
    const int N = desiredFirLength (mode, false);
    return linear.latencyFor (mode == P::phaseNatural ? fir::Mode::natural : fir::Mode::linear, N);
}

// Réglage de départ (prepareToPlay) : latence annoncée, retards, FIR neutre en attendant la conception
void AudioLabEqProcessor::applySwitch (int mode, int N)
{
    activeMode = mode;
    activeN = N;
    designMode = mode; designN = N;
    curMode = mode; curN = N;
    currentLatency = mode == P::phaseZero ? 0 : linear.latencyFor (mode == P::phaseNatural ? fir::Mode::natural : fir::Mode::linear, N);
    setLatencySamples (currentLatency);
    latencyNow = currentLatency;
    // (le signal sec et l'historique du FIR sont tenus à jour en continu : rien à vider)
    if (mode != P::phaseZero) linear.switchTo (mode == P::phaseNatural ? fir::Mode::natural : fir::Mode::linear, N);
    if (designer != nullptr) designer->poke();
}

void AudioLabEqProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    juce::ignoreUnused (samplesPerBlock);
    stereoLayout = getMainBusNumInputChannels() > 1;
    for (auto& e : engines) { e.prepare (sampleRate); e.setSolo (soloBand); }
    engIdx = 0;
    linear.prepare (sampleRate);
    autoG.prepare (sampleRate);
    const int maxLat = fir::maxLength (sampleRate) / 2 + fir::partitionSize (sampleRate);
    for (auto* v : { &dryRingL, &dryRingR, &scRingL, &scRingR }) v->assign ((size_t) (maxLat + chunk + 1), 0.0f);
    dryW = 0;
    for (auto* v : { &dlyL, &dlyR, &detL, &detR, &scDL, &scDR, &mono, &oldL, &oldR, &fadeT,
                     &nwL, &nwR, &dl2L, &dl2R, &det2L, &det2R, &sc2L, &sc2R }) v->assign ((size_t) chunk, 0.0f);
    traceBuf.assign ((size_t) maxBands * (size_t) chunk, 0.0f);
    trace2.assign ((size_t) maxBands * (size_t) chunk, 0.0f);
    for (int b = 0; b < maxBands; ++b)
    {
        tracePtr[(size_t) b] = traceBuf.data() + (size_t) b * (size_t) chunk;
        tracePtr2[(size_t) b] = trace2.data() + (size_t) b * (size_t) chunk;
        for (auto& e : engines) e.dynTrace[(size_t) b] = nullptr;
    }
    outSm.reset (sampleRate, 0.03);
    bypassSm.reset (sampleRate, 0.02);
    bypassSm.setCurrentAndTargetValue (getParam (P::bypass) > 0.5f ? 1.0f : 0.0f);
    {
        // Auto gain : part de la prévision d'après la courbe (la mesure de sonie prend le relais dès qu'il y a du signal)
        std::array<BandSpec, maxBands> specs;
        for (int b = 0; b < maxBands; ++b) specs[(size_t) b] = getBandSpec (b, true);
        autoG.setCurve (specs, stereoLayout.load(), phaseMode() == P::phaseLinear);
        autoG.setRunning (getParam (P::autoGain) > 0.5f);
    }
    const float g = juce::Decibels::decibelsToGain (getParam (P::outGain) + (getParam (P::autoGain) > 0.5f ? autoG.gainDb() : 0.0f));
    outSm.setCurrentAndTargetValue (g);

    // réglage de départ, sans fondu ; en mode FIR, le FIR est conçu tout de suite (hors temps réel)
    activeMode = P::phaseZero; activeN = 0;
    const int mode = phaseMode();
    // (résolution Auto : longueur vérifiée par la mesure dès le départ)
    if (mode == P::phaseLinear && linResIndex() == 0) verifyAutoLength (makeSnapshot (stereoLayout.load()));
    const int N = desiredFirLength (mode, false);
    applySwitch (mode, N);
    if (mode != P::phaseZero)
    {
        const auto m = mode == P::phaseNatural ? fir::Mode::natural : fir::Mode::linear;
        linear.design (makeSnapshot (stereoLayout.load()), m, N, false);
        linear.switchTo (m, N);
    }
    sw = Sw::idle; switching = false;
}

void AudioLabEqProcessor::updateEngineTargets (bool)
{
    const double sc = gainScale();
    for (int b = 0; b < maxBands; ++b)
    {
        const auto spec = getBandSpec (b, false);
        const auto dyn = getDyn (b);
        for (auto& e : engines) e.setBand (b, spec, dyn, sc);
    }
}

// ── Pré-écho du FIR en place (mesuré par le fil de conception) ─────────
AudioLabEqProcessor::PreEcho AudioLabEqProcessor::getPreEcho() const
{
    const int m = activeMode.load();
    if (m == P::phaseZero) return {};
    return linear.getPreEcho (m == P::phaseNatural ? fir::Mode::natural : fir::Mode::linear, activeN.load());
}

juce::String AudioLabEqProcessor::getPreEchoText() const
{
    if (activeMode.load() == P::phaseZero) return {};
    return juce::String::fromUTF8 (fir::preEchoText (getPreEcho()).c_str());
}

// ── Changement de mode de phase ou de longueur : les deux chemins en parallèle ──
// A = chemin en place (curMode, currentLatency, chaîne eng()), B = chemin qui arrive (pendMode, pendLat,
// l'autre chaîne). En entrée, l / r portent le signal sec du tronçon ; dlyL / dlyR, detL / detR,
// scDL / scDR sont déjà prêts pour A. En sortie, l / r et dlyL / dlyR portent le mélange des deux chemins.
void AudioLabEqProcessor::renderDual (float* l, float* r, int n, const float* scL, const float* scR, bool stereo, uint32_t dynMask)
{
    auto& eA = eng();
    auto& eB = engines[(size_t) (1 - engIdx.load (std::memory_order_relaxed))];
    const int mA = curMode, mB = pendMode;
    const bool firA = mA != P::phaseZero, firB = mB != P::phaseZero;
    const bool linA = mA == P::phaseLinear, linB = mB == P::phaseLinear;
    const bool natA = mA == P::phaseNatural, natB = mB == P::phaseNatural;
    const int ringLen = (int) dryRingL.size();
    const int w0 = (dryW - n + ringLen) % ringLen;
    const int lagB = ! firB ? 0 : linB ? std::max (0, pendLat - linear.partition()) : pendLat;

    // signal sec retardé de la latence de B, entrée et side-chain de son détecteur, copie de l'entrée pour B
    for (int i = 0; i < n; ++i)
    {
        const int w = (w0 + i) % ringLen;
        const int ri = (w - pendLat + ringLen) % ringLen;
        dl2L[(size_t) i] = dryRingL[(size_t) ri]; dl2R[(size_t) i] = dryRingR[(size_t) ri];
        if (firB)
        {
            const int di = (w - lagB + ringLen) % ringLen;
            det2L[(size_t) i] = dryRingL[(size_t) di]; det2R[(size_t) i] = dryRingR[(size_t) di];
            sc2L[(size_t) i] = scRingL[(size_t) di]; sc2R[(size_t) i] = scRingR[(size_t) di];
        }
        nwL[(size_t) i] = l[i]; nwR[(size_t) i] = r != nullptr ? r[i] : l[i];
    }
    float* bl = nwL.data();
    float* br = stereo ? nwR.data() : nullptr;
    for (int b = 0; b < maxBands; ++b)
    {
        const bool tr = linB && ((dynMask >> b) & 1u) != 0;
        float* row = trace2.data() + (size_t) b * (size_t) chunk;
        eB.dynTrace[(size_t) b] = tr ? row : nullptr;
        if (tr) std::fill (row, row + n, 0.0f);
    }
    const float* aScL = scL != nullptr ? scDL.data() : nullptr;
    const float* aScR = scR != nullptr ? scDR.data() : nullptr;
    const float* bScL = scL != nullptr ? sc2L.data() : nullptr;
    const float* bScR = scR != nullptr ? sc2R.data() : nullptr;

    // Linear phase : détecteurs seuls, sur l'entrée retardée de chaque chemin
    if (linA) eA.process (detL.data(), stereo ? detR.data() : nullptr, n, true, aScL, aScR);
    if (linB) eB.process (det2L.data(), stereo ? det2R.data() : nullptr, n, true, bScL, bScR);
    // zéro latence : la chaîne IIR
    if (! firA) eA.process (l, r, n, false, scL, scR);
    if (! firB) eB.process (bl, br, n, false, scL, scR);

    // FIR : par tronçons qui s'arrêtent aux blocs de partition (le partage FIR / moteur y est constant)
    if (firA || firB)
    {
        std::array<const float*, maxBands> trA {}, trB {};
        for (int o = 0; o < n;)
        {
            const auto st = linear.outState();
            const int m = std::min (n - o, linear.untilBoundary());
            for (int b = 0; b < maxBands; ++b) { trA[(size_t) b] = tracePtr[(size_t) b] + o; trB[(size_t) b] = tracePtr2[(size_t) b] + o; }
            float* ar = r != nullptr ? r + o : nullptr;
            float* brr = br != nullptr ? br + o : nullptr;
            Engine::FirSplit sp;
            if (firA && firB)
            {
                // le nouveau jeu sur place dans B, l'ancien rendu à part dans A (l / r)
                linear.process (bl + o, brr, m, linB ? trB.data() : nullptr, linB ? dynMask : 0u, l + o, ar, fadeT.data() + o,
                                linA ? trA.data() : nullptr, linA ? dynMask : 0u);
                if (natA) { sp.cur = st.old; sp.old = st.old; eA.process (l + o, ar, m, false, aScL != nullptr ? aScL + o : nullptr, aScR != nullptr ? aScR + o : nullptr, &sp); }
                if (natB) { sp.cur = st.cur; sp.old = st.cur; eB.process (bl + o, brr, m, false, bScL != nullptr ? bScL + o : nullptr, bScR != nullptr ? bScR + o : nullptr, &sp); }
            }
            else if (firB)
            {
                // zéro latence → FIR : seul le nouveau jeu existe
                linear.process (bl + o, brr, m, linB ? trB.data() : nullptr, linB ? dynMask : 0u);
                if (natB) { sp.cur = st.cur; sp.old = st.cur; eB.process (bl + o, brr, m, false, bScL != nullptr ? bScL + o : nullptr, bScR != nullptr ? bScR + o : nullptr, &sp); }
            }
            else
            {
                // FIR → zéro latence : le jeu en place continue comme en régime établi
                if (linA) linear.process (l + o, ar, m, trA.data(), dynMask);
                else
                {
                    linear.process (l + o, ar, m, nullptr, 0, oldL.data() + o, stereo ? oldR.data() + o : nullptr, fadeT.data() + o);
                    sp.cur = st.cur; sp.old = st.old;
                    if (st.split) { sp.oldL = oldL.data() + o; sp.oldR = stereo ? oldR.data() + o : nullptr; sp.t = fadeT.data() + o; }
                    eA.process (l + o, ar, m, false, aScL != nullptr ? aScL + o : nullptr, aScR != nullptr ? aScR + o : nullptr, &sp);
                }
            }
            o += m;
        }
    }
    if (firA) eA.solo (dlyL.data(), stereo ? dlyR.data() : nullptr, l, r, n);
    if (firB) eB.solo (dl2L.data(), stereo ? dl2R.data() : nullptr, bl, br, n);

    if (sw == Sw::preRoll)
    {
        // le nouveau chemin tourne sans être entendu ; corrélation des deux sorties sur les 40 dernières
        // millisecondes de la mise en route, en deux fenêtres de 20 ms
        for (int i = std::max (0, swLen - 2 * corrLen - swCount); i < n; ++i)
        {
            auto& c = corr[swCount + i < swLen - corrLen ? 0 : 1];
            const double a = l[i], b = bl[i];
            c.xx += a * a; c.yy += b * b; c.xy += a * b;
            if (r != nullptr && br != nullptr) { const double e = r[i], d = br[i]; c.xx += e * e; c.yy += d * d; c.xy += e * d; }
        }
        return;
    }
    // Fondu enchaîné à puissance constante pour la corrélation mesurée ρ (bornée à 0…1) :
    // sortie = a·ancien + b·nouveau, avec a² + b² + 2ab·ρ = 1 (ρ = 1 : fondu linéaire ; ρ = 0 : sinus / cosinus)
    const double rho = swRho, halfPi = 0.5 * juce::MathConstants<double>::pi;
    for (int i = 0; i < n; ++i)
    {
        const double u = (double) (swCount + i + 1) / (double) swLen, h = u * u * (3.0 - 2.0 * u);
        const double b = std::sin (halfPi * h);
        const double a = std::max (0.0, -b * rho + std::sqrt (std::max (0.0, 1.0 - b * b * (1.0 - rho * rho))));
        const float fa = (float) a, fb = (float) b;
        l[i] = fa * l[i] + fb * bl[i];
        dlyL[(size_t) i] = fa * dlyL[(size_t) i] + fb * dl2L[(size_t) i];
        if (r != nullptr)
        {
            r[i] = fa * r[i] + fb * nwR[(size_t) i];
            dlyR[(size_t) i] = fa * dlyR[(size_t) i] + fb * dl2R[(size_t) i];
        }
    }
}

// Fin du fondu : le nouveau chemin devient le chemin en place
void AudioLabEqProcessor::finishSwitch()
{
    if (curMode != P::phaseZero && pendMode != P::phaseZero) linear.endSwitch();
    for (auto& e : engines) for (auto& t : e.dynTrace) t = nullptr;
    engIdx = 1 - engIdx.load();
    curMode = pendMode; curN = pendN; currentLatency = pendLat;
    sw = Sw::idle; switching = false;
}

void AudioLabEqProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    const int total = buffer.getNumSamples();
    const int nIn = getMainBusNumInputChannels();
    const int nOut = getMainBusNumOutputChannels();
    for (int ch = nIn; ch < nOut; ++ch) buffer.clear (ch, 0, total);
    if (nIn == 0 || total == 0) return;
    const bool stereo = nIn > 1;
    stereoLayout = stereo;

    // side-chain externe : bus activé par l'hôte (sinon : détection interne)
    const float* scInL = nullptr;
    const float* scInR = nullptr;
    if (auto* scBus = getBus (true, 1); scBus != nullptr && scBus->isEnabled() && scBus->getNumberOfChannels() > 0)
    {
        auto scBuf = getBusBuffer (buffer, true, 1);
        if (scBuf.getNumChannels() > 0)
        {
            scInL = scBuf.getReadPointer (0);
            scInR = scBuf.getNumChannels() > 1 ? scBuf.getReadPointer (1) : nullptr;
        }
    }
    scConnected = scInL != nullptr;

    // lecture en cours ? (la latence du mode Auto ne change qu'à l'arrêt)
    bool playing = false;
    if (testPlaying.load() >= 0) playing = testPlaying.load() > 0;
    else if (auto* ph = getPlayHead())
        if (auto pos = ph->getPosition()) playing = pos->getIsPlaying();

    // Mode de phase / longueur du FIR. Un changement de latence se fait sans silence (3.0) : le chemin en
    // place continue pendant que le fil de conception prépare le FIR ; puis le nouveau chemin tourne en
    // parallèle (mise en route), et un fondu enchaîné de 20 ms passe de l'un à l'autre.
    const double fs = getSampleRateSafe();
    const int mode = phaseMode();
    const int wantN = desiredFirLength (mode, playing);
    auto request = [this] (int m, int N)
    {
        pendMode = m; pendN = N; swCount = 0;
        swDesigns = linear.designsDone();
        designMode = m; designN = N;
        if (designer != nullptr) designer->poke();
    };
    if (sw == Sw::idle)
    {
        const bool latencyChange = mode != activeMode.load() || (mode == P::phaseLinear && wantN != activeN.load());
        if (latencyChange) { sw = Sw::waitDesign; switching = true; request (mode, wantN); }
        else if (mode == P::phaseNatural && wantN != activeN.load())
        { activeN = wantN; designN = wantN; curN = wantN; }   // Natural phase : la longueur change sans toucher à la latence (fondu entre FIR)
    }
    else if (sw == Sw::waitDesign)
    {
        if (mode == curMode && (mode != P::phaseLinear || wantN == curN))
        {
            // demande annulée avant le fondu : on reste sur le chemin en place
            sw = Sw::idle; switching = false;
            designMode = curMode; designN = curN;
            if (designer != nullptr) designer->poke();
        }
        else if (mode != pendMode || (mode != P::phaseZero && wantN != pendN)) request (mode, wantN);
    }
    if (sw == Sw::waitDesign)
    {
        const bool toZero = pendMode == P::phaseZero;
        const auto fm = pendMode == P::phaseNatural ? fir::Mode::natural : fir::Mode::linear;
        // FIR prêt : un jeu conçu depuis la demande, pour le réglage demandé (au plus 0,5 s d'attente,
        // après quoi on part avec un FIR neutre, remplacé par fondu dès que le vrai arrive)
        const bool ready = toZero || (linear.designsDone() != swDesigns && linear.designReady (fm, pendN));
        if ((ready && ! linear.fading()) || swCount > juce::roundToInt (0.5 * fs))
        {
            auto& nb = engines[(size_t) (1 - engIdx.load())];
            nb.prepare (fs);
            nb.seedDetectors (eng());
            nb.setSolo (soloBand);
            pendLat = toZero ? 0 : linear.latencyFor (fm, pendN);
            if (! toZero) linear.beginSwitch (fm, pendN, curMode != P::phaseZero);
            sw = Sw::preRoll; swCount = 0;
            // mise en route : un bloc de partition (première sortie du nouveau FIR), 20 ms pour que les
            // filtres du nouveau chemin se posent, puis 40 ms de mesure de la corrélation des deux sorties
            swLen = linear.partition() + juce::roundToInt (0.06 * fs);
            corrLen = juce::roundToInt (0.02 * fs);
            corr[0] = corr[1] = Corr(); swRho = 0.0;
        }
        else swCount += total;
    }

    updateEngineTargets (stereo);
    // Auto gain à sonie égale : courbe en place (décalage immédiat du gain quand une bande bouge), mesure plus bas
    const bool agOn = getParam (P::autoGain) > 0.5f;
    {
        std::array<BandSpec, maxBands> specs;
        for (int b = 0; b < maxBands; ++b) specs[(size_t) b] = getBandSpec (b, true);
        // (modes FIR : le nouveau FIR n'est entendu qu'après sa conception, la mesure attend)
        if (autoG.setCurve (specs, stereo, curMode == P::phaseLinear) && curMode != P::phaseZero) autoG.hold (0.5);
        autoG.setRunning (agOn);
    }
    bypassSm.setTargetValue (getParam (P::bypass) > 0.5f ? 1.0f : 0.0f);

    float* L = buffer.getWritePointer (0);
    float* R = stereo ? buffer.getWritePointer (1) : nullptr;
    const int ringLen = (int) dryRingL.size();
    float pkL = 0, pkR = 0;
    // bandes dont la dynamique est en service (Linear phase : leur gain est suivi à chaque échantillon)
    uint32_t dynMask = 0;
    for (int b = 0; b < maxBands; ++b)
        if (bandValue (b, P::fDyn) > 0.5f && bandValue (b, P::fUsed) > 0.5f && bandValue (b, P::fOn) > 0.5f) dynMask |= 1u << b;

    for (int off = 0; off < total;)
    {
        const bool dual = sw == Sw::preRoll || sw == Sw::fade;
        int n = std::min (chunk, total - off);
        if (dual) n = std::min (n, swLen - swCount);   // (les étapes du changement de mode tombent sur un tronçon)
        auto& engine = eng();
        float* l = L + off;
        float* r = R != nullptr ? R + off : nullptr;
        const int am = curMode;
        const bool isFir = am != P::phaseZero, linPhase = am == P::phaseLinear;
        // Linear phase : le détecteur lit l'entrée retardée de la latence moins un bloc de partition,
        // son gain est appliqué par le mélange de FIR un bloc plus tard, sur le bon échantillon.
        // Natural phase : le moteur traite la sortie du FIR ; seul le side-chain externe est retardé, de la latence.
        const int detLag = ! isFir ? 0 : linPhase ? std::max (0, currentLatency - linear.partition()) : currentLatency;
        for (int b = 0; b < maxBands; ++b)
        {
            const bool tr = linPhase && ((dynMask >> b) & 1u) != 0;
            float* row = traceBuf.data() + (size_t) b * (size_t) chunk;
            engine.dynTrace[(size_t) b] = tr ? row : nullptr;
            if (tr) std::fill (row, row + n, 0.0f);
        }

        // analyseur (entrée), signal sec retardé de la latence, entrée et side-chain du détecteur
        for (int i = 0; i < n; ++i)
        {
            const float xl = l[i], xr = r != nullptr ? r[i] : xl;
            mono[(size_t) i] = 0.5f * (xl + xr);
            dryRingL[(size_t) dryW] = xl; dryRingR[(size_t) dryW] = xr;
            if (scInL != nullptr)
            {
                scRingL[(size_t) dryW] = scInL[off + i];
                scRingR[(size_t) dryW] = scInR != nullptr ? scInR[off + i] : scInL[off + i];
            }
            const int ri = (dryW - currentLatency + ringLen) % ringLen;
            dlyL[(size_t) i] = dryRingL[(size_t) ri]; dlyR[(size_t) i] = dryRingR[(size_t) ri];
            if (isFir)
            {
                const int di = (dryW - detLag + ringLen) % ringLen;
                detL[(size_t) i] = dryRingL[(size_t) di]; detR[(size_t) i] = dryRingR[(size_t) di];
                scDL[(size_t) i] = scRingL[(size_t) di]; scDR[(size_t) i] = scRingR[(size_t) di];
            }
            dryW = (dryW + 1) % ringLen;
        }
        tapPre.push (mono.data(), n);

        if (dual)
            renderDual (l, r, n, scInL != nullptr ? scInL + off : nullptr, scInR != nullptr ? scInR + off : nullptr, stereo, dynMask);
        else if (! isFir)
        {
            linear.feed (l, r, n);   // historique du FIR à jour : passage en phase linéaire sans attente
            engine.process (l, r, n, false, scInL != nullptr ? scInL + off : nullptr, scInR != nullptr ? scInR + off : nullptr);
        }
        else if (linPhase)
        {
            // détecteurs dynamiques seulement (sur des copies), puis le FIR et le mélange des FIR d'écart
            engine.process (detL.data(), stereo ? detR.data() : nullptr, n, true,
                            scInL != nullptr ? scDL.data() : nullptr, scInR != nullptr ? scDR.data() : nullptr);
            linear.process (l, r, n, tracePtr.data(), dynMask);
            engine.solo (dlyL.data(), stereo ? dlyR.data() : nullptr, l, r, n);
        }
        else
        {
            // Natural phase : FIR des bandes fixes, puis les bandes dynamiques par le moteur zéro latence.
            // Par tronçons qui s'arrêtent aux blocs de partition : le partage FIR / moteur y est constant.
            for (int o = 0; o < n;)
            {
                const auto st = linear.outState();
                const int m = std::min (n - o, linear.untilBoundary());
                float* rr = r != nullptr ? r + o : nullptr;
                linear.process (l + o, rr, m, nullptr, 0, oldL.data() + o, stereo ? oldR.data() + o : nullptr, fadeT.data() + o);
                Engine::FirSplit sp;
                sp.cur = st.cur; sp.old = st.old;
                if (st.split) { sp.oldL = oldL.data() + o; sp.oldR = stereo ? oldR.data() + o : nullptr; sp.t = fadeT.data() + o; }
                engine.process (l + o, rr, m, false, scInL != nullptr ? scDL.data() + o : nullptr,
                                scInR != nullptr ? scDR.data() + o : nullptr, &sp);
                o += m;
            }
            engine.solo (dlyL.data(), stereo ? dlyR.data() : nullptr, l, r, n);
        }

        // Auto gain : sonie de l'entrée (recalée sur la sortie) et de la sortie avant gain ; la mesure est
        // suspendue pendant un changement de mode et pendant l'écoute solo
        if (agOn)
        {
            if (dual || engine.soloActive()) autoG.hold (0.2);
            autoG.process (dlyL.data(), stereo ? dlyR.data() : nullptr, l, r, n);
        }
        const float og = getParam (P::outGain) + (agOn ? autoG.gainDb() : 0.0f);
        outSm.setTargetValue (juce::Decibels::decibelsToGain (og));

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
        if (dual)
        {
            swCount += n;
            if (swCount >= swLen)
            {
                if (sw == Sw::preRoll)
                {
                    // début du fondu : c'est maintenant que la nouvelle latence est annoncée à l'hôte
                    // Corrélation retenue : la plus petite des deux fenêtres, bornée à 0…1. Un son tenu (note
                    // de basse, sinus) donne deux fois la même valeur : le fondu est alors réglé pour lui.
                    // Un bruit donne deux valeurs au hasard : on revient vers le fondu à puissance constante.
                    auto rhoOf = [] (const Corr& c) { return c.xx > 1.0e-12 && c.yy > 1.0e-12 ? c.xy / std::sqrt (c.xx * c.yy) : 0.0; };
                    const int law = testFadeLaw.load();
                    swRho = law == 1 ? 0.0 : law == 2 ? 1.0 : law == 3 ? juce::jlimit (0.0, 1.0, rhoOf (corr[1]))
                                                                       : juce::jlimit (0.0, 1.0, std::min (rhoOf (corr[0]), rhoOf (corr[1])));
                    sw = Sw::fade; swCount = 0; swLen = juce::roundToInt (0.02 * fs);
                    activeMode = pendMode; activeN = pendN;
                    setLatencySamples (pendLat);
                    latencyNow = pendLat;
                }
                else finishSwitch();
            }
        }
        off += n;
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
    setBandValue (b, P::fSc, 0);
    setBandValue (b, P::fSlopeFree, 0);
    setBandValue (b, P::fBrick, 0);
    setBandValue (b, P::fThrAuto, 0);
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

void AudioLabEqProcessor::setStateVector (const std::vector<float>& vin)
{
    const size_t nGlobals = std::size (P::globalIds);
    std::vector<float> v = vin;
    // États A/B ou « annuler » aux anciens formats : 1.x (13 champs par bande, 4 réglages généraux) et
    // 2.0 (14 champs, 5 réglages généraux). Les champs ajoutés depuis prennent leur valeur neutre
    // (side-chain interne, pente de la liste, Brickwall et seuil automatique coupés) ; un réglage général
    // absent garde sa valeur actuelle.
    int oldFields = 0, oldGlobals = 0;
    if (v.size() == (size_t) (maxBands * P::legacyBandFields + P::legacyGlobals)) { oldFields = P::legacyBandFields; oldGlobals = P::legacyGlobals; }
    else if (v.size() == (size_t) (maxBands * P::v2BandFields) + nGlobals)        { oldFields = P::v2BandFields; oldGlobals = (int) nGlobals; }
    if (oldFields > 0)
    {
        v.clear();
        size_t k = 0;
        for (int b = 0; b < maxBands; ++b)
        {
            for (int f = 0; f < oldFields; ++f) v.push_back (vin[k++]);
            for (int f = oldFields; f < P::numBandFields; ++f) v.push_back (bandParams[(size_t) b][(size_t) f]->getDefaultValue());
        }
        // (le paramètre « shape » a gagné deux choix en 3.0 : sa valeur normalisée est ramenée à la nouvelle échelle)
        const int shapeField = fieldIndex (P::fShape);
        for (int b = 0; b < maxBands; ++b)
        {
            float& x = v[(size_t) (b * P::numBandFields + shapeField)];
            x = (float) juce::roundToInt (x * (float) (numLegacyShapes - 1)) / (float) (numShapes - 1);
        }
        for (int g = 0; g < oldGlobals; ++g) v.push_back (vin[k++]);
        for (size_t g = (size_t) oldGlobals; g < nGlobals; ++g) v.push_back (apvts.getParameter (P::globalIds[g])->getValue());
    }
    if (v.size() != (size_t) (maxBands * P::numBandFields) + nGlobals) return;
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
        const bool hasRes = ! legacy (P::linRes).isVoid();
        // Projet 1.x ou 2.0 : les paramètres de la 3.0 n'y sont pas (noté avant le remplacement de l'état)
        std::array<std::array<bool, 3>, maxBands> has3 {};
        const char* const fields3[] = { P::fSlopeFree, P::fBrick, P::fThrAuto };
        for (int b = 0; b < maxBands; ++b)
            for (size_t i = 0; i < 3; ++i) has3[(size_t) b][i] = ! legacy (P::bandId (b, fields3[i]).toRawUTF8()).isVoid();
        apvts.replaceState (state);
        // … ils sont remis explicitement à leur valeur neutre : mêmes pentes, pas de Brickwall, seuil manuel
        for (int b = 0; b < maxBands; ++b)
            for (size_t i = 0; i < 3; ++i)
                if (! has3[(size_t) b][i])
                    if (auto* prm = bandParam (b, fields3[i])) prm->setValueNotifyingHost (prm->getDefaultValue());
        // Projet 1.x : la phase linéaire avait une seule longueur (8192 points à 48 kHz) = Medium
        if (! hasRes) setValue (apvts.getParameter (P::linRes), (float) P::resMediumIndex);
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
