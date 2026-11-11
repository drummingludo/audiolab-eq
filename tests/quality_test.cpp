// Banc qualité de Lab EQ 3.0, SANS JUCE (moteur seul) :
//   g++ -O2 -std=c++17 -I Source tests/quality_test.cpp -o qt && ./qt
// Mesures chiffrées avec seuils ; renvoie 1 si un seuil n'est pas tenu.
//  1. Zéro latence : écart à l'analogique de 20 Hz à 20 kHz, 44,1 / 48 / 96 / 192 kHz
//     (bilinéaire 1.x → matched 2.0 → matched + sections de correction 3.0), seuil 0,1 dB partout.
//     Convention : cloches / étagères / tilt sur tout le spectre ; coupes et passe-bande dans la zone
//     utile (là où l'analogique est au-dessus de −24 dB) ; notch au-dessus de −3 dB.
//  2. Réponse mesurée du moteur = courbe affichée (impulsion → FFT), y compris après un balayage
//  3. Même courbe d'une fréquence d'échantillonnage à l'autre (seuil 0,05 dB)
//  4. Tests de nul (bandes à 0 dB : sortie = entrée, bit à bit)
//  5. EQ dynamique : raies parasites sur un sinus avec attaque rapide (avant / après)
//  6. Balayage rapide d'un réglage : bruit d'escalier (avant / après), grave et aigu
//  7. Phase linéaire (Auto, 48 kHz) : cloche 50 Hz −12 dB Q 8, notch 60 Hz Q 10, coupe-bas 30 Hz 48 dB/oct (avant = 1.x)
//  8. Natural phase : module et phase = analogique
//  9. Latences annoncées
// 10. Side-chain externe
// 11. Stabilité (réglages aléatoires)
// 12. EQ dynamique : résidu par rapport à la référence idéale (conception exacte à chaque échantillon), 10 cas
// 13. Cache des bandes dynamiques : précision de l'interpolation en gain, cellules irrégulières
// 14. Bande dynamique ET automatisée en même temps ; sortie indépendante de la taille des blocs de l'hôte
// 15. Coût CPU du moteur (information, sans seuil)
// 16. Sections de correction : coût (conception, sections par bande, traitement), bande dynamique dans l'aigu
// Modes FIR avec bandes dynamiques (3.0), sur le code du plugin (LinearPhaseCore.h, FirBank.h, moteur) :
// 17. Linear phase : résidu temporel d'une bande dynamique par rapport à l'idéal (avant / après)
// 18. Linear phase : erreur de courbe du mélange de FIR (FIR réels, puis toutes formes et plages)
// 19. Natural phase : bande dynamique faite par le moteur après le FIR, précision temporelle
// 20. Bandes fixes inchangées dans les deux modes
// 21. Linear phase : bandes dynamiques qui se recouvrent (paire, chaîne) ; répartition des bandes
// 22. Passage dyn on / off d'une bande pendant la lecture : transitoire
// 23. Coût des modes FIR avec 0, 1, 4 bandes dynamiques (FIR 8192 et 32768)
// Formes et pentes 3.0 (« ./qt 3.0 » ne lance que ces sections) :
// 24. Flat tilt : écart à la droite idéale (0,1 dB), symétrie +g / −g (0,01 dB), moteur, FIR
// 25. All pass : module (±0,001 dB), phase à la fréquence de la bande, neutre en Linear phase
// 26. Pentes libres : écart au Butterworth d'ordre fractionnaire (0,1 dB au-dessus de −24 dB), pente moyenne
// 27. Brickwall : ondulation (0,1 dB), atténuation (−96 dB à un tiers d'octave), mur des FIR selon la résolution
// 28. Seuil dynamique automatique : repos, dépassement, retour, déterminisme
// 29. Stabilité avec les nouvelles formes et pentes, coût CPU
// Auto gain, pré-écho, résolution Auto (« ./qt gain » ne lance que ces sections) :
// 30. Mesureur de sonie ITU-R BS.1770-4 : sinus de référence à 44,1 / 48 / 96 / 192 kHz, courbe K, portes
// 31. Auto gain à sonie égale : 6 réglages sur du bruit rose, écart de sonie sortie / entrée (avant / après), convergence
// 32. Auto gain : silence, mouvement d'une bande, bornes, coût
// 33. Pré-écho des FIR : cas calculable, valeurs annoncées contre la réponse impulsionnelle mesurée
// 34. Résolution Auto : écart réel du FIR à la cible dans le grave, aux quatre fréquences d'échantillonnage
#include "EqEngine.h"
#include "FirDesign.h"
#include "LinearPhaseCore.h"
#include "AutoGain.h"
#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace audiolab::eq;
static int failures = 0;
#define CHECK(cond, ...) do { if (! (cond)) { ++failures; std::printf ("  ÉCHEC : "); std::printf (__VA_ARGS__); std::printf ("\n"); } } while (0)

static double db (double x) { return 20.0 * std::log10 (std::max (1e-15, x)); }

// écart max (dB) entre la conception numérique et le prototype analogique, 20 Hz – 20 kHz
// mode : 0 = bilinéaire (1.x), 1 = matched sans correction (2.0), 2 = conception actuelle (3.0)
static double errOf (const BandSpec& b, double fs, const Design& dd, double floorDb, double* atF = nullptr, int npts = 1200)
{
    Design da;
    design (b, fs, da, true);
    double e = 0;
    for (int i = 0; i < npts; ++i)
    {
        const double f = 20.0 * std::pow (1000.0, i / (npts - 1.0));
        const double ra = magnitudeDb (da, f, fs, true);
        if (ra < floorDb) continue;
        const double x = std::abs (magnitudeDb (dd, f, fs, false) - ra);
        if (x > e) { e = x; if (atF) *atF = f; }
    }
    return e;
}

static double errVsAnalog (const BandSpec& b, double fs, int mode, double floorDb, double* atF = nullptr, int* nsec = nullptr)
{
    Design dd, da;
    if (mode == 0) designBilinear (b, fs, dd); else if (mode == 1) designMatched (b, fs, dd, 0); else design (b, fs, dd, false);
    if (nsec) *nsec = dd.n;
    design (b, fs, da, true);
    double e = 0;
    for (int i = 0; i < 1200; ++i)
    {
        const double f = 20.0 * std::pow (1000.0, i / 1199.0);
        const double ra = magnitudeDb (da, f, fs, true);
        if (ra < floorDb) continue;
        const double x = std::abs (magnitudeDb (dd, f, fs, false) - ra);
        if (x > e) { e = x; if (atF) *atF = f; }
    }
    return e;
}

static void fftReal (const std::vector<double>& x, std::vector<fir::cd>& X, int n)
{
    X.assign ((size_t) n, fir::cd (0, 0));
    for (int i = 0; i < n && i < (int) x.size(); ++i) X[(size_t) i] = x[(size_t) i];
    fir::fft (X.data(), n, false);
}

// fenêtre de Blackman-Harris 4 termes (mesures spectrales)
static double bh (int i, int n)
{
    const double x = 2.0 * pi * i / n;
    return 0.35875 - 0.48829 * std::cos (x) + 0.14128 * std::cos (2 * x) - 0.01168 * std::cos (3 * x);
}

// Raie la plus forte à f0 ± k·fs/32 (k = 1…6), en dB sous la fondamentale : signature des paliers de 32 éch.
static double blockLines (const std::vector<float>& y, int start, int n, double fs, double f0)
{
    std::vector<double> x ((size_t) n);
    for (int i = 0; i < n; ++i) x[(size_t) i] = y[(size_t) (start + i)] * bh (i, n);
    std::vector<fir::cd> X;
    fftReal (x, X, n);
    auto peakNear = [&] (double f) { const int k0 = (int) std::lround (f * n / fs); double m = 0; for (int k = k0 - 4; k <= k0 + 4; ++k) m = std::max (m, std::abs (X[(size_t) k])); return m; };
    const double fund = peakNear (f0);
    double worst = 0;
    for (int k = -6; k <= 6; ++k)
    {
        if (k == 0) continue;
        const double f = std::abs (f0 + k * fs / 32.0);
        if (f < 50 || f > 0.5 * fs - 50) continue;
        bool nearHarm = false;   // (ne pas confondre avec une harmonique du sinus)
        for (int h = 1; h * f0 < 0.5 * fs; ++h) if (std::abs (f - h * f0) < 60) nearHarm = true;
        if (! nearHarm) worst = std::max (worst, peakNear (f));
    }
    return db (worst / fund);
}

// Bruit large bande loin de la fondamentale (|f − f0| > 1,5 kHz), en dB sous la fondamentale : un réglage
// modulé lentement ne crée des raies que tout près de f0 ; ce qui reste au loin vient des transitions
// (paliers, sections qui apparaissent ou disparaissent).
static double farNoise (const std::vector<float>& y, int start, int n, double fs, double f0)
{
    std::vector<double> x ((size_t) n);
    for (int i = 0; i < n; ++i) x[(size_t) i] = y[(size_t) (start + i)] * bh (i, n);
    std::vector<fir::cd> X;
    fftReal (x, X, n);
    const int k0 = (int) std::lround (f0 * n / fs);
    double fund = 0, far = 0;
    for (int k = k0 - 4; k <= k0 + 4; ++k) fund += std::norm (X[(size_t) k]);
    for (int k = (int) (50.0 * n / fs); k < n / 2 - 8; ++k)
    {
        const double f = k * fs / n;
        bool nearHarm = false;
        for (int h = 1; h * f0 < 0.5 * fs; ++h) if (std::abs (f - h * f0) < 1500.0) nearHarm = true;
        if (! nearHarm) far += std::norm (X[(size_t) k]);
    }
    return 10.0 * std::log10 (std::max (1e-30, far / fund));
}

// 1.x : FIR de phase linéaire (échantillonnage à N points, fenêtre de Blackman-Harris sur tout le FIR)
static std::vector<double> legacyLinearFir (const BandSpec& b, double fs, int N)
{
    Design d; design (b, fs, d, true);
    std::vector<fir::cd> X ((size_t) N);
    for (int k = 0; k <= N / 2; ++k) X[(size_t) k] = std::abs (response (d, std::max (1.0, k * fs / N), fs, true));
    for (int k = 1; k < N / 2; ++k) X[(size_t) (N - k)] = std::conj (X[(size_t) k]);
    fir::fft (X.data(), N, true);
    std::vector<double> h ((size_t) N);
    for (int n = 0; n < N; ++n) h[(size_t) n] = X[(size_t) ((n - N / 2 + N) % N)].real() / N * bh (n, N);
    return h;
}

// ── Outils des sections 5, 6 et 12 à 15 ────────────────────────────────
// Résidu = sortie du moteur − référence idéale (Engine::referenceExact : réglages lissés et conception
// EXACTE à chaque échantillon, même détecteur). Tout ce qui reste dans le résidu est une erreur du
// moteur : paliers, interpolation des coefficients, raies à fs/32 ou ailleurs. On en donne la raie la
// plus forte (fenêtre de Blackman-Harris, tout le spectre) et la valeur efficace, en dB sous la
// fondamentale / sous le signal de la référence. exclHz > 0 : on ignore f0 ± exclHz (voir section 6).
struct Resid { double line = 0, rms = 0, atHz = 0; };
static Resid residual (const std::vector<float>& y, const std::vector<float>& ref, int start, int n, double fs, double f0 = 0, double exclHz = 0)
{
    std::vector<fir::cd> X ((size_t) n), R ((size_t) n);
    double rr = 0, ss = 0;
    for (int i = 0; i < n; ++i)
    {
        const double a = y[(size_t) (start + i)], b = ref[(size_t) (start + i)];
        X[(size_t) i] = b * bh (i, n); R[(size_t) i] = (a - b) * bh (i, n);
        rr += (a - b) * (a - b); ss += b * b;
    }
    fir::fft (X.data(), n, false); fir::fft (R.data(), n, false);
    double fund = 0, worst = 0; int wk = 0;
    for (int k = 1; k < n / 2; ++k) fund = std::max (fund, std::abs (X[(size_t) k]));
    for (int k = 1; k < n / 2; ++k)
    {
        if (exclHz > 0 && std::abs (k * fs / n - f0) < exclHz) continue;
        if (std::abs (R[(size_t) k]) > worst) { worst = std::abs (R[(size_t) k]); wk = k; }
    }
    Resid r;
    r.line = std::max (-200.0, db (worst / std::max (1e-30, fund)));
    r.rms = std::max (-200.0, 0.5 * db (rr / std::max (1e-30, ss)));
    r.atHz = wk * fs / n;
    return r;
}

// Un cas d'EQ dynamique : sinus f0 dont l'amplitude varie à 4 Hz (amp·(1 ± depth)) ; voie droite à 60 %
struct DynCase { const char* name; Shape shape; int slope; Place place; double freq, gain, q, thr, range, att, rel, f0, amp, depth; };
static const DynCase dynCases[] = {
    { "cloche 1 kHz Q 1.5, plage -12, attaque 0.5 ms",    Shape::bell,      12, Place::stereo, 1000,  0, 1.5,  -30, -12, 0.5, 20,  997, 0.25, 0.80 },
    { "la même, repasse sous le seuil (gain 0 dB)",       Shape::bell,      12, Place::stereo, 1000,  0, 1.5,  -30, -12, 0.5, 20,  997, 0.10, 0.97 },
    { "cloche +4 dB, plage -12 (traverse 0 dB)",          Shape::bell,      12, Place::stereo, 1000,  4, 1.5,  -30, -12, 0.5, 20,  997, 0.25, 0.80 },
    { "high shelf 6 kHz 12 dB/oct, plage +9",             Shape::highShelf, 12, Place::stereo, 6000,  0, 0.71, -22,   9, 1.0, 20, 7001, 0.25, 0.80 },
    { "cloche 80 Hz Q 4, plage -18",                      Shape::bell,      12, Place::stereo,   80,  0, 4.0,  -30, -18, 3.0, 80,   83, 0.25, 0.80 },
    { "cloche -3 dB Q 6 sur le Mid, plage +15",           Shape::bell,      12, Place::mid,    1000, -3, 6.0,  -30,  15, 0.5, 20,  997, 0.25, 0.80 },
    { "low shelf 300 Hz 6 dB/oct, plage -10",             Shape::lowShelf,   6, Place::stereo,  300,  0, 0.71, -30, -10, 2.0, 20,  211, 0.25, 0.80 },
    { "tilt 1.5 kHz -2 dB, plage +8",                     Shape::tiltShelf, 12, Place::stereo, 1500, -2, 0.71, -30,   8, 0.5, 20,  997, 0.25, 0.80 },
    { "cloche 9 kHz Q 3, plage -24",                      Shape::bell,      12, Place::stereo, 9000,  0, 3.0,  -30, -24, 0.5, 20, 9001, 0.25, 0.80 },
    { "cloche 3 kHz Q 12 sur le Side, plage -30",         Shape::bell,      12, Place::side,   3000,  0, 12.0, -36, -30, 0.5, 30, 3001, 0.25, 0.80 } };

static const int dynN = 1 << 17, dynPre = 8192;

// mode : 0 = moteur, 1 = référence idéale, 2 = 1.x (paliers de 32 échantillons)
static void runDynCase (const DynCase& c, int mode, int hostBlock, std::vector<float>& l, std::vector<float>& r)
{
    const double fs = 48000;
    Engine eng; eng.referenceExact = mode == 1; eng.legacyStepping = mode == 2; eng.prepare (fs);
    for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
    BandSpec s; s.active = true; s.shape = c.shape; s.slope = c.slope; s.place = c.place; s.freq = c.freq; s.gainDb = c.gain; s.q = c.q;
    DynSettings d; d.on = true; d.thrDb = c.thr; d.rangeDb = c.range; d.attackMs = c.att; d.releaseMs = c.rel;
    eng.setBand (0, s, d, 1.0);
    const int tot = dynN + dynPre;
    l.resize ((size_t) tot); r.resize ((size_t) tot);
    for (int i = 0; i < tot; ++i)
    {
        const double env = c.amp * (1.0 + c.depth * std::sin (2 * pi * 4.0 * i / fs));
        l[(size_t) i] = (float) (env * std::sin (2 * pi * c.f0 * i / fs));
        r[(size_t) i] = 0.6f * l[(size_t) i];
    }
    for (int off = 0; off < tot; off += hostBlock) eng.process (l.data() + off, r.data() + off, std::min (hostBlock, tot - off), false);
}

// Automation : cloche +12 dB Q 2 dont la fréquence est modulée (1 kHz × 2^(±1.5) à 3 Hz), sinus 3001 Hz.
// mode 0 / 2 : le réglage arrive par blocs de hostBlock échantillons ; mode 1 (référence) : à chaque
// échantillon, décalé de lagSamples (retard du réglage dans le moteur, voir section 6).
static void runSweep (int mode, int hostBlock, double lagSamples, std::vector<float>& l, bool dynamic = false)
{
    const double fs = 48000;
    Engine eng; eng.referenceExact = mode == 1; eng.legacyStepping = mode == 2; eng.prepare (fs);
    for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
    const int tot = dynN + dynPre, blk = mode == 1 ? 1 : hostBlock;
    l.resize ((size_t) tot);
    std::vector<float> r;
    for (int i = 0; i < tot; ++i)
        l[(size_t) i] = dynamic ? (float) (0.25 * (1.0 + 0.8 * std::sin (2 * pi * 4.0 * i / fs)) * std::sin (2 * pi * 997.0 * i / fs))
                                : (float) (0.1 * std::sin (2 * pi * 3001.0 * i / fs));
    r = l;
    for (int off = 0; off < tot; off += blk)
    {
        const double t = (mode == 1 ? off - lagSamples : off) / fs;
        BandSpec s; s.active = true; s.shape = Shape::bell;
        DynSettings d;
        if (! dynamic) { s.gainDb = 12; s.q = 2; s.freq = 1000.0 * std::pow (2.0, 1.5 * std::sin (2 * pi * 3.0 * t)); }
        else
        {
            s.gainDb = 0; s.q = 1.5; s.freq = 1000.0 * std::pow (2.0, 0.5 * std::sin (2 * pi * 3.0 * t));
            d.on = true; d.thrDb = -30; d.rangeDb = -12; d.attackMs = 0.5; d.releaseMs = 20;
        }
        eng.setBand (0, s, d, 1.0);
        eng.process (l.data() + off, r.data() + off, std::min (blk, tot - off), false);
    }
}

// ── Outils des sections 17 à 23 : modes FIR avec bandes dynamiques ──────
// FFT du banc (fir::fft, double) avec les conventions de juce::dsp::FFT : le banc exécute le même
// code que le plugin (LinearPhaseCore.h), seule la FFT diffère.
struct TestFft
{
    explicit TestFft (int order) : n (1 << order), w ((size_t) n) {}
    void forward (float* d)
    {
        for (int i = 0; i < n; ++i) w[(size_t) i] = d[i];
        fir::fft (w.data(), n, false);
        for (int k = 0; k <= n / 2; ++k) { d[2 * k] = (float) w[(size_t) k].real(); d[2 * k + 1] = (float) w[(size_t) k].imag(); }
    }
    void inverse (float* d)
    {
        for (int k = 0; k <= n / 2; ++k) w[(size_t) k] = fir::cd (d[2 * k], d[2 * k + 1]);
        for (int k = 1; k < n / 2; ++k) w[(size_t) (n - k)] = std::conj (w[(size_t) k]);
        fir::fft (w.data(), n, true);
        for (int i = 0; i < n; ++i) d[i] = (float) (w[(size_t) i].real() / n);
    }
    int n; std::vector<fir::cd> w;
};
using TestLinear = LinearPhaseT<TestFft>;

// La chaîne des modes FIR, comme dans PluginProcessor::processBlock : retard du détecteur, moteur,
// convolution. Le fil de conception est remplacé par des appels à redesign().
struct FirRig
{
    TestLinear lp;
    Engine eng;
    fir::Mode mode = fir::Mode::linear;
    double fs = 48000;
    int N = 4096, lat = 0, w = 0;
    bool legacy = false;                     // méthode 2.0 : reconception toutes les 25 ms, détecteur en avance de 30 ms
    long count = 0, lastDesign = 0;
    double lastKey = 0;
    double designSec = 0;                    // temps passé dans redesign() (fil de conception du plugin)
    std::array<BandSpec, maxBands> bands;
    std::array<DynSettings, maxBands> dyn;
    std::vector<float> ringL, ringR, scRing, detL, detR, scD, oldL, oldR, fadeT, trace;
    std::array<const float*, maxBands> tracePtr {};
    static constexpr int chunk = 1024;

    TestLinear::Snapshot snapshot() const
    {
        TestLinear::Snapshot s;
        s.bands = bands;
        for (int b = 0; b < maxBands; ++b)
            if (bands[(size_t) b].active && dyn[(size_t) b].on && canBeDynamic (bands[(size_t) b].shape))
            { s.dyn.full[(size_t) b] = dyn[(size_t) b].rangeDb; s.dyn.now[(size_t) b] = eng.getDynOffset (b); }
        return s;
    }
    void apply() { for (int b = 0; b < maxBands; ++b) eng.setBand (b, bands[(size_t) b], dyn[(size_t) b], 1.0); }
    void redesign()
    {
        const auto t0 = std::chrono::steady_clock::now();
        lp.collectGarbage();   // (comme le fil de conception du plugin : sans cela, plus aucun jeu n'est installé après 8)
        lp.design (snapshot(), mode, N, false);
        designSec += std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
    }
    void start (double rate, fir::Mode m, int length)
    {
        fs = rate; mode = m; N = length;
        eng.prepare (fs); lp.legacyDynamics = legacy; lp.prepare (fs);
        lat = lp.latencyFor (mode, N);
        const size_t ring = (size_t) (fir::maxLength (fs) / 2 + lp.partition() + chunk + 1);
        ringL.assign (ring, 0.0f); ringR.assign (ring, 0.0f); scRing.assign (ring, 0.0f);
        for (auto* v : { &detL, &detR, &scD, &oldL, &oldR, &fadeT }) v->assign ((size_t) chunk, 0.0f);
        trace.assign ((size_t) maxBands * chunk, 0.0f);
        for (int b = 0; b < maxBands; ++b) tracePtr[(size_t) b] = trace.data() + (size_t) b * chunk;
        w = 0; count = 0; lastDesign = 0;
        apply(); redesign(); lp.switchTo (mode, N);
    }
    // r nul : mono. sc : side-chain externe (nul : aucun).
    void run (float* l, float* r, int total, const float* sc = nullptr)
    {
        const int ring = (int) ringL.size();
        for (int off = 0, n = 0; off < total; off += n)
        {
            n = std::min (chunk, total - off);
            // (méthode 2.0 : on s'arrête à l'instant exact de la reconception suivante, toutes les 25 ms)
            if (legacy) n = std::max (1, std::min (n, (int) (lastDesign + (long) (0.025 * fs) - count)));
            float* L = l + off; float* R = r != nullptr ? r + off : nullptr;
            const bool lin = mode == fir::Mode::linear;
            const int lag = legacy ? std::max (0, lat - (int) std::lround (0.03 * fs)) : lin ? lat - lp.partition() : lat;
            uint32_t dynMask = 0;
            for (int b = 0; b < maxBands; ++b) if (dyn[(size_t) b].on && bands[(size_t) b].active) dynMask |= 1u << b;
            for (int b = 0; b < maxBands; ++b)
            {
                const bool tr = lin && ! legacy && ((dynMask >> b) & 1u) != 0;
                float* row = trace.data() + (size_t) b * chunk;
                eng.dynTrace[(size_t) b] = tr ? row : nullptr;
                if (tr) std::fill (row, row + n, 0.0f);
            }
            for (int i = 0; i < n; ++i)
            {
                ringL[(size_t) w] = L[i]; ringR[(size_t) w] = R != nullptr ? R[i] : L[i];
                if (sc != nullptr) scRing[(size_t) w] = sc[off + i];
                const int di = (w - lag + ring) % ring;
                detL[(size_t) i] = ringL[(size_t) di]; detR[(size_t) i] = ringR[(size_t) di]; scD[(size_t) i] = scRing[(size_t) di];
                w = (w + 1) % ring;
            }
            const float* scp = sc != nullptr ? scD.data() : nullptr;
            if (lin || legacy)
            {
                eng.process (detL.data(), R != nullptr ? detR.data() : nullptr, n, true, scp, nullptr);
                lp.process (L, R, n, tracePtr.data(), legacy ? 0u : dynMask);
            }
            else
            {
                for (int o = 0; o < n;)
                {
                    const auto st = lp.outState();
                    const int m = std::min (n - o, lp.untilBoundary());
                    float* rr = R != nullptr ? R + o : nullptr;
                    lp.process (L + o, rr, m, nullptr, 0, oldL.data() + o, R != nullptr ? oldR.data() + o : nullptr, fadeT.data() + o);
                    Engine::FirSplit sp; sp.cur = st.cur; sp.old = st.old;
                    if (st.split) { sp.oldL = oldL.data() + o; sp.oldR = R != nullptr ? oldR.data() + o : nullptr; sp.t = fadeT.data() + o; }
                    eng.process (L + o, rr, m, false, scp != nullptr ? scp + o : nullptr, nullptr, &sp);
                    o += m;
                }
            }
            count += n;
            // méthode 2.0 : le fil de conception relit le gain dynamique toutes les 25 ms
            // (seulement si un gain a bougé de 0,1 dB, comme la clé du fil de conception)
            if (legacy && count - lastDesign >= (long) (0.025 * fs))
            {
                lastDesign = count;
                double key = 0;
                for (int b = 0; b < maxBands; ++b) key = key * 31.0 + std::round (eng.getDynOffset (b) * 10.0);
                if (key != lastKey) { lastKey = key; redesign(); }
            }
        }
    }
};

// Détecteur et courbe de la dynamique, récrits à part (mêmes formules que le moteur) : gain en dB pour
// chaque échantillon d'un signal mono
static std::vector<double> detectorGain (const std::vector<float>& x, const BandSpec& b, const DynSettings& d, double fs)
{
    SectionRT det; toRT (listenSection (b.shape, b.freq, b.q, fs), det);
    const double aA = std::exp (-1.0 / (std::max (0.1, d.attackMs) * 0.001 * fs)), aR = std::exp (-1.0 / (std::max (1.0, d.releaseMs) * 0.001 * fs));
    double st[2] = { 0, 0 }, env = 0;
    std::vector<double> g (x.size());
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double v = std::abs (tick (det, st, x[i]));
        env = v > env ? aA * env + (1 - aA) * v : aR * env + (1 - aR) * v;
        const double over = 20.0 * std::log10 (std::max (1e-9, env)) - d.thrDb, W = 6.0;
        const double o = over <= -W / 2 ? 0.0 : over < W / 2 ? (over + W / 2) * (over + W / 2) / (2 * W) : over;
        g[i] = d.rangeDb < 0 ? -std::min (o, -d.rangeDb) : std::min (o, d.rangeDb);
    }
    return g;
}

static double convAt (const std::vector<double>& h, const std::vector<float>& x, long i)
{
    double s = 0;
    const long kmax = std::min ((long) h.size() - 1, i);
    for (long k = 0; k <= kmax; ++k) s += h[(size_t) k] * x[(size_t) (i - k)];
    return s;
}

// salves de 997 Hz (une toutes les 250 ms, montée 0,5 ms) + bruit faible : le signal du prototype
static std::vector<float> burstSignal (int n, double fs, double f0 = 997.0, double amp = 0.4)
{
    std::vector<float> x ((size_t) n);
    std::mt19937 rng (4); std::uniform_real_distribution<double> U (-1, 1);
    for (int i = 0; i < n; ++i)
    {
        const double ph = std::fmod (i / fs, 0.25);
        const double env = ph < 0.08 ? std::min (1.0, ph / 0.0005) : std::exp (-(ph - 0.08) / 0.01);
        x[(size_t) i] = (float) (amp * env * std::sin (2 * pi * f0 * i / fs) + 0.003 * U (rng));
    }
    return x;
}

struct Res2 { double rms = 0, peak = 0; };
// résidu de y (décalé de shift) par rapport à ref, à partir de start, un échantillon sur step
template <class A, class B>
static Res2 residual2 (const A& y, long shift, const B& ref, long start, long end, long step = 1)
{
    double rr = 0, ss = 0, pk = 0, spk = 0;
    for (long i = start; i < end; i += step)
    {
        const double e = (double) y[(size_t) (i + shift)] - (double) ref[(size_t) i];
        rr += e * e; ss += (double) ref[(size_t) i] * (double) ref[(size_t) i];
        pk = std::max (pk, std::abs (e)); spk = std::max (spk, std::abs ((double) ref[(size_t) i]));
    }
    return { 0.5 * db (rr / std::max (1e-300, ss)), db (pk / std::max (1e-300, spk)) };
}
// Erreur de courbe du mélange de FIR, mesurée sur les FIR réels du jeu (référence + écarts) : pour une
// grille de gains dynamiques, réponse du mélange contre celle du FIR conçu exactement pour ces gains.
// 10 Hz – 20 kHz, là où la cible est au-dessus de −60 dB. Grille de gains : bandes 1 et 2 indépendantes,
// les bandes 3 et 4 (s'il y en a) parcourent leur plage en fonction des deux premières.
static double bankCurveError (const std::array<BandSpec, maxBands>& bands, const fir::DynInfo& dyn, int N, double fs, int steps, int* nDev = nullptr, int* nMix = nullptr)
{
    static fir::Work w;
    const auto plan = fir::makePlan (bands, dyn, fir::Mode::linear, fs, true);
    if (nDev != nullptr) *nDev = plan.nDev;
    if (nMix != nullptr) *nMix = plan.nMix;
    if (plan.nMix < 1) return 99.0;
    fir::Request rq; rq.bands = bands; rq.N = N; rq.fs = fs; rq.mode = fir::Mode::linear;
    fir::Result ref, exact; std::vector<fir::Result> dev;
    fir::designBank (rq, dyn, plan, ref, dev, w);
    const int L = 4 * N;
    std::vector<fir::cd> A ((size_t) L), B ((size_t) L);
    std::vector<double> h ((size_t) N);
    const int two = plan.nMix >= 2 ? steps : 0;
    double worst = 0;
    for (int i1 = 0; i1 <= steps; ++i1)
        for (int i2 = 0; i2 <= two; ++i2)
        {
            double off[fir::maxMix] {}, wt[fir::maxMix][fir::maxSeg + 1] {};
            for (int k = 0; k < plan.nMix; ++k)
            {
                off[k] = plan.mb[k].full * (k == 0 ? i1 : k == 1 ? i2 : k == 2 ? steps - i1 : (i1 + i2) % (steps + 1)) / steps;
                int seg; double a; fir::locate (plan.mb[k], off[k], seg, a);
                wt[k][seg] = 1 - a; wt[k][seg + 1] = a;
            }
            h = ref.h[0];
            double c[fir::maxDev];
            fir::devWeights (plan, wt, c);
            for (int di = 0; di < plan.nDev; ++di)
                if (c[di] != 0.0)
                    for (int n = 0; n < N; ++n) h[(size_t) n] += c[di] * dev[(size_t) di].h[0][(size_t) n];
            fir::Request re = rq;
            for (int k = 0; k < plan.nMix; ++k) re.bands[(size_t) plan.mb[k].band].gainDb += off[k];
            fir::design (re, exact, w);
            std::fill (A.begin(), A.end(), fir::cd (0, 0)); std::fill (B.begin(), B.end(), fir::cd (0, 0));
            for (int n = 0; n < N; ++n) { A[(size_t) n] = h[(size_t) n]; B[(size_t) n] = exact.h[0][(size_t) n]; }
            fir::fft (A.data(), L, false); fir::fft (B.data(), L, false);
            for (int k = (int) std::ceil (10.0 * L / fs); k <= (int) (20000.0 * L / fs); ++k)
            {
                const double t = std::abs (B[(size_t) k]);
                if (t < 1e-3) continue;
                worst = std::max (worst, std::abs (db (std::abs (A[(size_t) k]) / t)));
            }
        }
    return worst;
}

// Erreur d'interpolation seule (cibles analytiques, 10 Hz – 24 kHz) : toutes formes, toutes plages
static double blendCurveError (BandSpec b, double range, double fs)
{
    std::array<BandSpec, maxBands> bands; bands[0] = b;
    fir::DynInfo d; d.full[0] = range;
    const auto pl = fir::makePlan (bands, d, fir::Mode::linear, fs, true);
    const auto& m = pl.mb[0];
    auto mag = [&] (double g, double f) { BandSpec x = b; x.gainDb += g; Design de; design (x, fs, de, true); return std::abs (response (de, f, fs, true)); };
    double e = 0;
    const int steps = std::max (24, (int) std::abs (range) * 4);
    for (int i = 0; i <= steps; ++i)
    {
        const double g = range * i / steps; int s; double a; fir::locate (m, g, s, a);
        for (int k = 0; k < 200; ++k)
        {
            const double f = 10.0 * std::pow (2400.0, k / 199.0);
            e = std::max (e, std::abs (db (((1 - a) * mag (m.offsetOf (s), f) + a * mag (m.offsetOf (s + 1), f)) / mag (g, f))));
        }
    }
    return e;
}

static std::string fmt2 (double v) { char s[32]; std::snprintf (s, sizeof s, "%.2f", v); return s; }


// ── Outils des sections 24 à 29 (formes et pentes 3.0) ──────────────────
// Réponse impulsionnelle du moteur (voie gauche) pour un jeu de bandes, après 4096 échantillons de mise en route
static std::vector<double> engineImpulse (const std::vector<BandSpec>& bands, double fs, int len)
{
    Engine e; e.prepare (fs);
    for (size_t i = 0; i < bands.size(); ++i) e.setBand ((int) i, bands[i], DynSettings(), 1.0);
    std::vector<float> l ((size_t) (4096 + len), 0.0f), r ((size_t) (4096 + len), 0.0f);
    l[4096] = 1.0f; r[4096] = 1.0f;
    for (int off = 0; off < 4096 + len; off += 512) e.process (l.data() + off, r.data() + off, std::min (512, 4096 + len - off), false);
    return std::vector<double> (l.begin() + 4096, l.end());
}

static std::complex<double> dftAt (const std::vector<double>& h, double f, double fs)
{
    const double w = 2.0 * pi * f / fs;
    std::complex<double> s (0, 0);
    for (size_t n = 0; n < h.size(); ++n) s += h[n] * std::polar (1.0, -w * (double) n);
    return s;
}

// Cible d'une coupe à pente libre : Butterworth d'ordre fractionnaire n = pente / 6
static double fracIdealDb (bool lowCut, double fc, double slope, double f)
{
    const double x = lowCut ? fc / f : f / fc;
    return -10.0 * std::log10 (1.0 + std::pow (x, 2.0 * slope / 6.0));
}

// Pente moyenne (dB/oct) d'une courbe entre ses passages à −12 et −36 dB (0 si un passage manque entre 20 Hz et 20 kHz)
template <typename F>
static double meanSlope (F&& magDb)
{
    double f12 = 0, f36 = 0, pm = 0, pf = 0;
    for (int i = 0; i <= 4000; ++i)
    {
        const double f = 20.0 * std::pow (1000.0, i / 4000.0), m = magDb (f);
        if (i > 0)
            for (double lv : { -12.0, -36.0 })
                if ((pm - lv) * (m - lv) < 0) (lv == -12.0 ? f12 : f36) = pf * std::pow (f / pf, (lv - pm) / (m - pm));
        pm = m; pf = f;
    }
    return f12 > 0 && f36 > 0 ? 24.0 / std::abs (std::log2 (f36 / f12)) : 0.0;
}

static void testShapes30()
{
    const double rates[] = { 44100, 48000, 96000, 192000 };
    using clk = std::chrono::steady_clock;

    // ── 17. Flat tilt ────────────────────────────────────────────────────
    std::printf ("24. Flat tilt : écart à la droite idéale (gain = écart 20 kHz − 20 Hz, 0 dB à la fréquence de la bande), 20 Hz – 20 kHz\n");
    {
        for (double fs : rates)
        {
            double w30 = 0, w60 = 0, wA = 0, wSym = 0;
            for (double g : { -60.0, -45.0, -30.0, -18.0, -6.0, -1.0, 1.0, 6.0, 18.0, 30.0, 45.0, 60.0 })
                for (double f0 : { 20.0, 200.0, 1000.0, 6000.0, 20000.0 })
                {
                    BandSpec b; b.active = true; b.shape = Shape::flatTilt; b.freq = f0; b.gainDb = g;
                    BandSpec bi = b; bi.gainDb = -g;
                    Design d, a, di; design (b, fs, d, false); design (b, fs, a, true); design (bi, fs, di, false);
                    for (int i = 0; i <= 800; ++i)
                    {
                        const double f = 20.0 * std::pow (1000.0, i / 800.0);
                        const double ideal = g / 60.0 * 20.0 * std::log10 (f / f0);
                        const double e = std::abs (magnitudeDb (d, f, fs, false) - ideal);
                        (std::abs (g) <= 30 ? w30 : w60) = std::max (std::abs (g) <= 30 ? w30 : w60, e);
                        if (std::abs (g) <= 30) wA = std::max (wA, std::abs (magnitudeDb (a, f, fs, true) - ideal));
                        wSym = std::max (wSym, std::abs (magnitudeDb (d, f, fs, false) + magnitudeDb (di, f, fs, false)));
                    }
                }
            std::printf ("   %.1f kHz : gains jusqu'à ±30 dB %.3f dB [seuil 0.10] ; jusqu'à ±60 dB (échelle 200 %%) %.3f dB [seuil 0.10] ; cible des FIR %.4f dB [seuil 0.02] ; +g puis −g : %.1e dB [seuil 0.01]\n",
                         fs / 1000, w30, w60, wA, wSym);
            CHECK (w30 <= 0.10 && w60 <= 0.10, "Flat tilt à %.0f Hz : %.3f / %.3f dB", fs, w30, w60);
            CHECK (wA <= 0.02, "Flat tilt, cible des FIR à %.0f Hz : %.4f dB", fs, wA);
            CHECK (wSym <= 0.01, "Flat tilt, symétrie à %.0f Hz : %.2e dB", fs, wSym);
        }
        // moteur : réponse mesurée = droite idéale ; +g puis −g dans le moteur = plat
        {
            const double fs = 44100;
            BandSpec b; b.active = true; b.shape = Shape::flatTilt; b.freq = 1000; b.gainDb = 12;
            BandSpec bi = b; bi.gainDb = -12;
            const auto h = engineImpulse ({ b }, fs, 1 << 17), hs = engineImpulse ({ b, bi }, fs, 1 << 15);
            double e = 0, es = 0;
            for (int i = 0; i <= 60; ++i)
            {
                const double f = 20.0 * std::pow (1000.0, i / 60.0);
                e = std::max (e, std::abs (db (std::abs (dftAt (h, f, fs))) - 12.0 / 60.0 * 20.0 * std::log10 (f / 1000.0)));
                es = std::max (es, std::abs (db (std::abs (dftAt (hs, f, fs)))));
            }
            std::printf ("   moteur, 44,1 kHz, +12 dB à 1 kHz : écart mesuré à la droite %.3f dB [seuil 0.10] ; +12 puis −12 dB : %.4f dB [seuil 0.01]\n", e, es);
            CHECK (e <= 0.10, "Flat tilt, moteur : %.3f dB", e);
            CHECK (es <= 0.01, "Flat tilt, moteur, symétrie : %.4f dB", es);
        }
        // Linear phase et Natural phase : FIR conçu sur la cible (longueur Auto), 20 Hz – 20 kHz
        {
            fir::Work w;
            double wl = 0, wn = 0, wp = 0; int nMin = 1 << 30, nMax = 0;
            for (double fs : { 44100.0, 48000.0, 96000.0 })
                for (double g : { 3.0, 12.0, 30.0, -3.0, -12.0, -30.0 })
                    for (int nat = 0; nat < 2; ++nat)
                    {
                        fir::Request rq; rq.fs = fs; rq.mode = nat ? fir::Mode::natural : fir::Mode::linear; rq.delay = fir::naturalDelay (fs);
                        rq.bands[0].active = true; rq.bands[0].shape = Shape::flatTilt; rq.bands[0].freq = 1000; rq.bands[0].gainDb = g;
                        rq.N = fir::chooseLength (fir::Res::automatic, rq.bands, fs, rq.mode);
                        if (! nat) { nMin = std::min (nMin, rq.N / fir::rateMult (fs)); nMax = std::max (nMax, rq.N / fir::rateMult (fs)); }
                        fir::Result r; fir::design (rq, r, w);
                        Design a; design (rq.bands[0], fs, a, true);
                        for (int i = 0; i <= 120; ++i)
                        {
                            const double f = 20.0 * std::pow (nat ? 900.0 : 1000.0, i / 120.0);
                            const auto hf = fir::responseAt (r.h[0], nat ? rq.delay : rq.N / 2, fs, f);
                            const double x = std::abs (db (std::abs (hf)) - g / 60.0 * 20.0 * std::log10 (f / 1000.0));
                            if (nat) { wn = std::max (wn, x); wp = std::max (wp, std::abs (std::arg (hf / response (a, f, fs, true))) * 180.0 / pi); }
                            else wl = std::max (wl, x);
                        }
                    }
            std::printf ("   FIR, longueur Auto (%d à %d points à 44,1 / 48 kHz), gains ±3, ±12, ±30 dB, 44,1 / 48 / 96 kHz : Linear phase %.3f dB de la droite [seuil 0.12] ;\n"
                         "      Natural phase (20 Hz – 18 kHz) %.3f dB de la droite, phase à %.2f° de la cible [seuils 0.10 dB, 1°]\n", nMin, nMax, wl, wn, wp);
            CHECK (wl <= 0.12, "Flat tilt en Linear phase : %.3f dB", wl);
            CHECK (wn <= 0.10 && wp <= 1.0, "Flat tilt en Natural phase : %.3f dB, %.2f°", wn, wp);
        }
    }

    // ── 18. All pass ─────────────────────────────────────────────────────
    std::printf ("25. All pass : module plat, rotation de phase à la fréquence de la bande (ordre 1 : pente 6 dB/oct ; ordre 2 : autres pentes, avec Q)\n");
    {
        double wMag = 0, wPh1 = 0, wPh2 = 0;
        for (double fs : rates)
            for (double f0 : { 30.0, 1000.0, 9000.0, 18000.0 })
                for (double q : { 0.3, 0.71, 4.0, 20.0 })
                    for (int slope : { 6, 12 })
                    {
                        BandSpec b; b.active = true; b.shape = Shape::allPass; b.freq = f0; b.q = q; b.slope = slope;
                        Design d; design (b, fs, d, false);
                        for (int i = 0; i <= 400; ++i) wMag = std::max (wMag, std::abs (magnitudeDb (d, 20.0 * std::pow (1000.0, i / 400.0), fs, false)));
                        const double ph = std::arg (response (d, f0, fs, false)) * 180.0 / pi;
                        if (slope == 6) wPh1 = std::max (wPh1, std::abs (ph + 90.0));
                        else wPh2 = std::max (wPh2, std::abs (std::abs (ph) - 180.0));
                    }
        std::printf ("   conception (4 fréquences d'échantillonnage, Q 0.3–20) : module %.1e dB [seuil 0.001] ; phase à la fréquence de la bande : ordre 1 −90° ± %.1e°, ordre 2 −180° ± %.1e° [seuil 0.01°]\n", wMag, wPh1, wPh2);
        CHECK (wMag <= 0.001, "All pass : module %.2e dB", wMag);
        CHECK (wPh1 <= 0.01 && wPh2 <= 0.01, "All pass : phase %.2e / %.2e", wPh1, wPh2);
        // test de nul dans le moteur : le module de la sortie est celui de l'entrée
        double wEng = 0, phEng = 0;
        for (int slope : { 6, 12 })
        {
            BandSpec b; b.active = true; b.shape = Shape::allPass; b.freq = 1000; b.q = 2; b.slope = slope;
            const auto h = engineImpulse ({ b }, 48000, 1 << 14);
            for (int i = 0; i <= 80; ++i) wEng = std::max (wEng, std::abs (db (std::abs (dftAt (h, 20.0 * std::pow (1000.0, i / 80.0), 48000)))));
            const double ph = std::arg (dftAt (h, 1000, 48000)) * 180.0 / pi;
            phEng = std::max (phEng, slope == 6 ? std::abs (ph + 90.0) : std::abs (std::abs (ph) - 180.0));
        }
        std::printf ("   moteur (réponse impulsionnelle, 48 kHz, 1 kHz Q 2) : module %.1e dB [seuil 0.001] ; phase à 1 kHz à %.1e° de −90° / −180° [seuil 0.01°]\n", wEng, phEng);
        CHECK (wEng <= 0.001 && phEng <= 0.01, "All pass, moteur : %.2e dB, %.2e°", wEng, phEng);
        // Linear phase : neutre (le FIR est un simple retard)
        {
            fir::Work w; fir::Request rq; rq.fs = 48000; rq.mode = fir::Mode::linear; rq.N = 4096; rq.delay = fir::naturalDelay (48000);
            rq.bands[0].active = true; rq.bands[0].shape = Shape::allPass; rq.bands[0].freq = 1000; rq.bands[0].q = 2;
            fir::Result r; fir::design (rq, r, w);
            double e = 0;
            for (int n = 0; n < rq.N; ++n) e = std::max (e, std::abs (r.h[0][(size_t) n] - (n == rq.N / 2 ? 1.0 : 0.0)));
            // Natural phase : module et phase du passe-tout analogique
            rq.mode = fir::Mode::natural; rq.N = 8192;
            fir::Result rn; fir::design (rq, rn, w);
            Design a; design (rq.bands[0], 48000, a, true);
            double em = 0, ep = 0;
            for (int i = 0; i <= 60; ++i)
            {
                const double f = 20.0 * std::pow (900.0, i / 60.0);
                const auto hf = fir::responseAt (rn.h[0], rq.delay, 48000, f), ha = response (a, f, 48000, true);
                em = std::max (em, std::abs (db (std::abs (hf)))); ep = std::max (ep, std::abs (std::arg (hf / ha)) * 180.0 / pi);
            }
            std::printf ("   Linear phase : FIR = retard pur à %.1e près [seuil 1e-9] (neutre) ; Natural phase (20 Hz – 18 kHz) : module %.4f dB, phase à %.3f° de l'analogique [seuils 0.01 dB, 1°]\n", e, em, ep);
            CHECK (e <= 1e-9, "All pass en Linear phase : %.2e", e);
            CHECK (em <= 0.01 && ep <= 1.0, "All pass en Natural phase : %.4f dB, %.3f°", em, ep);
        }
    }

    // ── 19. Pentes libres ────────────────────────────────────────────────
    std::printf ("26. Pentes libres des coupes : cible = Butterworth d'ordre fractionnaire, |H|² = 1/(1 + (f/fc)^(2n)), n = pente / 6 (Q 0.71)\n"
                 "   écart mesuré là où la cible est au-dessus de −24 dB, de 20 Hz à 20 kHz ; pente moyenne entre −12 et −36 dB\n"
                 "   (la cible elle-même a une pente moyenne de 0,992 × la consigne entre −12 et −36 dB : −0,76 dB/oct à 96 dB/oct)\n");
    {
        const double kIdeal = 24.0 / (0.5 * std::log2 ((std::pow (10.0, 3.6) - 1.0) / (std::pow (10.0, 1.2) - 1.0)) * 6.0);   // 0,9918
        for (double fs : rates)
        {
            double worst = 0, wS = 0, wF = 0, worstA = 0, sT = 0, sTS = 0, sTF = 0, sN = 0, sN48 = 0; int maxN = 0, wLow = 0, nSl = 0;
            for (int t = 61; t < 960; t += 4)
                for (double fc : { 30.0, 120.0, 500.0, 2000.0, 6000.0, 12000.0, 18000.0 })
                    for (int lowCut = 0; lowCut < 2; ++lowCut)
                    {
                        if (t % 60 == 0) continue;
                        const double slope = t / 10.0;
                        BandSpec b; b.active = true; b.shape = lowCut ? Shape::lowCut : Shape::highCut; b.freq = fc; b.q = 0.7071067811865476; b.slopeFree = slope;
                        Design d, a; design (b, fs, d, false); design (b, fs, a, true);
                        maxN = std::max (maxN, d.n);
                        for (int i = 0; i <= 1000; ++i)
                        {
                            const double f = 20.0 * std::pow (1000.0, i / 1000.0), ideal = fracIdealDb (lowCut, fc, slope, f);
                            if (ideal < -24.0) continue;
                            const double e = std::abs (magnitudeDb (d, f, fs, false) - ideal);
                            if (e > worst) { worst = e; wS = slope; wF = fc; wLow = lowCut; }
                            worstA = std::max (worstA, std::abs (magnitudeDb (a, f, fs, true) - ideal));
                        }
                        const double sl = meanSlope ([&] (double f) { return magnitudeDb (d, f, fs, false); });
                        if (sl > 0)
                        {
                            ++nSl;
                            if (std::abs (sl - kIdeal * slope) > sT) { sT = std::abs (sl - kIdeal * slope); sTS = slope; sTF = fc; }
                            sN = std::max (sN, std::abs (sl - slope));
                            if (slope <= 48.0) sN48 = std::max (sN48, std::abs (sl - slope));
                        }
                    }
            std::printf ("   %.1f kHz : écart %.3f dB (%s %.1f dB/oct à %.0f Hz) [seuil 0.10] ; prototype seul %.3f dB ; pente moyenne à %.2f dB/oct de celle de la cible (%.1f dB/oct à %.0f Hz) [seuil 0.50],\n"
                         "      à %.2f dB/oct de la consigne (%.2f jusqu'à 48 dB/oct) [seuils 1.10 et 0.50] (%d mesures) ; %d sections au plus\n",
                         fs / 1000, worst, wLow ? "coupe-bas" : "coupe-haut", wS, wF, worstA, sT, sTS, sTF, sN, sN48, nSl, maxN);
            CHECK (worst <= 0.10, "pente libre à %.0f Hz : %.3f dB", fs, worst);
            CHECK (sT <= 0.50, "pente libre à %.0f Hz : pente moyenne à %.2f dB/oct de la cible", fs, sT);
            CHECK (sN <= 1.10 && sN48 <= 0.50, "pente libre à %.0f Hz : pente moyenne à %.2f dB/oct de la consigne (%.2f jusqu'à 48 dB/oct)", fs, sN, sN48);
        }
        // une pente libre multiple de 6 dB/oct donne exactement la coupe de la liste ; 0 = pente de la liste
        {
            bool same = true;
            for (int s : { 6, 12, 24, 48, 96 })
                for (int lowCut = 0; lowCut < 2; ++lowCut)
                {
                    BandSpec b; b.active = true; b.shape = lowCut ? Shape::lowCut : Shape::highCut; b.freq = 3000; b.q = 1.3; b.slope = s;
                    BandSpec c = b; c.slope = 12; c.slopeFree = s;
                    BandSpec z = b; z.slopeFree = 0.0; z.brick = false;
                    Design d1, d2, d3; design (b, 44100, d1, false); design (c, 44100, d2, false); design (z, 44100, d3, false);
                    same = same && d1.n == d2.n && d1.n == d3.n;
                    for (int i = 0; i < d1.n && same; ++i)
                        same = d1.s[(size_t) i].g == d2.s[(size_t) i].g && d1.s[(size_t) i].k == d2.s[(size_t) i].k && d1.s[(size_t) i].m2 == d2.s[(size_t) i].m2
                            && d1.s[(size_t) i].g == d3.s[(size_t) i].g;
                }
            std::printf ("   pente libre de 6, 12, 24, 48 ou 96 dB/oct = coupe de la liste, coefficient pour coefficient : %s\n", same ? "oui" : "NON");
            CHECK (same, "pente libre entière différente de la pente de la liste");
        }
        // Q : la section la plus résonante suit le réglage, comme pour les pentes de la liste (information)
        {
            BandSpec b; b.active = true; b.shape = Shape::highCut; b.freq = 1000; b.slopeFree = 21.0;
            b.q = 0.7071067811865476; Design d0; design (b, 48000, d0, false);
            b.q = 2.0; Design d1; design (b, 48000, d1, false);
            double bump = -100; for (int i = 0; i <= 400; ++i) { const double f = 300.0 * std::pow (5.0, i / 400.0); bump = std::max (bump, magnitudeDb (d1, f, 48000, false) - magnitudeDb (d0, f, 48000, false)); }
            std::printf ("   coupe-haut 21 dB/oct à 1 kHz, Q 2 au lieu de 0.71 : bosse de %.2f dB à la coupure (9,03 dB pour un facteur 2,83 sur le Q de la section résonante)\n", bump);
            CHECK (bump > 6.0 && bump < 10.0, "Q d'une pente libre : %.2f dB", bump);
        }
        // moteur : réponse mesurée = cible
        {
            BandSpec b; b.active = true; b.shape = Shape::highCut; b.freq = 2000; b.q = 0.7071067811865476; b.slopeFree = 15.5;
            const auto h = engineImpulse ({ b }, 44100, 1 << 14);
            double e = 0;
            for (int i = 0; i <= 80; ++i) { const double f = 20.0 * std::pow (1000.0, i / 80.0), id = fracIdealDb (false, 2000, 15.5, f); if (id > -24) e = std::max (e, std::abs (db (std::abs (dftAt (h, f, 44100))) - id)); }
            std::printf ("   moteur, coupe-haut 15,5 dB/oct à 2 kHz, 44,1 kHz : écart mesuré à la cible %.3f dB [seuil 0.10]\n", e);
            CHECK (e <= 0.10, "pente libre, moteur : %.3f dB", e);
        }
    }

    // ── 20. Brickwall ────────────────────────────────────────────────────
    std::printf ("27. Brickwall : elliptique d'ordre 12 (6 sections), −3 dB à la fréquence de la bande\n");
    {
        const double third = std::pow (2.0, 1.0 / 3.0);
        for (double fs : rates)
        {
            double rip = 0, att = -1e9, m3 = 0, edge = 0;
            for (double fc : { 25.0, 100.0, 1000.0, 8000.0, 12000.0, 15800.0, 19000.0 })
                for (int lowCut = 0; lowCut < 2; ++lowCut)
                {
                    BandSpec b; b.active = true; b.shape = lowCut ? Shape::lowCut : Shape::highCut; b.freq = fc; b.brick = true;
                    Design d; design (b, fs, d, false);
                    m3 = std::max (m3, std::abs (magnitudeDb (d, fc, fs, false) + 3.0103));
                    double firstStop = 0;
                    for (int i = 0; i <= 8000; ++i)
                    {
                        const double f = 5.0 * std::pow (0.4999 * fs / 5.0, i / 8000.0);
                        const double x = lowCut ? fc / f : f / fc, m = magnitudeDb (d, f, fs, false);
                        if (x <= 0.95) rip = std::max (rip, std::abs (m));
                        if (x >= third) att = std::max (att, m);
                        if (x > 1 && m <= -96.0 && (firstStop == 0 || x < firstStop)) firstStop = x;
                    }
                    // largeur de la transition (−3 dB → −96 dB), en octaves : la plus grande rencontrée
                    double xs = 1.0; while (xs < 4.0 && magnitudeDb (d, lowCut ? fc / xs : fc * xs, fs, false) > -96.0) xs *= 1.0005;
                    if ((lowCut ? fc / xs : fc * xs) < 0.499 * fs) edge = std::max (edge, std::log2 (xs));
                }
            std::printf ("   %.1f kHz : ondulation en bande passante (jusqu'à 0,95 fc) %.3f dB [seuil 0.10] ; niveau max au-delà d'un tiers d'octave %.1f dB [seuil −96] ;\n"
                         "      −96 dB atteints %.3f octave après la coupure ; écart au −3 dB à la coupure %.1e dB\n", fs / 1000, rip, att, edge, m3);
            CHECK (rip <= 0.10, "Brickwall à %.0f Hz : ondulation %.3f dB", fs, rip);
            CHECK (att <= -96.0, "Brickwall à %.0f Hz : atténuation %.1f dB", fs, att);
            CHECK (edge <= 1.0 / 3.0 && m3 < 1e-6, "Brickwall à %.0f Hz : transition %.3f octave, −3 dB %.1e", fs, edge, m3);
        }
        // moteur : sinus un tiers d'octave au-delà de la coupure, niveau en sortie
        {
            const double fs = 48000, fc = 5000, f = fc * third * 1.001;
            Engine e; e.prepare (fs);
            BandSpec b; b.active = true; b.shape = Shape::highCut; b.freq = fc; b.brick = true;
            e.setBand (0, b, DynSettings(), 1.0);
            std::vector<float> l (48000), r (48000);
            for (int i = 0; i < 48000; ++i) l[(size_t) i] = r[(size_t) i] = (float) (0.5 * std::sin (2 * pi * f * i / fs));
            for (int off = 0; off < 48000; off += 480) e.process (l.data() + off, r.data() + off, 480, false);
            double s = 0; for (int i = 24000; i < 48000; ++i) s += (double) l[(size_t) i] * l[(size_t) i];
            const double lvl = db (std::sqrt (s / 24000) / (0.5 / std::sqrt (2.0)));
            std::printf ("   moteur, coupe-haut 5 kHz à 48 kHz, sinus à 6,3 kHz : %.1f dB en sortie [seuil −96]\n", lvl);
            CHECK (lvl <= -96.0, "Brickwall, moteur : %.1f dB", lvl);
        }
        // Linear phase : mur idéal, transition fixée par la longueur du FIR (48 kHz, coupe-haut 5 kHz et coupe-bas 100 Hz)
        {
            fir::Work w;
            std::printf ("   Linear phase à 48 kHz (vrai mur, la transition dépend de la longueur du FIR) : largeur entre −0,1 dB et −96 dB, niveau max au-delà [seuils : 16 fs / N, −96 dB, ± 0.1 dB]\n");
            for (auto res : { fir::Res::low, fir::Res::medium, fir::Res::high, fir::Res::max })
            {
                const int N = fir::lengthFor (res, 48000);
                double width[2] = { 0, 0 }, stop[2] = { -1e9, -1e9 }, pass[2] = { 0, 0 };
                for (int lowCut = 0; lowCut < 2; ++lowCut)
                {
                    const double fc = lowCut ? 100.0 : 5000.0;
                    fir::Request rq; rq.fs = 48000; rq.mode = fir::Mode::linear; rq.N = N; rq.delay = fir::naturalDelay (48000);
                    rq.bands[0].active = true; rq.bands[0].shape = lowCut ? Shape::lowCut : Shape::highCut; rq.bands[0].freq = fc; rq.bands[0].brick = true;
                    fir::Result r; fir::design (rq, r, w);
                    auto mag = [&] (double f) { return db (std::abs (fir::responseAt (r.h[0], N / 2, 48000, f))); };
                    // bord de la bande passante (−0,1 dB) et début de la bande coupée (−96 dB), en s'éloignant de la coupure par pas de 0,25 Hz
                    double fp = fc, fsb = fc;
                    const double dir = lowCut ? -1.0 : 1.0;
                    while (std::abs (fp - fc) < 0.9 * fc && mag (fp) < -0.1) fp -= dir * 0.25;
                    while (std::abs (fsb - fc) < 0.9 * fc && mag (fsb) > -96.0) fsb += dir * 0.25;
                    width[lowCut] = std::abs (fsb - fp);
                    for (int i = 0; i <= 300; ++i)
                    {
                        const double f = lowCut ? 1.0 + (fsb - 1.0) * i / 300.0 : fsb + (23900.0 - fsb) * i / 300.0;
                        if (f > 0.5) stop[lowCut] = std::max (stop[lowCut], mag (f));
                        const double g = lowCut ? fp + (20000.0 - fp) * i / 300.0 : 20.0 + (fp - 20.0) * i / 300.0;
                        pass[lowCut] = std::max (pass[lowCut], std::abs (mag (g)));
                    }
                }
                std::printf ("      %5d points : coupe-haut 5 kHz %.1f Hz (%.3f octave), bande coupée ≤ %.1f dB, bande passante ± %.3f dB ; coupe-bas 100 Hz %.1f Hz, ≤ %.1f dB, ± %.3f dB\n",
                             N, width[0], std::log2 ((5000.0 + width[0]) / 5000.0), stop[0], pass[0], width[1], stop[1], pass[1]);
                // seuils : bande coupée ≤ −96 dB, bande passante ± 0,1 dB, transition ≤ 16 fs / N ; coupe-bas 100 Hz : à partir de 8192 points
                // (à 4096 points la transition, 174 Hz, est plus large que la bande coupée elle-même)
                CHECK (stop[0] <= -96.0 && pass[0] <= 0.1 && width[0] <= 16.0 * 48000.0 / N, "Brickwall en Linear phase, %d points : %.1f dB, ± %.3f dB, %.1f Hz", N, stop[0], pass[0], width[0]);
                CHECK (N < 8192 || (stop[1] <= -96.0 && pass[1] <= 0.1 && width[1] <= 16.0 * 48000.0 / N), "Brickwall coupe-bas en Linear phase, %d points : %.1f dB, ± %.3f dB, %.1f Hz", N, stop[1], pass[1], width[1]);
            }
        }
    }

    // ── 21. Seuil dynamique automatique ──────────────────────────────────
    std::printf ("28. Seuil automatique : le seuil suit le niveau moyen du détecteur (constante de temps %.0f s), seuil = moyenne + %.0f dB\n", autoThrSeconds, autoThrMarginDb);
    {
        const double fs = 48000;
        // sinus 1 kHz : 6 s au niveau de base, saut de +9 dB, puis 12 s à ce nouveau niveau ; plage −12 dB, attaque 1 ms
        auto run = [&] (double baseDb, int block, std::vector<float>& trace, std::vector<float>& out)
        {
            Engine e; e.prepare (fs);
            BandSpec b; b.active = true; b.shape = Shape::bell; b.freq = 1000; b.q = 2; b.gainDb = 0;
            DynSettings d; d.on = true; d.autoThr = true; d.thrDb = -30; d.rangeDb = -12; d.attackMs = 1; d.releaseMs = 50;
            e.setBand (0, b, d, 1.0);
            const int total = (int) (18 * fs);
            out.assign ((size_t) total, 0.0f); trace.clear();
            std::vector<float> r ((size_t) total);
            for (int i = 0; i < total; ++i)
            {
                const double a = std::pow (10.0, (baseDb + (i >= 6 * fs ? 9.0 : 0.0)) / 20.0);
                out[(size_t) i] = r[(size_t) i] = (float) (a * std::sin (2 * pi * 1000.0 * i / fs));
            }
            for (int off = 0; off < total; off += block)
            {
                const int n = std::min (block, total - off);
                e.process (out.data() + off, r.data() + off, n, false);
                if ((off / block) % std::max (1, 480 / block) == 0) trace.push_back (e.getDynOffset (0));
            }
            return e.getAutoThreshold (0);
        };
        std::vector<float> tr, o1, tr2, o2, tr3, o3, tr4, o4;
        const float thrEnd = run (-20, 480, tr, o1);
        auto at = [&] (const std::vector<float>& t, double sec) { return (double) t[(size_t) (sec * 100.0)]; };   // (une valeur toutes les 10 ms)
        const double start = at (tr, 0.05), rest = at (tr, 5.9), hit = at (tr, 6.03), later = at (tr, 17.9);
        double maxRest = 0; for (double s = 1.0; s < 5.95; s += 0.01) maxRest = std::max (maxRest, std::abs (at (tr, s)));
        std::printf ("   niveau stable −20 dBFS : gain dynamique %.3f dB au départ (50 ms), %.3f dB au repos (au plus %.3f dB entre 1 et 6 s) [seuil 0.1]\n", start, rest, maxRest);
        std::printf ("   saut de +9 dB : %.2f dB 30 ms après (attendu ≈ −6 : dépassement de 9 dB, seuil 3 dB au-dessus de la moyenne) [seuil −6 ± 0.5] ; %.3f dB 12 s après (le seuil a rejoint le nouveau niveau) [seuil 0.1]\n", hit, later);
        std::printf ("   seuil lu à la fin : %.2f dBFS (crête du détecteur −11 dBFS + 3 dB = −8 ; l'enveloppe moyenne est un peu sous la crête) [seuil −8 ± 1]\n", thrEnd);
        CHECK (std::abs (start) <= 0.1 && maxRest <= 0.1, "seuil automatique au repos : %.3f / %.3f dB", start, maxRest);
        CHECK (std::abs (hit + 6.0) <= 0.5, "seuil automatique, saut de +9 dB : %.2f dB", hit);
        CHECK (std::abs (later) <= 0.1, "seuil automatique, retour : %.3f dB", later);
        CHECK (std::abs (thrEnd + 8.0) <= 1.0, "seuil automatique lu : %.2f dBFS", thrEnd);
        // même comportement 30 dB plus bas (le seuil est relatif au niveau)
        run (-50, 480, tr2, o2);
        double dLvl = 0; for (size_t i = 0; i < tr.size(); ++i) dLvl = std::max (dLvl, (double) std::abs (tr[i] - tr2[i]));
        // déterminisme : deux passes identiques bit à bit ; blocs de 480 ou de 96 échantillons
        run (-20, 480, tr3, o3);
        bool same = o1 == o3;
        run (-20, 96, tr4, o4);
        double dBlk = 0; for (size_t i = 0; i < o1.size(); ++i) dBlk = std::max (dBlk, (double) std::abs (o1[i] - o4[i]));
        std::printf ("   même signal 30 dB plus bas : gain dynamique identique à %.4f dB près [seuil 0.10] ; deux passes : sorties %s ; blocs de 96 au lieu de 480 : écart max %.1f dB [seuil −130]\n",
                     dLvl, same ? "identiques bit à bit" : "DIFFÉRENTES", db (dBlk));
        CHECK (dLvl <= 0.10, "seuil automatique : dépend du niveau (%.4f dB)", dLvl);
        CHECK (same, "seuil automatique : sortie non déterministe");
        CHECK (db (dBlk) <= -130.0, "seuil automatique : dépend de la taille des blocs (%.1f dB)", db (dBlk));
        // seuil automatique coupé : rien ne change par rapport au seuil manuel (même sortie qu'avec le champ par défaut)
        {
            auto manual = [&] (bool touch)
            {
                Engine e; e.prepare (fs);
                BandSpec b; b.active = true; b.shape = Shape::bell; b.freq = 1000; b.q = 2;
                DynSettings d; d.on = true; d.thrDb = -30; d.rangeDb = -12; d.attackMs = 1; d.releaseMs = 50;
                if (touch) { d.autoThr = true; e.setBand (0, b, d, 1.0); std::vector<float> z (4800, 0.1f), z2 (4800, 0.1f); e.process (z.data(), z2.data(), 4800, false); e.prepare (fs); d.autoThr = false; }
                e.setBand (0, b, d, 1.0);
                std::vector<float> l (48000), r (48000);
                for (int i = 0; i < 48000; ++i) l[(size_t) i] = r[(size_t) i] = (float) (0.3 * std::sin (2 * pi * 1000.0 * i / fs) * (i > 20000 ? 1.0 : 0.05));
                e.process (l.data(), r.data(), 48000, false);
                return l;
            };
            const bool eq = manual (false) == manual (true);
            std::printf ("   seuil automatique coupé : le seuil manuel donne la même sortie qu'avant, bit à bit : %s\n", eq ? "oui" : "NON");
            CHECK (eq, "le seuil manuel a changé");
        }
    }

    // ── 22. Stabilité et coût des nouvelles formes ───────────────────────
    std::printf ("29. Stabilité avec les formes et pentes 3.0 : 100 000 réglages aléatoires (10 formes, pentes libres 6–96 dB/oct, Brickwall, 44,1–192 kHz)\n");
    {
        std::mt19937 rng (7); std::uniform_real_distribution<double> U (0, 1);
        int bad = 0, maxN = 0;
        const double rr[] = { 44100, 48000, 88200, 96000, 176400, 192000 };
        for (int it = 0; it < 100000; ++it)
        {
            BandSpec b; b.active = true; b.shape = (Shape) (int) (U (rng) * numShapes); b.freq = 20 * std::pow (1000.0, U (rng));
            b.gainDb = U (rng) * 120 - 60; b.q = 0.025 * std::pow (1600.0, U (rng)); b.slope = slopeValues[(int) (U (rng) * 9)];
            if (U (rng) < 0.6) b.slopeFree = U (rng) * 96.0;
            b.brick = U (rng) < 0.15;
            for (int an = 0; an < 2; ++an)
            {
                Design d; design (b, rr[it % 6], d, an == 1);
                maxN = std::max (maxN, d.n);
                bool ok = d.n <= maxSections && std::isfinite (d.outGain);
                for (int i = 0; i < d.n && ok; ++i)
                {
                    const auto& s = d.s[(size_t) i];
                    ok = s.g > 0 && std::isfinite (s.g) && (s.kind == Section::svf ? (s.k > 0 && std::isfinite (s.m0 + s.m1 + s.m2)) : std::isfinite (s.cLp + s.cHp));
                }
                if (! ok) { ++bad; break; }
            }
        }
        std::printf ("   sections instables ou non finies (numérique et cible des FIR) : %d [seuil 0] ; %d sections au plus par bande (maximum prévu : %d)\n", bad, maxN, maxSections);
        CHECK (bad == 0, "%d conceptions 3.0 instables", bad);
        Engine eng; eng.prepare (44100);
        double peak = 0; bool nan = false;
        std::vector<float> l (256), r (256), sc (256);
        for (int it = 0; it < 3000; ++it)
        {
            if (it % 5 == 0)
                for (int bnd = 0; bnd < 8; ++bnd)
                {
                    BandSpec s; s.active = U (rng) < 0.8; s.shape = (Shape) (int) (U (rng) * numShapes); s.slope = slopeValues[(int) (U (rng) * 9)];
                    s.place = (Place) (int) (U (rng) * 5); s.freq = 10 * std::pow (2000.0, U (rng)); s.gainDb = U (rng) * 40 - 20; s.q = 0.1 * std::pow (100.0, U (rng));
                    if (U (rng) < 0.5) s.slopeFree = U (rng) * 96.0;
                    s.brick = U (rng) < 0.2;
                    DynSettings d; d.on = U (rng) < 0.5; d.thrDb = -60 * U (rng); d.rangeDb = U (rng) * 40 - 20; d.attackMs = 0.5 + 20 * U (rng); d.external = U (rng) < 0.3; d.autoThr = U (rng) < 0.5;
                    eng.setBand (bnd, s, d, 1.0);
                }
            for (int i = 0; i < 256; ++i) { l[(size_t) i] = (float) (U (rng) - 0.5); r[(size_t) i] = (float) (U (rng) - 0.5); sc[(size_t) i] = (float) (U (rng) - 0.5); }
            eng.process (l.data(), r.data(), 256, false, sc.data(), nullptr);
            for (int i = 0; i < 256; ++i)
            {
                if (! std::isfinite (l[(size_t) i]) || ! std::isfinite (r[(size_t) i])) nan = true;
                peak = std::max (peak, (double) std::max (std::abs (l[(size_t) i]), std::abs (r[(size_t) i])));
            }
        }
        std::printf ("   automatisation aléatoire (8 bandes, nouvelles formes, pentes libres, Brickwall, seuil automatique) : crête %.1f, NaN : %s [seuil < 600]\n", peak, nan ? "oui" : "non");
        CHECK (! nan && peak < 600, "moteur instable sous automatisation (formes 3.0)");

        // coût (durées indicatives, mesurées ici)
        std::printf ("   coût : conception (µs) et traitement d'une bande en stéréo à 44,1 kHz (%% d'un cœur, 10 s d'audio, meilleur de 3 passes)\n");
        struct C { const char* name; BandSpec b; };
        auto mk = [] (Shape s, double f, double g, double q, int slope, double sf, bool br) { BandSpec b; b.active = true; b.shape = s; b.freq = f; b.gainDb = g; b.q = q; b.slope = slope; b.slopeFree = sf; b.brick = br; return b; };
        const C cs[] = { { "cloche 1 kHz +6 dB (repère)       ", mk (Shape::bell, 1000, 6, 1, 12, 0, false) },
                         { "coupe-haut 24 dB/oct 8 kHz (repère)", mk (Shape::highCut, 8000, 0, 0.71, 24, 0, false) },
                         { "Flat tilt +6 dB                   ", mk (Shape::flatTilt, 1000, 6, 1, 12, 0, false) },
                         { "All pass ordre 2                  ", mk (Shape::allPass, 1000, 0, 2, 12, 0, false) },
                         { "coupe-haut 27,3 dB/oct à 8 kHz    ", mk (Shape::highCut, 8000, 0, 0.71, 12, 27.3, false) },
                         { "coupe-bas 9,5 dB/oct à 80 Hz      ", mk (Shape::lowCut, 80, 0, 0.71, 12, 9.5, false) },
                         { "coupe-haut 90,5 dB/oct à 16 kHz   ", mk (Shape::highCut, 16000, 0, 0.71, 12, 90.5, false) },
                         { "Brickwall coupe-haut 16 kHz       ", mk (Shape::highCut, 16000, 0, 0.71, 12, 0, true) } };
        for (const auto& c : cs)
        {
            Design d; double acc = 0;
            const int reps = 300;
            auto t0 = clk::now();
            for (int i = 0; i < reps; ++i) { BandSpec b = c.b; b.freq *= 1.0 + 1e-4 * i; if (hasGain (b.shape)) b.gainDb += 1e-3 * i; design (b, 44100, d, false); acc += d.outGain; }
            const double us = std::chrono::duration<double, std::micro> (clk::now() - t0).count() / reps;
            double best = 1e9;
            for (int pass = 0; pass < 3; ++pass)
            {
                Engine e; e.prepare (44100);
                e.setBand (0, c.b, DynSettings(), 1.0);
                std::vector<float> l2 (512), r2 (512);
                std::mt19937 g2 (3); std::uniform_real_distribution<float> V (-0.1f, 0.1f);
                t0 = clk::now();
                for (int k = 0; k < 861; ++k) { for (int i = 0; i < 512; ++i) { l2[(size_t) i] = V (g2); r2[(size_t) i] = V (g2); } e.process (l2.data(), r2.data(), 512, false); acc += l2[0]; }
                best = std::min (best, std::chrono::duration<double> (clk::now() - t0).count());
            }
            static double empty = -1;
            if (empty < 0)
            {
                // (temps du bruit seul et du moteur à vide, retiré des mesures)
                Engine e; e.prepare (44100);
                std::vector<float> l2 (512), r2 (512);
                std::mt19937 g2 (3); std::uniform_real_distribution<float> V (-0.1f, 0.1f);
                empty = 1e9;
                for (int pass = 0; pass < 3; ++pass)
                {
                    t0 = clk::now();
                    for (int k = 0; k < 861; ++k) { for (int i = 0; i < 512; ++i) { l2[(size_t) i] = V (g2); r2[(size_t) i] = V (g2); } e.process (l2.data(), r2.data(), 512, false); acc += l2[0]; }
                    empty = std::min (empty, std::chrono::duration<double> (clk::now() - t0).count());
                }
            }
            std::printf ("      %s : %d sections, conception %7.1f µs, traitement %.3f %%%s\n", c.name, d.n, us, std::max (0.0, best - empty) * 10.0, acc == 12345.678 ? "." : "");
        }
        // Flat tilt dont le gain est automatisé sans arrêt (une conception par sous-bloc de 32 échantillons)
        {
            double best = 1e9, acc = 0;
            for (int pass = 0; pass < 3; ++pass)
            {
                Engine e; e.prepare (44100);
                std::vector<float> l2 (512), r2 (512);
                std::mt19937 g2 (3); std::uniform_real_distribution<float> V (-0.1f, 0.1f);
                auto t0 = clk::now();
                for (int k = 0; k < 861; ++k)
                {
                    BandSpec b; b.active = true; b.shape = Shape::flatTilt; b.freq = 1000; b.gainDb = 12.0 * std::sin (k * 0.05);
                    e.setBand (0, b, DynSettings(), 1.0);
                    for (int i = 0; i < 512; ++i) { l2[(size_t) i] = V (g2); r2[(size_t) i] = V (g2); }
                    e.process (l2.data(), r2.data(), 512, false); acc += l2[0];
                }
                best = std::min (best, std::chrono::duration<double> (clk::now() - t0).count());
            }
            std::printf ("      Flat tilt dont le gain bouge sans arrêt (±12 dB) : %.2f %% d'un cœur, bruit et moteur à vide compris%s\n", best * 10.0, acc == 12345.678 ? "." : "");
        }
    }
}

// ── Outils des sections 30 à 34 (Auto gain à sonie égale, pré-écho, résolution Auto) ──
// Bruit rose stéréo (filtre de Voss-McCartney à 7 étages), voies en partie corrélées, niveau ≈ −20 dBFS efficace
static void pinkStereo (std::vector<float>& l, std::vector<float>& r, int n, unsigned seed, double corr = 0.6)
{
    std::mt19937 rng (seed); std::normal_distribution<double> g (0.0, 1.0);
    auto gen = [&] (std::vector<double>& x)
    {
        // somme de bruits blancs filtrés (Paul Kellet) : pente −3 dB/oct à ± 0,05 dB de 10 Hz à Nyquist
        double b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
        x.resize ((size_t) n);
        for (int i = 0; i < n; ++i)
        {
            const double w = g (rng);
            b0 = 0.99886 * b0 + w * 0.0555179; b1 = 0.99332 * b1 + w * 0.0750759; b2 = 0.96900 * b2 + w * 0.1538520;
            b3 = 0.86650 * b3 + w * 0.3104856; b4 = 0.55000 * b4 + w * 0.5329522; b5 = -0.7616 * b5 - w * 0.0168980;
            x[(size_t) i] = b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362; b6 = w * 0.115926;
        }
    };
    std::vector<double> a, b; gen (a); gen (b);
    double ss = 0; for (double v : a) ss += v * v;
    const double k = 0.1 / std::sqrt (ss / n);
    l.resize ((size_t) n); r.resize ((size_t) n);
    for (int i = 0; i < n; ++i)
    {
        l[(size_t) i] = (float) (k * a[(size_t) i]);
        r[(size_t) i] = (float) (k * (corr * a[(size_t) i] + std::sqrt (1.0 - corr * corr) * b[(size_t) i]));
    }
}

// Sonie intégrée d'un signal stéréo (ou mono si r est nul) entre deux instants
static double integratedLufs (const float* l, const float* r, int n, double fs)
{
    audiolab::loud::Meter m; m.prepare (fs, n / fs + 1.0);
    m.process (l, r, n);
    return m.integrated();
}

// Le plugin en zéro latence (moteur) avec Auto gain, comme PluginProcessor::processBlock : mesure sur
// l'entrée et la sortie avant gain, gain lissé sur 30 ms. Renvoie le gain (dB) à chaque bloc.
struct GainRig
{
    Engine eng; AutoGain ag;
    double fs = 48000, gLin = 1.0, gStep = 0.0; int gLeft = 0;
    std::array<BandSpec, maxBands> bands {};
    std::array<DynSettings, maxBands> dyn {};
    bool applyGain = true;
    void start (double rate)
    {
        fs = rate; eng.prepare (fs); ag.prepare (fs);
        push(); ag.setRunning (true);
        gLin = std::pow (10.0, ag.gainDb() / 20.0); gLeft = 0;
    }
    void push() { for (int b = 0; b < maxBands; ++b) eng.setBand (b, bands[(size_t) b], dyn[(size_t) b], 1.0); ag.setCurve (bands, true, false); }
    // traite n échantillons sur place (blocs de 512) ; pre : copie de la sortie avant gain (peut être nul)
    void run (float* l, float* r, int n, std::vector<float>* gTrace = nullptr, float* preL = nullptr, float* preR = nullptr)
    {
        std::vector<float> il (512), ir (512);
        for (int off = 0; off < n; off += 512)
        {
            const int m = std::min (512, n - off);
            push();
            std::copy (l + off, l + off + m, il.begin()); std::copy (r + off, r + off + m, ir.begin());
            eng.process (l + off, r + off, m, false);
            ag.process (il.data(), ir.data(), l + off, r + off, m);
            if (preL != nullptr) { std::copy (l + off, l + off + m, preL + off); std::copy (r + off, r + off + m, preR + off); }
            const double target = std::pow (10.0, ag.gainDb() / 20.0);
            if (std::abs (target - (gLin + gStep * gLeft)) > 1e-12) { gLeft = (int) (0.03 * fs); gStep = (target - gLin) / gLeft; }
            for (int i = 0; i < m; ++i)
            {
                if (gLeft > 0) { gLin += gStep; --gLeft; }
                if (applyGain) { l[off + i] = (float) (l[off + i] * gLin); r[off + i] = (float) (r[off + i] * gLin); }
            }
            if (gTrace != nullptr) gTrace->push_back (ag.gainDb());
        }
    }
};

static BandSpec mk (Shape s, double f, double g, double q, int slope = 12, Place p = Place::stereo, double slopeFree = 0, bool brick = false)
{
    BandSpec b; b.active = true; b.shape = s; b.freq = f; b.gainDb = g; b.q = q; b.slope = slope; b.place = p; b.slopeFree = slopeFree; b.brick = brick;
    return b;
}

static std::vector<float> pinkL, pinkR;
static void testGain30()
{
    namespace L = audiolab::loud;
    // ── 30. Mesureur de sonie ITU-R BS.1770-4 ────────────────────────────
    std::printf ("30. Mesureur de sonie ITU-R BS.1770-4 (pondération K recalculée pour chaque fréquence d'échantillonnage)\n");
    {
        L::Biquad s, h; L::kWeighting (48000.0, s, h);
        const double ref[10] = { 1.53512485958697, -2.69169618940638, 1.19839281085285, -1.69065929318241, 0.73248077421585,
                                 1.0, -2.0, 1.0, -1.99004745483398, 0.99007225036621 };
        const double got[10] = { s.b0, s.b1, s.b2, s.a1, s.a2, h.b0, h.b1, h.b2, h.a1, h.a2 };
        double e = 0; for (int i = 0; i < 10; ++i) e = std::max (e, std::abs (got[i] - ref[i]));
        std::printf ("   coefficients à 48 kHz : écart max aux coefficients de la norme %.1e [seuil 1e-9]\n", e);
        CHECK (e < 1e-9, "coefficients de la pondération K à 48 kHz (%.2e)", e);
        const double amp = std::pow (10.0, -23.0 / 20.0);
        double worst997 = 0, worstK = 0, worstCurve = 0;
        for (double fs : { 44100.0, 48000.0, 96000.0, 192000.0 })
        {
            const int n = (int) (12 * fs);
            std::vector<float> x ((size_t) n);
            double res[3];
            const double fr[3] = { 997.0, 100.0, 10000.0 };
            for (int k = 0; k < 3; ++k)
            {
                for (int i = 0; i < n; ++i) x[(size_t) i] = (float) (amp * std::sin (2 * pi * fr[k] * i / fs));
                res[k] = integratedLufs (x.data(), x.data(), n, fs);
            }
            // théorie : −23 dBFS sur deux voies = −23 − 0,691 + K(f) LUFS, K(f) = courbe des coefficients de la norme
            const double e100 = res[1] - (-23.0 - 0.691 + L::kReferenceDb (100.0)), e10k = res[2] - (-23.0 - 0.691 + L::kReferenceDb (10000.0));
            double curve = 0;   // écart entre la pondération recalculée et celle de la norme, 20 Hz – 20 kHz
            for (int i = 0; i < 200; ++i) { const double f = 20.0 * std::pow (1000.0, i / 199.0); curve = std::max (curve, std::abs (L::kGainDb (f, fs) - L::kReferenceDb (f))); }
            std::printf ("   %5.1f kHz : sinus 997 Hz −23 dBFS stéréo → %.3f LUFS ; 100 Hz → %.3f LUFS (écart à la courbe K %+.3f dB) ; 10 kHz → %.3f LUFS (%+.3f dB) ;\n"
                         "               courbe K recalculée contre celle de la norme, 20 Hz – 20 kHz : écart max %.3f dB\n",
                         fs / 1000, res[0], res[1], e100, res[2], e10k, curve);
            worst997 = std::max (worst997, std::abs (res[0] + 23.0));
            worstK = std::max (worstK, std::max (std::abs (e100), std::abs (e10k)));
            worstCurve = std::max (worstCurve, curve);
        }
        std::printf ("   écart max : 997 Hz %.3f LU [seuil 0.1] ; 100 Hz et 10 kHz contre la courbe K %.3f dB [seuil 0.1] ; courbe entière %.3f dB (information)\n", worst997, worstK, worstCurve);
        CHECK (worst997 <= 0.1, "sinus 997 Hz −23 dBFS (%.3f LU)", worst997);
        CHECK (worstK <= 0.1, "courbe K à 100 Hz / 10 kHz (%.3f dB)", worstK);
        // une seule voie : −3,01 LU ; portes : silence et passage 20 dB plus bas sans effet
        const double fs = 48000; const int n = (int) (10 * fs);
        std::vector<float> x ((size_t) (2 * n)), z ((size_t) (2 * n), 0.0f);
        for (int i = 0; i < 2 * n; ++i) x[(size_t) i] = (float) (amp * std::sin (2 * pi * 997.0 * i / fs) * (i < n ? 1.0 : 0.0));
        const double oneCh = integratedLufs (x.data(), z.data(), n, fs), withSilence = integratedLufs (x.data(), x.data(), 2 * n, fs);
        for (int i = n; i < 2 * n; ++i) x[(size_t) i] = (float) (0.1 * amp * std::sin (2 * pi * 997.0 * i / fs));
        const double withQuiet = integratedLufs (x.data(), x.data(), 2 * n, fs);
        std::printf ("   une seule voie : %.3f LUFS (attendu −26.01) ; suivi de 10 s de silence : %.3f (porte absolue) ; suivi de 10 s à −43 dBFS : %.3f (porte relative) [−23.0 ± 0.1]\n",
                     oneCh, withSilence, withQuiet);
        CHECK (std::abs (oneCh + 26.01) <= 0.1 && std::abs (withSilence + 23.0) <= 0.1 && std::abs (withQuiet + 23.0) <= 0.1, "voies et portes du mesureur");
    }

    // ── 31. Auto gain à sonie égale : 6 réglages sur du bruit rose ───────
    std::printf ("31. Auto gain à sonie égale, bruit rose stéréo à 48 kHz, 40 s par réglage (sonie intégrée BS.1770 de 10 à 40 s)\n"
                 "    avant = méthode 2.0 (moyenne de la courbe en dB, sans mesure) ; après = 3.0 ; convergence = dernier instant où le gain est à plus de 0,1 dB de sa valeur finale\n");
    const double fs = 48000;
    const int nTot = (int) (40 * fs), nSkip = (int) (10 * fs);
    pinkStereo (pinkL, pinkR, nTot, 21);
    const auto& pl = pinkL; const auto& pr = pinkR;
    struct GCase { const char* name; std::vector<BandSpec> b; int dynBand = -1; bool dark = false; };
    std::vector<GCase> cases;
    cases.push_back ({ "étagère grave +9 dB à 100 Hz", { mk (Shape::lowShelf, 100, 9, 0.71) } });
    cases.push_back ({ "coupe-bas 120 Hz 24 dB/oct", { mk (Shape::lowCut, 120, 0, 0.71, 24) } });
    cases.push_back ({ "cloche +12 dB à 3 kHz Q 1", { mk (Shape::bell, 3000, 12, 1.0) } });
    cases.push_back ({ "tilt +6 dB à 1 kHz", { mk (Shape::tiltShelf, 1000, 6, 0.71, 6) } });
    cases.push_back ({ "mixte (coupe-bas 40 Hz, −4 dB 250 Hz, +3 dB 2,5 kHz Mid, +5 dB 8 kHz Side)",
                       { mk (Shape::lowCut, 40, 0, 0.71, 18), mk (Shape::bell, 250, -4, 1.4), mk (Shape::bell, 2500, 3, 0.9, 12, Place::mid), mk (Shape::highShelf, 8000, 5, 0.71, 12, Place::side) } });
    cases.push_back ({ "bande dynamique en action (cloche 1 kHz Q 1, plage −9 dB, seuil −45 dB)", { mk (Shape::bell, 1000, 0, 1.0) }, 0 });
    cases.push_back ({ "étagère grave +9 dB à 100 Hz, signal au grave dominant (bruit rose filtré : −6 dB/oct au-dessus de 150 Hz)", { mk (Shape::lowShelf, 100, 9, 0.71) }, -1, true });
    double worstAfter = 0, worstConv = 0;
    // (signal au spectre non rose, pour que la prévision de départ, faite sur un spectre rose, soit fausse)
    std::vector<float> dl = pl, dr = pr;
    {
        double zl = 0, zr = 0; const double a = std::exp (-2 * pi * 150.0 / fs);
        for (int i = 0; i < nTot; ++i) { zl = a * zl + (1 - a) * dl[(size_t) i]; zr = a * zr + (1 - a) * dr[(size_t) i]; dl[(size_t) i] = (float) (3.0 * zl); dr[(size_t) i] = (float) (3.0 * zr); }
    }
    for (auto& c : cases)
    {
        const auto& pl = c.dark ? dl : ::pinkL; const auto& pr = c.dark ? dr : ::pinkR;
        GainRig rig;
        for (size_t i = 0; i < c.b.size(); ++i) rig.bands[i] = c.b[i];
        if (c.dynBand >= 0) { auto& d = rig.dyn[(size_t) c.dynBand]; d.on = true; d.thrDb = -45; d.rangeDb = -9; d.attackMs = 10; d.releaseMs = 150; }
        rig.start (fs);
        const double gStart = rig.ag.gainDb();
        auto l = pl, r = pr;
        std::vector<float> preL ((size_t) nTot), preR ((size_t) nTot), gt;
        rig.run (l.data(), r.data(), nTot, &gt, preL.data(), preR.data());
        const double lin = integratedLufs (pl.data() + nSkip, pr.data() + nSkip, nTot - nSkip, fs);
        const double lpre = integratedLufs (preL.data() + nSkip, preR.data() + nSkip, nTot - nSkip, fs);
        const double lout = integratedLufs (l.data() + nSkip, r.data() + nSkip, nTot - nSkip, fs);
        const double legacy = legacyAutoGainDb (rig.bands, fs);
        const double gEnd = gt.back();
        int last = -1; double wob = 0;
        for (size_t i = 0; i < gt.size(); ++i)
        {
            if (std::abs (gt[i] - gEnd) > 0.1) last = (int) i;
            if ((double) i * 512 / fs >= 10.0) wob = std::max (wob, (double) std::abs (gt[i] - gEnd));
        }
        const double conv = (last + 1) * 512 / fs;
        std::printf ("   %s :\n      sonie entrée %.2f LUFS, sortie sans compensation %+.2f LU ; avant : gain %+.2f dB → écart %+.2f LU ; après : gain %+.2f dB (départ %+.2f, d'après la courbe) → écart %+.3f LU ;\n"
                     "      convergence en %.1f s ; mouvement du gain ensuite : ± %.3f dB\n",
                     c.name, lin, lpre - lin, legacy, lpre + legacy - lin, gEnd, gStart, lout - lin, conv, wob);
        worstAfter = std::max (worstAfter, std::abs (lout - lin)); worstConv = std::max (worstConv, conv);
    }
    std::printf ("   écart de sonie sortie / entrée après convergence, pire réglage : %.3f LU [seuil 0.1] ; convergence la plus lente : %.1f s [seuil 6]\n", worstAfter, worstConv);
    CHECK (worstAfter <= 0.1, "Auto gain : écart de sonie %.3f LU", worstAfter);
    CHECK (worstConv <= 6.0, "Auto gain : convergence en %.1f s", worstConv);

    // ── 32. Auto gain : silence, mouvement d'une bande, bornes, coût ─────
    std::printf ("32. Auto gain : silence, mouvement d'une bande, bornes, coût\n");
    {
        // (a) 5 s de silence après 20 s de bruit rose : le gain ne bouge pas
        for (int kind = 0; kind < 2; ++kind)
        {
            GainRig rig; rig.bands[0] = mk (Shape::lowShelf, 100, 9, 0.71); rig.start (fs);
            const int n1 = (int) (20 * fs) + 1234, n2 = (int) (5 * fs);
            auto l = pl, r = pr; l.resize ((size_t) (n1 + n2)); r.resize ((size_t) (n1 + n2));
            std::mt19937 rng (4); std::uniform_real_distribution<float> U (-1.0f, 1.0f);
            for (int i = n1; i < n1 + n2; ++i) { l[(size_t) i] = kind == 0 ? 0.0f : 3.0e-5f * U (rng); r[(size_t) i] = kind == 0 ? 0.0f : 3.0e-5f * U (rng); }
            // (échantillon par échantillon autour du début du silence : le gain est lu après chaque échantillon)
            rig.run (l.data(), r.data(), n1);
            const double g0 = rig.ag.gainDb();
            double move = 0;
            std::vector<float> il (64), ir (64);
            for (int off = n1; off < n1 + n2; off += 64)
            {
                std::copy (l.begin() + off, l.begin() + off + 64, il.begin()); std::copy (r.begin() + off, r.begin() + off + 64, ir.begin());
                rig.eng.process (l.data() + off, r.data() + off, 64, false);
                rig.ag.process (il.data(), ir.data(), l.data() + off, r.data() + off, 64);
                move = std::max (move, std::abs ((double) rig.ag.gainDb() - g0));
            }
            // puis le signal revient : le gain repart du même point
            auto l2 = pl, r2 = pr; std::vector<float> gt;
            rig.run (l2.data(), r2.data(), (int) (3 * fs), &gt);
            double back = 0; for (float g : gt) back = std::max (back, std::abs ((double) g - g0));
            std::printf ("   %s de 5 s après 20 s de signal : mouvement du gain %.6f dB [seuil %s] ; au retour du signal (3 s) : ± %.3f dB\n",
                         kind == 0 ? "silence numérique" : "bruit de fond à −90 dBFS", move, kind == 0 ? "0" : "0.01", back);
            if (kind == 0) CHECK (move == 0.0, "le gain bouge pendant un silence (%.6f dB)", move);
            else CHECK (move <= 0.01, "le gain bouge pendant un bruit de fond sous la porte (%.4f dB)", move);
            CHECK (back <= 0.3, "saut du gain au retour du signal (%.3f dB)", back);
        }
        // (b) une bande bouge d'un coup, puis par un glissement de 2 s : décalage immédiat, puis convergence
        {
            GainRig rig; rig.bands[0] = mk (Shape::bell, 1000, 0, 1.0); rig.start (fs);
            auto l = pl, r = pr;
            const int n1 = (int) (15 * fs);
            rig.run (l.data(), r.data(), n1);
            const double gA = rig.ag.gainDb();
            rig.bands[0].gainDb = 12.0; rig.push();
            const double gNow = rig.ag.gainDb();
            std::vector<float> gt;
            rig.run (l.data() + n1, r.data() + n1, (int) (15 * fs), &gt);
            const double gEnd = gt.back();
            double exc = 0; for (float g : gt) exc = std::max (exc, std::abs ((double) g - gEnd));
            std::printf ("   cloche 1 kHz Q 1 de 0 à +12 dB d'un coup : gain %+.2f → %+.2f dB dans le même bloc (prévision), %+.2f dB après convergence :\n"
                         "      écart de la prévision immédiate %.3f dB [seuil 0.5], écart max du gain à sa valeur finale ensuite %.3f dB\n", gA, gNow, gEnd, std::abs (gNow - gEnd), exc);
            CHECK (std::abs (gNow - gA) > 1.0, "le gain ne suit pas tout de suite le mouvement d'une bande");
            CHECK (std::abs (gNow - gEnd) <= 0.5 && exc <= 0.5, "prévision immédiate trop loin de la mesure (%.3f dB)", std::max (std::abs (gNow - gEnd), exc));

            // glissement : gain de la cloche de +12 à −12 dB en 2 s (un pas par bloc), sonie momentanée pendant et après
            GainRig rg; rg.bands[0] = mk (Shape::bell, 1000, 12, 1.0); rg.start (fs);
            auto l2 = pl, r2 = pr;
            rg.run (l2.data(), r2.data(), n1);
            const int nDrag = (int) (2 * fs), nAfter = (int) (4 * fs);
            for (int off = 0; off < nDrag; off += 512)
            {
                rg.bands[0].gainDb = 12.0 - 24.0 * std::min (1.0, (double) (off + 512) / nDrag);
                rg.run (l2.data() + n1 + off, r2.data() + n1 + off, std::min (512, nDrag - off));
            }
            rg.run (l2.data() + n1 + nDrag, r2.data() + n1 + nDrag, nAfter);
            // sonie momentanée (400 ms) de la sortie contre l'entrée, toutes les 100 ms, du début du glissement à 4 s après
            L::Meter mi, mo; mi.prepare (fs); mo.prepare (fs);
            double worst = 0;
            const int hop = 4800, s0 = n1 - 4 * hop;
            for (int off = s0; off + hop <= n1 + nDrag + nAfter; off += hop)
            {
                mi.process (pl.data() + off, pr.data() + off, hop); mo.process (l2.data() + off, r2.data() + off, hop);
                if (off >= n1) worst = std::max (worst, std::abs (mo.momentary() - mi.momentary()));
            }
            std::printf ("   cloche 1 kHz glissée de +12 à −12 dB en 2 s : écart max de sonie momentanée sortie / entrée pendant le glissement et les 4 s suivantes %.2f LU [seuil 1.0]\n"
                         "      (sans Auto gain, la sonie irait de %+.1f à %+.1f LU par rapport à l'entrée)\n", worst, -gEnd, -(double) rg.ag.gainDb());
            CHECK (worst <= 1.0, "sonie pendant le glissement d'une bande (%.2f LU)", worst);
        }
        // (c) bornes : ± 24 dB
        {
            GainRig rig; rig.bands[0] = mk (Shape::bell, 1000, 30, 0.3); rig.bands[1] = mk (Shape::bell, 1000, 30, 0.3); rig.start (fs);
            auto l = pl, r = pr; for (auto& v : l) v *= 0.01f; for (auto& v : r) v *= 0.01f;
            std::vector<float> gt; rig.run (l.data(), r.data(), (int) (8 * fs), &gt);
            float lo = 0; for (float g : gt) lo = std::min (lo, g);
            std::printf ("   deux cloches larges de +30 dB : gain le plus bas %.2f dB [borne −24]\n", lo);
            CHECK (lo >= -24.0f && lo <= -23.9f, "borne du gain de compensation (%.2f dB)", lo);
        }
        // (d) coût : la mesure seule (pondération K ×4, FFT étalée du spectre), 20 s de bruit rose stéréo
        {
            AutoGain ag; ag.prepare (fs);
            std::array<BandSpec, maxBands> b {}; b[0] = mk (Shape::bell, 1000, 6, 1.0);
            ag.setCurve (b, true, false); ag.setRunning (true);
            double best = 1e9;
            const int n = (int) (20 * fs);
            for (int pass = 0; pass < 3; ++pass)
            {
                const auto t0 = std::chrono::steady_clock::now();
                for (int off = 0; off + 512 <= n; off += 512) ag.process (pl.data() + off, pr.data() + off, pr.data() + off, pl.data() + off, 512);
                best = std::min (best, std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count());
            }
            // changement de courbe : 24 bandes actives, une bande retouchée à chaque bloc
            for (int i = 0; i < maxBands; ++i) b[(size_t) i] = mk (i % 3 == 0 ? Shape::lowCut : Shape::bell, 40.0 * std::pow (1.3, i), 3, 1.0, i % 3 == 0 ? 48 : 12);
            ag.setCurve (b, true, false);
            const auto t1 = std::chrono::steady_clock::now();
            for (int k = 0; k < 2000; ++k) { b[5].gainDb = 3.0 + 0.001 * k; ag.setCurve (b, true, false); }
            const double us = 1e6 * std::chrono::duration<double> (std::chrono::steady_clock::now() - t1).count() / 2000;
            std::printf ("   coût de la mesure : %.2f %% d'un cœur à 48 kHz stéréo (%d opérations de FFT par échantillon, aucune pointe) ;\n"
                         "      courbe retouchée (24 bandes actives, une qui bouge) : %.0f µs par bloc où un réglage change (information)\n", 100.0 * best / 20.0, ag.unitsPerSample(), us);
        }
    }

    // ── 33. Pré-écho de la phase linéaire ────────────────────────────────
    std::printf ("33. Pré-écho des FIR (énergie cumulée avant le pic, rapportée au pic)\n");
    {
        // cas calculable : un pic et un écho 10 ms avant, 40 dB plus bas
        std::vector<double> h (8192, 0.0);
        h[4096] = 1.0; h[4096 - 480] = 0.01;
        const auto p = fir::measurePreEcho (h.data(), (const double*) nullptr, 8192, 48000.0);
        std::printf ("   écho de −40 dB placé 10 ms avant le pic : %.2f ms au-dessus de −60 dB, %.2f ms au-dessus de −90 dB, %.1f dB à 5 ms, %.1f dB à 20 ms [10 ; 10 ; −40 ; rien]\n",
                     p.ms60, p.ms90, p.dbAt5ms, p.dbAt20ms);
        CHECK (std::abs (p.ms60 - 10.0f) < 0.03f && std::abs (p.ms90 - 10.0f) < 0.03f && std::abs (p.dbAt5ms + 40.0f) < 0.01f && p.dbAt20ms == fir::preEchoFloorDb, "pré-écho d'un cas calculable");
        struct PC { const char* name; BandSpec b; fir::Mode mode; int N; };
        const PC pcs[] = {
            { "courbe plate, Linear Max", BandSpec(), fir::Mode::linear, 32768 },
            { "cloche 3 kHz +6 dB Q 1, Linear Low", mk (Shape::bell, 3000, 6, 1.0), fir::Mode::linear, 4096 },
            { "coupe-bas 80 Hz 24 dB/oct, Linear Medium", mk (Shape::lowCut, 80, 0, 0.71, 24), fir::Mode::linear, 8192 },
            { "cloche 40 Hz +6 dB Q 8, Linear Max", mk (Shape::bell, 40, 6, 8.0), fir::Mode::linear, 32768 },
            { "cloche 40 Hz +6 dB Q 8, Natural", mk (Shape::bell, 40, 6, 8.0), fir::Mode::natural, 32768 },
            { "coupe-bas 80 Hz 24 dB/oct, Natural", mk (Shape::lowCut, 80, 0, 0.71, 24), fir::Mode::natural, 8192 },
        };
        fir::PreEcho res[6];
        for (int i = 0; i < 6; ++i)
        {
            // par le code du plugin (LinearPhaseCore : conception, puis mesure faite par le fil de conception)
            TestLinear lp; lp.prepare (48000.0);
            TestLinear::Snapshot sn; sn.bands[0] = pcs[i].b;
            lp.design (sn, pcs[i].mode, pcs[i].N, false);
            lp.switchTo (pcs[i].mode, pcs[i].N);
            res[i] = lp.getPreEcho (pcs[i].mode, pcs[i].N);
            // mesure directe : réponse impulsionnelle de la convolution
            const int lat = lp.latencyFor (pcs[i].mode, pcs[i].N), len = lat + 4096;
            std::vector<float> l ((size_t) len, 0.0f), r ((size_t) len, 0.0f);
            l[0] = r[0] = 1.0f;
            lp.process (l.data(), r.data(), len);
            const auto direct = fir::measurePreEcho (l.data(), (const float*) nullptr, len, 48000.0);
            std::printf ("   %-42s : %7.2f ms > −60 dB, %7.2f ms > −90 dB, %7.1f dB à 5 ms, %7.1f dB à 20 ms ; réponse impulsionnelle mesurée : %7.2f / %7.2f ms ; « %s »\n",
                         pcs[i].name, res[i].ms60, res[i].ms90, res[i].dbAt5ms, res[i].dbAt20ms, direct.ms60, direct.ms90, fir::preEchoText (res[i]).c_str());
            CHECK (std::abs (direct.ms60 - res[i].ms60) <= 0.05f + 0.01f * res[i].ms60, "pré-écho annoncé / mesuré à −60 dB (%s : %.2f contre %.2f ms)", pcs[i].name, res[i].ms60, direct.ms60);
            // (à −90 dB la mesure directe passe par la convolution en simple précision : tolérance plus large)
            CHECK (std::abs (direct.ms90 - res[i].ms90) <= 0.5f + 0.05f * res[i].ms90, "pré-écho annoncé / mesuré à −90 dB (%s : %.2f contre %.2f ms)", pcs[i].name, res[i].ms90, direct.ms90);
        }
        CHECK (res[0].ms60 == 0.0f && res[0].ms90 == 0.0f && res[0].dbAt5ms == fir::preEchoFloorDb, "courbe plate : aucun pré-écho");
        CHECK (res[3].ms60 > 100.0f && res[3].ms60 > 20.0f * res[1].ms60, "cloche étroite dans le grave en Linear Max : pré-écho long (%.1f ms)", res[3].ms60);
        CHECK (res[4].ms60 <= 3.0f && res[5].ms60 <= 3.0f, "Natural phase : pré-écho au plus la petite latence (%.2f et %.2f ms)", res[4].ms60, res[5].ms60);
        CHECK (fir::preEchoText (res[0]) == std::string ("Pre-echo: none above \xe2\x88\x92" "60 dB"), "texte du pré-écho nul");
    }

    // ── 34. Résolution Auto vérifiée par la mesure ───────────────────────
    std::printf ("34. Linear phase, résolution Auto : écart entre la réponse réelle du FIR et la cible, 20 Hz – 20 kHz là où la cible dépasse −60 dB\n"
                 "    (hors du creux des notch sous −3 dB) ; avant = longueur de la règle seule ; après = règle vérifiée par la mesure (allongée jusqu'à 0,1 dB ou Max)\n");
    {
        struct AC { std::string name; std::array<BandSpec, maxBands> b; };
        std::vector<AC> acs;
        auto add1 = [&] (const std::string& n, const BandSpec& b) { AC a; a.name = n; a.b[0] = b; acs.push_back (a); };
        char s[96];
        for (double q : { 0.5, 1.0, 2.0, 5.0, 10.0, 20.0 }) for (double f : { 20.0, 30.0, 50.0, 100.0, 200.0 }) for (double g : { 12.0, -12.0, 3.0 })
        { std::snprintf (s, sizeof s, "cloche %g Hz Q %g %+g dB", f, q, g); add1 (s, mk (Shape::bell, f, g, q)); }
        for (int sl : { 12, 24, 48, 96 }) for (double f : { 20.0, 30.0, 60.0, 120.0 }) { std::snprintf (s, sizeof s, "coupe-bas %g Hz %d dB/oct", f, sl); add1 (s, mk (Shape::lowCut, f, 0, 0.71, sl)); }
        for (double f : { 30.0, 60.0, 120.0 }) for (double q : { 1.0, 5.0, 10.0 }) { std::snprintf (s, sizeof s, "notch %g Hz Q %g", f, q); add1 (s, mk (Shape::notch, f, 0, q)); }
        for (double f : { 30.0, 60.0, 120.0 }) for (double g : { 9.0, -9.0 }) for (int v = 0; v < 3; ++v)
        { std::snprintf (s, sizeof s, "étagère grave %g Hz %+g dB %s", f, g, v == 0 ? "6 dB/oct" : v == 1 ? "12 dB/oct" : "12 dB/oct Q 2"); add1 (s, mk (Shape::lowShelf, f, g, v == 2 ? 2.0 : 0.71, v == 0 ? 6 : 12)); }
        for (double g : { 6.0, -6.0, 12.0, -12.0 }) { std::snprintf (s, sizeof s, "Flat tilt %+g dB", g); add1 (s, mk (Shape::flatTilt, 1000, g, 0.71)); }
        for (double g : { 6.0, -6.0 }) { std::snprintf (s, sizeof s, "tilt 100 Hz %+g dB", g); add1 (s, mk (Shape::tiltShelf, 100, g, 0.71, 6)); }
        for (double sf : { 15.0, 40.5, 77.0 }) for (double f : { 40.0, 80.0 }) { std::snprintf (s, sizeof s, "coupe-bas %g Hz pente libre %g dB/oct", f, sf); add1 (s, mk (Shape::lowCut, f, 0, 0.71, 12, Place::stereo, sf)); }
        for (double f : { 30.0, 60.0, 120.0 }) { std::snprintf (s, sizeof s, "coupe-bas Brickwall %g Hz", f); add1 (s, mk (Shape::lowCut, f, 0, 0.71, 12, Place::stereo, 0, true)); }
        add1 ("coupe-haut 120 Hz 24 dB/oct", mk (Shape::highCut, 120, 0, 0.71, 24));
        add1 ("passe-bande 60 Hz Q 2", mk (Shape::bandPass, 60, 0, 2.0));
        { AC a; a.name = "grosse caisse (coupe-bas 30 Hz 24, +3 dB 60 Hz Q 1.4, −4 dB 350 Hz, +3 dB 4 kHz)";
          a.b[0] = mk (Shape::lowCut, 30, 0, 0.71, 24); a.b[1] = mk (Shape::bell, 60, 3, 1.4); a.b[2] = mk (Shape::bell, 350, -4, 1.5); a.b[3] = mk (Shape::bell, 4000, 3, 1.2); acs.push_back (a); }
        { AC a; a.name = "trois cloches graves (40 Hz +6 Q 3, 80 Hz −6 Q 3 Mid, 160 Hz +4 Q 2 Side)";
          a.b[0] = mk (Shape::bell, 40, 6, 3.0); a.b[1] = mk (Shape::bell, 80, -6, 3.0, 12, Place::mid); a.b[2] = mk (Shape::bell, 160, 4, 2.0, 12, Place::side); acs.push_back (a); }

        fir::Work w; fir::Result tmp; std::vector<fir::cd> sc;
        const std::array<double, maxBands> span {};
        // l'outil de mesure lui-même : contre une évaluation exacte de la réponse (somme directe), à 48 kHz
        {
            double worst = 0;
            for (size_t ci = 0; ci < acs.size(); ci += 7)
            {
                fir::Request rq; rq.bands = acs[ci].b; rq.fs = 48000; rq.N = 8192; rq.mode = fir::Mode::linear;
                fir::design (rq, tmp, w);
                double atHz = 0;
                const double e = fir::linearError (rq, tmp, sc, fir::autoFloorDb, &atHz);
                if (atHz <= 0) continue;
                Design des[maxBands];
                for (int i = 0; i < maxBands; ++i) if (rq.bands[(size_t) i].active) designFir (rq.bands[(size_t) i], rq.fs, des[i], true, rq.N);
                const double exact = std::abs (db (std::abs (fir::responseAt (tmp.h[0], rq.N / 2, rq.fs, atHz))) - db (std::abs (fir::targetAt (rq, des, atHz).a)));
                if (! tmp.matrix) worst = std::max (worst, std::abs (exact - e));
            }
            std::printf ("   outil de mesure (FFT + interpolation) contre la somme directe, au point le plus défavorable de %d réglages : écart %.4f dB [seuil 0.01]\n", (int) (acs.size() + 6) / 7, worst);
            CHECK (worst <= 0.01, "mesure de l'écart des FIR (%.4f dB)", worst);
        }
        for (double rate : { 44100.0, 48000.0, 96000.0, 192000.0 })
        {
            const int nMin = fir::minLength (rate), nMax = fir::maxLength (rate);
            int longer = 0, atMaxFail = 0, count[4] {}, bad = 0;
            double worstOk = 0, worstRule = 0, worstMax = 0, ms = 0; std::string worstRuleName, worstMaxName;
            std::vector<std::string> lines; std::string unreachable;
            for (auto& c : acs)
            {
                const int nRule = fir::chooseLength (fir::Res::automatic, c.b, rate, fir::Mode::linear);
                fir::Request rq; rq.bands = c.b; rq.fs = rate; rq.N = nRule; rq.mode = fir::Mode::linear;
                fir::design (rq, tmp, w);
                const double eRule = fir::linearError (rq, tmp, sc);
                double e = 0;
                const auto t0 = std::chrono::steady_clock::now();
                const int n = fir::verifiedAutoLength (c.b, span, true, rate, w, tmp, sc, &e);
                ms = std::max (ms, 1000.0 * std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count());
                int idx = 0; for (int len = nMin; len < n; len *= 2) ++idx;
                ++count[idx];
                if (n > nRule)
                {
                    ++longer;
                    if (rate == 48000.0)
                    {
                        std::snprintf (s, sizeof s, "%.3f dB à %d points → %.3f dB à %d points (latence %d → %d éch.)", eRule, nRule, e, n, nRule / 2 + fir::partitionSize (rate), n / 2 + fir::partitionSize (rate));
                        lines.push_back ("      " + c.name + " : " + s);
                    }
                }
                if (eRule > worstRule && nRule < nMax) { worstRule = eRule; worstRuleName = c.name; }
                if (e <= fir::autoTolDb) worstOk = std::max (worstOk, e);
                else
                {
                    if (n == nMax)
                    {
                        ++atMaxFail; if (e > worstMax) { worstMax = e; worstMaxName = c.name; }
                        if (rate == 48000.0) { std::snprintf (s, sizeof s, " %.2f dB", e); unreachable += (unreachable.empty() ? "" : " ; ") + c.name + s; }
                    }
                    else ++bad;
                }
            }
            std::printf ("   %5.1f kHz, %d réglages : longueurs choisies %d / %d / %d / %d (%d à %d points) ; la mesure allonge %d réglage(s) que la règle seule laissait à %.2f dB au pire (%s) ;\n"
                         "      tenus à 0,1 dB : %d, écart max %.3f dB [seuil 0.1] ; hors d'atteinte même à Max (Auto choisit alors Max) : %d, pire %.2f dB (%s) ; vérification la plus longue %.0f ms\n",
                         rate / 1000, (int) acs.size(), count[0], count[1], count[2], count[3], nMin, nMax, longer, worstRule, worstRuleName.c_str(),
                         (int) acs.size() - atMaxFail - bad, worstOk, atMaxFail, worstMax, worstMaxName.c_str(), ms);
            for (auto& ln : lines) std::printf ("%s\n", ln.c_str());
            if (! unreachable.empty()) std::printf ("      hors d'atteinte à 48 kHz (écart à la longueur Max, 32768 points) : %s\n", unreachable.c_str());
            CHECK (bad == 0, "résolution Auto à %.0f Hz : %d réglage(s) au-dessus de 0,1 dB sans être à la longueur Max", rate, bad);
            CHECK (worstOk <= fir::autoTolDb, "résolution Auto à %.0f Hz : %.3f dB", rate, worstOk);
        }
    }
}

int main (int argc, char** argv)
{
    // « ./qt gain » : seulement les sections 30 à 34 (Auto gain à sonie égale, pré-écho, résolution Auto)
    if (argc > 1 && std::string (argv[1]) == "gain")
    {
        testGain30();
        std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
        return failures == 0 ? 0 : 1;
    }
    // « ./qt 3.0 » : seulement les sections 17 à 22 (formes et pentes 3.0)
    if (argc > 1 && std::string (argv[1]) == "3.0")
    {
        testShapes30();
        std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
        return failures == 0 ? 0 : 1;
    }
    std::printf ("Lab EQ 3.0 : banc qualité (sans JUCE)\n\n");

    // ── 1. Matched + correction ─────────────────────────────────────────
    std::printf ("1. Zéro latence : écart max à l'analogique de 20 Hz à 20 kHz (1.x bilinéaire → 2.0 matched → 3.0 matched + correction)\n"
                 "   zone mesurée : tout le spectre (cloche, étagères, tilt) ; analogique ≥ −24 dB (coupes, passe-bande) ; ≥ −3 dB (notch)\n");
    {
        BandSpec b; b.active = true; b.shape = Shape::bell; b.freq = 16000; b.gainDb = 6; b.q = 1;
        Design d, a, l; design (b, 44100, d, false); design (b, 44100, a, true); designBilinear (b, 44100, l);
        const double v = magnitudeDb (d, 20000, 44100, false), va = magnitudeDb (a, 20000, 44100, true), vl = magnitudeDb (l, 20000, 44100, false);
        std::printf ("   cloche 16 kHz +6 dB Q 1 à 20 kHz (44,1 kHz) : analogique %+.2f dB ; 1.x %+.2f ; 3.0 %+.3f\n", va, vl, v);
        CHECK (std::abs (v - va) <= 0.1, "cloche 16 kHz : %.2f dB au lieu de %.2f", v, va);
    }
    struct Fam { const char* name; Shape s; std::vector<double> g, q; std::vector<int> sl; double floor; };
    const double tolZl = 0.1;   // seuil unique : toutes les formes, toutes les fréquences d'échantillonnage
    const std::vector<Fam> fams = {
        { "cloche",      Shape::bell,      { -18, -12, -6, 6, 12, 18 }, { 0.1, 0.3, 0.71, 1, 2, 5, 10, 40 }, { 12 },   -200 },
        { "low shelf",   Shape::lowShelf,  { -18, -6, 6, 18 },          { 0.3, 0.71, 1, 2, 4, 10 },          { 6, 12 }, -200 },
        { "high shelf",  Shape::highShelf, { -18, -6, 6, 18 },          { 0.3, 0.71, 1, 2, 4, 10 },          { 6, 12 }, -200 },
        { "tilt",        Shape::tiltShelf, { -12, -6, 6, 12 },          { 0.3, 0.71, 1, 2, 4, 10 },          { 6, 12 }, -200 },
        { "notch",       Shape::notch,     { 0 },                       { 1, 2, 5, 10, 40 },                 { 12 },    -3 },
        { "coupe-bas",   Shape::lowCut,    { 0 },                       { 0.71, 1, 2 },                      { 6, 12, 18, 24, 48, 72, 96 }, -24 },
        { "coupe-haut",  Shape::highCut,   { 0 },                       { 0.71, 1, 2 },                      { 6, 12, 18, 24, 48, 72, 96 }, -24 },
        { "passe-bande", Shape::bandPass,  { 0 },                       { 0.71, 1, 2, 5 },                   { 12, 24, 48 }, -24 } };
    const double testFreqs[] = { 20.0, 30.0, 100.0, 300.0, 1000.0, 3000.0, 5000.0, 8000.0, 10000.0, 12000.0, 14000.0, 16000.0, 18000.0, 20000.0 };
    double secBefore[8] {}, secAfter[8] {}; int secMax[8] {}, secCount[8] {};   // (sections par bande, pour le point 12, à 44,1 kHz)
    for (double fs : { 44100.0, 48000.0, 96000.0, 192000.0 })
    {
        std::printf ("   %.1f kHz :\n", fs / 1000);
        int fi = 0;
        for (const auto& fm : fams)
        {
            double wa = 0, wm = 0, wb = 0, wf = 0, wFreq = 0;
            for (double f : testFreqs)
                for (double g : fm.g) for (double q : fm.q) for (int sl : fm.sl)
                {
                    BandSpec b; b.active = true; b.shape = fm.s; b.freq = f; b.gainDb = g; b.q = q; b.slope = sl;
                    double at = 0; int n0 = 0, n1 = 0;
                    const double e = errVsAnalog (b, fs, 2, fm.floor, &at, &n1);
                    if (e > wa) { wa = e; wf = at; wFreq = f; }
                    wm = std::max (wm, errVsAnalog (b, fs, 1, fm.floor, nullptr, &n0));
                    wb = std::max (wb, errVsAnalog (b, fs, 0, fm.floor));
                    if (fs < 46000) { secBefore[fi] += n0; secAfter[fi] += n1; secMax[fi] = std::max (secMax[fi], n1); ++secCount[fi]; }
                }
            std::printf ("      %-11s 1.x %6.2f dB → 2.0 %6.3f dB → 3.0 %.3f dB (pire : bande à %.0f Hz, écart à %.0f Hz) [seuil %.2f]\n", fm.name, wb, wm, wa, wFreq, wf, tolZl);
            CHECK (wa <= tolZl, "%s à %.0f Hz : écart %.3f dB", fm.name, fs, wa);
            ++fi;
        }
    }
    {
        // réglages tirés au hasard (pas seulement la grille ci-dessus), la moitié entre 5 et 20 kHz
        std::mt19937 rng (7); std::uniform_real_distribution<double> U (0, 1);
        for (double fs : { 44100.0, 48000.0, 96000.0, 192000.0 })
        {
            double w = 0; int over = 0; BandSpec wb;
            const int N = 6000;
            for (int it = 0; it < N; ++it)
            {
                BandSpec b; b.active = true; b.shape = (Shape) (int) (U (rng) * 8);
                b.freq = U (rng) < 0.5 ? 20 * std::pow (1000.0, U (rng)) : 5000 * std::pow (4.0, U (rng));
                b.gainDb = U (rng) * 36 - 18; b.slope = slopeValues[(int) (U (rng) * 9)];
                const bool sh = b.shape == Shape::lowShelf || b.shape == Shape::highShelf || b.shape == Shape::tiltShelf;
                b.q = b.shape == Shape::bell ? 0.1 * std::pow (400.0, U (rng)) : sh ? 0.3 * std::pow (10.0 / 0.3, U (rng))
                    : b.shape == Shape::notch ? std::pow (40.0, U (rng)) : 0.5 * std::pow (10.0, U (rng));
                const double fl = b.shape == Shape::notch ? -3 : (isCut (b.shape) || b.shape == Shape::bandPass) ? -24 : -200;
                const double e = errVsAnalog (b, fs, 2, fl);
                if (e > w) { w = e; wb = b; }
                if (e > tolZl) ++over;
            }
            std::printf ("   %.1f kHz, %d réglages au hasard (toutes formes, Q : cloche 0.1–40, étagères 0.3–10, notch 1–40, coupes 0.5–5) :\n"
                         "      pire %.3f dB (forme %d, %.0f Hz, %+.1f dB, Q %.2f, %d dB/oct), au-dessus du seuil : %d [seuil %.2f]\n",
                         fs / 1000, N, w, (int) wb.shape, wb.freq, wb.gainDb, wb.q, wb.slope, over, tolZl);
            CHECK (w <= tolZl, "réglages au hasard à %.0f Hz : %.3f dB", fs, w);
        }
    }
    {
        // réglage qui bouge : chaque conception part de la précédente (comme dans le moteur)
        struct C { const char* n; Shape s; double g, q; int sl; double fl; };
        const C cs[] = { { "cloche +12 Q 2", Shape::bell, 12, 2, 12, -200 }, { "cloche −9 Q 0.5", Shape::bell, -9, 0.5, 12, -200 }, { "high shelf +8", Shape::highShelf, 8, 0.71, 12, -200 },
                         { "high shelf −6 Q 3", Shape::highShelf, -6, 3, 12, -200 }, { "tilt +5", Shape::tiltShelf, 5, 0.71, 12, -200 }, { "notch Q 4", Shape::notch, 0, 4, 12, -3 },
                         { "coupe-haut 24", Shape::highCut, 0, 0.71, 24, -24 }, { "coupe-haut 72 Q 1.5", Shape::highCut, 0, 1.5, 72, -24 },
                         { "coupe-bas 36", Shape::lowCut, 0, 0.71, 36, -24 }, { "passe-bande 24 Q 2", Shape::bandPass, 0, 2, 24, -24 } };
        double w = 0; const char* wn = ""; double wfs = 0;
        for (double fs : { 44100.0, 48000.0, 96000.0 })
            for (const auto& c : cs)
            {
                BandSpec b; b.active = true; b.shape = c.s; b.gainDb = c.g; b.q = c.q; b.slope = c.sl;
                Design d, h; bool have = false;
                auto step = [&]
                {
                    designMatched (b, fs, d, maxCorrections, have ? &h : nullptr);
                    const double e = errOf (b, fs, d, c.fl, nullptr, 400);
                    if (e > w) { w = e; wn = c.n; wfs = fs; }
                    h = d; have = d.n > 0;
                };
                for (int i = 0; i <= 1200; ++i) { b.freq = 1000.0 * std::pow (20.0, i <= 600 ? i / 600.0 : (1200 - i) / 600.0); step(); }   // fréquence 1 → 20 → 1 kHz
                if (hasGain (c.s)) { b.freq = 14000; for (int i = 0; i <= 600; ++i) { b.gainDb = 15.0 * std::sin (2 * pi * i / 600.0); step(); } b.gainDb = c.g; }
                b.freq = 12000;
                for (int i = 0; i <= 600; ++i) { b.q = (hasGain (c.s) || c.s == Shape::notch ? 0.3 : 0.5) * std::pow (hasGain (c.s) || c.s == Shape::notch ? 33.0 : 8.0, 0.5 - 0.5 * std::cos (2 * pi * i / 600.0)); step(); }
            }
        std::printf ("   réglage qui bouge (conceptions enchaînées, balayages de fréquence, de gain et de Q, 10 formes, 44,1 / 48 / 96 kHz) :\n"
                     "      pire %.3f dB (%s à %.1f kHz) [seuil %.2f]\n", w, wn, wfs / 1000, tolZl);
        CHECK (w <= tolZl, "conceptions enchaînées : %.3f dB", w);
    }

    // ── 2. Moteur = courbe affichée ──────────────────────────────────────
    std::printf ("2. Réponse impulsionnelle du moteur = réponse affichée (matched)\n");
    {
        Engine eng;
        struct Case { Shape s; double f, g, q; int sl; };
        const Case cases[] = { { Shape::bell, 16000, 6, 1, 12 }, { Shape::bell, 60, -12, 4, 12 }, { Shape::lowShelf, 200, 6, 0.71, 12 },
                               { Shape::lowShelf, 300, -8, 0.71, 6 }, { Shape::highShelf, 12000, 5, 1.2, 12 }, { Shape::highShelf, 4000, -6, 0.71, 6 },
                               { Shape::lowCut, 80, 0, 0.71, 18 }, { Shape::lowCut, 60, 0, 0.71, 96 }, { Shape::highCut, 18000, 0, 0.71, 24 },
                               { Shape::highCut, 5000, 0, 3, 72 }, { Shape::notch, 1000, 0, 4, 12 }, { Shape::bandPass, 800, 0, 2, 36 },
                               { Shape::tiltShelf, 1000, 6, 0.71, 6 }, { Shape::tiltShelf, 800, -4, 0.71, 12 } };
        double worst = 0;
        for (double fs : { 44100.0, 96000.0 })
            for (const auto& c : cases)
            {
                BandSpec s; s.active = true; s.shape = c.s; s.freq = c.f; s.gainDb = c.g; s.q = c.q; s.slope = c.sl;
                eng.prepare (fs);
                for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
                eng.setBand (0, s, {}, 1.0);
                std::vector<float> l (4096, 0.0f), r (4096, 0.0f);
                eng.process (l.data(), r.data(), 4096, false);
                const int n = 65536;
                std::vector<float> xl ((size_t) n, 0.0f), xr ((size_t) n, 0.0f);
                xl[0] = xr[0] = 1.0f;
                eng.process (xl.data(), xr.data(), n, false);
                std::vector<double> x (xl.begin(), xl.end());
                std::vector<fir::cd> X; fftReal (x, X, n);
                Design d; design (s, fs, d, false);
                for (int k = 1; k < n / 2; ++k)
                {
                    const double f = k * fs / n;
                    if (f < 20 || f > 0.45 * fs) continue;
                    const double ref = magnitudeDb (d, f, fs, false);
                    if (ref < -60) continue;
                    worst = std::max (worst, std::abs (db (std::abs (X[(size_t) k])) - ref));
                }
            }
        std::printf ("   14 formes, 44,1 et 96 kHz : écart max %.4f dB [seuil 0.05]\n", worst);
        CHECK (worst < 0.05, "réponse du moteur ≠ courbe affichée (%.3f dB)", worst);

        // Après un balayage : le moteur a suivi le réglage de proche en proche (chaque conception part de la
        // précédente), alors que l'affichage conçoit directement le réglage final. Les deux tiennent la même
        // cible analogique : on mesure l'écart entre ce qu'on entend et ce qui est affiché.
        struct Sweep { Shape s; double f0, f1, g0, g1, q; int sl; };
        const Sweep sweeps[] = { { Shape::bell, 3000, 14000, 9, 9, 1.5, 12 }, { Shape::bell, 16000, 16000, -12, 12, 3, 12 }, { Shape::highShelf, 10000, 10000, -8, 8, 0.71, 12 },
                                 { Shape::highShelf, 2000, 12000, 6, 6, 2, 12 }, { Shape::highCut, 6000, 17000, 0, 0, 0.71, 48 }, { Shape::lowCut, 2000, 12000, 0, 0, 1, 24 },
                                 { Shape::notch, 4000, 15000, 0, 0, 3, 12 }, { Shape::tiltShelf, 9000, 9000, -6, 6, 0.71, 12 } };
        double worstS = 0, worstA = 0;
        for (double fs : { 44100.0, 48000.0 })
            for (const auto& c : sweeps)
            {
                BandSpec sp; sp.active = true; sp.shape = c.s; sp.freq = c.f0; sp.gainDb = c.g0; sp.q = c.q; sp.slope = c.sl;
                eng.prepare (fs);
                for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
                eng.setBand (0, sp, {}, 1.0);
                std::vector<float> l (4096, 0.0f), r (4096, 0.0f);
                eng.process (l.data(), r.data(), 4096, false);
                for (int k = 1; k <= 300; ++k)
                {
                    sp.freq = c.f0 * std::pow (c.f1 / c.f0, k / 300.0); sp.gainDb = c.g0 + (c.g1 - c.g0) * k / 300.0;
                    eng.setBand (0, sp, {}, 1.0);
                    eng.process (l.data(), r.data(), 64, false);
                }
                for (int k = 0; k < 4; ++k) eng.process (l.data(), r.data(), 4096, false);
                const int n = 65536;
                std::vector<float> xl ((size_t) n, 0.0f), xr ((size_t) n, 0.0f);
                xl[0] = xr[0] = 1.0f;
                eng.process (xl.data(), xr.data(), n, false);
                std::vector<double> x (xl.begin(), xl.end());
                std::vector<fir::cd> X; fftReal (x, X, n);
                Design d, da; design (sp, fs, d, false); design (sp, fs, da, true);
                const double fl = c.s == Shape::notch ? -3 : (isCut (c.s) ? -24 : -200);
                for (int k = 1; k < n / 2; ++k)
                {
                    const double f = k * fs / n;
                    if (f < 20 || f > 20000) continue;
                    const double ra = magnitudeDb (da, f, fs, true);
                    if (ra < fl) continue;
                    const double got = db (std::abs (X[(size_t) k]));
                    worstS = std::max (worstS, std::abs (got - magnitudeDb (d, f, fs, false)));
                    worstA = std::max (worstA, std::abs (got - ra));
                }
            }
        std::printf ("   après un balayage (8 réglages aigus, 44,1 et 48 kHz) : moteur / courbe affichée %.3f dB ; moteur / analogique %.3f dB [seuils 0.05 et 0.10]\n", worstS, worstA);
        CHECK (worstS <= 0.05 && worstA <= 0.1, "moteur après balayage : %.3f dB de la courbe affichée, %.3f dB de l'analogique", worstS, worstA);
    }

    // ── 3. Même son à toutes les fréquences d'échantillonnage ─────────────
    std::printf ("3. Même courbe à 44,1 / 48 / 96 / 192 kHz (écart max à la courbe 192 kHz, 20 Hz – 20 kHz)\n");
    {
        struct Case { Shape s; double f, g, q; int sl; };
        const Case cases[] = { { Shape::bell, 12000, 8, 1.5, 12 }, { Shape::bell, 18000, -6, 2, 12 }, { Shape::highShelf, 10000, 6, 0.71, 12 },
                               { Shape::lowShelf, 5000, -9, 1, 12 }, { Shape::highShelf, 15000, 4, 0.71, 6 }, { Shape::notch, 15000, 0, 3, 12 },
                               { Shape::tiltShelf, 4000, 6, 0.71, 12 },
                               // 3.0 : étagères résonantes, coupes raides tout en haut, passe-bande, cloches extrêmes
                               { Shape::highShelf, 3000, 9, 4, 12 }, { Shape::highShelf, 6000, -12, 10, 12 }, { Shape::lowShelf, 10000, 6, 10, 12 }, { Shape::tiltShelf, 8000, -6, 4, 12 },
                               { Shape::highCut, 16000, 0, 0.71, 48 }, { Shape::highCut, 16000, 0, 0.71, 72 }, { Shape::highCut, 16000, 0, 0.71, 96 },
                               { Shape::highCut, 20000, 0, 0.71, 48 }, { Shape::highCut, 20000, 0, 1, 72 }, { Shape::highCut, 20000, 0, 0.71, 96 },
                               { Shape::lowCut, 16000, 0, 0.71, 48 }, { Shape::lowCut, 20000, 0, 0.71, 96 }, { Shape::bandPass, 12000, 0, 2, 24 },
                               { Shape::bell, 20000, 18, 0.3, 12 }, { Shape::bell, 16000, -18, 10, 12 } };
        const int nCases = (int) (sizeof (cases) / sizeof (cases[0]));
        double v1 = 0, v2 = 0, after = 0, after7 = 0;
        int ci = 0;
        for (const auto& c : cases)
        {
            BandSpec s; s.active = true; s.shape = c.s; s.freq = c.f; s.gainDb = c.g; s.q = c.q; s.slope = c.sl;
            Design ref; design (s, 192000, ref, false);
            const double fl = c.s == Shape::notch ? -3 : (isCut (c.s) || c.s == Shape::bandPass) ? -24 : -200;   // (même zone utile qu'au point 1)
            for (double fs : { 44100.0, 48000.0, 96000.0 })
            {
                Design d, m, l; design (s, fs, d, false); designMatched (s, fs, m, 0); designBilinear (s, fs, l);
                for (int i = 0; i < 400; ++i)
                {
                    const double f = 20 * std::pow (1000.0, i / 399.0);
                    const double r = magnitudeDb (ref, f, 192000, false);
                    if (r < fl) continue;
                    const double e = std::abs (magnitudeDb (d, f, fs, false) - r);
                    after = std::max (after, e);
                    if (ci < 7) after7 = std::max (after7, e);
                    v2 = std::max (v2, std::abs (magnitudeDb (m, f, fs, false) - r));
                    v1 = std::max (v1, std::abs (magnitudeDb (l, f, fs, false) - r));
                }
            }
            ++ci;
        }
        std::printf ("   %d réglages aigus : 1.x %.2f dB → 2.0 %.2f dB → 3.0 %.3f dB (les 7 réglages du banc 2.0 : %.3f dB) [seuil 0.05]\n", nCases, v1, v2, after, after7);
        CHECK (after <= 0.05, "courbe différente selon la fréquence d'échantillonnage (%.3f dB)", after);
    }

    // ── 4. Tests de nul ──────────────────────────────────────────────────
    std::printf ("4. Tests de nul\n");
    {
        Engine eng; eng.prepare (48000);
        const double fr[] = { 60, 300, 1000, 3000, 10000 };
        const Shape sh[] = { Shape::lowShelf, Shape::bell, Shape::bell, Shape::bell, Shape::highShelf };
        for (int b = 0; b < maxBands; ++b)
        {
            BandSpec s; s.active = b < 5; s.shape = sh[b % 5]; s.freq = fr[b % 5]; s.gainDb = 0; s.q = b == 0 || b == 4 ? 0.71 : 1;
            DynSettings d; d.on = b == 2; d.thrDb = 0; d.rangeDb = -6;   // une bande dynamique sous son seuil
            eng.setBand (b, s, d, 1.0);
        }
        std::mt19937 rng (5); std::uniform_real_distribution<float> U (-0.5f, 0.5f);
        float worst = 0;
        for (int blk = 0; blk < 50; ++blk)
        {
            std::vector<float> l (512), r (512), l0, r0;
            for (int i = 0; i < 512; ++i) { l[(size_t) i] = U (rng) * 0.1f; r[(size_t) i] = U (rng) * 0.1f; }
            l0 = l; r0 = r;
            eng.process (l.data(), r.data(), 512, false);
            for (int i = 0; i < 512; ++i) worst = std::max (worst, std::max (std::abs (l[(size_t) i] - l0[(size_t) i]), std::abs (r[(size_t) i] - r0[(size_t) i])));
        }
        std::printf ("   5 bandes à 0 dB (dont une dynamique sous le seuil) : écart max sortie / entrée %.1e [seuil 0]\n", worst);
        CHECK (worst == 0.0f, "les bandes à 0 dB modifient le signal (%.2e)", worst);
    }

    // ── 5. EQ dynamique : raies parasites ────────────────────────────────
    // Mesure : résidu par rapport à la référence idéale (voir residual()), sur TOUT le spectre.
    // La raie à f0 ± k·fs/32 de la sortie elle-même ne suffit pas : avec une attaque de 0,5 ms le gain
    // ondule au rythme du sinus redressé, et la référence idéale contient déjà des raies à ce niveau.
    std::printf ("5. EQ dynamique, sinus 997 Hz (amplitude variant à 4 Hz), attaque 0.5 ms / retour 20 ms, plage −12 dB :\n"
                 "   résidu par rapport à la référence idéale (conception exacte à chaque échantillon), tout le spectre\n");
    {
        const double fs = 48000;
        std::vector<float> ref, out, old, r;
        runDynCase (dynCases[0], 1, 480, ref, r);
        runDynCase (dynCases[0], 0, 480, out, r);
        runDynCase (dynCases[0], 2, 480, old, r);
        const Resid before = residual (old, ref, dynPre, dynN, fs), after = residual (out, ref, dynPre, dynN, fs);
        std::printf ("   raie max du résidu : 1.x (paliers de 32 éch.) %.1f dB → 3.0 (gain de chaque échantillon) %.1f dB (à %.0f Hz) [seuil : −110 dB]\n",
                     before.line, after.line, after.atHz);
        std::printf ("   valeur efficace du résidu : 1.x %.1f dB → 3.0 %.1f dB ; (2.0, mesuré avant ce travail : raie −71.2 dB, efficace −59.8 dB)\n", before.rms, after.rms);
        std::printf ("   pour mémoire, raie max à f0 ± k·fs/32 dans la sortie : 1.x %.1f dB, 3.0 %.1f dB, référence idéale %.1f dB\n",
                     blockLines (old, dynPre, dynN, fs, 997.0), blockLines (out, dynPre, dynN, fs, 997.0), blockLines (ref, dynPre, dynN, fs, 997.0));
        CHECK (after.line <= -110.0, "raies parasites de l'EQ dynamique (%.1f dB)", after.line);
    }

    // ── 6. Réglage qui bouge : bruit d'escalier ──────────────────────────
    // Deux mesures : (a) raie max à f0 ± k·fs/32 dans la sortie (signature des sous-blocs) ;
    // (b) résidu par rapport à la référence idéale, qui reçoit le même réglage à chaque échantillon,
    // décalé de 46,6 échantillons (le moteur lisse par sous-blocs de 32 et joue chaque tronçon un
    // sous-bloc plus tard). Près de f0 (± 300 Hz) le résidu ne contient que des bandes latérales de la
    // modulation elle-même (lissage par bloc et lissage par échantillon n'ont pas exactement la même
    // réponse) : on le donne à part, le seuil porte sur le reste du spectre.
    std::printf ("6. Fréquence d'une cloche +12 dB modulée (1 kHz × 2^(±1.5) à 3 Hz), sinus 3001 Hz\n");
    {
        const double fs = 48000;
        std::vector<float> ref, out, old;
        runSweep (1, 32, 46.6, ref);
        runSweep (0, 32, 0, out);
        runSweep (2, 32, 0, old);
        const double lOld = blockLines (old, dynPre, dynN, fs, 3001.0), lNew = blockLines (out, dynPre, dynN, fs, 3001.0);
        const Resid far = residual (out, ref, dynPre, dynN, fs, 3001.0, 300.0), all = residual (out, ref, dynPre, dynN, fs);
        std::printf ("   (a) raie max à f0 ± k·fs/32 : 1.x %.1f dB → 3.0 %.1f dB (2.0, mesuré avant ce travail : −108.8 dB) [seuil : −110 dB]\n", lOld, lNew);
        std::printf ("   (b) résidu / référence idéale, hors f0 ± 300 Hz : raie max %.1f dB (à %.0f Hz) [seuil : −110 dB] ;\n"
                     "       dans f0 ± 300 Hz (bandes latérales de la modulation) : %.1f dB\n", far.line, far.atHz, all.line);
        CHECK (lNew <= -110.0, "escalier quand un réglage bouge (%.1f dB)", lNew);
        CHECK (far.line <= -110.0, "raies parasites quand un réglage bouge (%.1f dB)", far.line);
    }

    {
        // même mesure dans l'aigu à 44,1 kHz (cloche 8 kHz × 2^(±1) : la section de correction apparaît,
        // évolue et disparaît à chaque passage), sinus 5001 Hz
        const double fs = 44100;
        double res[3] {}, noise[3] {};
        for (int mode = 0; mode < 3; ++mode)   // 0 : 3.0 ; 1 : paliers de 32 éch. (1.x) ; 2 : interpolation sans correction (2.0)
        {
            Engine eng; eng.legacyStepping = mode == 1; eng.corrections = mode == 2 ? 0 : maxCorrections; eng.prepare (fs);
            for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
            const int n = 1 << 17, tot = n + 8192;
            std::vector<float> l ((size_t) tot), r;
            for (int i = 0; i < tot; ++i) l[(size_t) i] = (float) (0.1 * std::sin (2 * pi * 5001.0 * i / fs));
            r = l;
            for (int off = 0; off < tot; off += 32)
            {
                BandSpec s; s.active = true; s.shape = Shape::bell; s.gainDb = 12; s.q = 2;
                s.freq = 8000.0 * std::pow (2.0, 1.0 * std::sin (2 * pi * 3.0 * off / fs));
                eng.setBand (0, s, {}, 1.0);
                eng.process (l.data() + off, r.data() + off, std::min (32, tot - off), false);
            }
            res[mode] = blockLines (l, 8192, n, fs, 5001.0);
            noise[mode] = farNoise (l, 8192, n, fs, 5001.0);
        }
        std::printf ("   (c) aigu, 44,1 kHz (cloche +12 dB Q 2 de 4 à 16 kHz à 3 Hz, sinus 5001 Hz : les sections de correction évoluent sans cesse)\n"
                     "       raie max à f0 ± k·fs/32 : 1.x %.1f dB → 3.0 %.1f dB [seuil : 20 dB de mieux]\n", res[1], res[0]);
        std::printf ("       bruit large bande loin du sinus : paliers 1.x %.1f dB ; sans correction %.1f dB ; avec corrections %.1f dB [seuil : −90 dB]\n", noise[1], noise[2], noise[0]);
        CHECK (res[0] <= res[1] - 20.0, "escalier quand un réglage bouge dans l'aigu (%.1f dB)", res[0]);
        CHECK (noise[0] <= -90.0, "bruit de transition des sections de correction (%.1f dB)", noise[0]);
    }
    {
        // coupe-haut 48 dB/oct balayé de 6 à 19 kHz (0,5 Hz), sinus 3001 Hz dans la bande passante : les
        // sections réajustées et les corrections changent sans cesse ; même mesure de bruit large bande
        const double fs = 44100;
        double noise[2] {};
        for (int mode = 0; mode < 2; ++mode)
        {
            Engine eng; eng.corrections = mode == 1 ? 0 : maxCorrections; eng.prepare (fs);
            for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
            const int n = 1 << 17, tot = n + 8192;
            std::vector<float> l ((size_t) tot), r;
            for (int i = 0; i < tot; ++i) l[(size_t) i] = (float) (0.1 * std::sin (2 * pi * 3001.0 * i / fs));
            r = l;
            for (int off = 0; off < tot; off += 32)
            {
                BandSpec s; s.active = true; s.shape = Shape::highCut; s.q = 0.71; s.slope = 48;
                s.freq = 6000.0 * std::pow (19.0 / 6.0, 0.5 + 0.5 * std::sin (2 * pi * 0.5 * off / fs));
                eng.setBand (0, s, {}, 1.0);
                eng.process (l.data() + off, r.data() + off, std::min (32, tot - off), false);
            }
            noise[mode] = farNoise (l, 8192, n, fs, 3001.0);
        }
        std::printf ("   (d) coupe-haut 48 dB/oct balayé 6 – 19 kHz, sinus 3001 Hz : bruit large bande sans correction %.1f dB ; avec corrections %.1f dB [seuil : −90 dB]\n", noise[1], noise[0]);
        CHECK (noise[0] <= -90.0, "bruit de transition du coupe-haut balayé (%.1f dB)", noise[0]);
    }

    // ── 7. Phase linéaire ────────────────────────────────────────────────
    std::printf ("7. Phase linéaire à 48 kHz, résolution Auto (avant = 1.x : 8192 points, Blackman-Harris)\n");
    {
        const double fs = 48000;
        fir::Work w;
        auto run = [&] (const BandSpec& b, int& N, double (*measure) (const std::vector<double>&, int, const BandSpec&, double), double& before, double& after)
        {
            fir::Request rq; rq.fs = fs; rq.mode = fir::Mode::linear; rq.bands[0] = b; rq.delay = fir::naturalDelay (fs);
            N = fir::chooseLength (fir::Res::automatic, rq.bands, fs, rq.mode);
            rq.N = N;
            fir::Result r; fir::design (rq, r, w);
            after = measure (r.h[0], N / 2, b, fs);
            const auto old = legacyLinearFir (b, fs, 8192);
            before = measure (old, 4096, b, fs);
        };
        int N; double before, after;
        BandSpec bell; bell.active = true; bell.shape = Shape::bell; bell.freq = 50; bell.gainDb = -12; bell.q = 8;
        run (bell, N, [] (const std::vector<double>& h, int lat, const BandSpec& b, double fs_) {
            Design d; design (b, fs_, d, true); double e = 0;
            for (int i = 0; i < 500; ++i) { const double f = 20 * std::pow (10.0, i / 499.0); e = std::max (e, std::abs (db (std::abs (fir::responseAt (h, lat, fs_, f))) - magnitudeDb (d, f, fs_, true))); }
            return e; }, before, after);
        std::printf ("   cloche 50 Hz −12 dB Q 8 : écart max 20–200 Hz avant %.2f dB → après %.2f dB (FIR %d points) [seuil ±1]\n", before, after, N);
        CHECK (after <= 1.0, "cloche 50 Hz en phase linéaire (%.2f dB)", after);
        BandSpec notch; notch.active = true; notch.shape = Shape::notch; notch.freq = 60; notch.q = 10;
        run (notch, N, [] (const std::vector<double>& h, int lat, const BandSpec&, double fs_) { return db (std::abs (fir::responseAt (h, lat, fs_, 60.0))); }, before, after);
        std::printf ("   notch 60 Hz Q 10 : creux au centre avant %.1f dB → après %.1f dB (FIR %d points) [seuil ≤ −20]\n", before, after, N);
        CHECK (after <= -20.0, "notch 60 Hz en phase linéaire (%.1f dB)", after);
        BandSpec lc; lc.active = true; lc.shape = Shape::lowCut; lc.freq = 30; lc.q = 0.71; lc.slope = 48;
        run (lc, N, [] (const std::vector<double>& h, int lat, const BandSpec& b, double fs_) {
            Design d; design (b, fs_, d, true); return std::abs (db (std::abs (fir::responseAt (h, lat, fs_, 20.0))) - magnitudeDb (d, 20.0, fs_, true)); }, before, after);
        {
            Design d; design (lc, fs, d, true);
            std::printf ("   coupe-bas 30 Hz 48 dB/oct : écart à 20 Hz (analogique %.1f dB) avant %.2f dB → après %.2f dB (FIR %d points) [seuil ±1]\n", magnitudeDb (d, 20, fs, true), before, after, N);
        }
        CHECK (after <= 1.0, "coupe-bas 30 Hz en phase linéaire (%.2f dB)", after);
        // symétrie et latence d'un FIR quelconque
        fir::Request rq; rq.fs = fs; rq.mode = fir::Mode::linear; rq.bands[0] = bell; rq.bands[1] = lc; rq.N = 16384;
        rq.bands[2].active = true; rq.bands[2].shape = Shape::highShelf; rq.bands[2].freq = 8000; rq.bands[2].gainDb = -6; rq.bands[2].place = Place::side;
        fir::Result r; fir::design (rq, r, w);
        double asym = 0;
        for (int m = 1; m < rq.N / 2; ++m) asym = std::max (asym, std::abs (r.h[0][(size_t) (rq.N / 2 + m)] - r.h[0][(size_t) (rq.N / 2 - m)]));
        std::printf ("   FIR symétrique autour de N/2 (avec Mid/Side, matrice %s) : asymétrie max %.1e [seuil 1e-6]\n", r.matrix ? "oui" : "non", asym);
        CHECK (asym < 1e-6 && r.matrix, "FIR de phase linéaire non symétrique");
    }

    // ── 8. Natural phase ─────────────────────────────────────────────────
    std::printf ("8. Natural phase : module et phase = prototype analogique (20 Hz – 18 kHz)\n");
    {
        fir::Work w;
        for (double fs : { 44100.0, 48000.0, 96000.0, 192000.0 })
        {
            fir::Request rq; rq.fs = fs; rq.mode = fir::Mode::natural; rq.delay = fir::naturalDelay (fs);
            auto& b0 = rq.bands[0]; b0.active = true; b0.shape = Shape::highShelf; b0.freq = 8000; b0.gainDb = 9; b0.q = 0.71;
            auto& b1 = rq.bands[1]; b1.active = true; b1.shape = Shape::bell; b1.freq = 100; b1.gainDb = -8; b1.q = 4;
            auto& b2 = rq.bands[2]; b2.active = true; b2.shape = Shape::lowCut; b2.freq = 30; b2.q = 0.71; b2.slope = 24;
            auto& b3 = rq.bands[3]; b3.active = true; b3.shape = Shape::bell; b3.freq = 16000; b3.gainDb = 6; b3.q = 1;
            rq.N = fir::chooseLength (fir::Res::automatic, rq.bands, fs, rq.mode);
            fir::Result r; fir::design (rq, r, w);
            Design d[4]; for (int i = 0; i < 4; ++i) design (rq.bands[(size_t) i], fs, d[i], true);
            double em = 0, ep = 0;
            for (int i = 0; i < 400; ++i)
            {
                const double f = 20 * std::pow (900.0, i / 399.0);
                fir::cd ref (1, 0);
                for (int k = 0; k < 4; ++k) ref *= response (d[k], f, fs, true);
                if (std::abs (ref) < 0.03) continue;
                const auto got = fir::responseAt (r.h[0], rq.delay, fs, f);
                em = std::max (em, std::abs (db (std::abs (got) / std::abs (ref))));
                ep = std::max (ep, std::abs (std::arg (got / ref)) * 180.0 / pi);
            }
            std::printf ("   %.1f kHz : FIR %d points, latence %d + %d éch. (%.1f ms) : module %.3f dB, phase %.2f° [seuils 0.1 dB, 1°]\n",
                         fs / 1000, rq.N, rq.delay, fir::partitionSize (fs), 1000.0 * (rq.delay + fir::partitionSize (fs)) / fs, em, ep);
            CHECK (em <= 0.1 && ep <= 1.0, "Natural phase à %.0f Hz (%.3f dB, %.2f°)", fs, em, ep);
        }
    }

    // ── 9. Latences ──────────────────────────────────────────────────────
    std::printf ("9. Latences annoncées (FIR + bloc de partition)\n");
    for (double fs : { 44100.0, 48000.0, 96000.0, 192000.0 })
    {
        const int P = fir::partitionSize (fs);
        std::printf ("   %.1f kHz : Low %d, Medium %d, High %d, Max %d éch. ; Natural %d éch. (%.1f ms)\n", fs / 1000,
                     fir::lengthFor (fir::Res::low, fs) / 2 + P, fir::lengthFor (fir::Res::medium, fs) / 2 + P, fir::lengthFor (fir::Res::high, fs) / 2 + P,
                     fir::lengthFor (fir::Res::max, fs) / 2 + P, fir::naturalDelay (fs) + P, 1000.0 * (fir::naturalDelay (fs) + P) / fs);
        CHECK (P >= 256 && fir::naturalDelay (fs) + P <= 0.012 * fs, "latence Natural phase trop grande à %.0f Hz", fs);
    }
    {
        // pic d'un FIR neutre au bon endroit
        fir::Work w; fir::Request rq; rq.fs = 48000; rq.N = 8192; fir::Result r; fir::design (rq, r, w);
        int pk = 0; for (int n = 0; n < rq.N; ++n) if (std::abs (r.h[0][(size_t) n]) > std::abs (r.h[0][(size_t) pk])) pk = n;
        rq.mode = fir::Mode::natural; rq.delay = fir::naturalDelay (48000); fir::Result rn; fir::design (rq, rn, w);
        int pn = 0; for (int n = 0; n < rq.N; ++n) if (std::abs (rn.h[0][(size_t) n]) > std::abs (rn.h[0][(size_t) pn])) pn = n;
        std::printf ("   courbe plate : pic linéaire à %d (attendu %d, valeur %.4f), naturel à %d (attendu %d)\n", pk, rq.N / 2, r.h[0][(size_t) pk], pn, rq.delay);
        CHECK (pk == rq.N / 2 && std::abs (r.h[0][(size_t) pk] - 1.0) < 1e-6 && pn == rq.delay, "pic du FIR neutre mal placé");
    }

    // ── 10. Side-chain externe ───────────────────────────────────────────
    std::printf ("10. Side-chain externe : la bande réagit au side-chain, pas au signal\n");
    {
        const double fs = 48000;
        double offs[3] {};
        for (int k = 0; k < 3; ++k)
        {
            Engine eng; eng.prepare (fs);
            for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
            BandSpec s; s.active = true; s.shape = Shape::bell; s.freq = 1000; s.q = 2;
            DynSettings d; d.on = true; d.thrDb = -30; d.rangeDb = -10; d.attackMs = 2; d.releaseMs = 50; d.external = k > 0;
            eng.setBand (0, s, d, 1.0);
            const int n = 24000;
            std::vector<float> l ((size_t) n), r, sc ((size_t) n);
            for (int i = 0; i < n; ++i)
            {
                l[(size_t) i] = (float) (0.3 * std::sin (2 * pi * 1000 * i / fs));                 // signal fort
                sc[(size_t) i] = k == 2 ? (float) (0.3 * std::sin (2 * pi * 1000 * i / fs)) : 0.0f; // side-chain muet / fort
            }
            r = l;
            eng.process (l.data(), r.data(), n, false, sc.data(), nullptr);
            offs[k] = eng.getDynOffset (0);
        }
        std::printf ("   interne %.1f dB ; externe muet %.1f dB ; externe fort %.1f dB\n", offs[0], offs[1], offs[2]);
        CHECK (offs[0] < -9 && std::abs (offs[1]) < 0.01 && offs[2] < -9, "side-chain externe");
    }

    // ── 11. Stabilité ────────────────────────────────────────────────────
    std::printf ("11. Stabilité : 100 000 réglages aléatoires (formes, pentes, Q 0.025–40, ±30 dB, 44,1–192 kHz)\n");
    {
        std::mt19937 rng (1); std::uniform_real_distribution<double> U (0, 1);
        int bad = 0;
        for (int it = 0; it < 100000; ++it)
        {
            const double rates[] = { 44100, 48000, 88200, 96000, 176400, 192000 };
            BandSpec b; b.active = true; b.shape = (Shape) (int) (U (rng) * 8); b.freq = 20 * std::pow (1000.0, U (rng));
            b.gainDb = U (rng) * 60 - 30; b.q = 0.025 * std::pow (1600.0, U (rng)); b.slope = slopeValues[(int) (U (rng) * 9)];
            Design d; design (b, rates[it % 6], d, false);
            for (int i = 0; i < d.n; ++i)
            {
                const auto& s = d.s[(size_t) i];
                if (! (s.g > 0 && s.k > 0 && std::isfinite (s.m0 + s.m1 + s.m2))) { ++bad; break; }
            }
        }
        std::printf ("   sections instables ou non finies : %d [seuil 0]\n", bad);
        CHECK (bad == 0, "%d conceptions instables", bad);
        // automatisation aléatoire du moteur (y compris la dynamique) : pas de NaN, pas d'explosion
        Engine eng; eng.prepare (44100);
        double peak = 0; bool nan = false;
        std::vector<float> l (256), r (256), sc (256);
        for (int it = 0; it < 3000; ++it)
        {
            if (it % 5 == 0)
                for (int bnd = 0; bnd < 8; ++bnd)
                {
                    BandSpec s; s.active = U (rng) < 0.8; s.shape = (Shape) (int) (U (rng) * 8); s.slope = slopeValues[(int) (U (rng) * 9)];
                    s.place = (Place) (int) (U (rng) * 5); s.freq = 10 * std::pow (2000.0, U (rng)); s.gainDb = U (rng) * 40 - 20; s.q = 0.1 * std::pow (100.0, U (rng));
                    DynSettings d; d.on = U (rng) < 0.5; d.thrDb = -60 * U (rng); d.rangeDb = U (rng) * 40 - 20; d.attackMs = 0.5 + 20 * U (rng); d.external = U (rng) < 0.3;
                    eng.setBand (bnd, s, d, 1.0);
                }
            for (int i = 0; i < 256; ++i) { l[(size_t) i] = (float) (U (rng) - 0.5); r[(size_t) i] = (float) (U (rng) - 0.5); sc[(size_t) i] = (float) (U (rng) - 0.5); }
            eng.process (l.data(), r.data(), 256, false, sc.data(), nullptr);
            for (int i = 0; i < 256; ++i)
            {
                if (! std::isfinite (l[(size_t) i]) || ! std::isfinite (r[(size_t) i])) nan = true;
                peak = std::max (peak, (double) std::max (std::abs (l[(size_t) i]), std::abs (r[(size_t) i])));
            }
        }
        std::printf ("   automatisation aléatoire (8 bandes, dynamique, side-chain) : crête %.1f, NaN : %s [seuil < 300]\n", peak, nan ? "oui" : "non");
        CHECK (! nan && peak < 300, "moteur instable sous automatisation");
    }

    // ── 12. EQ dynamique : résidu / référence idéale, autres cas ─────────
    std::printf ("12. EQ dynamique : résidu par rapport à la référence idéale, tout le spectre (raie max ; valeur efficace) [seuil : −110 dB]\n");
    {
        const double fs = 48000;
        double worst = -200;
        for (const auto& c : dynCases)
        {
            std::vector<float> refL, refR, outL, outR, oldL, oldR;
            runDynCase (c, 1, 480, refL, refR);
            runDynCase (c, 0, 480, outL, outR);
            runDynCase (c, 2, 480, oldL, oldR);
            const Resid a = residual (outL, refL, dynPre, dynN, fs), b = residual (outR, refR, dynPre, dynN, fs), o = residual (oldL, refL, dynPre, dynN, fs);
            const double line = std::max (a.line, b.line), rms = std::max (a.rms, b.rms);
            std::printf ("   %-46s : 1.x %6.1f dB → 3.0 %6.1f dB ; %6.1f dB\n", c.name, o.line, line, rms);
            worst = std::max (worst, line);
        }
        CHECK (worst <= -110.0, "résidu de l'EQ dynamique (%.1f dB)", worst);
    }

    // ── 13. Cache des bandes dynamiques ──────────────────────────────────
    // Toutes les cellules de 0,25 dB sur ±24 dB, 40 réglages par forme (30 Hz – 16 kHz, Q 0,3 – 12) :
    // réponse interpolée au quart, au milieu et aux trois quarts de la cellule contre une conception
    // exacte. « précises » : écart ≤ 3e-6 (−110 dB). Le reste vient de la conception elle-même, qui
    // n'est pas partout régulière en fonction du gain (EqMath : choix entre plusieurs solutions) ;
    // le cache lit alors la cellule en ligne droite (« droites ») ou sans interpolation (« paliers »).
    std::printf ("13. Cache des bandes dynamiques : conception interpolée contre conception exacte, cellule par cellule\n");
    {
        struct Fam { const char* name; Shape s; int slope; double minPrecise; };
        const Fam fams[] = { { "cloche", Shape::bell, 12, 95.0 }, { "low shelf 12", Shape::lowShelf, 12, 0 }, { "high shelf 12", Shape::highShelf, 12, 0 },
                             { "low shelf 6", Shape::lowShelf, 6, 0 }, { "high shelf 6", Shape::highShelf, 6, 0 }, { "tilt 12", Shape::tiltShelf, 12, 0 } };
        for (double fs : { 44100.0, 96000.0 })
            for (const auto& fm : fams)
            {
                int cells = 0, precise = 0, lin = 0, rough = 0, refused = 0;
                double worstCubic = 0, worstAny = 0;
                GainCache cache;
                for (double f : { 30.0, 100.0, 300.0, 1000.0, 3000.0, 8000.0, 12000.0, 16000.0 })
                    for (double q : { 0.3, 0.71, 1.5, 4.0, 12.0 })
                    {
                        BandSpec b; b.active = true; b.shape = fm.s; b.slope = fm.slope; b.freq = f; b.q = q;
                        cache.setup (b, -24.0, 24.0, fs);
                        int budget = 1 << 20;
                        if (! cache.ensure (0, cache.numCells() - 1, budget)) { ++refused; continue; }
                        const int linBefore = cache.linearCells() + cache.roughCells();
                        cells += cache.numCells(); lin += cache.linearCells(); rough += cache.roughCells();
                        for (int c = 0; c < cache.numCells(); ++c)
                        {
                            double e = 0;
                            for (double t : { 0.25, 0.5, 0.75 })
                            {
                                PDesign pd; cache.eval (c, t, pd);
                                Design di, de; di.n = pd.n; di.outGain = pd.outGain;
                                for (int i = 0; i < pd.n; ++i) sectionFromP (pd.s[(size_t) i], pd.kind[(size_t) i], di.s[(size_t) i]);
                                b.gainDb = -24.0 + (c + t) * cache.stepDb();
                                designDynExact (b, fs, de);   // (conception des bandes dynamiques : sans section de correction)
                                for (double m : { 0.5, 0.8, 1.0, 1.25, 2.0 })
                                {
                                    const double ff = std::min (f * m, 0.45 * fs);
                                    const auto he = response (de, ff, fs, false);
                                    e = std::max (e, std::abs (response (di, ff, fs, false) - he) / std::abs (he));
                                }
                            }
                            if (e <= 3e-6) ++precise;
                            worstAny = std::max (worstAny, e);
                            if (linBefore == 0) worstCubic = std::max (worstCubic, e);   // réglages entièrement en cubique
                        }
                    }
                const double pct = 100.0 * precise / std::max (1, cells);
                std::printf ("   %.1f kHz %-13s : %5.1f %% de cellules précises ; droites %4d, paliers %3d sur %d ; pire écart %.1e (réglages tout en cubique : %.1e)\n",
                             fs / 1000, fm.name, pct, lin, rough, cells, worstAny, worstCubic);
                CHECK (refused == 0, "cache refusé (%s)", fm.name);
                CHECK (pct >= fm.minPrecise, "cache dynamique des cloches : %.1f %% de cellules précises", pct);
                CHECK (worstCubic <= 1e-3, "interpolation cubique du cache (%s : %.1e)", fm.name, worstCubic);
            }
    }

    // ── 14. Dynamique + automation, taille des blocs de l'hôte ───────────
    std::printf ("14. Bande dynamique dont la fréquence est automatisée en même temps (± 0.5 octave à 3 Hz), attaque 0.5 ms\n");
    {
        const double fs = 48000;
        std::vector<float> ref, out, old;
        runSweep (1, 32, 14.6, ref, true);
        runSweep (0, 32, 0, out, true);
        runSweep (2, 32, 0, old, true);
        const Resid a = residual (out, ref, dynPre, dynN, fs, 997.0, 300.0), o = residual (old, ref, dynPre, dynN, fs, 997.0, 300.0);
        std::printf ("   résidu / référence idéale, hors f0 ± 300 Hz : 1.x %.1f dB → 3.0 %.1f dB (2.0, mesuré avant ce travail : −69.3 dB) [seuil : −85 dB]\n"
                     "   (cas limite connu : ici les coefficients sont interpolés en ligne droite entre conceptions exactes)\n", o.line, a.line);
        CHECK (a.line <= -85.0, "dynamique + automation (%.1f dB)", a.line);

        // la sortie ne dépend pas de la façon dont l'hôte découpe ses blocs
        double worst = -200;
        bool exact = true;
        for (int k : { 0, 2, 5 })
        {
            std::vector<float> l1, r1, l2, r2, l3, r3;
            runDynCase (dynCases[k], 0, 480, l1, r1);
            runDynCase (dynCases[k], 0, 100, l2, r2);
            runDynCase (dynCases[k], 0, 7, l3, r3);
            exact = exact && l1 == l2 && l1 == l3 && r1 == r2 && r1 == r3;
            worst = std::max ({ worst, residual (l2, l1, 0, dynN, fs).line, residual (l3, l1, 0, dynN, fs).line, residual (r3, r1, 0, dynN, fs).line });
        }
        std::printf ("   blocs de 480, 100 ou 7 échantillons (3 cas dynamiques) : sorties %s, écart max %.1f dB [seuil : −130 dB]\n",
                     exact ? "identiques bit à bit" : "non identiques", worst);
        CHECK (worst <= -130.0, "la sortie dépend de la taille des blocs de l'hôte (%.1f dB)", worst);
    }

    // ── 15. Coût CPU (information) ───────────────────────────────────────
    std::printf ("15. Coût CPU : 10 s d'audio à 48 kHz, 24 bandes stéréo dont 8 dynamiques, blocs de 512 (meilleur de 5 passes)\n");
    {
        const double fs = 48000; const int n = 480000, blk = 512;
        std::vector<float> src ((size_t) n);
        std::mt19937 rng (3); std::uniform_real_distribution<float> U (-1, 1);
        double lp = 0;
        for (int i = 0; i < n; ++i)
        {
            // bruit coloré + notes, enveloppe rythmique (120 à la noire) : les bandes dynamiques travaillent
            lp += 0.05 * (U (rng) - lp);
            const double t = i / fs;
            double x = 0.6 * lp + 0.05 * U (rng);
            for (double f : { 82.4, 220.0, 440.0, 997.0, 2500.0, 6500.0, 11000.0 }) x += 0.07 * std::sin (2 * pi * f * t);
            src[(size_t) i] = (float) (0.5 * x * (0.6 + 0.4 * std::sin (2 * pi * 0.3 * t)) * (0.3 + 0.7 * std::exp (-6.0 * std::fmod (t, 0.5))));
        }
        const char* names[] = { "8 bandes dynamiques en action", "8 bandes dynamiques sous leur seuil", "24 bandes fixes", "8 bandes dynamiques et automatisées" };
        for (int sc = 0; sc < 4; ++sc)
        {
            double best = 1e9;
            for (int rep = 0; rep < 5; ++rep)
            {
                Engine eng; eng.prepare (fs);
                auto setAll = [&] (double mod)
                {
                    for (int b = 0; b < maxBands; ++b)
                    {
                        BandSpec s; s.active = true;
                        s.freq = 30.0 * std::pow (600.0, b / 23.0) * (b % 3 == 1 ? mod : 1.0);
                        s.shape = b == 0 ? Shape::lowShelf : b == 23 ? Shape::highShelf : Shape::bell;
                        s.q = 0.7 + 0.2 * (b % 5); s.gainDb = b % 2 ? 3.0 : -3.0;
                        DynSettings d; d.on = sc != 2 && b % 3 == 1; d.thrDb = sc == 1 ? 0.0 : -50.0 + (b % 4) * 4;
                        d.rangeDb = b % 2 ? -9.0 : 6.0; d.attackMs = 0.5 + b; d.releaseMs = 40 + 10 * b;
                        eng.setBand (b, s, d, 1.0);
                    }
                };
                setAll (1.0);
                std::vector<float> l = src, r = src;
                const auto t0 = std::chrono::steady_clock::now();
                for (int off = 0; off < n; off += blk)
                {
                    if (sc == 3) setAll (std::pow (2.0, 0.5 * std::sin (2 * pi * 0.5 * off / fs)));
                    eng.process (l.data() + off, r.data() + off, std::min (blk, n - off), false);
                }
                best = std::min (best, std::chrono::duration<double, std::milli> (std::chrono::steady_clock::now() - t0).count());
            }
            std::printf ("   %-36s : %6.1f ms (%.2f %% d'un cœur)\n", names[sc], best, best / 100.0);
        }
    }

    // ── 16. Sections de correction : coût, bande dynamique dans l'aigu ───
    // (durées mesurées sur cette machine : indicatives, sans seuil)
    std::printf ("16. Sections de correction : coût (durées indicatives, mesurées ici)\n");
    {
        std::printf ("   sections par bande à 44,1 kHz, réglages du point 1 (moyenne 2.0 → 3.0, maximum 3.0) :\n     ");
        for (int i = 0; i < 8; ++i) std::printf (" %s %.2f → %.2f (max %d)%s", fams[(size_t) i].name, secBefore[i] / secCount[i], secAfter[i] / secCount[i], secMax[i], i == 3 ? "\n     " : i < 7 ? " ;" : "\n");
        auto usPer = [] (const BandSpec& b0, double fs, int mode)
        {
            BandSpec b = b0; Design d, h; double acc = 0;
            designMatched (b, fs, h);
            const int N = 2000;
            const auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < N; ++i)
            {
                b.freq = b0.freq * std::pow (2.0, 0.25 * std::sin (i * 0.002));   // réglage qui glisse (± 1/4 d'octave)
                if (mode == 0) designMatched (b, fs, d, 0);
                else if (mode == 1) designMatched (b, fs, d);
                else { designMatched (b, fs, d, maxCorrections, &h); h = d; }
                acc += d.outGain;
            }
            const double us = std::chrono::duration<double, std::micro> (std::chrono::steady_clock::now() - t0).count() / N;
            return acc > 0 ? us : 0.0;
        };
        struct C { const char* n; Shape s; double f, g, q; int sl; };
        const C cs[] = { { "cloche 1 kHz +6 dB Q 1", Shape::bell, 1000, 6, 1, 12 }, { "cloche 8 kHz +6 dB Q 1", Shape::bell, 8000, 6, 1, 12 }, { "cloche 16 kHz −12 dB Q 4", Shape::bell, 16000, -12, 4, 12 },
                         { "high shelf 8 kHz +6 dB", Shape::highShelf, 8000, 6, 0.71, 12 }, { "notch 10 kHz Q 5", Shape::notch, 10000, 0, 5, 12 }, { "coupe-bas 80 Hz 24 dB/oct", Shape::lowCut, 80, 0, 0.71, 24 },
                         { "coupe-haut 16 kHz 24 dB/oct", Shape::highCut, 16000, 0, 0.71, 24 }, { "coupe-haut 16 kHz 96 dB/oct", Shape::highCut, 16000, 0, 0.71, 96 }, { "passe-bande 10 kHz 24 dB/oct", Shape::bandPass, 10000, 0, 2, 24 } };
        std::printf ("   temps d'une conception à 44,1 kHz (µs) : 2.0 | 3.0 première conception | 3.0 réglage qui bouge (départ = conception précédente) ; sections 2.0 → 3.0\n");
        for (const auto& c : cs)
        {
            BandSpec b; b.active = true; b.shape = c.s; b.freq = c.f; b.gainDb = c.g; b.q = c.q; b.slope = c.sl;
            Design d0, d1; designMatched (b, 44100, d0, 0); designMatched (b, 44100, d1);
            std::printf ("      %-28s %6.1f | %6.1f | %6.1f ; %d → %d\n", c.n, usPer (b, 44100, 0), usPer (b, 44100, 1), usPer (b, 44100, 2), d0.n, d1.n);
        }
        // traitement : une section SVF sur un signal
        {
            Section sc; sc.g = 0.3; sc.k = 1.0; sc.m0 = 1.0; sc.m1 = 0.5; sc.m2 = 0.1;
            SectionRT rt; toRT (sc, rt);
            double st[2] {}, acc = 0;
            const int N = 4000000;
            const auto t0 = std::chrono::steady_clock::now();
            double x = 0.1;
            for (int i = 0; i < N; ++i) { x = 0.999 * x + 1e-4; acc += tick (rt, st, x + (i & 1 ? 0.01 : -0.01)); }
            const double ns = std::chrono::duration<double, std::nano> (std::chrono::steady_clock::now() - t0).count() / N;
            std::printf ("   traitement : %.1f ns par section et par échantillon%s, soit %.3f %% d'un cœur par section de correction en stéréo à 44,1 kHz (%.3f %% à 96 kHz)\n",
                         ns, acc != 0 ? "" : " ", ns * 1e-9 * 2 * 44100 * 100, ns * 1e-9 * 2 * 96000 * 100);
        }
        // moteur complet, 44,1 kHz stéréo : 8 bandes dans l'aigu, fixes puis une cloche balayée, puis une bande dynamique
        auto engineLoad = [] (int corr, int scenario)
        {
            const double fs = 44100;
            Engine eng; eng.corrections = corr; eng.prepare (fs);
            for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
            const double fr[8] = { 3000, 5000, 7000, 9000, 11000, 13000, 15000, 17000 };
            for (int b = 0; b < 8; ++b)
            {
                BandSpec s; s.active = true; s.shape = b == 6 ? Shape::highShelf : b == 7 ? Shape::highCut : Shape::bell; s.freq = fr[b]; s.gainDb = b % 2 ? -5 : 6; s.q = b == 7 ? 0.71 : 1.5; s.slope = b == 7 ? 24 : 12;
                DynSettings d; d.on = scenario == 2 && b == 4; d.thrDb = -40; d.rangeDb = -9; d.attackMs = 1; d.releaseMs = 60;
                eng.setBand (b, s, d, 1.0);
            }
            std::mt19937 rng (3); std::uniform_real_distribution<float> U (-0.3f, 0.3f);
            const int blk = 256, nb = 800;
            std::vector<float> l ((size_t) blk), r ((size_t) blk);
            double t = 0;
            for (int k = 0; k < nb + 20; ++k)
            {
                for (int i = 0; i < blk; ++i) { l[(size_t) i] = U (rng) * (1.0f + (float) std::sin (k * 0.3)); r[(size_t) i] = l[(size_t) i]; }
                if (scenario == 1)
                {
                    BandSpec s; s.active = true; s.shape = Shape::bell; s.gainDb = 6; s.q = 1.5; s.freq = 9000.0 * std::pow (2.0, 0.8 * std::sin (k * 0.05));
                    eng.setBand (2, s, {}, 1.0);
                }
                const auto t0 = std::chrono::steady_clock::now();
                eng.process (l.data(), r.data(), blk, false);
                if (k >= 20) t += std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
            }
            return 100.0 * t / (nb * blk / fs);
        };
        const char* names[3] = { "8 bandes fixes dans l'aigu (3 – 17 kHz)", "les mêmes, dont une cloche balayée sans arrêt (5 – 16 kHz)", "les mêmes, dont une cloche dynamique à 11 kHz" };
        std::printf ("   moteur complet, 44,1 kHz stéréo, %% d'un cœur (2.0 sans correction → 3.0) :\n");
        for (int sc = 0; sc < 3; ++sc) std::printf ("      %-60s %.2f %% → %.2f %%\n", names[sc], engineLoad (0, sc), engineLoad (maxCorrections, sc));
    }

    {
        // Exactitude du gain dynamique dans l'aigu : side-chain externe constant → gain dynamique fixe X ;
        // la réponse mesurée (impulsion → FFT) doit être celle de l'analogique au gain de base + X.
        struct C { Shape s; double f, g, q, range, fs; };
        const C cs[] = { { Shape::bell, 12000, 3, 2, -9, 44100 }, { Shape::bell, 16000, 0, 1, 12, 44100 }, { Shape::highShelf, 9000, 4, 0.71, -10, 48000 },
                         { Shape::bell, 15000, -3, 4, -12, 48000 }, { Shape::highShelf, 12000, 0, 1, 8, 44100 } };
        const double tolDynHf = 0.5;
        double worst = 0, wX = 0; int wi = 0, ci = 0;
        for (const auto& c : cs)
        {
            for (double frac : { 0.13, 0.37, 0.61, 0.89 })
            {
                Engine eng; eng.prepare (c.fs);
                for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
                BandSpec s; s.active = true; s.shape = c.s; s.freq = c.f; s.gainDb = c.g; s.q = c.q;
                DynSettings d; d.on = true; d.thrDb = -40; d.rangeDb = c.range; d.attackMs = 5; d.releaseMs = 2000; d.external = true;
                eng.setBand (0, s, d, 1.0);
                // niveau du side-chain : dépassement du seuil = fraction de la plage (au-dessus du genou de 6 dB)
                const double amp = std::pow (10.0, (-40.0 + 3.0 + frac * std::abs (c.range)) / 20.0);
                const double fsc = c.s == Shape::bell ? c.f : 0.45 * c.fs * 0.9;   // (détecteur : passe-bande, ou passe-haut pour l'étagère)
                const int n = 65536, pre = 6 * 65536;
                std::vector<float> l ((size_t) n, 0.0f), r ((size_t) n, 0.0f), sc ((size_t) n);
                long t = 0;
                for (int k = 0; k < pre / n; ++k)
                {
                    for (int i = 0; i < n; ++i, ++t) sc[(size_t) i] = (float) (amp * std::sin (2 * pi * fsc * t / c.fs));
                    std::fill (l.begin(), l.end(), 0.0f); r = l;
                    eng.process (l.data(), r.data(), n, false, sc.data(), nullptr);
                }
                const double x0 = eng.getDynOffset (0);
                for (int i = 0; i < n; ++i, ++t) sc[(size_t) i] = (float) (amp * std::sin (2 * pi * fsc * t / c.fs));
                std::fill (l.begin(), l.end(), 0.0f); l[0] = 1.0f; r = l;
                eng.process (l.data(), r.data(), n, false, sc.data(), nullptr);
                const double x1 = eng.getDynOffset (0);
                std::vector<double> x (l.begin(), l.end());
                std::vector<fir::cd> X; fftReal (x, X, n);
                BandSpec e = s; e.gainDb = c.g + 0.5 * (x0 + x1);
                Design da; design (e, c.fs, da, true);
                double w = 0;
                for (int k = 1; k < n / 2; ++k)
                {
                    const double f = k * c.fs / n;
                    if (f < 20 || f > 20000) continue;
                    w = std::max (w, std::abs (db (std::abs (X[(size_t) k])) - magnitudeDb (da, f, c.fs, true)));
                }
                w = std::max (0.0, w - 0.5 * std::abs (x1 - x0));   // (dérive du gain pendant la mesure)
                if (w > worst) { worst = w; wX = 0.5 * (x0 + x1); wi = ci; }
            }
            ++ci;
        }
        std::printf ("   bande dynamique dans l'aigu (5 bandes × 4 niveaux de gain dynamique fixe, side-chain externe) : écart max à l'analogique %.3f dB\n"
                     "      (bande %d, gain dynamique %+.2f dB) [seuil %.2f : les bandes dynamiques gardent la conception 2.0, sans section de correction]\n", worst, wi + 1, wX, tolDynHf);
        CHECK (worst <= tolDynHf, "bande dynamique dans l'aigu : %.3f dB", worst);
    }

    // Bandes du prototype (tests/proto_dynfir.cpp, partie D) : coupe-bas 40 Hz, high shelf 8 kHz +4 dB,
    // cloche 1 kHz Q 2 dynamique (seuil −30 dB, retour 100 ms)
    auto protoBands = [] (FirRig& r, double att, double range, double base)
    {
        auto& b = r.bands;
        b[0].active = true; b[0].shape = Shape::lowCut; b[0].freq = 40; b[0].slope = 12; b[0].q = 0.71;
        b[1].active = true; b[1].shape = Shape::highShelf; b[1].freq = 8000; b[1].gainDb = 4; b[1].q = 0.71;
        b[2].active = true; b[2].shape = Shape::bell; b[2].freq = 1000; b[2].q = 2; b[2].gainDb = base;
        r.dyn[2].on = true; r.dyn[2].thrDb = -30; r.dyn[2].rangeDb = range; r.dyn[2].attackMs = att; r.dyn[2].releaseMs = 100;
    };

    // ── 17. Linear phase : bande dynamique, précision temporelle ────────
    std::printf ("\n17. Linear phase, bande dynamique : résidu par rapport à l'idéal (à chaque échantillon, le FIR exact du gain du détecteur)\n"
                 "    bandes et signal du prototype (FIR 4096 points, cloche 1 kHz Q 2, plage −12 dB, salves de 997 Hz) [seuil : −60 dB efficace]\n");
    {
        const double fs = 48000; const int N = 4096, n = 60000, lat0 = N / 2;
        const auto x = burstSignal (n, fs);
        fir::Work w;
        for (double att : { 0.5, 10.0 })
        {
            FirRig ref; protoBands (ref, att, -12, 0);
            // idéal : détecteur récrit à part, banque de FIR exacts tous les 0,05 dB
            const auto g = detectorGain (x, ref.bands[2], ref.dyn[2], fs);
            const int nb = 241; std::vector<std::vector<double>> bank ((size_t) nb);
            for (int i = 0; i < nb; ++i)
            {
                fir::Request rq; rq.bands = ref.bands; rq.bands[2].gainDb = -0.05 * i; rq.N = N; rq.fs = fs; rq.mode = fir::Mode::linear;
                fir::Result r; fir::design (rq, r, w); bank[(size_t) i] = r.h[0];
            }
            std::vector<double> ideal ((size_t) n, 0.0), h ((size_t) N);
            for (int i = lat0; i < n; ++i)
            {
                const double p = std::clamp (-g[(size_t) (i - lat0)] / 0.05, 0.0, nb - 1.0);
                const int i0 = std::min ((int) p, nb - 2); const double t = p - i0;
                for (int k = 0; k < N; ++k) h[(size_t) k] = bank[(size_t) i0][(size_t) k] * (1 - t) + bank[(size_t) i0 + 1][(size_t) k] * t;
                ideal[(size_t) i] = convAt (h, x, i);
            }
            Res2 res[2];
            for (int legacy = 1; legacy >= 0; --legacy)
            {
                FirRig rig; rig.legacy = legacy != 0; protoBands (rig, att, -12, 0);
                rig.start (fs, fir::Mode::linear, N);
                std::vector<float> y = x; y.resize ((size_t) n + 4096, 0.0f);
                rig.run (y.data(), nullptr, (int) y.size());
                res[legacy] = residual2 (y, rig.lp.partition(), ideal, lat0 + 4096, n);
            }
            std::printf ("   attaque %4.1f ms : 2.0 (reconception toutes les 25 ms + fondu) efficace %.1f dB, crête %.1f dB → 3.0 (mélange de FIR) efficace %.1f dB, crête %.1f dB\n",
                         att, res[1].rms, res[1].peak, res[0].rms, res[0].peak);
            CHECK (res[0].rms <= -60.0, "Linear phase, attaque %.1f ms : résidu %.1f dB", att, res[0].rms);
        }
    }

    // ── 18. Linear phase : erreur de courbe du mélange ──────────────────
    std::printf ("\n18. Linear phase : erreur de courbe du mélange de FIR pour tous les gains dynamiques intermédiaires\n"
                 "   (a) sur les FIR réels du jeu, contre le FIR conçu exactement pour ce gain (10 Hz – 20 kHz) [seuil 0,1 dB]\n");
    {
        const double fs = 48000;
        struct C { const char* n; Shape s; int slope; double f, q, base, range; int N; };
        const C cs[] = { { "cloche 1 kHz Q 2, plage −12", Shape::bell, 12, 1000, 2, 0, -12, 4096 }, { "cloche 1 kHz Q 2, plage +12", Shape::bell, 12, 1000, 2, 0, 12, 4096 },
                         { "cloche 1 kHz Q 2, plage −18", Shape::bell, 12, 1000, 2, 0, -18, 4096 }, { "cloche 1 kHz Q 2, plage +18", Shape::bell, 12, 1000, 2, 0, 18, 4096 },
                         { "cloche 3 kHz Q 8 à +6 dB, plage −12", Shape::bell, 12, 3000, 8, 6, -12, 4096 }, { "cloche 100 Hz Q 4, plage −12", Shape::bell, 12, 100, 4, 0, -12, 16384 },
                         { "low shelf 200 Hz 12 dB/oct Q 0.71, plage +12", Shape::lowShelf, 12, 200, 0.71, 0, 12, 8192 }, { "high shelf 5 kHz 6 dB/oct, plage −12", Shape::highShelf, 6, 5000, 0.71, 0, -12, 4096 },
                         { "high shelf 5 kHz 12 dB/oct Q 0.71, plage +18", Shape::highShelf, 12, 5000, 0.71, 0, 18, 4096 },
                         { "high shelf 5 kHz Q 2 (résonant), plage +12", Shape::highShelf, 12, 5000, 2, 0, 12, 4096 }, { "low shelf 200 Hz Q 2 (résonant), plage −18", Shape::lowShelf, 12, 200, 2, 0, -18, 8192 },
                         { "tilt 1 kHz 6 dB/oct, plage +12", Shape::tiltShelf, 6, 1000, 0.71, 0, 12, 4096 } };
        for (const auto& c : cs)
        {
            std::array<BandSpec, maxBands> bands;
            bands[0].active = true; bands[0].shape = Shape::lowCut; bands[0].freq = 40; bands[0].slope = 12; bands[0].q = 0.71;
            bands[1].active = true; bands[1].shape = Shape::bell; bands[1].freq = 9000; bands[1].gainDb = -3; bands[1].q = 1;
            auto& b = bands[2]; b.active = true; b.shape = c.s; b.slope = c.slope; b.freq = c.f; b.q = c.q; b.gainDb = c.base;
            fir::DynInfo d; d.full[2] = c.range; int nd = 0;
            const double e = bankCurveError (bands, d, c.N, fs, 24, &nd);
            std::printf ("      %-46s : %2d FIR d'écart, erreur max %.3f dB\n", c.n, nd, e);
            CHECK (e <= 0.1, "courbe du mélange, %s : %.3f dB", c.n, e);
        }
        std::printf ("   (b) interpolation seule (cibles analytiques, 10 Hz – 24 kHz), gain de base 0 et +6 dB, plages ±3 à ±30 dB : pire cas par famille\n");
        struct F { const char* n; Shape s; int slope; std::vector<double> q; double f, tol; bool block; };
        const F fams[] = { { "cloche, Q 0.5 à 8", Shape::bell, 12, { 0.5, 1, 2, 4, 8 }, 1000, 0.1, true },
                           { "étagères 6 dB/oct", Shape::lowShelf, 6, { 0.71 }, 200, 0.1, true },
                           { "étagères 12 dB/oct, Q 0.5 à 0.8", Shape::highShelf, 12, { 0.5, 0.71, 0.8 }, 5000, 0.1, true },
                           { "étagères 12 dB/oct, Q 1 à 2 (résonantes)", Shape::highShelf, 12, { 1, 1.5, 2 }, 5000, 0.1, true },
                           { "étagères 12 dB/oct, Q 4 (résonantes)", Shape::lowShelf, 12, { 4 }, 200, 0.3, true },
                           { "tilt 6 et 12 dB/oct, Q 0.71", Shape::tiltShelf, 12, { 0.71 }, 1000, 0.1, true } };
        for (const auto& f : fams)
        {
            double w12 = 0, w18 = 0, w30 = 0;
            for (double q : f.q) for (double base : { 0.0, 6.0 }) for (double r : { -30.0, -18.0, -12.0, -6.0, -3.0, 3.0, 6.0, 12.0, 18.0, 30.0 })
                for (int sl : { f.slope, f.s == Shape::tiltShelf ? 6 : f.slope })
                {
                    BandSpec b; b.active = true; b.shape = f.s; b.slope = sl; b.freq = f.f; b.q = q; b.gainDb = base;
                    const double e = blendCurveError (b, r, fs);
                    if (std::abs (r) <= 12) w12 = std::max (w12, e);
                    if (std::abs (r) <= 18) w18 = std::max (w18, e);
                    w30 = std::max (w30, e);
                }
            std::printf ("      %-42s : jusqu'à ±12 dB %.3f ; jusqu'à ±18 dB %.3f ; jusqu'à ±30 dB %.3f [seuil %.2f jusqu'à ±18 dB]\n", f.n, w12, w18, w30, f.tol);
            CHECK (w18 <= f.tol, "interpolation, %s : %.3f dB", f.n, w18);
        }
    }

    // ── 19. Natural phase : précision temporelle ────────────────────────
    std::printf ("\n19. Natural phase, bande dynamique faite par le moteur zéro latence après le FIR des bandes fixes\n"
                 "    résidu par rapport à « FIR fixe puis moteur exact » (conception exacte à chaque échantillon) [seuil : −110 dB efficace],\n"
                 "    écart à « FIR fixe puis moteur zéro latence » [seuil : 0, bit à bit]\n");
    {
        const double fs = 48000; const int N = 4096, n = 60000;
        const auto x = burstSignal (n, fs);
        const char* names[3] = { "cloche +3 dB sur le Mid, plage −12, stéréo", "side-chain externe (retardé de la latence), plage −10", "mono, plage −12" };
        for (int cas = 0; cas < 3; ++cas)
        {
            FirRig rig; protoBands (rig, 0.5, cas == 1 ? -10 : -12, cas == 0 ? 3 : 0);
            rig.dyn[2].external = cas == 1;
            if (cas == 0) rig.bands[2].place = Place::mid;
            rig.start (fs, fir::Mode::natural, N);
            std::vector<float> l = x, r ((size_t) n), sc ((size_t) n);
            for (int i = 0; i < n; ++i) { r[(size_t) i] = 0.6f * x[(size_t) i]; sc[(size_t) i] = x[(size_t) ((i + 5000) % n)]; }
            if (cas == 1) for (int i = 0; i < n; ++i) { l[(size_t) i] = (float) (0.2 * std::sin (2 * pi * 1000.0 * i / fs)); r[(size_t) i] = l[(size_t) i]; }
            const auto l0 = l, r0 = r;
            rig.run (l.data(), cas == 2 ? nullptr : r.data(), n, cas == 1 ? sc.data() : nullptr);
            Res2 res[2]; double mx[2] {};
            for (int exact = 1; exact >= 0; --exact)
            {
                TestLinear lp; lp.prepare (fs);
                TestLinear::Snapshot s; s.bands = rig.bands; s.bands[2].active = false;
                lp.design (s, fir::Mode::natural, N, false); lp.switchTo (fir::Mode::natural, N);
                auto a = l0, b = r0;
                lp.process (a.data(), cas == 2 ? nullptr : b.data(), n);
                Engine e; e.referenceExact = exact != 0; e.prepare (fs);
                for (int k = 0; k < maxBands; ++k) e.setBand (k, BandSpec(), DynSettings(), 1.0);
                e.setBand (2, rig.bands[2], rig.dyn[2], 1.0);
                std::vector<float> scd ((size_t) n, 0.0f);
                for (int i = rig.lat; i < n; ++i) scd[(size_t) i] = sc[(size_t) (i - rig.lat)];
                e.process (a.data(), cas == 2 ? nullptr : b.data(), n, false, cas == 1 ? scd.data() : nullptr, nullptr);
                // (après la mise en route : le fondu d'entrée de la bande n'avance pas au même pas dans la référence exacte)
                res[exact] = residual2 (l, 0, a, 8192, n);
                for (int i = 8192; i < n; ++i) mx[exact] = std::max ({ mx[exact], (double) std::abs (l[(size_t) i] - a[(size_t) i]), cas == 2 ? 0.0 : (double) std::abs (r[(size_t) i] - b[(size_t) i]) });
            }
            std::printf ("   %-54s : latence %d ; contre le moteur exact %.1f dB efficace, %.1f dB crête ; contre le moteur zéro latence : écart max %.1e\n",
                         names[cas], rig.lat, res[1].rms, res[1].peak, mx[0]);
            CHECK (res[1].rms <= -110.0, "Natural phase, %s : résidu %.1f dB", names[cas], res[1].rms);
            CHECK (mx[0] == 0.0, "Natural phase, %s : la bande dynamique n'est pas celle du moteur zéro latence (%.1e)", names[cas], mx[0]);
            CHECK (std::abs (rig.eng.getDynOffset (2)) > 3.0, "Natural phase, %s : la dynamique n'a pas agi", names[cas]);
        }
    }

    // ── 20. Bandes fixes inchangées ─────────────────────────────────────
    std::printf ("\n20. Bandes fixes inchangées dans les modes FIR\n");
    {
        const double fs = 48000; const int N = 4096, n = 30000;
        std::mt19937 rng (7); std::uniform_real_distribution<float> U (-0.5f, 0.5f);
        std::vector<float> xl ((size_t) n), xr ((size_t) n);
        for (int i = 0; i < n; ++i) { xl[(size_t) i] = U (rng); xr[(size_t) i] = U (rng); }
        auto fixed = [] (FirRig& r)
        {
            auto& b = r.bands;
            b[0].active = true; b[0].shape = Shape::lowCut; b[0].freq = 60; b[0].slope = 24; b[0].q = 0.71;
            b[1].active = true; b[1].shape = Shape::bell; b[1].freq = 900; b[1].gainDb = 7; b[1].q = 3; b[1].place = Place::mid;
            b[2].active = true; b[2].shape = Shape::highShelf; b[2].freq = 7000; b[2].gainDb = -5; b[2].q = 0.71;
            b[3].active = true; b[3].shape = Shape::notch; b[3].freq = 3000; b[3].q = 8; b[3].place = Place::right;
        };
        for (auto mode : { fir::Mode::linear, fir::Mode::natural })
        {
            const char* mn = mode == fir::Mode::linear ? "Linear " : "Natural";
            // (a) contre la convolution directe (double précision) par les FIR conçus : c'est le comportement de la 2.0
            FirRig a; fixed (a); a.start (fs, mode, N);
            auto al = xl, ar = xr;
            a.run (al.data(), ar.data(), n);
            fir::Request rq; rq.bands = a.bands; rq.N = N; rq.fs = fs; rq.mode = mode; rq.delay = fir::naturalDelay (fs);
            fir::Result r; fir::Work w; fir::design (rq, r, w);
            const int P = a.lp.partition(), n0 = 6000;
            std::vector<double> refL ((size_t) n0, 0.0), refR ((size_t) n0, 0.0);
            for (int i = N; i < n0; ++i)
            {
                refL[(size_t) i] = convAt (r.h[0], xl, i) + convAt (r.h[1], xr, i);
                refR[(size_t) i] = convAt (r.h[2], xl, i) + convAt (r.h[3], xr, i);
            }
            const double eL = residual2 (al, P, refL, N, n0).rms, eR = residual2 (ar, P, refR, N, n0).rms;
            // (b) une bande de plus, dynamique mais sous son seuil (gain de base 0 dB) : sortie identique bit à bit
            FirRig bq; fixed (bq);
            bq.bands[4].active = true; bq.bands[4].freq = 1500; bq.bands[4].q = 2;
            bq.dyn[4].on = true; bq.dyn[4].thrDb = 0; bq.dyn[4].rangeDb = -12;
            bq.start (fs, mode, N);
            auto bl = xl, br = xr;
            bq.run (bl.data(), br.data(), n);
            double mx = 0;
            for (int i = 0; i < n; ++i) mx = std::max ({ mx, (double) std::abs (al[(size_t) i] - bl[(size_t) i]), (double) std::abs (ar[(size_t) i] - br[(size_t) i]) });
            const auto plan = bq.lp.planFor (bq.snapshot(), mode, N);
            std::printf ("   %s : sortie contre la convolution directe par le FIR des bandes fixes %.1f dB (G), %.1f dB (D) [seuil −120 dB] ;\n"
                         "             avec en plus une bande dynamique sous son seuil (%s) : écart max %.1e [seuil 0]\n",
                         mn, eL, eR, mode == fir::Mode::linear ? "mélange de FIR" : "moteur", mx);
            CHECK (eL <= -120.0 && eR <= -120.0, "%s : bandes fixes modifiées (%.1f / %.1f dB)", mn, eL, eR);
            CHECK (mx == 0.0, "%s : une bande dynamique au repos change la sortie (%.1e)", mn, mx);
            CHECK ((mode == fir::Mode::linear ? plan.mixMask : plan.engineMask) == (1u << 4), "%s : répartition de la bande dynamique", mn);
        }
        std::printf ("   (ancien LinearPhase.h contre le nouveau cœur, mêmes FIR, même FFT : comparaison bit à bit faite hors dépôt, voir le rapport)\n");
    }

    // ── 21. Deux bandes dynamiques qui se recouvrent ────────────────────
    std::printf ("\n21. Linear phase : bandes dynamiques qui se recouvrent (FIR croisés sur la grille des ancres de chaque paire couplée)\n");
    {
        const double fs = 48000;
        struct C { const char* n; double ratio, q, r1, r2, tol; };
        const C cs[] = { { "cloches Q 1 à ½ octave, plages −12 / −12", std::sqrt (2.0), 1, -12, -12, 0.1 }, { "cloches Q 1 à ½ octave, plages +12 / +12", std::sqrt (2.0), 1, 12, 12, 0.1 },
                         { "cloches Q 1 à 1 octave, plages −12 / +9", 2, 1, -12, 9, 0.1 }, { "cloches Q 1 à 2 octaves, plages −6 / −6", 4, 1, -6, -6, 0.1 },
                         { "cloches Q 4 à ½ octave, plages −18 / −18", std::sqrt (2.0), 4, -18, -18, 0.1 } };
        std::printf ("   (a) erreur de courbe sur les FIR réels, grille 10 × 10 de gains [seuil 0,1 dB]\n");
        for (const auto& c : cs)
        {
            std::array<BandSpec, maxBands> bands;
            bands[0].active = true; bands[0].shape = Shape::lowCut; bands[0].freq = 40; bands[0].slope = 12; bands[0].q = 0.71;
            bands[3].active = true; bands[3].freq = 1000; bands[3].q = c.q;
            bands[5].active = true; bands[5].freq = 1000 * c.ratio; bands[5].q = c.q;
            fir::DynInfo d; d.full[3] = c.r1; d.full[5] = c.r2;
            int nd = 0;
            const double e = bankCurveError (bands, d, 4096, fs, 9, &nd);
            const auto pl = fir::makePlan (bands, d, fir::Mode::linear, fs, true);
            const double cp = fir::coupling (bands[3], c.r1, bands[5], c.r2, fs, true);
            std::printf ("      %-42s : erreur du mélange additif %s dB → %2d FIR d'écart, erreur max %.3f dB\n", c.n,
                         cp > 90 ? "> 90 (signe inversé)" : fmt2 (cp).c_str(), nd, e);
            CHECK (pl.nEdges == 1 && pl.nMix == 2, "%s : les deux bandes devraient être traitées en paire", c.n);
            CHECK (e <= c.tol, "%s : %.3f dB", c.n, e);
        }
        // (b) résidu temporel : cloches 1 kHz et 1,41 kHz Q 1, plages −12 dB, détecteurs différents
        const int N = 4096, n = 40000, lat0 = N / 2;
        std::vector<float> x ((size_t) n);
        {
            const auto a = burstSignal (n, fs, 997.0, 0.3);
            for (int i = 0; i < n; ++i)
            {
                const double ph = std::fmod (i / fs, 0.17);
                x[(size_t) i] = a[(size_t) i] + (float) (0.25 * (ph < 0.05 ? std::min (1.0, ph / 0.002) : std::exp (-(ph - 0.05) / 0.02)) * std::sin (2 * pi * 1493.0 * i / fs));
            }
        }
        auto setup = [] (FirRig& r)
        {
            auto& b = r.bands;
            b[0].active = true; b[0].shape = Shape::lowCut; b[0].freq = 40; b[0].slope = 12; b[0].q = 0.71;
            b[1].active = true; b[1].freq = 1000; b[1].q = 1;
            b[2].active = true; b[2].freq = 1414; b[2].q = 1;
            r.dyn[1].on = true; r.dyn[1].thrDb = -30; r.dyn[1].rangeDb = -12; r.dyn[1].attackMs = 0.5; r.dyn[1].releaseMs = 100;
            r.dyn[2].on = true; r.dyn[2].thrDb = -26; r.dyn[2].rangeDb = -12; r.dyn[2].attackMs = 5; r.dyn[2].releaseMs = 60;
        };
        FirRig ref; setup (ref);
        const auto g1 = detectorGain (x, ref.bands[1], ref.dyn[1], fs), g2 = detectorGain (x, ref.bands[2], ref.dyn[2], fs);
        // idéal : FIR conçu exactement pour les deux gains du moment, un échantillon sur 13
        const long step = 13;
        std::vector<double> ideal ((size_t) n, 0.0);
        fir::Work w; fir::Result r; double gmin1 = 0, gmin2 = 0;
        for (long i = lat0 + 4096; i < n; i += step)
        {
            fir::Request rq; rq.bands = ref.bands; rq.N = N; rq.fs = fs; rq.mode = fir::Mode::linear;
            rq.bands[1].gainDb += g1[(size_t) (i - lat0)]; rq.bands[2].gainDb += g2[(size_t) (i - lat0)];
            gmin1 = std::min (gmin1, g1[(size_t) (i - lat0)]); gmin2 = std::min (gmin2, g2[(size_t) (i - lat0)]);
            fir::design (rq, r, w);
            ideal[(size_t) i] = convAt (r.h[0], x, i);
        }
        Res2 res[2];
        for (int legacy = 1; legacy >= 0; --legacy)
        {
            FirRig rig; rig.legacy = legacy != 0; setup (rig); rig.start (fs, fir::Mode::linear, N);
            std::vector<float> y = x; y.resize ((size_t) n + 4096, 0.0f);
            rig.run (y.data(), nullptr, (int) y.size());
            res[legacy] = residual2 (y, rig.lp.partition(), ideal, lat0 + 4096, n, step);
        }
        std::printf ("   (b) résidu par rapport à l'idéal, cloches 1 kHz et 1,41 kHz Q 1, gains dynamiques jusqu'à %.1f et %.1f dB :\n"
                     "       2.0 efficace %.1f dB, crête %.1f dB → 3.0 efficace %.1f dB, crête %.1f dB [seuil : −60 dB efficace]\n",
                     gmin1, gmin2, res[1].rms, res[1].peak, res[0].rms, res[0].peak);
        CHECK (res[0].rms <= -60.0, "deux bandes dynamiques qui se recouvrent : résidu %.1f dB", res[0].rms);

        // (c) chaîne de bandes couplées deux à deux (cas courant : 4 cloches dynamiques réparties sur le spectre)
        {
            std::array<BandSpec, maxBands> bands; fir::DynInfo d;
            bands[0].active = true; bands[0].shape = Shape::lowCut; bands[0].freq = 40; bands[0].slope = 12; bands[0].q = 0.71;
            const double fr[4] = { 150, 700, 3000, 9000 };
            for (int i = 0; i < 4; ++i) { auto& b = bands[(size_t) (2 + i)]; b.active = true; b.freq = fr[i]; b.q = 4; b.gainDb = 2; d.full[(size_t) (2 + i)] = -12; }
            int nd = 0, nm = 0;
            const double e = bankCurveError (bands, d, 4096, fs, 6, &nd, &nm);
            const auto pl = fir::makePlan (bands, d, fir::Mode::linear, fs, true);
            std::printf ("   (c) chaîne : 4 cloches Q 4 à 150 / 700 / 3000 / 9000 Hz, plages −12 dB : %d bandes mélangées, %d paires, %d FIR d'écart,\n"
                         "       erreur de couplage restante prévue %.3f dB ; erreur de courbe mesurée sur les FIR réels (49 combinaisons de gains) %.3f dB [seuil 0,1 dB]\n",
                         nm, pl.nEdges, nd, pl.coupleErr, e);
            CHECK (nm == 4 && pl.fallbackMask == 0, "chaîne de 4 bandes : toutes devraient être mélangées");
            CHECK (e <= 0.1, "chaîne de 4 bandes : %.3f dB", e);
        }
        // (d) répartition : 4 bandes mélangées au plus ; une bande dont l'ajout laisserait plus de 0,2 dB d'erreur
        // de couplage (ici la 3e de trois cloches larges très proches) reste en méthode 2.0
        {
            std::array<BandSpec, maxBands> bands; fir::DynInfo d;
            const double fr[6] = { 1000, 1200, 1440, 60, 12000, 250 };
            for (int i = 0; i < 6; ++i) { bands[(size_t) i].active = true; bands[(size_t) i].freq = fr[i]; bands[(size_t) i].q = i < 3 ? 1.0 : 8.0; d.full[(size_t) i] = i < 3 ? -12.0 : -6.0; d.now[(size_t) i] = -4; }
            const auto p = fir::makePlan (bands, d, fir::Mode::linear, fs, true);
            const auto pn = fir::makePlan (bands, d, fir::Mode::natural, fs, true);
            std::printf ("   (d) 6 bandes dynamiques (1 / 1,2 / 1,44 kHz Q 1 à −12 dB, puis 60 Hz, 12 kHz, 250 Hz Q 8 à −6 dB) : mélangées %#x, méthode 2.0 %#x, %d paire(s), %d FIR d'écart, couplage restant %.3f dB ;\n"
                         "       Natural phase : toutes au moteur (%#x)\n", p.mixMask, p.fallbackMask, p.nEdges, p.nDev, p.coupleErr, pn.engineMask);
            CHECK (p.mixMask == 0x1bu && p.fallbackMask == 0x24u && p.coupleErr <= fir::mixTol && pn.engineMask == 0x3fu && pn.mixMask == 0, "répartition des bandes dynamiques");
            // les bandes en trop : sortie finie, et le FIR de référence suit leur gain du moment
            FirRig rig;
            rig.bands = bands;
            for (int i = 0; i < 6; ++i) { rig.dyn[(size_t) i].on = true; rig.dyn[(size_t) i].thrDb = -40; rig.dyn[(size_t) i].rangeDb = d.full[(size_t) i]; }
            rig.start (fs, fir::Mode::linear, N);
            std::vector<float> y = x; bool bad = false;
            for (int o = 0; o + 1200 <= n; o += 1200) { rig.run (y.data() + o, nullptr, 1200); rig.redesign(); }
            for (float v : y) bad = bad || ! std::isfinite (v) || std::abs (v) > 10.0f;
            CHECK (! bad, "sortie invalide avec des bandes dynamiques en trop");
        }
    }

    // (e) plus de 4 bandes dynamiques sans recouvrement : les bandes en trop (méthode 2.0) ne font reconcevoir
    // que le FIR de référence ; les FIR d'écart des 4 autres sont gardés. Erreur de courbe ajoutée et temps gagné.
    {
        const double fs = 48000; const int N = 8192;
        FirRig rig;
        const double fr[6] = { 60, 250, 1000, 4000, 12000, 500 };
        for (int i = 0; i < 6; ++i)
        {
            auto& b = rig.bands[(size_t) i]; b.active = true; b.freq = fr[i]; b.q = 8; b.gainDb = 1;
            auto& d = rig.dyn[(size_t) i]; d.on = true; d.thrDb = -40; d.rangeDb = -6;
        }
        rig.start (fs, fir::Mode::linear, N);
        const auto snap = rig.snapshot();
        const auto plan = rig.lp.planFor (snap, fir::Mode::linear, N);
        // jeu conçu avec les bandes en trop au repos, puis FIR de référence seul reconçu avec ces bandes à pleine plage
        fir::Request rq; rq.bands = rig.bands; rq.N = N; rq.fs = fs; rq.mode = fir::Mode::linear;
        fir::DynInfo d0 = snap.dyn, d1 = snap.dyn;
        for (int i = 0; i < maxBands; ++i) { d0.now[(size_t) i] = 0; d1.now[(size_t) i] = (plan.fallbackMask >> i) & 1u ? -6.0 : 0.0; }
        fir::Work w; fir::Result ref0, ref1, exact; std::vector<fir::Result> dev;
        fir::designBank (rq, d0, plan, ref0, dev, w);
        fir::designBank (rq, d1, plan, ref1, dev, w, true);
        double wt[fir::maxMix][fir::maxSeg + 1] {}, c[fir::maxDev];
        fir::Request re = rq;
        for (int i = 0; i < maxBands; ++i) if ((plan.fallbackMask >> i) & 1u) re.bands[(size_t) i].gainDb += -6.0;
        for (int k = 0; k < plan.nMix; ++k)
        {
            const double off = plan.mb[k].full * (k % 2 == 0 ? 1.0 : 0.5);
            int seg; double a; fir::locate (plan.mb[k], off, seg, a);
            wt[k][seg] = 1 - a; wt[k][seg + 1] = a;
            re.bands[(size_t) plan.mb[k].band].gainDb += off;
        }
        fir::devWeights (plan, wt, c);
        std::vector<double> h = ref1.h[0];
        for (int di = 0; di < plan.nDev; ++di) for (int n = 0; n < N; ++n) h[(size_t) n] += c[di] * dev[(size_t) di].h[0][(size_t) n];
        fir::design (re, exact, w);
        double err = 0;
        for (int i = 0; i < 400; ++i)
        {
            const double f = 20.0 * std::pow (1000.0, i / 399.0);
            err = std::max (err, std::abs (db (std::abs (fir::responseAt (h, N / 2, fs, f)) / std::abs (fir::responseAt (exact.h[0], N / 2, fs, f)))));
        }
        // temps d'une reconception : jeu complet, puis FIR de référence seul (gain du moment des bandes en trop changé)
        rig.designSec = 0; rig.redesign(); const double tFull0 = rig.designSec;   // (rien n'a changé : FIR d'écart gardés)
        std::vector<float> x = burstSignal (24000, fs, 497.0, 0.4);
        rig.run (x.data(), nullptr, (int) x.size());
        rig.designSec = 0; rig.redesign(); const double tRef = rig.designSec;
        TestLinear fresh; fresh.prepare (fs);
        const auto t0 = std::chrono::steady_clock::now();
        fresh.design (snap, fir::Mode::linear, N, false);
        const double tFull = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
        (void) tFull0;
        std::printf ("   (e) 6 cloches dynamiques Q 8 sans recouvrement (FIR %d points) : mélangées %#x, méthode 2.0 %#x (recouvrent une bande mélangée : %s), %d FIR d'écart ;\n"
                     "       bandes en trop passées de 0 à −6 dB avec les FIR d'écart gardés : erreur de courbe max %.3f dB [seuil 0,15 dB : niveau atteint] ;\n"
                     "       reconception : jeu complet %.1f ms, FIR de référence seul %.1f ms\n",
                     N, plan.mixMask, plan.fallbackMask, plan.fallbackCoupled ? "oui" : "non", plan.nDev, err, 1000.0 * tFull, 1000.0 * tRef);
        CHECK (plan.mixMask == 0x0fu && plan.fallbackMask == 0x30u && ! plan.fallbackCoupled, "répartition de 6 bandes sans recouvrement");
        CHECK (err <= 0.15, "FIR d'écart gardés : %.3f dB", err);
        CHECK (tRef < 0.6 * tFull, "la reconception du FIR de référence seul devrait être plus courte (%.1f ms contre %.1f ms)", 1000.0 * tRef, 1000.0 * tFull);
    }

    // ── 22. Bande qui devient dynamique, ou qui cesse de l'être ─────────
    std::printf ("\n22. Passage dyn on / off d'une bande pendant la lecture (cloche 1 kHz +6 dB Q 2, signal sous le seuil : la sortie idéale ne change pas)\n"
                 "    écart de crête à la sortie sans bascule, par rapport à la crête du signal ; le FIR est reconçu 25 ms après la bascule\n");
    {
        const double fs = 48000; const int N = 4096, n = 72000;
        for (auto mode : { fir::Mode::natural, fir::Mode::linear })
            for (int sig = 0; sig < 2; ++sig)
            {
                std::vector<float> x ((size_t) n);
                std::mt19937 rng (9); std::uniform_real_distribution<double> U (-1, 1);
                for (int i = 0; i < n; ++i) x[(size_t) i] = sig == 0 ? (float) (0.05 * std::sin (2 * pi * 997.0 * i / fs)) : (float) (0.03 * U (rng));
                std::vector<float> out[2];
                for (int toggled = 0; toggled < 2; ++toggled)
                {
                    FirRig rig;
                    auto& b = rig.bands;
                    b[0].active = true; b[0].shape = Shape::lowCut; b[0].freq = 40; b[0].slope = 12; b[0].q = 0.71;
                    b[1].active = true; b[1].freq = 1000; b[1].q = 2; b[1].gainDb = 6;
                    rig.dyn[1].thrDb = -6; rig.dyn[1].rangeDb = -12;
                    rig.start (fs, mode, N);
                    auto& y = out[toggled]; y = x;
                    int done = 0;
                    auto go = [&] (int until) { rig.run (y.data() + done, nullptr, until - done); done = until; };
                    go (24000);
                    if (toggled) { rig.dyn[1].on = true; rig.apply(); }
                    go (25200); if (toggled) rig.redesign();
                    go (48000);
                    if (toggled) { rig.dyn[1].on = false; rig.apply(); }
                    go (49200); if (toggled) rig.redesign();
                    go (n);
                }
                auto pk = [&] (int a, int b2)
                {
                    double e = 0, s = 0;
                    for (int i = a; i < b2; ++i) { e = std::max (e, (double) std::abs (out[1][(size_t) i] - out[0][(size_t) i])); s = std::max (s, (double) std::abs (out[0][(size_t) i])); }
                    return std::max (-200.0, db (e / s));
                };
                const double on = pk (24000, 30000), steady = pk (36000, 48000), offT = pk (48000, 54000), after = pk (60000, n);
                const bool nat = mode == fir::Mode::natural;
                const double tol = nat ? -38.0 : -120.0;
                std::printf ("   %s, %s : passage en dynamique %.1f dB ; régime établi en dynamique %.1f dB ; retour en fixe %.1f dB ; ensuite %.1f dB [seuil %.0f dB]\n",
                             nat ? "Natural" : "Linear ", sig == 0 ? "sinus 997 Hz" : "bruit blanc ", on, steady, offT, after, tol);
                CHECK (on <= tol && offT <= tol && after <= -120.0, "%s : transitoire au passage dyn on / off (%.1f / %.1f dB)", nat ? "Natural" : "Linear", on, offT);
            }
        std::printf ("   (Natural phase : en régime établi, l'écart est la différence entre la bande faite par le FIR et la même bande faite par le moteur ;\n"
                     "    le seuil de −38 dB est le niveau atteint, pas une cible fixée d'avance)\n");
    }

    // ── 23. Coût des modes FIR avec des bandes dynamiques ───────────────
    std::printf ("\n23. Coût des modes FIR, stéréo 48 kHz, 3 s de bruit modulé (banc : FFT lente en double ; le plugin est mesuré par ALEQProcessorTest)\n"
                 "    2.0 : fil audio + fil de conception (une reconception toutes les 25 ms) ; 3.0 : fil audio seul, le jeu de FIR n'est conçu qu'une fois\n");
    {
        const double fs = 48000; const int n = 144000;
        std::vector<float> xl ((size_t) n), xr ((size_t) n);
        std::mt19937 rng (5); std::uniform_real_distribution<double> U (-0.3, 0.3);
        for (int i = 0; i < n; ++i) { const double v = (0.5 + 0.5 * std::sin (2 * pi * 3.0 * i / fs)) * U (rng); xl[(size_t) i] = (float) v; xr[(size_t) i] = (float) (0.8 * v); }
        for (auto mode : { fir::Mode::linear, fir::Mode::natural })
            for (int N : { 8192, 32768 })
            {
                if (mode == fir::Mode::natural && N != 8192) continue;
                double base = 0;
                for (int nd : { 0, 1, 4 })
                {
                    double audio[2] {}, des[2] {}, firs = 0; int nDev = 0, nGroups = 0, nFall = 0; double once = 0;
                    for (int legacy = 1; legacy >= 0; --legacy)
                    {
                        FirRig rig; rig.legacy = legacy != 0;
                        auto& b = rig.bands;
                        b[0].active = true; b[0].shape = Shape::lowCut; b[0].freq = 40; b[0].slope = 12; b[0].q = 0.71;
                        b[1].active = true; b[1].shape = Shape::highShelf; b[1].freq = 12000; b[1].gainDb = 3; b[1].q = 0.71;
                        const double fr[4] = { 150, 700, 3000, 9000 };
                        for (int k = 0; k < 4; ++k)
                        {
                            b[(size_t) (2 + k)].active = true; b[(size_t) (2 + k)].freq = fr[k]; b[(size_t) (2 + k)].q = 4; b[(size_t) (2 + k)].gainDb = 2;
                            auto& d = rig.dyn[(size_t) (2 + k)];
                            if (k < nd) { d.on = true; d.thrDb = -45; d.rangeDb = -12; d.attackMs = 5; d.releaseMs = 80; }
                        }
                        rig.start (fs, mode, N);
                        once = rig.designSec; rig.designSec = 0;
                        auto l = xl, r = xr;
                        const auto t0 = std::chrono::steady_clock::now();
                        rig.run (l.data(), r.data(), n);
                        const double sec = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
                        des[legacy] = 100.0 * rig.designSec / (n / fs);
                        audio[legacy] = 100.0 * (sec - rig.designSec) / (n / fs);
                        if (! legacy)
                        {
                            firs = (double) rig.lp.totalFirCount() / (2.0 * (double) rig.lp.blockCount());
                            const auto pl = rig.lp.planFor (rig.snapshot(), mode, N); nDev = pl.nDev; nGroups = pl.nEdges; nFall = pl.fallbackMask != 0 ? 1 : 0;
                        }
                    }
                    if (nd == 0) base = audio[0];
                    std::printf ("   %s %5d points, %d bande(s) dynamique(s) : 2.0 audio %5.2f %% + conception %6.2f %% → 3.0 audio %5.2f %% (%.2f FIR par bloc en moyenne ; jeu : %d paire(s), %d FIR d'écart, conçu en %.0f ms)\n",
                                 mode == fir::Mode::linear ? "Linear " : "Natural", N, nd, audio[1], des[1], audio[0], firs, nGroups, nDev, 1000.0 * once);
                    // seuil : pas plus de FIR par bloc que la structure ne le prévoit (référence + 2 par bande seule ou 4 par paire, marge pour les blocs où un gain change de tronçon)
                    CHECK (nFall == 0, "aucune bande ne devrait rester en méthode 2.0 dans ce cas");
                    if (mode == fir::Mode::linear) CHECK (firs <= 1.0 + 3.0 * nd, "nombre de FIR par bloc (%.2f)", firs);
                    else CHECK (firs <= 1.0001, "Natural phase : un seul FIR par bloc (%.2f)", firs);
                }
                (void) base;
            }
    }

    // ── 24 à 29 : formes et pentes 3.0 ───────────────────────────────────
    testShapes30();

    // ── 30 à 34 : Auto gain à sonie égale, pré-écho, résolution Auto ─────
    testGain30();

    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
