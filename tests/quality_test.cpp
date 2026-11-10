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
#include "EqEngine.h"
#include "FirDesign.h"
#include "LinearPhaseCore.h"
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

int main()
{
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


    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
