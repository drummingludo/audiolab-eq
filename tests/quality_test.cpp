// Banc qualité d'Audio Lab EQ 2.0, SANS JUCE (moteur seul) :
//   clang++ -O2 -std=c++17 -I Source tests/quality_test.cpp -o qt && ./qt
// Mesures chiffrées avec seuils ; renvoie 1 si un seuil n'est pas tenu.
//  1. Zéro latence « matched » : écart à l'analogique jusqu'à 20 kHz, 44,1 / 48 / 96 / 192 kHz (avant = bilinéaire 1.x)
//  2. Réponse mesurée du moteur = courbe affichée (impulsion → FFT)
//  3. Même son d'une fréquence d'échantillonnage à l'autre
//  4. Tests de nul (bandes à 0 dB : sortie = entrée, bit à bit)
//  5. EQ dynamique : raies parasites sur un sinus avec attaque rapide (avant / après)
//  6. Balayage rapide d'un réglage : bruit d'escalier (avant / après)
//  7. Phase linéaire (Auto, 48 kHz) : cloche 50 Hz −12 dB Q 8, notch 60 Hz Q 10, coupe-bas 30 Hz 48 dB/oct (avant = 1.x)
//  8. Natural phase : module et phase = analogique
//  9. Latences annoncées
// 10. Side-chain externe
// 11. Stabilité (réglages aléatoires)
// 12. EQ dynamique : résidu par rapport à la référence idéale (conception exacte à chaque échantillon), 10 cas
// 13. Cache des bandes dynamiques : précision de l'interpolation en gain, cellules irrégulières
// 14. Bande dynamique ET automatisée en même temps ; sortie indépendante de la taille des blocs de l'hôte
// 15. Coût CPU du moteur (information, sans seuil)
#include "EqEngine.h"
#include "FirDesign.h"
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

using namespace audiolab::eq;
static int failures = 0;
#define CHECK(cond, ...) do { if (! (cond)) { ++failures; std::printf ("  ÉCHEC : "); std::printf (__VA_ARGS__); std::printf ("\n"); } } while (0)

static double db (double x) { return 20.0 * std::log10 (std::max (1e-15, x)); }

// écart max (dB) entre la conception numérique et le prototype analogique, 20 Hz – 20 kHz
static double errVsAnalog (const BandSpec& b, double fs, bool bilinear, double floorDb, double* atF = nullptr)
{
    Design dd, da;
    if (bilinear) designBilinear (b, fs, dd); else design (b, fs, dd, false);
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

int main()
{
    std::printf ("Audio Lab EQ 2.0 : banc qualité (sans JUCE)\n\n");

    // ── 1. Matched ──────────────────────────────────────────────────────
    std::printf ("1. Zéro latence : écart max à l'analogique jusqu'à 20 kHz (avant = bilinéaire 1.x → après = matched 2.0)\n");
    {
        BandSpec b; b.active = true; b.shape = Shape::bell; b.freq = 16000; b.gainDb = 6; b.q = 1;
        Design d, a, l; design (b, 44100, d, false); design (b, 44100, a, true); designBilinear (b, 44100, l);
        const double v = magnitudeDb (d, 20000, 44100, false), va = magnitudeDb (a, 20000, 44100, true), vl = magnitudeDb (l, 20000, 44100, false);
        std::printf ("   cloche 16 kHz +6 dB Q 1 à 20 kHz (44,1 kHz) : analogique %+.2f dB ; avant %+.2f ; après %+.2f\n", va, vl, v);
        CHECK (std::abs (v - va) <= 0.5, "cloche 16 kHz : %.2f dB au lieu de %.2f", v, va);
    }
    struct Fam { const char* name; Shape s; std::vector<double> g, q; std::vector<int> sl; double floor; double fmax; double tol441, tolHi; };
    // tolérances : cloches / étagères / tilt ≤ 0,5 dB partout ; coupes et passe-bande : zone utile (≥ −24 dB),
    // fréquences de coupure jusqu'à 10 kHz (au-delà : limite connue près de Nyquist, voir la fiche)
    const std::vector<Fam> fams = {
        { "cloche",      Shape::bell,      { -18, -12, -6, 6, 12, 18 }, { 0.1, 0.3, 0.71, 1, 2, 5, 10, 40 }, { 12 },   -200, 20000, 0.5, 0.5 },
        { "low shelf",   Shape::lowShelf,  { -18, -6, 6, 18 },          { 0.3, 0.71, 1, 2 },                 { 6, 12 }, -200, 20000, 0.5, 0.5 },
        { "high shelf",  Shape::highShelf, { -18, -6, 6, 18 },          { 0.3, 0.71, 1, 2 },                 { 6, 12 }, -200, 20000, 0.5, 0.5 },
        { "tilt",        Shape::tiltShelf, { -12, -6, 6, 12 },          { 0.71, 1, 2 },                      { 6, 12 }, -200, 20000, 0.5, 0.5 },
        { "notch",       Shape::notch,     { 0 },                       { 1, 2, 5, 10, 40 },                 { 12 },    -3,   20000, 0.7, 0.7 },
        { "coupe-bas",   Shape::lowCut,    { 0 },                       { 0.71, 1, 2 },                      { 6, 12, 18, 24, 48, 96 }, -24, 5000, 0.5, 0.5 },
        { "coupe-haut",  Shape::highCut,   { 0 },                       { 0.71, 1, 2 },                      { 6, 12, 18, 24, 48, 96 }, -24, 20000, 1.5, 1.0 },
        { "passe-bande", Shape::bandPass,  { 0 },                       { 0.71, 1, 2, 5 },                   { 12, 24 }, -24, 10000, 1.5, 1.0 } };
    for (double fs : { 44100.0, 48000.0, 96000.0, 192000.0 })
    {
        std::printf ("   %.1f kHz :\n", fs / 1000);
        for (const auto& fm : fams)
        {
            double wa = 0, wb = 0, wf = 0, wFreq = 0;
            for (double f : { 30.0, 100.0, 300.0, 1000.0, 3000.0, 5000.0, 8000.0, 10000.0, 12000.0, 16000.0, 20000.0 })
            {
                if (f > fm.fmax) continue;
                for (double g : fm.g) for (double q : fm.q) for (int sl : fm.sl)
                {
                    BandSpec b; b.active = true; b.shape = fm.s; b.freq = f; b.gainDb = g; b.q = q; b.slope = sl;
                    double at = 0;
                    const double e = errVsAnalog (b, fs, false, fm.floor, &at);
                    if (e > wa) { wa = e; wf = at; wFreq = f; }
                    wb = std::max (wb, errVsAnalog (b, fs, true, fm.floor));
                }
            }
            const double tol = fs < 46000 ? fm.tol441 : fm.tolHi;
            std::printf ("      %-11s avant %6.2f dB → après %5.2f dB (pire : bande à %.0f Hz, écart à %.0f Hz) [seuil %.1f]\n", fm.name, wb, wa, wFreq, wf, tol);
            CHECK (wa <= tol, "%s à %.0f Hz : écart %.2f dB", fm.name, fs, wa);
        }
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
    }

    // ── 3. Même son à toutes les fréquences d'échantillonnage ─────────────
    std::printf ("3. Même courbe à 44,1 / 48 / 96 / 192 kHz (écart max à la courbe 192 kHz, 20 Hz – 20 kHz)\n");
    {
        struct Case { Shape s; double f, g, q; int sl; };
        const Case cases[] = { { Shape::bell, 12000, 8, 1.5, 12 }, { Shape::bell, 18000, -6, 2, 12 }, { Shape::highShelf, 10000, 6, 0.71, 12 },
                               { Shape::lowShelf, 5000, -9, 1, 12 }, { Shape::highShelf, 15000, 4, 0.71, 6 }, { Shape::notch, 15000, 0, 3, 12 },
                               { Shape::tiltShelf, 4000, 6, 0.71, 12 } };
        double before = 0, after = 0;
        for (const auto& c : cases)
        {
            BandSpec s; s.active = true; s.shape = c.s; s.freq = c.f; s.gainDb = c.g; s.q = c.q; s.slope = c.sl;
            Design ref; design (s, 192000, ref, false);
            for (double fs : { 44100.0, 48000.0, 96000.0 })
            {
                Design d, l; design (s, fs, d, false); designBilinear (s, fs, l);
                for (int i = 0; i < 400; ++i)
                {
                    const double f = 20 * std::pow (1000.0, i / 399.0);
                    const double r = magnitudeDb (ref, f, 192000, false);
                    if (c.s == Shape::notch && r < -3) continue;
                    after = std::max (after, std::abs (magnitudeDb (d, f, fs, false) - r));
                    before = std::max (before, std::abs (magnitudeDb (l, f, fs, false) - r));
                }
            }
        }
        std::printf ("   7 réglages aigus : avant %.2f dB → après %.2f dB [seuil 0.6]\n", before, after);
        CHECK (after <= 0.6, "courbe différente selon la fréquence d'échantillonnage (%.2f dB)", after);
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
                                design (b, fs, de, false);
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

    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
