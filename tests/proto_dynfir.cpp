// Prototype chiffré (PAS intégré au plugin) : EQ dynamique dans les modes FIR (« Linear phase » /
// « Natural phase ») aussi précise dans le temps qu'en « Zero latency ».
//   g++ -O2 -std=c++17 -I Source tests/proto_dynfir.cpp -o proto && ./proto
//
// Aujourd'hui : une bande dynamique fait reconcevoir le FIR complet toutes les 25 ms environ, puis
// fondu de 4 blocs (21 ms). Idée évaluée ici : un FIR « statique » S (toutes les bandes à leur gain de
// base) et, par bande dynamique k, un FIR d'écart D_k = F_k − S (F_k = S avec la bande k à pleine plage),
// de même longueur et même latence :
//     sortie(t) = (S*x)(t) + Σ_k α_k(t) · (D_k*x)(t)
// α_k(t) est calculé à CHAQUE échantillon à partir du gain du détecteur, de sorte que le gain au centre
// de la bande soit exactement celui demandé. Variante à plusieurs « ancres » : la plage est coupée en
// J tronçons égaux (en dB) et on interpole entre les deux FIR qui encadrent le gain du moment.
//
// Ce banc chiffre :
//  A. l'erreur de courbe entre la réponse obtenue pour un gain intermédiaire et la vraie cloche /
//     étagère à ce gain (1, 2, 3 ancres ; phase linéaire et phase naturelle) ;
//  B. l'erreur des termes croisés quand deux bandes dynamiques agissent ensemble ;
//  C. le coût : convolutions en plus, temps mesuré d'une convolution par partitions (256 à 48 kHz) ;
//  D. la précision temporelle, sur de vrais FIR : résidu par rapport à l'idéal (à chaque échantillon,
//     le FIR exact du gain du moment), pour le mélange proposé et pour la méthode actuelle ;
//  E. l'autre architecture pour « Natural phase » : bandes dynamiques confiées au moteur zéro latence.
#include "EqEngine.h"
#include "FirDesign.h"
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

using namespace audiolab::eq;
using fir::cd;

static const double FS = 48000.0;
static double dB (double x) { return 20.0 * std::log10 (std::max (1e-300, x)); }

// ── Famille de courbes d'une bande (prototype analogique = cible des FIR) ──
struct Fam { Shape shape; int slope; double f0, q; };

static cd bandAt (const Fam& fm, double gainDb, double f, bool linearPhase)
{
    BandSpec b; b.active = true; b.shape = fm.shape; b.slope = fm.slope; b.freq = fm.f0; b.q = fm.q; b.gainDb = gainDb;
    Design d; design (b, FS, d, true);
    const cd h = response (d, f, FS, true);
    return linearPhase ? cd (std::abs (h), 0.0) : h;
}

// fréquence où le gain de la bande est « le gain demandé » : centre d'une cloche, plateau d'une étagère
static double centreOf (const Fam& fm)
{
    if (fm.shape == Shape::lowShelf) return 0.01;
    if (fm.shape == Shape::highShelf) return 1.0e7;
    return fm.f0;
}

static std::vector<double> logGrid (double lo, double hi, int n)
{
    std::vector<double> f ((size_t) n);
    for (int i = 0; i < n; ++i) f[(size_t) i] = lo * std::pow (hi / lo, i / (n - 1.0));
    return f;
}

struct Err { double magDb = 0, phaseDeg = 0, atGain = 0; };

// Erreur du mélange à J ancres (J tronçons égaux entre le gain de base et base + plage)
static Err blendError (const Fam& fm, double baseDb, double rangeDb, int J, bool linearPhase)
{
    static const std::vector<double> grid = logGrid (10.0, 24000.0, 500);
    const double fc = centreOf (fm);
    Err e;
    for (int seg = 0; seg < J; ++seg)
    {
        const double g0 = baseDb + rangeDb * seg / J, g1 = baseDb + rangeDb * (seg + 1) / J;
        std::vector<cd> A (grid.size()), B (grid.size());
        for (size_t i = 0; i < grid.size(); ++i) { A[i] = bandAt (fm, g0, grid[i], linearPhase); B[i] = bandAt (fm, g1, grid[i], linearPhase); }
        const double c0 = std::abs (bandAt (fm, g0, fc, linearPhase)), c1 = std::abs (bandAt (fm, g1, fc, linearPhase));
        const int steps = std::max (4, (int) std::ceil (std::abs (g1 - g0) / 0.25));
        for (int s = 1; s < steps; ++s)
        {
            const double g = g0 + (g1 - g0) * s / steps;
            const double alpha = (std::abs (bandAt (fm, g, fc, linearPhase)) - c0) / (c1 - c0);   // gain exact au centre
            for (size_t i = 0; i < grid.size(); ++i)
            {
                const cd h = A[i] + alpha * (B[i] - A[i]), t = bandAt (fm, g, grid[i], linearPhase);
                const double m = std::abs (dB (std::abs (h) / std::abs (t)));
                if (m > e.magDb) { e.magDb = m; e.atGain = g; }
                e.phaseDeg = std::max (e.phaseDeg, std::abs (std::arg (h / t)) * 180.0 / pi);
            }
        }
    }
    return e;
}

// Meilleure base possible à J FIR (composantes principales de la famille de courbes sur la plage) :
// sortie = S + Σ_j a_j(g)·U_j, les a_j(g) étant tabulés. Borne basse de l'erreur à nombre de FIR donné.
static double pcaError (const Fam& fm, double rangeDb, int J)
{
    static const std::vector<double> grid = logGrid (10.0, 24000.0, 300);
    const int ng = 49, nf = (int) grid.size();
    std::vector<std::vector<double>> M ((size_t) ng, std::vector<double> ((size_t) nf)), T = M;
    for (int i = 0; i < ng; ++i)
        for (int j = 0; j < nf; ++j)
            T[(size_t) i][(size_t) j] = M[(size_t) i][(size_t) j] = std::abs (bandAt (fm, rangeDb * i / (ng - 1.0), grid[(size_t) j], true)) - 1.0;
    std::vector<std::vector<double>> approx ((size_t) ng, std::vector<double> ((size_t) nf, 0.0));
    std::mt19937 rng (1); std::uniform_real_distribution<double> U (-1, 1);
    for (int r = 0; r < J; ++r)
    {
        std::vector<double> v ((size_t) nf), u ((size_t) ng);
        for (auto& x : v) x = U (rng);
        for (int it = 0; it < 300; ++it)
        {
            for (int i = 0; i < ng; ++i) { double s = 0; for (int j = 0; j < nf; ++j) s += M[(size_t) i][(size_t) j] * v[(size_t) j]; u[(size_t) i] = s; }
            double nrm = 0;
            for (int j = 0; j < nf; ++j) { double s = 0; for (int i = 0; i < ng; ++i) s += M[(size_t) i][(size_t) j] * u[(size_t) i]; v[(size_t) j] = s; nrm += s * s; }
            nrm = std::sqrt (std::max (1e-300, nrm));
            for (auto& x : v) x /= nrm;
        }
        for (int i = 0; i < ng; ++i) { double s = 0; for (int j = 0; j < nf; ++j) s += M[(size_t) i][(size_t) j] * v[(size_t) j]; u[(size_t) i] = s; }
        for (int i = 0; i < ng; ++i)
            for (int j = 0; j < nf; ++j)
            {
                approx[(size_t) i][(size_t) j] += u[(size_t) i] * v[(size_t) j];
                M[(size_t) i][(size_t) j] -= u[(size_t) i] * v[(size_t) j];
            }
    }
    double e = 0;
    for (int i = 0; i < ng; ++i)
        for (int j = 0; j < nf; ++j)
            e = std::max (e, std::abs (dB (std::abs (1.0 + approx[(size_t) i][(size_t) j]) / (1.0 + T[(size_t) i][(size_t) j]))));
    return e;
}

// ── Convolution directe (référence) ─────────────────────────────────────
static void convolve (const std::vector<double>& h, const std::vector<double>& x, std::vector<double>& y)
{
    const int N = (int) h.size(), n = (int) x.size();
    y.assign ((size_t) n, 0.0);
    for (int i = 0; i < n; ++i)
    {
        double s = 0;
        const int kmax = std::min (N - 1, i);
        for (int k = 0; k <= kmax; ++k) s += h[(size_t) k] * x[(size_t) (i - k)];
        y[(size_t) i] = s;
    }
}

static double convAt (const std::vector<double>& h, const std::vector<double>& x, int i)
{
    double s = 0;
    const int kmax = std::min ((int) h.size() - 1, i);
    for (int k = 0; k <= kmax; ++k) s += h[(size_t) k] * x[(size_t) (i - k)];
    return s;
}

// ── Convolution par partitions uniformes (comme LinearPhase.h), plusieurs FIR sur le MÊME historique ──
// Mono, sans JUCE. FFT : fir::fft (complexe double), plus lente que celle du plugin ; les produits
// spectraux sont en float entrelacé, comme dans le plugin : c'est eux qui coûtent, et ils sont représentatifs.
struct Upols
{
    int P = 256, bins = 0, K = 0, head = 0;
    std::vector<float> fdl;                        // spectres des blocs d'entrée passés (K × bins)
    std::vector<std::vector<float>> sets;          // un jeu de spectres de partitions par FIR
    std::vector<float> lastIn, acc;
    std::vector<cd> work;
    double tFft = 0, tMac = 0;                     // temps cumulés (s)
    long blocks = 0;

    void prepare (int partition, int firLength)
    {
        P = partition; bins = 2 * (P + 1); K = (firLength + P - 1) / P;
        fdl.assign ((size_t) K * (size_t) bins, 0.0f);
        lastIn.assign ((size_t) P, 0.0f); acc.assign ((size_t) bins, 0.0f); work.resize ((size_t) 2 * P);
        head = 0; sets.clear(); tFft = tMac = 0; blocks = 0;
    }
    void addFir (const std::vector<double>& h)
    {
        std::vector<float> H ((size_t) K * (size_t) bins, 0.0f);
        for (int k = 0; k < K; ++k)
        {
            for (int j = 0; j < 2 * P; ++j) { const int n = k * P + j; work[(size_t) j] = (j < P && n < (int) h.size()) ? h[(size_t) n] : 0.0; }
            fir::fft (work.data(), 2 * P, false);
            for (int b = 0; b <= P; ++b) { H[(size_t) k * (size_t) bins + 2 * b] = (float) work[(size_t) b].real(); H[(size_t) k * (size_t) bins + 2 * b + 1] = (float) work[(size_t) b].imag(); }
        }
        sets.push_back (std::move (H));
    }
    // un bloc de P échantillons ; out[f] : sortie du FIR f (P échantillons chacun)
    void block (const float* in, std::vector<std::vector<float>>& out)
    {
        using clk = std::chrono::steady_clock;
        auto t0 = clk::now();
        head = (head + 1) % K;
        for (int j = 0; j < P; ++j) { work[(size_t) j] = lastIn[(size_t) j]; work[(size_t) (P + j)] = in[j]; lastIn[(size_t) j] = in[j]; }
        fir::fft (work.data(), 2 * P, false);
        float* slot = fdl.data() + (size_t) head * (size_t) bins;
        for (int b = 0; b <= P; ++b) { slot[2 * b] = (float) work[(size_t) b].real(); slot[2 * b + 1] = (float) work[(size_t) b].imag(); }
        auto t1 = clk::now();
        tFft += std::chrono::duration<double> (t1 - t0).count();
        out.resize (sets.size());
        for (size_t f = 0; f < sets.size(); ++f)
        {
            t0 = clk::now();
            std::fill (acc.begin(), acc.end(), 0.0f);
            for (int k = 0; k < K; ++k)
            {
                const float* x = fdl.data() + (size_t) ((head - k + K) % K) * (size_t) bins;
                const float* h = sets[f].data() + (size_t) k * (size_t) bins;
                float* a = acc.data();
                for (int b = 0; b < bins; b += 2)
                {
                    const float xr = x[b], xi = x[b + 1], hr = h[b], hi = h[b + 1];
                    a[b] += xr * hr - xi * hi;
                    a[b + 1] += xr * hi + xi * hr;
                }
            }
            t1 = clk::now();
            tMac += std::chrono::duration<double> (t1 - t0).count();
            for (int b = 0; b <= P; ++b) work[(size_t) b] = cd (acc[(size_t) (2 * b)], acc[(size_t) (2 * b + 1)]);
            for (int b = 1; b < P; ++b) work[(size_t) (2 * P - b)] = std::conj (work[(size_t) b]);
            fir::fft (work.data(), 2 * P, true);
            out[f].resize ((size_t) P);
            for (int j = 0; j < P; ++j) out[f][(size_t) j] = (float) (work[(size_t) (P + j)].real() / (2 * P));
            tFft += std::chrono::duration<double> (clk::now() - t1).count();
        }
        ++blocks;
    }
};

// FIR complet (voie gauche) pour un jeu de bandes
static std::vector<double> makeFir (const std::array<BandSpec, maxBands>& bands, int N, fir::Mode mode, fir::Work& w)
{
    fir::Request rq; rq.bands = bands; rq.N = N; rq.mode = mode; rq.fs = FS; rq.delay = fir::naturalDelay (FS);
    fir::Result r; fir::design (rq, r, w);
    return r.h[0];
}

// Détecteur et courbe de la dynamique : mêmes formules que le moteur (EqEngine.h)
static double dynCurveDb (double levelDb, double thr, double range)
{
    const double over = levelDb - thr, W = 6.0;
    double o;
    if (over <= -W / 2) o = 0;
    else if (over < W / 2) o = (over + W / 2) * (over + W / 2) / (2 * W);
    else o = over;
    return range < 0 ? -std::min (o, -range) : std::min (o, range);
}

int main()
{
    std::printf ("Prototype : EQ dynamique dans les modes FIR par mélange de FIR à chaque échantillon\n\n");

    // ── A. Erreur de courbe ──────────────────────────────────────────────
    std::printf ("A. Erreur de courbe max (dB, 10 Hz – 24 kHz, tous les gains intermédiaires), gain de base 0 dB, gain exact au centre\n");
    const double ranges[] = { -18, -12, -6, -3, 3, 6, 12, 18 };
    {
        std::printf ("   A1. Phase linéaire, cloche. Chaque case : 1 ancre / 2 ancres / 3 ancres\n");
        std::printf ("        Q     ");
        for (double r : ranges) std::printf ("   plage %+3.0f dB    ", r);
        std::printf ("\n");
        for (double q : { 0.5, 1.0, 2.0, 4.0, 8.0 })
        {
            std::printf ("      %4.1f   ", q);
            for (double r : ranges)
            {
                const Fam fm { Shape::bell, 12, 1000.0, q };
                std::printf ("  %5.2f/%5.3f/%5.3f ", blendError (fm, 0, r, 1, true).magDb, blendError (fm, 0, r, 2, true).magDb, blendError (fm, 0, r, 3, true).magDb);
            }
            std::printf ("\n");
        }
        std::printf ("   A2. Phase linéaire, étagères (1 / 2 / 3 ancres)\n");
        struct S { const char* name; Fam fm; };
        const S shelves[] = { { "low 12 dB/oct Q 0.71", { Shape::lowShelf, 12, 200.0, 0.71 } }, { "low 12 dB/oct Q 2   ", { Shape::lowShelf, 12, 200.0, 2.0 } },
                              { "low 6 dB/oct        ", { Shape::lowShelf, 6, 200.0, 0.71 } }, { "high 12 dB/oct Q 0.71", { Shape::highShelf, 12, 5000.0, 0.71 } },
                              { "high 6 dB/oct       ", { Shape::highShelf, 6, 5000.0, 0.71 } } };
        for (const auto& s : shelves)
        {
            std::printf ("      %-21s", s.name);
            for (double r : { -12.0, -6.0, 6.0, 12.0 })
                std::printf ("  %+3.0f dB : %5.2f/%5.3f/%5.3f", r, blendError (s.fm, 0, r, 1, true).magDb, blendError (s.fm, 0, r, 2, true).magDb, blendError (s.fm, 0, r, 3, true).magDb);
            std::printf ("\n");
        }
        std::printf ("   A3. Phase naturelle (module ET phase analogiques), cloche : erreur de module / de phase, 1 puis 3 ancres\n");
        for (double q : { 1.0, 4.0 })
        {
            std::printf ("      Q %3.1f :", q);
            for (double r : { -12.0, -6.0, 6.0, 12.0 })
            {
                const Fam fm { Shape::bell, 12, 1000.0, q };
                const Err e1 = blendError (fm, 0, r, 1, false), e3 = blendError (fm, 0, r, 3, false);
                std::printf ("  %+3.0f dB : %4.2f dB %4.1f° → %5.3f dB %4.2f° ;", r, e1.magDb, e1.phaseDeg, e3.magDb, e3.phaseDeg);
            }
            std::printf ("\n");
        }
        std::printf ("   A4. Gain de base non nul (phase linéaire, cloche Q 2) : base +6 dB, plage −12 dB : %.2f / %.3f / %.3f dB ; base −6, plage +12 : %.2f / %.3f / %.3f dB\n",
                     blendError ({ Shape::bell, 12, 1000.0, 2.0 }, 6, -12, 1, true).magDb, blendError ({ Shape::bell, 12, 1000.0, 2.0 }, 6, -12, 2, true).magDb,
                     blendError ({ Shape::bell, 12, 1000.0, 2.0 }, 6, -12, 3, true).magDb, blendError ({ Shape::bell, 12, 1000.0, 2.0 }, -6, 12, 1, true).magDb,
                     blendError ({ Shape::bell, 12, 1000.0, 2.0 }, -6, 12, 2, true).magDb, blendError ({ Shape::bell, 12, 1000.0, 2.0 }, -6, 12, 3, true).magDb);
        std::printf ("   A5. Pour comparaison, la meilleure base possible à nombre de FIR donné (composantes principales, tous actifs en\n"
                     "       permanence, gain au centre non imposé), cloche Q 2 : 1 FIR / 2 FIR / 3 FIR\n      ");
        for (double r : { -18.0, -12.0, -6.0, 6.0, 12.0, 18.0 })
        {
            const Fam fm { Shape::bell, 12, 1000.0, 2.0 };
            std::printf ("  %+3.0f dB : %5.2f/%5.3f/%6.4f", r, pcaError (fm, r, 1), pcaError (fm, r, 2), pcaError (fm, r, 3));
        }
        std::printf ("\n");
    }

    // ── B. Termes croisés ────────────────────────────────────────────────
    // Vraie cible : produit B1(g1)·B2(g2). Mélange additif : S + α1·D1 + α2·D2 = B1·B2 − (B1−1)(B2−1) à
    // pleine plage (bandes de base à 0 dB) : il manque le terme croisé. Chaque bande seule est ici exacte.
    // Parades : (1) résoudre α1 et α2 ENSEMBLE pour que le gain soit exact aux deux centres (petit système
    // linéaire, sans FIR de plus) ; (2) ajouter le FIR croisé D12 = S·(B1−1)(B2−1), pondéré par α1·α2 :
    // exact pour deux bandes (2^K FIR pour K bandes qui se recouvrent toutes).
    std::printf ("\nB. Termes croisés : deux cloches dynamiques à pleine plage en même temps (phase linéaire), erreur max en dB\n");
    {
        const std::vector<double> grid = logGrid (10.0, 24000.0, 600);
        std::printf ("      écart      Q    plage    mélange additif              α résolus ensemble    + FIR croisé α1·α2\n");
        for (double ratio : { std::sqrt (2.0), 2.0, 4.0, 8.0 })
            for (double q : { 1.0, 4.0 })
                for (double r : { -12.0, -6.0, 6.0, 12.0 })
                {
                    const Fam f1 { Shape::bell, 12, 1000.0, q }, f2 { Shape::bell, 12, 1000.0 * ratio, q };
                    auto b1 = [&] (double f) { return std::abs (bandAt (f1, r, f, true)); };
                    auto b2 = [&] (double f) { return std::abs (bandAt (f2, r, f, true)); };
                    // α ensemble : (b1(fi) − 1)·α1 + (b2(fi) − 1)·α2 = b1(fi)·b2(fi) − 1 aux deux centres
                    const double m11 = b1 (f1.f0) - 1, m12 = b2 (f1.f0) - 1, m21 = b1 (f2.f0) - 1, m22 = b2 (f2.f0) - 1;
                    const double t1 = b1 (f1.f0) * b2 (f1.f0) - 1, t2 = b1 (f2.f0) * b2 (f2.f0) - 1, det = m11 * m22 - m12 * m21;
                    const double a1 = (t1 * m22 - t2 * m12) / det, a2 = (m11 * t2 - m21 * t1) / det;
                    double e = 0, ej = 0; bool neg = false, negJ = false;
                    for (double f : grid)
                    {
                        const double x1 = b1 (f), x2 = b2 (f), t = x1 * x2;
                        const double h = 1.0 + (x1 - 1.0) + (x2 - 1.0), hj = 1.0 + a1 * (x1 - 1.0) + a2 * (x2 - 1.0);
                        if (h <= 0) neg = true;
                        if (hj <= 0) negJ = true;
                        e = std::max (e, std::abs (dB (std::abs (h) / t)));
                        ej = std::max (ej, std::abs (dB (std::abs (hj) / t)));
                    }
                    std::printf ("      %-10s %4.1f   %+3.0f dB   %6.2f dB%-19s   %5.2f dB%-10s   0 (exact)\n",
                                 ratio < 1.5 ? "½ octave" : ratio < 3 ? "1 octave" : ratio < 6 ? "2 octaves" : "3 octaves", q, r, e, neg ? " (signe inversé !)" : "", ej, negJ ? " (signe !)" : "");
                }
    }

    // ── C. Coût ──────────────────────────────────────────────────────────
    std::printf ("\nC. Coût, convolution par partitions de 256 à 48 kHz (un bloc = 5,33 ms), par voie\n");
    {
        fir::Work w;
        for (int N : { 8192, 32768 })
        {
            std::array<BandSpec, maxBands> bands;
            bands[0].active = true; bands[0].shape = Shape::bell; bands[0].freq = 1000; bands[0].q = 2; bands[0].gainDb = -12;
            const auto h = makeFir (bands, N, fir::Mode::linear, w);
            Upols u; u.prepare (256, N);
            const int nF = 4;
            for (int i = 0; i < nF; ++i) u.addFir (h);
            std::vector<float> in (256); std::vector<std::vector<float>> out;
            std::mt19937 rng (2); std::uniform_real_distribution<float> U (-1, 1);
            double best = 1e9;
            for (int rep = 0; rep < 5; ++rep)
            {
                u.tMac = u.tFft = 0; u.blocks = 0;
                for (int b = 0; b < 400; ++b) { for (auto& x : in) x = U (rng); u.block (in.data(), out); }
                best = std::min (best, u.tMac / (u.blocks * nF));
            }
            const double macUs = best * 1e6, blockUs = 256.0 / FS * 1e6;
            std::printf ("   FIR %5d points (%3d partitions) : produits spectraux d'un FIR %6.1f µs par bloc = %.2f %% d'un cœur par voie ;\n"
                         "       + 1 FFT inverse de 512 points par FIR et par voie (plugin : quelques µs ; ici FFT de test : %.0f µs)\n",
                         N, u.K, macUs, 100.0 * macUs / blockUs, 1e6 * u.tFft / (u.blocks * (nF + 1)));
            std::printf ("       → par bande dynamique, en stéréo : 1 ancre %.2f %% ; 2 ou 3 ancres %.2f %% en régime courant (2 FIR actifs), %.2f %% / %.2f %% au pire\n",
                         2 * 100.0 * macUs / blockUs, 2 * 2 * 100.0 * macUs / blockUs, 2 * 2 * 100.0 * macUs / blockUs, 3 * 2 * 100.0 * macUs / blockUs);
        }
        // Raccourcir les FIR d'écart : D_k = S·(B_kF/B_k0 − 1) n'a besoin que de la longueur qu'exige la bande k
        // elle-même. On le fenêtre autour du centre (N/2) : seules les partitions du milieu sont calculées.
        // Cas sévère pour S : coupe-bas 30 Hz 48 dB/oct + cloche 50 Hz Q 8 −12 dB, FIR de 32768 points.
        std::printf ("   FIR d'écart raccourci (fenêtré autour du centre), S de 32768 points très exigeant dans le grave :\n"
                     "   erreur max de la courbe à pleine plage (−12 dB), 10 Hz – 20 kHz, selon la longueur gardée pour D_k\n");
        struct C { const char* name; double f, q; };
        for (const C& c : { C { "cloche dynamique 4 kHz Q 2", 4000, 2 }, C { "cloche dynamique 1 kHz Q 2", 1000, 2 }, C { "cloche dynamique 250 Hz Q 4", 250, 4 }, C { "cloche dynamique 80 Hz Q 4", 80, 4 } })
        {
            const int N = 32768, L = 1 << 18;
            std::array<BandSpec, maxBands> a;
            a[0].active = true; a[0].shape = Shape::lowCut; a[0].freq = 30; a[0].slope = 48; a[0].q = 0.71;
            a[1].active = true; a[1].shape = Shape::bell; a[1].freq = 50; a[1].q = 8; a[1].gainDb = -12;
            a[2].active = true; a[2].shape = Shape::bell; a[2].freq = c.f; a[2].q = c.q; a[2].gainDb = 0;
            auto b = a; b[2].gainDb = -12;
            const auto hs = makeFir (a, N, fir::Mode::linear, w), hf = makeFir (b, N, fir::Mode::linear, w);
            std::vector<cd> T ((size_t) L, cd (0, 0)), X ((size_t) L);
            for (int n = 0; n < N; ++n) T[(size_t) n] = hf[(size_t) n];
            fir::fft (T.data(), L, false);
            BandSpec need = b[2];
            std::printf ("      %-27s (longueur voulue par la bande seule : %5.0f) :", c.name, fir::requiredLength (need, FS, fir::Mode::linear));
            for (int Nd : { 2048, 4096, 8192, 16384 })
            {
                std::fill (X.begin(), X.end(), cd (0, 0));
                const int tail = Nd / 8;
                for (int n = 0; n < N; ++n)
                {
                    const int d = std::abs (n - N / 2);
                    double win = d >= Nd / 2 ? 0.0 : d <= Nd / 2 - tail ? 1.0 : fir::halfBH ((double) (Nd / 2 - d) / tail);
                    X[(size_t) n] = hs[(size_t) n] + win * (hf[(size_t) n] - hs[(size_t) n]);
                }
                fir::fft (X.data(), L, false);
                double e = 0;
                for (int k = (int) (10.0 * L / FS); k < (int) (20000.0 * L / FS); ++k)
                    if (std::abs (T[(size_t) k]) > 1e-3) e = std::max (e, std::abs (dB (std::abs (X[(size_t) k]) / std::abs (T[(size_t) k]))));
                std::printf ("  %5d : %5.3f dB", Nd, e);
            }
            std::printf ("\n");
        }
    }

    // ── D. Précision temporelle, sur de vrais FIR ────────────────────────
    // Phase linéaire, FIR de 4096 points (latence 2048). Bandes : coupe-bas 40 Hz, high shelf 8 kHz +4 dB,
    // cloche 1 kHz Q 2 DYNAMIQUE (base 0 dB, plage −12 dB, seuil −30 dB). Signal : salves de 997 Hz + bruit.
    // Idéal : à chaque échantillon de sortie, le FIR exact du gain que donne le détecteur à cet instant
    // (détecteur sur l'entrée retardée de la latence : gain et son parfaitement alignés).
    std::printf ("\nD. Précision temporelle sur de vrais FIR (phase linéaire, 4096 points) : résidu par rapport à l'idéal\n");
    {
        const int N = 4096, lat = N / 2, n = 60000;
        fir::Work w;
        const double range = -12.0, thr = -30.0;
        auto bandsFor = [] (double g)
        {
            std::array<BandSpec, maxBands> b;
            b[0].active = true; b[0].shape = Shape::lowCut; b[0].freq = 40; b[0].slope = 12; b[0].q = 0.71;
            b[1].active = true; b[1].shape = Shape::highShelf; b[1].freq = 8000; b[1].gainDb = 4; b[1].q = 0.71;
            b[2].active = true; b[2].shape = Shape::bell; b[2].freq = 1000; b[2].q = 2; b[2].gainDb = g;
            return b;
        };
        // banque de FIR exacts tous les 0,05 dB (l'idéal interpole entre deux voisins : erreur négligeable)
        const double bankStep = 0.05; const int nb = (int) std::lround (-range / bankStep) + 1;
        std::vector<std::vector<double>> bank ((size_t) nb);
        for (int i = 0; i < nb; ++i) bank[(size_t) i] = makeFir (bandsFor (-i * bankStep), N, fir::Mode::linear, w);
        auto firAt = [&] (double g, std::vector<double>& h)
        {
            const double p = std::clamp (-g / bankStep, 0.0, nb - 1.0);
            const int i0 = std::min ((int) p, nb - 2); const double t = p - i0;
            h.resize ((size_t) N);
            for (int k = 0; k < N; ++k) h[(size_t) k] = bank[(size_t) i0][(size_t) k] * (1 - t) + bank[(size_t) i0 + 1][(size_t) k] * t;
        };

        for (double attackMs : { 0.5, 10.0 })
        {
            const double releaseMs = 100.0;
            // signal
            std::vector<double> x ((size_t) n);
            std::mt19937 rng (4); std::uniform_real_distribution<double> U (-1, 1);
            for (int i = 0; i < n; ++i)
            {
                const double ph = std::fmod (i / FS, 0.25);                       // une salve toutes les 250 ms
                const double env = ph < 0.08 ? std::min (1.0, ph / 0.0005) : std::exp (-(ph - 0.08) / 0.01);
                x[(size_t) i] = 0.4 * env * std::sin (2 * pi * 997.0 * i / FS) + 0.003 * U (rng);
            }
            // gain du détecteur, aligné sur la sortie : g[i] = gain pour l'échantillon de sortie i (entrée i − lat)
            std::vector<double> g ((size_t) n, 0.0), gLook ((size_t) n, 0.0);
            {
                SectionRT det; toRT (listenSection (Shape::bell, 1000, 2, FS), det);
                const double aA = std::exp (-1.0 / (attackMs * 0.001 * FS)), aR = std::exp (-1.0 / (releaseMs * 0.001 * FS));
                double st[2] = { 0, 0 }, env = 0;
                std::vector<double> raw ((size_t) n);
                for (int i = 0; i < n; ++i)
                {
                    const double d = std::abs (tick (det, st, x[(size_t) i]));
                    env = d > env ? aA * env + (1 - aA) * d : aR * env + (1 - aR) * d;
                    raw[(size_t) i] = dynCurveDb (20 * std::log10 (std::max (1e-9, env)), thr, range);
                }
                const int ahead = (int) (0.03 * FS);   // méthode actuelle : détecteur en avance de ≈ 30 ms (temps de conception)
                for (int i = 0; i < n; ++i)
                {
                    g[(size_t) i] = i >= lat ? raw[(size_t) (i - lat)] : 0.0;
                    gLook[(size_t) i] = i - lat + ahead >= 0 && i - lat + ahead < n ? raw[(size_t) (i - lat + ahead)] : 0.0;
                }
            }
            // idéal
            std::vector<double> ideal ((size_t) n), h, h2;
            for (int i = 0; i < n; ++i) { firAt (g[(size_t) i], h); ideal[(size_t) i] = convAt (h, x, i); }
            auto report = [&] (const char* name, const std::vector<double>& y)
            {
                double rr = 0, ss = 0, pk = 0, spk = 0;
                for (int i = lat + 4096; i < n; ++i)
                {
                    const double e = y[(size_t) i] - ideal[(size_t) i];
                    rr += e * e; ss += ideal[(size_t) i] * ideal[(size_t) i];
                    pk = std::max (pk, std::abs (e)); spk = std::max (spk, std::abs (ideal[(size_t) i]));
                }
                std::printf ("      %-58s : résidu efficace %6.1f dB, crête %6.1f dB\n", name, 0.5 * dB (rr / ss), dB (pk / spk));
            };
            std::printf ("   attaque %.1f ms, retour %.0f ms :\n", attackMs, releaseMs);

            // méthode actuelle : FIR reconçu toutes les 25 ms d'après le gain du moment (détecteur en avance
            // de 30 ms), installé au bloc suivant, fondu de 4 blocs de 256
            {
                std::vector<double> y ((size_t) n);
                std::vector<double> cur, old; firAt (0.0, cur); old = cur;
                int xf = 0; const int xfLen = 4 * 256, every = (int) (0.025 * FS);
                double pendingG = 0; bool pending = false;
                for (int i = 0; i < n; ++i)
                {
                    if (i % every == 0) { pendingG = gLook[(size_t) i]; pending = true; }
                    if (i % 256 == 0 && pending && xf == 0) { old = cur; firAt (pendingG, cur); xf = xfLen; pending = false; }
                    if (xf > 0)
                    {
                        const double t = (double) (xfLen - xf + 1) / xfLen;
                        y[(size_t) i] = convAt (old, x, i) * (1 - t) + convAt (cur, x, i) * t;
                        --xf;
                    }
                    else y[(size_t) i] = convAt (cur, x, i);
                }
                report ("méthode actuelle (FIR reconçu toutes les 25 ms, fondu 21 ms)", y);
            }
            // mélange à J ancres, α à chaque échantillon
            for (int J : { 1, 2, 3 })
            {
                std::vector<std::vector<double>> conv ((size_t) J + 1);
                std::vector<double> centre ((size_t) J + 1);
                for (int j = 0; j <= J; ++j)
                {
                    firAt (range * j / J, h);
                    convolve (h, x, conv[(size_t) j]);
                    centre[(size_t) j] = std::pow (10.0, (range * j / J) / 20.0);   // gain de la cloche à son centre
                }
                std::vector<double> y ((size_t) n);
                for (int i = 0; i < n; ++i)
                {
                    const double gi = g[(size_t) i];
                    const int seg = std::min (J - 1, (int) (gi / range * J));
                    const double c = std::pow (10.0, gi / 20.0);
                    const double a = (c - centre[(size_t) seg]) / (centre[(size_t) seg + 1] - centre[(size_t) seg]);
                    y[(size_t) i] = conv[(size_t) seg][(size_t) i] + a * (conv[(size_t) seg + 1][(size_t) i] - conv[(size_t) seg][(size_t) i]);
                }
                char name[96]; std::snprintf (name, sizeof name, "mélange de FIR, %d ancre%s, α à chaque échantillon", J, J > 1 ? "s" : "");
                report (name, y);
            }
            // le même mélange avec α seulement une fois par bloc de 256 (mise à jour du spectre à chaque partition)
            {
                std::vector<double> s0, s1; firAt (0.0, h); convolve (h, x, s0); firAt (range, h); convolve (h, x, s1);
                std::vector<double> y ((size_t) n);
                const double cF = std::pow (10.0, range / 20.0);
                double a = 0;
                for (int i = 0; i < n; ++i)
                {
                    if (i % 256 == 0) a = (std::pow (10.0, g[(size_t) i] / 20.0) - 1.0) / (cF - 1.0);
                    y[(size_t) i] = s0[(size_t) i] + a * (s1[(size_t) i] - s0[(size_t) i]);
                }
                report ("mélange 1 ancre, α une fois par partition de 256 (5,3 ms)", y);
            }
        }

        // suivi du gain à l'échantillon près : sinus stable au centre de la bande, gain imposé de l'extérieur
        // (side-chain) ; la sortie doit valoir gain(t) × entrée(t − latence), à chaque échantillon
        {
            std::vector<double> x ((size_t) 20000), s0, s1, hS, hF;
            for (size_t i = 0; i < x.size(); ++i) x[i] = 0.5 * std::sin (2 * pi * 1000.0 * i / FS);
            firAt (0.0, hS); convolve (hS, x, s0); firAt (range, hF); convolve (hF, x, s1);
            // gains réels des deux FIR à 1 kHz (réels positifs : phase linéaire)
            const double cS = fir::responseAt (hS, lat, FS, 1000.0).real(), cF = fir::responseAt (hF, lat, FS, 1000.0).real();
            double worst = 0;
            for (int i = 8192; i < 20000; ++i)
            {
                const double gi = range * 0.5 * (1.0 - std::cos (2 * pi * (i - 8192) / 96.0));   // gain qui va de 0 à −12 dB en 1 ms
                const double c = cS * std::pow (10.0, gi / 20.0), a = (c - cS) / (cF - cS);
                const double y = s0[(size_t) i] + a * (s1[(size_t) i] - s0[(size_t) i]);
                worst = std::max (worst, std::abs (y - c * x[(size_t) (i - lat)]));
            }
            std::printf ("   suivi du gain : sinus 1 kHz stable, gain imposé oscillant de 0 à −12 dB en 1 ms : écart max entre la sortie et\n"
                         "      gain(t) × entrée(t − latence) = %.1e (%.0f dB) → le gain suit le détecteur à l'échantillon près\n", worst, dB (worst / 0.5));
        }

        // contrôle : la convolution par partitions avec historique commun donne bien S*x et D*x
        {
            Upols u; u.prepare (256, N);
            std::vector<double> hS, hF, hD ((size_t) N); firAt (0.0, hS); firAt (range, hF);
            for (int k = 0; k < N; ++k) hD[(size_t) k] = hF[(size_t) k] - hS[(size_t) k];
            u.addFir (hS); u.addFir (hD);
            std::vector<double> x ((size_t) 16384), yS, yD;
            std::mt19937 rng (9); std::uniform_real_distribution<double> U (-1, 1);
            for (auto& v : x) v = U (rng);
            convolve (hS, x, yS); convolve (hD, x, yD);
            std::vector<float> in (256); std::vector<std::vector<float>> out; double e = 0;
            for (int b = 0; b < 64; ++b)
            {
                for (int j = 0; j < 256; ++j) in[(size_t) j] = (float) x[(size_t) (b * 256 + j)];
                u.block (in.data(), out);
                for (int j = 0; j < 256; ++j)
                {
                    e = std::max (e, std::abs (out[0][(size_t) j] - yS[(size_t) (b * 256 + j)]));
                    e = std::max (e, std::abs (out[1][(size_t) j] - yD[(size_t) (b * 256 + j)]));
                }
            }
            std::printf ("   contrôle : convolution par partitions (S et D sur le même historique d'entrée) = convolution directe à %.1e près (float)\n", e);
        }
    }

    // ── E. Natural phase : bandes dynamiques par le moteur zéro latence ───
    // En Natural phase la cible est le filtre analogique, phase comprise : c'est ce que fait déjà le
    // moteur IIR « matched ». Bandes dynamiques retirées du FIR et confiées au moteur, placé après le FIR.
    // Écart = celui de la conception matched par rapport à l'analogique (module et phase), 20 Hz – 20 kHz.
    std::printf ("\nE. Natural phase : bande dynamique faite par le moteur zéro latence (IIR matched) au lieu du FIR\n"
                 "   écart à la cible analogique, 20 Hz – 20 kHz (cloche ±12 dB) : module max / phase max\n");
    for (double fs : { 44100.0, 48000.0, 96000.0 })
    {
        std::printf ("   %.1f kHz :", fs / 1000);
        for (double f0 : { 100.0, 1000.0, 5000.0, 10000.0, 16000.0 })
        {
            double em = 0, ep = 0;
            for (double q : { 1.0, 4.0 })
                for (double g : { -12.0, 12.0 })
                {
                    BandSpec b; b.active = true; b.shape = Shape::bell; b.freq = f0; b.q = q; b.gainDb = g;
                    Design dm, da; design (b, fs, dm, false); design (b, fs, da, true);
                    for (int i = 0; i < 400; ++i)
                    {
                        const double f = 20.0 * std::pow (1000.0, i / 399.0);
                        const cd hm = response (dm, f, fs, false), ha = response (da, f, fs, true);
                        em = std::max (em, std::abs (dB (std::abs (hm) / std::abs (ha))));
                        ep = std::max (ep, std::abs (std::arg (hm / ha)) * 180.0 / pi);
                    }
                }
            std::printf ("  %5.0f Hz : %4.2f dB %4.1f° ;", f0, em, ep);
        }
        std::printf ("\n");
    }
    return 0;
}
