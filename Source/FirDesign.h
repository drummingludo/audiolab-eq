// Audio Lab EQ — conception des FIR des modes « Linear phase » et « Natural phase » (sans JUCE).
//
// Linear phase : FIR symétrique (aucune rotation de phase), cible = module du prototype analogique.
//   - échantillonnage dense de la cible (2N points) puis fenêtre « plate à bords Blackman-Harris »
//     (70 % du FIR intact, 15 % de fondu de chaque côté) : la fenêtre de Blackman-Harris de la 1.x
//     atténuait déjà le milieu du FIR et lissait la courbe (cloche 50 Hz Q 8 : −5 dB au lieu de −12 à 8192 points) ;
//   - notch : la cible |H| a une pointe en V au centre, qu'aucun FIR de longueur raisonnable ne suit ;
//     on corrige le FIR pour qu'il passe exactement par la cible au centre de chaque notch (creux garanti).
// Natural phase : FIR qui reproduit module ET phase du prototype analogique (comme un égaliseur
//   analogique idéal), avec une petite latence fixe D : la phase est ramenée doucement à zéro entre
//   20 kHz et Nyquist (inaudible), ce qui rend la réponse courte avant l'instant D.
// Placement Stéréo / G / D / Mid / Side : chaque bande est une matrice 2×2 ; le produit, dans l'ordre
// des bandes (comme la chaîne zéro latence), donne 4 FIR : L' = A·L + B·R ; R' = C·L + D·R.
#pragma once
#include "EqMath.h"
#include <string>
#include <vector>

namespace audiolab::eq::fir
{
using cd = std::complex<double>;

// FFT complexe radix 2 en place (double). inverse : sans normalisation.
inline void fft (cd* x, int n, bool inverse)
{
    for (int i = 1, j = 0; i < n; ++i)
    {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap (x[i], x[j]);
    }
    for (int len = 2; len <= n; len <<= 1)
    {
        const double ang = 2.0 * pi / len * (inverse ? 1.0 : -1.0);
        const cd wl (std::cos (ang), std::sin (ang));
        const int half = len >> 1;
        // facteurs de rotation par récurrence, recalés tous les 64 pour la précision
        for (int i = 0; i < n; i += len)
        {
            cd w (1.0, 0.0);
            for (int j = 0; j < half; ++j)
            {
                if ((j & 63) == 0 && j > 0) w = cd (std::cos (ang * j), std::sin (ang * j));
                const cd u = x[i + j], v = x[i + j + half] * w;
                x[i + j] = u + v;
                x[i + j + half] = u - v;
                w *= wl;
            }
        }
    }
}

enum class Mode { linear = 0, natural = 1 };

// Résolutions (longueur du FIR) : Auto, Low, Medium, High, Max
enum class Res { automatic = 0, low, medium, high, max };
constexpr int numRes = 5;

inline int rateMult (double fs) { return fs <= 50000.0 ? 1 : fs <= 100000.0 ? 2 : 4; }
// longueurs à 44,1 / 48 kHz : 4096, 8192, 16384, 32768 (×2 à 88,2 / 96 kHz, ×4 au-delà)
inline int lengthFor (Res r, double fs)
{
    const int m = rateMult (fs);
    switch (r)
    {
        case Res::low:    return 4096 * m;
        case Res::medium: return 8192 * m;
        case Res::high:   return 16384 * m;
        case Res::max:
        case Res::automatic: break;
    }
    return 32768 * m;
}
inline int minLength (double fs) { return lengthFor (Res::low, fs); }
inline int maxLength (double fs) { return lengthFor (Res::max, fs); }

// Bloc de la convolution par partitions et petite latence du mode Natural phase
inline int partitionSize (double fs) { return 256 * rateMult (fs); }
inline int naturalDelay (double fs)  { return 128 * rateMult (fs); }

// Longueur demandée par une bande (mode Auto) : K fois la constante de temps de sa résonance
// la plus lente (bande la plus grave et la plus étroite). Calibré sur les cibles : cloche 50 Hz Q 8
// à ±1 dB, coupe-bas 30 Hz 48 dB/oct à ±1 dB à 20 Hz (48 kHz).
inline double requiredLength (const BandSpec& b, double fs, Mode mode, double extraGainDb = 0.0)
{
    if (! b.active) return 0;
    const double W = 2.0 * pi * std::clamp (b.freq, 5.0, 0.49 * fs) / fs;
    const double q = std::clamp (b.q, 0.025, 40.0);
    const double gmax = std::max (std::abs (b.gainDb), std::abs (b.gainDb + extraGainDb));
    const double A = std::pow (10.0, gmax / 40.0);
    double tau = 0, K = 7;
    switch (b.shape)
    {
        case Shape::bell:
            if (gmax < 0.05) return 0;
            tau = 2.0 * q * A / W; K = 7; break;
        case Shape::lowShelf: case Shape::highShelf: case Shape::tiltShelf:
            if (gmax < 0.05) return 0;
            tau = b.slope <= 6 ? A / W : 2.0 * std::max (q, 0.5) * std::sqrt (A) / W; K = 7; break;
        case Shape::lowCut: case Shape::highCut:
        {
            const int order = cutOrder (b);   // (3.0 : pente libre et Brickwall compris, voir EqMath.h)
            double qs[8]; bool first = false;
            const int nb = butterworthQs (order, qs, first);
            const double qm = nb > 0 ? qs[nb - 1] * q / 0.7071067811865476 : 0.5;
            tau = nb > 0 ? 2.0 * qm / W : 1.0 / W; K = 13; break;
        }
        case Shape::bandPass: tau = 2.0 * q / W * std::sqrt ((double) std::clamp (b.slope / 12, 1, 8)); K = 9; break;
        case Shape::notch:    tau = 2.0 * q / W; K = 14; break;
        // 3.0. Flat tilt : pas de résonance, mais la droite doit être tenue jusqu'à 20 Hz (mesuré : ±0,1 dB avec
        // 8192 points à 44,1 / 48 kHz quand l'aigu monte, 16384 quand c'est le grave). All pass : rien à tenir.
        case Shape::flatTilt: return gmax < 0.05 ? 0.0 : fs * (b.gainDb < 0 ? 0.30 : 0.15);
        case Shape::allPass:  break;
    }
    (void) mode;   // Natural phase : même longueur (la latence n'en dépend pas, seule la charge CPU)
    return K * tau;
}

// Bande dont le gain bouge avec la dynamique (plage utile non nulle)
inline bool isDynamicSpan (const BandSpec& b, double span) { return b.active && canBeDynamic (b.shape) && std::abs (span) > 1.0e-6; }

// Longueur choisie : réglage fixe, ou Auto d'après les bandes (puissance de 2 entre Low et Max)
inline int chooseLength (Res r, const std::array<BandSpec, maxBands>& bands, double fs, Mode mode,
                         const std::array<double, maxBands>* dynSpan = nullptr)
{
    if (r != Res::automatic) return lengthFor (r, fs);
    double need = 0;
    for (int i = 0; i < maxBands; ++i)
        need = std::max (need, requiredLength (bands[(size_t) i], fs, mode, dynSpan != nullptr ? (*dynSpan)[(size_t) i] : 0.0));
    int n = minLength (fs);
    while (n < maxLength (fs) && n < need) n *= 2;
    return n;
}

struct Request
{
    std::array<BandSpec, maxBands> bands;
    bool stereo = true;
    Mode mode = Mode::linear;
    int N = 8192;            // longueur du FIR
    int delay = 128;         // Natural phase : instant du départ de la réponse
    double fs = 48000.0;
};

struct Result
{
    int N = 0;
    bool matrix = false;      // B et C non nuls (bandes Mid / Side avec du stéréo)
    std::vector<double> h[4]; // A (L→L), B (R→L), C (L→R), D (R→R)
};

// Espace de travail réutilisé (fil de conception)
constexpr int maxCorr = 8;   // notch corrigés au plus (les 8 premiers)

struct Work
{
    std::vector<cd> M[4], buf;
    std::vector<double> basis[maxCorr];
    std::vector<double> win;
    int winN = -1, winMode = -1, winDelay = -1;
    // réponse de chaque bande sur la grille de conception, gardée d'une conception à la suivante :
    // un jeu de FIR d'écart (FirBank.h) ne change qu'une ou deux bandes à la fois
    std::vector<cd> bandH[maxBands];
    BandSpec bandKey[maxBands];
    double bandFs[maxBands] {};
    int bandMode[maxBands] {};
};

// Fenêtre « plate à bords Blackman-Harris »
inline double halfBH (double t)   // t de 0 (bord) à 1 (plat)
{
    const double x = pi * t;
    return 0.35875 - 0.48829 * std::cos (x) + 0.14128 * std::cos (2 * x) - 0.01168 * std::cos (3 * x);
}

inline void makeWindow (Work& w, int N, Mode mode, int delay)
{
    if (w.winN == N && w.winMode == (int) mode && w.winDelay == delay) return;
    w.win.assign ((size_t) N, 1.0);
    const int tail = std::max (1, (int) (0.15 * N));
    if (mode == Mode::linear)
    {
        for (int i = 0; i < tail; ++i) { const double v = halfBH ((double) i / tail); w.win[(size_t) i] = v; w.win[(size_t) (N - 1 - i)] = v; }
    }
    else
    {
        const int rise = std::max (1, delay / 2);
        for (int i = 0; i < rise; ++i) w.win[(size_t) i] = halfBH ((double) i / rise);
        for (int i = 0; i < tail; ++i) w.win[(size_t) (N - 1 - i)] = halfBH ((double) i / tail);
    }
    w.winN = N; w.winMode = (int) mode; w.winDelay = delay;
}

// Matrice 2×2 d'une bande (réponse h) selon son placement
struct M2 { cd a, b, c, d; };
inline M2 bandMatrix (Place p, cd h, bool stereo)
{
    const cd one (1.0, 0.0);
    if (! stereo)
    {
        // mono : seule la voie gauche existe ; Mid = tout le signal, Side et Droite n'agissent pas
        if (p == Place::stereo || p == Place::left || p == Place::mid) return { h, 0, 0, one };
        return { one, 0, 0, one };
    }
    switch (p)
    {
        case Place::stereo: return { h, 0, 0, h };
        case Place::left:   return { h, 0, 0, one };
        case Place::right:  return { one, 0, 0, h };
        case Place::mid:    return { 0.5 * (h + one), 0.5 * (h - one), 0.5 * (h - one), 0.5 * (h + one) };
        case Place::side:   return { 0.5 * (one + h), 0.5 * (one - h), 0.5 * (one - h), 0.5 * (one + h) };
    }
    return { one, 0, 0, one };
}
inline M2 mul (const M2& x, const M2& y)   // x · y
{
    return { x.a * y.a + x.b * y.c, x.a * y.b + x.b * y.d, x.c * y.a + x.d * y.c, x.c * y.b + x.d * y.d };
}

// Cible (matrice complexe) à la fréquence f, toutes bandes, dans l'ordre des bandes
inline M2 targetAt (const Request& rq, const Design* des, double f)
{
    M2 m { 1.0, 0.0, 0.0, 1.0 };
    for (int i = 0; i < maxBands; ++i)
    {
        const auto& b = rq.bands[(size_t) i];
        if (! b.active) continue;
        cd h = response (des[i], std::max (f, 0.01), rq.fs, true);
        if (rq.mode == Mode::linear) h = cd (std::abs (h), 0.0);
        m = mul (bandMatrix (b.place, h, rq.stereo), m);
    }
    return m;
}

inline void design (const Request& rq, Result& out, Work& w)
{
    const int N = rq.N, M = 2 * N;   // échantillonnage dense : 2N points
    const double fs = rq.fs;
    Design des[maxBands];
    bool anyMS = false, allStereo = rq.stereo;   // allStereo : toutes les bandes sur les deux voies (D = A)
    for (int i = 0; i < maxBands; ++i)
    {
        const auto& b = rq.bands[(size_t) i];
        if (! b.active) continue;
        eq::designFir (b, fs, des[i], rq.mode == Mode::linear, N);   // (3.0 : Brickwall = mur idéal en Linear phase)
        anyMS = anyMS || (rq.stereo && (b.place == Place::mid || b.place == Place::side));
        allStereo = allStereo && b.place == Place::stereo;
    }
    for (auto& v : w.M) v.resize ((size_t) M / 2 + 1);
    const double w1 = std::min (2.0 * pi * 20000.0 / fs, 0.85 * pi);
    for (int i = 0; i < maxBands; ++i)
    {
        const auto& b = rq.bands[(size_t) i];
        if (! b.active) continue;
        const auto& key = w.bandKey[i];
        auto& H = w.bandH[i];
        // (3.0 : la pente libre, le Brickwall et le mode comptent aussi, la cible d'un Brickwall dépend du mode)
        if ((int) H.size() == M / 2 + 1 && w.bandFs[i] == fs && key.shape == b.shape && key.slope == b.slope
            && key.freq == b.freq && key.gainDb == b.gainDb && key.q == b.q
            && key.slopeFree == b.slopeFree && key.brick == b.brick && w.bandMode[i] == (int) rq.mode) continue;
        H.resize ((size_t) M / 2 + 1);
        for (int k = 0; k <= M / 2; ++k) H[(size_t) k] = response (des[i], std::max (k * fs / M, 0.01), fs, true);
        w.bandKey[i] = b; w.bandFs[i] = fs; w.bandMode[i] = (int) rq.mode;
    }
    for (int k = 0; k <= M / 2; ++k)
    {
        const double f = k * fs / M;
        // (même calcul que targetAt, avec les réponses gardées)
        M2 t { 1.0, 0.0, 0.0, 1.0 };
        for (int i = 0; i < maxBands; ++i)
        {
            const auto& b = rq.bands[(size_t) i];
            if (! b.active) continue;
            cd h = w.bandH[i][(size_t) k];
            if (rq.mode == Mode::linear) h = cd (std::abs (h), 0.0);
            t = mul (bandMatrix (b.place, h, rq.stereo), t);
        }
        if (rq.mode == Mode::natural)
        {
            // phase ramenée doucement vers 0 entre 20 kHz et Nyquist, puis retard D
            const double om = 2.0 * pi * f / fs;
            const double c = om <= w1 ? 1.0 : 0.5 + 0.5 * std::cos (pi * (om - w1) / (pi - w1));
            auto blend = [c] (cd x) { return c * x + (1.0 - c) * cd (std::abs (x), 0.0); };
            const cd rot = std::polar (1.0, -om * rq.delay);
            t = { blend (t.a) * rot, blend (t.b) * rot, blend (t.c) * rot, blend (t.d) * rot };
        }
        w.M[0][(size_t) k] = t.a; w.M[1][(size_t) k] = t.b; w.M[2][(size_t) k] = t.c; w.M[3][(size_t) k] = t.d;
    }
    makeWindow (w, N, rq.mode, rq.delay);
    out.N = N;
    out.matrix = anyMS;
    w.buf.resize ((size_t) M);
    // fréquences de correction : centre des notch (phase linéaire seulement)
    double corrF[maxCorr]; int nCorr = 0;
    if (rq.mode == Mode::linear)
        for (const auto& b : rq.bands)
            // (seulement si le FIR fait au moins le quart de la longueur voulue : plus court, la
            // correction creuserait le centre au prix d'ondulations fortes autour)
            if (b.active && b.shape == Shape::notch && b.freq < 0.45 * fs && nCorr < maxCorr
                && N >= 0.25 * requiredLength (b, fs, Mode::linear)) corrF[nCorr++] = b.freq;

    for (int e = 0; e < 4; ++e)
    {
        auto& h = out.h[e];
        if (! anyMS && (e == 1 || e == 2)) { h.assign ((size_t) N, 0.0); continue; }
        // toutes les bandes en stéréo : le FIR de droite est celui de gauche (même calcul, fait une fois)
        if (allStereo && e == 3) { h = out.h[0]; continue; }
        // spectre hermitien → réponse réelle
        for (int k = 0; k <= M / 2; ++k) w.buf[(size_t) k] = w.M[e][(size_t) k];
        for (int k = 1; k < M / 2; ++k) w.buf[(size_t) (M - k)] = std::conj (w.M[e][(size_t) k]);
        w.buf[(size_t) (M / 2)] = cd (w.M[e][(size_t) (M / 2)].real(), 0.0);
        fft (w.buf.data(), M, true);
        h.resize ((size_t) N);
        if (rq.mode == Mode::linear)
            for (int n = 0; n < N; ++n) h[(size_t) n] = w.buf[(size_t) ((n - N / 2 + M) % M)].real() / M * w.win[(size_t) n];
        else
            for (int n = 0; n < N; ++n) h[(size_t) n] = w.buf[(size_t) n].real() / M * w.win[(size_t) n];

        if (nCorr > 0)
        {
            // h += Σ c_j · win · cos(ω_j (n − N/2)) pour que la réponse vaille la cible en chaque centre
            for (int j = 0; j < nCorr; ++j)
            {
                auto& bj = w.basis[(size_t) j];
                bj.resize ((size_t) N);
                const double wj = 2.0 * pi * corrF[j] / fs;
                for (int n = 0; n < N; ++n) bj[(size_t) n] = std::cos (wj * (n - N / 2));
            }
            double G[maxCorr][maxCorr], r[maxCorr], c[maxCorr];
            for (int i = 0; i < nCorr; ++i)
            {
                const auto& bi = w.basis[(size_t) i];
                double cur = 0;
                for (int n = 0; n < N; ++n) cur += h[(size_t) n] * bi[(size_t) n];
                const M2 t = targetAt (rq, des, corrF[i]);
                const cd tv = e == 0 ? t.a : e == 1 ? t.b : e == 2 ? t.c : t.d;
                r[i] = tv.real() - cur;
                for (int j = 0; j < nCorr; ++j)
                {
                    const auto& bj = w.basis[(size_t) j];
                    double sum = 0;
                    for (int n = 0; n < N; ++n) sum += w.win[(size_t) n] * bj[(size_t) n] * bi[(size_t) n];
                    G[i][j] = sum;
                }
            }
            // élimination de Gauss avec pivot partiel
            bool ok = true;
            for (int col = 0; col < nCorr && ok; ++col)
            {
                int piv = col;
                for (int i = col + 1; i < nCorr; ++i) if (std::abs (G[i][col]) > std::abs (G[piv][col])) piv = i;
                if (std::abs (G[piv][col]) < 1e-12) { ok = false; break; }
                if (piv != col) { for (int j = 0; j < nCorr; ++j) std::swap (G[col][j], G[piv][j]); std::swap (r[col], r[piv]); }
                for (int i = col + 1; i < nCorr; ++i)
                {
                    const double fct = G[i][col] / G[col][col];
                    for (int j = col; j < nCorr; ++j) G[i][j] -= fct * G[col][j];
                    r[i] -= fct * r[col];
                }
            }
            if (ok)
            {
                for (int i = nCorr - 1; i >= 0; --i)
                {
                    double sum = r[i];
                    for (int j = i + 1; j < nCorr; ++j) sum -= G[i][j] * c[j];
                    c[i] = sum / G[i][i];
                }
                for (int j = 0; j < nCorr; ++j)
                {
                    const auto& bj = w.basis[(size_t) j];
                    for (int n = 0; n < N; ++n) h[(size_t) n] += c[j] * w.win[(size_t) n] * bj[(size_t) n];
                }
            }
        }
    }
}

// Réponse en fréquence réelle d'un FIR (latence retirée), en dB, sur une grille de fréquences.
// Calcul par FFT avec bourrage (×4, au plus 2^20 points) puis interpolation.
inline void responseDb (const std::vector<double>& h, int latency, double fs, const double* freqs, int nf, float* outDb,
                        std::vector<cd>& scratch)
{
    const int N = (int) h.size();
    int L = 1; while (L < 4 * N && L < (1 << 20)) L <<= 1;
    scratch.assign ((size_t) L, cd (0, 0));
    for (int n = 0; n < N && n < L; ++n) scratch[(size_t) n] = h[(size_t) n];
    fft (scratch.data(), L, false);
    (void) latency;
    for (int i = 0; i < nf; ++i)
    {
        const double k = freqs[i] * L / fs;
        const int k0 = std::clamp ((int) k, 0, L / 2 - 1);
        const double t = k - k0;
        const double m = std::abs (scratch[(size_t) k0]) * (1 - t) + std::abs (scratch[(size_t) k0 + 1]) * t;
        outDb[i] = (float) (20.0 * std::log10 (std::max (1e-12, m)));
    }
}

// Réponse complexe exacte d'un FIR à une fréquence (latence retirée) : pour les tests
inline cd responseAt (const std::vector<double>& h, int latency, double fs, double f)
{
    const double w = 2.0 * pi * f / fs;
    cd s (0, 0);
    // récurrence de rotation, recalée régulièrement
    cd rot (std::cos (w), -std::sin (w)), z (1, 0);
    for (size_t n = 0; n < h.size(); ++n)
    {
        if ((n & 1023) == 0) z = std::polar (1.0, -w * (double) n);
        s += h[n] * z;
        z *= rot;
    }
    return s * std::polar (1.0, w * latency);
}

// ── 3.0 : pré-écho d'un FIR ─────────────────────────────────────────────
// Un FIR à phase linéaire répond AVANT son pic principal (pré-écho), d'autant plus longtemps que la
// courbe est étroite et grave. Mesure : énergie cumulée depuis le début de la réponse, rapportée à
// l'énergie du pic (échantillon le plus fort).
//   ms60 / ms90 : durée, avant le pic, pendant laquelle cette énergie cumulée dépasse −60 / −90 dB ;
//   dbAt5ms / dbAt20ms : énergie cumulée de tout ce qui arrive plus de 5 / 20 ms avant le pic (dB).
// Rien avant le pic : durées nulles, niveaux à preEchoFloorDb.
constexpr float preEchoFloorDb = -200.0f;
struct PreEcho { float ms60 = 0.0f, ms90 = 0.0f, dbAt5ms = preEchoFloorDb, dbAt20ms = preEchoFloorDb; };

// a : réponse principale (n points) ; b : réponse croisée de la même sortie (R → L), ou nul.
template <class T>
inline PreEcho measurePreEcho (const T* a, const T* b, int n, double fs)
{
    PreEcho r;
    if (a == nullptr || n <= 0) return r;
    int pk = 0;
    for (int i = 1; i < n; ++i) if (std::abs ((double) a[i]) > std::abs ((double) a[pk])) pk = i;
    const double ref = (double) a[pk] * (double) a[pk];
    if (ref <= 0.0 || pk == 0) return r;
    const int i5 = pk - (int) std::lround (0.005 * fs), i20 = pk - (int) std::lround (0.020 * fs);
    int n60 = -1, n90 = -1;
    double cum = 0.0;
    for (int i = 0; i < pk; ++i)
    {
        cum += (double) a[i] * (double) a[i];
        if (b != nullptr) cum += (double) b[i] * (double) b[i];
        if (n90 < 0 && cum > 1.0e-9 * ref) n90 = i;
        if (n60 < 0 && cum > 1.0e-6 * ref) n60 = i;
        if (i == i5 && cum > 0.0)  r.dbAt5ms  = (float) std::max ((double) preEchoFloorDb, 10.0 * std::log10 (cum / ref));
        if (i == i20 && cum > 0.0) r.dbAt20ms = (float) std::max ((double) preEchoFloorDb, 10.0 * std::log10 (cum / ref));
    }
    if (n60 >= 0) r.ms60 = (float) (1000.0 * (pk - n60) / fs);
    if (n90 >= 0) r.ms90 = (float) (1000.0 * (pk - n90) / fs);
    return r;
}

// Pré-écho d'un jeu de FIR : la pire des deux sorties (gauche : A et B ; droite : D et C)
inline PreEcho measurePreEcho (const Result& r, double fs)
{
    const int n = (int) r.h[0].size();
    if (n == 0) return {};
    PreEcho p = measurePreEcho (r.h[0].data(), r.matrix && (int) r.h[1].size() == n ? r.h[1].data() : nullptr, n, fs);
    if ((int) r.h[3].size() == n)
    {
        const PreEcho q = measurePreEcho (r.h[3].data(), r.matrix && (int) r.h[2].size() == n ? r.h[2].data() : nullptr, n, fs);
        p.ms60 = std::max (p.ms60, q.ms60); p.ms90 = std::max (p.ms90, q.ms90);
        p.dbAt5ms = std::max (p.dbAt5ms, q.dbAt5ms); p.dbAt20ms = std::max (p.dbAt20ms, q.dbAt20ms);
    }
    return p;
}

// Texte court pour l'interface (anglais, UTF-8), par exemple « Pre-echo: 12 ms above −60 dB ».
// (mise en forme à la main : point décimal quel que soit le réglage régional de l'hôte)
inline std::string preEchoText (const PreEcho& p)
{
    if (p.ms60 < 0.05f) return "Pre-echo: none above \xe2\x88\x92" "60 dB";
    const int tenths = (int) std::lround ((double) p.ms60 * 10.0);
    std::string s = "Pre-echo: ";
    if (tenths >= 100) s += std::to_string ((tenths + 5) / 10);
    else s += std::to_string (tenths / 10) + "." + std::to_string (tenths % 10);
    return s + " ms above \xe2\x88\x92" "60 dB";
}

// ── 3.0 : résolution Auto vérifiée par la mesure (Linear phase) ─────────
// Écart max (dB) entre la réponse réelle d'un jeu de FIR Linear phase et sa cible, de 20 Hz à 20 kHz,
// là où la cible dépasse floorDb. Les creux des notch sont jugés à part (leur centre est tenu exactement
// par la correction, voir design()) : on ne compare pas là où un notch est sous −3 dB, comme dans le banc.
// Réponse : FFT du FIR avec bourrage (×4) puis interpolation cubique de l'amplitude (réelle, latence retirée).
constexpr double autoTolDb = 0.1;      // écart admis par la résolution Auto
constexpr double autoFloorDb = -60.0;  // on ne juge pas la courbe sous ce niveau
inline double linearError (const Request& rq, const Result& r, std::vector<cd>& scratch, double floorDb = autoFloorDb, double* atHz = nullptr)
{
    const int N = (int) r.h[0].size();
    if (N <= 0) return 0.0;
    const double fs = rq.fs;
    Design des[maxBands];
    for (int i = 0; i < maxBands; ++i)
        if (rq.bands[(size_t) i].active) eq::designFir (rq.bands[(size_t) i], fs, des[i], true, N);
    int L = 1; while (L < 4 * N) L <<= 1;
    const int lat = N / 2;
    const double fTop = std::min (20000.0, 0.499 * fs);
    double worst = 0.0;
    const bool rows[4] = { true, r.matrix, r.matrix, r.h[3] != r.h[0] };
    for (int e = 0; e < 4; ++e)
    {
        if (! rows[e] || (int) r.h[e].size() != N) continue;
        // réponse sans la latence : h tourné pour que le centre soit à l'indice 0 (spectre réel et lisse)
        scratch.assign ((size_t) L, cd (0, 0));
        for (int n = 0; n < N; ++n) scratch[(size_t) ((n - lat + L) % L)] = r.h[e][(size_t) n];
        fft (scratch.data(), L, false);
        auto amp = [&] (double f)
        {
            const double x = f * L / fs;
            const int k = std::clamp ((int) x, 1, L / 2 - 2);
            const double t = x - k;
            const double y0 = scratch[(size_t) (k - 1)].real(), y1 = scratch[(size_t) k].real(),
                         y2 = scratch[(size_t) (k + 1)].real(), y3 = scratch[(size_t) (k + 2)].real();
            return y1 + 0.5 * t * (y2 - y0 + t * (2.0 * y0 - 5.0 * y1 + 4.0 * y2 - y3 + t * (3.0 * (y1 - y2) + y3 - y0)));
        };
        auto check = [&] (double f)
        {
            if (f < 20.0 || f > fTop) return;
            for (int i = 0; i < maxBands; ++i)
            {
                const auto& b = rq.bands[(size_t) i];
                if (b.active && b.shape == Shape::notch && std::abs (response (des[i], f, fs, true)) < 0.7079) return;
            }
            const M2 t = targetAt (rq, des, f);
            const double tv = std::abs (e == 0 ? t.a : e == 1 ? t.b : e == 2 ? t.c : t.d);
            const double tdb = 20.0 * std::log10 (std::max (1e-12, tv));
            if (tdb < floorDb) return;
            const double d = std::abs (20.0 * std::log10 (std::max (1e-12, std::abs (amp (f)))) - tdb);
            if (d > worst) { worst = d; if (atHz != nullptr) *atHz = f; }
        };
        constexpr int grid = 360;
        for (int i = 0; i < grid; ++i) check (20.0 * std::pow (fTop / 20.0, i / (grid - 1.0)));
        // (points serrés autour de chaque bande : ± 1 octave)
        for (const auto& b : rq.bands)
            if (b.active)
                for (int i = -40; i <= 40; ++i) check (b.freq * std::pow (2.0, i / 40.0));
    }
    return worst;
}

// Longueur de la résolution Auto en Linear phase : celle de la règle (requiredLength), puis vérifiée par
// la mesure et allongée tant que l'écart à la cible dépasse autoTolDb (au plus la longueur Max).
// Bandes dynamiques : la courbe est vérifiée au gain de base et à pleine plage. Fil de conception
// seulement (plusieurs conceptions de FIR). errOut : écart mesuré à la longueur retenue.
inline int verifiedAutoLength (const std::array<BandSpec, maxBands>& bands, const std::array<double, maxBands>& dynSpan,
                               bool stereo, double fs, Work& w, Result& tmp, std::vector<cd>& scratch, double* errOut = nullptr)
{
    int n = chooseLength (Res::automatic, bands, fs, Mode::linear, &dynSpan);
    bool anyDyn = false;
    for (int i = 0; i < maxBands; ++i) anyDyn = anyDyn || isDynamicSpan (bands[(size_t) i], dynSpan[(size_t) i]);
    for (;; n *= 2)
    {
        Request rq; rq.bands = bands; rq.stereo = stereo; rq.mode = Mode::linear; rq.N = n; rq.fs = fs;
        design (rq, tmp, w);
        double e = linearError (rq, tmp, scratch);
        if (anyDyn && e <= autoTolDb)
        {
            for (int i = 0; i < maxBands; ++i)
                if (isDynamicSpan (bands[(size_t) i], dynSpan[(size_t) i])) rq.bands[(size_t) i].gainDb += dynSpan[(size_t) i];
            design (rq, tmp, w);
            e = std::max (e, linearError (rq, tmp, scratch));
        }
        if (errOut != nullptr) *errOut = e;
        if (e <= autoTolDb || n >= maxLength (fs)) return n;
    }
}
} // namespace audiolab::eq::fir
