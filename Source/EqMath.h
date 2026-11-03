// Audio Lab EQ — calcul des filtres (conception + réponse en fréquence analytique).
// Les filtres sont des SVF « trapézoïdaux » (Andrew Simper, Cytomic) : stables quand on
// module la fréquence ou le gain à chaque échantillon, ce qui permet l'EQ dynamique sans clics.
// La même description sert au traitement temps réel, à l'affichage de la courbe et à la
// conception des filtres FIR des modes à phase linéaire / naturelle : ce qu'on voit est ce qu'on entend.
//
// 2.0 : le mode zéro latence n'utilise plus la transformation bilinéaire (qui « écrase » la courbe
// près de Nyquist : une cloche à 16 kHz donnait +0,7 dB au lieu de +5 dB à 20 kHz, à 44,1 kHz).
// Conception « matched » (d'après M. Vicanek, « Matched Second Order Digital Filters », 2016) :
//  - pôles placés exactement par invariance impulsionnelle (z = e^{sT}) : résonance et amortissement exacts ;
//  - numérateur ajusté au module analogique sur toute la bande (moindres carrés relatifs, DC exact,
//    deux itérations de Lawson vers le minimax), avec un 2e candidat à 3 points (DC, Nyquist, résonance) ;
//  - atténuations (cloches, étagères) conçues comme l'inverse exact de l'accentuation, pour que
//    la paire la plus étroite soit toujours celle placée exactement ;
//  - sections du 1er ordre réalisées par un biquad (un pôle exact + un pôle fixe en e^{-π}) : la
//    liberté supplémentaire suit la pente analogique jusqu'à 20 kHz.
// Chaque biquad obtenu est remis sous la forme du SVF existant (g, k, m0, m1, m2) : tout biquad
// stable s'y ramène exactement, et response() (en tan) donne alors la réponse RÉELLE du filtre numérique.
#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <complex>

namespace audiolab::eq
{
constexpr int maxBands = 24;
constexpr double pi = 3.14159265358979323846;

enum class Shape { bell = 0, lowShelf, lowCut, highShelf, highCut, notch, bandPass, tiltShelf };
constexpr int numShapes = 8;
enum class Place { stereo = 0, left, right, mid, side };
constexpr int numPlaces = 5;

constexpr int slopeValues[] = { 6, 12, 18, 24, 30, 36, 48, 72, 96 };
constexpr int numSlopes = 9;

inline bool isCut (Shape s)        { return s == Shape::lowCut || s == Shape::highCut; }
inline bool hasGain (Shape s)      { return s == Shape::bell || s == Shape::lowShelf || s == Shape::highShelf || s == Shape::tiltShelf; }
inline bool usesSlope (Shape s)    { return s == Shape::lowCut || s == Shape::highCut || s == Shape::bandPass
                                         || s == Shape::lowShelf || s == Shape::highShelf || s == Shape::tiltShelf; }
inline bool usesQ (Shape s, int slope)
{
    if (s == Shape::tiltShelf) return slope > 6;
    if (s == Shape::lowShelf || s == Shape::highShelf) return slope > 6;
    if (isCut (s)) return slope > 6;
    return true;
}
inline bool canBeDynamic (Shape s) { return hasGain (s); }

struct BandSpec
{
    bool active = false;           // bande utilisée et activée
    Shape shape = Shape::bell;
    double freq = 1000.0, gainDb = 0.0, q = 1.0;
    int slope = 12;                // dB/octave
    Place place = Place::stereo;
};

// Section élémentaire : SVF du 2e ordre ou filtre du 1er ordre (TPT).
struct Section
{
    enum Kind { svf = 0, onePole = 1 };
    Kind kind = svf;
    double g = 0.1;                    // tan (pi f / fs), éventuellement décalé (shelfs)
    double k = 1.4142135623730951;     // 1/Q
    double m0 = 1, m1 = 0, m2 = 0;     // SVF : y = m0·x + m1·bp + m2·lp
    double cLp = 1, cHp = 0;           // 1er ordre : y = cLp·lp + cHp·hp
};

struct Design
{
    std::array<Section, 9> s {};
    int n = 0;
    double outGain = 1.0;
};

// Coefficients de Butterworth pour un filtre d'ordre N (sections du 2e ordre).
inline int butterworthQs (int order, double* qs, bool& hasFirstOrder)
{
    hasFirstOrder = (order % 2) == 1;
    const int nb = order / 2;
    for (int i = 0; i < nb; ++i)
    {
        const double ang = hasFirstOrder ? (i + 1) * pi / order : (2.0 * (i + 1) - 1.0) * pi / (2.0 * order);
        qs[i] = 1.0 / (2.0 * std::cos (ang));
    }
    return nb;
}

// Pulsation « analogique » normalisée utilisée pour la conception :
// - bilinéaire (ancienne conception zéro latence, gardée pour les mesures « avant ») : tan (pi f / fs)
// - analogique (prototype : FIR, conception matched) : pi f / fs, sans compression près de Nyquist
inline double warp (double f, double fs, bool analog)
{
    const double x = pi * f / fs;
    return analog ? x : std::tan (std::min (x, pi * 0.4999));
}

inline double clampFreq (double f, double fs, bool analog)
{
    return std::clamp (f, 5.0, analog ? 0.5 * fs * 1.5 : 0.4985 * fs);
}

// Prototype analogique (analog = true) ou conception bilinéaire (analog = false), comme en 1.x.
inline void designClassic (const BandSpec& b, double fs, Design& d, bool analog)
{
    d.n = 0;
    d.outGain = 1.0;
    const double f = clampFreq (b.freq, fs, analog);
    const double g0 = warp (f, fs, analog);
    const double q = std::clamp (b.q, 0.025, 40.0);
    const double gain = b.gainDb;

    auto addSvf = [&] (double g, double k, double m0, double m1, double m2)
    {
        auto& s = d.s[(size_t) d.n++];
        s.kind = Section::svf; s.g = g; s.k = k; s.m0 = m0; s.m1 = m1; s.m2 = m2;
    };
    auto addOne = [&] (double g, double cLp, double cHp)
    {
        auto& s = d.s[(size_t) d.n++];
        s.kind = Section::onePole; s.g = g; s.cLp = cLp; s.cHp = cHp;
    };

    switch (b.shape)
    {
        case Shape::bell:
        {
            const double A = std::pow (10.0, gain / 40.0);
            const double k = 1.0 / (q * A);
            addSvf (g0, k, 1.0, k * (A * A - 1.0), 0.0);
            break;
        }
        case Shape::lowShelf:
        {
            const double A = std::pow (10.0, gain / 40.0);
            if (b.slope <= 6) addOne (g0 / A, A * A, 1.0);
            else addSvf (g0 / std::sqrt (A), 1.0 / q, 1.0, (1.0 / q) * (A - 1.0), A * A - 1.0);
            break;
        }
        case Shape::highShelf:
        {
            const double A = std::pow (10.0, gain / 40.0);
            if (b.slope <= 6) addOne (g0 * A, 1.0, A * A);
            else addSvf (g0 * std::sqrt (A), 1.0 / q, A * A, (1.0 / q) * (1.0 - A) * A, 1.0 - A * A);
            break;
        }
        case Shape::tiltShelf:
        {
            // pivot à f : -gain/2 dans le grave, +gain/2 dans l'aigu
            const double A = std::pow (10.0, gain / 40.0);
            if (b.slope <= 6) addOne (g0 * A, 1.0, A * A);
            else addSvf (g0 * std::sqrt (A), 1.0 / q, A * A, (1.0 / q) * (1.0 - A) * A, 1.0 - A * A);
            d.outGain = 1.0 / A;
            break;
        }
        case Shape::lowCut:
        case Shape::highCut:
        {
            const bool hp = b.shape == Shape::lowCut;
            const int order = std::clamp (b.slope / 6, 1, 16);
            double qs[8]; bool first = false;
            const int nb = butterworthQs (order, qs, first);
            if (first) addOne (g0, hp ? 0.0 : 1.0, hp ? 1.0 : 0.0);
            // la section la plus résonante suit le Q choisi (Q 0,71 = Butterworth)
            for (int i = 0; i < nb; ++i)
            {
                double qi = qs[i];
                if (i == nb - 1) qi *= q / 0.7071067811865476;
                const double k = 1.0 / std::clamp (qi, 0.025, 60.0);
                if (hp) addSvf (g0, k, 1.0, -k, -1.0);
                else addSvf (g0, k, 0.0, 0.0, 1.0);
            }
            break;
        }
        case Shape::notch:
        {
            const double k = 1.0 / q;
            addSvf (g0, k, 1.0, -k, 0.0);
            break;
        }
        case Shape::bandPass:
        {
            const int n = std::clamp (b.slope / 12, 1, 8);
            const double k = 1.0 / q;
            for (int i = 0; i < n; ++i) addSvf (g0, k, 0.0, k, 0.0);
            break;
        }
    }
}

// ── Conception « matched » ──────────────────────────────────────────────
namespace matched
{
// Points de calcul fixes pour une fréquence d'échantillonnage (log de 10 Hz à 20,5 kHz, plus le
// haut du spectre en linéaire). Au-delà de 20,5 kHz (96 / 192 kHz), poids réduit : la courbe y reste
// sage sans gêner la précision dans l'audible.
struct Grid
{
    static constexpr int fixedPts = 30, maxPts = 40;
    double fs = 0, top = pi, wAudible = pi;
    int n = 0;
    double w[maxPts] {}, p0[maxPts] {}, p1[maxPts] {};

    void init (double rate)
    {
        fs = rate;
        wAudible = std::min (pi, 2.0 * pi * 20500.0 / fs);
        top = fs < 60000.0 ? wAudible : pi;
        const double lo = 2.0 * pi * 10.0 / fs;
        n = 0;
        for (int i = 0; i < 20; ++i) w[n++] = lo * std::pow (wAudible / lo, i / 19.0);
        for (int i = 0; i < 5; ++i) w[n++] = wAudible * (0.5 + 0.5 * (i + 0.5) / 5.0);
        if (top > wAudible * 1.05)
            for (int i = 0; i < 3; ++i) w[n++] = wAudible + (top - wAudible) * (i + 1) / 3.0;
        for (int i = 0; i < n; ++i) { const double c = std::cos (0.5 * w[i]); p0[i] = c * c; p1[i] = 1.0 - c * c; }
    }
};

inline const Grid& gridFor (double fs)
{
    static thread_local Grid g;
    if (g.fs != fs) g.init (fs);
    return g;
}

// Section analogique : H(s) = m0 + (m1 s + m2)/(s² + k s + 1), s = jω/W (ω en rad/éch.)
// ou 1er ordre : (cLp + cHp s)/(1 + s).
struct ASec
{
    bool svf = true;
    double W = 0.1, k = 1.414, m0 = 1, m1 = 0, m2 = 0, cLp = 1, cHp = 0;
};

inline ASec fromSection (const Section& s)
{
    ASec a;
    a.svf = s.kind == Section::svf;
    a.W = 2.0 * s.g;   // prototype analogique : g = pi f / fs = W/2
    a.k = s.k; a.m0 = s.m0; a.m1 = s.m1; a.m2 = s.m2; a.cLp = s.cLp; a.cHp = s.cHp;
    return a;
}

inline double mag2 (const ASec& a, double w)
{
    const double x = w / a.W;   // s = jx
    if (a.svf)
    {
        // N = m0 (jx)² + (m0 k + m1) jx + (m0 + m2) ; D = (jx)² + k jx + 1
        const double nr = a.m0 + a.m2 - a.m0 * x * x, ni = (a.m0 * a.k + a.m1) * x;
        const double dr = 1.0 - x * x, di = a.k * x;
        return (nr * nr + ni * ni) / std::max (1e-300, dr * dr + di * di);
    }
    return (a.cLp * a.cLp + a.cHp * a.cHp * x * x) / (1.0 + x * x);
}

struct Phi { double p0, p1, p2; };
inline Phi phi (double w)
{
    const double c = std::cos (0.5 * w), s = std::sin (0.5 * w);
    return { c * c, s * s, 4.0 * c * c * s * s };
}

// Pôles de s² + k W s + W² par invariance impulsionnelle : 1 + a1 z⁻¹ + a2 z⁻²
inline void polesII (double W, double k, double& a1, double& a2)
{
    const double z = 0.5 * k;
    if (z <= 1.0) a1 = -2.0 * std::exp (-z * W) * std::cos (std::sqrt (1.0 - z * z) * W);
    else
    {
        const double r = std::sqrt (z * z - 1.0);
        a1 = -(std::exp (-(z - r) * W) + std::exp (-(z + r) * W));
    }
    a2 = std::exp (-2.0 * z * W);
}

// |B(e^jω)|² = B0 φ0 + B1 φ1 + B2 φ2 → b0 + b1 z⁻¹ + b2 z⁻² (phase minimale)
inline void factor (double B0, double B1, double B2, double* b)
{
    const double S0 = std::sqrt (std::max (0.0, B0)), S1 = std::sqrt (std::max (0.0, B1));
    const double W = 0.5 * (S0 + S1);
    const double disc = std::max (0.0, W * W + B2);
    b[0] = 0.5 * (W + std::sqrt (disc));
    b[1] = 0.5 * (S0 - S1);
    b[2] = b[0] > 0 ? -B2 / (4.0 * b[0]) : 0.0;
}

// Biquad (b0, b1, b2 ; 1, a1, a2) → SVF trapézoïdal (g, k, m0, m1, m2). Faux si instable.
inline bool svfFromBiquad (const double* b, double a1, double a2, Section& s)
{
    const double P = 1.0 + a1 + a2, M = 1.0 - a1 + a2;
    if (! (P > 1e-14 && M > 1e-14 && 1.0 - a2 > 1e-14)) return false;
    const double g = std::sqrt (P / M);
    const double k = 2.0 * (1.0 - a2) / (M * g);
    const double D = 1.0 + k * g + g * g;
    const double B1 = b[0] + b[1] + b[2], Bm1 = b[0] - b[1] + b[2];
    const double n0 = B1 * D / (4.0 * g * g);
    const double n2 = Bm1 * D / 4.0;
    const double n1 = (b[0] * D - n2 - n0 * g * g) / g;
    s.kind = Section::svf; s.g = g; s.k = k; s.m0 = n2; s.m1 = n1 - k * n2; s.m2 = n0 - n2;
    return std::isfinite (g) && std::isfinite (k) && std::isfinite (s.m0) && std::isfinite (s.m1) && std::isfinite (s.m2);
}

// Points de calcul d'une section : grille fixe + points autour de la résonance (pôles) et des zéros.
struct Pts
{
    int n = 0;
    double w[Grid::maxPts], ref[Grid::maxPts], wt[Grid::maxPts], t[Grid::maxPts];
    Phi ph[Grid::maxPts];
    double tmax = 0;
};

inline void addFeature (Pts& p, double W, double Q, double top)
{
    if (! (W > 0) || W >= top) return;
    const double d = 0.5 / std::max (0.5, Q);
    for (double m : { std::pow (2.0, -d), 1.0, std::pow (2.0, d) })
        if (W * m < top && p.n < Grid::maxPts) p.w[p.n++] = W * m;
}

// Prépare les points : cible T(ω)·|A(e^jω)|² et poids relatifs
inline void preparePts (Pts& p, const Grid& g, const ASec& a, const double* A, double Wz, double Qz)
{
    p.n = 0;
    for (int i = 0; i < g.n; ++i)
    {
        p.w[p.n] = g.w[i];
        p.ph[p.n] = { g.p0[i], g.p1[i], 4.0 * g.p0[i] * g.p1[i] };
        ++p.n;
    }
    const int nFixed = p.n;
    addFeature (p, a.W, a.svf ? 1.0 / a.k : 0.5, g.top);
    addFeature (p, Wz, Qz, g.top);
    for (int i = nFixed; i < p.n; ++i) p.ph[i] = phi (p.w[i]);
    p.tmax = 0;
    for (int i = 0; i < p.n; ++i)
    {
        const double Aw = A[0] * p.ph[i].p0 + A[1] * p.ph[i].p1 + A[2] * p.ph[i].p2;
        p.t[i] = mag2 (a, p.w[i]);
        p.tmax = std::max (p.tmax, p.t[i]);
        p.ref[i] = p.t[i] * Aw;
        p.wt[i] = (p.w[i] <= g.wAudible * 1.0001 ? 1.0 : 0.1) / std::max (1e-300, p.ref[i]);
    }
}

// Moindres carrés pondérés (1 ou 2 inconnues) + 2 itérations de Lawson (vers le minimax)
// modèle : ref ≈ c + x1·u1 + x2·u2 (u = fonctions de φ)
template <typename F>
inline void lawson (const Pts& p, int nUnknowns, F&& basis, double* x)
{
    double lw[Grid::maxPts];
    for (int i = 0; i < p.n; ++i) lw[i] = 1.0;
    for (int it = 0; it < 3; ++it)
    {
        double a11 = 0, a12 = 0, a22 = 0, r1 = 0, r2 = 0;
        for (int i = 0; i < p.n; ++i)
        {
            double u1, u2, y;
            basis (i, u1, u2, y);
            const double w = p.wt[i] * lw[i], w2 = w * w;
            a11 += w2 * u1 * u1; a12 += w2 * u1 * u2; a22 += w2 * u2 * u2;
            r1 += w2 * u1 * y; r2 += w2 * u2 * y;
        }
        if (nUnknowns == 1) x[0] = a11 > 0 ? r1 / a11 : 0.0;
        else
        {
            const double det = a11 * a22 - a12 * a12;
            if (std::abs (det) < 1e-300) { x[0] = a11 > 0 ? r1 / a11 : 0.0; x[1] = 0; }
            else { x[0] = (r1 * a22 - r2 * a12) / det; x[1] = (a11 * r2 - a12 * r1) / det; }
        }
        if (it == 2) break;
        double mx = 0;
        for (int i = 0; i < p.n; ++i)
        {
            double u1, u2, y;
            basis (i, u1, u2, y);
            const double e = std::abs (x[0] * u1 + (nUnknowns > 1 ? x[1] * u2 : 0.0) - y) * p.wt[i];
            lw[i] *= std::max (e, 1e-12);
            mx = std::max (mx, lw[i]);
        }
        for (int i = 0; i < p.n; ++i) lw[i] = lw[i] / mx + 0.05;
    }
}

// Écart d'une section complète (b, a) à la cible analogique (pour choisir entre candidats)
inline double checkSection (const Pts& p, const ASec& a, const double* b, double a1, double a2, bool soften)
{
    double B[3], A[3];
    B[0] = (b[0] + b[1] + b[2]) * (b[0] + b[1] + b[2]); B[1] = (b[0] - b[1] + b[2]) * (b[0] - b[1] + b[2]); B[2] = -4.0 * b[0] * b[2];
    A[0] = (1 + a1 + a2) * (1 + a1 + a2); A[1] = (1 - a1 + a2) * (1 - a1 + a2); A[2] = -4.0 * a2;
    // cloches / étagères : pas de zone profonde, écart relatif pur ; coupes, passe-bande, notch :
    // les zones très atténuées (< −30 dB) comptent moins
    const double eps = soften ? 1e-3 * p.tmax : 0.0;
    double e = 1.0;   // écart sous forme de rapport (≥ 1), sans logarithme
    for (int i = 0; i < p.n; ++i)
    {
        const double num = B[0] * p.ph[i].p0 + B[1] * p.ph[i].p1 + B[2] * p.ph[i].p2;
        const double den = A[0] * p.ph[i].p0 + A[1] * p.ph[i].p1 + A[2] * p.ph[i].p2;
        if (! (den > 0)) return 1e9;
        const double r = (std::max (num / den, 0.0) + eps) / (p.t[i] + eps);
        e = std::max (e, r > 1.0 ? r : 1.0 / r);
    }
    (void) a;
    return std::isfinite (e) ? e : 1e9;
}

enum class Kind { general, lowpassLike, highpass, bandpass, notch, firstOrder };

// Une section analogique → une section SVF numérique (vrai si réussi)
inline bool section (const ASec& a, Kind kind, double Wz, double Qz, double fs, bool invert, Section& out)
{
    const Grid& g = gridFor (fs);
    double a1, a2;   // pôles par invariance impulsionnelle (ou pôle exact + e^{-π} au 1er ordre)
    if (kind == Kind::firstOrder)
    {
        // pôle exact e^{-W} + pôle fixe e^{-π} : un zéro de plus pour suivre la pente jusqu'à 20 kHz
        const double p1 = std::exp (-a.W), p2 = std::exp (-pi);
        a1 = -(p1 + p2); a2 = p1 * p2;
    }
    else polesII (a.W, a.k, a1, a2);
    const double A[3] = { (1 + a1 + a2) * (1 + a1 + a2), (1 - a1 + a2) * (1 - a1 + a2), -4.0 * a2 };

    Pts p;
    preparePts (p, g, a, A, Wz, Qz);
    double b[3] = { 1, 0, 0 }, ba1 = a1, ba2 = a2;   // meilleur candidat retenu
    double bestErr = 1e18;
    double cand[3];
    auto considerFull = [&] (const double* bb, double na1, double na2)
    {
        if (! (std::abs (na2) < 0.999999 && std::abs (na1) < 1 + na2)) return;
        const double e = checkSection (p, a, bb, na1, na2, kind != Kind::general);
        if (e < bestErr) { bestErr = e; b[0] = bb[0]; b[1] = bb[1]; b[2] = bb[2]; ba1 = na1; ba2 = na2; }
    };
    auto consider = [&] (const double* bb) { considerFull (bb, a1, a2); };

    const double T0 = mag2 (a, 0.0);
    if (kind == Kind::highpass)
    {
        // double zéro en DC : |B|² = 16 b0² φ1² ; facteur d'échelle minimax (moyenne géométrique)
        double rmax = 0, rmin = 1e300;
        for (int i = 0; i < p.n; ++i)
        {
            if (p.t[i] < 1e-3 * p.tmax) continue;
            const double r = p.ref[i] / (16.0 * p.ph[i].p1 * p.ph[i].p1);
            rmax = std::max (rmax, r); rmin = std::min (rmin, r);
        }
        if (rmax > 0)
        {
            const double b0 = std::sqrt (std::sqrt (rmax * rmin));
            cand[0] = b0; cand[1] = -2 * b0; cand[2] = b0;
            consider (cand);
        }
        // 2e candidat : pôles libres (asymptote basse exacte, Nyquist, valeur à la coupure)
        {
            const double beta = 16.0;
            const double A0 = std::pow (a.W, 4.0);
            const double TN = mag2 (a, pi);
            const double wm = std::min (a.W, 0.95 * pi);
            const Phi pm = phi (wm);
            const double tm = mag2 (a, wm);
            if (TN > 0 && tm > 0)
            {
                const double A1 = beta / TN;
                const double A2 = (beta * pm.p1 * pm.p1 / tm - A0 * pm.p0 - A1 * pm.p1) / pm.p2;
                double aa[3];
                factor (A0, A1, A2, aa);
                if (aa[0] > 0 && std::isfinite (aa[0]))
                {
                    const double s = 1.0 / aa[0];
                    const double na1 = aa[1] * s, na2 = aa[2] * s;
                    const double bb[3] = { s, -2 * s, s };
                    considerFull (bb, na1, na2);
                }
            }
        }
    }
    else if (kind == Kind::notch)
    {
        // zéros exacts sur le cercle en W
        const double cw = std::cos (a.W), xW = std::sin (0.5 * a.W) * std::sin (0.5 * a.W);
        // candidat 1 : pôles exacts, gain exact en DC (le reste du spectre ne doit pas bouger)
        {
            const double b0 = std::sqrt (A[0]) / std::max (1e-300, 2.0 - 2.0 * cw);
            cand[0] = b0; cand[1] = -2 * cw * b0; cand[2] = b0;
            consider (cand);
        }
        // candidat 2 : pôles libres (DC = 1, Nyquist analogique, point à -3 dB inférieur)
        {
            const double Q = 1.0 / a.k;
            const double A0 = xW * xW;
            const double TN = mag2 (a, pi);
            const double d = 0.5 / Q, wl = a.W * (std::sqrt (1 + d * d) - d);
            const Phi pl = phi (wl);
            const double tl = mag2 (a, wl);
            if (TN > 0 && tl > 0)
            {
                const double A1 = (1 - xW) * (1 - xW) / TN;
                const double A2 = ((pl.p1 - xW) * (pl.p1 - xW) / tl - A0 * pl.p0 - A1 * pl.p1) / pl.p2;
                double aa[3];
                factor (A0, A1, A2, aa);
                if (aa[0] > 0 && std::isfinite (aa[0]))
                {
                    const double s = 1.0 / aa[0];
                    const double na1 = aa[1] * s, na2 = aa[2] * s;
                    const double bb[3] = { 0.25 * s, -0.5 * cw * s, 0.25 * s };
                    considerFull (bb, na1, na2);
                }
            }
        }
    }
    else if (kind == Kind::bandpass || (kind == Kind::firstOrder && T0 <= 1e-20))
    {
        // zéro simple en DC, asymptote basse exacte : B1 + 4 B2 = c, une inconnue (B2)
        // près de DC : T ≈ (m1 ω / W)² (passe-bande m1 s / D) ou (cHp ω / W)² (1er ordre), et
        // |B|²/|A|² ≈ (B1 + 4 B2) x / A0 avec x ≈ ω²/4  →  c = 4 A0 m² / W²
        const double m = kind == Kind::bandpass ? a.m1 : a.cHp;
        const double c = 4.0 * A[0] * m * m / (a.W * a.W);
        double x[2] = { 0, 0 };
        lawson (p, 1, [&] (int i, double& u1, double& u2, double& y)
        {
            u1 = p.ph[i].p2 - 4.0 * p.ph[i].p1; u2 = 0; y = p.ref[i] - c * p.ph[i].p1;
        }, x);
        const double B[3] = { 0.0, c - 4.0 * x[0], x[0] };
        factor (B[0], B[1], B[2], cand);
        consider (cand);
    }
    else
    {
        // DC exact ; deux inconnues (B1, B2) par moindres carrés + Lawson
        const double B0 = A[0] * T0;
        double x[2] = { 0, 0 };
        lawson (p, 2, [&] (int i, double& u1, double& u2, double& y)
        {
            u1 = p.ph[i].p1; u2 = p.ph[i].p2; y = p.ref[i] - B0 * p.ph[i].p0;
        }, x);
        factor (B0, x[0], x[1], cand);
        consider (cand);
        if (kind != Kind::firstOrder && a.m0 > 0 && a.m0 + a.m2 > 0)
        {
            // candidat « zéros exacts » : paire de zéros placée elle aussi par invariance impulsionnelle
            // (étagères à Q élevé : creux / bosse étroits que l'ajustement ne suit pas), gain DC exact
            const double Wzr = a.W * std::sqrt ((a.m0 + a.m2) / a.m0);
            const double kz = (a.m0 * a.k + a.m1) / std::sqrt (a.m0 * (a.m0 + a.m2));
            if (Wzr < 0.9 * pi && kz > 0)
            {
                double c1, c2;
                polesII (Wzr, kz, c1, c2);
                const double K = std::sqrt (B0) / std::max (1e-300, std::abs (1.0 + c1 + c2));
                cand[0] = K; cand[1] = K * c1; cand[2] = K * c2;
                consider (cand);
            }
        }
        if (kind != Kind::firstOrder)
        {
            // candidat à 3 points : DC, Nyquist, résonance
            const double wm = std::min (a.W, 0.98 * pi);
            const Phi pm = phi (wm);
            const double B1 = A[1] * mag2 (a, pi);
            const double B2 = (mag2 (a, wm) * (A[0] * pm.p0 + A[1] * pm.p1 + A[2] * pm.p2) - B0 * pm.p0 - B1 * pm.p1) / pm.p2;
            factor (B0, B1, B2, cand);
            consider (cand);
        }
    }
    if (a.svf)
    {
        // dernier candidat : la conception bilinéaire de la 1.x (jamais pire qu'avant, section par section)
        const double gb = std::tan (std::min (0.5 * a.W, 0.4999 * pi));
        const double D = 1.0 + a.k * gb + gb * gb;
        const double n2 = a.m0, n1 = a.m0 * a.k + a.m1, n0 = a.m0 + a.m2;
        const double bb[3] = { (n2 + n1 * gb + n0 * gb * gb) / D, 2.0 * (n0 * gb * gb - n2) / D, (n2 - n1 * gb + n0 * gb * gb) / D };
        considerFull (bb, 2.0 * (gb * gb - 1.0) / D, (1.0 - a.k * gb + gb * gb) / D);
    }
    if (bestErr >= 1e17) return false;

    if (invert)
    {
        // H⁻¹ = A/B (B est à phase minimale : ses zéros deviennent des pôles stables)
        if (! (std::abs (b[0]) > 1e-300) || std::abs (b[2] / b[0]) >= 0.999999) return false;
        const double s = 1.0 / b[0];
        const double nb[3] = { s, ba1 * s, ba2 * s };
        return svfFromBiquad (nb, b[1] * s, b[2] * s, out);
    }
    return svfFromBiquad (b, ba1, ba2, out);
}
} // namespace matched

// Conception numérique « matched » (mode zéro latence, affichage, gain auto).
inline void designMatched (const BandSpec& b, double fs, Design& d)
{
    d.n = 0;
    d.outGain = 1.0;
    // bande à 0 dB : traversée exacte (aucune section)
    if (hasGain (b.shape) && std::abs (b.gainDb) < 1e-9) return;

    const double f = std::clamp (b.freq, 5.0, 0.49 * fs);
    BandSpec proto = b;
    proto.freq = f;
    bool invert = false;
    const bool firstOrderShelf = (b.shape == Shape::lowShelf || b.shape == Shape::highShelf || b.shape == Shape::tiltShelf) && b.slope <= 6;
    if (! firstOrderShelf)
    {
        // atténuation = inverse exact de l'accentuation ; on conçoit la version dont les pôles sont
        // les plus graves / les plus étroits (placés exactement), puis on l'inverse si besoin
        if (b.shape == Shape::bell && b.gainDb < 0) { invert = true; proto.gainDb = -b.gainDb; }
        if (b.shape == Shape::lowShelf && b.gainDb < 0) { invert = true; proto.gainDb = -b.gainDb; }
        if ((b.shape == Shape::highShelf || b.shape == Shape::tiltShelf) && b.gainDb > 0) { invert = true; proto.gainDb = -b.gainDb; }
    }
    Design ad;
    designClassic (proto, fs, ad, true);
    const double A = std::pow (10.0, std::abs (proto.gainDb) / 40.0);

    for (int i = 0; i < ad.n; ++i)
    {
        const auto a = matched::fromSection (ad.s[(size_t) i]);
        matched::Kind kind = matched::Kind::general;
        double Wz = a.W, Qz = a.svf ? 1.0 / a.k : 0.5;
        if (! a.svf) kind = matched::Kind::firstOrder;
        else switch (b.shape)
        {
            case Shape::bell:      Qz = std::clamp (b.q, 0.025, 40.0) / A; break;
            case Shape::lowShelf:  Wz = a.W * A; Qz = 1.0 / a.k; break;
            case Shape::highShelf:
            case Shape::tiltShelf: Wz = a.W / A; Qz = 1.0 / a.k; break;
            case Shape::lowCut:    kind = matched::Kind::highpass; break;
            case Shape::highCut:   kind = matched::Kind::lowpassLike; break;
            case Shape::bandPass:  kind = matched::Kind::bandpass; break;
            case Shape::notch:     kind = matched::Kind::notch; break;
        }
        Section s;
        if (! matched::section (a, kind, Wz, Qz, fs, invert && a.svf, s))
        {
            // secours (ne devrait pas arriver) : conception bilinéaire de cette bande
            designClassic (b, fs, d, false);
            return;
        }
        d.s[(size_t) d.n++] = s;
    }
    if (b.shape == Shape::tiltShelf) d.outGain = std::pow (10.0, -b.gainDb / 40.0);
}

// Conception d'une bande : analog = prototype analogique (FIR, référence), sinon numérique « matched ».
inline void design (const BandSpec& b, double fs, Design& d, bool analog = false)
{
    if (analog) designClassic (b, fs, d, true);
    else designMatched (b, fs, d);
}

// Ancienne conception (bilinéaire), pour les mesures « avant / après »
inline void designBilinear (const BandSpec& b, double fs, Design& d) { designClassic (b, fs, d, false); }

// Réponse complexe à la fréquence f (Hz) d'une conception.
inline std::complex<double> response (const Design& d, double f, double fs, bool analog)
{
    std::complex<double> h (d.outGain, 0.0);
    const double w = analog ? pi * f / fs : std::tan (std::min (pi * f / fs, pi * 0.49999));
    for (int i = 0; i < d.n; ++i)
    {
        const auto& s = d.s[(size_t) i];
        const std::complex<double> sj (0.0, w / s.g);
        if (s.kind == Section::svf)
        {
            const auto den = sj * sj + s.k * sj + 1.0;
            h *= s.m0 + (s.m1 * sj + s.m2) / den;
        }
        else
        {
            const auto den = sj + 1.0;
            h *= (s.cLp + s.cHp * sj) / den;
        }
    }
    return h;
}

inline double magnitudeDb (const Design& d, double f, double fs, bool analog)
{
    return 20.0 * std::log10 (std::max (1.0e-12, std::abs (response (d, f, fs, analog))));
}

// Note la plus proche d'une fréquence (La 440) : nom et écart en cents.
inline void nearestNote (double f, int& midi, double& cents)
{
    const double n = 69.0 + 12.0 * std::log2 (std::max (1.0, f) / 440.0);
    midi = (int) std::lround (n);
    cents = (n - midi) * 100.0;
}

inline const char* noteName (int midi)
{
    static const char* names[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
    return names[((midi % 12) + 12) % 12];
}

inline int noteOctave (int midi) { return midi / 12 - 1; }

// Largeur de bande (octaves) ↔ Q pour une cloche
inline double qFromOctaves (double bw)
{
    const double p = std::pow (2.0, bw);
    return std::sqrt (p) / (p - 1.0);
}
} // namespace audiolab::eq
