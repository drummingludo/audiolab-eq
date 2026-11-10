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
//
// 3.0 : écart à l'analogique < 0,1 dB de 20 Hz à 20 kHz, à toutes les fréquences d'échantillonnage.
// Une seule section d'ordre 2 par section analogique ne suffit pas près de Nyquist : quand l'écart de la
// conception matched dépasse 0,015 dB, la bande reçoit 1 à 3 sections SVF de correction (phase minimale,
// gain 1 en continu, aucune latence) ajustées sur le module, en même temps que les sections de base des
// cloches, étagères et coupe-haut (conception directe du module, départ = conception matched).
// Voir matched::correct. Une bande loin de Nyquist garde exactement sa conception 2.0.
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

// Nombre maximal de sections d'une bande : 8 SVF + 1er ordre éventuel (coupes), plus les sections
// de correction du mode zéro latence (voir matched::correct).
constexpr int maxCorrections = 3;
constexpr int maxSections = 9 + maxCorrections;

struct Design
{
    std::array<Section, maxSections> s {};
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

// Réglages des sections de correction (voir plus bas). Les écarts sont des écarts de puissance
// relatifs, mesurés sur les points de calcul : 0,0023 ≈ 0,01 dB.
constexpr double corrThreshold = 0.0035;        // (≈ 0,015 dB) au-dessus : la bande reçoit une section de correction
constexpr double corrThreshold2 = 0.0046;       // (≈ 0,02 dB) au-dessus : une section de correction de plus
constexpr double corrThresholdTrack = 0.0058;   // (≈ 0,025 dB) suivi d'un réglage qui bouge : au-dessus, recherche complète
constexpr double corrThresholdKeep = 0.0116;    // (≈ 0,05 dB) au-dessus, la solution suivie est abandonnée (nouveau départ)
constexpr int corrIterations = 40;              // passes au plus d'une recherche complète
constexpr int corrHintIterations = 10;          // passes au plus d'un suivi
constexpr int corrStarts = 2;                   // points de départ essayés pour une nouvelle section
constexpr double corrMaxQ = 8.0;                // Q maximal des pôles et des zéros d'une section de correction
constexpr double corrPullMin = 1e-7, corrPullNear = 1e-2, corrPullFar = 1e-6;   // rappel vers la conception sans correction

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

// Biquad numérique b0 + b1 z⁻¹ + b2 z⁻² / 1 + a1 z⁻¹ + a2 z⁻²
struct Bq { double b[3] = { 1, 0, 0 }; double a1 = 0, a2 = 0; };

// Une section analogique → un biquad numérique (vrai si réussi)
inline bool sectionBiquad (const ASec& a, Kind kind, double Wz, double Qz, double fs, Bq& out)
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
    out.b[0] = b[0]; out.b[1] = b[1]; out.b[2] = b[2]; out.a1 = ba1; out.a2 = ba2;
    return true;
}

// Biquad → section SVF ; invert : H⁻¹ = A/B (B est à phase minimale : ses zéros deviennent des pôles stables)
inline bool toSection (const Bq& q, bool invert, Section& out)
{
    if (invert)
    {
        if (! (std::abs (q.b[0]) > 1e-300) || std::abs (q.b[2] / q.b[0]) >= 0.999999) return false;
        const double s = 1.0 / q.b[0];
        const double nb[3] = { s, q.a1 * s, q.a2 * s };
        return svfFromBiquad (nb, q.b[1] * s, q.b[2] * s, out);
    }
    return svfFromBiquad (q.b, q.a1, q.a2, out);
}

// Une section analogique → une section SVF numérique (vrai si réussi)
inline bool section (const ASec& a, Kind kind, double Wz, double Qz, double fs, bool invert, Section& out)
{
    Bq q;
    return sectionBiquad (a, kind, Wz, Qz, fs, q) && toSection (q, invert, out);
}

// ── Sections de correction (3.0) ─────────────────────────────────────────
// Une section numérique d'ordre 2 ne peut pas suivre exactement le module analogique jusqu'à 20 kHz
// quand la bande agit près de Nyquist (ou quand ses pôles sont très amortis). On mesure donc le résidu
// R(ω) = |cible analogique|² / |filtre obtenu|² de la bande entière, et s'il dépasse le seuil on ajoute
// une section correctrice C telle que |C|² ≈ R. C est décrite directement dans le domaine du SVF
// (s = j·tan(ω/2)) par une paire de pôles (g, k) et une paire de zéros (gz, kz), tous libres :
//   C(s) = ((s/gz)² + kz·s/gz + 1) / ((s/g)² + k·s/g + 1)
// g, k, gz, kz > 0 : toujours stable, à phase minimale, gain exactement 1 en continu, et son inverse
// (pôles ↔ zéros) l'est aussi. Ajustement non linéaire (Levenberg-Marquardt sur les logarithmes des
// quatre paramètres, erreur relative de puissance) avec des poids de Lawson (vers le minimax).
struct Corr { double g = 1, k = 1.4, gz = 1, kz = 1.4; };

struct CGrid
{
    static constexpr int fixedMax = 48, maxPts = 76;
    static constexpr double aboveWeight = 0.05;
    double fs = 0, wAudible = pi, top = pi;
    int n = 0;
    double w[fixedMax] {}, wt[fixedMax] {};

    void init (double rate)
    {
        fs = rate; n = 0;
        const double c = 2.0 * pi / fs;
        wAudible = c * 20000.0;
        top = 0.998 * pi;
        for (int i = 0; i < 8; ++i) { w[n] = c * 20.0 * std::pow (100.0, i / 8.0); wt[n++] = 1.0; }    // 20 Hz … 2 kHz
        for (int i = 0; i < 10; ++i) { w[n] = c * (2000.0 + 1000.0 * i); wt[n++] = 1.0; }              // 2 … 11 kHz
        for (int i = 0; i <= 18; ++i) { w[n] = c * (11000.0 + 500.0 * i); wt[n++] = 1.0; }             // 11 … 20 kHz
        // au-dessus de 20 kHz : la courbe reste sage, sans gêner la précision dans l'audible
        for (int i = 1; i <= 4; ++i) { w[n] = wAudible + (top - wAudible) * i / 4.0; wt[n++] = aboveWeight; }
    }
};

inline const CGrid& cgridFor (double fs)
{
    static thread_local CGrid g;
    if (g.fs != fs) g.init (fs);
    return g;
}

struct CPts
{
    int n = 0;
    double w[CGrid::maxPts], wt[CGrid::maxPts], R[CGrid::maxPts], t2[CGrid::maxPts];   // t2 = tan²(ω/2)
};

inline double bqMag2 (const Bq& q, const Phi& p)
{
    const double s1 = q.b[0] + q.b[1] + q.b[2], s2 = q.b[0] - q.b[1] + q.b[2];
    const double t1 = 1.0 + q.a1 + q.a2, t2 = 1.0 - q.a1 + q.a2;
    const double num = s1 * s1 * p.p0 + s2 * s2 * p.p1 - 4.0 * q.b[0] * q.b[2] * p.p2;
    const double den = t1 * t1 * p.p0 + t2 * t2 * p.p1 - 4.0 * q.a2 * p.p2;
    return num / std::max (1e-300, den);
}

inline double corrMag2 (const Corr& c, double t2)
{
    const double v = t2 / (c.g * c.g), vz = t2 / (c.gz * c.gz);
    return ((1.0 - vz) * (1.0 - vz) + c.kz * c.kz * vz) / ((1.0 - v) * (1.0 - v) + c.k * c.k * v);
}

// gain : gain en continu de la section (1 pour une correction)
inline void corrToSection (const Corr& c, bool invert, double gain, Section& s)
{
    const double g = invert ? c.gz : c.g, k = invert ? c.kz : c.k, gz = invert ? c.g : c.gz, kz = invert ? c.k : c.kz;
    const double r = g / gz, G = invert ? 1.0 / gain : gain;
    s.kind = Section::svf; s.g = g; s.k = k;
    s.m0 = G * r * r; s.m1 = G * (kz * r - k * r * r); s.m2 = G * (1.0 - r * r);
}

// Résolution de M x = r par Cholesky (M symétrique définie positive, n ≤ 12). Faux si échec.
constexpr int maxFreeBase = 3;   // sections de base réajustées avec les corrections
constexpr int fitMaxParams = 4 * (maxCorrections + maxFreeBase);
inline bool cholSolve (double M[fitMaxParams][fitMaxParams], double* r, int n)
{
    for (int j = 0; j < n; ++j)
    {
        double d = M[j][j];
        for (int k = 0; k < j; ++k) d -= M[j][k] * M[j][k];
        if (! (d > 1e-300)) return false;
        d = std::sqrt (d);
        M[j][j] = d;
        for (int i = j + 1; i < n; ++i)
        {
            double v = M[i][j];
            for (int k = 0; k < j; ++k) v -= M[i][k] * M[j][k];
            M[i][j] = v / d;
        }
    }
    for (int i = 0; i < n; ++i) { double v = r[i]; for (int k = 0; k < i; ++k) v -= M[i][k] * r[k]; r[i] = v / M[i][i]; }
    for (int i = n - 1; i >= 0; --i) { double v = r[i]; for (int k = i + 1; k < n; ++k) v -= M[k][i] * r[k]; r[i] = v / M[i][i]; }
    return true;
}

// Écart d'un rapport de puissances à 1, symétrique (0,1 dB ↔ 0,0233)
inline double ratioErr (double r) { return r >= 1.0 ? r - 1.0 : 1.0 / std::max (1e-300, r) - 1.0; }

// Ajuste ensemble m sections c[0..m) (départ → résultat) pour que le produit de leurs |C|² suive le
// résidu p.R. wide[i] : section de base (bornes larges) ; sinon section de correction (Q borné).
// Renvoie l'écart max pondéré.
// stopAt : écart sous lequel on s'arrête.
// pull : rappel (poids) vers « sections de base = ctr (conception matched), corrections neutres
// (zéros = pôles) ». Fort quand l'écart à corriger est petit, faible quand il est grand : la solution
// part continûment de la conception sans correction, ce qui permet d'interpoler les coefficients quand
// une correction apparaît ou disparaît en cours de réglage. Il lève aussi l'indétermination des
// sections presque neutres.
// track : suivi d'une solution voisine (réglage qui bouge). Pas de recherche du minimax par étapes :
// les poids viennent une fois pour toutes de l'écart au point de départ, et l'on fait toujours au
// moins un pas, pour que la solution glisse avec les réglages au lieu d'avancer par à-coups.
inline double fitSections (const CPts& p, Corr* c, const bool* wide, int m, int maxIt, double stopAt,
                           const Corr* ctr, double pull, bool track, int* passes = nullptr)
{
    constexpr int NP = fitMaxParams;
    const int np = 4 * m;
    double lo[NP], hi[NP], th[NP];
    for (int s = 0; s < m; ++s)
    {
        const double kLo = wide[s] ? 1e-4 : 1.0 / corrMaxQ, kHi = wide[s] ? 1e4 : 6.0;
        lo[4 * s] = lo[4 * s + 2] = std::log (1e-4); hi[4 * s] = hi[4 * s + 2] = std::log (1e4);
        lo[4 * s + 1] = lo[4 * s + 3] = std::log (kLo); hi[4 * s + 1] = hi[4 * s + 3] = std::log (kHi);
        th[4 * s] = std::log (c[s].g); th[4 * s + 1] = std::log (c[s].k); th[4 * s + 2] = std::log (c[s].gz); th[4 * s + 3] = std::log (c[s].kz);
    }
    double th0[NP], thc[NP];
    for (int a = 0; a < np; ++a) th0[a] = thc[a] = th[a] = std::clamp (th[a], lo[a], hi[a]);
    for (int s = 0; s < m; ++s)
        if (wide[s] && ctr != nullptr)
        {
            thc[4 * s] = std::log (ctr[s].g); thc[4 * s + 1] = std::log (ctr[s].k); thc[4 * s + 2] = std::log (ctr[s].gz); thc[4 * s + 3] = std::log (ctr[s].kz);
        }
    int nPass = 0;
    double lw[CGrid::maxPts], e[CGrid::maxPts];
    for (int i = 0; i < p.n; ++i) lw[i] = 1.0;

    // une passe : coût, écart max, équations normales (triangle inférieur)
    auto pass = [&] (const double* t, double (*Nn)[NP], double* r, double& emax)
    {
        double ig2[NP / 4], k2[NP / 4], igz2[NP / 4], kz2[NP / 4];
        for (int s = 0; s < m; ++s) { ig2[s] = std::exp (-2.0 * t[4 * s]); k2[s] = std::exp (2.0 * t[4 * s + 1]); igz2[s] = std::exp (-2.0 * t[4 * s + 2]); kz2[s] = std::exp (2.0 * t[4 * s + 3]); }
        double cost = 0; emax = 0;
        ++nPass;
        for (int a = 0; a < np; ++a)
        {
            for (int b = 0; b < a; ++b) Nn[a][b] = 0;
            const double dt = t[a] - th0[a];
            Nn[a][a] = corrPullMin; r[a] = corrPullMin * dt; cost += corrPullMin * dt * dt;
        }
        for (int s = 0; s < m; ++s)
        {
            if (wide[s])
                for (int a = 4 * s; a < 4 * s + 4; ++a) { const double dt = t[a] - thc[a]; Nn[a][a] += pull; r[a] += pull * dt; cost += pull * dt * dt; }
            else
                for (int a = 4 * s; a < 4 * s + 2; ++a)
                {
                    const double dt = t[a + 2] - t[a];   // ln (gz / g), ln (kz / k)
                    Nn[a][a] += pull; Nn[a + 2][a + 2] += pull; Nn[a + 2][a] -= pull;
                    r[a] -= pull * dt; r[a + 2] += pull * dt; cost += pull * dt * dt;
                }
        }
        for (int i = 0; i < p.n; ++i)
        {
            if (! (p.wt[i] > 0)) { e[i] = 0; continue; }
            double q = 1.0 / p.R[i], J[NP];
            for (int s = 0; s < m; ++s)
            {
                const double v = p.t2[i] * ig2[s], vz = p.t2[i] * igz2[s];
                const double f = (1.0 - v) * (1.0 - v) + k2[s] * v, fz = (1.0 - vz) * (1.0 - vz) + kz2[s] * vz;
                const double invf = 1.0 / f, invfz = 1.0 / fz;
                q *= fz * invf;
                J[4 * s] = -2.0 * v * (2.0 * (1.0 - v) - k2[s]) * invf; J[4 * s + 1] = -2.0 * k2[s] * v * invf;
                J[4 * s + 2] = 2.0 * vz * (2.0 * (1.0 - vz) - kz2[s]) * invfz; J[4 * s + 3] = 2.0 * kz2[s] * vz * invfz;
            }
            // écart symétrique ≈ ln q, sans logarithme : 2 (q − 1)/(q + 1)
            const double iq = 1.0 / (q + 1.0);
            const double res = 2.0 * (q - 1.0) * iq, dq = 4.0 * q * iq * iq;
            e[i] = p.wt[i] * std::abs (res);
            emax = std::max (emax, e[i]);
            const double om = p.wt[i] * lw[i], y = om * res, oj = om * dq;
            cost += y * y;
            for (int a = 0; a < np; ++a) { J[a] *= oj; r[a] += J[a] * y; for (int b = 0; b <= a; ++b) Nn[a][b] += J[a] * J[b]; }
        }
        return cost;
    };

    double N[NP][NP], rhs[NP];
    double emax = 0, best = 1e300, bestTh[NP];
    auto keep = [&] { if (emax < best) { best = emax; for (int a = 0; a < np; ++a) bestTh[a] = th[a]; } };
    double cost = pass (th, N, rhs, emax);
    double lambda = 1e-2;
    int sinceLawson = 0, lawsonRounds = 0;
    if (track)
    {
        if (emax > 0)
        {
            for (int i = 0; i < p.n; ++i) { const double x = e[i] / emax; lw[i] = x * x + 0.1; }
            cost = pass (th, N, rhs, emax);
        }
        lambda = 1e-3;
        for (int it = 0; it < maxIt; ++it)
        {
            double M[NP][NP], dlt[NP];
            for (int a = 0; a < np; ++a)
            {
                for (int b = 0; b < a; ++b) M[a][b] = N[a][b];
                M[a][a] = N[a][a] * (1.0 + lambda) + 1e-14;
                dlt[a] = -rhs[a];
            }
            if (! cholSolve (M, dlt, np)) { lambda *= 10.0; if (lambda > 1e8) break; continue; }
            double tn[NP], N2[NP][NP], r2[NP], em2 = 0;
            for (int a = 0; a < np; ++a) tn[a] = std::clamp (th[a] + std::clamp (dlt[a], -1.0, 1.0), lo[a], hi[a]);
            const double c2 = pass (tn, N2, r2, em2);
            if (c2 < cost)
            {
                const bool done = c2 > 0.97 * cost;
                for (int a = 0; a < np; ++a) { th[a] = tn[a]; rhs[a] = r2[a]; for (int b = 0; b <= a; ++b) N[a][b] = N2[a][b]; }
                cost = c2; emax = em2; lambda = std::max (1e-7, lambda * 0.3);
                if (done || emax < stopAt) break;
            }
            else { lambda *= 6.0; if (lambda > 1e6) break; }
        }
        if (passes != nullptr) *passes += nPass;
        for (int s = 0; s < m; ++s) { c[s].g = std::exp (th[4 * s]); c[s].k = std::exp (th[4 * s + 1]); c[s].gz = std::exp (th[4 * s + 2]); c[s].kz = std::exp (th[4 * s + 3]); }
        return emax;
    }
    for (int it = 0; it < maxIt; ++it)
    {
        keep();
        if (best < stopAt) break;
        double M[NP][NP], dlt[NP];
        for (int a = 0; a < np; ++a)
        {
            for (int b = 0; b < a; ++b) M[a][b] = N[a][b];
            M[a][a] = N[a][a] * (1.0 + lambda) + 1e-14;
            dlt[a] = -rhs[a];
        }
        if (! cholSolve (M, dlt, np)) { lambda *= 10.0; if (lambda > 1e8) break; continue; }
        double tn[NP], N2[NP][NP], r2[NP], em2 = 0;
        for (int a = 0; a < np; ++a) tn[a] = std::clamp (th[a] + std::clamp (dlt[a], -1.0, 1.0), lo[a], hi[a]);
        const double c2 = pass (tn, N2, r2, em2);
        const bool better = c2 < cost;
        const bool stalled = better && c2 > 0.998 * cost;
        if (better)
        {
            for (int a = 0; a < np; ++a) { th[a] = tn[a]; rhs[a] = r2[a]; for (int b = 0; b <= a; ++b) N[a][b] = N2[a][b]; }
            cost = c2; emax = em2; lambda = std::max (1e-7, lambda * 0.3);
        }
        else lambda *= 6.0;
        ++sinceLawson;
        if ((stalled || lambda > 1e5 || sinceLawson >= 10) && it < maxIt - 2)
        {
            // poids de Lawson : les points où l'écart est le plus grand pèsent davantage
            keep();
            if (++lawsonRounds > 4) break;
            pass (th, N, rhs, emax);   // (e[] du point courant)
            double mx = 0;
            for (int i = 0; i < p.n; ++i) { lw[i] *= std::max (e[i], 1e-9); mx = std::max (mx, lw[i]); }
            for (int i = 0; i < p.n; ++i) lw[i] = lw[i] / mx + 0.03;
            cost = pass (th, N, rhs, emax);
            lambda = 1e-2; sinceLawson = 0;
        }
    }
    keep();
    if (passes != nullptr) *passes += nPass;
    for (int s = 0; s < m; ++s) { c[s].g = std::exp (bestTh[4 * s]); c[s].k = std::exp (bestTh[4 * s + 1]); c[s].gz = std::exp (bestTh[4 * s + 2]); c[s].kz = std::exp (bestTh[4 * s + 3]); }
    return best;
}

inline int& statPasses() { static thread_local int n = 0; return n; }   // (mesure du coût : passes d'ajustement)

// Section d'une conception existante → (g, k, gz, kz) de la version non inversée. Faux si impossible.
inline bool fromHint (const Section& s, bool invert, Corr& c)
{
    if (s.kind != Section::svf) return false;
    const double n2 = s.m0, n1 = s.m0 * s.k + s.m1, n0 = s.m0 + s.m2;
    if (! (n2 > 1e-12 && n1 > 1e-12 && n0 > 1e-12 && s.g > 0 && s.k > 0)) return false;
    c.g = s.g; c.k = s.k; c.gz = s.g * std::sqrt (n0 / n2); c.kz = n1 / std::sqrt (n0 * n2);
    if (invert) { std::swap (c.g, c.gz); std::swap (c.k, c.kz); }
    return std::isfinite (c.g + c.k + c.gz + c.kz);
}

// Zone utile des formes qui atténuent : sous ce niveau (puissance), l'écart relatif compte de moins en moins
enum class Soft { none, cut, notch };

// Résultat de la correction d'une bande
struct Corrected
{
    bool replaced[maxSections] {};   // section de base réajustée avec les corrections
    Corr base[maxSections]; double baseGain[maxSections] {};
    int n = 0;
    Corr c[maxCorrections];
};

// Calcule jusqu'à maxCorr sections de correction pour la bande bq[0..n). feat : (W, Q) des résonances
// et des zéros. freeBase : les sections de base (les maxFreeBase plus résonantes) sont réajustées en
// même temps que les corrections : conception directe du module, en partant de la conception matched.
// Les sections à zéros imposés (coupe-bas, passe-bande, notch) ne sont jamais réajustées.
// hint : sections d'une conception voisine de la même bande (réglages proches), pour repartir de sa
// solution : quelques passes suffisent, et la solution suit continûment les réglages.
inline void correct (const ASec* as, int na, const double (*feat)[2], int nfeat, Soft soft, double fs,
                     const Bq* bq, int n, int maxCorr, bool freeBase, bool invert, const Section* hint, int hintN,
                     int cutSide, int cutOrder, Corrected& out)
{
    out.n = 0;
    for (int i = 0; i < n; ++i) out.replaced[i] = false;
    if (maxCorr <= 0) return;
    const CGrid& g = cgridFor (fs);
    CPts p;
    p.n = g.n;
    for (int i = 0; i < g.n; ++i) { p.w[i] = g.w[i]; p.wt[i] = g.wt[i]; }
    const double wLo = 2.0 * pi * 20.0 / fs;
    for (int f = 0; f < nfeat; ++f)
    {
        const double W = feat[f][0], d = 0.5 / std::max (0.5, feat[f][1]);
        if (! (W > 0)) continue;
        for (double m : { -1.0, -0.5, 0.0, 0.5, 1.0 })
        {
            const double w = W * std::pow (2.0, m * d);
            if (w > wLo && w < g.top && p.n < CGrid::maxPts)
            {
                p.w[p.n] = w; p.wt[p.n] = w <= g.wAudible ? 1.0 : CGrid::aboveWeight;
                ++p.n;
            }
        }
    }
    if (soft == Soft::cut && cutOrder > 0 && nfeat > 0)
    {
        // coupes, passe-bande : la pente jusqu'au bas de la zone utile (−27 dB) est échantillonnée de près
        // (cutSide : +1 coupe-haut, −1 coupe-bas, 0 les deux côtés)
        const double span = std::pow (10.0, 2.7 / (2.0 * cutOrder));
        for (int side : { -1, 1 })
        {
            // côté atténué : 6 points jusqu'à −27 dB ; côté passant d'une coupe : 4 points sur une demi-octave
            const bool stopSide = cutSide == 0 || side == cutSide;
            for (int j = 1; j <= (stopSide ? 6 : 4); ++j)
            {
                const double w = feat[0][0] * (stopSide ? std::pow (span, side * j / 6.0) : std::pow (2.0, side * j / 8.0));
                if (w > wLo && w < g.top && p.n < CGrid::maxPts)
                {
                    p.w[p.n] = w; p.wt[p.n] = w <= g.wAudible ? 1.0 : CGrid::aboveWeight;
                    ++p.n;
                }
            }
        }
    }
    const double floorP = soft == Soft::cut ? 2.0e-3 : 0.25;   // −27 dB (coupes, passe-bande) ; −6 dB (notch)
    double e0 = 0, T[CGrid::maxPts];
    Phi ph[CGrid::maxPts];
    int worst = 0;
    for (int i = 0; i < p.n; ++i)
    {
        ph[i] = phi (p.w[i]);
        p.t2[i] = ph[i].p1 / ph[i].p0;
        double D = 1.0;
        T[i] = 1.0;
        for (int j = 0; j < na; ++j) T[i] *= mag2 (as[j], p.w[i]);
        for (int j = 0; j < n; ++j) D *= bqMag2 (bq[j], ph[i]);
        if (! (T[i] > 1e-200 && D > 1e-200 && std::isfinite (T[i]) && std::isfinite (D))) { p.R[i] = 1.0; T[i] = 0.0; p.wt[i] = 0.0; continue; }
        if (soft != Soft::none && T[i] < floorP) p.wt[i] *= T[i] / floorP;
        p.R[i] = T[i] / D;
        const double e = p.wt[i] * ratioErr (p.R[i]);
        if (e > e0) { e0 = e; worst = i; }
    }
    if (e0 <= corrThreshold) return;

    // sections de base réajustées : mises sous la forme (g, k, gz, kz, gain)
    int freeIdx[maxFreeBase], nb = 0;
    Corr cur[maxFreeBase + maxCorrections];
    double gain[maxFreeBase];
    bool wide[maxFreeBase + maxCorrections];
    if (freeBase)
    {
        Corr rep[maxSections]; double rg[maxSections]; bool ok[maxSections];
        for (int i = 0; i < n; ++i)
        {
            Section sb;
            ok[i] = toSection (bq[i], false, sb);
            if (! ok[i]) continue;
            const double n2 = sb.m0, n1 = sb.m0 * sb.k + sb.m1, n0 = sb.m0 + sb.m2;
            ok[i] = n2 > 1e-9 * n0 && n1 > 1e-9 * n0 && n0 > 1e-12;
            if (ok[i]) { rep[i].g = sb.g; rep[i].k = sb.k; rep[i].gz = sb.g * std::sqrt (n0 / n2); rep[i].kz = n1 / std::sqrt (n0 * n2); rg[i] = n0; }
            else if (n == 1 && as[0].svf && as[0].m0 > 0 && as[0].m0 + as[0].m2 > 0 && as[0].m0 * as[0].k + as[0].m1 > 0)
            {
                // numérateur matched inutilisable (zéro sur le cercle) : pôles matched, zéros de l'analogique
                // reportés à leur fréquence (étagères très résonantes près de Nyquist)
                const auto& a = as[0];
                const double Wz = a.W * std::sqrt ((a.m0 + a.m2) / a.m0), kz = (a.m0 * a.k + a.m1) / std::sqrt (a.m0 * (a.m0 + a.m2));
                rep[i].g = sb.g; rep[i].k = sb.k;
                rep[i].gz = Wz < 0.98 * pi ? std::tan (0.5 * Wz) : std::tan (0.49 * pi) * Wz / (0.98 * pi);
                rep[i].kz = kz * sb.k / a.k; rg[i] = a.m0 + a.m2;
                ok[i] = true;
            }
        }
        while (nb < maxFreeBase)
        {
            int bi = -1;   // la plus résonante parmi celles qui restent
            for (int i = 0; i < n; ++i)
            {
                bool used = ! ok[i];
                for (int j = 0; j < nb; ++j) used = used || freeIdx[j] == i;
                if (! used && (bi < 0 || rep[i].k < rep[bi].k)) bi = i;
            }
            if (bi < 0) break;
            freeIdx[nb] = bi; cur[nb] = rep[bi]; gain[nb] = rg[bi]; ++nb;
        }
        // cible des sections libres : analogique / (sections figées × gains)
        e0 = 0;
        for (int i = 0; i < p.n; ++i)
        {
            if (! (p.wt[i] > 0)) continue;
            double D = 1.0;
            for (int j = 0; j < n; ++j)
            {
                bool isFree = false;
                for (int f = 0; f < nb; ++f) isFree = isFree || freeIdx[f] == j;
                if (! isFree) D *= bqMag2 (bq[j], ph[i]);
            }
            for (int f = 0; f < nb; ++f) D *= gain[f] * gain[f];
            p.R[i] = T[i] / D;
            double q = 1.0 / p.R[i];
            for (int f = 0; f < nb; ++f) q *= corrMag2 (cur[f], p.t2[i]);
            const double e = p.wt[i] * ratioErr (q);
            if (e > e0) { e0 = e; worst = i; }
        }
    }
    for (int i = 0; i < maxFreeBase + maxCorrections; ++i) wide[i] = i < nb;
    // rappel vers la conception sans correction : fort juste au-dessus du seuil, faible ensuite
    const double x = corrThreshold / e0;
    const double pull = std::max (corrPullFar, corrPullNear * x * x * x);
    Corr ctr[maxFreeBase + maxCorrections];
    for (int f = 0; f < nb; ++f) ctr[f] = cur[f];
    int m = nb;          // sections actuellement ajustées (base comprise)
    bool tracked = false;
    if (hint != nullptr && hintN > n && hintN - n <= maxCorr)
    {
        const int mh = hintN - n;
        Corr t[maxFreeBase + maxCorrections];
        bool okH = true;
        for (int f = 0; f < nb; ++f) if (! fromHint (hint[freeIdx[f]], invert, t[f])) t[f] = cur[f];
        for (int j = 0; j < mh; ++j) okH = okH && fromHint (hint[n + j], invert, t[nb + j]);
        if (okH)
        {
            double e = fitSections (p, t, wide, nb + mh, corrHintIterations, 0.5 * corrThreshold, ctr, pull, true, &statPasses());
            // le suivi ne suffit plus : recherche complète à partir de la solution suivie (elle reste voisine)
            if (e > corrThresholdTrack) e = fitSections (p, t, wide, nb + mh, corrIterations / 2, corrThreshold2, ctr, pull, false, &statPasses());
            if (e <= corrThresholdKeep)
            {
                // (si l'écart reste au-dessus du seuil de suivi, la boucle ci-dessous ajoute une section à cette solution)
                m = nb + mh; e0 = e; tracked = true;
                for (int i = 0; i < m; ++i) cur[i] = t[i];
            }
        }
    }
    const double eStart = e0;
    const int mStart = m;
    while (m - nb < maxCorr && e0 > (m == nb ? corrThreshold : tracked ? corrThresholdTrack : corrThreshold2))
    {
        // on ajoute une section neutre (là où l'écart est le plus grand, ou tout en haut du spectre) et on réajuste tout
        Corr bestC[maxFreeBase + maxCorrections]; double e1 = 1e300;
        for (int start = 0; start < corrStarts; ++start)
        {
            Corr t[maxFreeBase + maxCorrections];
            for (int i = 0; i < m; ++i) t[i] = cur[i];
            t[m] = Corr();
            t[m].g = t[m].gz = start == 0 ? std::sqrt (std::max (1e-12, p.t2[worst])) : std::tan (0.47 * pi);
            const double e = fitSections (p, t, wide, m + 1, corrIterations, corrThreshold, ctr, pull, false, &statPasses());
            if (e < e1) { e1 = e; for (int i = 0; i <= m; ++i) bestC[i] = t[i]; }
            if (e1 < corrThreshold2) break;
        }
        if (! (e1 < e0)) break;
        ++m;
        for (int i = 0; i < m; ++i) cur[i] = bestC[i];
        e0 = e1;
        // nouveau pire point (pour placer la section suivante)
        double ew = 0;
        for (int i = 0; i < p.n; ++i)
        {
            double q = 1.0 / p.R[i];
            for (int j = 0; j < m; ++j) q *= corrMag2 (cur[j], p.t2[i]);
            const double e = p.wt[i] * ratioErr (q);
            if (e > ew) { ew = e; worst = i; }
        }
    }
    // (un résidu qui oscille demande parfois deux sections d'un coup : on juge le gain à la fin)
    if (m > mStart && ! tracked && ! (e0 < 0.8 * eStart)) return;
    if (m == nb) return;
    for (int f = 0; f < nb; ++f) { out.replaced[freeIdx[f]] = true; out.base[freeIdx[f]] = cur[f]; out.baseGain[freeIdx[f]] = gain[f]; }
    out.n = m - nb;
    for (int i = 0; i < out.n; ++i) out.c[i] = cur[nb + i];
}
} // namespace matched

// Conception numérique « matched » (mode zéro latence, affichage, gain auto).
// maxCorr : nombre maximal de sections de correction (0 = conception 2.0, pour les mesures « avant »).
// hint : conception précédente de la même bande (mêmes forme et pente, réglages voisins), ou nul.
inline void designMatched (const BandSpec& b, double fs, Design& d, int maxCorr = maxCorrections, const Design* hint = nullptr)
{
    Section hs[maxSections];
    const int hintN = hint != nullptr ? hint->n : 0;
    for (int i = 0; i < hintN; ++i) hs[i] = hint->s[(size_t) i];
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

    matched::ASec as[maxSections];
    matched::Bq bq[maxSections];
    double feat[4][2];
    int n = 0, nfeat = 0;
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
            // (la version conçue est toujours l'atténuation : ses zéros sont au-dessus des pôles, en W·A.
            //  La 2.0 les cherchait en W/A ; on garde ce repère pour maxCorr = 0, qui sert de mesure « avant ».)
            case Shape::tiltShelf: Wz = maxCorr > 0 ? a.W * A : a.W / A; Qz = 1.0 / a.k; break;
            case Shape::lowCut:    kind = matched::Kind::highpass; break;
            case Shape::highCut:   kind = matched::Kind::lowpassLike; break;
            case Shape::bandPass:  kind = matched::Kind::bandpass; break;
            case Shape::notch:     kind = matched::Kind::notch; break;
        }
        if (! matched::sectionBiquad (a, kind, Wz, Qz, fs, bq[n]))
        {
            // secours (ne devrait pas arriver) : conception bilinéaire de cette bande
            designClassic (b, fs, d, false);
            return;
        }
        as[n++] = a;
        // repères pour la correction : résonance la plus marquée et zéros (une seule fréquence de coupure par bande)
        if (i == ad.n - 1)
        {
            feat[nfeat][0] = a.W; feat[nfeat++][1] = a.svf ? 1.0 / a.k : 0.5;
            if (hasGain (b.shape) && a.svf) { feat[nfeat][0] = Wz; feat[nfeat++][1] = Qz; }
        }
    }
    // sections de correction : ajustées sur la version non inversée, puis inversées avec le reste
    // (une atténuation reste l'inverse exact de l'accentuation)
    const int nBase = n;
    const auto soft = b.shape == Shape::notch ? matched::Soft::notch
                    : (isCut (b.shape) || b.shape == Shape::bandPass) ? matched::Soft::cut : matched::Soft::none;
    matched::Corrected cr;
    matched::correct (as, nBase, feat, nfeat, soft, fs, bq, n, std::min (maxCorr, std::min (maxCorrections, maxSections - nBase)),
                      hasGain (b.shape) || b.shape == Shape::highCut, invert, hintN > 0 ? hs : nullptr, hintN,
                      b.shape == Shape::highCut ? 1 : b.shape == Shape::lowCut ? -1 : 0,
                      isCut (b.shape) ? std::clamp (b.slope / 6, 1, 16) : b.shape == Shape::bandPass ? std::clamp (b.slope / 12, 1, 8) : 0, cr);

    for (int i = 0; i < n; ++i)
    {
        Section s;
        if (cr.replaced[i]) matched::corrToSection (cr.base[i], invert, cr.baseGain[i], s);
        else if (! matched::toSection (bq[i], invert, s))
        {
            designClassic (b, fs, d, false);   // secours (ne devrait pas arriver)
            return;
        }
        d.s[(size_t) d.n++] = s;
    }
    for (int i = 0; i < cr.n; ++i) matched::corrToSection (cr.c[i], invert, 1.0, d.s[(size_t) d.n++]);
    if (b.shape == Shape::tiltShelf) d.outGain = std::pow (10.0, -b.gainDb / 40.0);
}

// Écart max d'une conception numérique d au prototype analogique de b, de 20 Hz à 20 kHz, sur les
// points de calcul de la correction (rapport de puissances − 1 : 0,0023 ≈ 0,01 dB). Sert à vérifier
// qu'une conception interpolée entre deux autres (bandes dynamiques) reste exacte.
inline double designError (const BandSpec& b, double fs, const Design& d)
{
    BandSpec proto = b;
    proto.freq = std::clamp (b.freq, 5.0, 0.49 * fs);
    Design ad;
    designClassic (proto, fs, ad, true);
    const matched::CGrid& g = matched::cgridFor (fs);
    const double W = 2.0 * pi * proto.freq / fs, dq = 0.25 / std::max (0.5, b.q);
    const double extra[3] = { W * std::pow (2.0, -dq), W, W * std::pow (2.0, dq) };
    double e = 0;
    for (int i = 0; i < g.n + 3; ++i)
    {
        const double w = i < g.n ? g.w[i] : extra[i - g.n];
        if (! (w <= g.wAudible)) continue;
        double T = ad.outGain * ad.outGain, D = d.outGain * d.outGain;
        for (int j = 0; j < ad.n; ++j) T *= matched::mag2 (matched::fromSection (ad.s[(size_t) j]), w);
        const double tn = std::tan (0.5 * w);
        for (int j = 0; j < d.n; ++j)
        {
            matched::ASec a = matched::fromSection (d.s[(size_t) j]);
            a.W = d.s[(size_t) j].g;   // (domaine du SVF : s = j·tan(ω/2)/g)
            D *= matched::mag2 (a, tn);
        }
        if (T > 1e-200 && D > 1e-200) e = std::max (e, matched::ratioErr (D / T));
    }
    return e;
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
