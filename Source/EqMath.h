// Audio Lab EQ — calcul des filtres (conception + réponse en fréquence analytique).
// Les filtres sont des SVF « trapézoïdaux » (Andrew Simper, Cytomic) : stables quand on
// module la fréquence ou le gain à chaque bloc, ce qui permet l'EQ dynamique sans clics.
// La même description sert au traitement temps réel, à l'affichage de la courbe et à la
// conception des filtres FIR du mode phase linéaire : ce qu'on voit est ce qu'on entend.
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
// - mode numérique (temps réel, zéro latence) : tan (pi f / fs) — transformation bilinéaire
// - mode analogique (phase linéaire, affichage de ce mode) : pi f / fs, sans compression près de Nyquist
inline double warp (double f, double fs, bool analog)
{
    const double x = pi * f / fs;
    return analog ? x : std::tan (std::min (x, pi * 0.4999));
}

inline double clampFreq (double f, double fs, bool analog)
{
    return std::clamp (f, 5.0, analog ? 0.5 * fs * 1.5 : 0.4985 * fs);
}

inline void design (const BandSpec& b, double fs, Design& d, bool analog = false)
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
