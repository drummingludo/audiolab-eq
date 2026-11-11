// Audio Lab EQ — mesure de sonie ITU-R BS.1770-4 (sans JUCE).
//
// Pondération K : pré-filtre en étagère (+4 dB dans l'aigu) puis passe-haut « RLB ». La norme donne les
// coefficients à 48 kHz seulement ; ici les deux filtres sont recalculés pour la fréquence
// d'échantillonnage en cours à partir de leurs paramètres analogiques (fréquence, Q, gain), par la
// transformée bilinéaire avec précorrection de la fréquence. À 48 kHz, on retrouve les coefficients
// de la norme (vérifié par le banc qualité).
// Sonie : L = −0,691 + 10·log10 (Σ voies G·⟨y²⟩), G = 1 pour la gauche et la droite (LUFS).
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace audiolab::loud
{
constexpr double kPi = 3.14159265358979323846;
constexpr double absGateLufs = -70.0;      // porte absolue
constexpr double relGateLu = -10.0;        // porte relative (sonie intégrée)
constexpr double lufsOffset = -0.691;

struct Biquad
{
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    std::complex<double> at (double f, double fs) const
    {
        const std::complex<double> z1 = std::polar (1.0, -2.0 * kPi * f / fs), z2 = z1 * z1;
        return (b0 + b1 * z1 + b2 * z2) / (1.0 + a1 * z1 + a2 * z2);
    }
};

// Les deux filtres de la pondération K pour la fréquence d'échantillonnage fs
inline void kWeighting (double fs, Biquad& shelf, Biquad& highPass)
{
    {
        // étagère aiguë : f0 = 1681,97 Hz, gain +3,9998 dB, Q = 0,7072
        const double f0 = 1681.974450955533, G = 3.999843853973347, Q = 0.7071752369554196;
        const double K = std::tan (kPi * f0 / fs), Vh = std::pow (10.0, G / 20.0), Vb = std::pow (Vh, 0.4996667741545416);
        const double a0 = 1.0 + K / Q + K * K;
        shelf.b0 = (Vh + Vb * K / Q + K * K) / a0;
        shelf.b1 = 2.0 * (K * K - Vh) / a0;
        shelf.b2 = (Vh - Vb * K / Q + K * K) / a0;
        shelf.a1 = 2.0 * (K * K - 1.0) / a0;
        shelf.a2 = (1.0 - K / Q + K * K) / a0;
    }
    {
        // passe-haut RLB : f0 = 38,135 Hz, Q = 0,5003 (numérateur 1, −2, 1 comme dans la norme)
        const double f0 = 38.13547087602444, Q = 0.5003270373238773;
        const double K = std::tan (kPi * f0 / fs);
        const double a0 = 1.0 + K / Q + K * K;
        highPass.b0 = 1.0; highPass.b1 = -2.0; highPass.b2 = 1.0;
        highPass.a1 = 2.0 * (K * K - 1.0) / a0;
        highPass.a2 = (1.0 - K / Q + K * K) / a0;
    }
}

// Gain (dB) de la pondération K recalculée pour fs, à la fréquence f
inline double kGainDb (double f, double fs)
{
    Biquad s, h; kWeighting (fs, s, h);
    return 20.0 * std::log10 (std::max (1e-30, std::abs (s.at (f, fs) * h.at (f, fs))));
}

// Courbe K de référence : les coefficients de la norme (48 kHz), évalués à la fréquence f (< 24 kHz)
inline double kReferenceDb (double f)
{
    Biquad s, h;
    s.b0 = 1.53512485958697; s.b1 = -2.69169618940638; s.b2 = 1.19839281085285; s.a1 = -1.69065929318241; s.a2 = 0.73248077421585;
    h.b0 = 1.0; h.b1 = -2.0; h.b2 = 1.0; h.a1 = -1.99004745483398; h.a2 = 0.99007225036621;
    return 20.0 * std::log10 (std::max (1e-30, std::abs (s.at (f, 48000.0) * h.at (f, 48000.0))));
}

// Une voie de pondération K (deux biquads, forme directe II transposée, en double)
struct KFilter
{
    Biquad s, h;
    double z[4] {};
    void set (double fs) { kWeighting (fs, s, h); reset(); }
    void reset() { z[0] = z[1] = z[2] = z[3] = 0.0; }
    inline double tick (double x)
    {
        const double y = s.b0 * x + z[0];
        z[0] = s.b1 * x - s.a1 * y + z[1];
        z[1] = s.b2 * x - s.a2 * y;
        const double w = h.b0 * y + z[2];
        z[2] = h.b1 * y - h.a1 * w + z[3];
        z[3] = h.b2 * y - h.a2 * w;
        return w;
    }
};

inline double lufs (double meanSquare) { return lufsOffset + 10.0 * std::log10 (std::max (1e-30, meanSquare)); }

// Mesureur complet (sonie momentanée 400 ms, courte 3 s, intégrée avec les deux portes), pour les
// mesures hors temps réel et le banc de test : il garde l'énergie de chaque tranche de 100 ms
// (réserve faite dans prepare() ; au-delà, le vecteur grandit : ne pas l'utiliser dans le fil audio).
class Meter
{
public:
    void prepare (double sampleRate, double reserveSeconds = 600.0)
    {
        fs = sampleRate;
        hop = std::max (1, (int) std::lround (0.1 * fs));
        for (auto& k : kf) k.set (fs);
        hops.clear(); hops.reserve ((size_t) (reserveSeconds * 10.0) + 8);
        acc = 0.0; pos = 0;
    }
    // R nul : mono (une seule voie comptée)
    void process (const float* L, const float* R, int n)
    {
        for (int i = 0; i < n; ++i)
        {
            const double a = kf[0].tick (L[i]);
            acc += a * a;
            if (R != nullptr) { const double b = kf[1].tick (R[i]); acc += b * b; }
            if (++pos == hop) { hops.push_back (acc / hop); acc = 0.0; pos = 0; }
        }
    }
    // sonie des dernières tranches (4 = momentanée, 30 = courte)
    double recent (int nHops) const
    {
        if (hops.empty()) return -200.0;
        const int n = std::min (nHops, (int) hops.size());
        double s = 0; for (int i = 0; i < n; ++i) s += hops[hops.size() - 1 - (size_t) i];
        return lufs (s / n);
    }
    double momentary() const { return recent (4); }
    double shortTerm() const { return recent (30); }
    // Sonie intégrée : blocs de 400 ms (recouvrement 75 %), porte absolue −70 LUFS puis porte relative −10 LU
    double integrated() const
    {
        if (hops.size() < 4) return -200.0;
        double sum = 0; long cnt = 0;
        for (size_t i = 3; i < hops.size(); ++i)
        {
            const double b = 0.25 * (hops[i] + hops[i - 1] + hops[i - 2] + hops[i - 3]);
            if (lufs (b) > absGateLufs) { sum += b; ++cnt; }
        }
        if (cnt == 0) return -200.0;
        const double rel = lufs (sum / cnt) + relGateLu;
        sum = 0; cnt = 0;
        for (size_t i = 3; i < hops.size(); ++i)
        {
            const double b = 0.25 * (hops[i] + hops[i - 1] + hops[i - 2] + hops[i - 3]);
            if (lufs (b) > absGateLufs && lufs (b) > rel) { sum += b; ++cnt; }
        }
        return cnt > 0 ? lufs (sum / cnt) : -200.0;
    }

private:
    double fs = 48000.0;
    int hop = 4800, pos = 0;
    double acc = 0.0;
    KFilter kf[2];
    std::vector<double> hops;
};
} // namespace audiolab::loud
