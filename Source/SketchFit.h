// Audio Lab EQ — mode « Dessiner » : on trace la courbe voulue, des bandes sont ajustées
// automatiquement au relâchement (cloches, shelfs, coupe-bas / coupe-haut).
// Méthode : ajout glouton d'une bande à l'endroit de la plus grande erreur, puis
// optimisation locale de toutes les bandes (descente par coordonnées).
#pragma once
#include "EqMath.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace audiolab::eq
{
struct FitBand
{
    Shape shape = Shape::bell;
    double freq = 1000, gain = 0, q = 1;
    int slope = 12;
};

class SketchFitter
{
public:
    // target : dB voulus sur une grille log de fréquences (grid), weight : importance de chaque point.
    std::vector<FitBand> fit (const std::vector<double>& grid, const std::vector<double>& target,
                              const std::vector<double>& weight, double fs, int maxBandsToUse = 12)
    {
        G = grid; T = target; W = weight; fsr = fs;
        const size_t n = G.size();
        bands.clear(); resp.clear();
        total.assign (n, 0.0);

        for (int it = 0; it < maxBandsToUse; ++it)
        {
            if (error() < 0.35) break;
            std::vector<double> R (n);
            for (size_t i = 0; i < n; ++i) R[i] = T[i] - total[i];
            // point de plus grande erreur (pondérée)
            size_t idx = 0; double best = -1;
            for (size_t i = 0; i < n; ++i) { const double v = std::abs (R[i]) * (0.25 + W[i]); if (v > best) { best = v; idx = i; } }
            const double peak = R[idx];
            if (std::abs (peak) < 0.4) break;

            FitBand b;
            const double edge = 0.07 * (double) n;
            auto halfPoint = [&] (int dir)
            {
                long i = (long) idx;
                while (i + dir >= 0 && i + dir < (long) n && std::abs (R[(size_t) (i + dir)]) > std::abs (peak) * 0.5
                       && R[(size_t) (i + dir)] * peak > 0) i += dir;
                return (size_t) i;
            };
            if ((double) idx < edge || (double) idx > (double) n - 1 - edge)
            {
                const bool low = (double) idx < edge;
                const size_t h = halfPoint (low ? 1 : -1);
                if (peak < -18.0)
                {
                    b.shape = low ? Shape::lowCut : Shape::highCut;
                    // fréquence de coupure ≈ là où la courbe voulue passe -3 dB
                    size_t c = idx;
                    while (c + (low ? 1 : -1) < n && c > 0 && R[c] < -3.0) c = low ? c + 1 : c - 1;
                    b.freq = G[c]; b.q = 0.71; b.slope = 24; b.gain = 0;
                }
                else
                {
                    b.shape = low ? Shape::lowShelf : Shape::highShelf;
                    b.freq = G[h]; b.gain = peak; b.q = 0.71; b.slope = 12;
                }
            }
            else
            {
                const size_t lo = halfPoint (-1), hi = halfPoint (1);
                const double bw = std::max (0.15, std::log2 (G[hi] / G[lo]));
                b.shape = Shape::bell; b.freq = G[idx]; b.gain = peak; b.q = std::clamp (qFromOctaves (bw), 0.35, 12.0);
            }
            addBand (b);
            optimise (25);
        }
        optimise (40);
        // bandes inutiles
        std::vector<FitBand> out;
        for (auto& b : bands)
            if (isCut (b.shape) || std::abs (b.gain) >= 0.3) out.push_back (b);
        return out;
    }

private:
    void addBand (const FitBand& b)
    {
        bands.push_back (b);
        resp.emplace_back (G.size(), 0.0);
        evalBand (bands.size() - 1);
        recomputeTotal();
    }

    void evalBand (size_t k)
    {
        const auto& b = bands[k];
        BandSpec s; s.active = true; s.shape = b.shape; s.freq = b.freq; s.gainDb = b.gain; s.q = b.q; s.slope = b.slope;
        Design d; design (s, fsr, d, false);
        for (size_t i = 0; i < G.size(); ++i) resp[k][i] = magnitudeDb (d, G[i], fsr, false);
    }

    void recomputeTotal()
    {
        std::fill (total.begin(), total.end(), 0.0);
        for (auto& r : resp) for (size_t i = 0; i < r.size(); ++i) total[i] += r[i];
    }

    double error() const
    {
        double s = 0, w = 0;
        for (size_t i = 0; i < G.size(); ++i)
        {
            const double e = std::clamp (T[i] - total[i], -40.0, 40.0);
            s += W[i] * e * e; w += W[i];
        }
        return std::sqrt (s / std::max (1e-9, w));
    }

    void optimise (int iterations)
    {
        double stepF = 0.12, stepG = 1.0, stepQ = 0.25;   // ln(f), dB, ln(Q)
        for (int it = 0; it < iterations; ++it)
        {
            bool improved = false;
            for (size_t k = 0; k < bands.size(); ++k)
                for (int p = 0; p < 3; ++p)
                {
                    auto& b = bands[k];
                    if (p == 1 && isCut (b.shape)) continue;
                    for (int dir : { 1, -1 })
                    {
                        const FitBand keep = b;
                        const auto keepR = resp[k];
                        const double before = error();
                        if (p == 0) b.freq = std::clamp (b.freq * std::exp (dir * stepF), 15.0, std::min (25000.0, 0.48 * fsr));
                        if (p == 1) b.gain = std::clamp (b.gain + dir * stepG, -30.0, 30.0);
                        if (p == 2) b.q = std::clamp (b.q * std::exp (dir * stepQ), 0.2, isCut (b.shape) ? 3.0 : 15.0);
                        for (size_t i = 0; i < total.size(); ++i) total[i] -= resp[k][i];
                        evalBand (k);
                        for (size_t i = 0; i < total.size(); ++i) total[i] += resp[k][i];
                        if (error() < before - 1e-6) { improved = true; break; }
                        for (size_t i = 0; i < total.size(); ++i) total[i] -= resp[k][i];
                        b = keep; resp[k] = keepR;
                        for (size_t i = 0; i < total.size(); ++i) total[i] += resp[k][i];
                    }
                }
            if (! improved) { stepF *= 0.6; stepG *= 0.6; stepQ *= 0.6; if (stepG < 0.02) break; }
        }
    }

    std::vector<double> G, T, W, total;
    std::vector<FitBand> bands;
    std::vector<std::vector<double>> resp;
    double fsr = 48000;
};
} // namespace audiolab::eq
