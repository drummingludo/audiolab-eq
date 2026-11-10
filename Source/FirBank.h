// Audio Lab EQ — bandes dynamiques dans les modes FIR (3.0), sans JUCE.
//
// Natural phase : les bandes dynamiques sortent du FIR ; elles sont traitées par le moteur zéro
//   latence, en série après la convolution (le FIR ne garde que les bandes fixes).
// Linear phase : mélange de FIR à CHAQUE échantillon. Le jeu de FIR contient
//   - le FIR de référence S : toutes les bandes, les bandes dynamiques à leur gain de base ;
//   - par bande dynamique seule, des FIR d'écart D_a = F_a − S (F_a = S avec la bande au gain de
//     l'ancre a), une ancre tous les 4 dB au plus sur la plage utile (moins pour un tilt ou une
//     étagère résonante, voir setAnchors) ;
//   - par paire de bandes dynamiques qui se recouvrent (couplage > 0,1 dB) : les FIR croisés
//     D_ab = F_ab − S (les deux bandes aux ancres a et b), sur la grille des ancres des deux bandes.
//   sortie(t) = (S*x)(t) + Σ poids(t) · (D*x)(t)
//   Bande seule : 2 FIR d'écart actifs au plus (les deux ancres qui encadrent le gain du moment),
//   poids (1 − α) et α, α calculé à chaque échantillon pour que le gain au centre de la bande (plateau
//   d'une étagère) soit exactement celui du détecteur. Paire : 4 FIR actifs, poids bilinéaires
//   (1 − α1)(1 − α2), α1(1 − α2), (1 − α1)α2, α1α2 : le produit des deux bandes interpolées est
//   reproduit exactement (aucun terme croisé manquant ; un seul FIR croisé pondéré par α1·α2 laisse
//   jusqu'à 1,2 dB d'erreur aux gains intermédiaires, voir le banc qualité, section 21).
//   Une bande couplée à plusieurs autres (chaîne A–B–C) : chaque paire couplée a sa grille, et l'écart
//   propre de la bande, compté une fois par paire, est retiré (nombre de paires − 1) fois :
//     H/S = 1 + Σ_paires [(1 + u_i)(1 + u_j) − 1] + Σ_bandes (1 − nombre de paires) · u_i
//   Il ne manque que les produits des paires non couplées et les produits triples ; la répartition
//   vérifie, à tous les coins des plages, que cette erreur reste sous mixTol (0,2 dB).
//   Limites : 4 bandes dynamiques mélangées ; une bande en plus, ou dont l'ajout dépasserait mixTol
//   ou le nombre de FIR d'écart permis, reste dans S avec son gain dynamique du moment (méthode 2.0 :
//   reconception du jeu + fondu).
#pragma once
#include "FirDesign.h"
#include <cstdint>

namespace audiolab::eq::fir
{
constexpr int maxMix = 4;            // bandes dynamiques mélangées à l'échantillon près
constexpr int maxSeg = 16;           // tronçons (ancres − 1) par bande
constexpr int maxEdges = 6;          // paires couplées
constexpr int maxDev = 64;           // FIR d'écart par jeu
constexpr double anchorStep = 4.0, anchorStepResonant = 2.0;   // dB entre deux ancres
constexpr double coupleTol = 0.1;    // dB : au-delà, deux bandes dynamiques sont traitées en paire
constexpr double mixTol = 0.2;       // dB : erreur de couplage restante admise (tous les coins des plages)

// Dynamique des bandes vue par la conception : plage utile en dB (0 = bande fixe) et décalage du moment
struct DynInfo
{
    std::array<double, maxBands> full {};
    std::array<double, maxBands> now {};
};

struct MixBand
{
    int band = -1;
    double full = 0;                 // plage utile (dB, signée)
    int nSeg = 1;                    // ancres : 0 (gain de base) … nSeg (pleine plage)
    double kExp = 0;                 // gain linéaire au point de référence = exp (kExp · décalage)
    int deg = 0;                     // nombre de paires auxquelles la bande appartient
    int home = -1;                   // paire qui porte la correction (1 − deg) · u
    int first = 0;                   // bande seule : son premier FIR d'écart (ancre 1)
    double lin[maxSeg + 1] {};       // gain linéaire de référence de chaque ancre
    double offsetOf (int a) const { return full * a / nSeg; }
};

struct Edge
{
    int k1 = -1, k2 = -1;            // indices dans Plan::mb
    int first = 0, count = 0;        // FIR d'écart de la grille, (a, b) ≠ (0, 0)
    int devOf (int a, int b, int n2) const { const int i = a * (n2 + 1) + b; return i == 0 ? -1 : first + i - 1; }
};

struct Plan
{
    uint32_t engineMask = 0;         // Natural : bandes confiées au moteur zéro latence
    uint32_t mixMask = 0;            // Linear : bandes mélangées à l'échantillon près
    uint32_t fallbackMask = 0;       // Linear : bandes dynamiques en trop (reconception + fondu)
    int nMix = 0, nEdges = 0, nDev = 0;
    double coupleErr = 0;            // erreur de couplage restante (dB), pire coin
    bool fallbackCoupled = false;    // une bande en trop recouvre une bande mélangée (couplage > coupleTol)
    MixBand mb[maxMix];
    Edge e[maxEdges];
};

inline bool isDynamic (const BandSpec& b, double full)
{
    return b.active && canBeDynamic (b.shape) && std::abs (full) > 1.0e-6;
}

// Bandes qui ne peuvent pas se coupler : elles ne touchent pas la même partie du signal
inline bool disjoint (const BandSpec& b1, const BandSpec& b2, bool stereo)
{
    if (! stereo) return false;
    const auto p = b1.place, q = b2.place;
    return (p == Place::left && q == Place::right) || (p == Place::right && q == Place::left)
        || (p == Place::mid && q == Place::side) || (p == Place::side && q == Place::mid);
}

// Écart relatif u(f) d'une bande à pleine plage, sur la grille de la répartition : |B(base + plage)| / |B(base)| − 1
constexpr int planPoints = 160;
inline void relativeDeviation (const BandSpec& b, double full, double fs, double* u)
{
    Design d, e;
    BandSpec x = b; eq::design (x, fs, d, true); x.gainDb += full; eq::design (x, fs, e, true);
    const double f1 = std::min (24000.0, 0.5 * fs);
    for (int i = 0; i < planPoints; ++i)
    {
        const double f = 10.0 * std::pow (f1 / 10.0, i / (planPoints - 1.0));
        u[i] = std::abs (response (e, f, fs, true)) / std::max (1e-12, std::abs (response (d, f, fs, true))) - 1.0;
    }
}

// Couplage de deux bandes dynamiques : erreur maximale (dB) du mélange additif à pleine plage,
// c'est-à-dire le terme croisé oublié, (1 + u1)(1 + u2) contre 1 + u1 + u2.
inline double coupling (const double* u1, const double* u2)
{
    double worst = 0;
    for (int i = 0; i < planPoints; ++i)
    {
        const double add = 1.0 + u1[i] + u2[i];
        if (add <= 1e-6) return 1e9;
        worst = std::max (worst, std::abs (20.0 * std::log10 ((1.0 + u1[i]) * (1.0 + u2[i]) / add)));
    }
    return worst;
}
inline double coupling (const BandSpec& b1, double full1, const BandSpec& b2, double full2, double fs, bool stereo)
{
    if (disjoint (b1, b2, stereo)) return 0.0;
    double u1[planPoints], u2[planPoints];
    relativeDeviation (b1, full1, fs, u1); relativeDeviation (b2, full2, fs, u2);
    return coupling (u1, u2);
}

// Ancres d'une bande : pas de 4 dB (cloches, étagères non résonantes), 3 dB pour un tilt ; étagère
// résonante (12 dB/oct, Q > 0,8) : 2 dB, et 4/Q dB au-delà de Q 2 (l'erreur d'interpolation croît
// comme Q²). Au plus maxSeg tronçons : au-delà, le pas s'allonge (plages très grandes).
inline void setAnchors (MixBand& m, const BandSpec& b, double factor)
{
    const bool shelf = b.shape == Shape::lowShelf || b.shape == Shape::highShelf || b.shape == Shape::tiltShelf;
    double step = anchorStep;
    if (shelf && b.slope > 6 && b.q > 0.8) step = std::min (anchorStepResonant, anchorStep / b.q);
    if (b.shape == Shape::tiltShelf) step *= 0.75;
    step *= factor;
    m.nSeg = std::clamp ((int) std::ceil (std::abs (m.full) / step - 1e-9), 1, maxSeg);
    // point de référence : centre d'une cloche, plateau d'une étagère ; tilt : plateau à gain / 2
    m.kExp = std::log (10.0) / (b.shape == Shape::tiltShelf ? 40.0 : 20.0);
    for (int a = 0; a <= m.nSeg; ++a) m.lin[a] = std::exp (m.kExp * m.offsetOf (a));
}

// Répartition des bandes dynamiques. bands : gains de base. devBudget : FIR d'écart permis.
inline Plan makePlan (const std::array<BandSpec, maxBands>& bands, const DynInfo& dyn, Mode mode, double fs, bool stereo,
                      int devBudget = maxDev)
{
    Plan p;
    int acc[maxMix], nAcc = 0;                 // bandes retenues
    double u[maxMix][planPoints];              // leur écart relatif à pleine plage
    bool edge[maxMix][maxMix] {};

    // jeu pour les n premières bandes retenues ; fine : ancres resserrées (× 0,75) pour les bandes en paire
    auto build = [&] (int n, bool fine)
    {
        Plan t;
        t.nMix = n;
        for (int k = 0; k < n; ++k)
        {
            auto& m = t.mb[k];
            m.band = acc[k]; m.full = dyn.full[(size_t) acc[k]];
            for (int j = 0; j < n; ++j) if (j != k && edge[k][j]) ++m.deg;
            // (paire : les erreurs d'interpolation des deux bandes s'ajoutent)
            setAnchors (m, bands[(size_t) acc[k]], fine && m.deg > 0 ? 0.75 : 1.0);
            t.mixMask |= 1u << acc[k];
        }
        for (int k = 0; k < n; ++k)
        {
            if (t.mb[k].deg == 0) { t.mb[k].first = t.nDev; t.nDev += t.mb[k].nSeg; continue; }
            for (int j = k + 1; j < n; ++j)
            {
                if (! edge[k][j]) continue;
                auto& e = t.e[t.nEdges];
                e.k1 = k; e.k2 = j; e.first = t.nDev; e.count = (t.mb[k].nSeg + 1) * (t.mb[j].nSeg + 1) - 1;
                t.nDev += e.count;
                if (t.mb[k].home < 0) t.mb[k].home = t.nEdges;
                if (t.mb[j].home < 0) t.mb[j].home = t.nEdges;
                ++t.nEdges;
            }
        }
        return t;
    };
    // erreur de couplage restante pour les n premières bandes : produit exact contre modèle (paires +
    // corrections), à tous les coins des plages (chaque bande au repos ou à pleine plage)
    auto cornerError = [&] (int n)
    {
        double worst = 0;
        for (int corner = 1; corner < (1 << n); ++corner)
            for (int i = 0; i < planPoints; ++i)
            {
                double exact = 1.0, model = 1.0, v[maxMix];
                for (int k = 0; k < n; ++k)
                {
                    v[k] = ((corner >> k) & 1) ? u[k][i] : 0.0;
                    exact *= 1.0 + v[k];
                    int deg = 0;
                    for (int j = 0; j < n; ++j) if (j != k && edge[k][j]) ++deg;
                    model += (1.0 - deg) * v[k];
                }
                for (int k = 0; k < n; ++k)
                    for (int j = k + 1; j < n; ++j)
                        if (edge[k][j]) model += (1.0 + v[k]) * (1.0 + v[j]) - 1.0;
                if (model <= 1e-6) return 1e9;
                worst = std::max (worst, std::abs (20.0 * std::log10 (exact / model)));
            }
        return worst;
    };

    for (int i = 0; i < maxBands; ++i)
    {
        const auto& b = bands[(size_t) i];
        const double full = dyn.full[(size_t) i];
        if (! isDynamic (b, full)) continue;
        const uint32_t bit = 1u << i;
        if (mode == Mode::natural) { p.engineMask |= bit; continue; }
        if (nAcc >= maxMix) { p.fallbackMask |= bit; continue; }

        const int k = nAcc;
        acc[k] = i;
        relativeDeviation (b, full, fs, u[k]);
        for (int j = 0; j < k; ++j)
            edge[k][j] = edge[j][k] = ! disjoint (b, bands[(size_t) acc[j]], stereo) && coupling (u[k], u[j]) > coupleTol;
        const Plan t = build (k + 1, false);
        if (t.nDev > devBudget || cornerError (k + 1) > mixTol)
        {
            for (int j = 0; j < k; ++j) edge[k][j] = edge[j][k] = false;
            p.fallbackMask |= bit;
            continue;
        }
        ++nAcc;
    }
    if (nAcc > 0)
    {
        Plan t = build (nAcc, true);
        if (t.nDev > devBudget) t = build (nAcc, false);
        t.coupleErr = cornerError (nAcc);
        t.fallbackMask = p.fallbackMask;
        p = t;
        // bandes en trop : si aucune ne recouvre une bande mélangée, leur mouvement ne change que le FIR
        // de référence (les FIR d'écart restent valables à coupleTol près, voir LinearPhaseCore.h)
        for (int i = 0; i < maxBands && ! p.fallbackCoupled; ++i)
        {
            if (! (p.fallbackMask & (1u << i))) continue;
            double uf[planPoints];
            relativeDeviation (bands[(size_t) i], dyn.full[(size_t) i], fs, uf);
            for (int k = 0; k < nAcc; ++k)
                if (! disjoint (bands[(size_t) i], bands[(size_t) acc[k]], stereo) && coupling (uf, u[k]) > coupleTol) { p.fallbackCoupled = true; break; }
        }
    }
    return p;
}

// Tronçon et position α d'un décalage dynamique (dB) : gain exact au point de référence de la bande
inline void locate (const MixBand& m, double offDb, int& seg, double& alpha)
{
    const double r = std::clamp (offDb / m.full, 0.0, 1.0);
    seg = std::min ((int) (r * m.nSeg), m.nSeg - 1);
    const double c = std::exp (m.kExp * r * m.full);
    alpha = (c - m.lin[seg]) / (m.lin[seg + 1] - m.lin[seg]);
}

// Poids d'un FIR d'écart d'une paire pour les poids d'ancre wa (bande 1, ancre a) et wb (bande 2, ancre b) :
// produit bilinéaire, plus la correction (1 − deg) · u de chaque bande, portée par les FIR (a, 0) et
// (0, b) de sa paire « home ».
template <class T>
inline T edgeWeight (const Plan& p, int ei, int a, int b, T wa, T wb)
{
    const auto& e = p.e[ei];
    T w = wa * wb;
    if (b == 0 && p.mb[e.k1].home == ei) w += (T) (1 - p.mb[e.k1].deg) * wa;
    if (a == 0 && p.mb[e.k2].home == ei) w += (T) (1 - p.mb[e.k2].deg) * wb;
    return w;
}

// Poids de tous les FIR d'écart pour des poids d'ancre donnés (wt[k][a]) : pour les tests et l'affichage
inline void devWeights (const Plan& p, const double wt[maxMix][maxSeg + 1], double* c)
{
    for (int i = 0; i < p.nDev; ++i) c[i] = 0.0;
    for (int k = 0; k < p.nMix; ++k)
        if (p.mb[k].deg == 0)
            for (int a = 1; a <= p.mb[k].nSeg; ++a) c[p.mb[k].first + a - 1] = wt[k][a];
    for (int ei = 0; ei < p.nEdges; ++ei)
    {
        const auto& e = p.e[ei];
        const int n1 = p.mb[e.k1].nSeg, n2 = p.mb[e.k2].nSeg;
        for (int a = 0; a <= n1; ++a)
            for (int b = 0; b <= n2; ++b)
            {
                const int di = e.devOf (a, b, n2);
                if (di >= 0) c[di] = edgeWeight (p, ei, a, b, wt[e.k1][a], wt[e.k2][b]);
            }
    }
}

// Bandes vues par le FIR de référence : gains de base ; bandes confiées au moteur retirées ; bandes
// dynamiques en trop à leur gain du moment.
inline void referenceBands (std::array<BandSpec, maxBands>& bands, const DynInfo& dyn, const Plan& plan)
{
    for (int i = 0; i < maxBands; ++i)
    {
        const uint32_t bit = 1u << i;
        if (plan.engineMask & bit) bands[(size_t) i].active = false;
        else if ((plan.fallbackMask & bit) && hasGain (bands[(size_t) i].shape)) bands[(size_t) i].gainDb += dyn.now[(size_t) i];
    }
}

// Conception du jeu complet : ref = S ; dev[i] = FIR d'écart i (F − S). rq.bands : bandes de base.
// refOnly : seulement S (les FIR d'écart d'un jeu précédent sont repris tels quels).
inline void designBank (const Request& rq, const DynInfo& dyn, const Plan& plan, Result& ref, std::vector<Result>& dev, Work& w,
                        bool refOnly = false)
{
    Request r0 = rq;
    referenceBands (r0.bands, dyn, plan);
    design (r0, ref, w);
    if (refOnly) return;
    dev.resize ((size_t) plan.nDev);
    auto one = [&] (int di, int k1, int a, int k2, int b)
    {
        Request r = r0;
        r.bands[(size_t) plan.mb[k1].band].gainDb += plan.mb[k1].offsetOf (a);
        if (k2 >= 0) r.bands[(size_t) plan.mb[k2].band].gainDb += plan.mb[k2].offsetOf (b);
        auto& d = dev[(size_t) di];
        design (r, d, w);
        for (int e = 0; e < 4; ++e)
            for (size_t n = 0; n < d.h[e].size() && n < ref.h[e].size(); ++n) d.h[e][n] -= ref.h[e][n];
        d.matrix = d.matrix || ref.matrix;
    };
    for (int k = 0; k < plan.nMix; ++k)
        if (plan.mb[k].deg == 0)
            for (int a = 1; a <= plan.mb[k].nSeg; ++a) one (plan.mb[k].first + a - 1, k, a, -1, 0);
    for (int ei = 0; ei < plan.nEdges; ++ei)
    {
        const auto& e = plan.e[ei];
        const int n1 = plan.mb[e.k1].nSeg, n2 = plan.mb[e.k2].nSeg;
        for (int a = 0; a <= n1; ++a)
            for (int b = 0; b <= n2; ++b)
                if (const int di = e.devOf (a, b, n2); di >= 0) one (di, e.k1, a, e.k2, b);
    }
}
} // namespace audiolab::eq::fir
