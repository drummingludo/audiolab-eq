// Audio Lab EQ — moteur temps réel (mode zéro latence) :
// chaîne de 24 bandes SVF, EQ dynamique par bande, placement Stéréo / G / D / Mid / Side,
// écoute (solo) d'une bande, transitions sans clic.
//
// 2.0 :
//  - filtres conçus en « matched » (EqMath.h) : la courbe suit l'analogique jusqu'à 20 kHz ;
//  - side-chain externe par bande (Internal / External) pour le détecteur.
// 3.0 (raies parasites sous −110 dB quand un réglage ou le gain dynamique bouge) :
//  - le moteur avance par sous-blocs de 32 échantillons RÉGULIERS, indépendants de la taille des
//    blocs de l'hôte (la phase du sous-bloc est gardée d'un appel à l'autre) ;
//  - réglage qui bouge (automation) : une conception par sous-bloc, puis coefficients interpolés à
//    CHAQUE échantillon par une courbe cubique (Catmull-Rom) passant par les 4 dernières conceptions,
//    au lieu d'une droite entre 2 : trajectoire sans cassure (coût : un retard de 32 échantillons
//    sur le réglage, pas sur le son) ;
//  - EQ dynamique : le détecteur donne un gain à chaque échantillon ; les coefficients de chaque
//    échantillon sont lus dans un cache de conceptions espacées de 0,25 dB (GainCache), interpolé
//    en cubique suivant le gain RÉEL de l'échantillon. Chaque cellule du cache est vérifiée contre
//    une conception exacte en son milieu ; une cellule où la conception n'est pas régulière est
//    lue sans interpolation ;
//  - bande dynamique revenue à 0 dB : les filtres continuent de tourner (états à jour, pas de
//    transitoire quand la dynamique repart) et la sortie reste l'entrée, bit à bit.
#pragma once
#include "EqMath.h"
#include <atomic>
#include <vector>

namespace audiolab::eq
{
// Coefficients temps réel d'une section
struct SectionRT
{
    int kind = 0;
    double a1 = 1, a2 = 0, a3 = 0, m0 = 1, m1 = 0, m2 = 0;   // SVF
    double G = 0, cLp = 1, cHp = 0;                           // 1er ordre
};

inline void toRT (const Section& s, SectionRT& r)
{
    r.kind = (int) s.kind;
    if (s.kind == Section::svf)
    {
        r.a1 = 1.0 / (1.0 + s.g * (s.g + s.k));
        r.a2 = s.g * r.a1;
        r.a3 = s.g * r.a2;
        r.m0 = s.m0; r.m1 = s.m1; r.m2 = s.m2;
    }
    else
    {
        r.G = s.g / (1.0 + s.g);
        r.cLp = s.cLp; r.cHp = s.cHp;
    }
}

// Section intermédiaire (interpolation linéaire des paramètres physiques : g, k, mélanges)
inline void lerpRT (const Section& a, const Section& b, double t, SectionRT& r)
{
    if (a.kind != b.kind) { toRT (t < 0.5 ? a : b, r); return; }
    Section s = b;
    s.g = a.g + (b.g - a.g) * t;
    s.k = a.k + (b.k - a.k) * t;
    s.m0 = a.m0 + (b.m0 - a.m0) * t; s.m1 = a.m1 + (b.m1 - a.m1) * t; s.m2 = a.m2 + (b.m2 - a.m2) * t;
    s.cLp = a.cLp + (b.cLp - a.cLp) * t; s.cHp = a.cHp + (b.cHp - a.cHp) * t;
    toRT (s, r);
}

inline double tick (const SectionRT& c, double* st, double x)
{
    if (c.kind == 0)
    {
        const double v3 = x - st[1];
        const double v1 = c.a1 * st[0] + c.a2 * v3;
        const double v2 = st[1] + c.a2 * st[0] + c.a3 * v3;
        st[0] = 2.0 * v1 - st[0];
        st[1] = 2.0 * v2 - st[1];
        return c.m0 * x + c.m1 * v1 + c.m2 * v2;
    }
    const double v = (x - st[0]) * c.G;
    const double lp = v + st[0];
    st[0] = lp + v;
    return c.cLp * lp + c.cHp * (x - lp);
}

// Filtre du détecteur dynamique / de l'écoute solo : ce que la bande « regarde »
inline Section listenSection (Shape shape, double freq, double q, double fs)
{
    Section s;
    s.kind = Section::svf;
    s.g = warp (clampFreq (freq, fs, false), fs, false);
    switch (shape)
    {
        case Shape::lowShelf: case Shape::lowCut:
            s.k = 1.4142135623730951; s.m0 = 0; s.m1 = 0; s.m2 = 1; break;           // passe-bas
        case Shape::highShelf: case Shape::highCut:
            s.k = 1.4142135623730951; s.m0 = 1; s.m1 = -s.k; s.m2 = -1; break;      // passe-haut
        case Shape::tiltShelf:
            s.k = 1.0 / 0.5; s.m0 = 0; s.m1 = s.k; s.m2 = 0; break;
        default:
            s.k = 1.0 / std::clamp (q, 0.3, 40.0); s.m0 = 0; s.m1 = s.k; s.m2 = 0; break;  // passe-bande 0 dB
    }
    return s;
}

// ── Conceptions sous forme interpolable ─────────────────────────────────
// Paramètres physiques d'une section (SVF : g, k, m0, m1, m2 ; 1er ordre : g, cLp → m0, cHp → m1).
// On interpole ces paramètres, jamais les coefficients finaux : tant que g et k restent positifs,
// le filtre reste stable quelle que soit la vitesse de la modulation.
struct SecP { double g = 0.1, k = 1.4142135623730951, m0 = 1, m1 = 0, m2 = 0; };

struct PDesign
{
    std::array<SecP, 9> s {};
    std::array<signed char, 9> kind {};
    int n = 0;
    double outGain = 1.0;
    bool ident = false;      // gain total exactement nul : la sortie est l'entrée (les filtres tournent quand même)
};

inline void toP (const Design& d, PDesign& p)
{
    p.n = d.n; p.outGain = d.outGain; p.ident = false;
    for (int i = 0; i < d.n; ++i)
    {
        const auto& s = d.s[(size_t) i];
        p.kind[(size_t) i] = (signed char) s.kind;
        if (s.kind == Section::svf) p.s[(size_t) i] = { s.g, s.k, s.m0, s.m1, s.m2 };
        else p.s[(size_t) i] = { s.g, 0.0, s.cLp, s.cHp, 0.0 };
    }
}

inline void sectionFromP (const SecP& p, int kind, Section& s)
{
    s.kind = kind == 0 ? Section::svf : Section::onePole;
    s.g = p.g;
    if (kind == 0) { s.k = p.k; s.m0 = p.m0; s.m1 = p.m1; s.m2 = p.m2; }
    else { s.cLp = p.m0; s.cHp = p.m1; }
}

inline void rtFromP (const SecP& p, int kind, SectionRT& r)
{
    r.kind = kind;
    if (kind == 0)
    {
        r.a1 = 1.0 / (1.0 + p.g * (p.g + p.k));
        r.a2 = p.g * r.a1;
        r.a3 = p.g * r.a2;
        r.m0 = p.m0; r.m1 = p.m1; r.m2 = p.m2;
    }
    else
    {
        r.G = p.g / (1.0 + p.g);
        r.cLp = p.m0; r.cHp = p.m1;
    }
}

inline bool sameStructure (const PDesign& a, const PDesign& b)
{
    if (a.n != b.n) return false;
    for (int i = 0; i < a.n; ++i) if (a.kind[(size_t) i] != b.kind[(size_t) i]) return false;
    return true;
}

// Poids de Catmull-Rom pour la position t (0…1) entre les points 1 et 2 de quatre points réguliers
inline void catmullRom (double t, double* w)
{
    const double t2 = t * t, t3 = t2 * t;
    w[0] = 0.5 * (-t3 + 2.0 * t2 - t);
    w[1] = 0.5 * (3.0 * t3 - 5.0 * t2 + 2.0);
    w[2] = 0.5 * (-3.0 * t3 + 4.0 * t2 + t);
    w[3] = 0.5 * (t3 - t2);
}

// Mélange de quatre sections (poids w). g et k sont gardés positifs (la cubique peut dépasser un peu).
inline void mix4 (const SecP& p, const SecP& a, const SecP& b, const SecP& q, const double* w, SecP& o)
{
    o.g  = std::max (w[0] * p.g + w[1] * a.g + w[2] * b.g + w[3] * q.g, 0.5 * std::min (a.g, b.g));
    o.k  = std::max (w[0] * p.k + w[1] * a.k + w[2] * b.k + w[3] * q.k, 0.5 * std::min (a.k, b.k));
    o.m0 = w[0] * p.m0 + w[1] * a.m0 + w[2] * b.m0 + w[3] * q.m0;
    o.m1 = w[0] * p.m1 + w[1] * a.m1 + w[2] * b.m1 + w[3] * q.m1;
    o.m2 = w[0] * p.m2 + w[1] * a.m2 + w[2] * b.m2 + w[3] * q.m2;
}

// Conception d'une bande dynamique pour un gain total donné. À 0 dB exactement, design() ne rend
// aucune section : on prend alors la limite de la conception (gain de ±1e-6 dB, du côté « sign »),
// pour que les filtres gardent leurs états à jour ; la sortie est alors l'entrée (ident).
inline void designDyn (BandSpec s, double fs, double sign, PDesign& p)
{
    Design d;
    if (hasGain (s.shape) && std::abs (s.gainDb) < 1e-9)
    {
        for (double eps : { 1e-6, 1e-4, 1e-2 })
        {
            s.gainDb = sign < 0 ? -eps : eps;
            design (s, fs, d, false);
            if (d.n > 0) break;
        }
        toP (d, p);
        p.ident = true;
        return;
    }
    design (s, fs, d, false);
    toP (d, p);
}

// ── Cache des conceptions d'une bande dynamique, en fonction du gain ─────
// Fréquence, Q et gain de base fixés ; un nœud tous les 0,25 dB de gain TOTAL (multiples entiers du
// pas : 0 dB est toujours un nœud), conçu à la demande. Le gain d'un échantillon est interpolé en
// cubique (Catmull-Rom) entre les 4 nœuds voisins : erreur ≈ 1e-7 en relatif, contre 1e-4 en linéaire.
//  - 0 dB : la conception y a une cassure (accentuation et atténuation ne se conçoivent pas de la même
//    façon) ; on ne l'enjambe pas : deux nœuds (0⁺ et 0⁻) et interpolation d'un seul côté ;
//  - chaque cellule est vérifiée à sa création : réponse interpolée au milieu contre une conception
//    exacte. La conception « matched » n'est pas partout régulière en fonction du gain (elle choisit
//    entre plusieurs solutions, surtout pour les étagères et près de Nyquist). Écart > 1e-4 : la
//    cubique n'a plus de sens, la cellule est lue en ligne droite entre ses deux nœuds (passage
//    continu d'une solution à l'autre) ; si même la droite s'écarte de plus de 3 % (0,26 dB), la
//    cellule est lue sans interpolation (nœud le plus proche) ;
//  - conception à plus de maxSec sections, ou dont la structure change avec le gain : cache refusé
//    (ok() faux), le moteur revient à une conception exacte tous les 8 échantillons.
class GainCache
{
public:
    static constexpr int maxSec = 3, maxCells = 244;
    static constexpr double baseStep = 0.25, tolCubic = 1.0e-4, tolLinear = 3.0e-2;

    struct Node { SecP s[maxSec]; double outGain = 1.0; };
    struct Tap { const Node* nd[4]; double w[4]; };

    GainCache() : nodes ((size_t) maxCells + 4), nodeOk ((size_t) maxCells + 4, 0), flags ((size_t) maxCells, 0) {}

    // Définit la bande (spec.gainDb sans importance) et l'étendue du gain total [lo, hi] en dB.
    void setup (const BandSpec& sp, double lo, double hi, double rate)
    {
        if (valid && sp.freq == spec.freq && sp.q == spec.q && sp.shape == spec.shape && sp.slope == spec.slope
            && lo == keyLo && hi == keyHi && rate == fs) return;
        spec = sp; keyLo = lo; keyHi = hi; fs = rate; valid = true;
        step = baseStep;
        while ((hi - lo) / step > maxCells - 2) step *= 2.0;
        inv = 1.0 / step;
        mLo = (int) std::floor (lo * inv);
        cells = std::clamp ((int) std::ceil (hi * inv) - mLo, 1, maxCells);
        n = -1; bad = false; rough = 0; linear = 0;
        std::fill (nodeOk.begin(), nodeOk.end(), (unsigned char) 0);
        std::fill (flags.begin(), flags.end(), (unsigned char) 0);
    }

    bool ok() const { return valid && ! bad; }
    int sections() const { return std::max (0, n); }
    int kindOf (int i) const { return kinds[i]; }
    int numCells() const { return cells; }
    int roughCells() const { return rough; }       // cellules lues sans interpolation
    int linearCells() const { return linear; }     // cellules lues en ligne droite
    double stepDb() const { return step; }

    // Cellule et position (0…1) d'un gain total
    inline int cellOf (double totalDb, double& t) const
    {
        const double u = totalDb * inv - mLo;
        const int c = std::clamp ((int) u, 0, cells - 1);
        t = std::clamp (u - c, 0.0, 1.0);
        return c;
    }

    // Prépare les cellules c0…c1 (conceptions manquantes, vérification). budget : nombre de conceptions
    // permises (fil audio) ; faux si le budget ne suffit pas ou si le cache est refusé.
    bool ensure (int c0, int c1, int& budget)
    {
        for (int c = c0; c <= c1; ++c)
        {
            if (flags[(size_t) c] & ready) continue;
            if (bad || budget < 5) return false;
            const int m = mLo + c;
            const bool pos = m >= 0;
            const bool noP = m == 0, noQ = m + 1 == 0;   // on n'enjambe pas 0 dB
            if (! node (m, pos, budget) || ! node (m + 1, pos, budget)) return false;
            if (! noP && ! node (m - 1, pos, budget)) return false;
            if (! noQ && ! node (m + 2, pos, budget)) return false;
            flags[(size_t) c] = (unsigned char) (ready | (noP ? missP : 0) | (noQ ? missQ : 0));

            // vérification au milieu de la cellule : cubique, sinon droite, sinon nœud le plus proche
            Design de;
            BandSpec e = spec; e.active = true; e.gainDb = (m + 0.5) * step;
            design (e, fs, de, false); --budget;
            bool same = de.n == n;
            for (int i = 0; i < n && same; ++i) same = (int) de.s[(size_t) i].kind == kinds[i];
            if (! same) { flags[(size_t) c] |= isRough; ++rough; continue; }
            if (midError (c, de) > tolCubic)
            {
                flags[(size_t) c] |= isLinear; ++linear;
                if (midError (c, de) > tolLinear) { flags[(size_t) c] |= isRough; --linear; ++rough; }
            }
        }
        return true;
    }

    // Nœuds et poids pour la position t de la cellule c (prête)
    inline void tap (int c, double t, Tap& tp) const
    {
        const int m = mLo + c;
        const bool pos = m >= 0;
        const unsigned char fl = flags[(size_t) c];
        const Node* a = &nodes[(size_t) slot (m, pos)];
        const Node* b = &nodes[(size_t) slot (m + 1, pos)];
        tp.nd[1] = a; tp.nd[2] = b;
        if (fl & (isRough | isLinear))
        {
            tp.nd[0] = a; tp.nd[3] = b;
            tp.w[0] = 0; tp.w[3] = 0;
            tp.w[2] = (fl & isRough) ? (t < 0.5 ? 0.0 : 1.0) : t;
            tp.w[1] = 1.0 - tp.w[2];
            return;
        }
        tp.nd[0] = (fl & missP) ? a : &nodes[(size_t) slot (m - 1, pos)];
        tp.nd[3] = (fl & missQ) ? b : &nodes[(size_t) slot (m + 2, pos)];
        catmullRom (t, tp.w);
        if ((fl & missP) && (fl & missQ)) { tp.w[0] = 0; tp.w[1] = 1.0 - t; tp.w[2] = t; tp.w[3] = 0; }
        else if (fl & missP)
        {
            // point manquant remplacé par l'extrapolation quadratique p = 3a − 3b + q
            tp.w[1] += 3.0 * tp.w[0]; tp.w[2] -= 3.0 * tp.w[0]; tp.w[3] += tp.w[0]; tp.w[0] = 0;
        }
        else if (fl & missQ)
        {
            tp.w[2] += 3.0 * tp.w[3]; tp.w[1] -= 3.0 * tp.w[3]; tp.w[0] += tp.w[3]; tp.w[3] = 0;
        }
    }

    // Conception interpolée complète (cellule prête)
    void eval (int c, double t, PDesign& out) const
    {
        Tap tp; tap (c, t, tp);
        out.n = n; out.ident = false; out.outGain = 0;
        for (int j = 0; j < 4; ++j) out.outGain += tp.w[j] * tp.nd[j]->outGain;
        for (int i = 0; i < n; ++i)
        {
            out.kind[(size_t) i] = (signed char) kinds[i];
            mix4 (tp.nd[0]->s[i], tp.nd[1]->s[i], tp.nd[2]->s[i], tp.nd[3]->s[i], tp.w, out.s[(size_t) i]);
        }
    }

private:
    enum : unsigned char { ready = 1, isRough = 2, missP = 4, missQ = 8, isLinear = 16 };

    // Écart relatif de réponse, au milieu de la cellule c, entre la conception interpolée et l'exacte
    double midError (int c, const Design& exact) const
    {
        PDesign pd; eval (c, 0.5, pd);
        Design di; di.n = pd.n; di.outGain = pd.outGain;
        for (int i = 0; i < pd.n; ++i) sectionFromP (pd.s[(size_t) i], pd.kind[(size_t) i], di.s[(size_t) i]);
        double err = 0;
        for (double r : { 0.5, 0.7071, 1.0, 1.4142, 2.0 })
        {
            const double f = std::clamp (spec.freq * r, 5.0, 0.45 * fs);
            const auto he = response (exact, f, fs, false);
            const double x = std::abs (response (di, f, fs, false) - he) / std::max (1e-300, std::abs (he));
            err = std::isfinite (x) ? std::max (err, x) : 1e9;
        }
        return err;
    }

    // rangement : nœuds m = mLo−1 … mLo+cells+1, puis le nœud 0⁺ (0⁻ est à sa place normale)
    inline int slot (int m, bool pos) const { return (m == 0 && pos) ? maxCells + 3 : m - mLo + 1; }

    bool node (int m, bool pos, int& budget)
    {
        const int sl = slot (m, pos);
        if (nodeOk[(size_t) sl]) return true;
        BandSpec e = spec; e.active = true; e.gainDb = m * step;
        PDesign p;
        designDyn (e, fs, pos ? 1.0 : -1.0, p);
        --budget;
        if (p.n < 1 || p.n > maxSec) { bad = true; return false; }
        if (n < 0) { n = p.n; for (int i = 0; i < n; ++i) kinds[i] = p.kind[(size_t) i]; }
        else
        {
            bool same = p.n == n;
            for (int i = 0; i < n && same; ++i) same = p.kind[(size_t) i] == kinds[i];
            if (! same) { bad = true; return false; }
        }
        auto& nd = nodes[(size_t) sl];
        for (int i = 0; i < n; ++i) nd.s[i] = p.s[(size_t) i];
        nd.outGain = p.outGain;
        nodeOk[(size_t) sl] = 1;
        return true;
    }

    std::vector<Node> nodes;
    std::vector<unsigned char> nodeOk, flags;
    BandSpec spec;
    double keyLo = 0, keyHi = 0, fs = 0, step = baseStep, inv = 1.0 / baseStep;
    int mLo = 0, cells = 1, n = -1, rough = 0, linear = 0;
    int kinds[maxSec] {};
    bool valid = false, bad = false;
};

struct DynSettings
{
    bool on = false;
    double thrDb = -30.0, rangeDb = -6.0;
    double attackMs = 10.0, releaseMs = 150.0;
    bool external = false;     // détecteur alimenté par le side-chain externe (s'il est branché)
};

class Engine
{
public:
    static constexpr int subBlock = 32;
    static constexpr int dynStep = 8;        // secours des bandes dynamiques : conception exacte tous les 8 échantillons
    static constexpr int designBudget = 16;  // conceptions permises par sous-bloc pour remplir le cache d'une bande

    // Mesures « avant » (1.x) : coefficients changés par paliers de 32 échantillons, gain dynamique
    // du sous-bloc précédent. Uniquement pour le banc de test.
    bool legacyStepping = false;
    // Référence idéale, uniquement pour le banc de test (très coûteux) : sous-blocs d'un échantillon,
    // donc lissage des réglages et conception EXACTE à chaque échantillon, y compris pour le gain dynamique.
    bool referenceExact = false;

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        for (auto& b : bands) b.reset();
        soloWet = 0; soloBandCur = -1;
        phase = 0;
    }

    // Cibles venant des paramètres (bloc audio). scale : échelle de gain (1 = 100 %).
    // Prises en compte au début du sous-bloc suivant (au plus 32 échantillons plus tard).
    void setBand (int i, const BandSpec& spec, const DynSettings& dyn, double scale)
    {
        auto& b = bands[(size_t) i];
        b.target = spec;
        b.dyn = dyn;
        b.scale = scale;
        if (dyn.attackMs != b.coefAtk || dyn.releaseMs != b.coefRel)
        {
            // temps propres à chaque bande
            b.coefAtk = dyn.attackMs; b.coefRel = dyn.releaseMs;
            b.envA = std::exp (-1.0 / (std::max (0.1, dyn.attackMs) * 0.001 * fs));
            b.envR = std::exp (-1.0 / (std::max (1.0, dyn.releaseMs) * 0.001 * fs));
        }
    }

    void setSolo (int band) { soloTarget = band; }

    // Retour au mode zéro latence : les états des filtres (figés pendant les modes FIR) repartent de zéro
    void clearFilterStates()
    {
        for (auto& b : bands) { for (auto& c : b.st) for (auto& x : c) x[0] = x[1] = 0.0; }
        soloSt[0][0] = soloSt[0][1] = soloSt[1][0] = soloSt[1][1] = 0;
    }

    // Décalage de gain dynamique actuel (dB), lu par l'interface
    float getDynOffset (int i) const { return bands[(size_t) i].dynOut.load (std::memory_order_relaxed); }


    // Traitement en place. R peut être nul (mono). bypassIIR : n'applique pas la chaîne IIR
    // (modes FIR) mais fait tourner les détecteurs ; l'écoute solo se fait alors à part (solo()).
    // scL / scR : side-chain externe (nuls s'il n'est pas branché ; scR nul si mono).
    void process (float* L, float* R, int n, bool bypassIIR, const float* scL = nullptr, const float* scR = nullptr)
    {
        // Sous-blocs réguliers : la phase est gardée d'un appel à l'autre, pour que la trajectoire des
        // réglages ne dépende pas de la taille des blocs de l'hôte.
        const int B = referenceExact ? 1 : subBlock;
        if (phase >= B) phase = 0;
        for (int off = 0; off < n;)
        {
            if (phase == 0)
                for (auto& b : bands) b.advance (fs, B, legacyStepping);
            const int len = std::min (B - phase, n - off);
            float* l = L + off;
            float* r = R != nullptr ? R + off : nullptr;
            const float* sl = scL != nullptr ? scL + off : nullptr;
            const float* sr = scL != nullptr ? (scR != nullptr ? scR + off : sl) : nullptr;

            // Copie d'entrée pour l'écoute solo (signal avant égalisation)
            for (int i = 0; i < len; ++i) { inL[i] = l[i]; inR[i] = r != nullptr ? r[i] : l[i]; }

            for (auto& b : bands)
            {
                if (b.wet <= 0.0 && b.wetTarget <= 0.0 && ! b.detecting) continue;
                processBand (b, l, r, len, phase, B, bypassIIR, sl, sr);
            }
            if (! bypassIIR) solo (inL, inR, l, r, len);
            phase = (phase + len) % B;
            off += len;
        }
    }

    // Écoute d'une bande : on n'entend que la zone du spectre qu'elle traite.
    // src : signal avant égalisation (aligné sur la sortie), l/r : sortie remplacée par fondu.
    void solo (const float* srcL, const float* srcR, float* l, float* r, int len)
    {
        const int want = soloTarget.load (std::memory_order_relaxed);
        const bool valid = want >= 0 && want < maxBands && bands[(size_t) want].init;
        if (valid && want != soloBandCur && soloWet <= 1.0e-4)
        {
            soloBandCur = want;
            soloSt[0][0] = soloSt[0][1] = soloSt[1][0] = soloSt[1][1] = 0;
        }
        const double target = (valid && want == soloBandCur) ? 1.0 : 0.0;
        if (soloWet <= 0 && target <= 0) { if (! valid) soloBandCur = -1; return; }
        if (soloBandCur < 0) { soloWet = 0; return; }

        auto& b = bands[(size_t) soloBandCur];
        SectionRT c; toRT (listenSection (b.shape, std::exp (b.fLog), std::exp (b.qLog), fs), c);
        const double step = 1.0 / (0.015 * fs);
        const bool stereo = r != nullptr;
        for (int i = 0; i < len; ++i)
        {
            soloWet = target > soloWet ? std::min (target, soloWet + step) : std::max (target, soloWet - step);
            const double xl = srcL[i], xr = srcR != nullptr ? srcR[i] : xl;
            double ol, orr;
            switch (b.place)
            {
                case Place::left:  ol = tick (c, soloSt[0], xl); orr = 0; break;
                case Place::right: ol = 0; orr = tick (c, soloSt[1], xr); break;
                case Place::mid:  { const double m = tick (c, soloSt[0], 0.5 * (xl + xr)); ol = m; orr = m; break; }
                case Place::side: { const double s = tick (c, soloSt[0], 0.5 * (xl - xr)); ol = s; orr = -s; break; }
                default: ol = tick (c, soloSt[0], xl); orr = tick (c, soloSt[1], xr); break;
            }
            l[i] = (float) (l[i] * (1.0 - soloWet) + ol * soloWet);
            if (stereo) r[i] = (float) (r[i] * (1.0 - soloWet) + orr * soloWet);
        }
        if (soloWet <= 0 && ! valid) soloBandCur = -1;
    }

    bool soloActive() const { return soloBandCur >= 0 || soloTarget.load() >= 0; }

    // Réponse actuelle d'une bande telle qu'elle est entendue (avec le décalage dynamique)
    double currentGainDb (int i) const { return bands[(size_t) i].target.gainDb * bands[(size_t) i].scale + bands[(size_t) i].dynOffset; }

private:
    struct Band
    {
        BandSpec target;
        DynSettings dyn;
        double scale = 1.0;

        // valeurs lissées
        double fLog = std::log (1000.0), gDb = 0, qLog = 0;
        bool init = false;
        // structure en cours (forme / pente / placement)
        Shape shape = Shape::bell; int slope = 12; Place place = Place::stereo;
        bool structure = false;
        double wet = 0, wetTarget = 0, wetStart = 0;

        // Conceptions.
        //  - bande fixe ou automatisée : hs = les 4 dernières conceptions (fins de sous-bloc) ; le sous-bloc
        //    en cours joue le tronçon hs[1] → hs[2] (cubique si « smooth », sinon hs[2] tel quel) ;
        //  - bande dynamique (et mesures 1.x) : play = conception jouée à la fin du dernier échantillon.
        PDesign hs[4], play;
        bool histValid = false;       // hs à jour (sinon c'est play qui fait foi)
        int settle = 0;               // sous-blocs restants avant que hs soit de nouveau constant
        bool smooth = false;          // coefficients interpolés à chaque échantillon dans ce sous-bloc
        bool playDyn = false;         // play vient du chemin dynamique, pour le gain total lastG
        std::array<SectionRT, 9> rt {};   // coefficients constants (quand rien ne bouge)
        double rtOut = 1.0;
        bool rtIdent = false, rtOk = false;
        int nsec = 0;
        bool fresh = true;            // pas encore de conception (première, ou nouvelle structure)
        double st[2][9][2] {};
        double lastF = -1, lastG = 1e9, lastQ = -1, lastBase = 1e9;
        bool baseMoved = false;       // fréquence / gain / Q ont bougé dans ce sous-bloc

        // dynamique
        bool detecting = false, detValid = false;
        SectionRT det;
        double detSt[2][2] {};
        double env = 0, dynOffset = 0;
        double envA = 0.99, envR = 0.999, coefAtk = -1, coefRel = -1;
        std::atomic<float> dynOut { 0.0f };
        // zones sans logarithme : sous le genou (gain 0) et au-delà de la plage (gain = plage)
        double mapThr = 1e9, mapRange = 1e9, mapScale = -1, envLo = 0, envHi = 1e300, dynFull = 0;
        GainCache cache;
        int budget = 0;               // conceptions encore permises dans ce sous-bloc pour remplir le cache

        void reset()
        {
            coefAtk = coefRel = -1;
            init = false; structure = false; wet = 0; wetTarget = 0; wetStart = 0; nsec = 0; detValid = false;
            env = 0; dynOffset = 0; dynOut.store (0.0f);
            fresh = true; smooth = false; histValid = false; settle = 0; playDyn = false; rtOk = false;
            for (auto& h : hs) h = PDesign();
            play = PDesign();
            clearStates();
        }
        void clearStates()
        {
            for (auto& c : st) for (auto& s : c) s[0] = s[1] = 0.0;
            for (auto& c : detSt) c[0] = c[1] = 0.0;
        }

        BandSpec specFor (double f, double q, double g) const
        {
            BandSpec s; s.active = true; s.shape = shape; s.slope = slope; s.place = place;
            s.freq = f; s.q = q; s.gainDb = g;
            return s;
        }

        void setRT (const PDesign& d)
        {
            for (int i = 0; i < d.n; ++i) rtFromP (d.s[(size_t) i], d.kind[(size_t) i], rt[(size_t) i]);
            rtOut = d.outGain; rtIdent = d.ident; rtOk = true;
        }

        // Prépare un sous-bloc : lissage des réglages, transitions de structure.
        void advance (double fs, int len, bool legacy)
        {
            const bool wantOn = target.active;
            const bool structChanged = structure && (target.shape != shape || target.slope != slope || target.place != place);

            if (! init && wantOn)
            {
                fLog = std::log (target.freq); gDb = target.gainDb * scale; qLog = std::log (target.q);
                shape = target.shape; slope = target.slope; place = target.place;
                structure = true; init = true; clearStates();
                lastF = -1; fresh = true;
            }

            wetStart = wet;
            if (structChanged || ! wantOn)
            {
                // fondu de sortie avant de changer de structure (ou de couper la bande)
                wetTarget = 0.0;
                if (wet <= 1.0e-4)
                {
                    wet = 0.0; wetStart = 0.0;
                    if (structChanged && wantOn)
                    {
                        shape = target.shape; slope = target.slope; place = target.place;
                        clearStates(); lastF = -1; fresh = true;
                    }
                    else if (! wantOn) { init = false; env = 0; dynOffset = 0; dynOut.store (0.0f); }
                }
            }
            else wetTarget = 1.0;

            const double step = (double) len / (0.012 * fs);   // fondu ≈ 12 ms
            if (wet < wetTarget) wet = std::min (wetTarget, wet + step);
            else if (wet > wetTarget) wet = std::max (wetTarget, wet - step);

            if (! init) { detecting = false; return; }

            // lissage ≈ 15 ms (fréquence et Q en log)
            const double a = 1.0 - std::exp (-(double) len / (0.015 * fs));
            const double tf = std::log (std::max (5.0, target.freq)), tq = std::log (std::max (0.025, target.q));
            const bool dynOk = dyn.on && canBeDynamic (shape) && shape == target.shape;
            const double tg = target.gainDb * scale;
            fLog += (tf - fLog) * a;
            qLog += (tq - qLog) * a;
            gDb += (tg - gDb) * a;
            // arrêt du lissage quand l'écart n'est plus audible (évite des reconceptions inutiles)
            if (std::abs (tf - fLog) < 2e-5) fLog = tf;
            if (std::abs (tq - qLog) < 2e-5) qLog = tq;
            if (std::abs (tg - gDb) < 1e-4) gDb = tg;

            detecting = dynOk && wantOn;
            if (! detecting) { dynOffset = 0; env = 0; detValid = false; dynOut.store (0.0f, std::memory_order_relaxed); }

            const double f = std::exp (fLog), q = std::exp (qLog);
            baseMoved = std::abs (f - lastF) > 1e-9 * f || std::abs (gDb - lastBase) > 1e-7 || std::abs (q - lastQ) > 1e-9 * q;
            if (detecting && (! detValid || baseMoved)) { toRT (listenSection (shape, f, q, fs), det); detValid = true; }

            if (detecting)
            {
                budget = designBudget;
                const double full = dyn.rangeDb * scale;
                if (dyn.thrDb != mapThr || full != mapRange || scale != mapScale)
                {
                    mapThr = dyn.thrDb; mapRange = full; mapScale = scale; dynFull = full;
                    const double w = std::abs (full), overFull = w >= 3.0 ? w : std::sqrt (12.0 * w) - 3.0;
                    envLo = std::pow (10.0, (dyn.thrDb - 3.0) / 20.0);
                    envHi = std::pow (10.0, (dyn.thrDb + overFull) / 20.0);
                }
            }

            // 1.x : la conception se faisait ici, avec le gain dynamique du sous-bloc précédent.
            // Bande dynamique (3.0) : les coefficients suivent le gain de chaque échantillon (processDynamic).
            if (legacy) redesignLegacy (fs);
            else if (! detecting) redesignSmooth (fs);
        }

        // Bande fixe ou automatisée : nouvelle conception pour la fin du sous-bloc, rangée dans
        // l'historique ; le sous-bloc joue le tronçon précédent (hs[1] → hs[2]).
        void redesignSmooth (double fs)
        {
            const double f = std::exp (fLog), q = std::exp (qLog), g = gDb;
            bool changed = fresh || std::abs (f - lastF) > 1e-9 * f || std::abs (g - lastG) > 1e-7 || std::abs (q - lastQ) > 1e-9 * q;
            if (! histValid)
            {
                // on sort du chemin dynamique : on repart de ce qui est joué, vers la conception de base
                for (auto& h : hs) h = play;
                histValid = true; settle = 0; playDyn = false; changed = true;
            }
            smooth = false;
            if (changed)
            {
                Design d;
                design (specFor (f, q, g), fs, d, false);
                PDesign pd; toP (d, pd);
                if (fresh || ! sameStructure (pd, hs[3])) { for (auto& h : hs) h = pd; settle = 0; }
                else { hs[0] = hs[1]; hs[1] = hs[2]; hs[2] = hs[3]; hs[3] = pd; settle = 3; smooth = true; }
                fresh = false;
                lastF = f; lastG = g; lastQ = q; lastBase = gDb;
                rtOk = false;
            }
            else if (settle > 0)
            {
                hs[0] = hs[1]; hs[1] = hs[2]; hs[2] = hs[3];
                smooth = --settle > 0;
                rtOk = false;
            }
            nsec = hs[2].n;
            if (! smooth && ! rtOk) setRT (hs[2]);
        }

        // 1.x (mesures « avant ») : une conception par sous-bloc, appliquée d'un coup
        void redesignLegacy (double fs)
        {
            const double f = std::exp (fLog), q = std::exp (qLog), g = gDb + dynOffset;
            if (histValid) { play = hs[2]; histValid = false; settle = 0; }
            const bool changed = fresh || std::abs (f - lastF) > 1e-9 * f || std::abs (g - lastG) > 1e-7 || std::abs (q - lastQ) > 1e-9 * q;
            if (changed)
            {
                Design d;
                design (specFor (f, q, g), fs, d, false);
                toP (d, play);
                fresh = false; rtOk = false; playDyn = false;
                lastF = f; lastG = g; lastQ = q; lastBase = gDb;
            }
            smooth = false;
            nsec = play.n;
            if (! rtOk) setRT (play);
        }

        // Gain dynamique (dB) pour un niveau de détecteur
        inline double dynGain (double e) const
        {
            if (e <= envLo) return 0.0;
            if (e >= envHi) return dynFull;
            return dynCurve (20.0 * std::log10 (std::max (1.0e-9, e)), dyn, scale);
        }

        // Chaîne de la bande pour un échantillon ; coef : coefficients (interpolés ou non).
        // ident : gain total nul, les filtres tournent mais la sortie est l'entrée, bit à bit.
        inline double chain (int ch, double x, const SectionRT* coef, double outGain, bool ident)
        {
            double y = x;
            for (int i = 0; i < nsec; ++i) y = tick (coef[i], st[ch][i], y);
            return ident ? x : y * outGain;
        }
    };

    static inline void follow (Band& b, double d)
    {
        const double x = std::abs (d);
        b.env = x > b.env ? b.envA * b.env + (1.0 - b.envA) * x : b.envR * b.env + (1.0 - b.envR) * x;
    }

    // Courbe de réponse de la dynamique : genou doux de 6 dB, gain qui suit le dépassement
    // du seuil jusqu'à la plage choisie.
    static double dynCurve (double levelDb, const DynSettings& d, double scale)
    {
        const double over = levelDb - d.thrDb, W = 6.0;
        double o;
        if (over <= -W / 2) o = 0;
        else if (over < W / 2) o = (over + W / 2) * (over + W / 2) / (2 * W);
        else o = over;
        const double range = d.rangeDb * scale;
        return range < 0 ? -std::min (o, -range) : std::min (o, range);
    }

    // Détecteur sur un échantillon (entrée de la bande ou side-chain externe)
    void detect (Band& b, double xl, double xr, bool stereo)
    {
        switch (b.place)
        {
            case Place::stereo:
            {
                const double dl = tick (b.det, b.detSt[0], xl);
                const double dr = stereo ? tick (b.det, b.detSt[1], xr) : dl;
                follow (b, std::max (std::abs (dl), std::abs (dr)));
                break;
            }
            case Place::left:  follow (b, tick (b.det, b.detSt[0], xl)); break;
            case Place::right: follow (b, tick (b.det, b.detSt[0], stereo ? xr : xl)); break;
            case Place::mid:   follow (b, tick (b.det, b.detSt[0], stereo ? 0.5 * (xl + xr) : xl)); break;
            case Place::side:  follow (b, tick (b.det, b.detSt[0], stereo ? 0.5 * (xl - xr) : 0.0)); break;
        }
    }

    // Un échantillon de la bande, selon son placement. w : fondu d'entrée / sortie de la bande.
    static inline void applySample (Band& b, float* l, float* r, int i, bool stereo, double w,
                                    const SectionRT* coef, double og, bool ident)
    {
        const double xl = l[i], xr = stereo ? r[i] : xl;
        switch (b.place)
        {
            case Place::stereo:
                l[i] = (float) (xl + w * (b.chain (0, xl, coef, og, ident) - xl));
                if (stereo) r[i] = (float) (xr + w * (b.chain (1, xr, coef, og, ident) - xr));
                break;
            case Place::left:
            case Place::right:
            {
                const bool left = b.place == Place::left;
                if (! stereo && ! left) break;     // mono : la voie droite n'existe pas
                const double x = left ? xl : xr;
                const double y = x + w * (b.chain (0, x, coef, og, ident) - x);
                if (left) l[i] = (float) y; else r[i] = (float) y;
                break;
            }
            case Place::mid:
            case Place::side:
            {
                if (! stereo)
                {
                    if (b.place == Place::side) break;     // mono : pas de côtés
                    l[i] = (float) (xl + w * (b.chain (0, xl, coef, og, ident) - xl));
                    break;
                }
                double mm = 0.5 * (xl + xr), ss = 0.5 * (xl - xr);
                double& v = b.place == Place::mid ? mm : ss;
                const double y = b.chain (0, v, coef, og, ident);
                if (ident) break;                          // (mm + ss ne redonnerait pas l'entrée bit à bit)
                v = v + w * (y - v);
                l[i] = (float) (mm + ss);
                r[i] = (float) (mm - ss);
                break;
            }
        }
    }

    // len échantillons d'un sous-bloc de B échantillons, à partir de la position ph
    void processBand (Band& b, float* l, float* r, int len, int ph, int B, bool bypassIIR, const float* scl, const float* scr)
    {
        const bool stereo = r != nullptr;
        const bool det = b.detecting;
        const double dw = (b.wet - b.wetStart) / B, w0 = b.wetStart + dw * ph;
        const bool filt = ! bypassIIR && (b.wetStart > 0.0 || b.wet > 0.0);

        if (det && ! legacyStepping) { processDynamic (b, l, r, len, w0, dw, filt, scl, scr); return; }

        if (det)
        {
            // 1.x : détecteur sur le sous-bloc, gain appliqué au sous-bloc suivant
            const bool ext = b.dyn.external && scl != nullptr;
            const bool scStereo = scr != nullptr && scr != scl;
            for (int i = 0; i < len; ++i)
            {
                if (ext) detect (b, scl[i], scStereo ? scr[i] : scl[i], scStereo || stereo);
                else detect (b, l[i], stereo ? r[i] : l[i], stereo);
            }
            b.dynOffset = dynCurve (20.0 * std::log10 (std::max (1.0e-9, b.env)), b.dyn, b.scale);
            b.dynOut.store ((float) b.dynOffset, std::memory_order_relaxed);
        }
        if (! filt || b.nsec == 0) return;

        if (! b.smooth)
        {
            for (int i = 0; i < len; ++i) applySample (b, l, r, i, stereo, w0 + dw * (i + 1), b.rt.data(), b.rtOut, b.rtIdent);
            return;
        }
        // réglage qui bouge : cubique passant par les 4 dernières conceptions, à chaque échantillon
        SectionRT now[9];
        const auto& h = b.hs;
        for (int i = 0; i < len; ++i)
        {
            double cw[4];
            catmullRom ((double) (ph + i + 1) / B, cw);
            for (int sct = 0; sct < b.nsec; ++sct)
            {
                SecP o;
                mix4 (h[0].s[(size_t) sct], h[1].s[(size_t) sct], h[2].s[(size_t) sct], h[3].s[(size_t) sct], cw, o);
                rtFromP (o, h[2].kind[(size_t) sct], now[sct]);
            }
            const double og = cw[0] * h[0].outGain + cw[1] * h[1].outGain + cw[2] * h[2].outGain + cw[3] * h[3].outGain;
            applySample (b, l, r, i, stereo, w0 + dw * (i + 1), now, og, false);
        }
    }

    // Bande dynamique : détecteur puis filtrage, les coefficients suivant le gain de CHAQUE échantillon.
    void processDynamic (Band& b, float* l, float* r, int len, double w0, double dw, bool filt, const float* scl, const float* scr)
    {
        const bool stereo = r != nullptr;
        const bool ext = b.dyn.external && scl != nullptr;
        const bool scStereo = scr != nullptr && scr != scl;

        // 1) détecteur sur tout le tronçon (entrée de la bande, ou side-chain externe) : gain total par échantillon
        double tot[subBlock];
        double dynLast = b.dynOffset;
        for (int i = 0; i < len; ++i)
        {
            if (ext) detect (b, scl[i], scStereo ? scr[i] : scl[i], scStereo || stereo);
            else detect (b, l[i], stereo ? r[i] : l[i], stereo);
            dynLast = b.dynGain (b.env);
            tot[i] = b.gDb + dynLast;
        }
        b.dynOffset = dynLast;
        b.dynOut.store ((float) dynLast, std::memory_order_relaxed);
        if (! filt) return;

        if (b.histValid)
        {
            // la bande vient de devenir dynamique : on repart de ce qui est joué
            b.play = b.hs[2]; b.histValid = false; b.smooth = false; b.playDyn = false; b.rtOk = false;
            if (b.settle > 0) b.lastF = -1;
            b.settle = 0;
        }
        const double f = std::exp (b.fLog), q = std::exp (b.qLog);
        const double sign = b.dynFull < 0 ? -1.0 : 1.0;
        // (bande neuve : rien n'est encore joué, le cache sert tout de suite)
        const bool moved = ! b.fresh && (std::abs (f - b.lastF) > 1e-9 * f || std::abs (b.gDb - b.lastBase) > 1e-7 || std::abs (q - b.lastQ) > 1e-9 * q);
        BandSpec sp = b.specFor (f, q, b.gDb);
        SectionRT now[9];

        if (referenceExact)
        {
            // référence (banc de test) : conception exacte à chaque échantillon
            for (int i = 0; i < len; ++i)
            {
                sp.gainDb = tot[i];
                designDyn (sp, fs, sign, b.play);
                b.nsec = b.play.n;
                for (int sct = 0; sct < b.nsec; ++sct) rtFromP (b.play.s[(size_t) sct], b.play.kind[(size_t) sct], now[sct]);
                applySample (b, l, r, i, stereo, w0 + dw * (i + 1), now, b.play.outGain, b.play.ident);
            }
        }
        else
        {
            bool fast = ! moved;
            if (fast)
            {
                b.cache.setup (sp, b.gDb + std::min (0.0, b.dynFull), b.gDb + std::max (0.0, b.dynFull), fs);
                fast = b.cache.ok();
            }
            if (fast)
            {
                // gain immobile (sous le seuil, ou au bout de la plage) : coefficients constants
                bool still = b.playDyn && ! b.fresh;
                for (int i = 0; i < len && still; ++i) still = tot[i] == b.lastG;
                if (still)
                {
                    if (! b.rtOk) b.setRT (b.play);
                    b.nsec = b.play.n;
                    for (int i = 0; i < len; ++i) applySample (b, l, r, i, stereo, w0 + dw * (i + 1), b.rt.data(), b.rtOut, b.rtIdent);
                    return;
                }
                int cell[subBlock]; double pos[subBlock];
                int c0 = GainCache::maxCells, c1 = 0;
                for (int i = 0; i < len; ++i)
                {
                    cell[i] = b.cache.cellOf (tot[i], pos[i]);
                    c0 = std::min (c0, cell[i]); c1 = std::max (c1, cell[i]);
                }
                fast = b.cache.ensure (c0, c1, b.budget);
                if (fast)
                {
                    // 2) coefficients de chaque échantillon : cache interpolé suivant son gain
                    const int ns = b.cache.sections();
                    b.nsec = ns;
                    GainCache::Tap tp;
                    for (int i = 0; i < len; ++i)
                    {
                        b.cache.tap (cell[i], pos[i], tp);
                        for (int sct = 0; sct < ns; ++sct)
                        {
                            SecP o;
                            mix4 (tp.nd[0]->s[sct], tp.nd[1]->s[sct], tp.nd[2]->s[sct], tp.nd[3]->s[sct], tp.w, o);
                            rtFromP (o, b.cache.kindOf (sct), now[sct]);
                        }
                        const double og = tp.w[0] * tp.nd[0]->outGain + tp.w[1] * tp.nd[1]->outGain + tp.w[2] * tp.nd[2]->outGain + tp.w[3] * tp.nd[3]->outGain;
                        applySample (b, l, r, i, stereo, w0 + dw * (i + 1), now, og, tot[i] == 0.0);
                    }
                    b.cache.eval (cell[len - 1], pos[len - 1], b.play);
                    b.play.ident = tot[len - 1] == 0.0;
                    b.setRT (b.play);
                    b.lastF = f; b.lastQ = q; b.lastBase = b.gDb; b.lastG = tot[len - 1];
                    b.playDyn = true; b.fresh = false;
                    return;
                }
            }
            // Secours (fréquence / gain de base / Q en mouvement, cache refusé ou en cours de remplissage).
            double gLo = tot[0], gHi = tot[0];
            for (int i = 1; i < len; ++i) { gLo = std::min (gLo, tot[i]); gHi = std::max (gHi, tot[i]); }
            if (! b.fresh && gHi - gLo <= GainCache::baseStep)
            {
                // Gain qui bouge peu dans le tronçon (cas courant) : deux conceptions exactes, aux gains
                // extrêmes du tronçon, pour les réglages d'arrivée. Chaque échantillon est interpolé entre
                // les deux suivant SON gain ; l'écart avec ce qui était joué s'efface en ligne droite.
                PDesign lo, hi;
                sp.gainDb = gLo; designDyn (sp, fs, sign, lo);
                if (gHi > gLo) { sp.gainDb = gHi; designDyn (sp, fs, sign, hi); }
                if (sameStructure (lo, b.play) && (gHi <= gLo || sameStructure (lo, hi)))
                {
                    const PDesign& top = gHi > gLo ? hi : lo;
                    const double inv = gHi > gLo ? 1.0 / (gHi - gLo) : 0.0;
                    const double u0 = ((b.playDyn ? b.lastG : tot[0]) - gLo) * inv;
                    const bool allIdent = lo.ident && top.ident && b.play.ident;
                    const auto& p = b.play;
                    b.nsec = lo.n;
                    double cw[4] = { 0, 0, 0, 0 };
                    for (int i = 0; i < len; ++i)
                    {
                        const double u = (tot[i] - gLo) * inv, rest = 1.0 - (double) (i + 1) / len;
                        cw[0] = rest; cw[1] = (1.0 - u) - rest * (1.0 - u0); cw[2] = u - rest * u0;
                        for (int sct = 0; sct < lo.n; ++sct)
                        {
                            SecP o;
                            mix4 (p.s[(size_t) sct], lo.s[(size_t) sct], top.s[(size_t) sct], top.s[(size_t) sct], cw, o);
                            rtFromP (o, lo.kind[(size_t) sct], now[sct]);
                        }
                        applySample (b, l, r, i, stereo, w0 + dw * (i + 1), now, cw[0] * p.outGain + cw[1] * lo.outGain + cw[2] * top.outGain, allIdent);
                    }
                    PDesign end = lo;
                    for (int sct = 0; sct < lo.n; ++sct) mix4 (lo.s[(size_t) sct], lo.s[(size_t) sct], top.s[(size_t) sct], top.s[(size_t) sct], cw, end.s[(size_t) sct]);
                    end.outGain = cw[1] * lo.outGain + cw[2] * top.outGain;
                    end.ident = tot[len - 1] == 0.0;
                    b.play = end;
                    b.lastF = f; b.lastQ = q; b.lastBase = b.gDb; b.lastG = tot[len - 1];
                    b.playDyn = true; b.rtOk = false;
                    return;
                }
            }
            // Gain qui bouge vite, ou structure qui change : conception exacte tous les 8 échantillons,
            // interpolation linéaire entre les deux
            for (int s0 = 0; s0 < len; s0 += dynStep)
            {
                const int m = std::min (dynStep, len - s0);
                const double g = tot[s0 + m - 1];
                if (! moved && b.playDyn && g == b.lastG)
                {
                    if (! b.rtOk) b.setRT (b.play);
                    b.nsec = b.play.n;
                    for (int i = s0; i < s0 + m; ++i) applySample (b, l, r, i, stereo, w0 + dw * (i + 1), b.rt.data(), b.rtOut, b.rtIdent);
                    continue;
                }
                PDesign d;
                sp.gainDb = g;
                designDyn (sp, fs, sign, d);
                b.nsec = d.n;
                if (! b.fresh && sameStructure (d, b.play))
                {
                    const auto& p = b.play;
                    for (int i = s0; i < s0 + m; ++i)
                    {
                        const double t = (double) (i - s0 + 1) / m;
                        const double cw[4] = { 0.0, 1.0 - t, t, 0.0 };
                        for (int sct = 0; sct < d.n; ++sct)
                        {
                            SecP o;
                            mix4 (p.s[(size_t) sct], p.s[(size_t) sct], d.s[(size_t) sct], d.s[(size_t) sct], cw, o);
                            rtFromP (o, d.kind[(size_t) sct], now[sct]);
                        }
                        applySample (b, l, r, i, stereo, w0 + dw * (i + 1), now, p.outGain + (d.outGain - p.outGain) * t, p.ident && d.ident);
                    }
                }
                else
                {
                    for (int sct = 0; sct < d.n; ++sct) rtFromP (d.s[(size_t) sct], d.kind[(size_t) sct], now[sct]);
                    for (int i = s0; i < s0 + m; ++i) applySample (b, l, r, i, stereo, w0 + dw * (i + 1), now, d.outGain, d.ident);
                }
                b.play = d; b.fresh = false; b.rtOk = false; b.playDyn = true; b.lastG = g;
            }
        }
        b.lastF = f; b.lastQ = q; b.lastBase = b.gDb; b.lastG = tot[len - 1];
        b.fresh = false; b.playDyn = true; b.rtOk = false;
    }

    double fs = 48000.0;
    std::array<Band, maxBands> bands;
    std::atomic<int> soloTarget { -1 };
    int soloBandCur = -1;
    double soloWet = 0;
    double soloSt[2][2] {};
    float inL[subBlock] {}, inR[subBlock] {};
    int phase = 0;     // position dans le sous-bloc en cours
};
} // namespace audiolab::eq
