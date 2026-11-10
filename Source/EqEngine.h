// Audio Lab EQ — moteur temps réel (mode zéro latence) :
// chaîne de 24 bandes SVF, EQ dynamique par bande, placement Stéréo / G / D / Mid / Side,
// écoute (solo) d'une bande, transitions sans clic.
//
// 2.0 :
//  - filtres conçus en « matched » (EqMath.h) : la courbe suit l'analogique jusqu'à 20 kHz ;
//  - coefficients interpolés à CHAQUE échantillon entre deux conceptions (sous-blocs de 32) :
//    plus d'effet d'escalier quand un réglage bouge ;
//  - EQ dynamique : le détecteur tourne d'abord sur le sous-bloc, le gain est conçu pour la fin
//    du sous-bloc, puis chaque échantillon prend le gain que le détecteur lui donne (interpolation
//    des coefficients suivant le gain de l'échantillon) : plus de raies à fs/32 ;
//  - side-chain externe par bande (Internal / External) pour le détecteur.
//
// 3.0 :
//  - une bande peut recevoir des sections de correction (EqMath.h) : leur nombre change avec les
//    réglages ; une section qui apparaît ou disparaît est interpolée depuis / vers une section neutre ;
//  - quand un réglage bouge, chaque conception part de la précédente (suivi : rapide et continu) ;
//  - cache des bandes dynamiques à plusieurs sections, chaque intervalle vérifié avant d'être interpolé.
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
    static constexpr int dynStep = 8;   // pas de reconception des bandes dynamiques

    // Mesures « avant » (1.x) : coefficients changés par paliers de 32 échantillons, gain dynamique
    // du sous-bloc précédent. Uniquement pour le banc de test.
    bool legacyStepping = false;
    // Nombre maximal de sections de correction par bande (0 = conceptions de la 2.0, pour les mesures « avant »).
    int corrections = maxCorrections;

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        for (auto& b : bands) b.reset();
        soloWet = 0; soloBandCur = -1;
    }

    // Cibles venant des paramètres (bloc audio). scale : échelle de gain (1 = 100 %).
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
        for (int off = 0; off < n; off += subBlock)
        {
            const int len = std::min (subBlock, n - off);
            float* l = L + off;
            float* r = R != nullptr ? R + off : nullptr;
            const float* sl = scL != nullptr ? scL + off : nullptr;
            const float* sr = scL != nullptr ? (scR != nullptr ? scR + off : sl) : nullptr;

            // Copie d'entrée pour l'écoute solo (signal avant égalisation)
            for (int i = 0; i < len; ++i) { inL[i] = l[i]; inR[i] = r != nullptr ? r[i] : l[i]; }

            for (int bi = 0; bi < maxBands; ++bi)
            {
                auto& b = bands[(size_t) bi];
                b.maxCorr = corrections;
                b.advance (fs, len, legacyStepping);
                if (b.wet <= 0.0 && b.wetTarget <= 0.0 && ! b.detecting) continue;
                processBand (b, l, r, len, bypassIIR, sl, sr);
            }
            if (! bypassIIR) solo (inL, inR, l, r, len);
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
        // conceptions : début (prev) et fin (cur) du sous-bloc ; coefficients de cur précalculés
        Design prev, cur;
        std::array<SectionRT, maxSections> rt {};
        int nsec = 0;
        bool interp = false;          // prev ≠ cur : coefficients interpolés à chaque échantillon
        bool fresh = true;            // pas encore de conception (première, ou nouvelle structure)
        double st[2][maxSections][2] {};
        double lastF = -1, lastG = 1e9, lastQ = -1, lastBase = 1e9;
        int maxCorr = maxCorrections;
        bool baseMoved = false;       // fréquence / gain / Q ont bougé dans ce sous-bloc

        // dynamique
        bool detecting = false, detValid = false;
        SectionRT det;
        double detSt[2][2] {};
        double env = 0, dynOffset = 0, dynPrev = 0;
        double envA = 0.99, envR = 0.999, coefAtk = -1, coefRel = -1;
        std::atomic<float> dynOut { 0.0f };

        void reset()
        {
            coefAtk = coefRel = -1;
            init = false; structure = false; wet = 0; wetTarget = 0; wetStart = 0; nsec = 0; detValid = false;
            env = 0; dynOffset = 0; dynPrev = 0; dynOut.store (0.0f);
            fresh = true; interp = false;
            clearStates();
        }
        void clearStates()
        {
            for (auto& c : st) for (auto& s : c) s[0] = s[1] = 0.0;
            for (auto& c : detSt) c[0] = c[1] = 0.0;
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
                    else if (! wantOn) { init = false; env = 0; dynOffset = 0; dynPrev = 0; dynOut.store (0.0f); }
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
            if (! detecting) { dynOffset = 0; dynPrev = 0; env = 0; detValid = false; dynOut.store (0.0f, std::memory_order_relaxed); }

            const double f = std::exp (fLog), q = std::exp (qLog);
            baseMoved = std::abs (f - lastF) > 1e-9 * f || std::abs (gDb - lastBase) > 1e-7 || std::abs (q - lastQ) > 1e-9 * q;
            if (detecting && (! detValid || baseMoved)) { toRT (listenSection (shape, f, q, fs), det); detValid = true; }

            // 1.x : la conception se faisait ici, avec le gain dynamique du sous-bloc précédent
            if (legacy || ! detecting) redesign (fs, dynOffset, legacy);
        }

        // Section neutre (traversée) gardant les pôles de s : sert d'extrémité quand une section de
        // correction apparaît ou disparaît, pour que la transition reste une interpolation.
        static Section neutral (Section s)
        {
            s.m0 = 1.0; s.m1 = 0.0; s.m2 = 0.0; s.cLp = 1.0; s.cHp = 1.0;
            return s;
        }
        static bool isNeutral (const Section& s)
        {
            return s.kind == Section::svf ? (s.m0 == 1.0 && s.m1 == 0.0 && s.m2 == 0.0) : (s.cLp == 1.0 && s.cHp == 1.0);
        }
        // Retire les sections neutres laissées en fin de chaîne par une transition terminée
        void trim()
        {
            while (cur.n > 0 && isNeutral (cur.s[(size_t) cur.n - 1])) --cur.n;
            nsec = cur.n;
        }

        // Nouvelle conception pour la fin du sous-bloc (gain de base lissé + décalage dynamique)
        void redesign (double fs, double dynDb, bool legacy, bool useCache = false)
        {
            const double f = std::exp (fLog), q = std::exp (qLog), g = gDb + dynDb;
            const bool changed = fresh || std::abs (f - lastF) > 1e-9 * f || std::abs (g - lastG) > 1e-7 || std::abs (q - lastQ) > 1e-9 * q;
            if (! changed) { if (interp) { prev = cur; interp = false; } trim(); return; }
            trim();
            BandSpec s; s.active = true; s.shape = shape; s.slope = slope; s.place = place;
            s.freq = f; s.q = q; s.gainDb = g;
            prev = cur;
            // la conception précédente sert de départ à la nouvelle (sections de correction : quelques
            // passes d'ajustement au lieu d'une recherche complète, et une solution qui suit les réglages)
            const Design* hint = fresh || legacy ? nullptr : &prev;
            if (useCache && ! baseMoved && ! fresh) designCached (s, fs, dynDb);
            else designMatched (s, fs, cur, maxCorr, hint);
            if (fresh || legacy || prev.n == 0 || cur.n == 0)
            {
                if (prev.n == 0) for (auto& c : st) for (auto& x : c) x[0] = x[1] = 0.0;   // (états laissés par un ancien réglage)
                prev = cur; interp = false;
            }
            else
            {
                // le nombre de sections change (correction qui apparaît / disparaît) : le côté le plus court
                // est complété par des sections neutres de mêmes pôles, et on interpole comme d'habitude
                for (int i = prev.n; i < cur.n; ++i)
                {
                    prev.s[(size_t) i] = neutral (cur.s[(size_t) i]);
                    st[0][i][0] = st[0][i][1] = st[1][i][0] = st[1][i][1] = 0.0;
                }
                for (int i = cur.n; i < prev.n; ++i) cur.s[(size_t) i] = neutral (prev.s[(size_t) i]);
                prev.n = cur.n = std::max (prev.n, cur.n);
                interp = true;
            }
            fresh = false;
            nsec = cur.n;
            for (int i = 0; i < nsec; ++i) toRT (cur.s[(size_t) i], rt[(size_t) i]);
            lastF = f; lastG = g; lastQ = q; lastBase = gDb;
        }

        // Bande dynamique : conceptions mises en cache tous les 0,25 dB sur la plage du gain dynamique
        // (fréquence, Q et gain de base fixés), le gain du moment est interpolé entre les deux voisines,
        // section par section (une section absente d'un côté y est neutre, avec les pôles de l'autre).
        // Chaque intervalle est vérifié à son premier usage : si la conception interpolée en son milieu
        // s'écarte de l'analogique (sections de correction trop différentes d'un bord à l'autre), cet
        // intervalle n'est pas interpolé : conception directe à chaque pas, en partant de la précédente.
        static constexpr int cacheSize = 128, cacheSecs = 1 + maxCorrections;   // 1 section + corrections
        static constexpr double cellTol = 0.0058;              // ≈ 0,025 dB
        struct Cached { Section s[cacheSecs]; int n = 0; double outGain = 1; };
        std::vector<Cached> cache = std::vector<Cached> ((size_t) cacheSize);
        std::array<signed char, cacheSize> cacheOk {}, cellOk {};   // intervalle : 0 inconnu, 1 interpolable, −1 non
        double cKeyF = -1, cKeyQ = -1, cKeyBase = 1e9, cKeyRange = 1e9, cMin = 0, cStep = 0.25;
        int cKeyShape = -1, cKeySlope = -1;

        static void lerpCached (const Cached& a, const Cached& b, double t, Design& o)
        {
            o.n = std::max (a.n, b.n);
            for (int j = 0; j < o.n; ++j)
            {
                const Section sa = j < a.n ? a.s[j] : neutral (b.s[j]);
                const Section sb = j < b.n ? b.s[j] : neutral (a.s[j]);
                auto& x = o.s[(size_t) j];
                x = sb;
                x.g = sa.g + (sb.g - sa.g) * t; x.k = sa.k + (sb.k - sa.k) * t;
                x.m0 = sa.m0 + (sb.m0 - sa.m0) * t; x.m1 = sa.m1 + (sb.m1 - sa.m1) * t; x.m2 = sa.m2 + (sb.m2 - sa.m2) * t;
            }
            o.outGain = a.outGain + (b.outGain - a.outGain) * t;
        }

        void designCached (const BandSpec& s, double fs, double dynDb)
        {
            // gain total nul : traversée exacte
            if (std::abs (s.gainDb) < 1e-9) { cur.n = 0; cur.outGain = 1.0; return; }
            const double range = dyn.rangeDb * scale;
            if (cKeyF != s.freq || cKeyQ != s.q || cKeyBase != gDb || cKeyRange != range || cKeyShape != (int) s.shape || cKeySlope != s.slope)
            {
                cKeyF = s.freq; cKeyQ = s.q; cKeyBase = gDb; cKeyRange = range; cKeyShape = (int) s.shape; cKeySlope = s.slope;
                cMin = std::min (0.0, range);
                cStep = std::max (0.25, std::abs (range) / (cacheSize - 2));
                cacheOk.fill (0); cellOk.fill (0);
            }
            const double pos = std::clamp ((dynDb - cMin) / cStep, 0.0, (double) (cacheSize - 2));
            const int i0 = (int) pos;
            const double t = pos - i0;
            Design tmp;
            for (int i : { i0, i0 + 1 })
                if (! cacheOk[(size_t) i])
                {
                    BandSpec e = s; e.gainDb = gDb + cMin + i * cStep;
                    // départ : une entrée voisine déjà conçue, sinon la conception en cours
                    Design h; h.n = 0;
                    for (int nb : { i - 1, i + 1 })
                        if (h.n == 0 && nb >= 0 && nb < cacheSize && cacheOk[(size_t) nb])
                        {
                            h.n = cache[(size_t) nb].n;
                            for (int j = 0; j < h.n; ++j) h.s[(size_t) j] = cache[(size_t) nb].s[j];
                        }
                    designMatched (e, fs, tmp, maxCorr, h.n > 0 ? &h : &prev);
                    if (tmp.n > cacheSecs) { designMatched (s, fs, cur, maxCorr, &prev); return; }   // (n'arrive pas : formes à gain)
                    auto& c = cache[(size_t) i];
                    c.n = tmp.n; c.outGain = tmp.outGain;
                    for (int j = 0; j < tmp.n; ++j) c.s[j] = tmp.s[(size_t) j];
                    cacheOk[(size_t) i] = 1;
                }
            const auto& a = cache[(size_t) i0];
            const auto& b = cache[(size_t) i0 + 1];
            if (cellOk[(size_t) i0] == 0)
            {
                BandSpec e = s; e.gainDb = gDb + cMin + (i0 + 0.5) * cStep;
                lerpCached (a, b, 0.5, tmp);
                cellOk[(size_t) i0] = designError (e, fs, tmp) <= cellTol ? 1 : -1;
            }
            if (cellOk[(size_t) i0] < 0) { designMatched (s, fs, cur, maxCorr, &prev); return; }
            lerpCached (a, b, t, cur);
        }

        // Chaîne de la bande pour un échantillon ; coef : coefficients (interpolés ou non)
        inline double chain (int ch, double x, const SectionRT* coef, double outGain)
        {
            double y = x;
            for (int i = 0; i < nsec; ++i) y = tick (coef[i], st[ch][i], y);
            return y * outGain;
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

    void processBand (Band& b, float* l, float* r, int len, bool bypassIIR, const float* scl, const float* scr)
    {
        const bool stereo = r != nullptr;
        const bool det = b.detecting;
        const bool ext = det && b.dyn.external && scl != nullptr;
        const bool scStereo = scr != nullptr && scr != scl;
        const double w0 = b.wetStart, w1 = b.wet;
        const bool filt = ! bypassIIR && (w0 > 0.0 || w1 > 0.0);
        const double dw = (w1 - w0) / len;
        // bande dynamique : reconception tous les 8 échantillons (le gain d'un détecteur rapide n'est pas
        // monotone sur 32 échantillons) ; sinon une fois par sous-bloc de 32
        const int step = det && ! legacyStepping ? dynStep : len;
        double dynDb[subBlock];

        for (int s0 = 0; s0 < len; s0 += step)
        {
            const int m = std::min (step, len - s0);
            if (det)
            {
                // 1) détecteur sur le pas (entrée de la bande, ou side-chain externe)
                for (int i = s0; i < s0 + m; ++i)
                {
                    if (ext) detect (b, scl[i], scStereo ? scr[i] : scl[i], scStereo || stereo);
                    else detect (b, l[i], stereo ? r[i] : l[i], stereo);
                }
                if (! legacyStepping) dynDb[s0 + m - 1] = dynCurve (20.0 * std::log10 (std::max (1.0e-9, b.env)), b.dyn, b.scale);
                if (! legacyStepping)
                {
                    // 2) conception pour la fin du pas
                    b.dynPrev = b.dynOffset;
                    b.dynOffset = dynDb[s0 + m - 1];
                    if (filt) b.redesign (fs, b.dynOffset, false, true);
                }
                else if (s0 + m == len)
                    b.dynOffset = dynCurve (20.0 * std::log10 (std::max (1.0e-9, b.env)), b.dyn, b.scale);   // 1.x : sous-bloc suivant
            }
            if (! filt) continue;

            // 3) filtrage ; coefficients interpolés à chaque échantillon si la conception a changé
            SectionRT now[maxSections];
            const SectionRT* coef = b.rt.data();
            const bool interp = b.interp;
            const bool byGain = false;
            const double gSpan = b.dynOffset - b.dynPrev;
            double og = b.cur.outGain;
            for (int i = s0; i < s0 + m; ++i)
            {
                const double w = w0 + dw * (i + 1);
                if (interp)
                {
                    // position entre prev et cur : celle du gain de l'échantillon (dynamique), sinon le temps
                    double t = byGain ? (dynDb[i] - b.dynPrev) / gSpan : (double) (i - s0 + 1) / m;
                    t = std::clamp (t, 0.0, 1.0);
                    for (int sct = 0; sct < b.nsec; ++sct) lerpRT (b.prev.s[(size_t) sct], b.cur.s[(size_t) sct], t, now[sct]);
                    coef = now;
                    og = b.prev.outGain + (b.cur.outGain - b.prev.outGain) * t;
                }
                const double xl = l[i], xr = stereo ? r[i] : xl;
                switch (b.place)
                {
                    case Place::stereo:
                        l[i] = (float) (xl + w * (b.chain (0, xl, coef, og) - xl));
                        if (stereo) r[i] = (float) (xr + w * (b.chain (1, xr, coef, og) - xr));
                        break;
                    case Place::left:
                    case Place::right:
                    {
                        const bool left = b.place == Place::left;
                        if (! stereo && ! left) break;     // mono : la voie droite n'existe pas
                        const double x = left ? xl : xr;
                        const double y = x + w * (b.chain (0, x, coef, og) - x);
                        if (left) l[i] = (float) y; else r[i] = (float) y;
                        break;
                    }
                    case Place::mid:
                    case Place::side:
                    {
                        if (! stereo)
                        {
                            if (b.place == Place::side) break;     // mono : pas de côtés
                            l[i] = (float) (xl + w * (b.chain (0, xl, coef, og) - xl));
                            break;
                        }
                        double mm = 0.5 * (xl + xr), ss = 0.5 * (xl - xr);
                        double& v = b.place == Place::mid ? mm : ss;
                        v = v + w * (b.chain (0, v, coef, og) - v);
                        l[i] = (float) (mm + ss);
                        r[i] = (float) (mm - ss);
                        break;
                    }
                }
            }
            if (interp) { b.prev = b.cur; b.interp = false; }
        }
        if (det) b.dynOut.store ((float) b.dynOffset, std::memory_order_relaxed);
    }

    double fs = 48000.0;
    std::array<Band, maxBands> bands;
    std::atomic<int> soloTarget { -1 };
    int soloBandCur = -1;
    double soloWet = 0;
    double soloSt[2][2] {};
    float inL[subBlock] {}, inR[subBlock] {};
};
} // namespace audiolab::eq
