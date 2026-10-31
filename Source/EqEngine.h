// Audio Lab EQ — moteur temps réel (mode zéro latence) :
// chaîne de 24 bandes SVF, EQ dynamique par bande, placement Stéréo / G / D / Mid / Side,
// écoute (solo) d'une bande, transitions sans clic.
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
};

class Engine
{
public:
    static constexpr int subBlock = 32;

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

    // Décalage de gain dynamique actuel (dB), lu par l'interface
    float getDynOffset (int i) const { return bands[(size_t) i].dynOut.load (std::memory_order_relaxed); }

    // Traitement en place. R peut être nul (mono). bypassIIR : n'applique pas la chaîne IIR
    // (mode phase linéaire) mais fait tourner les détecteurs ; l'écoute solo se fait alors à part (solo()).
    void process (float* L, float* R, int n, bool bypassIIR)
    {
        for (int off = 0; off < n; off += subBlock)
        {
            const int len = std::min (subBlock, n - off);
            float* l = L + off;
            float* r = R != nullptr ? R + off : nullptr;

            // Copie d'entrée pour l'écoute solo (signal avant égalisation)
            for (int i = 0; i < len; ++i) { inL[i] = l[i]; inR[i] = r != nullptr ? r[i] : l[i]; }

            for (int bi = 0; bi < maxBands; ++bi)
            {
                auto& b = bands[(size_t) bi];
                b.advance (fs, len);
                if (b.wet <= 0.0 && b.wetTarget <= 0.0 && ! b.detecting) continue;
                processBand (b, l, r, len, bypassIIR);
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
        Design des;
        std::array<SectionRT, 9> rt {};
        int nsec = 0;
        double outGain = 1.0;
        double st[2][9][2] {};
        double lastF = -1, lastG = 1e9, lastQ = -1;

        // dynamique
        bool detecting = false, detValid = false;
        SectionRT det;
        double detSt[2][2] {};
        double env = 0, dynOffset = 0;
        double envA = 0.99, envR = 0.999, coefAtk = -1, coefRel = -1;
        std::atomic<float> dynOut { 0.0f };

        void reset()
        {
            coefAtk = coefRel = -1;
            init = false; structure = false; wet = 0; wetTarget = 0; wetStart = 0; nsec = 0; detValid = false;
            env = 0; dynOffset = 0; dynOut.store (0.0f);
            clearStates();
        }
        void clearStates()
        {
            for (auto& c : st) for (auto& s : c) s[0] = s[1] = 0.0;
            for (auto& c : detSt) c[0] = c[1] = 0.0;
        }

        // Prépare un sous-bloc : lissage des réglages, transitions de structure, coefficients.
        void advance (double fs, int len)
        {
            const bool wantOn = target.active;
            const bool structChanged = structure && (target.shape != shape || target.slope != slope || target.place != place);

            if (! init && wantOn)
            {
                fLog = std::log (target.freq); gDb = target.gainDb * scale; qLog = std::log (target.q);
                shape = target.shape; slope = target.slope; place = target.place;
                structure = true; init = true; clearStates();
                lastF = -1;
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
                        clearStates(); lastF = -1;
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
            if (std::abs (tf - fLog) < 1e-6) fLog = tf;
            if (std::abs (tq - qLog) < 1e-6) qLog = tq;
            if (std::abs (tg - gDb) < 1e-6) gDb = tg;

            detecting = dynOk && wantOn;
            if (! detecting) { dynOffset = 0; env = 0; detValid = false; dynOut.store (0.0f, std::memory_order_relaxed); }

            const double f = std::exp (fLog), q = std::exp (qLog), g = gDb + dynOffset;
            if (std::abs (f - lastF) > 1e-9 * f || std::abs (g - lastG) > 1e-7 || std::abs (q - lastQ) > 1e-9 * q)
            {
                BandSpec s; s.active = true; s.shape = shape; s.slope = slope; s.place = place;
                s.freq = f; s.q = q; s.gainDb = g;
                design (s, fs, des, false);
                nsec = des.n; outGain = des.outGain;
                for (int i = 0; i < nsec; ++i) toRT (des.s[(size_t) i], rt[(size_t) i]);
                lastF = f; lastG = g; lastQ = q;
                detValid = false;
            }
            if (detecting && ! detValid) { toRT (listenSection (shape, f, q, fs), det); detValid = true; }
        }

        inline double chain (int ch, double x)
        {
            double y = x;
            for (int i = 0; i < nsec; ++i) y = tick (rt[(size_t) i], st[ch][i], y);
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

    void processBand (Band& b, float* l, float* r, int len, bool bypassIIR)
    {
        const double w0 = b.wetStart, w1 = b.wet;
        const bool iir = ! bypassIIR && (w0 > 0.0 || w1 > 0.0);
        const bool stereo = r != nullptr;
        const double dw = (w1 - w0) / len;
        double w = w0;
        const bool det = b.detecting;

        for (int i = 0; i < len; ++i)
        {
            w += dw;
            const double xl = l[i], xr = stereo ? r[i] : xl;
            switch (b.place)
            {
                case Place::stereo:
                {
                    if (det)
                    {
                        const double dl = tick (b.det, b.detSt[0], xl);
                        const double dr = stereo ? tick (b.det, b.detSt[1], xr) : dl;
                        follow (b, std::max (std::abs (dl), std::abs (dr)));
                    }
                    if (iir)
                    {
                        l[i] = (float) (xl + w * (b.chain (0, xl) - xl));
                        if (stereo) r[i] = (float) (xr + w * (b.chain (1, xr) - xr));
                    }
                    break;
                }
                case Place::left:
                case Place::right:
                {
                    const bool left = b.place == Place::left;
                    if (! stereo && ! left) break;     // mono : la voie droite n'existe pas
                    const double x = left ? xl : xr;
                    if (det) follow (b, tick (b.det, b.detSt[0], x));
                    if (iir)
                    {
                        const double y = x + w * (b.chain (0, x) - x);
                        if (left) l[i] = (float) y; else r[i] = (float) y;
                    }
                    break;
                }
                case Place::mid:
                case Place::side:
                {
                    if (! stereo)
                    {
                        if (b.place == Place::side) break;     // mono : pas de côtés
                        if (det) follow (b, tick (b.det, b.detSt[0], xl));
                        if (iir) l[i] = (float) (xl + w * (b.chain (0, xl) - xl));
                        break;
                    }
                    double m = 0.5 * (xl + xr), s = 0.5 * (xl - xr);
                    double& v = b.place == Place::mid ? m : s;
                    if (det) follow (b, tick (b.det, b.detSt[0], v));
                    if (iir)
                    {
                        v = v + w * (b.chain (0, v) - v);
                        l[i] = (float) (m + s);
                        r[i] = (float) (m - s);
                    }
                    break;
                }
            }
        }

        if (det)
        {
            const double lev = 20.0 * std::log10 (std::max (1.0e-9, b.env));
            b.dynOffset = dynCurve (lev, b.dyn, b.scale);
            b.dynOut.store ((float) b.dynOffset, std::memory_order_relaxed);
        }
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
