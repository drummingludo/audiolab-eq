// Audio Lab EQ — Auto gain à sonie égale (3.0), sans JUCE.
//
// But : quand Auto gain est en service, la sonie intégrée (ITU-R BS.1770) de la sortie est celle de
// l'entrée. Le gain de compensation G (dB) vient de deux sources :
//  1. la MESURE : énergies pondérées K de l'entrée (recalée sur la sortie) et de la sortie avant gain,
//     par tranches de 100 ms ; porte absolue à −70 LUFS (tranche et bloc de 400 ms, comme la norme),
//     somme à oubli exponentiel (10 s en régime établi : c'est une sonie intégrée glissante, qui ne
//     pompe pas ; elle repart de zéro après un changement de courbe et vaut alors la moyenne depuis
//     ce changement). G rejoint l'écart mesuré avec une constante de temps de 1 s.
//  2. la PRÉVISION : spectre moyen de l'entrée (FFT, 3 s de moyenne) × courbe de l'égaliseur, pondéré K.
//     Quand une bande bouge, G est décalé tout de suite de (prévision nouvelle courbe − prévision
//     ancienne courbe) : pas d'attente, pas de saut à retardement ; la mesure repart ensuite et
//     corrige ce que la prévision ne voit pas (bandes dynamiques, résolution du spectre).
// Le gain ne bouge pas pendant un silence : une tranche ne compte que si l'entrée dépasse la porte, et
// G n'est mis à jour que si le dernier échantillon de la tranche n'est pas un silence numérique.
// G est borné à ± 24 dB ; le lissage sans clic (30 ms) est fait par l'appelant.
//
// Fil audio : rien n'alloue après prepare(). La FFT du spectre est étalée sur les échantillons
// (quelques papillons par échantillon), pour ne jamais faire de pointe de calcul dans un bloc.
#pragma once
#include "FirDesign.h"
#include "Loudness.h"
#include <atomic>

namespace audiolab::eq
{
// Méthode des versions 1.x et 2.0 (pour les mesures « avant » du banc) : moyenne de la courbe statique
// en dB, de 40 Hz à 16 kHz en échelle log, demi-poids pour les bandes Mid / Side / G / D.
inline double legacyAutoGainDb (const std::array<BandSpec, maxBands>& specs, double fs)
{
    constexpr int np = 48;
    double sum = 0;
    Design d;
    for (const auto& s : specs)
    {
        if (! s.active) continue;
        design (s, fs, d, false);
        const double w = s.place == Place::stereo ? 1.0 : 0.5;
        for (int i = 0; i < np; ++i)
        {
            const double f = 40.0 * std::pow (16000.0 / 40.0, (i + 0.5) / np);
            sum += w * 20.0 * std::log10 (std::max (1e-6, std::abs (response (d, f, fs, false))));
        }
    }
    return std::clamp (-sum / np, -24.0, 24.0);
}

class AutoGain
{
public:
    static constexpr int maxPts = 96;
    static constexpr double limitDb = 24.0;
    static constexpr double hopSeconds = 0.1;        // tranche de mesure
    static constexpr double windowSeconds = 10.0;    // oubli de la sonie intégrée glissante
    static constexpr double trackSeconds = 1.0;      // G rejoint la mesure
    static constexpr double spectrumSeconds = 3.0;   // moyenne du spectre de l'entrée
    static constexpr double settleSeconds = 0.1;     // après un changement de courbe (lissage du moteur)

    // Fil audio arrêté
    void prepare (double sampleRate)
    {
        fs = sampleRate;
        hop = std::max (1, (int) std::lround (hopSeconds * fs));
        lambda = std::exp (-hopSeconds / windowSeconds);
        alphaSlow = 1.0 - std::exp (-hopSeconds / trackSeconds);
        quietMin = 1;
        for (auto& k : kIn) k.set (fs);
        for (auto& k : kOut) k.set (fs);

        // FFT du spectre : 8192 points à 44,1 / 48 kHz (5,9 Hz par raie), ×2 et ×4 au-delà
        N = 8192 * fir::rateMult (fs);
        stages = 0; while ((1 << stages) < N) ++stages;
        win.resize ((size_t) N); rev.resize ((size_t) N);
        cosT.resize ((size_t) N / 2); sinT.resize ((size_t) N / 2);
        for (int i = 0; i < N; ++i)
        {
            win[(size_t) i] = (float) (0.5 - 0.5 * std::cos (2.0 * pi * i / N));
            int r = 0; for (int b = 0; b < stages; ++b) if (i & (1 << b)) r |= 1 << (stages - 1 - b);
            rev[(size_t) i] = r;
        }
        for (int i = 0; i < N / 2; ++i) { cosT[(size_t) i] = (float) std::cos (2.0 * pi * i / N); sinT[(size_t) i] = (float) std::sin (2.0 * pi * i / N); }
        for (auto* v : { &col[0], &col[1], &src[0], &src[1], &re[0], &re[1], &im[0], &im[1] }) v->assign ((size_t) N, 0.0f);
        budget = (N + stages * (N / 2) + N / 2 + 1) / N + 2;
        cSpec = 1.0 - std::exp (-((double) N / fs) / spectrumSeconds);

        // bandes d'analyse : un sixième d'octave, jamais plus étroites qu'une raie, de 2 raies à Nyquist
        const double binW = fs / N, ratio = std::pow (2.0, 1.0 / 6.0);
        nPts = 0;
        double lo = 2.0 * binW;
        binBand.assign ((size_t) N / 2 + 1, (short) -1);
        while (nPts < maxPts && lo < 0.5 * fs)
        {
            double hi = std::max (lo * ratio, lo + binW);
            if (nPts == maxPts - 1 || hi > 0.5 * fs) hi = 0.5 * fs + binW;
            const int k0 = (int) std::ceil (lo / binW), k1 = std::min (N / 2, (int) std::ceil (hi / binW) - 1);
            for (int k = k0; k <= k1; ++k) binBand[(size_t) k] = (short) nPts;
            const double top = std::min (hi, 0.5 * fs);
            freq[(size_t) nPts] = std::sqrt (lo * top);
            prior[(size_t) nPts] = std::log (top / lo);   // spectre rose : même puissance par octave
            kw[(size_t) nPts] = std::pow (10.0, loud::kGainDb (freq[(size_t) nPts], fs) / 10.0);
            ++nPts;
            lo = hi;
        }
        for (auto& b : bandOk) b = false;
        curveValid = false;
        for (int i = 0; i < maxPts; ++i) { qLL[(size_t) i] = 1.0; qRR[(size_t) i] = 1.0; qXr[(size_t) i] = 0.0; qXi[(size_t) i] = 0.0; }
        running = false;
        resetSpectrum();
        restart();
        g = 0.0; publish();
    }

    // Auto gain mis en service ou coupé (fil audio). En service : la mesure repart, G part de la prévision.
    void setRunning (bool on)
    {
        if (on == running) return;
        running = on;
        restart();
        g = std::clamp (predict (qLL, qRR, qXr, qXi), -limitDb, limitDb);
        publish();
    }
    bool isRunning() const { return running; }

    // Courbe de l'égaliseur (gains de base, échelle de gain comprise). Fil audio ; ne recalcule que les
    // bandes qui ont changé. linearPhase : la cible est le module seul. Renvoie vrai si la courbe a changé.
    bool setCurve (const std::array<BandSpec, maxBands>& bands, bool stereo, bool linearPhase)
    {
        bool changed = ! curveValid || stereo != curveStereo || linearPhase != curveLinear;
        for (int b = 0; b < maxBands; ++b)
        {
            const auto& s = bands[(size_t) b];
            const auto& c = cache[(size_t) b];
            if (bandOk[(size_t) b] && same (s, c)) continue;
            changed = true;
            cache[(size_t) b] = s; bandOk[(size_t) b] = true;
            if (! s.active) continue;
            Design d;
            design (s, fs, d, true);
            for (int i = 0; i < nPts; ++i) H[(size_t) b][(size_t) i] = response (d, freq[(size_t) i], fs, true);
        }
        if (! changed) return false;

        std::array<double, maxPts> nLL, nRR, nXr, nXi;
        for (int i = 0; i < nPts; ++i)
        {
            fir::M2 m { 1.0, 0.0, 0.0, 1.0 };
            for (int b = 0; b < maxBands; ++b)
            {
                if (! cache[(size_t) b].active) continue;
                fir::cd h = H[(size_t) b][(size_t) i];
                if (linearPhase) h = fir::cd (std::abs (h), 0.0);
                m = fir::mul (fir::bandMatrix (cache[(size_t) b].place, h, stereo), m);
            }
            // puissance de sortie = gLL·P_L + gRR·P_R + 2·Re (x · C), C = ⟨L·R*⟩
            const fir::cd x = m.a * std::conj (m.b) + m.c * std::conj (m.d);
            nLL[(size_t) i] = std::norm (m.a) + (stereo ? std::norm (m.c) : 0.0);
            nRR[(size_t) i] = stereo ? std::norm (m.b) + std::norm (m.d) : 0.0;
            nXr[(size_t) i] = stereo ? x.real() : 0.0;
            nXi[(size_t) i] = stereo ? x.imag() : 0.0;
        }
        const bool first = ! curveValid;
        const double before = predict (qLL, qRR, qXr, qXi), after = predict (nLL, nRR, nXr, nXi);
        qLL = nLL; qRR = nRR; qXr = nXr; qXi = nXi;
        curveValid = true; curveStereo = stereo; curveLinear = linearPhase;
        if (running && ! first)
        {
            // décalage immédiat d'après la prévision ; la mesure repart de zéro pour la nouvelle courbe
            g = std::clamp (g + after - before, -limitDb, limitDb);
            eInSum = eOutSum = 0.0; nAcc = 0;
            hold (settleSeconds);
        }
        else g = std::clamp (after, -limitDb, limitDb);
        publish();
        return true;
    }

    // Suspend la mesure (changement de mode de phase, FIR en cours de conception, écoute solo)
    void hold (double seconds) { holdLeft = std::max (holdLeft, (long) std::lround (seconds * fs)); }
    // Oublie la mesure en cours sans toucher au gain (l'alignement entrée / sortie a changé)
    void forget() { eInSum = eOutSum = 0.0; nAcc = 0; nRing = 0; }

    // Fil audio. in : entrée recalée sur la sortie (retardée de la latence) ; out : sortie de l'égaliseur
    // avant le gain de sortie. inR / outR nuls : mono.
    void process (const float* inL, const float* inR, const float* outL, const float* outR, int n)
    {
        if (! running) return;
        const bool stereo = inR != nullptr && outR != nullptr;
        for (int i = 0; i < n; ++i)
        {
            const float xl = inL[i], xr = stereo ? inR[i] : 0.0f;
            quiet = (std::abs (xl) + std::abs (xr) < 1.0e-5f) ? quiet + 1 : 0;
            double a = kIn[0].tick (xl), o = kOut[0].tick (outL[i]);
            double ei = a * a, eo = o * o;
            if (stereo)
            {
                a = kIn[1].tick (xr); o = kOut[1].tick (outR[i]);
                ei += a * a; eo += o * o;
            }
            pushSpectrum (xl, xr);
            if (holdLeft > 0) { --holdLeft; eIn = eOut = 0.0; hopPos = 0; continue; }
            eIn += ei; eOut += eo;
            if (++hopPos >= hop) endHop();
        }
    }

    // Gain de compensation actuel (dB). Lisible de tous les fils.
    float gainDb() const { return gOut.load (std::memory_order_relaxed); }
    // Dernier écart de sonie mesuré entrée − sortie (dB), et prévision pour la courbe en place (diagnostic, tests)
    float measuredDb() const { return measOut.load (std::memory_order_relaxed); }
    double predictedDb() const { return predict (qLL, qRR, qXr, qXi); }
    int spectrumFrames() const { return frames; }
    int unitsPerSample() const { return budget; }

private:
    static bool same (const BandSpec& a, const BandSpec& b)
    {
        if (a.active != b.active) return false;
        if (! a.active) return true;
        return a.shape == b.shape && a.slope == b.slope && a.place == b.place && a.freq == b.freq && a.gainDb == b.gainDb
            && a.q == b.q && a.slopeFree == b.slopeFree && a.brick == b.brick;
    }

    void publish() { gOut.store ((float) g, std::memory_order_relaxed); }

    void restart()
    {
        for (auto& k : kIn) k.reset();
        for (auto& k : kOut) k.reset();
        eIn = eOut = 0.0; hopPos = 0; holdLeft = 0;
        eInSum = eOutSum = 0.0; nAcc = 0; nRing = 0; ringPos = 0; nUpd = 0; quiet = 0;
        measOut.store (0.0f);
    }

    void resetSpectrum()
    {
        for (int i = 0; i < maxPts; ++i) { pL[(size_t) i] = pR[(size_t) i] = prior[(size_t) i]; cRe[(size_t) i] = cIm[(size_t) i] = 0.0; }
        frames = 0; fill = 0; phase = 0;
    }

    // Écart de sonie prévu entrée − sortie (dB) pour une courbe, avec le spectre moyen de l'entrée
    double predict (const std::array<double, maxPts>& gLL, const std::array<double, maxPts>& gRR,
                    const std::array<double, maxPts>& xr, const std::array<double, maxPts>& xi) const
    {
        if (! curveValid) return 0.0;
        double in = 0.0, out = 0.0;
        for (int i = 0; i < nPts; ++i)
        {
            const double l = pL[(size_t) i], r = curveStereo ? pR[(size_t) i] : 0.0;
            in += kw[(size_t) i] * (l + r);
            out += kw[(size_t) i] * std::max (0.0, gLL[(size_t) i] * l + gRR[(size_t) i] * r
                                                   + 2.0 * (xr[(size_t) i] * cRe[(size_t) i] - xi[(size_t) i] * cIm[(size_t) i]));
        }
        if (in <= 0.0 || out <= 0.0) return 0.0;
        return 10.0 * std::log10 (in / out);
    }

    void endHop()
    {
        const double hi = eIn / hop, ho = eOut / hop;
        eIn = eOut = 0.0; hopPos = 0;
        ring[ringPos] = hi; ringPos = (ringPos + 1) & 3; nRing = std::min (4, nRing + 1);
        double blk = 0.0; for (int i = 0; i < nRing; ++i) blk += ring[i];
        blk /= nRing;
        // porte absolue : la tranche et le bloc de 400 ms qui la contient
        if (loud::lufs (hi) <= loud::absGateLufs || loud::lufs (blk) <= loud::absGateLufs) return;
        eInSum = lambda * eInSum + hi; eOutSum = lambda * eOutSum + ho;
        if (++nAcc < 4 || eOutSum <= 0.0 || quiet >= quietMin) return;
        const double meas = std::clamp (10.0 * std::log10 (eInSum / eOutSum), -limitDb, limitDb);
        measOut.store ((float) meas, std::memory_order_relaxed);
        // première mesure après la mise en service : prise telle quelle, puis moyenne, puis suivi lent
        const double alpha = std::max (alphaSlow, 1.0 / (nUpd + 1.0));
        if (nUpd < 1000) ++nUpd;
        g += (meas - g) * alpha;
        publish();
    }

    // ── spectre moyen de l'entrée : FFT étalée ─────────────────────────────
    inline void pushSpectrum (float l, float r)
    {
        col[0][(size_t) fill] = l; col[1][(size_t) fill] = r;
        if (++fill == N)
        {
            fill = 0;
            if (phase == 0) { std::swap (col[0], src[0]); std::swap (col[1], src[1]); phase = 1; idx = 0; frameE = 0.0; }
        }
        if (phase != 0) step (budget);
    }

    void step (int units)
    {
        while (units-- > 0)
        {
            if (phase == 1)
            {
                // fenêtre de Hann, rangement dans l'ordre de la FFT
                const size_t i = (size_t) idx, j = (size_t) rev[i];
                const float w = win[i], a = src[0][i], b = src[1][i];
                re[0][j] = a * w; im[0][j] = 0.0f; re[1][j] = b * w; im[1][j] = 0.0f;
                frameE += (double) a * a + (double) b * b;
                if (++idx == N) { phase = 2; len = 2; bi = 0; bj = 0; }
            }
            else if (phase == 2)
            {
                const int half = len >> 1;
                const size_t t = (size_t) (bj * (N / len)), p = (size_t) (bi + bj), q = p + (size_t) half;
                const float wr = cosT[t], wi = -sinT[t];
                for (int c = 0; c < 2; ++c)
                {
                    const float vr = re[c][q] * wr - im[c][q] * wi, vi = re[c][q] * wi + im[c][q] * wr;
                    const float ur = re[c][p], ui = im[c][p];
                    re[c][p] = ur + vr; im[c][p] = ui + vi; re[c][q] = ur - vr; im[c][q] = ui - vi;
                }
                if (++bj == half)
                {
                    bj = 0; bi += len;
                    if (bi >= N)
                    {
                        bi = 0; len <<= 1;
                        if (len > N)
                        {
                            phase = 3; idx = 1;
                            for (int i = 0; i < nPts; ++i) aL[(size_t) i] = aR[(size_t) i] = aRe[(size_t) i] = aIm[(size_t) i] = 0.0;
                        }
                    }
                }
            }
            else if (phase == 3)
            {
                const size_t k = (size_t) idx;
                const int b = binBand[k];
                if (b >= 0)
                {
                    const double lr = re[0][k], li = im[0][k], rr = re[1][k], ri = im[1][k];
                    aL[(size_t) b] += lr * lr + li * li; aR[(size_t) b] += rr * rr + ri * ri;
                    aRe[(size_t) b] += lr * rr + li * ri; aIm[(size_t) b] += li * rr - lr * ri;
                }
                if (++idx > N / 2) { finishFrame(); return; }
            }
            else return;
        }
    }

    void finishFrame()
    {
        phase = 0;
        if (frameE / N < 1.0e-8) return;   // trame presque muette (−80 dBFS) : le spectre moyen est gardé
        if (frames < 1000000) ++frames;
        const double c = std::max (1.0 / frames, cSpec);
        if (frames == 1) for (int i = 0; i < nPts; ++i) { pL[(size_t) i] = pR[(size_t) i] = 0.0; cRe[(size_t) i] = cIm[(size_t) i] = 0.0; }
        for (int i = 0; i < nPts; ++i)
        {
            pL[(size_t) i] += c * (aL[(size_t) i] - pL[(size_t) i]); pR[(size_t) i] += c * (aR[(size_t) i] - pR[(size_t) i]);
            cRe[(size_t) i] += c * (aRe[(size_t) i] - cRe[(size_t) i]); cIm[(size_t) i] += c * (aIm[(size_t) i] - cIm[(size_t) i]);
        }
    }

    double fs = 48000.0;
    bool running = false;
    // mesure
    loud::KFilter kIn[2], kOut[2];
    int hop = 4800, hopPos = 0, quiet = 0, quietMin = 1;
    long holdLeft = 0;
    double eIn = 0, eOut = 0, eInSum = 0, eOutSum = 0, lambda = 0.99, alphaSlow = 0.1;
    double ring[4] {};
    int ringPos = 0, nRing = 0, nAcc = 0, nUpd = 0;
    double g = 0.0;
    std::atomic<float> gOut { 0.0f }, measOut { 0.0f };
    // courbe
    std::array<BandSpec, maxBands> cache {};
    std::array<bool, maxBands> bandOk {};
    std::array<std::array<fir::cd, maxPts>, maxBands> H {};
    std::array<double, maxPts> qLL {}, qRR {}, qXr {}, qXi {};
    bool curveValid = false, curveStereo = true, curveLinear = false;
    // spectre
    int N = 8192, stages = 13, nPts = 0, budget = 10, fill = 0, phase = 0, idx = 0, len = 2, bi = 0, bj = 0, frames = 0;
    double cSpec = 0.05, frameE = 0.0;
    std::vector<float> win, cosT, sinT, col[2], src[2], re[2], im[2];
    std::vector<int> rev;
    std::vector<short> binBand;
    std::array<double, maxPts> freq {}, prior {}, kw {}, pL {}, pR {}, cRe {}, cIm {}, aL {}, aR {}, aRe {}, aIm {};
};
} // namespace audiolab::eq
