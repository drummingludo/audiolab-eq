// Audio Lab EQ — modes FIR « Linear phase » et « Natural phase ».
// Conception des FIR : FirDesign.h (fil de conception). Lecture : convolution par partitions
// uniformes (UPOLS) écrite pour ce plugin, au lieu de juce::dsp::Convolution :
//  - l'historique d'entrée (spectres des blocs passés) est COMMUN à l'ancien et au nouveau FIR :
//    quand le FIR change (réglage, EQ dynamique), on passe de l'un à l'autre par un fondu sans
//    trou (juce::dsp::Convolution repartait d'un historique vide : la sortie du nouveau FIR manquait
//    pendant la moitié de sa longueur, soit un creux de plusieurs dizaines de ms en phase linéaire) ;
//  - matrice 2×2 (Mid / Side) : 2 FFT d'entrée, 2 FFT inverses, 4 produits seulement si besoin ;
//  - latence = un bloc de partition P (256 éch. à 48 kHz), ajoutée à celle du FIR et annoncée.
// Aucune allocation dans le fil audio : tout est réservé dans prepare() ; les jeux de FIR sont créés
// sur le fil de conception et transmis sans verrou (pointeurs atomiques).
#pragma once
#include <juce_dsp/juce_dsp.h>
#include "FirDesign.h"
#include <mutex>

namespace audiolab::eq
{
class LinearPhase
{
public:
    struct Snapshot
    {
        std::array<BandSpec, maxBands> bands;
        bool stereo = true;
    };

    // Courbe réelle du FIR pour l'écran (grille logarithmique fixe)
    static constexpr int displayPoints = 512;
    struct DisplayData
    {
        int version = 0;
        int mode = -1, N = 0;
        bool valid = false, matrix = false;
        std::array<float, displayPoints> freq {}, lr0 {}, lr1 {}, ms0 {}, ms1 {};
    };

    LinearPhase() { for (auto& t : trash) t.store (nullptr); }
    ~LinearPhase()
    {
        dropOwned();
        delete pending.exchange (nullptr);
        for (auto& t : trash) delete t.exchange (nullptr);
    }

    // Fil audio arrêté (prepareToPlay) : réserve tout pour la longueur maximale à cette fréquence
    void prepare (double sampleRate)
    {
        const std::lock_guard<std::mutex> lock (designMutex);
        dropOwned();
        delete pending.exchange (nullptr);
        for (auto& t : trash) delete t.exchange (nullptr);
        fs = sampleRate;
        P = fir::partitionSize (fs);
        D = fir::naturalDelay (fs);
        order = 0; while ((1 << order) < 2 * P) ++order;
        fftRT = std::make_unique<juce::dsp::FFT> (order);
        fftDesign = std::make_unique<juce::dsp::FFT> (order);
        {
            // facteur d'échelle de la FFT inverse (dépend de l'implémentation : vDSP, IPP, interne…)
            std::vector<float> d ((size_t) 4 * P, 0.0f);
            for (int k = 0; k <= P; ++k) d[(size_t) (2 * k)] = 1.0f;
            fftRT->performRealOnlyInverseTransform (d.data());
            invScale = std::abs (d[0]) > 1e-12f ? 1.0f / d[0] : 1.0f;
        }
        Kmax = (fir::maxLength (fs) + P - 1) / P;
        bins = 2 * P + 2;   // P + 1 complexes entrelacés
        for (auto& f : fdl) f.assign ((size_t) Kmax * (size_t) bins, 0.0f);
        for (auto* v : { &lastIn, &inBuf, &outBuf, &outOld })
            for (auto& c : *v) c.assign ((size_t) P, 0.0f);
        work.assign ((size_t) 4 * P, 0.0f);
        acc.assign ((size_t) bins, 0.0f);

        // FIR neutres (retard pur) de toutes les longueurs, prêts d'avance
        int N = fir::minLength (fs);
        for (int i = 0; i < 4; ++i, N *= 2)
            neutralLin[(size_t) i] = makeNeutral ((int) fir::Mode::linear, N, N / 2);
        neutralNat = makeNeutral ((int) fir::Mode::natural, 0, D);
        mode = fir::Mode::linear; activeN = fir::minLength (fs);
        clearHistory();
        cur = neutralLin[0];
        prepared.store (true);
    }

    int partition() const { return P; }
    int naturalDelaySamples() const { return D; }
    double sampleRate() const { return fs; }
    // latence totale d'un réglage (FIR + bloc de partition)
    int latencyFor (fir::Mode m, int N) const { return (m == fir::Mode::linear ? N / 2 : D) + P; }

    // Fil audio : nouveau réglage (mode / longueur), pendant le silence d'un changement de latence.
    // L'historique d'entrée est gardé (il est tenu à jour même en zéro latence, voir feed()) : le
    // nouveau FIR a tout de suite un passé complet. Installe le FIR conçu s'il est déjà là, sinon un neutre.
    void switchTo (fir::Mode m, int N)
    {
        mode = m; activeN = N;
        if (old != nullptr) { release (old); old = nullptr; xfade = 0; }
        if (auto* p = pending.exchange (nullptr))
        {
            if (matches (*p)) { setCurrent (p); return; }
            if (! toTrash (p)) delete p;   // (ne devrait pas arriver : 8 places)
        }
        setCurrent (m == fir::Mode::linear ? neutralLin[(size_t) indexOfLength (N)] : neutralNat);
    }

    // Vrai quand le FIR en place correspond au réglage actif (conçu, pas neutre)
    bool ready() const { return cur != nullptr && ! cur->neutral && matches (*cur); }

    // Fil de conception : calcule les FIR et les dépose pour le fil audio.
    void design (const Snapshot& snap, fir::Mode m, int N, bool wantDisplay)
    {
        const std::lock_guard<std::mutex> lock (designMutex);
        if (! prepared.load()) return;
        fir::Request rq;
        rq.bands = snap.bands; rq.stereo = snap.stereo; rq.mode = m; rq.N = N; rq.delay = D; rq.fs = fs;
        fir::design (rq, result, work2);
        auto* set = makeSet (result, (int) m, N, false);
        if (auto* replaced = pending.exchange (set)) delete replaced;
        if (wantDisplay) makeDisplay (m, N);
    }

    // Fil de conception : libère les jeux déjà remplacés
    void collectGarbage() { for (auto& t : trash) delete t.exchange (nullptr); }

    void getDisplay (DisplayData& out) const
    {
        const std::lock_guard<std::mutex> lock (displayMutex);
        out = display;
    }
    int displayVersion() const { return displayVer.load(); }

    // Fil audio, mode zéro latence : tient seulement l'historique d'entrée à jour (2 FFT par bloc)
    void feed (const float* L, const float* R, int n)
    {
        const int nch = R != nullptr ? 2 : 1;
        const float* ch[2] = { L, R };
        for (int i = 0; i < n; ++i)
        {
            for (int c = 0; c < nch; ++c) inBuf[(size_t) c][(size_t) pos] = ch[c][i];
            if (++pos == P) { pos = 0; inputSpectra (nch); }
        }
    }

    // Fil audio : traitement en place (R nul en mono)
    void process (float* L, float* R, int n)
    {
        if (cur == nullptr) return;
        if (old == nullptr && freeTrashSlots() >= 2)
            if (auto* p = pending.load())
                if (matches (*p)) install (pending.exchange (nullptr));
        float* ch[2] = { L, R };
        const int nch = R != nullptr ? 2 : 1;
        for (int i = 0; i < n; ++i)
        {
            for (int c = 0; c < nch; ++c)
            {
                inBuf[(size_t) c][(size_t) pos] = ch[c][i];
                ch[c][i] = outBuf[(size_t) c][(size_t) pos];
            }
            if (++pos == P) { pos = 0; processBlock (nch); }
        }
    }

    void reset() { clearHistory(); }

private:
    struct Set
    {
        int mode = 0, N = 0, K = 0;
        bool matrix = false, neutral = false;
        std::vector<float> H[4];   // spectres des partitions : K × bins
    };

    bool matches (const Set& s) const
    {
        return s.mode == (int) mode && (mode == fir::Mode::natural || s.N == activeN);
    }

    int indexOfLength (int N) const
    {
        int i = 0, n = fir::minLength (fs);
        while (n < N && i < 3) { n *= 2; ++i; }
        return i;
    }

    void clearHistory()
    {
        for (auto& f : fdl) std::fill (f.begin(), f.end(), 0.0f);
        for (auto* v : { &lastIn, &inBuf, &outBuf, &outOld })
            for (auto& c : *v) std::fill (c.begin(), c.end(), 0.0f);
        head = 0; pos = 0;
        if (old != nullptr) { release (old); old = nullptr; }
        xfade = 0;
    }

    // remplace le FIR courant sans fondu
    void setCurrent (Set* s)
    {
        if (cur != nullptr && cur != s) release (cur);
        cur = s;
    }

    // nouveau FIR avec fondu (même historique d'entrée)
    void install (Set* s)
    {
        if (s == nullptr) return;
        if (cur != nullptr) { if (old != nullptr) release (old); old = cur; xfade = xfadeBlocks; }
        cur = s;
    }

    void release (Set* s)
    {
        if (s == nullptr || s->neutral) return;
        if (! toTrash (s)) lost = s;   // ne devrait jamais arriver (l'installation attend 2 places libres)
    }
    bool toTrash (Set* s)
    {
        for (auto& t : trash)
        {
            Set* expected = nullptr;
            if (t.compare_exchange_strong (expected, s)) return true;
        }
        return false;
    }
    int freeTrashSlots() const
    {
        int n = 0;
        for (auto& t : trash) n += t.load() == nullptr ? 1 : 0;
        return n;
    }

    void dropOwned()
    {
        if (cur != nullptr && ! cur->neutral) delete cur;
        if (old != nullptr && ! old->neutral) delete old;
        cur = old = nullptr;
        delete lost; lost = nullptr;
        for (auto*& s : neutralLin) { delete s; s = nullptr; }
        delete neutralNat; neutralNat = nullptr;
    }

    Set* makeNeutral (int m, int N, int at)
    {
        fir::Result r; r.N = std::max (N, at + 1); r.matrix = false;
        for (auto& h : r.h) h.assign ((size_t) r.N, 0.0);
        r.h[0][(size_t) at] = 1.0; r.h[3][(size_t) at] = 1.0;
        return makeSet (r, m, N, true);
    }

    // Spectres des partitions d'un jeu de FIR (fil de conception, ou prepare)
    Set* makeSet (const fir::Result& r, int m, int N, bool neutral)
    {
        auto* s = new Set();
        s->mode = m; s->N = N; s->neutral = neutral; s->matrix = r.matrix;
        const int len = (int) r.h[0].size();
        s->K = std::min (Kmax, (len + P - 1) / P);
        std::vector<float> tmp ((size_t) 4 * P);
        for (int e = 0; e < 4; ++e)
        {
            if (! r.matrix && (e == 1 || e == 2)) continue;
            auto& H = s->H[e];
            H.assign ((size_t) s->K * (size_t) bins, 0.0f);
            for (int k = 0; k < s->K; ++k)
            {
                std::fill (tmp.begin(), tmp.end(), 0.0f);
                for (int j = 0; j < P; ++j)
                {
                    const int n = k * P + j;
                    if (n < (int) r.h[e].size()) tmp[(size_t) j] = (float) r.h[e][(size_t) n];
                }
                fftDesign->performRealOnlyForwardTransform (tmp.data(), true);
                std::copy (tmp.begin(), tmp.begin() + bins, H.begin() + (size_t) k * (size_t) bins);
            }
        }
        return s;
    }

    // Spectre du dernier bloc d'entrée (recouvrement : bloc précédent + bloc courant)
    void inputSpectra (int nch)
    {
        head = (head + 1) % Kmax;
        for (int c = 0; c < nch; ++c)
        {
            float* w = work.data();
            std::copy (lastIn[(size_t) c].begin(), lastIn[(size_t) c].end(), w);
            std::copy (inBuf[(size_t) c].begin(), inBuf[(size_t) c].end(), w + P);
            std::fill (w + 2 * P, w + 4 * P, 0.0f);
            fftRT->performRealOnlyForwardTransform (w, true);
            std::copy (w, w + bins, fdl[(size_t) c].begin() + (size_t) head * (size_t) bins);
            std::swap (lastIn[(size_t) c], inBuf[(size_t) c]);
        }
    }

    // Un bloc de P échantillons : spectres d'entrée, produits avec les partitions, sorties
    void processBlock (int nch)
    {
        inputSpectra (nch);
        for (int o = 0; o < nch; ++o)
        {
            outputBlock (*cur, o, nch, outBuf[(size_t) o].data());
            if (old != nullptr) outputBlock (*old, o, nch, outOld[(size_t) o].data());
        }
        if (old != nullptr)
        {
            // fondu linéaire de l'ancien vers le nouveau FIR, sur xfadeBlocks blocs
            const int done = xfadeBlocks - xfade;
            for (int o = 0; o < nch; ++o)
                for (int j = 0; j < P; ++j)
                {
                    const float t = (float) (done * P + j + 1) / (float) (xfadeBlocks * P);
                    outBuf[(size_t) o][(size_t) j] = outOld[(size_t) o][(size_t) j] * (1.0f - t) + outBuf[(size_t) o][(size_t) j] * t;
                }
            if (--xfade <= 0) { release (old); old = nullptr; }
        }
    }

    void outputBlock (const Set& s, int o, int nch, float* out)
    {
        std::fill (acc.begin(), acc.end(), 0.0f);
        const int K = std::min (s.K, Kmax);
        for (int in = 0; in < nch; ++in)
        {
            const int e = o == 0 ? (in == 0 ? 0 : 1) : (in == 0 ? 2 : 3);
            if (nch == 1 && e != 0) continue;
            if (! s.matrix && (e == 1 || e == 2)) continue;
            const auto& H = s.H[e];
            if (H.empty()) continue;
            for (int k = 0; k < K; ++k)
            {
                const int slot = (head - k + Kmax) % Kmax;
                const float* x = fdl[(size_t) in].data() + (size_t) slot * (size_t) bins;
                const float* h = H.data() + (size_t) k * (size_t) bins;
                float* a = acc.data();
                for (int b = 0; b < bins; b += 2)
                {
                    const float xr = x[b], xi = x[b + 1], hr = h[b], hi = h[b + 1];
                    a[b]     += xr * hr - xi * hi;
                    a[b + 1] += xr * hi + xi * hr;
                }
            }
        }
        float* w = work.data();
        std::copy (acc.begin(), acc.end(), w);
        std::fill (w + bins, w + 4 * P, 0.0f);
        fftRT->performRealOnlyInverseTransform (w);
        for (int j = 0; j < P; ++j) out[j] = w[P + j] * invScale;
    }

    // Courbe réelle pour l'écran (fil de conception)
    void makeDisplay (fir::Mode m, int N)
    {
        const int lat = m == fir::Mode::linear ? N / 2 : D;
        std::array<double, displayPoints> f {};
        for (int i = 0; i < displayPoints; ++i) f[(size_t) i] = 10.0 * std::pow (std::min (24000.0, 0.5 * fs) / 10.0, i / (displayPoints - 1.0));
        std::array<float, displayPoints> a {}, d {}, mid {}, side {};
        fir::responseDb (result.h[0], lat, fs, f.data(), displayPoints, a.data(), scratch);
        fir::responseDb (result.h[3], lat, fs, f.data(), displayPoints, d.data(), scratch);
        if (result.matrix)
        {
            // Mid → Mid : (A + B + C + D)/2 ; Side → Side : (A − B − C + D)/2
            hm.resize (result.h[0].size()); hs.resize (result.h[0].size());
            for (size_t n = 0; n < hm.size(); ++n)
            {
                hm[n] = 0.5 * (result.h[0][n] + result.h[1][n] + result.h[2][n] + result.h[3][n]);
                hs[n] = 0.5 * (result.h[0][n] - result.h[1][n] - result.h[2][n] + result.h[3][n]);
            }
            fir::responseDb (hm, lat, fs, f.data(), displayPoints, mid.data(), scratch);
            fir::responseDb (hs, lat, fs, f.data(), displayPoints, side.data(), scratch);
        }
        else { mid = a; side = d; }
        const std::lock_guard<std::mutex> lock (displayMutex);
        for (int i = 0; i < displayPoints; ++i) display.freq[(size_t) i] = (float) f[(size_t) i];
        display.lr0 = a; display.lr1 = d; display.ms0 = mid; display.ms1 = side;
        display.mode = (int) m; display.N = N; display.valid = true; display.matrix = result.matrix;
        display.version = displayVer.fetch_add (1) + 1;
    }

    double fs = 48000.0;
    int P = 256, D = 128, order = 9, Kmax = 1, bins = 514;
    float invScale = 1.0f;
    std::unique_ptr<juce::dsp::FFT> fftRT, fftDesign;
    std::array<std::vector<float>, 2> fdl, lastIn, inBuf, outBuf, outOld;
    std::vector<float> work, acc;
    int head = 0, pos = 0;
    static constexpr int xfadeBlocks = 4;   // fondu entre deux FIR : 4 blocs (≈ 21 ms à 48 kHz)
    int xfade = 0;

    fir::Mode mode = fir::Mode::linear;
    int activeN = 4096;
    Set* cur = nullptr;
    Set* old = nullptr;
    Set* lost = nullptr;
    std::array<Set*, 4> neutralLin {};
    Set* neutralNat = nullptr;
    std::atomic<Set*> pending { nullptr };
    std::array<std::atomic<Set*>, 8> trash {};
    std::atomic<bool> prepared { false };
    std::mutex designMutex;

    // fil de conception
    fir::Result result;
    fir::Work work2;
    std::vector<fir::cd> scratch;
    std::vector<double> hm, hs;
    mutable std::mutex displayMutex;
    DisplayData display;
    std::atomic<int> displayVer { 0 };
};
} // namespace audiolab::eq
