// Audio Lab EQ — modes FIR « Linear phase » et « Natural phase » : cœur sans JUCE.
// Conception des FIR : FirDesign.h et FirBank.h (fil de conception). Lecture : convolution par
// partitions uniformes (UPOLS) écrite pour ce plugin, au lieu de juce::dsp::Convolution :
//  - l'historique d'entrée (spectres des blocs passés) est COMMUN à l'ancien et au nouveau FIR :
//    quand le FIR change (réglage, EQ dynamique), on passe de l'un à l'autre par un fondu sans
//    trou (juce::dsp::Convolution repartait d'un historique vide : la sortie du nouveau FIR manquait
//    pendant la moitié de sa longueur, soit un creux de plusieurs dizaines de ms en phase linéaire) ;
//  - matrice 2×2 (Mid / Side) : 2 FFT d'entrée, 2 FFT inverses, 4 produits seulement si besoin ;
//  - latence = un bloc de partition P (256 éch. à 48 kHz), ajoutée à celle du FIR et annoncée.
// 3.0, bandes dynamiques (FirBank.h) :
//  - Linear phase : le jeu contient le FIR de référence et des FIR d'écart, qui lisent le MÊME
//    historique d'entrée ; leurs sorties sont mélangées à chaque échantillon d'après le gain du
//    détecteur (process() reçoit ce gain, en avance d'un bloc de partition sur la sortie) ;
//  - Natural phase : le jeu dit quelles bandes sont confiées au moteur zéro latence (engineMask).
//    Quand ce partage change, le fondu entre l'ancien et le nouveau FIR n'est pas fait ici : les deux
//    sorties sont rendues séparément, le moteur traite chacune avec ses bandes puis fait le fondu.
// 3.0, changement de mode de phase ou de longueur sans silence (beginSwitch / endSwitch) : l'ancien
// jeu et le nouveau, de latences différentes, sont rendus en parallèle à partir du même historique
// d'entrée ; le fondu enchaîné entre les deux sorties est fait par l'appelant (PluginProcessor).
// Aucune allocation dans le fil audio : tout est réservé dans prepare() ; les jeux de FIR sont créés
// sur le fil de conception et transmis sans verrou (pointeurs atomiques).
//
// Fft : classe qui fournit Fft (int ordre), forward (float*) et inverse (float*), avec les conventions
// de juce::dsp::FFT (performRealOnlyForwardTransform / performRealOnlyInverseTransform).
#pragma once
#include "FirBank.h"
#include <atomic>
#include <memory>
#include <mutex>

namespace audiolab::eq
{
template <class Fft>
class LinearPhaseT
{
public:
    struct Snapshot
    {
        std::array<BandSpec, maxBands> bands;   // gains de base (sans le décalage dynamique)
        bool stereo = true;
        fir::DynInfo dyn;                       // plage utile et décalage du moment de chaque bande
    };

    // Courbe réelle du FIR pour l'écran (grille logarithmique fixe)
    static constexpr int displayPoints = 512;
    struct DisplayData
    {
        int version = 0;
        int mode = -1, N = 0;
        bool valid = false, matrix = false;
        // bandes dynamiques absentes de la courbe du FIR (Natural : faites par le moteur) et bandes
        // présentes à leur gain de base (Linear : le mélange ajoute leur mouvement)
        uint32_t engineMask = 0, mixMask = 0;
        std::array<float, displayPoints> freq {}, lr0 {}, lr1 {}, ms0 {}, ms1 {};
    };

    // Sortie en cours de lecture : bandes à traiter par le moteur après le FIR en place (cur) et après
    // le FIR sortant (old) ; split : les deux sorties sont rendues séparément (fondu à faire après).
    struct OutState { uint32_t cur = 0, old = 0; bool split = false; };

    LinearPhaseT() { for (auto& t : trash) t.store (nullptr); }
    ~LinearPhaseT()
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
        lastDevs.reset(); lastDevKey.clear();
        fs = sampleRate;
        P = fir::partitionSize (fs);
        D = fir::naturalDelay (fs);
        order = 0; while ((1 << order) < 2 * P) ++order;
        fftRT = std::make_unique<Fft> (order);
        fftDesign = std::make_unique<Fft> (order);
        {
            // facteur d'échelle de la FFT inverse (dépend de l'implémentation : vDSP, IPP, interne…)
            std::vector<float> d ((size_t) 4 * P, 0.0f);
            for (int k = 0; k <= P; ++k) d[(size_t) (2 * k)] = 1.0f;
            fftRT->inverse (d.data());
            invScale = std::abs (d[0]) > 1e-12f ? 1.0f / d[0] : 1.0f;
        }
        Kmax = (fir::maxLength (fs) + P - 1) / P;
        bins = 2 * P + 2;   // P + 1 complexes entrelacés
        for (auto& f : fdl) f.assign ((size_t) Kmax * (size_t) bins, 0.0f);
        for (auto* v : { &lastIn, &inBuf, &outBuf, &outOld })
            for (auto& c : *v) c.assign ((size_t) P, 0.0f);
        work.assign ((size_t) 4 * P, 0.0f);
        acc.assign ((size_t) bins, 0.0f);
        for (auto& bank : gIn) for (auto& g : bank) g.assign ((size_t) P, 0.0f);
        wts.assign ((size_t) fir::maxMix * (size_t) (fir::maxSeg + 1) * (size_t) P, 0.0f);
        devOut.assign ((size_t) P, 0.0f);
        wProd.assign ((size_t) P, 0.0f);
        gHave[0] = gHave[1] = 0; gSeen[0] = gSeen[1] = 0;
        swHold = false;
        pendingStamp.store (0);
        { const std::lock_guard<std::mutex> dl (displayMutex); preEcho = {}; preEchoMode = -1; preEchoN = 0; }

        // FIR neutres (retard pur) de toutes les longueurs, prêts d'avance
        int N = fir::minLength (fs);
        for (int i = 0; i < 4; ++i, N *= 2)
            neutralLin[(size_t) i] = makeNeutral ((int) fir::Mode::linear, N, N / 2);
        neutralNat = makeNeutral ((int) fir::Mode::natural, 0, D);
        mode = fir::Mode::linear; activeN = fir::minLength (fs);
        clearHistory();
        cur = neutralLin[0];
        rdCur = 0;
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
        rdSplit = false; swHold = false;
        for (auto& bank : gIn) for (auto& g : bank) std::fill (g.begin(), g.end(), 0.0f);
        if (auto* p = pending.exchange (nullptr))
        {
            if (matches (*p)) { setCurrent (p); rdCur = cur->plan.engineMask; return; }
            if (! toTrash (p)) delete p;   // (ne devrait pas arriver : 8 places)
        }
        setCurrent (m == fir::Mode::linear ? neutralLin[(size_t) indexOfLength (N)] : neutralNat);
        rdCur = 0;
    }

    // Vrai quand le FIR en place correspond au réglage actif (conçu, pas neutre)
    bool ready() const { return cur != nullptr && ! cur->neutral && matches (*cur); }

    // ── Changement de mode / de longueur sans silence (3.0) ────────────────
    // Fil audio : vrai quand le dernier jeu déposé par le fil de conception est pour ce réglage
    // (lecture d'une étiquette atomique : le jeu lui-même n'est pas touché).
    bool designReady (fir::Mode m, int N) const { return pendingStamp.load() == stampOf (m, N); }
    // Fil audio : un fondu entre deux FIR du même réglage est en cours (attendre avant beginSwitch)
    bool fading() const { return old != nullptr; }

    // Fil audio : passe au réglage (m, N). keepOld : le jeu en place continue d'être rendu en parallèle
    // (sortie par oldL / oldR de process(), avec ses propres gains dynamiques : oldTrace) jusqu'à
    // endSwitch() ; sinon (on vient du mode zéro latence) seul le nouveau jeu est rendu. La sortie du
    // nouveau jeu n'existe qu'à partir du bloc de partition suivant : d'ici là, c'est celle de l'ancien
    // (ou du silence) ; l'appelant ne commence son fondu qu'après ce délai.
    void beginSwitch (fir::Mode m, int N, bool keepOld)
    {
        if (old != nullptr) { release (old); old = nullptr; xfade = 0; }
        Set* prev = cur;
        mode = m; activeN = N;
        Set* next = nullptr;
        if (auto* p = pending.exchange (nullptr))
        {
            if (matches (*p)) next = p;
            else if (! toTrash (p)) delete p;   // (ne devrait pas arriver : 8 places)
        }
        if (next == nullptr) next = m == fir::Mode::linear ? neutralLin[(size_t) indexOfLength (N)] : neutralNat;
        if (keepOld && prev != nullptr && prev != next)
        {
            old = prev; cur = next; swHold = true;
            for (size_t c = 0; c < 2; ++c) std::copy (outBuf[c].begin() + pos, outBuf[c].end(), outOld[c].begin() + pos);
            for (size_t b = 0; b < (size_t) maxBands; ++b)
            {
                std::copy (gIn[0][b].begin(), gIn[0][b].end(), gIn[1][b].begin());
                std::fill (gIn[0][b].begin(), gIn[0][b].end(), 0.0f);
            }
            gHave[1] = gHave[0]; gSeen[1] = gSeen[0]; gHave[0] = 0; gSeen[0] = 0;
            rdSplit = true; rdDone = 0; rdOld = rdCur;
        }
        else
        {
            setCurrent (next); swHold = false; rdSplit = false;
            for (auto& c : outBuf) std::fill (c.begin() + pos, c.end(), 0.0f);
            for (auto& g : gIn[0]) std::fill (g.begin(), g.end(), 0.0f);
            gHave[0] = 0; gSeen[0] = 0;
        }
        rdCur = cur->plan.engineMask;
    }

    // Fil audio : fin du fondu de l'appelant, l'ancien jeu n'est plus rendu
    void endSwitch()
    {
        if (! swHold) return;
        if (old != nullptr) { release (old); old = nullptr; }
        swHold = false; rdSplit = false; xfade = 0;
    }
    bool switching() const { return swHold; }

    // Répartition des bandes dynamiques qu'aurait un jeu conçu pour ce réglage (fil de conception)
    fir::Plan planFor (const Snapshot& snap, fir::Mode m, int N) const
    {
        if (legacyDynamics)
        {
            fir::Plan p;
            for (int i = 0; i < maxBands; ++i)
                if (fir::isDynamic (snap.bands[(size_t) i], snap.dyn.full[(size_t) i])) p.fallbackMask |= 1u << i;
            return p;
        }
        return fir::makePlan (snap.bands, snap.dyn, m, fs, snap.stereo, devBudget (N));
    }

    // Mesures « avant » (2.0), uniquement pour le banc de test : toutes les bandes dynamiques restent
    // dans le FIR avec leur gain du moment (reconception du FIR + fondu).
    bool legacyDynamics = false;

    // Fil de conception : calcule les FIR et les dépose pour le fil audio.
    void design (const Snapshot& snap, fir::Mode m, int N, bool wantDisplay)
    {
        const std::lock_guard<std::mutex> lock (designMutex);
        if (! prepared.load()) return;
        fir::Request rq;
        rq.bands = snap.bands; rq.stereo = snap.stereo; rq.mode = m; rq.N = N; rq.delay = D; rq.fs = fs;
        const auto plan = planFor (snap, m, N);
        // Bandes en trop (méthode 2.0) qui ne recouvrent aucune bande mélangée : quand seul leur gain du
        // moment a changé, on ne reconçoit que le FIR de référence et on garde les FIR d'écart (ils
        // dépendent du FIR de référence, mais là où ces bandes n'agissent pas : écart sous coupleTol).
        // Sinon la reconception continue de tout le jeu serait bien plus lente que celle de la 2.0.
        std::vector<double> key;
        key.insert (key.end(), { (double) (int) m, (double) N, snap.stereo ? 1.0 : 0.0, (double) plan.mixMask, (double) plan.fallbackMask,
                                 (double) plan.nDev, (double) plan.nEdges });
        for (int i = 0; i < maxBands; ++i)
        {
            const auto& b = snap.bands[(size_t) i];
            if (! b.active) { key.push_back (-1.0); continue; }
            key.insert (key.end(), { (double) (int) b.shape, (double) b.slope, (double) (int) b.place, b.freq, b.gainDb, b.q, snap.dyn.full[(size_t) i] });
        }
        const bool reuse = plan.nDev > 0 && plan.fallbackMask != 0 && ! plan.fallbackCoupled && lastDevs != nullptr && key == lastDevKey;
        fir::designBank (rq, snap.dyn, plan, result, devResults, work2, reuse);
        auto* set = makeSet (result, (int) m, N, false);
        set->plan = plan;
        if (! reuse)
        {
            auto devs = std::make_shared<std::vector<Dev>> ((size_t) plan.nDev);
            for (int i = 0; i < plan.nDev; ++i)
            {
                bool same = false;   // (H[3] vide = identique à H[0], décidé FIR par FIR)
                fillSpectra (devResults[(size_t) i], (*devs)[(size_t) i].H, set->matrix, set->K, same);
            }
            lastDevs = devs; lastDevKey = key;
        }
        set->dev = lastDevs;
        if (auto* replaced = pending.exchange (set)) delete replaced;
        pendingStamp.store (stampOf (m, N));
        designCount.fetch_add (1);
        {
            // pré-écho du FIR de référence (jamais calculé dans le fil audio)
            const auto pe = fir::measurePreEcho (result, fs);
            const std::lock_guard<std::mutex> dl (displayMutex);
            preEcho = pe; preEchoMode = (int) m; preEchoN = N;
        }
        if (wantDisplay) makeDisplay (m, N, plan);
    }

    // Pré-écho du dernier FIR conçu pour ce réglage (tous les fils) ; valeurs nulles s'il n'y en a pas encore
    fir::PreEcho getPreEcho (fir::Mode m, int N) const
    {
        const std::lock_guard<std::mutex> dl (displayMutex);
        if (preEchoMode != (int) m || (m == fir::Mode::linear && preEchoN != N)) return {};
        return preEcho;
    }

    // Fil de conception : libère les jeux déjà remplacés
    void collectGarbage() { for (auto& t : trash) delete t.exchange (nullptr); }

    void getDisplay (DisplayData& out) const
    {
        const std::lock_guard<std::mutex> lock (displayMutex);
        out = display;
    }
    int displayVersion() const { return displayVer.load(); }
    int designsDone() const { return designCount.load(); }   // nombre de jeux conçus (tests)

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

    // Échantillons restant avant la fin du bloc de partition en cours (outState() est constant jusque-là)
    int untilBoundary() const { return P - pos; }
    OutState outState() const { return { rdCur, rdSplit ? rdOld : rdCur, rdSplit }; }

    // Fil audio : traitement en place (R nul en mono).
    // trace / traceMask : décalage dynamique (dB) de chaque bande du masque, par échantillon, pour la
    //   sortie qui sera rendue un bloc de partition plus tard (détecteur alimenté par l'entrée retardée
    //   de la latence moins un bloc) ; les autres bandes sont à 0.
    // oldL / oldR / tOut : quand outState().split est vrai, sortie du FIR sortant et position du fondu
    //   (0 → 1) pour chaque échantillon ; s'ils sont nuls, le fondu est fait ici.
    // oldTrace / oldTraceMask : pendant un changement de mode (beginSwitch), gains dynamiques du jeu sortant.
    void process (float* L, float* R, int n, const float* const* trace = nullptr, uint32_t traceMask = 0,
                  float* oldL = nullptr, float* oldR = nullptr, float* tOut = nullptr,
                  const float* const* oldTrace = nullptr, uint32_t oldTraceMask = 0)
    {
        if (cur == nullptr) return;
        // (l'étiquette dit si le jeu déposé est pour le réglage en place : le jeu n'est lu qu'une fois retiré)
        if (old == nullptr && freeTrashSlots() >= 2 && pendingStamp.load() == stampOf (mode, activeN))
            if (auto* p = pending.exchange (nullptr))
            {
                if (matches (*p)) install (p);
                else if (! toTrash (p)) lost = p;
            }
        float* ch[2] = { L, R };
        float* och[2] = { oldL, oldR };
        const int nch = R != nullptr ? 2 : 1;
        if (trace == nullptr) traceMask = 0;
        if (oldTrace == nullptr || ! swHold) oldTraceMask = 0;
        const float* const* trs[2] = { trace, oldTrace };
        const uint32_t masks[2] = { traceMask, oldTraceMask };
        for (int i = 0; i < n;)
        {
            const int m = std::min (n - i, P - pos);
            for (int k = 0; k < (swHold ? 2 : 1); ++k)
            {
                gSeen[k] |= masks[k];
                for (uint32_t rest = masks[k] | gHave[k]; rest != 0; rest &= rest - 1)
                {
                    int b = 0; while (((rest >> b) & 1u) == 0) ++b;
                    float* g = gIn[(size_t) k][(size_t) b].data() + pos;
                    if ((masks[k] >> b) & 1u) std::copy (trs[k][b] + i, trs[k][b] + i + m, g);
                    else std::fill (g, g + m, 0.0f);
                }
            }
            for (int c = 0; c < nch; ++c)
            {
                float* io = ch[c] + i;
                std::copy (io, io + m, inBuf[(size_t) c].data() + pos);
                std::copy (outBuf[(size_t) c].data() + pos, outBuf[(size_t) c].data() + pos + m, io);
                if (rdSplit)
                {
                    const float* o = outOld[(size_t) c].data() + pos;
                    if (och[c] != nullptr) std::copy (o, o + m, och[c] + i);
                    else if (oldL == nullptr)
                        for (int j = 0; j < m; ++j) { const float t = fadePos (pos + j); io[j] = o[j] * (1.0f - t) + io[j] * t; }
                }
            }
            if (rdSplit && tOut != nullptr)
                for (int j = 0; j < m; ++j) tOut[i + j] = fadePos (pos + j);
            pos += m; i += m;
            if (pos == P)
            {
                pos = 0;
                processBlock (nch);
                // (bande sortie du masque : son gain est remis à 0 pendant un bloc entier, puis oublié)
                for (int k = 0; k < 2; ++k) { gHave[k] = gSeen[k]; gSeen[k] = 0; }
            }
        }
    }

    void reset() { clearHistory(); }

    // Nombre de FIR (référence et écarts, par voie de sortie) calculés dans le dernier bloc : pour les tests
    int lastFirCount() const { return firCount; }
    long totalFirCount() const { return firTotal; }
    long blockCount() const { return blockTotal; }

private:
    struct Dev { std::vector<float> H[4]; };
    struct Set
    {
        int mode = 0, N = 0, K = 0;
        bool matrix = false, neutral = false;
        bool same = false;         // H[3] identique à H[0] (toutes les bandes en stéréo) : non stocké
        std::vector<float> H[4];   // spectres des partitions : K × bins
        fir::Plan plan;
        std::shared_ptr<std::vector<Dev>> dev;   // FIR d'écart (Linear phase, bandes dynamiques), partagés entre jeux
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

    // FIR d'écart permis pour cette longueur (mémoire : ≈ 48 Mo par jeu au plus, en stéréo)
    int devBudget (int N) const
    {
        const double perDev = 2.0 * (double) ((std::max (N, P) + P - 1) / P) * (double) bins;
        return std::clamp ((int) (12.0e6 / perDev), 4, fir::maxDev);
    }

    float fadePos (int j) const { return (float) (rdDone * P + j + 1) / (float) (xfadeBlocks * P); }

    void clearHistory()
    {
        for (auto& f : fdl) std::fill (f.begin(), f.end(), 0.0f);
        for (auto* v : { &lastIn, &inBuf, &outBuf, &outOld })
            for (auto& c : *v) std::fill (c.begin(), c.end(), 0.0f);
        for (auto& bank : gIn) for (auto& g : bank) std::fill (g.begin(), g.end(), 0.0f);
        head = 0; pos = 0;
        if (old != nullptr) { release (old); old = nullptr; }
        xfade = 0; rdSplit = false; swHold = false; gHave[0] = gHave[1] = 0; gSeen[0] = gSeen[1] = 0;
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

    // Spectres des partitions d'un jeu de 4 FIR (fil de conception, ou prepare)
    void fillSpectra (const fir::Result& r, std::vector<float>* Hs, bool matrix, int K, bool& same)
    {
        same = ! matrix && r.h[3] == r.h[0];
        std::vector<float> tmp ((size_t) 4 * P);
        for (int e = 0; e < 4; ++e)
        {
            if (! matrix && (e == 1 || e == 2)) continue;
            if (same && e == 3) continue;
            auto& H = Hs[e];
            H.assign ((size_t) K * (size_t) bins, 0.0f);
            for (int k = 0; k < K; ++k)
            {
                std::fill (tmp.begin(), tmp.end(), 0.0f);
                for (int j = 0; j < P; ++j)
                {
                    const int n = k * P + j;
                    if (n < (int) r.h[e].size()) tmp[(size_t) j] = (float) r.h[e][(size_t) n];
                }
                fftDesign->forward (tmp.data());
                std::copy (tmp.begin(), tmp.begin() + bins, H.begin() + (size_t) k * (size_t) bins);
            }
        }
    }

    Set* makeSet (const fir::Result& r, int m, int N, bool neutral)
    {
        auto* s = new Set();
        s->mode = m; s->N = N; s->neutral = neutral; s->matrix = r.matrix;
        const int len = (int) r.h[0].size();
        s->K = std::min (Kmax, (len + P - 1) / P);
        fillSpectra (r, s->H, s->matrix, s->K, s->same);
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
            fftRT->forward (w);
            std::copy (w, w + bins, fdl[(size_t) c].begin() + (size_t) head * (size_t) bins);
            std::swap (lastIn[(size_t) c], inBuf[(size_t) c]);
        }
    }

    // Un bloc de P échantillons : spectres d'entrée, produits avec les partitions, sorties
    void processBlock (int nch)
    {
        inputSpectra (nch);
        firCount = 0;
        renderSet (*cur, nch, outBuf, 0);
        if (old != nullptr && swHold)
        {
            // changement de mode : l'ancien jeu est rendu à part, avec ses gains ; le fondu est fait par l'appelant
            renderSet (*old, nch, outOld, 1);
            rdSplit = true; rdDone = 0; rdOld = old->plan.engineMask;
        }
        else if (old != nullptr)
        {
            renderSet (*old, nch, outOld, 0);
            const int done = xfadeBlocks - xfade;
            // partage FIR / moteur différent : les deux sorties sont rendues telles quelles
            const bool split = cur->plan.engineMask != old->plan.engineMask;
            if (! split)
            {
                // fondu linéaire de l'ancien vers le nouveau FIR, sur xfadeBlocks blocs
                for (int o = 0; o < nch; ++o)
                    for (int j = 0; j < P; ++j)
                    {
                        const float t = (float) (done * P + j + 1) / (float) (xfadeBlocks * P);
                        outBuf[(size_t) o][(size_t) j] = outOld[(size_t) o][(size_t) j] * (1.0f - t) + outBuf[(size_t) o][(size_t) j] * t;
                    }
            }
            rdSplit = split; rdDone = done; rdOld = old->plan.engineMask;
            if (--xfade <= 0) { release (old); old = nullptr; }
        }
        else rdSplit = false;
        rdCur = cur->plan.engineMask;
        firTotal += firCount; ++blockTotal;
    }

    // Sortie d'un jeu pour le bloc : référence, puis FIR d'écart mélangés à chaque échantillon
    void renderSet (const Set& s, int nch, std::array<std::vector<float>, 2>& dst, int bank)
    {
        for (int o = 0; o < nch; ++o) outputBlock (s.H, s, o, nch, dst[(size_t) o].data());
        firCount += nch;
        const auto& pl = s.plan;
        if (pl.nMix == 0 || s.dev == nullptr || (int) s.dev->size() < pl.nDev) return;

        // poids de chaque ancre de chaque bande, par échantillon
        const size_t stride = (size_t) P, perBand = (size_t) (fir::maxSeg + 1) * stride;
        bool used[fir::maxMix][fir::maxSeg + 1] {};
        for (int k = 0; k < pl.nMix; ++k)
        {
            const auto& m = pl.mb[k];
            float* w = wts.data() + (size_t) k * perBand;
            std::fill (w, w + (size_t) (m.nSeg + 1) * stride, 0.0f);
            const float* g = gIn[(size_t) bank][(size_t) m.band].data();
            for (int j = 0; j < P; ++j)
            {
                if (g[j] == 0.0f) { w[j] = 1.0f; used[k][0] = true; continue; }
                int seg; double a;
                fir::locate (m, g[j], seg, a);
                w[(size_t) seg * stride + (size_t) j] = (float) (1.0 - a);
                w[(size_t) (seg + 1) * stride + (size_t) j] = (float) a;
                used[k][seg] = true; used[k][seg + 1] = true;
            }
        }
        auto add = [&] (int di, const float* w)
        {
            const auto& dv = (*s.dev)[(size_t) di];
            for (int o = 0; o < nch; ++o)
            {
                outputBlock (dv.H, s, o, nch, devOut.data());
                float* d = dst[(size_t) o].data();
                for (int j = 0; j < P; ++j) d[j] += w[j] * devOut[(size_t) j];
            }
            firCount += nch;
        };
        // bandes seules : les ancres qui encadrent le gain du moment
        for (int k = 0; k < pl.nMix; ++k)
        {
            const auto& m = pl.mb[k];
            if (m.deg != 0) continue;
            for (int a = 1; a <= m.nSeg; ++a)
                if (used[k][a]) add (m.first + a - 1, wts.data() + (size_t) k * perBand + (size_t) a * stride);
        }
        // paires couplées : poids bilinéaires sur la grille des deux bandes (+ corrections, voir FirBank.h)
        for (int ei = 0; ei < pl.nEdges; ++ei)
        {
            const auto& e = pl.e[ei];
            const int n1 = pl.mb[e.k1].nSeg, n2 = pl.mb[e.k2].nSeg;
            const float* w1 = wts.data() + (size_t) e.k1 * perBand;
            const float* w2 = wts.data() + (size_t) e.k2 * perBand;
            for (int a = 0; a <= n1; ++a)
            {
                // (l'ancre 0 est toujours examinée : les FIR (a, 0) et (0, b) portent aussi les corrections)
                if (! used[e.k1][a] && a != 0) continue;
                for (int b = 0; b <= n2; ++b)
                {
                    const int di = e.devOf (a, b, n2);
                    if (di < 0 || (! used[e.k2][b] && b != 0)) continue;
                    const float* wa = w1 + (size_t) a * stride;
                    const float* wb = w2 + (size_t) b * stride;
                    bool any = false;
                    for (int j = 0; j < P; ++j)
                    {
                        const float w = fir::edgeWeight (pl, ei, a, b, wa[j], wb[j]);
                        wProd[(size_t) j] = w; any = any || w != 0.0f;
                    }
                    if (any) add (di, wProd.data());
                }
            }
        }
    }

    void outputBlock (const std::vector<float>* Hs, const Set& s, int o, int nch, float* out)
    {
        std::fill (acc.begin(), acc.end(), 0.0f);
        const int K = std::min (s.K, Kmax);
        for (int in = 0; in < nch; ++in)
        {
            const int e = o == 0 ? (in == 0 ? 0 : 1) : (in == 0 ? 2 : 3);
            if (nch == 1 && e != 0) continue;
            if (! s.matrix && (e == 1 || e == 2)) continue;
            const auto& H = (e == 3 && Hs[3].empty()) ? Hs[0] : Hs[e];
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
        fftRT->inverse (w);
        for (int j = 0; j < P; ++j) out[j] = w[P + j] * invScale;
    }

    // Courbe réelle pour l'écran (fil de conception)
    void makeDisplay (fir::Mode m, int N, const fir::Plan& plan)
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
        display.engineMask = plan.engineMask; display.mixMask = plan.mixMask;
        display.version = displayVer.fetch_add (1) + 1;
    }

    double fs = 48000.0;
    int P = 256, D = 128, order = 9, Kmax = 1, bins = 514;
    float invScale = 1.0f;
    std::unique_ptr<Fft> fftRT, fftDesign;
    std::array<std::vector<float>, 2> fdl, lastIn, inBuf, outBuf, outOld;
    std::vector<float> work, acc;
    int head = 0, pos = 0;
    static constexpr int xfadeBlocks = 4;   // fondu entre deux FIR : 4 blocs (≈ 21 ms à 48 kHz)
    int xfade = 0;

    // bandes dynamiques : gain du détecteur pour le bloc à venir, poids des ancres, sortie d'un FIR d'écart
    // (deux jeux de gains : 0 = jeu en place, 1 = jeu sortant pendant un changement de mode)
    std::array<std::array<std::vector<float>, maxBands>, 2> gIn;
    uint32_t gHave[2] {}, gSeen[2] {};
    bool swHold = false;
    std::vector<float> wts, devOut, wProd;
    int firCount = 0;
    long firTotal = 0, blockTotal = 0;
    // état de la sortie en cours de lecture
    uint32_t rdCur = 0, rdOld = 0;
    bool rdSplit = false;
    int rdDone = 0;

    fir::Mode mode = fir::Mode::linear;
    int activeN = 4096;
    Set* cur = nullptr;
    Set* old = nullptr;
    Set* lost = nullptr;
    std::array<Set*, 4> neutralLin {};
    Set* neutralNat = nullptr;
    std::atomic<Set*> pending { nullptr };
    // réglage du dernier jeu déposé dans pending (0 : aucun) ; Natural phase : la longueur ne compte pas
    static int stampOf (fir::Mode m, int N) { return m == fir::Mode::linear ? (N << 1) | 1 : 2; }
    std::atomic<int> pendingStamp { 0 };
    std::array<std::atomic<Set*>, 8> trash {};
    std::atomic<bool> prepared { false };
    std::mutex designMutex;

    // fil de conception
    fir::Result result;
    std::vector<fir::Result> devResults;
    std::shared_ptr<std::vector<Dev>> lastDevs;   // FIR d'écart du dernier jeu conçu, et ce dont ils dépendent
    std::vector<double> lastDevKey;
    fir::Work work2;
    std::vector<fir::cd> scratch;
    std::vector<double> hm, hs;
    mutable std::mutex displayMutex;
    DisplayData display;
    fir::PreEcho preEcho;
    int preEchoMode = -1, preEchoN = 0;
    std::atomic<int> displayVer { 0 };
    std::atomic<int> designCount { 0 };
};
} // namespace audiolab::eq
