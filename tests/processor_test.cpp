// Test du plugin complet hors DAW : presets, annuler/rétablir, A/B, sauvegarde de l'état,
// latences (Linear phase Auto / Low…Max, Natural phase), bypass recalé, changement de mode sans clic,
// projets 1.x, side-chain externe, mono, blocs de tailles variables, charge CPU,
// coût des modes FIR avec des bandes dynamiques.
#include "PluginProcessor.h"
#include <thread>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace P = audiolab::params;
static int failures = 0;
#define CHECK(cond, ...) do { if (! (cond)) { ++failures; std::printf ("  ÉCHEC : "); std::printf (__VA_ARGS__); std::printf ("\n"); } } while (0)

// laisse le temps au fil de conception et au changement de latence (fondu, attente de l'historique)
static void settle (AudioLabEqProcessor& p, juce::AudioBuffer<float>& buf, juce::MidiBuffer& midi, int blocks = 70)
{
    for (int k = 0; k < blocks; ++k) { buf.clear(); p.processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::milliseconds (2)); }
}

static int usedCount (AudioLabEqProcessor& p) { int n = 0; for (int b = 0; b < audiolab::eq::maxBands; ++b) n += p.isUsed (b) ? 1 : 0; return n; }

// ── 3.0 : outils de mesure communs à cette version et à celle d'avant ───
// Tout ce bloc n'utilise que des fonctions du processeur qui existaient avant le travail sur l'Auto gain
// et le changement de mode : compilé avec -DALEQ_BASELINE contre l'ancien code, ce même fichier donne
// les mesures « avant » (changement de mode) et le fichier de référence du test bit à bit (--dump).
static const double kTwoPi = 2.0 * juce::MathConstants<double>::pi;

// bruit rose (filtre de Paul Kellet), ≈ −20 dBFS efficace
struct Pink
{
    explicit Pink (unsigned seed) : rnd ((juce::int64) seed) {}
    float next()
    {
        const double w = rnd.nextDouble() * 2.0 - 1.0;
        b0 = 0.99886 * b0 + w * 0.0555179; b1 = 0.99332 * b1 + w * 0.0750759; b2 = 0.96900 * b2 + w * 0.1538520;
        b3 = 0.86650 * b3 + w * 0.3104856; b4 = 0.55000 * b4 + w * 0.5329522; b5 = -0.7616 * b5 - w * 0.0168980;
        const double y = b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362; b6 = w * 0.115926;
        return (float) (0.08 * y);
    }
    juce::Random rnd; double b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
};

// Réglage commun des essais de changement de mode : une courbe réelle (les chemins diffèrent par leur phase)
static void switchSetup (AudioLabEqProcessor& p, int mode, int res, double fs, int block)
{
    p.setPlayConfigDetails (2, 2, fs, block);
    p.clearAllBands();
    p.createBand (audiolab::eq::Shape::lowShelf, 100.0f, 3.0f, 0.71f, 12);
    p.createBand (audiolab::eq::Shape::bell, 400.0f, -2.0f, 1.0f, 12);
    p.createBand (audiolab::eq::Shape::highShelf, 8000.0f, 2.0f, 0.71f, 12);
    p.setValue (p.apvts.getParameter (P::phase), (float) mode);
    p.setValue (p.apvts.getParameter (P::linRes), (float) res);
    p.testPlaying = 1;
    p.prepareToPlay (fs, block);
    // le fil de conception refait le FIR une fois après prepareToPlay : on le laisse finir (fondu compris)
    juce::AudioBuffer<float> buf (2, block); juce::MidiBuffer midi;
    for (int k = 0; k < 60; ++k) { buf.clear(); p.processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::milliseconds (3)); }
}

struct SwRes
{
    double holeMs = 0;       // durée pendant laquelle le niveau (2,5 ms) est à plus de 20 dB sous le niveau établi
    double dipDb = 0, bumpDb = 0;   // sinus : creux et bosse du niveau (fenêtre glissante de 10 ms) pendant la transition
    double levelDb = 0;      // bruit : énergie de la sortie pendant la transition, rapportée à la moyenne des deux chemins établis
    double stepRatio = 0;    // saut max entre deux échantillons / saut max du signal établi
    double transMs = 0;      // durée de la transition (du premier écart avec l'ancien chemin au dernier écart avec le nouveau)
    double latMs = 0;        // délai entre la demande et l'annonce de la nouvelle latence à l'hôte
    double latToAudioMs = 0; // de l'annonce de la latence au milieu de la transition audible
    int latFrom = 0, latTo = 0;
    bool ok = false, same = false;
};

// sig : 0 = bruit rose, 1 = sinus 100 Hz, 2 = sinus 1 kHz
static SwRes runSwitch (int fromMode, int fromRes, int toMode, int toRes, int sig, unsigned seed, int law = 0, double pace = 1.0)
{
    const double fs = 48000.0; const int block = 64;
    AudioLabEqProcessor sw, ra, rb;
    switchSetup (sw, fromMode, fromRes, fs, block);
#ifndef ALEQ_BASELINE
    sw.testFadeLaw = law;
#else
    juce::ignoreUnused (law);
#endif
    switchSetup (ra, fromMode, fromRes, fs, block);
    switchSetup (rb, toMode, toRes, fs, block);
    const int nPre = (int) (0.5 * fs), nTot = (int) (2.2 * fs);
    std::vector<float> y, a, b; std::vector<int> lat;
    juce::AudioBuffer<float> buf (2, block), in (2, block); juce::MidiBuffer midi;
    Pink p1 (seed), p2 (seed + 7919u);
    const auto wall0 = std::chrono::steady_clock::now();
    int64_t t = 0;
    for (int k = 0; k * block < nTot; ++k)
    {
        if (k * block >= nPre && (k - 1) * block < nPre)
        {
            sw.setValue (sw.apvts.getParameter (P::phase), (float) toMode);
            sw.setValue (sw.apvts.getParameter (P::linRes), (float) toRes);
        }
        for (int i = 0; i < block; ++i, ++t)
        {
            float l, r;
            if (sig == 0) { l = p1.next(); r = 0.6f * l + 0.8f * p2.next(); }
            else { l = r = (float) (0.25 * std::sin (kTwoPi * (sig == 1 ? 100.0 : 1000.0) * (double) t / fs)); }
            in.setSample (0, i, l); in.setSample (1, i, r);
        }
        buf.makeCopyOf (in); sw.processBlock (buf, midi);
        for (int i = 0; i < block; ++i) y.push_back (buf.getSample (0, i));
        lat.push_back (sw.getLatencySamples());
        buf.makeCopyOf (in); ra.processBlock (buf, midi);
        for (int i = 0; i < block; ++i) a.push_back (buf.getSample (0, i));
        buf.makeCopyOf (in); rb.processBlock (buf, midi);
        for (int i = 0; i < block; ++i) b.push_back (buf.getSample (0, i));
        // cadence : temps réel (le fil de conception travaille comme dans un hôte)
        std::this_thread::sleep_until (wall0 + std::chrono::microseconds ((int64_t) ((k + 1) * block * 1.0e6 / (fs * pace))));
    }
    SwRes res;
    const int n = (int) y.size();
    res.latFrom = lat.front(); res.latTo = lat.back();
    int kLat = -1; for (int k = 0; k < (int) lat.size(); ++k) if (lat[(size_t) k] != lat.front()) { kLat = k; break; }
    if (kLat < 0) return res;
    const int tLat = kLat * block;
    res.latMs = 1000.0 * (tLat - nPre) / fs;
    double ssA = 0, ssB = 0; float stepRef = 0;
    for (int i = nPre / 2; i < n; ++i) { ssA += (double) a[(size_t) i] * a[(size_t) i]; ssB += (double) b[(size_t) i] * b[(size_t) i]; }
    for (int i = nPre / 2 + 1; i < n; ++i) stepRef = std::max (stepRef, std::max (std::abs (a[(size_t) i] - a[(size_t) i - 1]), std::abs (b[(size_t) i] - b[(size_t) i - 1])));
    const double rmsA = std::sqrt (ssA / (n - nPre / 2)), rmsB = std::sqrt (ssB / (n - nPre / 2)), rms = 0.5 * (rmsA + rmsB);
    // transition : du premier écart avec l'ancien chemin au dernier écart avec le nouveau
    int tStart = -1, tEnd = -1;
    for (int i = nPre; i < n; ++i) if (std::abs (y[(size_t) i] - a[(size_t) i]) > 0.02 * rms) { tStart = i; break; }
    for (int i = n - 1; i >= nPre; --i) if (std::abs (y[(size_t) i] - b[(size_t) i]) > 0.02 * rms) { tEnd = i + 1; break; }
    if (tStart < 0 && tEnd < 0)
    {
        // les deux chemins donnent le même signal (sinus décalé d'un nombre entier de périodes) et la sortie
        // ne s'écarte jamais d'eux : transition indiscernable, mesurée sur ± 20 ms autour de l'annonce de la latence
        tStart = tLat; tEnd = tLat + (int) (0.02 * fs);
        res.same = true;
    }
    if (tStart < 0 || tEnd <= tStart || tEnd > n - (int) (0.1 * fs)) return res;
    res.ok = true;
    res.transMs = 1000.0 * (tEnd - tStart) / fs;
    res.latToAudioMs = 1000.0 * (0.5 * (tStart + tEnd) - tLat) / fs;
    // trou : niveau sur 2,5 ms à plus de 20 dB sous le niveau établi
    {
        const int w = (int) (0.0025 * fs); int count = 0;
        double acc = 0;
        for (int i = tStart - w; i < tEnd + w; ++i)
        {
            acc += (double) y[(size_t) i] * y[(size_t) i];
            if (i >= tStart) { acc -= (double) y[(size_t) (i - w)] * y[(size_t) (i - w)]; if (std::sqrt (std::max (0.0, acc) / w) < 0.1 * rms) ++count; }
        }
        res.holeMs = 1000.0 * count / fs;
    }
    // sinus : niveau glissant sur 10 ms (un nombre entier de périodes), rapporté au niveau établi
    {
        const int w = (int) (0.010 * fs);
        double acc = 0, lo = 1e9, hi = 0;
        for (int i = tStart - w; i < tEnd + w; ++i)
        {
            acc += (double) y[(size_t) i] * y[(size_t) i];
            if (i >= tStart) { acc -= (double) y[(size_t) (i - w)] * y[(size_t) (i - w)]; const double v = std::sqrt (std::max (0.0, acc) / w); lo = std::min (lo, v); hi = std::max (hi, v); }
        }
        res.dipDb = 20.0 * std::log10 (std::max (1e-9, lo / rms)); res.bumpDb = 20.0 * std::log10 (std::max (1e-9, hi / rms));
    }
    // bruit : énergie de la transition contre la moyenne des deux chemins établis sur la même fenêtre
    {
        double ey = 0, ea = 0, eb = 0;
        for (int i = tStart; i < tEnd; ++i) { ey += (double) y[(size_t) i] * y[(size_t) i]; ea += (double) a[(size_t) i] * a[(size_t) i]; eb += (double) b[(size_t) i] * b[(size_t) i]; }
        res.levelDb = 10.0 * std::log10 (std::max (1e-12, ey) / std::max (1e-12, 0.5 * (ea + eb)));
    }
    float step = 0;
    for (int i = tStart - 960; i < tEnd + 960; ++i) step = std::max (step, std::abs (y[(size_t) i] - y[(size_t) i - 1]));
    res.stepRatio = step / std::max (1e-9f, stepRef);
    return res;
}

struct SwCase { const char* name; int m0, r0, m1, r1; };
static const SwCase swCases[] = {
    { "Zero latency → Linear Low   ", 0, 1, 1, 1 }, { "Linear Low → Zero latency   ", 1, 1, 0, 1 },
    { "Zero latency → Natural      ", 0, 1, 2, 1 }, { "Natural → Zero latency      ", 2, 1, 0, 1 },
    { "Linear Medium → Natural     ", 1, 2, 2, 2 }, { "Natural → Linear Medium     ", 2, 2, 1, 2 },
    { "Linear Low → Linear Max     ", 1, 1, 1, 4 }, { "Linear Max → Linear Medium  ", 1, 4, 1, 2 },
};
struct SwSummary { double hole = 0, dip = 0, bump = 0, step = 0, levelLo = 0, levelHi = 0, levelMean = 0, trans = 0, lat = 0, latToAudio = 0; int bad = 0; };

// Mesure tous les cas ; renvoie le résumé (pire cas de chaque grandeur)
static SwSummary measureSwitches (int pinkTrials)
{
    SwSummary sum; sum.dip = 0; sum.levelLo = 0;
    std::printf ("    cas                          | latence      | annonce après | bruit rose : trou, niveau moyen de la transition (extrêmes), saut | sinus 100 Hz : trou, creux / bosse, saut | sinus 1 kHz : trou, creux / bosse, saut | durée\n");
    for (const auto& c : swCases)
    {
        double hole[3] {}, dip[3] {}, bump[3] {}, step[3] {}, lvLo = 1e9, lvHi = -1e9, lvMean = 0, trans = 0, latMs = 0, l2a = 0;
        int latFrom = 0, latTo = 0, okPink = 0;
        for (int sig = 0; sig < 3; ++sig)
            for (int trial = 0; trial < (sig == 0 ? pinkTrials : 1); ++trial)
            {
                const auto r = runSwitch (c.m0, c.r0, c.m1, c.r1, sig, 100u + (unsigned) trial * 13u);
                if (! r.ok) { ++sum.bad; continue; }
                hole[sig] = std::max (hole[sig], r.holeMs); step[sig] = std::max (step[sig], r.stepRatio);
                dip[sig] = std::min (dip[sig], r.dipDb); bump[sig] = std::max (bump[sig], r.bumpDb);
                if (! r.same) trans = std::max (trans, r.transMs); latMs = std::max (latMs, r.latMs); l2a = r.latToAudioMs; latFrom = r.latFrom; latTo = r.latTo;
                if (sig == 0) { lvLo = std::min (lvLo, r.levelDb); lvHi = std::max (lvHi, r.levelDb); lvMean += r.levelDb; ++okPink; }
            }
        if (okPink > 0) lvMean /= okPink;
        std::printf ("    %s | %5d → %5d | %6.0f ms     | %5.1f ms, %+5.2f dB (%+5.2f … %+5.2f), × %.2f | %5.1f ms, %+6.2f / %+5.2f dB, × %.2f | %5.1f ms, %+6.2f / %+5.2f dB, × %.2f | %5.1f ms\n",
                     c.name, latFrom, latTo, latMs, hole[0], lvMean, lvLo, lvHi, step[0], hole[1], dip[1], bump[1], step[1], hole[2], dip[2], bump[2], step[2], trans);
        sum.hole = std::max (sum.hole, std::max (hole[0], std::max (hole[1], hole[2])));
        sum.dip = std::min (sum.dip, std::min (dip[1], dip[2])); sum.bump = std::max (sum.bump, std::max (bump[1], bump[2]));
        sum.step = std::max (sum.step, std::max (step[0], std::max (step[1], step[2])));
        sum.levelLo = std::min (sum.levelLo, lvLo); sum.levelHi = std::max (sum.levelHi, lvHi); sum.levelMean = std::max (sum.levelMean, std::abs (lvMean));
        sum.trans = std::max (sum.trans, trans); sum.lat = std::max (sum.lat, latMs); sum.latToAudio = std::max (sum.latToAudio, std::abs (l2a));
    }
    return sum;
}

// Rendu de référence, Auto gain coupé : trois modes de phase, bandes fixes / dynamiques / Mid-Side, stéréo
// et mono. Les FIR sont à longueur fixe et conçus avant le rendu : la sortie ne dépend d'aucun autre fil.
static std::vector<float> renderReference()
{
    std::vector<float> out;
    struct RC { int mode, res, preset, channels, block; double fs; float outGain; };
    const RC rcs[] = {
        { 0, 2, 2, 2, 512, 48000.0, 0.0f }, { 0, 2, 3, 2, 512, 48000.0, -3.5f }, { 0, 2, 9, 2, 300, 44100.0, 2.0f }, { 0, 2, 4, 1, 512, 96000.0, 0.0f },
        { 1, 2, 2, 2, 512, 48000.0, 0.0f }, { 1, 1, 3, 2, 512, 48000.0, -3.5f }, { 1, 3, 9, 2, 300, 44100.0, 2.0f }, { 1, 2, 4, 1, 512, 96000.0, 0.0f },
        { 2, 2, 2, 2, 512, 48000.0, 0.0f }, { 2, 2, 3, 2, 512, 48000.0, -3.5f }, { 2, 2, 9, 2, 300, 44100.0, 2.0f }, { 2, 2, 4, 1, 512, 96000.0, 0.0f },
    };
    for (const auto& c : rcs)
    {
        AudioLabEqProcessor p;
        p.setPlayConfigDetails (c.channels, c.channels, c.fs, c.block);
        p.applyPreset (c.preset);
        p.setValue (p.apvts.getParameter (P::outGain), c.outGain);
        p.setValue (p.apvts.getParameter (P::phase), (float) c.mode);
        p.setValue (p.apvts.getParameter (P::linRes), (float) c.res);
        p.testPlaying = 1;
        p.prepareToPlay (c.fs, c.block);
        juce::AudioBuffer<float> buf (c.channels, c.block); juce::MidiBuffer midi;
        for (int k = 0; k < 80; ++k) { buf.clear(); p.processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::milliseconds (3)); }
        juce::Random rnd (77);
        const int blocks = (int) (2.0 * c.fs / c.block);
        for (int k = 0; k < blocks; ++k)
        {
            const float amp = (k / 20) % 2 == 0 ? 0.5f : 0.03f;   // (les bandes dynamiques bougent)
            for (int ch = 0; ch < c.channels; ++ch) for (int i = 0; i < c.block; ++i) buf.setSample (ch, i, amp * (rnd.nextFloat() * 2.0f - 1.0f));
            p.processBlock (buf, midi);
            for (int ch = 0; ch < c.channels; ++ch) for (int i = 0; i < c.block; ++i) out.push_back (buf.getSample (ch, i));
        }
    }
    return out;
}

#ifndef ALEQ_BASELINE
// Comparaison des lois de fondu (« ALEQProcessorTest --fade-laws ») : bruit rose (8 essais) et sinus, trois transitions
static void compareFadeLaws()
{
    const char* lawName[4] = { "corrélation mesurée, 2 fenêtres (plugin)", "puissance constante (sinus / cosinus)   ", "linéaire                                ", "corrélation mesurée, 1 fenêtre          " };
    const SwCase cs[] = { swCases[0], swCases[4], swCases[6], swCases[7] };
    std::printf ("Lois de fondu enchaîné (20 ms) : bruit rose = niveau de la transition, moyenne et extrêmes de 8 essais ; sinus = creux / bosse du niveau sur 10 ms\n");
    for (int law : { 0, 1, 2, 3 })
    {
        std::printf ("  %s\n", lawName[law]);
        double absMean = 0, lo = 0, hi = 0, sLo = 0, sHi = 0;
        for (const auto& c : cs)
        {
            double mean = 0, l = 1e9, h = -1e9; int nOk = 0;
            for (int trial = 0; trial < 8; ++trial)
            {
                const auto r = runSwitch (c.m0, c.r0, c.m1, c.r1, 0, 300u + (unsigned) trial * 17u, law, 3.0);
                if (! r.ok) continue;
                mean += r.levelDb; l = std::min (l, r.levelDb); h = std::max (h, r.levelDb); ++nOk;
            }
            mean /= std::max (1, nOk);
            const auto s1 = runSwitch (c.m0, c.r0, c.m1, c.r1, 1, 1, law, 3.0), s2 = runSwitch (c.m0, c.r0, c.m1, c.r1, 2, 1, law, 3.0);
            std::printf ("    %s : bruit rose %+5.2f dB (%+5.2f … %+5.2f) ; sinus 100 Hz %+6.2f / %+5.2f dB ; sinus 1 kHz %+6.2f / %+5.2f dB\n",
                         c.name, mean, l, h, s1.dipDb, s1.bumpDb, s2.dipDb, s2.bumpDb);
            absMean = std::max (absMean, std::abs (mean)); lo = std::min (lo, l); hi = std::max (hi, h);
            sLo = std::min (sLo, std::min (s1.dipDb, s2.dipDb)); sHi = std::max (sHi, std::max (s1.bumpDb, s2.bumpDb));
        }
        std::printf ("    pire cas : bruit rose moyenne ± %.2f dB, extrêmes %+.2f … %+.2f dB ; sinus creux %.2f dB, bosse %+.2f dB\n", absMean, lo, hi, sLo, sHi);
    }
}
#endif

#ifdef ALEQ_BASELINE
// Version d'avant : seulement les mesures de changement de mode et le fichier de référence
int main (int argc, char** argv)
{
    juce::ScopedJuceInitialiser_GUI init;
    if (argc > 2 && std::string (argv[1]) == "--dump")
    {
        const auto v = renderReference();
        if (FILE* f = std::fopen (argv[2], "wb")) { std::fwrite (v.data(), sizeof (float), v.size(), f); std::fclose (f); }
        std::printf ("référence écrite : %d échantillons\n", (int) v.size());
        return 0;
    }
    std::printf ("AVANT : changement de mode ou de résolution pendant la lecture (48 kHz, blocs de 64, cadence temps réel)\n");
    const auto s = measureSwitches (4);
    std::printf ("    pire cas : trou %.1f ms, creux %.2f dB, bosse %+.2f dB, saut × %.2f, annonce de la latence %.0f ms après la demande ; %d essai(s) sans mesure\n",
                 s.hole, s.dip, s.bump, s.step, s.lat, s.bad);
    return 0;
}
#else
static SwSummary swSummary;

int main (int argc, char** argv)
{
    juce::ScopedJuceInitialiser_GUI init;
    if (argc > 1 && std::string (argv[1]) == "--fade-laws") { compareFadeLaws(); return 0; }
    {
        AudioLabEqProcessor p;
        p.setPlayConfigDetails (2, 2, 48000, 512);
        p.prepareToPlay (48000, 512);
        std::printf ("1. Démarrage : %d bande(s) utilisée(s), latence %d\n", usedCount (p), p.getLatencySamples());
        CHECK (usedCount (p) == 5, "le réglage initial doit contenir 5 bandes");
        {
            const int expectShape[] = { 1, 0, 0, 0, 3 };   // low shelf, 3 cloches, high shelf
            const float expectFreq[] = { 60, 300, 1000, 3000, 10000 };
            for (int b = 0; b < 5; ++b)
                CHECK (juce::roundToInt (p.bandValue (b, P::fShape)) == expectShape[b] && std::abs (p.bandValue (b, P::fFreq) - expectFreq[b]) < 0.5f
                       && std::abs (p.bandValue (b, P::fGain)) < 1e-6f, "bande %d par défaut", b + 1);
            // à plat : la sortie est identique à l'entrée
            juce::AudioBuffer<float> buf (2, 4096), ref (2, 4096); juce::MidiBuffer midi; juce::Random rnd (11);
            for (int pass = 0; pass < 3; ++pass)
            {
                for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 4096; ++i) buf.setSample (ch, i, rnd.nextFloat() - 0.5f);
                ref.makeCopyOf (buf);
                p.processBlock (buf, midi);
            }
            float maxDiff = 0;
            for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 4096; ++i) maxDiff = std::max (maxDiff, std::abs (buf.getSample (ch, i) - ref.getSample (ch, i)));
            std::printf ("   5 bandes à plat : écart max sortie / entrée %.2e\n", maxDiff);
            CHECK (maxDiff < 1e-4f, "le réglage par défaut doit laisser passer le son (%.2e)", maxDiff);
        }

        // presets
        const auto& list = P::presets();
        for (int i = 0; i < (int) list.size(); ++i)
        {
            p.applyPreset (i);
            CHECK (usedCount (p) == (int) list[(size_t) i].bands.size(), "preset %d : %d bandes au lieu de %d", i, usedCount (p), (int) list[(size_t) i].bands.size());
        }
        std::printf ("2. %d presets chargés\n", (int) list.size());

        // annuler / rétablir
        p.applyPreset (0);
        p.pushUndo();
        p.applyPreset (4);   // kick : 4 bandes
        p.pushUndo();
        p.setBandValue (1, P::fGain, 9.0f);
        CHECK (p.undo() && std::abs (p.bandValue (1, P::fGain) - 3.0f) < 0.01f, "annuler le gain");
        CHECK (p.undo() && usedCount (p) == 5, "annuler le preset");
        CHECK (p.redo() && usedCount (p) == 4, "rétablir le preset");
        std::printf ("3. Annuler / rétablir : OK si aucun échec\n");

        // A/B
        p.applyPreset (2);
        p.switchAB ("B");
        CHECK (usedCount (p) == 4, "B vide reprend le réglage courant");
        p.applyPreset (11);
        p.switchAB ("A");
        CHECK (usedCount (p) == 4 && juce::roundToInt (p.bandValue (0, P::fShape)) == (int) audiolab::eq::Shape::lowCut, "retour à A");
        p.switchAB ("B");
        CHECK (usedCount (p) == 3, "retour à B (téléphone)");
        std::printf ("4. A/B : OK si aucun échec\n");

        // sauvegarde / rechargement
        p.applyPreset (9);
        p.setValue (p.apvts.getParameter (P::outGain), -3.5f);
        juce::MemoryBlock mb;
        p.getStateInformation (mb);
        AudioLabEqProcessor q;
        q.setStateInformation (mb.getData(), (int) mb.getSize());
        CHECK (usedCount (q) == 2 && std::abs (q.getParam (P::outGain) + 3.5f) < 0.01f
               && juce::roundToInt (q.bandValue (1, P::fPlace)) == (int) audiolab::eq::Place::side, "état rechargé");
        std::printf ("5. Sauvegarde / rechargement de l'état : %d octets\n", (int) mb.getSize());

        // Attaque / Retour propres à chaque bande, conservés à la sauvegarde
        p.setBandValue (0, P::fAtt, 2.0f); p.setBandValue (1, P::fRel, 900.0f);
        juce::MemoryBlock mb2; p.getStateInformation (mb2);
        AudioLabEqProcessor q2; q2.setStateInformation (mb2.getData(), (int) mb2.getSize());
        CHECK (std::abs (q2.bandValue (0, P::fAtt) - 2.0f) < 0.05f && std::abs (q2.bandValue (1, P::fRel) - 900.0f) < 1.0f
               && std::abs (q2.bandValue (1, P::fAtt) - 10.0f) < 0.05f, "attaque / retour par bande rechargés");

        // Projet v1.0 : réglages communs « dyn_attack / dyn_release » reportés sur chaque bande
        {
            auto st = juce::ValueTree::fromXml (*juce::AudioProcessor::getXmlFromBinary (mb.getData(), (int) mb.getSize()));
            for (int i = st.getNumChildren(); --i >= 0;)
            {
                const auto id = st.getChild (i).getProperty ("id").toString();
                if (id.endsWith ("_att") || id.endsWith ("_rel")) st.removeChild (i, nullptr);
            }
            juce::ValueTree a ("PARAM"); a.setProperty ("id", "dyn_attack", nullptr); a.setProperty ("value", 25.0f, nullptr);
            juce::ValueTree r ("PARAM"); r.setProperty ("id", "dyn_release", nullptr); r.setProperty ("value", 400.0f, nullptr);
            st.appendChild (a, nullptr); st.appendChild (r, nullptr);
            juce::MemoryBlock old; juce::AudioProcessor::copyXmlToBinary (*st.createXml(), old);
            AudioLabEqProcessor q3; q3.setStateInformation (old.getData(), (int) old.getSize());
            CHECK (std::abs (q3.bandValue (5, P::fAtt) - 25.0f) < 0.1f && std::abs (q3.bandValue (0, P::fRel) - 400.0f) < 1.0f,
                   "projet v1.0 : attaque %.1f / retour %.1f", q3.bandValue (5, P::fAtt), q3.bandValue (0, P::fRel));
            std::printf ("   attaque / retour par bande, projet v1.0 relu : OK si aucun échec\n");
        }

        // phase linéaire : latence et alignement du bypass
        p.applyPreset (0);
        p.setValue (p.apvts.getParameter (P::outGain), 0.0f);
        p.setValue (p.apvts.getParameter (P::phase), (float) P::phaseLinear);
        juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi;
        // (3.0 : pas de silence ; la latence est annoncée au début du fondu enchaîné, une fois le FIR conçu)
        settle (p, buf, midi, 30);
        const int lat = p.getLatencySamples();
        std::printf ("6. Phase linéaire (Auto, 5 bandes à plat) : latence %d échantillons (%.1f ms)\n", lat, 1000.0 * lat / 48000);
        CHECK (lat == 4096 / 2 + 256, "latence attendue 2304 (FIR 4096 + bloc 256)");
        // courbe neutre : la sortie doit être l'entrée retardée de la latence
        for (int k = 0; k < 40; ++k) { buf.clear(); p.processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::milliseconds (5)); }
        auto impulsePeak = [&] (int& pkOut, float& valOut)
        {
            std::vector<float> outv;
            for (int blk = 0; blk < 80; ++blk)
            {
                buf.clear();
                if (blk == 0) { buf.setSample (0, 0, 1.0f); buf.setSample (1, 0, 1.0f); }
                p.processBlock (buf, midi);
                for (int i = 0; i < 512; ++i) outv.push_back (buf.getSample (0, i));
            }
            int pk = 0; for (int i = 0; i < (int) outv.size(); ++i) if (std::abs (outv[(size_t) i]) > std::abs (outv[(size_t) pk])) pk = i;
            pkOut = pk; valOut = outv[(size_t) pk];
        };
        int pk; float pv;
        impulsePeak (pk, pv);
        CHECK (pk == lat && std::abs (pv - 1.0f) < 0.01f, "impulsion retardée de la latence (pic %d, %.3f)", pk, pv);
        // bypass : le signal sec est recalé sur la même latence
        p.setValue (p.apvts.getParameter (P::bypass), 1.0f);
        for (int k = 0; k < 10; ++k) { buf.clear(); p.processBlock (buf, midi); }
        impulsePeak (pk, pv);
        CHECK (pk == lat && std::abs (pv - 1.0f) < 0.01f, "bypass recalé (pic %d, %.3f)", pk, pv);
        p.setValue (p.apvts.getParameter (P::bypass), 0.0f);
        for (int k = 0; k < 10; ++k) { buf.clear(); p.processBlock (buf, midi); }

        // résolutions fixes
        const int resLat[] = { 0, 2048 + 256, 4096 + 256, 8192 + 256, 16384 + 256 };
        for (int ri = 1; ri <= 4; ++ri)
        {
            p.setValue (p.apvts.getParameter (P::linRes), (float) ri);
            settle (p, buf, midi);
            CHECK (p.getLatencySamples() == resLat[ri], "résolution %d : latence %d au lieu de %d", ri, p.getLatencySamples(), resLat[ri]);
        }
        std::printf ("   Low / Medium / High / Max : %d / %d / %d / %d échantillons\n", resLat[1], resLat[2], resLat[3], resLat[4]);

        // Auto : la latence suit la bande la plus exigeante, mais pas pendant la lecture
        p.setValue (p.apvts.getParameter (P::linRes), 1.0f);   // Low (2304), puis Auto pendant la lecture
        settle (p, buf, midi);
        p.testPlaying = 1;
        p.setValue (p.apvts.getParameter (P::linRes), 0.0f);
        for (int k = 0; k < 4; ++k) { buf.clear(); p.processBlock (buf, midi); }
        const int latPlay0 = p.getLatencySamples();
        p.createBand (audiolab::eq::Shape::notch, 60.0f, 0.0f, 10.0f, 12);
        settle (p, buf, midi);
        const int latPlay1 = p.getLatencySamples();
        p.testPlaying = 0;
        settle (p, buf, midi);
        const int latStop = p.getLatencySamples();
        std::printf ("   Auto + notch 60 Hz Q 10 : latence %d → %d pendant la lecture, %d à l'arrêt\n", latPlay0, latPlay1, latStop);
        CHECK (latPlay0 == 2048 + 256 && latPlay1 == latPlay0 && latStop == 16384 + 256, "Auto : latence pendant la lecture / à l'arrêt");
        p.testPlaying = -1;

        // Natural phase : petite latence fixe
        p.setValue (p.apvts.getParameter (P::phase), (float) P::phaseNatural);
        settle (p, buf, midi);
        std::printf ("   Natural phase : latence %d échantillons (%.1f ms)\n", p.getLatencySamples(), 1000.0 * p.getLatencySamples() / 48000);
        CHECK (p.getLatencySamples() == 128 + 256, "latence Natural phase (%d)", p.getLatencySamples());

        // changement de mode pendant un sinus : fondu (aucun saut d'échantillon à échantillon > celui du sinus)
        {
            p.applyPreset (2);
            float maxJump = 0, last = 0; bool first = true; int64_t t = 0; int jumpAt = 0;
            for (int blk = 0; blk < 200; ++blk)
            {
                if (blk == 40) p.setValue (p.apvts.getParameter (P::phase), (float) P::phaseZero);
                if (blk == 120) p.setValue (p.apvts.getParameter (P::phase), (float) P::phaseLinear);
                for (int i = 0; i < 512; ++i, ++t) { const float v = (float) (0.5 * std::sin (2 * juce::MathConstants<double>::pi * 220 * (double) t / 48000)); buf.setSample (0, i, v); buf.setSample (1, i, v); }
                p.processBlock (buf, midi);
                for (int i = 0; i < 512; ++i)
                {
                    const float v = buf.getSample (0, i);
                    if (! first && std::abs (v - last) > maxJump) { maxJump = std::abs (v - last); jumpAt = blk * 512 + i; }
                    last = v; first = false;
                }
                std::this_thread::sleep_for (std::chrono::milliseconds (2));
            }
            // pente max d'un sinus 220 Hz d'amplitude ≈ 0,5 × gain : 2π·220/48000·0,5 ≈ 0,0144 (× gain de l'EQ)
            std::printf ("   changements de mode (Natural → Zero latency → Linear) sur un sinus : saut max %.4f (bloc %d)\n", maxJump, jumpAt / 512);
            CHECK (maxJump < 0.05f, "clic au changement de mode (%.4f)", maxJump);
        }
        p.setValue (p.apvts.getParameter (P::phase), 0.0f);
        settle (p, buf, midi, 10);
        CHECK (p.getLatencySamples() == 0, "retour en zéro latence");
    }

    // Compatibilité : projet 1.x (sans les paramètres 2.0) → valeurs par défaut, résolution Medium
    {
        AudioLabEqProcessor p;
        p.applyPreset (3);
        p.setValue (p.apvts.getParameter (P::phase), (float) P::phaseLinear);
        juce::MemoryBlock mb; p.getStateInformation (mb);
        auto st = juce::ValueTree::fromXml (*juce::AudioProcessor::getXmlFromBinary (mb.getData(), (int) mb.getSize()));
        for (int i = st.getNumChildren(); --i >= 0;)
        {
            const auto id = st.getChild (i).getProperty ("id").toString();
            if (id == P::linRes || id.endsWith ("_sc")) st.removeChild (i, nullptr);
        }
        // ancien état A/B (13 champs par bande, 4 réglages généraux)
        {
            std::vector<float> v = p.getStateVector();
            std::vector<float> old;
            for (int b = 0; b < audiolab::eq::maxBands; ++b) for (int f = 0; f < P::legacyBandFields; ++f) old.push_back (v[(size_t) (b * P::numBandFields + f)]);
            for (int g = 0; g < P::legacyGlobals; ++g) old.push_back (v[(size_t) (audiolab::eq::maxBands * P::numBandFields + g)]);
            juce::StringArray a; for (float x : old) a.add (juce::String (x, 7));
            auto u = st.getChildWithName ("ui");
            if (u.isValid()) u.setProperty ("slotB", a.joinIntoString (" "), nullptr);
        }
        juce::MemoryBlock old; juce::AudioProcessor::copyXmlToBinary (*st.createXml(), old);
        AudioLabEqProcessor q;
        q.setBandValue (1, P::fSc, 1.0f);   // (doit revenir à Internal)
        q.setStateInformation (old.getData(), (int) old.getSize());
        CHECK (q.linResIndex() == P::resMediumIndex, "projet 1.x : résolution %d au lieu de Medium", q.linResIndex());
        CHECK (q.phaseMode() == P::phaseLinear, "projet 1.x : le mode Linear phase doit rester Linear");
        bool allInternal = true; for (int b = 0; b < audiolab::eq::maxBands; ++b) allInternal = allInternal && q.bandValue (b, P::fSc) < 0.5f;
        CHECK (allInternal && usedCount (q) == (int) P::presets()[3].bands.size(), "projet 1.x : side-chain interne et bandes rechargées");
        q.switchAB ("B");
        CHECK (usedCount (q) == (int) P::presets()[3].bands.size(), "ancien état A/B relu");
        std::printf ("9. Projet 1.x relu : résolution Medium, side-chain Internal, A/B à l'ancien format : OK si aucun échec\n");
        // nouvel état : les réglages 2.0 sont gardés
        q.setValue (q.apvts.getParameter (P::linRes), 3.0f); q.setBandValue (0, P::fSc, 1.0f);
        juce::MemoryBlock mb2; q.getStateInformation (mb2);
        AudioLabEqProcessor r; r.setStateInformation (mb2.getData(), (int) mb2.getSize());
        CHECK (r.linResIndex() == 3 && r.bandValue (0, P::fSc) > 0.5f, "réglages 2.0 sauvegardés");
    }

    // Side-chain externe : bus facultatif, la bande dynamique suit le side-chain
    {
        AudioLabEqProcessor p;
        auto layout = p.getBusesLayout();
        CHECK (layout.inputBuses.size() == 2 && layout.getChannelSet (true, 1).isDisabled(), "bus side-chain facultatif et désactivé par défaut");
        layout.getChannelSet (true, 1) = juce::AudioChannelSet::stereo();
        CHECK (p.setBusesLayout (layout), "bus side-chain stéréo accepté");
        p.setRateAndBufferSizeDetails (48000, 512);
        p.prepareToPlay (48000, 512);
        p.clearAllBands();
        const int b = p.createBand (audiolab::eq::Shape::bell, 1000.0f, 0.0f, 2.0f, 12);
        p.setBandValue (b, P::fDyn, 1); p.setBandValue (b, P::fThr, -30); p.setBandValue (b, P::fRange, -10); p.setBandValue (b, P::fSc, 1);
        juce::AudioBuffer<float> buf (4, 512); juce::MidiBuffer midi;
        int64_t t = 0;
        auto run = [&] (float mainAmp, float scAmp)
        {
            for (int blk = 0; blk < 60; ++blk)
            {
                for (int i = 0; i < 512; ++i, ++t)
                {
                    const float v = (float) std::sin (2 * juce::MathConstants<double>::pi * 1000 * (double) t / 48000);
                    buf.setSample (0, i, mainAmp * v); buf.setSample (1, i, mainAmp * v);
                    buf.setSample (2, i, scAmp * v); buf.setSample (3, i, scAmp * v);
                }
                p.processBlock (buf, midi);
            }
            return p.getDynOffset (b);
        };
        const float quietSc = run (0.3f, 0.0f), loudSc = run (0.01f, 0.3f);
        std::printf ("10. Side-chain externe : signal fort + side-chain muet → %.1f dB ; signal faible + side-chain fort → %.1f dB (connecté : %s)\n",
                     quietSc, loudSc, p.sideChainConnected() ? "oui" : "non");
        CHECK (std::abs (quietSc) < 0.1f && loudSc < -9.0f && p.sideChainConnected(), "side-chain externe");
    }

    // mono + blocs de tailles variables + toutes les bandes
    {
        AudioLabEqProcessor p;
        p.setPlayConfigDetails (1, 1, 44100, 256);
        p.prepareToPlay (44100, 256);
        p.clearAllBands();
        juce::Random rnd (3);
        for (int b = 0; b < audiolab::eq::maxBands; ++b)
            p.createBand ((audiolab::eq::Shape) (b % 8), 30.0f * std::pow (1.3f, (float) b), (float) (b % 5) - 2.0f, 1.0f, 12,
                          (audiolab::eq::Place) (b % 5));
        for (int b = 0; b < audiolab::eq::maxBands; b += 3) p.setBandValue (b, P::fDyn, 1);
        bool bad = false;
        for (int it = 0; it < 300; ++it)
        {
            const int n = 1 + rnd.nextInt (2048);
            juce::AudioBuffer<float> buf (1, n); juce::MidiBuffer midi;
            for (int i = 0; i < n; ++i) buf.setSample (0, i, rnd.nextFloat() * 0.5f - 0.25f);
            if (it == 150) p.setValue (p.apvts.getParameter (P::phase), 1.0f);
            p.processBlock (buf, midi);
            for (int i = 0; i < n; ++i) if (! std::isfinite (buf.getSample (0, i)) || std::abs (buf.getSample (0, i)) > 100) bad = true;
        }
        std::printf ("7. Mono, 24 bandes, blocs de 1 à 2048 échantillons, changement de mode : %s\n", bad ? "PROBLÈME" : "OK");
        CHECK (! bad, "sortie invalide en mono");
    }

    // charge CPU : 24 bandes stéréo dont 8 dynamiques, zéro latence puis phase linéaire
    {
        AudioLabEqProcessor p;
        p.setPlayConfigDetails (2, 2, 48000, 512);
        p.prepareToPlay (48000, 512);
        for (int b = 0; b < audiolab::eq::maxBands; ++b)
            p.createBand (b % 3 == 0 ? audiolab::eq::Shape::lowCut : audiolab::eq::Shape::bell, 40.0f * std::pow (1.3f, (float) b), 3.0f, 1.0f, b % 3 == 0 ? 48 : 12);
        for (int b = 1; b < audiolab::eq::maxBands; b += 3) p.setBandValue (b, P::fDyn, 1);
        juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi; juce::Random rnd (1);
        for (int mode = 0; mode < 3; ++mode)
        {
            p.setValue (p.apvts.getParameter (P::phase), (float) mode);
            for (int k = 0; k < 60; ++k) { buf.clear(); p.processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::milliseconds (2)); }
            const int blocks = 940;   // 10 s d'audio
            const auto t0 = juce::Time::getHighResolutionTicks();
            for (int k = 0; k < blocks; ++k)
            {
                for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 512; ++i) buf.setSample (ch, i, rnd.nextFloat() * 0.2f - 0.1f);
                p.processBlock (buf, midi);
            }
            const double sec = juce::Time::highResolutionTicksToSeconds (juce::Time::getHighResolutionTicks() - t0);
            std::printf ("8. Charge %s : %.1f %% d'un cœur (10 s d'audio traités en %.2f s, FIR %d points)\n",
                         mode == 0 ? "zéro latence" : mode == 1 ? "phase linéaire" : "phase naturelle", 100.0 * sec / 10.0, sec, p.activeFirLength());
        }
    }

    // coût des modes FIR avec 0, 1 ou 4 bandes dynamiques, en temps réel (3 s par cas) : temps du fil
    // audio (processBlock) et temps des autres fils du plugin (conception des FIR)
    {
        std::printf ("11. Coût des modes FIR, bandes dynamiques en action (%% d'un cœur ; fil audio + fil de conception)\n");
        auto cpuNow = [] (clockid_t id) { timespec ts; clock_gettime (id, &ts); return (double) ts.tv_sec + 1e-9 * (double) ts.tv_nsec; };
        for (int mode = 1; mode <= 2; ++mode)
            for (int res : { P::resMediumIndex, 4 })
            {
                if (mode == 2 && res == 4) continue;   // Natural phase : pas de réglage de longueur
                for (int nd : { 0, 1, 4 })
                {
                    AudioLabEqProcessor p;
                    p.setPlayConfigDetails (2, 2, 48000, 512);
                    p.clearAllBands();
                    p.createBand (audiolab::eq::Shape::lowCut, 40.0f, 0.0f, 0.71f, 12);
                    p.createBand (audiolab::eq::Shape::highShelf, 12000.0f, 3.0f, 0.71f, 12);
                    const float fr[4] = { 150.0f, 700.0f, 3000.0f, 9000.0f };
                    for (int k = 0; k < 4; ++k)
                    {
                        const int b = p.createBand (audiolab::eq::Shape::bell, fr[k], 2.0f, 4.0f, 12);
                        if (k < nd) { p.setBandValue (b, P::fDyn, 1); p.setBandValue (b, P::fThr, -45); p.setBandValue (b, P::fRange, -12); p.setBandValue (b, P::fAtt, 5); p.setBandValue (b, P::fRel, 80); }
                    }
                    p.setValue (p.apvts.getParameter (P::phase), (float) mode);
                    p.setValue (p.apvts.getParameter (P::linRes), (float) res);
                    p.prepareToPlay (48000, 512);
                    juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi; juce::Random rnd (5);
                    const int blocks = 281;   // 3 s
                    const int designs0 = p.firDesignCount();
                    double audio = 0; int64_t t = 0;
                    const double proc0 = cpuNow (CLOCK_PROCESS_CPUTIME_ID), main0 = cpuNow (CLOCK_THREAD_CPUTIME_ID);
                    const auto wall0 = std::chrono::steady_clock::now();
                    for (int k = 0; k < blocks; ++k)
                    {
                        for (int i = 0; i < 512; ++i, ++t)
                        {
                            const float env = 0.5f + 0.5f * (float) std::sin (2 * juce::MathConstants<double>::pi * 3.0 * (double) t / 48000);
                            const float v = env * (rnd.nextFloat() * 0.6f - 0.3f);
                            buf.setSample (0, i, v); buf.setSample (1, i, 0.8f * v);
                        }
                        const double a0 = cpuNow (CLOCK_THREAD_CPUTIME_ID);
                        p.processBlock (buf, midi);
                        audio += cpuNow (CLOCK_THREAD_CPUTIME_ID) - a0;
                        std::this_thread::sleep_until (wall0 + std::chrono::microseconds ((int64_t) (k + 1) * 10667));
                    }
                    const double others = (cpuNow (CLOCK_PROCESS_CPUTIME_ID) - proc0) - (cpuNow (CLOCK_THREAD_CPUTIME_ID) - main0), dur = blocks * 512 / 48000.0;
                    std::printf ("   %s, FIR %5d points, %d bande(s) dynamique(s) : fil audio %5.2f %% ; fil de conception %5.2f %% (%d conception(s) pendant la lecture)\n",
                                 mode == 1 ? "Linear " : "Natural", p.activeFirLength(), nd, 100.0 * audio / dur, 100.0 * others / dur, p.firDesignCount() - designs0);
                    // 3.0 : le mouvement d'une bande dynamique ne fait plus reconcevoir le FIR
                    CHECK (p.firDesignCount() - designs0 <= 2, "reconceptions pendant la lecture (%d)", p.firDesignCount() - designs0);
                }
            }
    }

    // bandes dynamiques dans les modes FIR (3.0) : le plugin complet, avec son fil de conception
    {
        std::printf ("12. Bandes dynamiques dans les modes FIR (Linear : mélange de FIR ; Natural : moteur zéro latence après le FIR)\n");
        const double twoPi = 2 * juce::MathConstants<double>::pi;
        for (int mode = 1; mode <= 2; ++mode)
        {
            const char* mn = mode == 1 ? "Linear " : "Natural";
            AudioLabEqProcessor p;
            p.setPlayConfigDetails (2, 2, 48000, 512);
            p.clearAllBands();
            p.createBand (audiolab::eq::Shape::lowCut, 40.0f, 0.0f, 0.71f, 12);
            const int b = p.createBand (audiolab::eq::Shape::bell, 1000.0f, 0.0f, 2.0f, 12);
            p.setBandValue (b, P::fDyn, 1); p.setBandValue (b, P::fThr, -30); p.setBandValue (b, P::fRange, -10);
            p.setBandValue (b, P::fAtt, 2); p.setBandValue (b, P::fRel, 50);
            p.setValue (p.apvts.getParameter (P::phase), (float) mode);
            p.setValue (p.apvts.getParameter (P::linRes), 1.0f);   // Low
            p.editorOpen = true;
            p.prepareToPlay (48000, 512);
            juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi;
            int64_t t = 0;
            auto run = [&] (int blocks, float amp, bool pace, float* maxStep = nullptr)
            {
                float peak = 0, last = 0; bool have = false;
                for (int k = 0; k < blocks; ++k)
                {
                    for (int i = 0; i < 512; ++i, ++t)
                    {
                        const float v = amp * (float) std::sin (twoPi * 1000.0 * (double) t / 48000);
                        buf.setSample (0, i, v); buf.setSample (1, i, v);
                    }
                    p.processBlock (buf, midi);
                    if (pace) std::this_thread::sleep_for (std::chrono::milliseconds (2));
                    peak = 0;
                    for (int i = 0; i < 512; ++i)
                    {
                        const float y = buf.getSample (0, i);
                        peak = std::max (peak, std::abs (y));
                        if (maxStep != nullptr && have) *maxStep = std::max (*maxStep, std::abs (y - last));
                        last = y; have = true;
                    }
                }
                return peak;
            };
            const float loud = run (150, 0.3f, true);
            const double gLoud = juce::Decibels::gainToDecibels (loud / 0.3f);
            const float dynNow = p.getDynOffset (b);
            audiolab::eq::LinearPhase::DisplayData dd;
            p.getFirDisplay (dd);
            int k1 = 0;
            for (int i = 0; i < audiolab::eq::LinearPhase::displayPoints; ++i) if (std::abs (dd.freq[(size_t) i] - 1000.0f) < std::abs (dd.freq[(size_t) k1] - 1000.0f)) k1 = i;
            const float quiet = run (150, 0.001f, true);
            const double gQuiet = juce::Decibels::gainToDecibels (quiet / 0.001f);
            std::printf ("   %s : latence %d ; sinus 1 kHz fort → gain mesuré %.2f dB (détecteur %.2f dB) ; sinus faible → %.2f dB ;\n"
                         "             courbe du FIR à 1 kHz %.2f dB, bandes ajoutées par l'écran : moteur %#x, mélange %#x\n",
                         mn, p.getLatencySamples(), gLoud, dynNow, gQuiet, dd.lr0[(size_t) k1], dd.engineMask, dd.mixMask);
            CHECK (p.getLatencySamples() == (mode == 1 ? 2304 : 384), "%s : latence %d", mn, p.getLatencySamples());
            CHECK (std::abs (gLoud + 10.0) < 0.15 && std::abs (dynNow + 10.0f) < 0.05f, "%s : gain dynamique au centre (%.2f dB)", mn, gLoud);
            CHECK (std::abs (gQuiet) < 0.05, "%s : sous le seuil, gain %.2f dB", mn, gQuiet);
            CHECK (dd.valid && std::abs (dd.lr0[(size_t) k1]) < 0.1f, "%s : la courbe du FIR ne doit pas contenir le mouvement de la bande dynamique", mn);
            CHECK (mode == 1 ? (dd.mixMask == (1u << b) && dd.engineMask == 0) : (dd.engineMask == (1u << b) && dd.mixMask == 0), "%s : bandes signalées à l'écran", mn);

            // passage dyn off / on d'une bande à +6 dB sous son seuil, pendant un sinus : pas de saut
            p.setBandValue (b, P::fGain, 6.0f); p.setBandValue (b, P::fThr, 0.0f);
            run (120, 0.05f, true);
            float stepRef = 0, stepSw = 0;
            run (40, 0.05f, true, &stepRef);
            p.setBandValue (b, P::fDyn, 0);
            run (60, 0.05f, true, &stepSw);
            p.setBandValue (b, P::fDyn, 1);
            const float pk = run (60, 0.05f, true, &stepSw);
            std::printf ("             dyn off puis on sur un sinus : saut max entre deux échantillons %.5f (sans bascule %.5f), niveau final %.2f dB\n",
                         stepSw, stepRef, juce::Decibels::gainToDecibels (pk / 0.05f));
            CHECK (stepSw <= stepRef * 1.02f, "%s : clic au passage dyn on / off (%.5f contre %.5f)", mn, stepSw, stepRef);
        }
        // Linear phase : 6 bandes dynamiques dont 3 cloches larges très proches (bandes en trop : méthode 2.0), sortie valide
        {
            AudioLabEqProcessor p;
            p.setPlayConfigDetails (2, 2, 48000, 512);
            p.clearAllBands();
            const float fr[6] = { 1000, 1200, 1440, 60, 12000, 250 };
            for (int i = 0; i < 6; ++i)
            {
                const int b = p.createBand (audiolab::eq::Shape::bell, fr[i], 2.0f, i < 3 ? 1.0f : 8.0f, 12);
                p.setBandValue (b, P::fDyn, 1); p.setBandValue (b, P::fThr, -50); p.setBandValue (b, P::fRange, i < 3 ? -12.0f : -6.0f);
            }
            p.setValue (p.apvts.getParameter (P::phase), 1.0f);
            p.prepareToPlay (48000, 512);
            juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi; juce::Random rnd (2);
            bool bad = false;
            for (int k = 0; k < 200; ++k)
            {
                for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 512; ++i) buf.setSample (ch, i, (rnd.nextFloat() - 0.5f) * (k % 40 < 20 ? 0.5f : 0.02f));
                p.processBlock (buf, midi);
                std::this_thread::sleep_for (std::chrono::milliseconds (2));
                for (int i = 0; i < 512; ++i) if (! std::isfinite (buf.getSample (0, i)) || std::abs (buf.getSample (0, i)) > 10) bad = true;
            }
            std::printf ("   Linear, 6 bandes dynamiques dont 2 en méthode 2.0 (%d conceptions en 2 s) : %s\n", p.firDesignCount(), bad ? "PROBLÈME" : "OK");
            CHECK (! bad, "sortie invalide avec des bandes dynamiques en trop");
        }
    }

    // ── 3.0 : nouveaux paramètres et relecture des anciens états ──────────
    // Un état 2.0.0 (ou 1.x) ne contient ni bN_slopefree, ni bN_brick, ni bN_thrauto : il doit redonner les
    // mêmes formes, les mêmes pentes et le même son, quoi que le plugin ait eu comme réglages avant.
    {
        namespace E = audiolab::eq;
        const int slopes[] = { 12, 24, 18, 6, 48, 12, 36, 12 };
        auto build = [&] (AudioLabEqProcessor& p)
        {
            p.setPlayConfigDetails (2, 2, 48000, 512);
            p.prepareToPlay (48000, 512);
            p.clearAllBands();
            for (int b = 0; b < E::numLegacyShapes; ++b)   // les 8 formes des versions 1.x et 2.0
                p.createBand ((E::Shape) b, 60.0f * std::pow (2.1f, (float) b), (b % 2) ? 4.0f : -5.0f, 0.9f + 0.2f * (float) b, slopes[b], (E::Place) (b % 5));
            p.setBandValue (0, P::fDyn, 1); p.setBandValue (0, P::fThr, -40); p.setBandValue (0, P::fRange, -8);
        };
        auto render = [] (AudioLabEqProcessor& p, std::vector<float>& out)
        {
            juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi; juce::Random rnd (5);
            out.clear();
            for (int k = 0; k < 60; ++k)
            {
                for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 512; ++i) buf.setSample (ch, i, rnd.nextFloat() * 0.6f - 0.3f);
                p.processBlock (buf, midi);
                for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 512; ++i) out.push_back (buf.getSample (ch, i));
            }
        };
        auto dirty = [] (AudioLabEqProcessor& p)   // réglages 3.0 non neutres, qui ne doivent pas survivre à la relecture
        {
            for (int b = 0; b < E::maxBands; ++b) { p.setBandValue (b, P::fSlopeFree, 33.3f); p.setBandValue (b, P::fBrick, 1); p.setBandValue (b, P::fThrAuto, 1); p.setBandValue (b, P::fShape, 9); }
        };
        auto sameSettings = [] (AudioLabEqProcessor& a, AudioLabEqProcessor& b)
        {
            bool ok = true;
            for (int k = 0; k < E::maxBands; ++k)
                for (int f = 0; f < P::v2BandFields; ++f)
                    ok = ok && std::abs (a.bandValue (k, P::bandFields[f]) - b.bandValue (k, P::bandFields[f])) <= 1e-4f * std::max (1.0f, std::abs (a.bandValue (k, P::bandFields[f])));
            return ok;
        };
        auto neutral3 = [] (AudioLabEqProcessor& a)
        {
            bool ok = true;
            for (int k = 0; k < E::maxBands; ++k)
                ok = ok && a.bandValue (k, P::fSlopeFree) == 0.0f && a.bandValue (k, P::fBrick) < 0.5f && a.bandValue (k, P::fThrAuto) < 0.5f;
            return ok;
        };

        AudioLabEqProcessor p; build (p);
        juce::MemoryBlock mb; p.getStateInformation (mb);
        auto st = juce::ValueTree::fromXml (*juce::AudioProcessor::getXmlFromBinary (mb.getData(), (int) mb.getSize()));
        int removed = 0; bool shapeIsIndex = false;
        for (int i = st.getNumChildren(); --i >= 0;)
        {
            const auto id = st.getChild (i).getProperty ("id").toString();
            if (id.endsWith ("_slopefree") || id.endsWith ("_brick") || id.endsWith ("_thrauto")) { st.removeChild (i, nullptr); ++removed; }
            // (l'état garde l'index de la forme, pas sa valeur normalisée : il ne dépend pas du nombre de formes)
            if (id == "b3_shape") shapeIsIndex = std::abs ((float) st.getChild (i).getProperty ("value") - 2.0f) < 1e-6f;
        }
        CHECK (removed == 3 * E::maxBands && shapeIsIndex, "état 2.0.0 reconstitué (%d paramètres 3.0 retirés)", removed);
        juce::MemoryBlock old; juce::AudioProcessor::copyXmlToBinary (*st.createXml(), old);

        AudioLabEqProcessor q; build (q); dirty (q);
        q.setStateInformation (old.getData(), (int) old.getSize());
        const bool okSet = sameSettings (p, q), okNeutral = neutral3 (q);
        bool okShapes = true;
        for (int b = 0; b < E::numLegacyShapes; ++b)
            okShapes = okShapes && juce::roundToInt (q.bandValue (b, P::fShape)) == b && E::slopeValues[juce::roundToInt (q.bandValue (b, P::fSlope))] == slopes[b];
        // même son : p n'a jamais quitté les réglages neutres, q revient d'un état 2.0.0 après des réglages 3.0
        AudioLabEqProcessor p2; build (p2); p2.prepareToPlay (48000, 512);
        AudioLabEqProcessor q2; build (q2); dirty (q2); q2.setStateInformation (old.getData(), (int) old.getSize());
        q2.prepareToPlay (48000, 512);
        std::vector<float> a, c; render (p2, a); render (q2, c);
        float diff = 0; for (size_t i = 0; i < a.size(); ++i) diff = std::max (diff, std::abs (a[i] - c[i]));
        std::printf ("13. État 2.0.0 relu (sans les paramètres 3.0, après des réglages 3.0 non neutres) : formes et pentes %s, autres réglages %s,\n"
                     "    pente libre / Brickwall / seuil automatique remis au neutre : %s ; écart max de la sortie avec un plugin jamais déréglé : %.1e [seuil 0]\n",
                     okShapes ? "identiques" : "DIFFÉRENTES", okSet ? "identiques" : "DIFFÉRENTS", okNeutral ? "oui" : "NON", diff);
        CHECK (okShapes && okSet && okNeutral, "relecture d'un état 2.0.0");
        CHECK (diff == 0.0f, "un état 2.0.0 relu ne donne pas le même son (%.2e)", diff);

        // A/B et annuler aux anciens formats : 2.0 (14 champs par bande, 5 réglages généraux) et 1.x (13 et 4).
        // La forme y est une valeur normalisée sur 8 choix (index / 7).
        auto oldVector = [&] (int fields, int globals)
        {
            const auto v = p.getStateVector();
            std::vector<float> o;
            for (int b = 0; b < E::maxBands; ++b)
                for (int f = 0; f < fields; ++f)
                    o.push_back (std::strcmp (P::bandFields[f], P::fShape) == 0 ? (float) juce::roundToInt (p.bandValue (b, P::fShape)) / (float) (E::numLegacyShapes - 1)
                                                                               : v[(size_t) (b * P::numBandFields + f)]);
            for (int g = 0; g < globals; ++g) o.push_back (v[(size_t) (E::maxBands * P::numBandFields + g)]);
            return o;
        };
        bool okAB = true, okUndo = true;
        for (int fmt = 0; fmt < 2; ++fmt)
        {
            const auto o = oldVector (fmt == 0 ? P::v2BandFields : P::legacyBandFields, fmt == 0 ? 5 : P::legacyGlobals);
            juce::StringArray sa; for (float x : o) sa.add (juce::String (x, 7));
            // A/B : l'état B vient d'une ancienne session, A a des réglages 3.0
            AudioLabEqProcessor r; build (r); dirty (r);
            r.ui.setProperty ("slotB", sa.joinIntoString (" "), nullptr);
            r.switchAB ("B");
            bool sh = true;
            for (int b = 0; b < E::numLegacyShapes; ++b)
                sh = sh && juce::roundToInt (r.bandValue (b, P::fShape)) == b && E::slopeValues[juce::roundToInt (r.bandValue (b, P::fSlope))] == slopes[b];
            // (1.x : pas de side-chain dans l'état, il revient à Internal comme dans p)
            okAB = okAB && sh && neutral3 (r) && sameSettings (p, r);
            // retour à A : les réglages 3.0 de A sont toujours là (format actuel)
            r.switchAB ("A");
            okAB = okAB && r.bandValue (2, P::fBrick) > 0.5f && std::abs (r.bandValue (2, P::fSlopeFree) - 33.3f) < 0.01f && juce::roundToInt (r.bandValue (2, P::fShape)) == 9;
            // annuler : un état de l'historique à l'ancien format
            AudioLabEqProcessor u; build (u); dirty (u);
            u.setStateVector (o);
            bool su = true;
            for (int b = 0; b < E::numLegacyShapes; ++b)
                su = su && juce::roundToInt (u.bandValue (b, P::fShape)) == b && E::slopeValues[juce::roundToInt (u.bandValue (b, P::fSlope))] == slopes[b];
            okUndo = okUndo && su && neutral3 (u) && sameSettings (p, u);
        }
        std::printf ("14. A/B et annuler aux anciens formats (2.0 : 14 champs et 5 réglages généraux ; 1.x : 13 et 4) : A/B %s, annuler %s\n", okAB ? "OK" : "ÉCHEC", okUndo ? "OK" : "ÉCHEC");
        CHECK (okAB, "A/B à l'ancien format");
        CHECK (okUndo, "annuler à l'ancien format");

        // Format actuel : les réglages 3.0 sont sauvegardés, relus, et suivent A/B et annuler
        {
            AudioLabEqProcessor n; build (n);
            n.setBandValue (2, P::fSlopeFree, 17.5f); n.setBandValue (4, P::fBrick, 1); n.setBandValue (0, P::fThrAuto, 1);
            n.setBandValue (5, P::fShape, (float) (int) E::Shape::flatTilt); n.setBandValue (6, P::fShape, (float) (int) E::Shape::allPass);
            juce::MemoryBlock m3; n.getStateInformation (m3);
            AudioLabEqProcessor n2; n2.setStateInformation (m3.getData(), (int) m3.getSize());
            const bool okSave = std::abs (n2.bandValue (2, P::fSlopeFree) - 17.5f) < 0.01f && n2.bandValue (4, P::fBrick) > 0.5f && n2.bandValue (0, P::fThrAuto) > 0.5f
                             && juce::roundToInt (n2.bandValue (5, P::fShape)) == 8 && juce::roundToInt (n2.bandValue (6, P::fShape)) == 9;
            const auto spec = n2.getBandSpec (2), spec4 = n2.getBandSpec (4);
            n.pushUndo(); n.setBandValue (2, P::fSlopeFree, 0.0f); n.setBandValue (5, P::fShape, 0);
            const bool okU = n.undo() && std::abs (n.bandValue (2, P::fSlopeFree) - 17.5f) < 0.01f && juce::roundToInt (n.bandValue (5, P::fShape)) == 8;
            n.switchAB ("B"); n.setBandValue (4, P::fBrick, 0); n.switchAB ("A");
            const bool okB = n.bandValue (4, P::fBrick) > 0.5f;
            n.switchAB ("B");
            std::printf ("15. Réglages 3.0 : sauvegarde / relecture %s (bande 3 : %.1f dB/oct, bande 5 : Brickwall %s), annuler %s, A/B %s\n",
                         okSave ? "OK" : "ÉCHEC", audiolab::eq::cutSlope (spec), audiolab::eq::isBrick (spec4) ? "oui" : "non", okU ? "OK" : "ÉCHEC", okB && n.bandValue (4, P::fBrick) < 0.5f ? "OK" : "ÉCHEC");
            CHECK (okSave && okU && okB && n.bandValue (4, P::fBrick) < 0.5f, "réglages 3.0 : sauvegarde, annuler, A/B");
        }

        // Les 10 formes, pentes libres et Brickwall dans les trois modes de phase : sortie valide
        {
            AudioLabEqProcessor m; m.setPlayConfigDetails (2, 2, 44100, 512); m.prepareToPlay (44100, 512);
            m.clearAllBands();
            for (int b = 0; b < 20; ++b)
            {
                const int k = m.createBand ((E::Shape) (b % E::numShapes), 40.0f * std::pow (1.35f, (float) b), (float) (b % 5) - 2.0f, 1.0f, 12, (E::Place) (b % 5));
                if (b % 3 == 0) m.setBandValue (k, P::fSlopeFree, 7.0f + 4.3f * (float) b);
                if (b == 4 || b == 12) m.setBandValue (k, P::fBrick, 1);
                if (b % 4 == 0) { m.setBandValue (k, P::fDyn, 1); m.setBandValue (k, P::fThrAuto, 1); }
            }
            juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi; juce::Random rnd (9);
            bool bad = false; float pk = 0;
            for (int mode = 0; mode < 3; ++mode)
            {
                m.setValue (m.apvts.getParameter (P::phase), (float) mode);
                for (int k = 0; k < 120; ++k)
                {
                    for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 512; ++i) buf.setSample (ch, i, rnd.nextFloat() * 0.2f - 0.1f);
                    m.processBlock (buf, midi);
                    for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 512; ++i) { const float v = buf.getSample (ch, i); if (! std::isfinite (v)) bad = true; pk = std::max (pk, std::abs (v)); }
                    if (mode > 0) std::this_thread::sleep_for (std::chrono::milliseconds (2));
                }
            }
            std::printf ("16. 20 bandes (10 formes, pentes libres, Brickwall, seuil automatique), Zero latency puis Linear puis Natural phase : %s (crête %.2f)\n", bad ? "PROBLÈME" : "OK", pk);
            CHECK (! bad && pk < 100.0f, "sortie invalide avec les formes 3.0");
        }
    }

    // ── 3.0 : Auto gain à sonie égale, pré-écho, résolution Auto, changement de mode sans silence ──
    {
        namespace E = audiolab::eq;
        namespace L = audiolab::loud;
        // 17. Auto gain dans le plugin complet, trois modes de phase : sonie intégrée de la sortie = celle de l'entrée.
        //     Signal en rafales (bruit rose 150 ms, silence 350 ms) : si l'entrée n'était pas recalée sur la sortie
        //     (latence des modes FIR), la porte ne retiendrait pas les mêmes tranches des deux côtés.
        std::printf ("17. Auto gain à sonie égale dans le plugin (étagère grave +6 dB à 120 Hz, cloche +9 dB à 3 kHz Q 1), bruit rose en rafales, 20 s\n");
        for (int mode = 0; mode < 3; ++mode)
        {
            AudioLabEqProcessor p;
            p.setPlayConfigDetails (2, 2, 48000, 512);
            p.clearAllBands();
            p.createBand (E::Shape::lowShelf, 120.0f, 6.0f, 0.71f, 12);
            p.createBand (E::Shape::bell, 3000.0f, 9.0f, 1.0f, 12);
            p.setValue (p.apvts.getParameter (P::phase), (float) mode);
            p.setValue (p.apvts.getParameter (P::linRes), 3.0f);   // High : 8448 échantillons de latence (176 ms)
            p.setValue (p.apvts.getParameter (P::autoGain), 1.0f);
            p.prepareToPlay (48000, 512);
            const float g0 = p.getAutoGainDb();
            const int lat = p.getLatencySamples();
            juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi;
            Pink p1 (5), p2 (6);
            L::Meter mi, mo; mi.prepare (48000); mo.prepare (48000);
            std::vector<float> inL, inR, outL, outR;
            const int blocks = (int) (20.0 * 48000 / 512);
            int64_t t = 0;
            for (int k = 0; k < blocks; ++k)
            {
                for (int i = 0; i < 512; ++i, ++t)
                {
                    const bool on = (t % 24000) < 7200;
                    const float a = p1.next(), b = p2.next();
                    buf.setSample (0, i, on ? a : 0.0f); buf.setSample (1, i, on ? 0.6f * a + 0.8f * b : 0.0f);
                }
                for (int i = 0; i < 512; ++i) { inL.push_back (buf.getSample (0, i)); inR.push_back (buf.getSample (1, i)); }
                p.processBlock (buf, midi);
                for (int i = 0; i < 512; ++i) { outL.push_back (buf.getSample (0, i)); outR.push_back (buf.getSample (1, i)); }
                if (mode > 0 && k < 40) std::this_thread::sleep_for (std::chrono::milliseconds (2));
            }
            // de 8 à 20 s ; la sortie est décalée de la latence
            const int s0 = 8 * 48000, len = (int) inL.size() - s0 - lat;
            mi.process (inL.data() + s0, inR.data() + s0, len); mo.process (outL.data() + s0 + lat, outR.data() + s0 + lat, len);
            const double d = mo.integrated() - mi.integrated();
            std::printf ("   %s : latence %5d ; gain de départ %+.2f dB (prévision), final %+.2f dB ; sonie entrée %.2f LUFS, sortie %.2f LUFS : écart %+.3f LU [seuil 0.1]\n",
                         mode == 0 ? "Zero latency" : mode == 1 ? "Linear High " : "Natural     ", lat, g0, p.getAutoGainDb(), mi.integrated(), mo.integrated(), d);
            CHECK (std::abs (d) <= 0.1, "Auto gain, mode %d : écart de sonie %.3f LU", mode, d);
        }
        // Auto gain coupé : la valeur affichée est la prévision, et la sortie n'en dépend pas ; aucun paramètre ajouté
        {
            AudioLabEqProcessor p; p.setPlayConfigDetails (2, 2, 48000, 512);
            p.applyPreset (4);
            p.prepareToPlay (48000, 512);
            const float shown = p.getAutoGainDb();
            int nParams = p.getParameters().size();
            std::printf ("   Auto gain coupé : valeur affichée %+.2f dB (prévision pour un spectre rose) ; %d paramètres (inchangé : 24 × 17 + 6 = 414)\n", shown, nParams);
            CHECK (std::isfinite (shown) && std::abs (shown) <= 24.0f, "valeur affichée de l'Auto gain");
            CHECK (nParams == 24 * P::numBandFields + 6, "nombre de paramètres (%d)", nParams);
        }

        // 18. Auto gain coupé : sortie bit à bit identique à celle de la version d'avant (fichier écrit par le
        //     même test compilé contre l'ancien code : ALEQProcessorTest --dump <fichier>)
        {
            const char* ref = nullptr;
            for (int i = 1; i + 1 < argc; ++i) if (std::string (argv[i]) == "--compare") ref = argv[i + 1];
            if (argc > 2 && std::string (argv[1]) == "--dump")
            {
                const auto v = renderReference();
                if (FILE* f = std::fopen (argv[2], "wb")) { std::fwrite (v.data(), sizeof (float), v.size(), f); std::fclose (f); }
                std::printf ("18. référence écrite : %d échantillons\n", (int) v.size());
            }
            else if (ref == nullptr) std::printf ("18. Auto gain coupé, sortie identique à la version d'avant : NON EXÉCUTÉ (lancer avec --compare <fichier de référence>)\n");
            else
            {
                const auto v = renderReference();
                std::vector<float> old (v.size() + 16);
                size_t got = 0;
                if (FILE* f = std::fopen (ref, "rb")) { got = std::fread (old.data(), sizeof (float), old.size(), f); std::fclose (f); }
                size_t diff = 0; float worst = 0;
                for (size_t i = 0; i < std::min (got, v.size()); ++i)
                    if (std::memcmp (&v[i], &old[i], sizeof (float)) != 0) { ++diff; worst = std::max (worst, std::abs (v[i] - old[i])); }
                std::printf ("18. Auto gain coupé, 12 rendus (3 modes de phase, bandes fixes / dynamiques / Mid-Side, stéréo et mono, 44,1 / 48 / 96 kHz) :\n"
                             "    %d échantillons comparés à la version d'avant, %d différents (écart max %.1e) [seuil 0]\n", (int) v.size(), (int) diff, worst);
                CHECK (got == v.size() && diff == 0, "la sortie a changé avec l'Auto gain coupé (%d échantillons, référence de %d)", (int) diff, (int) got);
            }
        }

        // 19. Pré-écho annoncé par le plugin = pré-écho de sa réponse impulsionnelle
        std::printf ("19. Pré-écho annoncé par le plugin, contre sa réponse impulsionnelle mesurée (coupe-bas 80 Hz 24 dB/oct + cloche 60 Hz +4 dB Q 4)\n");
        for (int cfg = 0; cfg < 4; ++cfg)
        {
            const int mode = cfg == 0 ? 0 : cfg == 3 ? 2 : 1, res = cfg == 2 ? 4 : 2;
            AudioLabEqProcessor p; p.setPlayConfigDetails (2, 2, 48000, 512);
            p.clearAllBands();
            p.createBand (E::Shape::lowCut, 80.0f, 0.0f, 0.71f, 24);
            p.createBand (E::Shape::bell, 60.0f, 4.0f, 4.0f, 12);
            p.setValue (p.apvts.getParameter (P::phase), (float) mode);
            p.setValue (p.apvts.getParameter (P::linRes), (float) res);
            p.prepareToPlay (48000, 512);
            juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi;
            settle (p, buf, midi, 40);
            const int lat = p.getLatencySamples();
            std::vector<float> ir;
            for (int k = 0; (int) ir.size() < lat + 8192; ++k)
            {
                buf.clear(); if (k == 0) { buf.setSample (0, 0, 1.0f); buf.setSample (1, 0, 1.0f); }
                p.processBlock (buf, midi);
                for (int i = 0; i < 512; ++i) ir.push_back (buf.getSample (0, i));
            }
            const auto said = p.getPreEcho();
            const auto meas = E::fir::measurePreEcho (ir.data(), (const float*) nullptr, (int) ir.size(), 48000.0);
            std::printf ("   %s : annoncé %7.2f ms > −60 dB, %7.2f ms > −90 dB, %6.1f dB à 5 ms, %6.1f dB à 20 ms ; mesuré %7.2f / %7.2f ms, %6.1f / %6.1f dB ; « %s »\n",
                         cfg == 0 ? "Zero latency " : cfg == 1 ? "Linear Medium" : cfg == 2 ? "Linear Max   " : "Natural      ",
                         said.ms60, said.ms90, said.dbAt5ms, said.dbAt20ms, meas.ms60, meas.ms90, meas.dbAt5ms, meas.dbAt20ms, p.getPreEchoText().toRawUTF8());
            if (mode == 0)
            {
                // (zéro latence : rien n'est annoncé ; la réponse d'un filtre récursif commence à son pic ou juste avant lui)
                CHECK (said.ms60 == 0.0f && said.ms90 == 0.0f && said.dbAt5ms == E::fir::preEchoFloorDb && p.getPreEchoText().isEmpty(), "pré-écho en Zero latency");
            }
            else
            {
                CHECK (std::abs (said.ms60 - meas.ms60) <= 0.05f + 0.01f * meas.ms60 && std::abs (said.ms90 - meas.ms90) <= 0.5f + 0.05f * meas.ms90,
                       "pré-écho annoncé / mesuré (%.2f / %.2f ms contre %.2f / %.2f)", said.ms60, said.ms90, meas.ms60, meas.ms90);
                CHECK (std::abs (said.dbAt5ms - meas.dbAt5ms) <= 0.5f && std::abs (said.dbAt20ms - meas.dbAt20ms) <= 0.5f, "niveau du pré-écho annoncé / mesuré");
                CHECK (p.getPreEchoText().startsWith ("Pre-echo: "), "texte du pré-écho");
            }
            if (mode == 2) CHECK (said.ms60 <= 3.0f, "Natural phase : pré-écho %.2f ms", said.ms60);
        }

        // 20. Résolution Auto vérifiée par la mesure, dans le plugin
        {
            AudioLabEqProcessor p; p.setPlayConfigDetails (2, 2, 48000, 512);
            p.clearAllBands();
            p.createBand (E::Shape::bell, 30.0f, -12.0f, 0.5f, 12);
            p.setValue (p.apvts.getParameter (P::phase), (float) P::phaseLinear);
            p.setValue (p.apvts.getParameter (P::linRes), 0.0f);
            p.testPlaying = 0;
            p.prepareToPlay (48000, 512);
            const int lat0 = p.getLatencySamples();
            // à l'arrêt, une autre bande arrive : la règle donne d'abord sa longueur, la mesure l'allonge ensuite
            juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi;
            p.deleteBand (0);
            settle (p, buf, midi, 120);
            const int lat1 = p.getLatencySamples();
            p.createBand (E::Shape::lowShelf, 30.0f, 9.0f, 0.71f, 12);
            settle (p, buf, midi, 160);
            const int lat2 = p.getLatencySamples();
            std::printf ("20. Résolution Auto dans le plugin : cloche 30 Hz −12 dB Q 0.5 → latence %d (règle seule : 2304) ; sans bande → %d ; étagère grave 30 Hz +9 dB → %d (règle seule : 2304)\n", lat0, lat1, lat2);
            CHECK (lat0 == 8192 + 256, "résolution Auto vérifiée au départ (%d)", lat0);
            CHECK (lat1 == 2048 + 256, "résolution Auto, courbe plate (%d)", lat1);
            CHECK (lat2 == 4096 + 256, "résolution Auto vérifiée à l'arrêt (%d)", lat2);
        }

        // 21. Changement de mode ou de résolution pendant la lecture : sans silence, sans clic
        std::printf ("21. Changement de mode ou de résolution pendant la lecture (48 kHz, blocs de 64, cadence temps réel) : l'ancien chemin continue pendant la conception du FIR,\n"
                     "    puis fondu enchaîné de 20 ms entre les deux chemins, chacun avec sa latence (gains à puissance constante pour la corrélation mesurée des deux sorties)\n");
        const auto s = measureSwitches (4);
        std::printf ("    pire cas : trou %.1f ms [seuil 0] ; bruit rose : niveau de la transition %+.2f … %+.2f dB [seuils −4.5 et +3.0] ; sinus : creux %.2f dB [−7], bosse %+.2f dB [+1] ;\n"
                     "    saut entre deux échantillons × %.2f [1.25] ; transition la plus longue %.1f ms [21] ; latence annoncée au plus %.0f ms après la demande, %.0f ms avant le milieu du fondu\n",
                     s.hole, s.levelLo, s.levelHi, s.dip, s.bump, s.step, s.trans, s.lat, s.latToAudio);
        CHECK (s.bad == 0, "%d changement(s) de mode sans mesure", s.bad);
        // Seuils posés au niveau atteint. Le creux des sinus est celui des cas en opposition de phase (les deux
        // chemins s'annulent en partie au milieu du fondu : aucun fondu d'amplitude ne l'évite).
        CHECK (s.hole == 0.0, "trou pendant un changement de mode (%.1f ms)", s.hole);
        CHECK (s.levelLo >= -4.5 && s.levelHi <= 3.0 && s.levelMean <= 1.5, "bruit rose : niveau de la transition %+.2f … %+.2f dB (moyenne %.2f)", s.levelLo, s.levelHi, s.levelMean);
        CHECK (s.dip >= -7.0 && s.bump <= 1.0, "sinus : creux %.2f dB, bosse %+.2f dB", s.dip, s.bump);
        CHECK (s.step <= 1.25, "saut entre deux échantillons × %.2f", s.step);
        CHECK (s.trans <= 21.0, "transition de %.1f ms", s.trans);
        CHECK (s.lat <= 150.0, "latence annoncée %.0f ms après la demande", s.lat);
        swSummary = s;
    }

    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
#endif
