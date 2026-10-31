// Banc de test du moteur DSP d'Audio Lab EQ (hors plugin) :
//  1. réponse impulsionnelle de chaque forme de filtre = réponse analytique (courbe affichée)
//  2. placement Mid / Side / Gauche / Droite
//  3. EQ dynamique : gain réduit au-dessus du seuil, inchangé en dessous
//  4. phase linéaire : latence N/2, réponse = prototype analogique, symétrie de la RI
//  5. écoute solo, robustesse (automatisation aléatoire : pas de NaN, pas d'explosion)
#include <juce_dsp/juce_dsp.h>
#include "EqEngine.h"
#include "LinearPhase.h"
#include <thread>
#include <cstdio>

using namespace audiolab::eq;

static int failures = 0;
#define CHECK(cond, ...) do { if (! (cond)) { ++failures; std::printf ("  ÉCHEC : "); std::printf (__VA_ARGS__); std::printf ("\n"); } } while (0)

static std::vector<float> fftMagDb (const std::vector<float>& x, int order)
{
    const int n = 1 << order;
    juce::dsp::FFT fft (order);
    std::vector<float> d ((size_t) n * 2, 0.0f);
    std::copy (x.begin(), x.begin() + std::min ((size_t) n, x.size()), d.begin());
    fft.performFrequencyOnlyForwardTransform (d.data(), true);
    std::vector<float> m ((size_t) n / 2 + 1);
    for (int k = 0; k <= n / 2; ++k) m[(size_t) k] = 20.0f * std::log10 (std::max (1e-12f, d[(size_t) k]));
    return m;
}

// Réponse impulsionnelle d'une seule bande dans le moteur zéro latence
static void impulse (Engine& e, double fs, const BandSpec& s, const DynSettings& dyn, std::vector<float>& outL, std::vector<float>& outR,
                     float inL = 1.0f, float inR = 1.0f, int len = 65536)
{
    e.prepare (fs);
    for (int b = 0; b < maxBands; ++b) e.setBand (b, BandSpec(), DynSettings(), 1.0);
    e.setBand (0, s, dyn, 1.0);
    std::vector<float> l (4096, 0.0f), r (4096, 0.0f);
    e.process (l.data(), r.data(), 4096, false);   // fondu d'entrée de la bande
    outL.assign ((size_t) len, 0.0f); outR.assign ((size_t) len, 0.0f);
    outL[0] = inL; outR[0] = inR;
    e.process (outL.data(), outR.data(), len, false);
}

static double maxErrorDb (const std::vector<float>& ir, const BandSpec& s, double fs, double fmin, double fmax, bool analog, double* worstF = nullptr)
{
    const int order = 16;
    auto m = fftMagDb (ir, order);
    Design d; design (s, fs, d, analog);
    double worst = 0;
    const int n = 1 << order;
    for (int k = 1; k < n / 2; ++k)
    {
        const double f = k * fs / n;
        if (f < fmin || f > fmax) continue;
        const double ref = magnitudeDb (d, f, fs, analog);
        if (ref < -60) continue;      // zones très atténuées : l'erreur relative n'a pas de sens
        const double err = std::abs (m[(size_t) k] - ref);
        if (err > worst) { worst = err; if (worstF) *worstF = f; }
    }
    return worst;
}

static double rmsDb (const float* x, int n)
{
    double s = 0; for (int i = 0; i < n; ++i) s += (double) x[i] * x[i];
    return 10.0 * std::log10 (std::max (1e-20, s / n));
}

int main()
{
    Engine eng;
    std::vector<float> L, R;

    std::printf ("1. Formes de filtre : réponse mesurée = réponse affichée\n");
    struct Case { Shape shape; double f, g, q; int slope; };
    const Case cases[] = {
        { Shape::bell, 1000, 6, 1, 12 }, { Shape::bell, 60, -12, 4, 12 }, { Shape::bell, 12000, 9, 0.5, 12 },
        { Shape::lowShelf, 200, 6, 0.71, 12 }, { Shape::lowShelf, 300, -8, 0.71, 6 }, { Shape::highShelf, 6000, 5, 1.2, 12 },
        { Shape::highShelf, 4000, -6, 0.71, 6 }, { Shape::lowCut, 80, 0, 0.71, 6 }, { Shape::lowCut, 80, 0, 0.71, 12 },
        { Shape::lowCut, 100, 0, 0.71, 18 }, { Shape::lowCut, 120, 0, 2.0, 24 }, { Shape::lowCut, 60, 0, 0.71, 48 },
        { Shape::lowCut, 40, 0, 0.71, 96 }, { Shape::highCut, 8000, 0, 0.71, 12 }, { Shape::highCut, 5000, 0, 0.71, 30 },
        { Shape::highCut, 2000, 0, 3.0, 72 }, { Shape::notch, 1000, 0, 4, 12 }, { Shape::bandPass, 1500, 0, 1, 12 },
        { Shape::bandPass, 800, 0, 2, 36 }, { Shape::tiltShelf, 1000, 6, 0.71, 6 }, { Shape::tiltShelf, 800, -4, 0.71, 12 },
    };
    const char* names[] = { "cloche", "low shelf", "coupe-bas", "high shelf", "coupe-haut", "notch", "passe-bande", "tilt" };
    for (double fs : { 44100.0, 48000.0, 96000.0 })
        for (const auto& c : cases)
        {
            BandSpec s; s.active = true; s.shape = c.shape; s.freq = c.f; s.gainDb = c.g; s.q = c.q; s.slope = c.slope;
            impulse (eng, fs, s, {}, L, R);
            double wf = 0;
            const double e = maxErrorDb (L, s, fs, 20, 0.45 * fs, false, &wf);
            const double eR = maxErrorDb (R, s, fs, 20, 0.45 * fs, false);
            std::printf ("   %-11s %6.0f Hz %+5.1f dB Q%.2f %2d dB/oct  @%.0f kHz : écart max %.4f dB\n",
                         names[(int) c.shape], c.f, c.g, c.q, c.slope, fs / 1000, std::max (e, eR), std::max (e, eR));
            CHECK (std::max (e, eR) < 0.05, "%s %.0f Hz : écart %.3f dB à %.0f Hz", names[(int) c.shape], c.f, e, wf);
        }

    std::printf ("2. Placement stéréo\n");
    {
        const double fs = 48000;
        BandSpec s; s.active = true; s.shape = Shape::bell; s.freq = 1000; s.gainDb = 12; s.q = 1;
        Design d; design (s, fs, d, false);
        auto peakAt1k = [&] (const std::vector<float>& x) { auto m = fftMagDb (x, 16); return m[(size_t) std::lround (1000.0 * 65536 / fs)]; };
        s.place = Place::mid;
        impulse (eng, fs, s, {}, L, R, 1.0f, 1.0f);   // signal mono (Mid seul)
        CHECK (std::abs (peakAt1k (L) - 12) < 0.1, "Mid : signal mono non traité (%.2f dB)", peakAt1k (L));
        impulse (eng, fs, s, {}, L, R, 1.0f, -1.0f);  // signal Side seul
        CHECK (std::abs (peakAt1k (L)) < 0.1, "Mid : le Side ne doit pas être traité (%.2f dB)", peakAt1k (L));
        s.place = Place::side;
        impulse (eng, fs, s, {}, L, R, 1.0f, -1.0f);
        CHECK (std::abs (peakAt1k (R) - 12) < 0.1, "Side : signal Side non traité (%.2f dB)", peakAt1k (R));
        impulse (eng, fs, s, {}, L, R, 1.0f, 1.0f);
        CHECK (std::abs (peakAt1k (R)) < 0.1, "Side : le Mid ne doit pas être traité");
        s.place = Place::left;
        impulse (eng, fs, s, {}, L, R);
        CHECK (std::abs (peakAt1k (L) - 12) < 0.1 && std::abs (peakAt1k (R)) < 0.1, "Gauche seulement");
        s.place = Place::right;
        impulse (eng, fs, s, {}, L, R);
        CHECK (std::abs (peakAt1k (R) - 12) < 0.1 && std::abs (peakAt1k (L)) < 0.1, "Droite seulement");
        std::printf ("   Mid, Side, Gauche, Droite : OK si aucun échec ci-dessus\n");
    }

    std::printf ("3. EQ dynamique\n");
    {
        const double fs = 48000;
        BandSpec s; s.active = true; s.shape = Shape::bell; s.freq = 1000; s.gainDb = 0; s.q = 2;
        DynSettings dyn; dyn.on = true; dyn.thrDb = -30; dyn.rangeDb = -10;
        for (double lev : { -10.0, -25.0, -45.0 })
        {
            eng.prepare (fs); eng.setDynTimes (5, 50);
            for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
            eng.setBand (0, s, dyn, 1.0);
            const int n = 96000;
            std::vector<float> l ((size_t) n), r ((size_t) n);
            const double a = std::pow (10.0, lev / 20.0) * std::sqrt (2.0);
            for (int i = 0; i < n; ++i) l[(size_t) i] = r[(size_t) i] = (float) (a * std::sin (2 * pi * 1000 * i / fs));
            eng.process (l.data(), r.data(), n, false);
            const double out = rmsDb (l.data() + n - 9600, 9600);
            // le détecteur suit l'enveloppe (entre niveau moyen et crête du sinus) : on vérifie les bornes
            const double red = lev - out;
            std::printf ("   sinus 1 kHz à %+.0f dBFS → %+.2f dBFS (réduction %.2f dB)\n", lev, out, red);
            if (lev > -15) CHECK (std::abs (red - 10.0) < 0.2, "au-dessus du seuil : réduction plafonnée à la plage (%.2f)", red);
            else if (lev > -30) CHECK (red > 4.0 && red < 10.0, "près du seuil : réduction partielle attendue (%.2f)", red);
            else CHECK (std::abs (red) < 0.1, "sous le seuil : aucun changement attendu (%.2f)", red);
        }
    }

    std::printf ("4. Phase linéaire\n");
    for (double fs : { 48000.0, 96000.0 })
    {
        LinearPhase lp;
        lp.prepare (fs, 512);
        LinearPhase::Snapshot snap;
        snap.stereo = true;
        BandSpec b1; b1.active = true; b1.shape = Shape::bell; b1.freq = 2000; b1.gainDb = 8; b1.q = 1.5;
        BandSpec b2; b2.active = true; b2.shape = Shape::lowCut; b2.freq = 200; b2.slope = 24; b2.q = 0.71;
        BandSpec b3; b3.active = true; b3.shape = Shape::highShelf; b3.freq = 8000; b3.gainDb = -6; b3.q = 0.71; b3.place = Place::side;
        snap.bands[0] = b1; snap.bands[1] = b2; snap.bands[2] = b3;
        lp.design (snap);
        // laisse la file de chargement des RI faire son travail
        std::vector<float> l (512, 0.0f), r (512, 0.0f);
        for (int k = 0; k < 80; ++k) { std::fill (l.begin(), l.end(), 0.0f); std::fill (r.begin(), r.end(), 0.0f); lp.process (l.data(), r.data(), 512); std::this_thread::sleep_for (std::chrono::milliseconds (5)); }
        const int N = lp.latency() * 2, len = 4 * N;
        std::vector<float> xl ((size_t) len, 0.0f), xr ((size_t) len, 0.0f);
        xl[0] = 1; xr[0] = 1;   // impulsion « Mid »
        for (int off = 0; off < len; off += 512) lp.process (xl.data() + off, xr.data() + off, std::min (512, len - off));
        int pk = 0; for (int i = 0; i < len; ++i) if (std::abs (xl[(size_t) i]) > std::abs (xl[(size_t) pk])) pk = i;
        std::printf ("   %.0f kHz : FIR %d points, pic de la RI à %d (latence annoncée %d)\n", fs / 1000, N, pk, lp.latency());
        CHECK (pk == lp.latency(), "latence %d ≠ pic %d", lp.latency(), pk);
        double asym = 0; for (int m = 1; m < N / 2 - 1; ++m) asym = std::max (asym, (double) std::abs (xl[(size_t) (pk + m)] - xl[(size_t) (pk - m)]));
        CHECK (asym < 1e-5, "RI non symétrique (%.2e)", asym);
        // réponse : cloche + coupe-bas (signal mono : la bande Side n'agit pas)
        std::vector<float> ir (xl.begin() + pk - N / 2, xl.begin() + pk + N / 2);
        auto m = fftMagDb (ir, (int) std::log2 (N));
        double worst = 0, wf = 0;
        Design d1, d2; design (b1, fs, d1, true); design (b2, fs, d2, true);
        for (int k = 1; k < N / 2; ++k)
        {
            const double f = k * fs / N;
            if (f < 150 || f > 0.45 * fs) continue;
            const double ref = magnitudeDb (d1, f, fs, true) + magnitudeDb (d2, f, fs, true);
            if (ref < -40) continue;
            const double e = std::abs (m[(size_t) k] - ref);
            if (e > worst) { worst = e; wf = f; }
        }
        std::printf ("   réponse mesurée vs analogique (150 Hz–%.0f kHz) : écart max %.3f dB à %.0f Hz\n", 0.45 * fs / 1000, worst, wf);
        CHECK (worst < 0.5, "phase linéaire : écart %.2f dB à %.0f Hz", worst, wf);
        // Side : impulsion L=1, R=-1 → shelf -6 dB dans l'aigu
        std::fill (xl.begin(), xl.end(), 0.0f); std::fill (xr.begin(), xr.end(), 0.0f);
        lp.reset();
        xl[0] = 1; xr[0] = -1;
        for (int off = 0; off < len; off += 512) lp.process (xl.data() + off, xr.data() + off, std::min (512, len - off));
        std::vector<float> irs (xl.begin() + lp.latency() - N / 2, xl.begin() + lp.latency() + N / 2);
        auto ms = fftMagDb (irs, (int) std::log2 (N));
        const double hf = ms[(size_t) std::lround (18000.0 * N / fs)];
        const double expectHf = magnitudeDb (d1, 18000, fs, true) + magnitudeDb (d2, 18000, fs, true) - 6 + 0.0;
        std::printf ("   Side à 18 kHz : %.2f dB (attendu ≈ %.2f)\n", hf, expectHf);
        CHECK (std::abs (hf - expectHf) < 0.7, "Side en phase linéaire");
    }

    std::printf ("5. Écoute solo et robustesse\n");
    {
        const double fs = 44100;
        eng.prepare (fs);
        juce::Random rnd (42);
        const int blk = 256;
        std::vector<float> l (blk), r (blk);
        double maxAbs = 0; bool nan = false;
        for (int it = 0; it < 2000; ++it)
        {
            if (it % 7 == 0)
                for (int b = 0; b < 8; ++b)
                {
                    BandSpec s; s.active = rnd.nextFloat() < 0.8f;
                    s.shape = (Shape) rnd.nextInt (numShapes); s.slope = slopeValues[rnd.nextInt (numSlopes)];
                    s.place = (Place) rnd.nextInt (numPlaces);
                    s.freq = 10.0 * std::pow (3000.0, rnd.nextDouble()); s.gainDb = rnd.nextDouble() * 40 - 20;
                    s.q = 0.1 * std::pow (100.0, rnd.nextDouble());
                    DynSettings d; d.on = rnd.nextBool(); d.thrDb = -60 + rnd.nextDouble() * 60; d.rangeDb = rnd.nextDouble() * 40 - 20;
                    eng.setBand (b, s, d, 1.0);
                }
            if (it % 50 == 0) eng.setSolo (rnd.nextBool() ? rnd.nextInt (8) : -1);
            for (int i = 0; i < blk; ++i) { l[(size_t) i] = (rnd.nextFloat() * 2 - 1) * 0.5f; r[(size_t) i] = (rnd.nextFloat() * 2 - 1) * 0.5f; }
            eng.process (l.data(), r.data(), blk, false);
            for (int i = 0; i < blk; ++i)
            {
                if (! std::isfinite (l[(size_t) i]) || ! std::isfinite (r[(size_t) i])) nan = true;
                maxAbs = std::max (maxAbs, (double) std::max (std::abs (l[(size_t) i]), std::abs (r[(size_t) i])));
            }
        }
        std::printf ("   automatisation aléatoire (8 bandes, formes/pentes/placements/dynamique/solo) : crête max %.1f, NaN : %s\n", maxAbs, nan ? "oui" : "non");
        CHECK (! nan, "NaN en sortie");
        CHECK (maxAbs < 300, "sortie explosive (%.1f)", maxAbs);

        // Solo d'une cloche à 1 kHz : un sinus à 100 Hz doit disparaître, un sinus à 1 kHz rester
        eng.prepare (fs);
        for (int b = 0; b < maxBands; ++b) eng.setBand (b, BandSpec(), DynSettings(), 1.0);
        BandSpec s; s.active = true; s.shape = Shape::bell; s.freq = 1000; s.gainDb = 0; s.q = 4;
        eng.setBand (3, s, {}, 1.0);
        eng.setSolo (3);
        const int n = 44100;
        std::vector<float> a ((size_t) n), bb ((size_t) n);
        for (int i = 0; i < n; ++i) a[(size_t) i] = bb[(size_t) i] = (float) (0.5 * std::sin (2 * pi * 100 * i / fs));
        eng.process (a.data(), bb.data(), n, false);
        const double lo = rmsDb (a.data() + n / 2, n / 2);
        for (int i = 0; i < n; ++i) a[(size_t) i] = bb[(size_t) i] = (float) (0.5 * std::sin (2 * pi * 1000 * i / fs));
        eng.process (a.data(), bb.data(), n, false);
        const double hi = rmsDb (a.data() + n / 2, n / 2);
        std::printf ("   solo cloche 1 kHz : sinus 100 Hz → %.1f dB, sinus 1 kHz → %.1f dB (entrée -9 dB)\n", lo, hi);
        CHECK (lo < -40 && std::abs (hi + 9.03) < 0.3, "écoute solo");
    }

    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
