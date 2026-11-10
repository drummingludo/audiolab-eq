// Test du plugin complet hors DAW : presets, annuler/rétablir, A/B, sauvegarde de l'état,
// latences (Linear phase Auto / Low…Max, Natural phase), bypass recalé, changement de mode sans clic,
// projets 1.x, side-chain externe, mono, blocs de tailles variables, charge CPU,
// coût des modes FIR avec des bandes dynamiques.
#include "PluginProcessor.h"
#include <thread>
#include <chrono>
#include <ctime>
#include <cstdio>

namespace P = audiolab::params;
static int failures = 0;
#define CHECK(cond, ...) do { if (! (cond)) { ++failures; std::printf ("  ÉCHEC : "); std::printf (__VA_ARGS__); std::printf ("\n"); } } while (0)

// laisse le temps au fil de conception et au changement de latence (fondu, attente de l'historique)
static void settle (AudioLabEqProcessor& p, juce::AudioBuffer<float>& buf, juce::MidiBuffer& midi, int blocks = 70)
{
    for (int k = 0; k < blocks; ++k) { buf.clear(); p.processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::milliseconds (2)); }
}

static int usedCount (AudioLabEqProcessor& p) { int n = 0; for (int b = 0; b < audiolab::eq::maxBands; ++b) n += p.isUsed (b) ? 1 : 0; return n; }

int main()
{
    juce::ScopedJuceInitialiser_GUI init;
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
        buf.clear(); p.processBlock (buf, midi);
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

    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
