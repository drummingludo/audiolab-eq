// Test du plugin complet hors DAW : presets, annuler/rétablir, A/B, sauvegarde de l'état,
// latences (Linear phase Auto / Low…Max, Natural phase), bypass recalé, changement de mode sans clic,
// projets 1.x, side-chain externe, mono, blocs de tailles variables, charge CPU.
#include "PluginProcessor.h"
#include <thread>
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

    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
