// Test du plugin complet hors DAW : presets, annuler/rétablir, A/B, sauvegarde de l'état,
// latence en phase linéaire, mono, blocs de tailles variables, bypass sans clic, charge CPU.
#include "PluginProcessor.h"
#include <thread>
#include <cstdio>

namespace P = audiolab::params;
static int failures = 0;
#define CHECK(cond, ...) do { if (! (cond)) { ++failures; std::printf ("  ÉCHEC : "); std::printf (__VA_ARGS__); std::printf ("\n"); } } while (0)

static int usedCount (AudioLabEqProcessor& p) { int n = 0; for (int b = 0; b < audiolab::eq::maxBands; ++b) n += p.isUsed (b) ? 1 : 0; return n; }

int main()
{
    juce::ScopedJuceInitialiser_GUI init;
    {
        AudioLabEqProcessor p;
        p.setPlayConfigDetails (2, 2, 48000, 512);
        p.prepareToPlay (48000, 512);
        std::printf ("1. Démarrage : %d bande(s) utilisée(s), latence %d\n", usedCount (p), p.getLatencySamples());
        CHECK (usedCount (p) == 0, "le réglage initial doit être neutre");

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
        CHECK (p.undo() && usedCount (p) == 0, "annuler le preset");
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
        p.setValue (p.apvts.getParameter (P::phase), 1.0f);
        juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi;
        buf.clear(); p.processBlock (buf, midi);
        const int lat = p.getLatencySamples();
        std::printf ("6. Phase linéaire : latence %d échantillons (%.1f ms)\n", lat, 1000.0 * lat / 48000);
        CHECK (lat == 4096, "latence attendue 4096");
        // courbe neutre : la sortie doit être l'entrée retardée de la latence
        for (int k = 0; k < 40; ++k) { buf.clear(); p.processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::milliseconds (5)); }
        std::vector<float> outv;
        for (int blk = 0; blk < 20; ++blk)
        {
            buf.clear();
            if (blk == 0) { buf.setSample (0, 0, 1.0f); buf.setSample (1, 0, 1.0f); }
            p.processBlock (buf, midi);
            for (int i = 0; i < 512; ++i) outv.push_back (buf.getSample (0, i));
        }
        int pk = 0; for (int i = 0; i < (int) outv.size(); ++i) if (std::abs (outv[(size_t) i]) > std::abs (outv[(size_t) pk])) pk = i;
        CHECK (pk == lat && std::abs (outv[(size_t) pk] - 1.0f) < 0.01f, "impulse retardée de la latence (pic %d, %.3f)", pk, outv[(size_t) pk]);
        p.setValue (p.apvts.getParameter (P::phase), 0.0f);
        buf.clear(); p.processBlock (buf, midi);
        CHECK (p.getLatencySamples() == 0, "retour en zéro latence");
    }

    // mono + blocs de tailles variables + toutes les bandes
    {
        AudioLabEqProcessor p;
        p.setPlayConfigDetails (1, 1, 44100, 256);
        p.prepareToPlay (44100, 256);
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
        for (int mode = 0; mode < 2; ++mode)
        {
            p.setValue (p.apvts.getParameter (P::phase), (float) mode);
            for (int k = 0; k < 20; ++k) { buf.clear(); p.processBlock (buf, midi); }
            const int blocks = 940;   // 10 s d'audio
            const auto t0 = juce::Time::getHighResolutionTicks();
            for (int k = 0; k < blocks; ++k)
            {
                for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 512; ++i) buf.setSample (ch, i, rnd.nextFloat() * 0.2f - 0.1f);
                p.processBlock (buf, midi);
            }
            const double sec = juce::Time::highResolutionTicksToSeconds (juce::Time::getHighResolutionTicks() - t0);
            std::printf ("8. Charge %s : %.1f %% d'un cœur (10 s d'audio traités en %.2f s)\n", mode ? "phase linéaire" : "zéro latence", 100.0 * sec / 10.0, sec);
        }
    }

    std::printf ("\n%s (%d échec%s)\n", failures == 0 ? "TOUS LES TESTS PASSENT" : "DES TESTS ÉCHOUENT", failures, failures > 1 ? "s" : "");
    return failures == 0 ? 0 : 1;
}
