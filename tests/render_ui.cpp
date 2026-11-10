// Test hors DAW : fait tourner le plugin sur un signal musical et enregistre des captures
// de l'interface (presets, thèmes, sélection, dynamique, dessin, phase linéaire) en PNG.
#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <thread>

static void fillMusic (juce::AudioBuffer<float>& b, double fs, int64_t& pos, juce::Random& rnd)
{
    static float pinkL = 0, pinkB1 = 0, pinkB2 = 0;
    for (int i = 0; i < b.getNumSamples(); ++i, ++pos)
    {
        const double t = pos / fs, beat = 0.5, tb = std::fmod (t, beat);
        const int k = (int) (t / beat);
        double s = std::sin (2 * juce::MathConstants<double>::pi * (50 + 120 * std::exp (-tb * 30)) * tb) * std::exp (-tb * 9) * 0.6;
        const float w = rnd.nextFloat() * 2 - 1;
        pinkB1 = 0.99f * pinkB1 + 0.1f * w; pinkB2 = 0.9f * pinkB2 + 0.3f * w; pinkL = pinkB1 + pinkB2 + 0.2f * w;
        if (k % 2) s += pinkL * std::exp (-tb * 14) * 0.25;
        s += 0.02 * pinkL;
        for (int h = 1; h < 8; ++h) s += 0.08 / h * std::sin (2 * juce::MathConstants<double>::pi * 220 * h * t + 3 * std::sin (2 * juce::MathConstants<double>::pi * 5 * t));
        b.setSample (0, i, (float) s); b.setSample (1, i, (float) (s * 0.9 + 0.03 * w));
    }
}

int main (int argc, char** argv)
{
    juce::ScopedJuceInitialiser_GUI init;
    const juce::File out (argc > 1 ? argv[1] : "/tmp");
    auto proc = std::make_unique<AudioLabEqProcessor>();
    proc->setPlayConfigDetails (2, 2, 48000, 512);
    proc->prepareToPlay (48000, 512);

    std::atomic<bool> run { true }, pureTones { false };
    std::thread audio ([&] {
        juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi; int64_t pos = 0; juce::Random rnd (7);
        while (run) { fillMusic (buf, 48000, pos, rnd);
            if (pureTones) for (int i = 0; i < 512; ++i)
            {   // trois sinus purs : mesure de la précision de la saisie d'une crête
                const double t = (double) (pos - 512 + i) / 48000.0, tp = 2 * juce::MathConstants<double>::pi;
                const float s = (float) (0.2 * std::sin (tp * 110.0 * t) + 0.1 * std::sin (tp * 1000.0 * t) + 0.05 * std::sin (tp * 6300.0 * t));
                buf.setSample (0, i, s); buf.setSample (1, i, s);
            }
            proc->processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::microseconds (10600)); }
    });

    std::unique_ptr<AudioLabEqEditor> ed;
    std::vector<std::pair<int, std::function<void()>>> steps;
    auto shot = [&] (const juce::String& name)
    {
        auto img = ed->createComponentSnapshot (ed->getLocalBounds(), true, 1.0f);
        juce::PNGImageFormat png;
        auto f = out.getChildFile (name + ".png");
        f.deleteFile();
        juce::FileOutputStream os (f);
        png.writeImageToStream (img, os);
        std::printf ("%s %dx%d\n", f.getFullPathName().toRawUTF8(), img.getWidth(), img.getHeight());
    };
    auto rebuild = [&]
    {
        if (ed != nullptr) { proc->editorBeingDeleted (ed.get()); ed.reset(); }
        ed.reset (dynamic_cast<AudioLabEqEditor*> (proc->createEditorAndMakeActive()));
        ed->setVisible (true);
    };
    auto theme = [&] (const char* t) { proc->ui.setProperty ("themeOverride", t, nullptr); };
    auto preset = [&] (const char* name)
    {
        const auto& l = audiolab::params::presets();
        for (int i = 0; i < (int) l.size(); ++i) if (juce::String::fromUTF8 (l[(size_t) i].name).startsWith (juce::String::fromUTF8 (name))) proc->applyPreset (i);
    };

    auto& P_phase = *proc->apvts.getParameter (audiolab::params::phase);
    steps.push_back ({ 0,    [&] { theme ("light"); rebuild(); } });   // réglage par défaut d'une nouvelle instance
    steps.push_back ({ 1500, [&] { shot ("00_clair_defaut"); preset ("Vocal: clarity"); ed->getDisplay().select (2); } });
    steps.push_back ({ 1800, [&] { shot ("01_clair_bande3"); theme ("dark"); preset ("Vocal: de-esser"); rebuild(); ed->getDisplay().select (1); } });
    steps.push_back ({ 1800, [&] { shot ("02_sombre_dynamique"); theme ("light"); proc->clearAllBands(); rebuild(); } });
    auto click = [&] (juce::Point<float> pos)
    {
        auto& d = ed->getDisplay();
        auto src = juce::Desktop::getInstance().getMainMouseSource();
        const auto now = juce::Time::getCurrentTime();
        juce::MouseEvent down (src, pos, {}, juce::MouseInputSource::defaultPressure, 0, 0, 0, 0, &d, &d, now, pos, now, 1, false);
        juce::MouseEvent up (src, pos, {}, juce::MouseInputSource::defaultPressure, 0, 0, 0, 0, &d, &d, now, pos, now, 1, false);
        d.mouseDown (down); d.mouseUp (up);
    };
    steps.push_back ({ 1200, [&] { shot ("03_clair_vide_aide");
        // gestes : croix de l'aide, puis clic dans la courbe → nouvelle bande + panneau
        auto& d = ed->getDisplay();
        const auto r = d.plot();
        click ({ r.getRight() - 16 - 16 - 11, r.getY() + 32 + 12 + 11 });
        std::printf ("aide masquée : %s\n", (bool) proc->ui.getProperty ("helpHidden", false) ? "oui" : "NON");
        click ({ r.getX() + r.getWidth() * 0.5f, r.getY() + 120 });
        int used = 0; for (int b = 0; b < audiolab::eq::maxBands; ++b) used += proc->isUsed (b) ? 1 : 0;
        std::printf ("clic dans la courbe : %d bande(s), sélection %d\n", used, d.getSelection().size());
        proc->ui.setProperty ("helpHidden", false, nullptr);
        proc->clearAllBands(); d.clearSelection();
        ed->getDisplay().testSketch ([] (double f) {
            double v = 5.0 * std::exp (-std::pow (std::log2 (f / 3000.0) / 0.6, 2));
            v += -4.0 / (1 + std::pow (10000.0 / f, 3));
            v += f < 60 ? -12.0 * std::log2 (60.0 / f) : 0.0;
            return (float) std::max (-12.0, v);
        }); } });
    steps.push_back ({ 1500, [&] { shot ("04_clair_dessin_ajuste"); ed->getDisplay().selectAll(); } });
    steps.push_back ({ 800,  [&] { shot ("05_clair_multi_selection"); preset ("Kick"); ed->getDisplay().clearSelection(); ed->getDisplay().select (2);
                                   proc->setBandValue (2, audiolab::params::fGain, -10.0f); } });
    steps.push_back ({ 1500, [&] { shot ("06_clair_poignee_sur_panneau"); P_phase.setValueNotifyingHost (P_phase.convertTo0to1 ((float) audiolab::params::phaseLinear)); preset ("Mid/Side"); theme ("dark");
                                   rebuild(); ed->getDisplay().select (1); ed->getDisplay().testHover (1); } });
    steps.push_back ({ 1800, [&] { shot ("07_sombre_mid_side_lineaire"); P_phase.setValueNotifyingHost (0.0f); proc->clearAllBands();
                                   proc->ui.setProperty ("helpHidden", true, nullptr); rebuild();
                                   ed->setSize (AudioLabEqEditor::baseW * 7 / 10, AudioLabEqEditor::baseH * 7 / 10); } });
    auto& P_res = *proc->apvts.getParameter (audiolab::params::linRes);
    // 2.0 : Natural phase, panneau avancé, side-chain par bande
    steps.push_back ({ 1200, [&] { shot ("08_sombre_vide_petite_taille");
        preset ("Vocal: de-esser"); proc->setBandValue (1, audiolab::params::fSc, 1.0f);
        P_phase.setValueNotifyingHost (P_phase.convertTo0to1 ((float) audiolab::params::phaseNatural));
        proc->ui.setProperty ("adv", true, nullptr); proc->ui.setProperty ("scale", 1.0, nullptr); rebuild(); ed->getDisplay().select (1); } });
    steps.push_back ({ 2000, [&] { shot ("09_naturel_avance_sidechain");
        // phase linéaire en résolution Low : la courbe affichée est celle du FIR réel (notch 60 Hz, cloche 50 Hz)
        proc->clearAllBands();
        proc->createBand (audiolab::eq::Shape::notch, 60.0f, 0.0f, 10.0f, 12);
        proc->createBand (audiolab::eq::Shape::bell, 150.0f, -9.0f, 6.0f, 12);
        proc->createBand (audiolab::eq::Shape::highShelf, 9000.0f, 4.0f, 0.71f, 12);
        P_res.setValueNotifyingHost (P_res.convertTo0to1 (1.0f));
        P_phase.setValueNotifyingHost (P_phase.convertTo0to1 ((float) audiolab::params::phaseLinear));
        ed->getDisplay().clearSelection(); } });
    steps.push_back ({ 2500, [&] { shot ("10_lineaire_low_courbe_reelle");
        P_res.setValueNotifyingHost (P_res.convertTo0to1 (0.0f)); } });
    steps.push_back ({ 2500, [&] { shot ("11_lineaire_auto_courbe_reelle");
        ed->setSize (AudioLabEqEditor::baseW * 6 / 10, AudioLabEqEditor::baseHOpen * 6 / 10); } });
    steps.push_back ({ 1200, [&] { shot ("12_avance_60pc"); ed->setSize (AudioLabEqEditor::baseW * 2, AudioLabEqEditor::baseHOpen * 2); } });
    steps.push_back ({ 1200, [&] { shot ("13_avance_200pc"); preset ("Vocal: de-esser"); proc->ui.setProperty ("adv", false, nullptr);
        P_phase.setValueNotifyingHost (0.0f); rebuild(); ed->setSize (AudioLabEqEditor::baseW * 2, AudioLabEqEditor::baseH * 2); ed->getDisplay().select (1); } });
    steps.push_back ({ 1500, [&] { shot ("14_principal_200pc_dynamique"); ed->setSize (AudioLabEqEditor::baseW * 6 / 10, AudioLabEqEditor::baseH * 6 / 10); } });
    // 3.0 : saisie d'une valeur au clavier, puis crête du spectre saisie d'un clic
    steps.push_back ({ 1200, [&] { shot ("15_principal_60pc_dynamique"); ed->setSize (AudioLabEqEditor::baseW, AudioLabEqEditor::baseH); ed->getPanel().testEditFrequency(); } });
    steps.push_back ({ 600,  [&] { shot ("16_saisie_clavier"); proc->clearAllBands(); proc->ui.setProperty ("helpHidden", true, nullptr); rebuild(); } });
    steps.push_back ({ 2500, [&] { const double f = ed->getDisplay().testGrab(); std::printf ("crête du spectre : %.1f Hz\n", f); } });
    steps.push_back ({ 300,  [&] { shot ("17_crete_du_spectre");
        auto& d = ed->getDisplay(); const auto pos = d.testGrabPos();
        auto src = juce::Desktop::getInstance().getMainMouseSource(); const auto now = juce::Time::getCurrentTime();
        juce::MouseEvent ev (src, pos, {}, juce::MouseInputSource::defaultPressure, 0, 0, 0, 0, &d, &d, now, pos, now, 1, false);
        d.mouseMove (ev); d.mouseDown (ev); d.mouseUp (ev);
        for (int b = 0; b < audiolab::eq::maxBands; ++b) if (proc->isUsed (b))
            std::printf ("bande créée sur la crête : %.1f Hz, Q %.1f, %.1f dB\n", (double) proc->getBandSpec (b, false).freq, (double) proc->getBandSpec (b, false).q, (double) proc->getBandSpec (b, false).gainDb); } });
    steps.push_back ({ 800,  [&] { shot ("18_bande_sur_la_crete"); proc->clearAllBands(); pureTones = true; } });
    steps.push_back ({ 3000, [&] {
        auto& d = ed->getDisplay(); const auto r = d.plot();
        for (double f : { 110.0, 1000.0, 6300.0 })
        {
            const float x = r.getX() + r.getWidth() * (float) (std::log (f / 20.0) / std::log (1000.0));
            double best = 0; for (float y = r.getY(); y < r.getBottom(); y += 8) { const double g = d.testGrabNear ({ x + 6, y }); if (g > 0) best = g; }
            std::printf ("sinus %.0f Hz : crête saisie à %.2f Hz (écart %+.2f %%)\n", f, best, best > 0 ? 100.0 * (best / f - 1.0) : 0.0);
        }
        juce::MessageManager::getInstance()->stopDispatchLoop(); } });

    std::function<void (size_t)> next = [&] (size_t i)
    {
        if (i >= steps.size()) return;
        juce::Timer::callAfterDelay (std::max (1, steps[i].first), [&, i] { steps[i].second(); next (i + 1); });
    };
    next (0);
    juce::MessageManager::getInstance()->runDispatchLoop();

    run = false; audio.join();
    proc->editorBeingDeleted (ed.get()); ed.reset();
    proc->releaseResources();
    return 0;
}
