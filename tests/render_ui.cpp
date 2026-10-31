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

    std::atomic<bool> run { true };
    std::thread audio ([&] {
        juce::AudioBuffer<float> buf (2, 512); juce::MidiBuffer midi; int64_t pos = 0; juce::Random rnd (7);
        while (run) { fillMusic (buf, 48000, pos, rnd); proc->processBlock (buf, midi); std::this_thread::sleep_for (std::chrono::microseconds (10600)); }
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
    steps.push_back ({ 0,    [&] { theme ("light"); preset ("Voix — clarté"); rebuild(); ed->getDisplay().select (2); } });
    steps.push_back ({ 1800, [&] { shot ("01_clair_bande3"); theme ("dark"); preset ("Voix — de-esser"); rebuild(); ed->getDisplay().select (1); } });
    steps.push_back ({ 1800, [&] { shot ("02_sombre_dynamique"); theme ("light"); preset ("Défaut"); rebuild(); } });
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
        click ({ r.getRight() - 16 - 16 - 11, r.getY() + 16 + 12 + 11 });
        std::printf ("aide masquée : %s\n", (bool) proc->ui.getProperty ("helpHidden", false) ? "oui" : "NON");
        click ({ r.getX() + r.getWidth() * 0.5f, r.getY() + 120 });
        int used = 0; for (int b = 0; b < audiolab::eq::maxBands; ++b) used += proc->isUsed (b) ? 1 : 0;
        std::printf ("clic dans la courbe : %d bande(s), sélection %d\n", used, d.getSelection().size());
        proc->ui.setProperty ("helpHidden", false, nullptr);
        preset ("Défaut"); d.clearSelection();
        ed->getDisplay().testSketch ([] (double f) {
            double v = 5.0 * std::exp (-std::pow (std::log2 (f / 3000.0) / 0.6, 2));
            v += -4.0 / (1 + std::pow (10000.0 / f, 3));
            v += f < 60 ? -12.0 * std::log2 (60.0 / f) : 0.0;
            return (float) std::max (-12.0, v);
        }); } });
    steps.push_back ({ 1500, [&] { shot ("04_clair_dessin_ajuste"); ed->getDisplay().selectAll(); } });
    steps.push_back ({ 800,  [&] { shot ("05_clair_multi_selection"); preset ("Kick"); ed->getDisplay().clearSelection(); ed->getDisplay().select (2); } });
    steps.push_back ({ 1500, [&] { shot ("06_clair_panneau_en_haut"); P_phase.setValueNotifyingHost (1.0f); preset ("Mid/Side"); theme ("dark");
                                   rebuild(); ed->getDisplay().select (1); ed->getDisplay().testHover (1); } });
    steps.push_back ({ 1800, [&] { shot ("07_sombre_mid_side_lineaire"); P_phase.setValueNotifyingHost (0.0f); preset ("Défaut");
                                   proc->ui.setProperty ("helpHidden", true, nullptr); rebuild();
                                   ed->setSize (AudioLabEqEditor::baseW * 7 / 10, AudioLabEqEditor::baseH * 7 / 10); } });
    steps.push_back ({ 1200, [&] { shot ("08_sombre_vide_petite_taille"); juce::MessageManager::getInstance()->stopDispatchLoop(); } });

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
