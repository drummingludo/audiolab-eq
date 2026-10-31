// Audio Lab EQ — mode phase linéaire.
// La courbe complète (toutes les bandes, avec leur placement Stéréo / G / D / Mid / Side) est
// convertie en filtres FIR symétriques : aucune rotation de phase, latence = moitié du FIR.
// Avec des bandes Mid/Side, la sortie est une matrice 2×2 de FIR :
//   L' = A·L + B·R   ;   R' = C·L + D·R
// Les FIR sont calculés sur un fil d'exécution séparé puis transmis au fil audio sans verrou.
#pragma once
#include <juce_dsp/juce_dsp.h>
#include "EqMath.h"
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

    LinearPhase()
        : queue (16),
          conv { juce::dsp::Convolution (juce::dsp::Convolution::Latency { 0 }, queue),
                 juce::dsp::Convolution (juce::dsp::Convolution::Latency { 0 }, queue),
                 juce::dsp::Convolution (juce::dsp::Convolution::Latency { 0 }, queue),
                 juce::dsp::Convolution (juce::dsp::Convolution::Latency { 0 }, queue) }
    {
    }

    ~LinearPhase()
    {
        delete pending.exchange (nullptr);
        delete trash.exchange (nullptr);
    }

    static int firLengthFor (double fs) { return fs <= 50000 ? 8192 : fs <= 100000 ? 16384 : 32768; }

    // Fil audio arrêté (prepareToPlay)
    void prepare (double sampleRate, int maxBlock)
    {
        const std::lock_guard<std::mutex> lock (designMutex);
        fs = sampleRate;
        N = firLengthFor (fs);
        order = 0; while ((1 << order) < N) ++order;
        fft = std::make_unique<juce::dsp::FFT> (order);
        // facteur d'échelle de la FFT inverse (dépend de l'implémentation : vDSP, IPP, interne…)
        {
            std::vector<float> d ((size_t) N * 2, 0.0f);
            for (int k = 0; k <= N / 2; ++k) d[(size_t) (2 * k)] = 1.0f;
            fft->performRealOnlyInverseTransform (d.data());
            invScale = std::abs (d[0]) > 1e-12f ? 1.0f / d[0] : 1.0f;
        }
        window.resize ((size_t) N);
        for (int n = 0; n < N; ++n)
        {
            const double x = 2.0 * pi * n / N;   // Blackman-Harris, centré sur N/2
            window[(size_t) n] = (float) (0.35875 - 0.48829 * std::cos (x) + 0.14128 * std::cos (2 * x) - 0.01168 * std::cos (3 * x));
        }
        bufA.setSize (1, std::max (512, maxBlock));
        bufB.setSize (1, std::max (512, maxBlock));
        juce::dsp::ProcessSpec spec { fs, (juce::uint32) std::max (512, maxBlock), 1 };
        // FIR neutres (retard pur) tant que la première conception n'est pas arrivée
        for (int i = 0; i < 4; ++i)
        {
            juce::AudioBuffer<float> ir (1, N);
            ir.clear();
            if (i == 0 || i == 3) ir.setSample (0, N / 2, 1.0f);
            conv[(size_t) i].loadImpulseResponse (std::move (ir), fs, juce::dsp::Convolution::Stereo::no,
                                                  juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
            conv[(size_t) i].prepare (spec);
        }
        delete pending.exchange (nullptr);
        prepared.store (true);
    }

    int latency() const { return N / 2; }
    double sampleRate() const { return fs; }

    // Fil de conception : calcule les 4 FIR et les dépose pour le fil audio.
    void design (const Snapshot& snap)
    {
        const std::lock_guard<std::mutex> lock (designMutex);
        if (! prepared.load()) return;
        const int half = N / 2;
        std::vector<double> HL ((size_t) half + 1, 1.0), HR = HL, HM = HL, HS = HL;
        bool anyMS = false;
        Design d;
        for (const auto& b : snap.bands)
        {
            if (! b.active) continue;
            design_ (b, d);
            std::vector<double>* targets[2] = { nullptr, nullptr };
            switch (b.place)
            {
                case Place::stereo: targets[0] = &HL; targets[1] = &HR; break;
                case Place::left:   targets[0] = &HL; break;
                case Place::right:  if (snap.stereo) targets[0] = &HR; break;
                case Place::mid:    if (snap.stereo) { targets[0] = &HM; anyMS = true; } else targets[0] = &HL; break;
                case Place::side:   if (snap.stereo) { targets[0] = &HS; anyMS = true; } break;
            }
            if (targets[0] == nullptr) continue;
            for (int k = 0; k <= half; ++k)
            {
                const double f = std::max (1.0, k * fs / N);
                const double m = std::abs (response (d, f, fs, true));
                (*targets[0])[(size_t) k] *= m;
                if (targets[1] != nullptr) (*targets[1])[(size_t) k] *= m;
            }
        }

        auto* set = new FirSet();
        std::vector<double> H ((size_t) half + 1);
        auto build = [&] (int slot, auto&& fn)
        {
            for (int k = 0; k <= half; ++k) H[(size_t) k] = fn (k);
            set->ir[slot].setSize (1, N);
            makeFir (H, set->ir[slot].getWritePointer (0));
        };
        if (anyMS)
        {
            build (0, [&] (int k) { return 0.5 * (HM[(size_t) k] + HS[(size_t) k]) * HL[(size_t) k]; });
            build (1, [&] (int k) { return 0.5 * (HM[(size_t) k] - HS[(size_t) k]) * HR[(size_t) k]; });
            build (2, [&] (int k) { return 0.5 * (HM[(size_t) k] - HS[(size_t) k]) * HL[(size_t) k]; });
            build (3, [&] (int k) { return 0.5 * (HM[(size_t) k] + HS[(size_t) k]) * HR[(size_t) k]; });
        }
        else
        {
            build (0, [&] (int k) { return HL[(size_t) k]; });
            build (1, [&] (int)   { return 0.0; });
            build (2, [&] (int)   { return 0.0; });
            build (3, [&] (int k) { return HR[(size_t) k]; });
        }
        delete pending.exchange (set);
    }

    // Fil de conception : libère les FIR déjà transmis
    void collectGarbage() { delete trash.exchange (nullptr); }

    // Fil audio
    void process (float* L, float* R, int n)
    {
        if (trash.load() == nullptr)
            if (auto* set = pending.exchange (nullptr))
            {
                for (int i = 0; i < 4; ++i)
                    conv[(size_t) i].loadImpulseResponse (std::move (set->ir[i]), fs, juce::dsp::Convolution::Stereo::no,
                                                          juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::no);
                trash.store (set);
            }

        const int cap = bufA.getNumSamples();
        for (int off = 0; off < n; off += cap)
        {
            const int len = std::min (cap, n - off);
            float* l = L + off;
            float* r = R != nullptr ? R + off : nullptr;
            if (r == nullptr)
            {
                run (0, l, l, len);
                continue;
            }
            // A·L → bufA ; B·R → bufB ; L' = bufA + bufB
            run (0, l, bufA.getWritePointer (0), len);
            run (1, r, bufB.getWritePointer (0), len);
            // C·L → l ; D·R → r ; R' = l + r
            run (2, l, l, len);
            run (3, r, r, len);
            auto* a = bufA.getReadPointer (0); auto* b = bufB.getReadPointer (0);
            for (int i = 0; i < len; ++i)
            {
                const float right = l[i] + r[i];
                l[i] = a[i] + b[i];
                r[i] = right;
            }
        }
    }

    void reset() { for (auto& c : conv) c.reset(); }

private:
    struct FirSet { juce::AudioBuffer<float> ir[4]; };

    void design_ (const BandSpec& b, Design& d) const { eq::design (b, fs, d, true); }

    void run (int i, const float* in, float* out, int len)
    {
        juce::dsp::AudioBlock<const float> ib (&in, 1, (size_t) len);
        juce::dsp::AudioBlock<float> ob (&out, 1, (size_t) len);
        juce::dsp::ProcessContextNonReplacing<float> ctx (ib, ob);
        conv[(size_t) i].process (ctx);
    }

    // Réponse réelle (phase nulle) → FIR symétrique centré sur N/2, fenêtré.
    void makeFir (const std::vector<double>& H, float* out)
    {
        std::vector<float> d ((size_t) N * 2, 0.0f);
        for (int k = 0; k <= N / 2; ++k) d[(size_t) (2 * k)] = (float) H[(size_t) k];
        fft->performRealOnlyInverseTransform (d.data());
        for (int n = 0; n < N; ++n)
        {
            const int src = (n - N / 2 + N) % N;
            out[n] = d[(size_t) src] * invScale * window[(size_t) n];
        }
    }

    double fs = 48000.0;
    int N = 8192, order = 13;
    float invScale = 1.0f;
    std::unique_ptr<juce::dsp::FFT> fft;
    std::vector<float> window;
    juce::dsp::ConvolutionMessageQueue queue;
    std::array<juce::dsp::Convolution, 4> conv;
    juce::AudioBuffer<float> bufA, bufB;
    std::atomic<FirSet*> pending { nullptr }, trash { nullptr };
    std::atomic<bool> prepared { false };
    std::mutex designMutex;
};
} // namespace audiolab::eq
