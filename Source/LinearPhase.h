// Audio Lab EQ — modes FIR « Linear phase » et « Natural phase ».
// Tout le traitement est dans LinearPhaseCore.h (sans JUCE, pour que le banc qualité teste le même
// code que le plugin) ; ici, seulement la FFT de JUCE.
#pragma once
#include <juce_dsp/juce_dsp.h>
#include "LinearPhaseCore.h"

namespace audiolab::eq
{
struct JuceFft
{
    explicit JuceFft (int order) : fft (order) {}
    void forward (float* d) { fft.performRealOnlyForwardTransform (d, true); }
    void inverse (float* d) { fft.performRealOnlyInverseTransform (d); }
    juce::dsp::FFT fft;
};

using LinearPhase = LinearPhaseT<JuceFft>;
} // namespace audiolab::eq
