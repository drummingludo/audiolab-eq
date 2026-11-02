// Paramètres d'Audio Lab EQ : 24 bandes + réglages généraux, réglages types (tests, rendus).
#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "EqMath.h"

namespace audiolab::params
{
using eq::maxBands;

// ── Identifiants ────────────────────────────────────────────────────────
inline juce::String bandId (int band, const char* field) { return "b" + juce::String (band + 1) + "_" + field; }

constexpr const char* fUsed  = "used";
constexpr const char* fOn    = "on";
constexpr const char* fFreq  = "freq";
constexpr const char* fGain  = "gain";
constexpr const char* fQ     = "q";
constexpr const char* fShape = "shape";
constexpr const char* fSlope = "slope";
constexpr const char* fPlace = "place";
constexpr const char* fDyn   = "dyn";
constexpr const char* fThr   = "thr";
constexpr const char* fRange = "range";
constexpr const char* fAtt   = "att";
constexpr const char* fRel   = "rel";
constexpr const char* bandFields[] = { fUsed, fOn, fFreq, fGain, fQ, fShape, fSlope, fPlace, fDyn, fThr, fRange, fAtt, fRel };
constexpr int numBandFields = 13;

constexpr const char* outGain   = "out_gain";
constexpr const char* gainScale = "gain_scale";
constexpr const char* phase     = "phase";
constexpr const char* autoGain  = "auto_gain";
constexpr const char* bypass    = "bypass";
constexpr const char* globalIds[] = { outGain, gainScale, phase, autoGain };
// Réglages globaux de la v1.0 (remplacés par Attaque / Retour par bande), relus à l'ouverture d'anciens projets
constexpr const char* legacyAttack  = "dyn_attack";
constexpr const char* legacyRelease = "dyn_release";

inline juce::String U (const char* s) { return juce::String::fromUTF8 (s); }

inline juce::StringArray shapeNames()
{
    return { U ("Bell"), U ("Low shelf"), U ("Low cut"), U ("High shelf"), U ("High cut"),
             U ("Notch"), U ("Band pass"), U ("Tilt") };
}
inline juce::StringArray slopeNames()
{
    juce::StringArray s;
    for (int v : eq::slopeValues) s.add (juce::String (v) + " dB/oct");
    return s;
}
inline juce::StringArray placeNames() { return { U ("Stereo"), U ("Left"), U ("Right"), U ("Mid"), U ("Side") }; }
inline juce::StringArray placeShort() { return { U ("Stereo"), "L", "R", "M", "S" }; }
inline juce::StringArray phaseNames() { return { U ("Zero latency"), U ("Linear phase") }; }

inline int slopeIndex (int dbPerOct)
{
    for (int i = 0; i < eq::numSlopes; ++i) if (eq::slopeValues[i] == dbPerOct) return i;
    return 1;
}

// ── Mise en forme ───────────────────────────────────────────────────────
inline juce::String fmtHz (float v)
{
    if (v >= 10000) return juce::String (v / 1000, 1) + " kHz";
    if (v >= 1000) return juce::String (v / 1000, 2) + " kHz";
    if (v >= 100) return juce::String (juce::roundToInt (v)) + " Hz";
    return juce::String (v, 1) + " Hz";
}
inline juce::String fmtHzShort (float v)
{
    if (v >= 10000) return juce::String (v / 1000, 1) + "k";
    if (v >= 1000) return juce::String (v / 1000, 2) + "k";
    if (v >= 100) return juce::String (juce::roundToInt (v));
    return juce::String (v, 1);
}
inline juce::String fmtDb (float v, int dec = 1)
{
    if (std::abs (v) < 0.05f) v = 0.0f;
    return (v > 0 ? "+" : "") + juce::String (v, dec);
}
inline juce::String fmtQ (float v) { return v < 10 ? juce::String (v, 2) : juce::String (v, 1); }

// ── Plages ──────────────────────────────────────────────────────────────
inline juce::NormalisableRange<float> logRange (float lo, float hi)
{
    return { lo, hi,
             [] (float a, float b, float t) { return a * std::pow (b / a, t); },
             [] (float a, float b, float v) { return std::log (v / a) / std::log (b / a); },
             [] (float a, float b, float v) { return juce::jlimit (a, b, v); } };
}
inline juce::NormalisableRange<float> freqRange() { return logRange (20.0f, 20000.0f); }
inline juce::NormalisableRange<float> attackRange()  { juce::NormalisableRange<float> r (0.5f, 200.0f, 0.1f); r.setSkewForCentre (15.0f); return r; }
inline juce::NormalisableRange<float> releaseRange() { juce::NormalisableRange<float> r (10.0f, 2000.0f, 1.0f); r.setSkewForCentre (200.0f); return r; }
inline juce::String fmtMs (float v) { return v < 10 ? juce::String (v, 1) + " ms" : juce::String (juce::roundToInt (v)) + " ms"; }
inline juce::NormalisableRange<float> qRange()    { return logRange (0.025f, 40.0f); }

// ── Réglage par défaut : cinq bandes à plat (0 dB, le son n'est pas modifié) ──
struct DefaultBand { eq::Shape shape; float freq, q; };
constexpr DefaultBand defaultBands[] = {
    { eq::Shape::lowShelf,  60.0f,    0.71f },
    { eq::Shape::bell,      300.0f,   1.0f },
    { eq::Shape::bell,      1000.0f,  1.0f },
    { eq::Shape::bell,      3000.0f,  1.0f },
    { eq::Shape::highShelf, 10000.0f, 0.71f },
};
constexpr int numDefaultBands = 5;

// ── Création ────────────────────────────────────────────────────────────
inline juce::AudioProcessorValueTreeState::ParameterLayout createLayout()
{
    using namespace juce;
    AudioProcessorValueTreeState::ParameterLayout layout;
    auto pid = [] (const String& id, int version = 1) { return ParameterID { id, version }; };

    for (int b = 0; b < maxBands; ++b)
    {
        const String pre = U ("Band ") + String (b + 1) + " ";
        const bool isDef = b < numDefaultBands;
        const DefaultBand def = isDef ? defaultBands[b] : DefaultBand { eq::Shape::bell, 1000.0f, 1.0f };
        auto grp = std::make_unique<AudioProcessorParameterGroup> ("band" + String (b + 1), pre.trim(), " | ");
        grp->addChild (std::make_unique<AudioParameterBool> (pid (bandId (b, fUsed)), pre + U ("Used"), isDef));
        grp->addChild (std::make_unique<AudioParameterBool> (pid (bandId (b, fOn)), pre + U ("Active"), true));
        grp->addChild (std::make_unique<AudioParameterFloat> (pid (bandId (b, fFreq)), pre + U ("Frequency"), freqRange(), def.freq,
            AudioParameterFloatAttributes().withLabel ("Hz").withStringFromValueFunction ([] (float v, int) { return fmtHz (v); })));
        grp->addChild (std::make_unique<AudioParameterFloat> (pid (bandId (b, fGain)), pre + "Gain", NormalisableRange<float> (-30.0f, 30.0f, 0.01f), 0.0f,
            AudioParameterFloatAttributes().withLabel ("dB").withStringFromValueFunction ([] (float v, int) { return fmtDb (v, 2); })));
        grp->addChild (std::make_unique<AudioParameterFloat> (pid (bandId (b, fQ)), pre + "Q", qRange(), def.q,
            AudioParameterFloatAttributes().withStringFromValueFunction ([] (float v, int) { return fmtQ (v); })));
        grp->addChild (std::make_unique<AudioParameterChoice> (pid (bandId (b, fShape)), pre + U ("Shape"), shapeNames(), (int) def.shape));
        grp->addChild (std::make_unique<AudioParameterChoice> (pid (bandId (b, fSlope)), pre + U ("Slope"), slopeNames(), 1));
        grp->addChild (std::make_unique<AudioParameterChoice> (pid (bandId (b, fPlace)), pre + U ("Placement"), placeNames(), 0));
        grp->addChild (std::make_unique<AudioParameterBool> (pid (bandId (b, fDyn)), pre + U ("Dynamic"), false));
        grp->addChild (std::make_unique<AudioParameterFloat> (pid (bandId (b, fThr)), pre + U ("Threshold"), NormalisableRange<float> (-80.0f, 0.0f, 0.1f), -30.0f,
            AudioParameterFloatAttributes().withLabel ("dB").withStringFromValueFunction ([] (float v, int) { return juce::String (v, 1); })));
        grp->addChild (std::make_unique<AudioParameterFloat> (pid (bandId (b, fRange)), pre + U ("Dynamic range"), NormalisableRange<float> (-30.0f, 30.0f, 0.01f), -6.0f,
            AudioParameterFloatAttributes().withLabel ("dB").withStringFromValueFunction ([] (float v, int) { return fmtDb (v, 1); })));
        grp->addChild (std::make_unique<AudioParameterFloat> (pid (bandId (b, fAtt), 2), pre + U ("Attack"), attackRange(), 10.0f,
            AudioParameterFloatAttributes().withLabel ("ms").withStringFromValueFunction ([] (float v, int) { return fmtMs (v); })));
        grp->addChild (std::make_unique<AudioParameterFloat> (pid (bandId (b, fRel), 2), pre + U ("Release"), releaseRange(), 150.0f,
            AudioParameterFloatAttributes().withLabel ("ms").withStringFromValueFunction ([] (float v, int) { return fmtMs (v); })));
        layout.add (std::move (grp));
    }

    layout.add (std::make_unique<AudioParameterFloat> (pid (outGain), U ("Output"), NormalisableRange<float> (-36.0f, 36.0f, 0.01f), 0.0f,
        AudioParameterFloatAttributes().withLabel ("dB").withStringFromValueFunction ([] (float v, int) { return fmtDb (v, 1); })));
    layout.add (std::make_unique<AudioParameterFloat> (pid (gainScale), U ("Gain scale"), NormalisableRange<float> (0.0f, 200.0f, 0.1f), 100.0f,
        AudioParameterFloatAttributes().withLabel ("%").withStringFromValueFunction ([] (float v, int) { return String (juce::roundToInt (v)); })));
    layout.add (std::make_unique<AudioParameterChoice> (pid (phase), U ("Phase mode"), phaseNames(), 0));
    layout.add (std::make_unique<AudioParameterBool> (pid (autoGain), U ("Auto gain"), false));
    layout.add (std::make_unique<AudioParameterBool> (pid (bypass), "Bypass", false));
    return layout;
}

// ── Presets d'usine ─────────────────────────────────────────────────────
struct PresetBand
{
    eq::Shape shape; float freq, gain, q; int slope; eq::Place place = eq::Place::stereo;
    bool dyn = false; float thr = -30, range = -6;
};
struct Preset
{
    const char* name;
    const char* hint;
    std::vector<PresetBand> bands;
};

inline const std::vector<Preset>& presets()
{
    using S = eq::Shape; using P = eq::Place;
    static const std::vector<Preset> list = {
        { "Default (neutral)", "Five flat bands (60 Hz, 300 Hz, 1 kHz, 3 kHz, 10 kHz): the sound passes through unchanged.",
          { { S::lowShelf, 60, 0, 0.71f, 12 }, { S::bell, 300, 0, 1.0f, 12 }, { S::bell, 1000, 0, 1.0f, 12 },
            { S::bell, 3000, 0, 1.0f, 12 }, { S::highShelf, 10000, 0, 0.71f, 12 } } },
        { "Low cut 80 Hz", "Removes rumble and handling noise below 80 Hz.",
          { { S::lowCut, 80, 0, 0.71f, 24 } } },
        { "Vocal: clarity", "Low cut, less \"boxiness\" around 250 Hz, presence and air.",
          { { S::lowCut, 90, 0, 0.71f, 18 }, { S::bell, 250, -2.5f, 1.2f, 12 }, { S::bell, 3500, 2.0f, 0.9f, 12 },
            { S::highShelf, 10000, 2.5f, 0.71f, 12 } } },
        { "Vocal: de-esser (dynamic)", "The 6.5 kHz bell only dips when the sibilants go above the threshold.",
          { { S::lowCut, 80, 0, 0.71f, 18 }, { S::bell, 6500, 0, 3.0f, 12, P::stereo, true, -32, -8 } } },
        { "Kick: punch", "Cuts the useless sub, adds impact, scoops the \"boxiness\", adds the click.",
          { { S::lowCut, 30, 0, 0.71f, 24 }, { S::bell, 60, 3, 1.4f, 12 }, { S::bell, 350, -4, 1.5f, 12 }, { S::bell, 4000, 3, 1.2f, 12 } } },
        { "Bass: definition", "Clearer fundamental, lighter low mids, string attack.",
          { { S::lowCut, 35, 0, 0.71f, 24 }, { S::bell, 100, 2, 1.0f, 12 }, { S::bell, 250, -2.5f, 1.4f, 12 },
            { S::bell, 1200, 2.5f, 1.5f, 12 }, { S::highCut, 8000, 0, 0.71f, 12 } } },
        { "Acoustic guitar", "Less body resonance, more brilliance.",
          { { S::lowCut, 100, 0, 0.71f, 18 }, { S::bell, 200, -3, 1.2f, 12 }, { S::bell, 5000, 2, 1.0f, 12 },
            { S::highShelf, 12000, 2, 0.71f, 12 } } },
        { "Drum bus", "Weight in the lows, less mud, more open cymbals.",
          { { S::bell, 80, 2, 0.9f, 12 }, { S::bell, 400, -2, 1.1f, 12 }, { S::highShelf, 8000, 2.5f, 0.71f, 12 } } },
        { "Master: air and weight", "Gentle mastering touches: a little low end and air.",
          { { S::lowCut, 20, 0, 0.71f, 12 }, { S::lowShelf, 80, 1, 0.71f, 12 }, { S::highShelf, 12000, 1.5f, 0.71f, 12 } } },
        { "Mid/Side: wider highs", "Highs boosted on the sides only; low end of the sides cut (mono bass).",
          { { S::lowCut, 120, 0, 0.71f, 12, P::side }, { S::highShelf, 8000, 3, 0.71f, 12, P::side } } },
        { "Anti-resonance (dynamic)", "Two bells that only dip when the resonances stick out.",
          { { S::bell, 250, 0, 2.0f, 12, P::stereo, true, -24, -6 }, { S::bell, 2500, 0, 3.0f, 12, P::stereo, true, -30, -5 } } },
        { "Phone / radio", "Narrow bandwidth, like a telephone.",
          { { S::lowCut, 400, 0, 0.71f, 24 }, { S::highCut, 3400, 0, 0.71f, 24 }, { S::bell, 1500, 4, 1.0f, 12 } } },
        { "Tilt: brighter", "Tilts the whole spectrum around 1 kHz: more highs, less lows.",
          { { S::tiltShelf, 1000, 4, 0.71f, 6 } } },
    };
    return list;
}
} // namespace audiolab::params
