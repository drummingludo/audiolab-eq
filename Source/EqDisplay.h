// Audio Lab EQ — affichage principal : grille, analyseur de spectre (avant / après),
// courbe d'égalisation, poignées des bandes, clavier de piano, mode « Dessiner ».
#pragma once
#include <juce_dsp/juce_dsp.h>
#include "PluginProcessor.h"
#include "Theme.h"
#include "SketchFit.h"

namespace audiolab
{
class EqDisplay : public juce::Component, private juce::Timer
{
public:
    EqDisplay (Theme&, AudioLabEqProcessor&);
    ~EqDisplay() override;

    void paint (juce::Graphics&) override;
    void resized() override;

    void mouseMove (const juce::MouseEvent&) override;
    void mouseExit (const juce::MouseEvent&) override;
    void mouseDown (const juce::MouseEvent&) override;
    void mouseDrag (const juce::MouseEvent&) override;
    void mouseUp (const juce::MouseEvent&) override;
    void mouseDoubleClick (const juce::MouseEvent&) override;
    void mouseWheelMove (const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
    bool keyPressed (const juce::KeyPress&) override;

    // Sélection (partagée avec le panneau de bande)
    const juce::Array<int>& getSelection() const { return selection; }
    void select (int band, bool add = false);
    void clearSelection();
    void deleteSelected();
    void selectAll();
    std::function<void()> onSelectionChanged;
    std::function<void (const juce::String&)> onMessage;
    std::function<void()> onUndoRedo;

    void setSketchMode (bool on);
    bool isSketchMode() const { return sketchMode; }
    std::function<void (bool)> onSketchModeChanged;

    void refresh() { dirty = true; repaint(); }
    std::function<void()> onCurvesChanged;   // poignées déplacées : le panneau flottant suit

    static constexpr double fMin = 20.0, fMax = 20000.0;
    static constexpr float rightGutter = 0.0f, bottomGutter = 24.0f;
    // 3.0 : écran 1072 × 400, 0 dB au centre (y = 200), 16 px par dB à ±12 dB (2 modules de 8 px)
    static constexpr float curveTop = 8.0f, curveBottom = 392.0f;
    // niveaux de sortie, dessinés dans la barre du bas (deux barres horizontales)
    void paintMeters (juce::Graphics&, juce::Rectangle<float> area) const;
    std::function<void()> onMeters;
    void paintNode (juce::Graphics&, int band) const;
    int hitBandAt (juce::Point<float> p) const { return hitBand (p); }

    juce::Rectangle<float> plot() const;
    juce::Point<float> nodePos (int band) const;

    // Outils de test (captures hors DAW)
    void testSketch (const std::function<float (double)>& curve)
    {
        setSketchMode (true);
        for (int i = 0; i < numCols; ++i) sketch[(size_t) i] = curve (colFreq[(size_t) i]);
        fitSketch();
    }
    void testHover (int b) { hover = b; repaint(); }
    // tests : place le curseur sur la crête du spectre la plus haute ; rend sa fréquence (0 = aucune crête)
    double testGrab()
    {
        auto r = plot(); int best = -1; float top = -200.0f;
        for (int i = 8; i < numCols - 8; ++i)
        {
            const float v = anPost[(size_t) i];
            const int c = findPeak ({ r.getX() + i + 0.5f, yAn (v) });
            if (c == i && v > top) { top = v; best = i; }
        }
        grabCol = best; repaint();
        return best < 0 ? 0.0 : peakFreq (best);
    }
    double testGrabNear (juce::Point<float> pos) const { const int c = findPeak (pos); return c < 0 ? 0.0 : peakFreq (c); }
    juce::Point<float> testGrabPos() const { auto r = plot(); return { r.getX() + grabCol + 0.5f, yAn (anPost[(size_t) std::max (0, grabCol)]) }; }

private:
    void timerCallback() override;
    void updateCurves();
    void updateAnalyzer();
    void showBandMenu (int band, juce::Point<float> at);
    void showEmptyMenu (juce::Point<float> at);
    void fitSketch();
    void beginEdit();
    void endEdit();

    juce::Rectangle<float> pianoArea() const;
    juce::Rectangle<float> helpCard() const;
    juce::Rectangle<float> helpClose() const;
    bool helpShown() const;
    void paintEmpty (juce::Graphics&);
    juce::Rectangle<float> rangeButton() const;
    float xOf (double f) const;
    double fOf (float x) const;
    float yOf (double db) const;
    double dbOf (float y) const;
    float yAn (double db) const;
    double range() const;

    int hitBand (juce::Point<float> p) const;
    void paintAnalyzer (juce::Graphics&, const std::vector<float>& col, juce::Colour c, bool fill);
    void paintPiano (juce::Graphics&);
    void paintReadout (juce::Graphics&, int band);

    Theme& theme;
    AudioLabEqProcessor& proc;
    juce::Array<int> selection;

    // courbes (dB par colonne de pixels)
    int numCols = 0;
    std::vector<double> colFreq;
    std::array<std::vector<float>, eq::maxBands> bandCurve;
    std::vector<float> totalCurve, curveB;
    int curveMode = 0;   // 0 : une courbe ; 1 : Gauche/Droite ; 2 : Mid/Side
    std::array<bool, eq::maxBands> bandShown {};
    std::vector<float> lastKey;
    bool dirty = true;

    // analyseur
    static constexpr int fftOrder = 13, fftSize = 1 << fftOrder;
    juce::dsp::FFT fft { fftOrder };
    std::vector<float> fftBuf, window;
    std::vector<float> anPre, anPost;
    // « spectrum grab » : crête du spectre sous le curseur (colonne, −1 = aucune) ; un clic y pose une cloche
    int grabCol = -1, grabDown = -1;
    int findPeak (juce::Point<float> pos) const;
    double peakFreq (int col) const;   // fréquence de la crête, affinée entre colonnes (parabole sur les dB)
    void paintGrab (juce::Graphics&);
    bool analyzerLive = false;
    void analyse (const AudioLabEqProcessor::Tap& tap, std::vector<float>& col);

    float meterL = -90, meterR = -90;

    // interaction
    enum class Drag { none, node, lasso, empty, sketch };
    Drag drag = Drag::none;
    int hover = -1, dragBand = -1;
    juce::Point<float> downPos, lastPos;
    struct Start { int band; double f, g, q; };
    std::vector<Start> starts;
    juce::Rectangle<float> lasso;
    bool soloHeld = false;
    int createdBand = -1;
    juce::uint32 createdAt = 0, lastWheel = 0;
    std::vector<juce::RangedAudioParameter*> gestureParams;

    bool sketchMode = false;
    std::vector<float> sketch;   // dB voulus par colonne (NaN = non dessiné)
    int lastSketchCol = -1;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EqDisplay)
};
} // namespace audiolab
