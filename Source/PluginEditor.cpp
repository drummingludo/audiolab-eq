#include "PluginEditor.h"

using namespace audiolab;
using namespace audiolab::eq;
namespace P = audiolab::params;
using juce::String;

static String U (const char* s) { return String::fromUTF8 (s); }

// ── Construction ────────────────────────────────────────────────────────
AudioLabEqEditor::AudioLabEqEditor (AudioLabEqProcessor& p)
    : AudioProcessorEditor (&p), proc (p),
      display (theme, p),
      panel (theme, p, display),
      autoBtn (theme, PillButton::Style::ibtn, U ("Gain auto"))
{
    setLookAndFeel (&lnf);
    tooltips.setLookAndFeel (&lnf);
    setWantsKeyboardFocus (true);
    addAndMakeVisible (content);
    auto& ap = proc.apvts;
    auto prm = [&] (const char* id) -> juce::RangedAudioParameter& { return *ap.getParameter (id); };

    // Courbe + panneau flottant
    display.onSelectionChanged = [this] { rebuildPanel(); };
    display.onMessage = [this] (const String& s) { toast.show (s); };
    display.onUndoRedo = [this] { rebuildPanel(); toast.show (U ("Annulé / rétabli")); };
    display.onCurvesChanged = [this] { placePanel(); };
    content.addAndMakeVisible (display);
    content.addAndMakeVisible (shadow);
    content.addChildComponent (panel);
    content.addChildComponent (overlay);

    // Barre du bas : phase, analyseur, échelle, sortie, gain auto
    phaseSeg.attach (prm (P::phase));
    {
        auto f = phaseSeg.onSelect;
        phaseSeg.onSelect = [this, f] (int i)
        {
            f (i);
            toast.show (i == 1 ? U ("Phase linéaire : aucune rotation de phase, latence ") + String (juce::roundToInt (1000.0 * LinearPhase::firLengthFor (proc.getSampleRateSafe()) / 2 / proc.getSampleRateSafe())) + " ms"
                               : U ("Zéro latence : filtres analogiques, aucune latence"));
        };
    }
    phaseSeg.setTitle (U ("Mode de phase"));
    phaseSeg.setTooltip (U ("Zéro latence : comme un égaliseur analogique · Linéaire : aucune rotation de phase (ajoute de la latence)"));

    analyzerBox.addItem (U ("Analyseur : avant + après"), 1);
    analyzerBox.addItem (U ("Analyseur : après"), 2);
    analyzerBox.addItem (U ("Analyseur : avant"), 3);
    analyzerBox.addItem (U ("Analyseur : désactivé"), 4);
    analyzerBox.setTitle (U ("Analyseur"));
    {
        const auto m = proc.ui.getProperty ("analyzer", "prepost").toString();
        analyzerBox.setSelectedId (m == "post" ? 2 : m == "pre" ? 3 : m == "off" ? 4 : 1, juce::dontSendNotification);
    }
    analyzerBox.onChange = [this]
    {
        const char* modes[] = { "prepost", "post", "pre", "off" };
        proc.ui.setProperty ("analyzer", modes[juce::jlimit (0, 3, analyzerBox.getSelectedId() - 1)], nullptr);
        display.refresh();
    };

    // petits potentiomètres : échelle (forme de la courbe, teal), sortie (gain, teal secondaire, départ à 12 h)
    vScale = std::make_unique<ArcKnob> (theme, prm (P::gainScale), U ("Échelle"), "%", Fam::teal, ArcKnob::Size::small,
        [] (float v) { return String (juce::roundToInt (v)) + " %"; }, 0.0f,
        U ("Échelle de gain : multiplie le gain de toutes les bandes (100 % = tel quel, 0 % = courbe plate, 200 % = doublé). Double-clic : 100 %"));
    vOut = std::make_unique<ArcKnob> (theme, prm (P::outGain), U ("Sortie"), "dB", Fam::teal2, ArcKnob::Size::small,
        [] (float v) { return P::fmtDb (v) + " dB"; }, 0.0f, U ("Gain de sortie. Double-clic : 0 dB"));
    for (auto* v : { vScale.get(), vOut.get() }) v->onGestureStart = [this] { proc.pushUndo(); };

    autoBtn.setTooltip (U ("Gain auto : compense le volume moyen ajouté ou retiré par l'égalisation, pour comparer à niveau égal"));
    autoBtn.onClick = [this]
    {
        proc.pushUndo();
        proc.setValue (proc.apvts.getParameter (P::autoGain), proc.getParam (P::autoGain) > 0.5f ? 0.0f : 1.0f);
    };

    for (juce::Component* c : { (juce::Component*) &phaseSeg, (juce::Component*) &analyzerBox, (juce::Component*) vScale.get(),
                                (juce::Component*) vOut.get(), (juce::Component*) &autoBtn })
        content.addAndMakeVisible (c);

    // Bypass du logiciel hôte : la courbe s'estompe
    bypassAtt = std::make_unique<juce::ParameterAttachment> (prm (P::bypass), [this] (float v) { display.setAlpha (v > 0.5f ? 0.45f : 1.0f); }, nullptr);
    bypassAtt->sendInitialUpdate();

    toast.atTop = true;
    content.addAndMakeVisible (toast);

    juce::Desktop::getInstance().addDarkModeSettingListener (this);
    applyTheme();

    const double sc = juce::jlimit (0.6, 2.0, (double) proc.ui.getProperty ("scale", 1.0));
    setResizable (true, true);
    getConstrainer()->setFixedAspectRatio ((double) baseW / baseH);
    setResizeLimits (baseW * 6 / 10, baseH * 6 / 10, baseW * 2, baseH * 2);
    setSize (juce::roundToInt (baseW * sc), juce::roundToInt (baseH * sc));
    rebuildPanel();
    startTimerHz (10);
}

AudioLabEqEditor::~AudioLabEqEditor()
{
    stopTimer();
    juce::Desktop::getInstance().removeDarkModeSettingListener (this);
    tooltips.setLookAndFeel (nullptr);
    setLookAndFeel (nullptr);
}

void AudioLabEqEditor::applyTheme()
{
    // charte V2 : un seul thème, sombre
    lnf.refresh();
    sendLookAndFeelChange();
    content.repaint();
    for (auto* c : content.getChildren()) c->repaint();
}

bool AudioLabEqEditor::keyPressed (const juce::KeyPress& k) { return display.keyPressed (k); }

// ── Panneau flottant ────────────────────────────────────────────────────
void AudioLabEqEditor::rebuildPanel()
{
    panel.rebuild();
    placePanel();
}

void AudioLabEqEditor::placePanel()
{
    if (! panel.isVisible() || display.getSelection().isEmpty())
    {
        if (panel.isVisible()) panel.setVisible (false);
        shadow.repaint();
        overlay.update();
        return;
    }
    // Comme Pro-Q : toujours en bas de la courbe, il suit la bande de gauche à droite.
    const auto r = display.plot().translated ((float) display.getX(), (float) display.getY());
    float cx = 0;
    for (int b : display.getSelection()) cx += display.nodePos (b).x;
    cx = cx / (float) display.getSelection().size() + (float) display.getX();
    const int w = panel.getWidth(), h = panel.getHeight();
    const int x = juce::jlimit (juce::roundToInt (r.getX()) + 12, juce::roundToInt (r.getRight()) - 12 - w, juce::roundToInt (cx - w * 0.5f));
    const juce::Rectangle<int> nb (x, juce::roundToInt (r.getBottom()) - 12 - h, w, h);
    if (panel.getBounds() != nb) { panel.setBounds (nb); shadow.repaint(); }
    overlay.update();
}

void AudioLabEqEditor::NodeOverlay::update()
{
    juce::Array<int> now;
    if (ed.panel.isVisible())
    {
        const auto zone = ed.panel.getBounds().toFloat().expanded (14);
        for (int b : ed.display.getSelection())
            if (zone.contains (ed.display.nodePos (b) + ed.display.getPosition().toFloat())) now.add (b);
    }
    shown = now;
    const bool vis = ! now.isEmpty();
    if (vis) setBounds (ed.panel.getBounds().expanded (16));
    if (isVisible() != vis) setVisible (vis);
    if (vis) repaint();
}

void AudioLabEqEditor::NodeOverlay::paint (juce::Graphics& g)
{
    g.addTransform (juce::AffineTransform::translation ((float) (ed.display.getX() - getX()), (float) (ed.display.getY() - getY())));
    for (int b : shown) ed.display.paintNode (g, b);
}

bool AudioLabEqEditor::NodeOverlay::hitTest (int x, int y)
{
    const juce::Point<float> pt ((float) (x + getX() - ed.display.getX()), (float) (y + getY() - ed.display.getY()));
    for (int b : shown) if (ed.display.nodePos (b).getDistanceFrom (pt) < 13.0f) return true;
    return false;
}

void AudioLabEqEditor::Shadow::paint (juce::Graphics& g)
{
    if (! ed.panel.isVisible()) return;
    juce::Path box; box.addRoundedRectangle (ed.panel.getBounds().toFloat(), 12);
    const bool dark = ed.theme.pal().dark;
    juce::DropShadow (juce::Colours::black.withAlpha (dark ? 0.6f : 0.16f), 36, { 0, 14 }).drawForPath (g, box);
}

void AudioLabEqEditor::timerCallback()
{
    const bool ag = proc.getParam (P::autoGain) > 0.5f;
    if (autoBtn.on != ag) { autoBtn.on = ag; autoBtn.repaint(); }
    if (ag)
    {
        const String tip = U ("Gain auto : ") + P::fmtDb (proc.getAutoGainDb()) + " dB";
        if (autoBtn.getTooltip() != tip) autoBtn.setTooltip (tip);
    }
    // le panneau suit les changements faits ailleurs (automation, menu, molette…)
    const auto& sel = display.getSelection();
    if (sel.size() != panel.getCount() || (panel.getCount() > 0 && sel[0] != panel.getBand())) rebuildPanel();
    else { panel.sync(); placePanel(); }
}

// ── Mise en page ────────────────────────────────────────────────────────
void AudioLabEqEditor::resized()
{
    const float s = (float) getWidth() / baseW;
    content.setTransform (juce::AffineTransform::scale (s));
    content.setBounds (0, 0, baseW, baseH);
    proc.ui.setProperty ("scale", (double) s, nullptr);
}

void AudioLabEqEditor::Content::paint (juce::Graphics& g)
{
    g.fillAll (ed.theme.pal().win);
}

void AudioLabEqEditor::Content::resized()
{
    auto& e = ed;
    // courbe 1040 × 520 en (16, 16) ; à droite l'échelle dB et les crêtes ; dessous le clavier
    // écran 1000 × 520 en (24, 24) (marges de 24) ; à droite l'échelle dB et les barres de sortie ; dessous le clavier
    e.display.setBounds (24, 24, 1000 + (int) EqDisplay::rightGutter, 520 + (int) EqDisplay::bottomGutter);
    e.shadow.setBounds (getLocalBounds());
    e.toast.setBounds (24, 24, 1000, 520);

    // barre du bas : y = 584, hauteur 32, écarts de 16
    const int y = 584, h = 32;
    e.phaseSeg.setBounds (24, y, 216, h);
    e.analyzerBox.setBounds (e.phaseSeg.getRight() + 16, y, 200, h);
    int rx = baseW - 24;
    const int aw = juce::roundToInt (e.autoBtn.idealWidth());
    e.autoBtn.setBounds (rx - aw, y, aw, h); rx -= aw + 16;
    const int ow = e.vOut->idealWidth(), sw = e.vScale->idealWidth();
    e.vOut->setBounds (rx - ow, y + 2, ow, 28); rx -= ow + 16;
    e.vScale->setBounds (rx - sw, y + 2, sw, 28);
    e.placePanel();
}
