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

    vScale = std::make_unique<ValueBox> (theme, prm (P::gainScale), U ("Échelle"), [] (float v) { return String (juce::roundToInt (v)) + " %"; }, false);
    vScale->setTooltip (U ("Échelle de gain : multiplie le gain de toutes les bandes (100 % = tel quel, 0 % = courbe plate, 200 % = doublé). Glisser verticalement, double-clic : 100 %"));
    vOut = std::make_unique<ValueBox> (theme, prm (P::outGain), U ("Sortie"), [] (float v) { return P::fmtDb (v) + " dB"; }, false);
    vOut->setTooltip (U ("Gain de sortie. Glisser verticalement, double-clic : 0 dB"));
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
    const String t = proc.ui.getProperty ("themeOverride", "auto").toString();
    const bool dark = t == "dark" || (t != "light" && juce::Desktop::getInstance().isDarkModeActive());
    theme.setDark (dark);
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
}

void AudioLabEqEditor::Shadow::paint (juce::Graphics& g)
{
    if (! ed.panel.isVisible()) return;
    juce::Path box; box.addRoundedRectangle (ed.panel.getBounds().toFloat(), 14);
    const bool dark = ed.theme.pal().dark;
    juce::DropShadow (juce::Colours::black.withAlpha (dark ? 0.55f : 0.16f), 28, { 0, 8 }).drawForPath (g, box);
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
    e.display.setBounds (16, 16, 1040 + (int) EqDisplay::rightGutter, 520 + (int) EqDisplay::bottomGutter);
    e.shadow.setBounds (getLocalBounds());
    e.toast.setBounds (16, 16, 1040, 520);

    // barre du bas (hauteur 40 à partir de y = 570)
    const int y = 575, h = 30;
    const int pw = juce::roundToInt (e.phaseSeg.idealWidth());
    e.phaseSeg.setBounds (16, y, pw, h);
    e.analyzerBox.setBounds (e.phaseSeg.getRight() + 8, y, 196, h);
    int rx = baseW - 16;
    const int aw = juce::roundToInt (e.autoBtn.idealWidth());
    e.autoBtn.setBounds (rx - aw, y, aw, h); rx -= aw + 8;
    e.vOut->setBounds (rx - 106, y, 106, h); rx -= 106 + 8;
    e.vScale->setBounds (rx - 108, y, 108, h);
    e.placePanel();
}
