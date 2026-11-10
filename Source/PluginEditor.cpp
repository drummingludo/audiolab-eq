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
      autoBtn (theme, PillButton::Style::ibtn, U ("Auto gain"))
{
    setLookAndFeel (&lnf);
    tooltips.setLookAndFeel (&lnf);
    setWantsKeyboardFocus (true);
    addAndMakeVisible (content);
    auto& ap = proc.apvts;
    auto prm = [&] (const char* id) -> juce::RangedAudioParameter& { return *ap.getParameter (id); };

    // Courbe + bandeau de la bande
    display.onSelectionChanged = [this] { rebuildPanel(); };
    display.onMessage = [this] (const String& s) { toast.show (s); };
    display.onUndoRedo = [this] { rebuildPanel(); toast.show (U ("Undone / redone")); };
    display.onMeters = [this] { content.repaint (meterArea().expanded (2)); };
    content.addAndMakeVisible (display);
    content.addAndMakeVisible (panel);

    // Barre du bas : phase, analyseur, réglages avancés, échelle, sortie, gain auto
    // (segments : Zero latency / Natural / Linear ; valeurs du paramètre : 0 / 2 / 1)
    phaseAtt = std::make_unique<juce::ParameterAttachment> (prm (P::phase), [this] (float v)
    {
        const int m = juce::roundToInt (v);
        phaseSeg.setSelected (m == P::phaseLinear ? 2 : m == P::phaseNatural ? 1 : 0);
    }, nullptr);
    phaseSeg.onSelect = [this] (int i)
    {
        const int m = i == 2 ? P::phaseLinear : i == 1 ? P::phaseNatural : P::phaseZero;
        phaseAtt->setValueAsCompleteGesture ((float) m);
        const double ms = 1000.0 * proc.latencyForMode (m) / proc.getSampleRateSafe();
        toast.show (m == P::phaseLinear ? U ("Linear phase: no phase rotation, latency ") + String (ms, 1) + " ms"
                  : m == P::phaseNatural ? U ("Natural phase: analogue magnitude and phase, latency ") + String (ms, 1) + " ms"
                                         : U ("Zero latency: analogue-matched filters, no latency"));
    };
    phaseAtt->sendInitialUpdate();
    phaseSeg.setTitle (U ("Phase mode"));
    phaseSeg.setTooltip (U ("Zero latency: analogue-matched filters, no latency. Natural: the magnitude and phase of an analogue equaliser, 8 ms latency. "
                            "Linear: no phase rotation at all, longer latency (resolution in Advanced settings)"));

    // Réglages avancés : résolution de la phase linéaire, état du side-chain
    resSeg.onSurface = true;
    resSeg.attach (prm (P::linRes));
    resSeg.setTitle (U ("Linear phase resolution"));
    resSeg.setTooltip (U ("Length of the linear phase filter. Auto: chosen from the lowest and narrowest band (the latency only changes when playback is stopped). "
                          "Low: shortest latency, less precise in the low end. Max: most precise, longest latency."));
    advBtn.setTooltip (U ("Show / hide the linear phase resolution and the side-chain status"));
    advBtn.onClick = [this] { setAdvanced (! advOpen, true); };

    analyzerBox.addItem (U ("Analyser: pre + post"), 1);
    analyzerBox.addItem (U ("Analyser: post"), 2);
    analyzerBox.addItem (U ("Analyser: pre"), 3);
    analyzerBox.addItem (U ("Analyser: off"), 4);
    analyzerBox.setTitle (U ("Analyser"));
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
    vScale = std::make_unique<ArcKnob> (theme, prm (P::gainScale), U ("Scale"), "%", Fam::teal, ArcKnob::Size::small,
        [] (float v) { return String (juce::roundToInt (v)) + " %"; }, 0.0f,
        U ("Gain scale: multiplies the gain of every band (100 % = as is, 0 % = flat curve, 200 % = doubled). Double-click: 100 %"));
    vOut = std::make_unique<ArcKnob> (theme, prm (P::outGain), U ("Output"), "dB", Fam::teal2, ArcKnob::Size::small,
        [] (float v) { return P::fmtDb (v) + " dB"; }, 0.0f, U ("Output gain. Double-click: 0 dB"));
    for (auto* v : { vScale.get(), vOut.get() }) { v->labelLeft = true; v->onGestureStart = [this] { proc.pushUndo(); }; }

    autoBtn.setTooltip (U ("Auto gain: compensates the average loudness added or removed by the EQ, to compare at equal level"));
    autoBtn.onClick = [this]
    {
        proc.pushUndo();
        proc.setValue (proc.apvts.getParameter (P::autoGain), proc.getParam (P::autoGain) > 0.5f ? 0.0f : 1.0f);
    };

    for (juce::Component* c : { (juce::Component*) &phaseSeg, (juce::Component*) &analyzerBox, (juce::Component*) vScale.get(),
                                (juce::Component*) vOut.get(), (juce::Component*) &autoBtn, (juce::Component*) &advBtn })
        content.addAndMakeVisible (c);
    content.addChildComponent (resSeg);

    // Bypass du logiciel hôte : la courbe s'estompe
    bypassAtt = std::make_unique<juce::ParameterAttachment> (prm (P::bypass), [this] (float v) { display.setAlpha (v > 0.5f ? 0.45f : 1.0f); }, nullptr);
    bypassAtt->sendInitialUpdate();

    toast.atTop = true;
    content.addAndMakeVisible (toast);

    juce::Desktop::getInstance().addDarkModeSettingListener (this);
    applyTheme();

    const double sc = juce::jlimit (0.6, 2.0, (double) proc.ui.getProperty ("scale", 1.0));   // lu avant que les limites ne redimensionnent
    setAdvanced ((bool) proc.ui.getProperty ("adv", false), false);   // panneau avancé caché par défaut
    setResizable (true, true);
    getConstrainer()->setFixedAspectRatio ((double) baseW / currentH());
    setResizeLimits (baseW * 6 / 10, currentH() * 6 / 10, baseW * 2, currentH() * 2);
    setSize (juce::roundToInt (baseW * sc), juce::roundToInt (currentH() * sc));
    rebuildPanel();
    proc.editorOpen = true;
    startTimerHz (10);
}

AudioLabEqEditor::~AudioLabEqEditor()
{
    proc.editorOpen = false;
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

// Ouvre / ferme le panneau « Advanced settings » : la fenêtre grandit ou rétrécit de 74 px (à l'échelle)
void AudioLabEqEditor::setAdvanced (bool open, bool resizeWindow)
{
    advOpen = open;
    proc.ui.setProperty ("adv", open, nullptr);
    advBtn.open = open; advBtn.repaint();
    resSeg.setVisible (open);
    if (resizeWindow)
    {
        const double s = (double) getWidth() / baseW;   // échelle actuelle, lue avant de changer les limites
        getConstrainer()->setFixedAspectRatio ((double) baseW / currentH());
        setResizeLimits (baseW * 6 / 10, currentH() * 6 / 10, baseW * 2, currentH() * 2);
        setSize (juce::roundToInt (baseW * s), juce::roundToInt (currentH() * s));
    }
    resized();
    content.repaint();
}

juce::String AudioLabEqEditor::latencyText() const
{
    const int mode = proc.activePhaseMode();
    const int lat = proc.reportedLatency();
    const double ms = 1000.0 * lat / proc.getSampleRateSafe();
    if (mode == P::phaseZero) return U ("Latency: 0 ms (Zero latency mode)");
    return U ("Latency: ") + String (ms, 1) + " ms (" + String (lat) + U (" smp) · filter ") + String (proc.activeFirLength()) + U (" points");
}

juce::String AudioLabEqEditor::sideChainText() const
{
    return proc.sideChainConnected() ? U ("Side-chain input: connected") : U ("Side-chain input: not connected");
}

// ── Bandeau de la bande ────────────────────────────────────────────────────
void AudioLabEqEditor::rebuildPanel()
{
    panel.rebuild();
}

void AudioLabEqEditor::timerCallback()
{
    {
        const auto lt = latencyText(), st = sideChainText();
        if (lt != latText || st != scText) { latText = lt; scText = st; if (advOpen) content.repaint(); }
    }
    const bool ag = proc.getParam (P::autoGain) > 0.5f;
    if (autoBtn.on != ag) { autoBtn.on = ag; autoBtn.repaint(); }
    if (ag)
    {
        const String tip = U ("Auto gain: ") + P::fmtDb (proc.getAutoGainDb()) + " dB";
        if (autoBtn.getTooltip() != tip) autoBtn.setTooltip (tip);
    }
    // le panneau suit les changements faits ailleurs (automation, menu, molette…)
    const auto& sel = display.getSelection();
    if (sel.size() != panel.getCount() || (panel.getCount() > 0 && sel[0] != panel.getBand())) rebuildPanel();
    else panel.sync();
}

// ── Mise en page ────────────────────────────────────────────────────────
void AudioLabEqEditor::resized()
{
    const float s = (float) getWidth() / baseW;
    content.setTransform (juce::AffineTransform::scale (s));
    content.setBounds (0, 0, baseW, currentH());
    proc.ui.setProperty ("scale", (double) s, nullptr);
}

void AudioLabEqEditor::Content::paint (juce::Graphics& g)
{
    const auto& p = ed.theme.pal();
    g.fillAll (p.win);
    ed.display.paintMeters (g, ed.meterArea().toFloat());
    if (! ed.advOpen) return;
    // panneau des réglages avancés (« Surface », coins 12, marge intérieure 16)
    g.setColour (p.surface); g.fillRoundedRectangle (24, 632, 1072, 58, 12);
    g.setColour (p.text2); g.setFont (ed.theme.ui (12));
    g.drawText (U ("Linear phase resolution"), juce::Rectangle<float> (40, 648, 150, 26), juce::Justification::centredLeft, false);
    // à droite : état du side-chain ; au milieu : latence (place restante)
    const auto f11 = ed.theme.ui (12);
    const float right = 1096 - 16, sw = juce::GlyphArrangement::getStringWidth (f11, ed.scText) + 2;
    const float x0 = (float) ed.resSeg.getRight() + 24, sepX = right - sw - 12;
    g.setFont (f11);
    g.setColour (p.soft);
    g.drawFittedText (ed.latText, juce::Rectangle<float> (x0, 648, sepX - 12 - x0, 26).toNearestInt(), juce::Justification::centredLeft, 1, 0.9f);
    g.setColour (p.rail); g.fillRect (sepX, 652.0f, 1.0f, 18.0f);   // séparateur
    g.setColour (ed.proc.sideChainConnected() ? p.teal2 : p.soft);
    g.drawText (ed.scText, juce::Rectangle<float> (right - sw, 648, sw, 26), juce::Justification::centredRight, false);
}

void AudioLabEqEditor::Content::resized()
{
    auto& e = ed;
    // écran 1072 × 400 en (24, 24) ; clavier de 16 à y = 432 ; bandeau de bande 1072 × 104 à y = 464
    e.display.setBounds (24, 24, 1072 + (int) EqDisplay::rightGutter, 400 + (int) EqDisplay::bottomGutter);
    e.panel.setBounds (24, 464, BandPanel::panelW, BandPanel::panelH);
    e.toast.setBounds (24, 24, 1072, 400);

    // barre du bas : y = 584, hauteur 32, écarts de 16
    const int y = 584, h = 32;
    e.phaseSeg.setBounds (24, y, 320, h);            // 3 segments de 104, marge intérieure 4
    e.analyzerBox.setBounds (360, y, 176, h);
    e.advBtn.setBounds (552, y, juce::roundToInt (e.advBtn.idealWidth()), h);
    // panneau avancé : une ligne de 26 px à y = 648
    e.resSeg.setBounds (40 + 150, 648, juce::roundToInt (e.resSeg.idealWidth()), 26);
    int rx = baseW - 24;
    const int aw = juce::roundToInt (e.autoBtn.idealWidth());
    e.autoBtn.setBounds (rx - aw, y, aw, h); rx -= aw + 16;
    const int ow = e.vOut->idealWidth(), sw = e.vScale->idealWidth();
    e.vOut->setBounds (rx - ow, y, ow, h); rx -= ow + 16;
    e.vScale->setBounds (rx - sw, y, sw, h);
}
