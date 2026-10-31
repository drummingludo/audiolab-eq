// Charte Audio Lab : couleurs (thème clair / sombre) et polices.
#pragma once
#include <juce_gui_basics/juce_gui_basics.h>
#include "BinaryData.h"

namespace audiolab
{
struct Pal
{
    bool dark = false;
    juce::Colour bg, bg2, bg3, bg4, text, dim, muted, border, accent, green, red, orange, yellow, teal,
                 cv, cv2, grid, grid2, txt, txt2, win, panel;
    // fond des commandes : « raised » = posées sur le panneau flottant (plus clair en thème sombre)
    juce::Colour ctl (bool raised) const { return raised && dark ? bg4 : bg3; }
    juce::Colour ctlHover (bool raised) const { return raised && dark ? juce::Colour (0xFF48484A) : bg4; }
};

inline Pal lightPal()
{
    using C = juce::Colour;
    Pal p;
    p.dark = false;
    p.bg = C (0xFFF5F5F7); p.bg2 = C (0xFFFFFFFF); p.bg3 = C (0xFFF2F2F7); p.bg4 = C (0xFFE5E5EA);
    p.text = C (0xFF1D1D1F); p.dim = C (0xFF515154); p.muted = C (0xFF86868B); p.border = C (0x14000000);
    p.accent = C (0xFF007AFF); p.green = C (0xFF34C759); p.red = C (0xFFFF3B30); p.orange = C (0xFFFF9500);
    p.yellow = C (0xFFFFCC00); p.teal = C (0xFF5AC8FA);
    p.cv = C (0xFFFAFAFC); p.cv2 = C (0xFFF2F2F7); p.grid = C (0xFFE5E5EA); p.grid2 = C (0xFFD1D1D6);
    p.txt = C (0xFF6E6E73); p.txt2 = C (0xFF3A3A3C);
    p.win = C (0xFFFFFFFF); p.panel = C (0xF7FFFFFF);
    return p;
}

inline Pal darkPal()
{
    using C = juce::Colour;
    Pal p;
    p.dark = true;
    p.bg = C (0xFF000000); p.bg2 = C (0xFF1C1C1E); p.bg3 = C (0xFF2C2C2E); p.bg4 = C (0xFF3A3A3C);
    p.text = C (0xFFF5F5F7); p.dim = C (0xFFC7C7CC); p.muted = C (0xFF8E8E93); p.border = C (0x1AFFFFFF);
    p.accent = C (0xFF0A84FF); p.green = C (0xFF30D158); p.red = C (0xFFFF453A); p.orange = C (0xFFFF9F0A);
    p.yellow = C (0xFFFFD60A); p.teal = C (0xFF64D2FF);
    p.cv = C (0xFF1C1C1E); p.cv2 = C (0xFF2C2C2E); p.grid = C (0xFF3A3A3C); p.grid2 = C (0xFF48484A);
    p.txt = C (0xFF8E8E93); p.txt2 = C (0xFFD1D1D6);
    p.win = C (0xFF161618); p.panel = C (0xFF2C2C2E);
    return p;
}

// Thème partagé par tous les éléments d'une fenêtre de plugin.
class Theme
{
public:
    Theme()
    {
        regular = juce::Typeface::createSystemTypefaceFor (BinaryData::InterRegular_ttf, BinaryData::InterRegular_ttfSize);
        semi    = juce::Typeface::createSystemTypefaceFor (BinaryData::InterSemiBold_ttf, BinaryData::InterSemiBold_ttfSize);
        bold    = juce::Typeface::createSystemTypefaceFor (BinaryData::InterBold_ttf, BinaryData::InterBold_ttfSize);
        setDark (false);
    }

    void setDark (bool d) { p = d ? darkPal() : lightPal(); }
    const Pal& pal() const { return p; }

    // Taille en points, comme « font-size » en CSS. weight : 400, 600 ou 700.
    juce::Font ui (float size, int weight = 400, float tracking = 0) const
    {
        auto tf = weight >= 700 ? bold : weight >= 600 ? semi : regular;
        return juce::Font (juce::FontOptions (tf).withPointHeight (size)).withExtraKerningFactor (tracking);
    }
    juce::Font mono (float size, bool isBold = false) const
    {
        return juce::Font (juce::FontOptions (juce::Font::getDefaultMonospacedFontName(), 10.0f,
                                              isBold ? juce::Font::bold : juce::Font::plain).withPointHeight (size));
    }

private:
    Pal p;
    juce::Typeface::Ptr regular, semi, bold;
};

inline juce::String fr (const char* utf8) { return juce::String::fromUTF8 (utf8); }

// Couleurs des bandes (couleurs système Apple, comme Audio Lab)
inline juce::Colour bandColour (int i, const Pal& p)
{
    static const juce::uint32 light[] = { 0xFF007AFF, 0xFFFF9500, 0xFF34C759, 0xFFFF2D55, 0xFFAF52DE, 0xFF32ADE6,
                                          0xFFD9A400, 0xFF5856D6, 0xFFFF3B30, 0xFF00C7BE, 0xFFA2845E, 0xFF30B0C7 };
    static const juce::uint32 dark[]  = { 0xFF0A84FF, 0xFFFF9F0A, 0xFF30D158, 0xFFFF375F, 0xFFBF5AF2, 0xFF64D2FF,
                                          0xFFFFD60A, 0xFF5E5CE6, 0xFFFF453A, 0xFF63E6E2, 0xFFAC8E68, 0xFF40C8E0 };
    const int k = ((i % 12) + 12) % 12;
    return juce::Colour (p.dark ? dark[k] : light[k]);
}
} // namespace audiolab
