// Charte UI des plugins Audio Lab, V2 (01/11/2026) : thème sombre unique, Poppins,
// deux familles de sens (teal : niveau / forme ; orange : temps / action) et le rouge d'alerte.
#pragma once
#include <juce_gui_basics/juce_gui_basics.h>
#include "BinaryData.h"

namespace audiolab
{
struct Pal
{
    bool dark = true;
    // Neutres
    juce::Colour win      { 0xFF1D1E1C };   // fond fenêtre, fond des pistes de barres (C)
    juce::Colour surface  { 0xFF2A2B29 };   // écrans, panneaux (D)
    juce::Colour cap      { 0xFF373837 };   // capuchons, menus, boutons (D)
    juce::Colour capHover { 0xFF41423F };   // capuchon survolé (D)
    juce::Colour rail     { 0xFF3E3F3D };   // rails, grilles, bords (D)
    juce::Colour ref      { 0xFF757675 };   // repères secondaires, pointillés neutres (D)
    juce::Colour text     { 0xFFF9F9FB };   // texte (C)
    juce::Colour text2    { 0xFFC9C9CA };   // texte secondaire (D)
    juce::Colour soft     { 0xFFA5A6A6 };   // gris doux : unités, graduations, légendes (D)
    // Familles de sens
    juce::Colour teal     { 0xFF0A9396 };   // niveau, seuil, forme de la courbe (C)
    juce::Colour teal2    { 0xFF79AFB0 };   // gain et niveaux mesurés, texte teal (C)
    juce::Colour select   { 0xFF005F73 };   // élément choisi, toujours avec texte clair (C)
    juce::Colour orange0  { 0xFFBB3E03 };   // départ des dégradés orange posés sur le fond (C)
    juce::Colour orange   { 0xFFEAA93D };   // temps, action ; ambre d'avertissement (C)
    juce::Colour red      { 0xFFBC6D71 };   // alerte sur fond sombre (D)

    // Anciens noms, gardés pour le code existant
    juce::Colour bg { 0xFF1D1E1C }, bg2 { 0xFF2A2B29 }, bg3 { 0xFF373837 }, bg4 { 0xFF3E3F3D },
                 dim { 0xFFC9C9CA }, muted { 0xFFA5A6A6 }, border { 0xFF3E3F3D }, accent { 0xFF005F73 },
                 green { 0xFF79AFB0 }, cv { 0xFF2A2B29 }, cv2 { 0xFF2A2B29 }, grid { 0xFF3E3F3D }, grid2 { 0xFF3E3F3D },
                 txt { 0xFFA5A6A6 }, txt2 { 0xFFC9C9CA }, panel { 0xFF2A2B29 };
    juce::Colour ctl (bool = false) const { return cap; }
    juce::Colour ctlHover (bool = false) const { return capHover; }
};

// Famille de couleur d'une commande (arc, curseur)
enum class Fam { teal, orange, teal2 };

class Theme
{
public:
    Theme()
    {
        regular = juce::Typeface::createSystemTypefaceFor (BinaryData::PoppinsRegular_ttf, BinaryData::PoppinsRegular_ttfSize);
        medium  = juce::Typeface::createSystemTypefaceFor (BinaryData::PoppinsMedium_ttf, BinaryData::PoppinsMedium_ttfSize);
        semi    = juce::Typeface::createSystemTypefaceFor (BinaryData::PoppinsSemiBold_ttf, BinaryData::PoppinsSemiBold_ttfSize);
        bold    = juce::Typeface::createSystemTypefaceFor (BinaryData::PoppinsBold_ttf, BinaryData::PoppinsBold_ttfSize);
    }

    void setDark (bool) {}   // un seul thème (sombre)
    const Pal& pal() const { return p; }

    // Taille en px (comme « font-size » en CSS). weight : 400, 500, 600 ou 700.
    juce::Font ui (float size, int weight = 400, float tracking = 0) const
    {
        auto tf = weight >= 700 ? bold : weight >= 600 ? semi : weight >= 500 ? medium : regular;
        return juce::Font (juce::FontOptions (tf).withPointHeight (size)).withExtraKerningFactor (tracking);
    }
    juce::Font mono (float size, bool isBold = false) const { return ui (size, isBold ? 600 : 400); }

    // Couleurs de départ et d'arrivée d'une famille. onSurface : posé sur un écran ou un panneau
    // (l'orange y est plein, #BB3E03 n'étant lisible que sur le fond de la fenêtre).
    std::pair<juce::Colour, juce::Colour> family (Fam f, bool onSurface = true) const
    {
        switch (f)
        {
            case Fam::orange: return { onSurface ? p.orange : p.orange0, p.orange };
            case Fam::teal2:  return { p.teal2, p.teal2 };
            case Fam::teal:   break;
        }
        return { p.teal, p.teal2 };
    }

    // Valeur en chiffres à cellules fixes (Poppins n'a pas de chiffres tabulaires) : chaque chiffre
    // est centré dans une cellule de 0,635 / 0,647 / 0,661 em selon la graisse.
    void drawValue (juce::Graphics& g, const juce::String& s, juce::Rectangle<float> r, float size, int weight,
                    juce::Justification just = juce::Justification::centred) const
    {
        const auto f = ui (size, weight);
        const float cell = size * (weight >= 600 ? 0.661f : weight >= 500 ? 0.647f : 0.635f);
        juce::Array<float> w;
        float total = 0;
        for (auto c : s)
        {
            float cw = cell;
            if (! juce::CharacterFunctions::isDigit (c))
            {
                cw = juce::GlyphArrangement::getStringWidth (f, juce::String::charToString (c));
            }
            w.add (cw); total += cw;
        }
        float x = just.testFlags (juce::Justification::left) ? r.getX()
                : just.testFlags (juce::Justification::right) ? r.getRight() - total : r.getCentreX() - total / 2;
        g.setFont (f);
        int i = 0;
        for (auto c : s)
        {
            g.drawText (juce::String::charToString (c), juce::Rectangle<float> (x, r.getY(), w[i], r.getHeight()), juce::Justification::centred, false);
            x += w[i++];
        }
    }

private:
    Pal p;
    juce::Typeface::Ptr regular, medium, semi, bold;
};

inline juce::String fr (const char* utf8) { return juce::String::fromUTF8 (utf8); }

// Les bandes ne portent plus chacune une couleur (la charte n'admet que teal, orange et rouge) :
// elles se repèrent par leur numéro ; la bande choisie est en #005F73.
inline juce::Colour bandColour (int, const Pal& p) { return p.teal; }
} // namespace audiolab
