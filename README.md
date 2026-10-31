# Audio Lab EQ

L'égaliseur d'**Audio Lab** (« Chimiste du son », Transversal Studio) en plugin **VST3 / AU** pour Logic Pro, Ableton Live, FL Studio, Cubase, Reaper, Studio One, Bitwig…

Un égaliseur paramétrique complet, pensé pour le mixage et le mastering comme pour l'apprentissage :

- **24 bandes**, chacune : cloche, low shelf, high shelf, coupe-bas, coupe-haut, notch, passe-bande ou tilt ; pentes de 6 à 96 dB/octave pour les coupes.
- **EQ dynamique par bande** : seuil et plage (positive ou négative), attaque / retour communs. Exemples : de-esser, anti-résonances qui ne creusent que lorsque c'est nécessaire.
- **Placement par bande** : stéréo, gauche, droite, **Mid** ou **Side** ; deux courbes s'affichent quand le traitement diffère entre Mid et Side (ou gauche et droite).
- **Deux modes de phase** : *Zéro latence* (filtres analogiques modélisés, sans latence) et *Phase linéaire* (FIR symétriques, aucune rotation de phase, latence compensée par le logiciel).
- **Analyseur de spectre** avant / après (pente d'affichage de 4,5 dB/octave), **clavier de piano** sous la courbe et nom de la note de chaque bande.
- **Mode « Dessiner »** : tracez la courbe voulue, les bandes sont placées et réglées automatiquement au relâchement.
- **Écoute d'une bande** (Alt + clic maintenu ou bouton *Écouter*) pour entendre seulement la zone traitée.
- Annuler / rétablir, comparaison **A/B**, **échelle de gain** (0–200 %), **gain auto** pour comparer à niveau égal, 13 presets d'usine, thème clair / sombre qui suit l'ordinateur, fenêtre redimensionnable.
- Le réglage par défaut est **neutre** : aucune bande, le son passe sans modification.

## Prise en main

| Geste | Action |
|---|---|
| Clic dans la courbe | Nouvelle bande (près des bords : coupe-bas / coupe-haut) |
| Glisser une poignée | Fréquence et gain (coupes, notch, passe-bande : la hauteur règle le Q) |
| Maj + glisser | Réglage fin |
| Molette sur une poignée | Q (largeur) |
| Alt + molette | Plage de l'EQ dynamique |
| Alt + clic maintenu | Écouter seulement cette bande |
| Double-clic sur une poignée | Activer / désactiver la bande |
| Clic droit | Forme, pente, placement, dynamique, suppression |
| Glisser dans le vide | Sélectionner plusieurs bandes, puis les déplacer ensemble |
| Effacer | Supprimer la sélection |
| Cmd+Z / Cmd+Maj+Z | Annuler / rétablir |
| Cmd+A | Tout sélectionner |
| Clic sur « ±12 » | Échelle de l'affichage : ±3, ±6, ±12 ou ±30 dB |

## Télécharger

Dernière version : page **[Releases](https://github.com/drummingludo/audiolab-eq/releases/latest)** du dépôt. Deux archives :
- `AudioLabEQ-Mac.zip` : VST3 + AU, Apple Silicon et Intel, avec un installateur en un double-clic.
- `AudioLabEQ-Windows.zip` : VST3 64 bits.

Chaque envoi sur `main` met à jour la version indiquée dans `CMakeLists.txt` (`project(AudioLabEQ VERSION …)`) : changer ce numéro crée une nouvelle version.
Mode d'emploi : fichier `LISEZ-MOI.txt` dans chaque archive. Pro Tools n'est pas compatible (format AAX).

## Fabrication et tests

Chaque envoi sur `main` déclenche les tests (Linux), puis la fabrication sur des machines Mac et Windows de GitHub, la validation automatique (`auval` pour l'AU, `pluginval` niveau 10 pour tous les formats) et la publication sur la page Releases.

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Tests (Linux ou Mac) :

```
cmake -B build -DALEQ_TESTS=ON -DALEQ_STANDALONE=ON
cmake --build build --target ALEQDspTest ALEQProcessorTest ALEQRenderUI
./build/ALEQDspTest_artefacts/Release/ALEQDspTest            # réponse mesurée = courbe affichée, Mid/Side, dynamique, phase linéaire
./build/ALEQProcessorTest_artefacts/Release/ALEQProcessorTest  # presets, annuler, A/B, état, latence, mono, charge CPU
./build/ALEQRenderUI_artefacts/Release/ALEQRenderUI /tmp       # captures PNG de l'interface
```

## Architecture

- `Source/EqMath.h` : conception des filtres (SVF trapézoïdaux, Butterworth pour les coupes) et réponse analytique — la même description sert au son, à la courbe affichée et aux FIR de la phase linéaire.
- `Source/EqEngine.h` : moteur temps réel zéro latence (lissage des réglages, fondus sans clic, Mid/Side, détecteurs dynamiques, écoute).
- `Source/LinearPhase.h` : FIR symétriques calculés sur un fil séparé, matrice 2×2 pour le Mid/Side.
- `Source/SketchFit.h` : ajustement des bandes sur une courbe dessinée.
- `Source/EqDisplay.*`, `Source/PluginEditor.*` : interface (charte Audio Lab).

## Licence

Code sous **GNU AGPLv3** (voir `LICENSE`), comme la version libre du framework [JUCE](https://juce.com) qu'il utilise. Police Inter sous licence SIL OFL 1.1 (`Resources/Inter-OFL-LICENSE.txt`).

© Transversal Studio — Ludovic Diaz
