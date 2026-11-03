# Audio Lab EQ

L'égaliseur d'**Audio Lab** (« Chimiste du son », Transversal Studio) en plugin **VST3 / AU** pour Logic Pro, Ableton Live, FL Studio, Cubase, Reaper, Studio One, Bitwig…

Un égaliseur paramétrique complet, pensé pour le mixage et le mastering comme pour l'apprentissage :

- **24 bandes**, chacune : cloche, low shelf, high shelf, coupe-bas, coupe-haut, notch, passe-bande ou tilt ; pentes de 6 à 96 dB/octave pour les coupes.
- **EQ dynamique par bande** : seuil, plage (positive ou négative), attaque et retour propres à chaque bande, détecteur sur le signal de la bande ou sur le **side-chain externe** (Internal / External). Exemples : de-esser, anti-résonances qui ne creusent que lorsque c'est nécessaire, basse qui s'efface sous le kick.
- **Placement par bande** : stéréo, gauche, droite, **Mid** ou **Side** ; deux courbes s'affichent quand le traitement diffère entre Mid et Side (ou gauche et droite).
- **Trois modes de phase** : *Zero latency* (filtres « matched » : la courbe analogique jusqu'à 20 kHz, sans latence), *Natural* (module et phase de l'égaliseur analogique, latence 8 ms) et *Linear* (FIR symétriques, aucune rotation de phase ; résolution Auto / Low / Medium / High / Max dans *Advanced settings*, latence compensée par le logiciel). L'écran montre toujours la réponse réelle.
- **Analyseur de spectre** avant / après (pente d'affichage de 4,5 dB/octave), **clavier de piano** sous la courbe et nom de la note de chaque bande.
- **Dessiner la courbe** (clic droit dans la courbe) : tracez la courbe voulue, les bandes sont placées et réglées automatiquement au relâchement.
- **Écoute d'une bande** (Alt + clic maintenu ou bouton *Écouter*) pour entendre seulement la zone traitée.
- Interface épurée : la courbe occupe toute la fenêtre, les réglages de la bande sélectionnée apparaissent dans un **panneau flottant** sous la courbe.
- Annuler / rétablir (Cmd+Z), **échelle de gain** (0–200 %), **gain auto** pour comparer à niveau égal, interface sombre selon la charte Audio Lab V2 (teal pour les niveaux et la forme de la courbe, orange pour le temps et l'action du traitement, police Poppins), fenêtre redimensionnable.
- Le réglage par défaut contient **cinq bandes à plat** (low shelf 60 Hz, cloches 300 Hz, 1 kHz, 3 kHz, high shelf 10 kHz, toutes à 0 dB) : le son passe sans modification, il suffit de tirer une poignée.

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
| Clic droit sur une poignée | Forme, pente, placement, dynamique, suppression |
| Clic droit dans le vide | Ajouter une bande d'une forme précise, dessiner la courbe, afficher l'aide |
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
./build/ALEQProcessorTest_artefacts/Release/ALEQProcessorTest  # réglages types, annuler, état, projets v1.0, latence, mono, charge CPU
./build/ALEQRenderUI_artefacts/Release/ALEQRenderUI /tmp       # captures PNG de l'interface
```

Banc qualité du moteur, sans JUCE (seuils chiffrés, aussi lancé par la fabrication Mac) :

```
clang++ -O2 -std=c++17 -I Source tests/quality_test.cpp -o qt && ./qt
```

Fiche du plugin et mesures avant / après de la 2.0.0 : `docs/Lab_EQ_fiche.md`.

## Architecture

- `Source/EqMath.h` : conception des filtres (SVF trapézoïdaux ; conception « matched » en zéro latence, prototype analogique pour les FIR) et réponse analytique : la même description sert au son, à la courbe affichée et aux FIR.
- `Source/EqEngine.h` : moteur temps réel zéro latence (coefficients interpolés à chaque échantillon, fondus sans clic, Mid/Side, détecteurs dynamiques, side-chain externe, écoute).
- `Source/FirDesign.h` : conception des FIR Linear / Natural phase (sans JUCE), matrice 2×2 pour le Mid/Side.
- `Source/LinearPhase.h` : convolution par partitions qui garde l'historique d'un FIR à l'autre (fondu sans trou), calcul sur un fil séparé, courbe réelle pour l'écran.
- `Source/SketchFit.h` : ajustement des bandes sur une courbe dessinée.
- `Source/EqDisplay.*`, `Source/PluginEditor.*` : interface (charte Audio Lab).

## Licence

Code sous **GNU AGPLv3** (voir `LICENSE`), comme la version libre du framework [JUCE](https://juce.com) qu'il utilise. Police Poppins sous licence SIL OFL 1.1 (`Resources/Poppins-OFL-LICENSE.txt`).

© Transversal Studio — Ludovic Diaz
