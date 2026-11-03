# Lab EQ (Audio Lab EQ) : fiche du plugin

Égaliseur paramétrique 24 bandes d'Audio Lab (Transversal Studio). Interface en anglais (fiche 14),
charte UI V2 (fiche 02). Fenêtre 1120 × 640, redimensionnable de 60 % à 200 % ; panneau
« Advanced settings » ouvert : 1120 × 714. Cette fiche décrit d'abord la version 2.0.0.

## 2.0.0 (03/11/2026) : qualité audio de niveau professionnel

En une phrase : la courbe que l'on règle est enfin celle que l'on entend, jusqu'à 20 kHz, dans les
trois modes, et l'EQ dynamique ne crée plus de parasites.

### Ce qui change

1. **Zero latency sans « écrasement » près de Nyquist.** La 1.x concevait ses filtres par
   transformation bilinéaire : près de 20 kHz, la courbe était tordue (une cloche +6 dB à 16 kHz ne
   donnait que +0,7 dB à 20 kHz au lieu de +5 dB, à 44,1 kHz). La 2.0 utilise une conception
   « matched » (d'après Vicanek, 2016) : les pôles sont placés exactement, le reste du filtre est
   ajusté au filtre analogique sur toute la bande, avec le gain exact en DC. Le résultat est remis
   sous la forme du filtre SVF existant : la stabilité en modulation (EQ dynamique) est gardée.
   L'écran montre la réponse réelle du filtre numérique.
2. **Linear phase : résolution réglable et courbe réelle.** Nouveau réglage « Linear phase
   resolution » (Auto / Low / Medium / High / Max). La fenêtre du FIR a changé (plate au centre,
   bords Blackman-Harris) : à longueur égale, la précision dans le grave est bien meilleure. Les
   notch ont un creux garanti au centre. L'écran affiche la réponse RÉELLE du FIR (longueur et
   fenêtre comprises) : en Low, on voit qu'une cloche étroite à 50 Hz est adoucie.
3. **Nouveau mode « Natural phase ».** Troisième choix entre Zero latency et Linear : un FIR qui
   reproduit le module ET la phase de l'égaliseur analogique idéal (sans l'écrasement du zéro
   latence, sans le pré-écho de la phase linéaire), avec une petite latence fixe de 8 ms.
4. **EQ dynamique.** Side-chain externe (bus « Side-chain » facultatif) et choix par bande
   « Internal / External » sous le bouton « Dynamic ». Le gain dynamique suit le détecteur à chaque
   échantillon (avant : par paliers de 32 échantillons, d'où des raies parasites).
5. **Lecture des FIR sans trou.** En 1.x, chaque changement du FIR (tourner un bouton, EQ dynamique)
   en phase linéaire faisait taire la sortie environ 55 ms (le moteur de convolution de JUCE
   repartait d'un historique vide). La 2.0 a son propre moteur de convolution par partitions qui garde
   l'historique et passe d'un FIR à l'autre par un fondu de 21 ms.
6. **Changement de mode sans clic.** Fondu de sortie (5 ms), bascule, attente du FIR, fondu
   d'entrée. Les réglages qui bougent (fréquence, gain, Q) ne font plus d'escalier : les coefficients
   sont interpolés à chaque échantillon.

### Nouveaux réglages

| Identifiant | Nom (logiciel) | Plage | Défaut | Où |
|---|---|---|---|---|
| `phase` (existant) | Phase mode | Zero latency (0) / Linear phase (1) / **Natural phase (2)** | Zero latency | barre du bas : Zero latency / Natural / Linear |
| `lin_res` | Linear phase resolution | Auto / Low / Medium / High / Max | Auto (projets 1.x : Medium) | Advanced settings |
| `b1_sc` … `b24_sc` | Band n Side-chain | Internal / External | Internal | panneau de bande, sous « Dynamic » |

Le sens des valeurs 0 et 1 de `phase` ne change pas (les projets rouvrent dans le même mode).
Seule une automation du mode de phase écrite par l'hôte en valeur normalisée verrait « 1.0 »
devenir Natural phase (cas très rare : ce réglage change la latence).

Advanced settings (fermé par défaut, état gardé avec la session) : résolution, latence en cours
(ms, échantillons, longueur du FIR), état de l'entrée side-chain (connectée ou non). Si une bande
est en External sans side-chain branché, elle suit son propre signal (bulle d'aide).

### Latences (annoncées à l'hôte, signal sec et bypass recalés)

| Mode | 44,1 / 48 kHz | 88,2 / 96 kHz | 176,4 / 192 kHz |
|---|---|---|---|
| Zero latency | 0 | 0 | 0 |
| Natural phase | 384 éch. (8,7 / 8,0 ms) | 768 éch. (8,0 ms) | 1536 éch. (8,0 ms) |
| Linear, Low (FIR 4096 à 48 kHz) | 2304 éch. (48 ms à 48 kHz) | 4608 | 9216 |
| Linear, Medium (8192) | 4352 (91 ms) | 8704 | 17408 |
| Linear, High (16384) | 8448 (176 ms) | 16896 | 33792 |
| Linear, Max (32768) | 16640 (347 ms) | 33280 | 66560 |

Latence = moitié du FIR + un bloc de partition (256 éch. à 48 kHz) ; Natural phase = 128 éch. de
départ + le bloc. **Auto** choisit la longueur d'après la bande la plus grave et la plus étroite
(gain de base + plage dynamique comprise : l'EQ dynamique ne change jamais la latence). Pendant la
lecture, la latence en place est gardée ; elle ne change qu'à l'arrêt. Un changement fait à la main
(mode, résolution) passe par un court silence (fondu) le temps de concevoir le FIR.

### Mesures avant (1.3.0) / après (2.0.0)

Banc `tests/quality_test.cpp` (sans JUCE, seuils chiffrés), `tests/dsp_test.cpp`, `tests/processor_test.cpp`.

**Zero latency : écart max à l'analogique, 20 Hz à 20 kHz** (cloches ±18 dB Q 0,1 à 40 ; étagères et
tilt Q 0,3 à 2 ; notch Q 1 à 40 ; coupes et passe-bande : zone utile au-dessus de −24 dB)

| Forme | 44,1 kHz avant → après | 48 kHz | 96 kHz | 192 kHz |
|---|---|---|---|---|
| Cloche | 12,67 → **0,42 dB** | 10,10 → 0,24 | 2,01 → 0,45 | 0,48 → 0,11 |
| Low / high shelf | 12,02 → **0,43** | 10,07 → 0,28 | 2,17 → 0,28 | 0,51 → 0,08 |
| Tilt | 8,71 → **0,37** | 7,26 → 0,25 | 1,56 → 0,13 | 0,37 → 0,01 |
| Notch | 2,84 → **0,54** | 2,67 → 0,63 | 1,03 → 0,19 | 0,28 → 0,01 |
| Coupe-bas (jusqu'à 5 kHz) | 1,94 → **0,11** | 1,63 → 0,07 | 0,40 → 0,03 | 0,10 → 0,01 |
| Coupe-haut | 83,0 → **1,33** | 53,1 → 0,60 | 6,69 → 0,43 | 1,49 → 0,12 |
| Passe-bande (jusqu'à 10 kHz) | 27,6 → **0,95** | 18,9 → 0,57 | 3,06 → 0,12 | 0,71 → 0,13 |

- Cloche 16 kHz +6 dB Q 1 à 20 kHz, 44,1 kHz : analogique +4,95 dB ; 1.3 : **+0,71** ; 2.0 : **+5,05**.
- Même courbe d'une fréquence d'échantillonnage à l'autre (7 réglages aigus, écart à la courbe
  192 kHz) : 4,42 dB → **0,20 dB**.
- Réponse mesurée du moteur = courbe affichée : écart 0,0001 dB.
- Bandes à 0 dB : sortie identique à l'entrée, bit à bit.

**Linear phase, 48 kHz, résolution Auto** (cibles de l'audit) :

| Réglage | 1.3 (8192 points, Blackman-Harris) | 2.0 Auto | Cible |
|---|---|---|---|
| Cloche 50 Hz −12 dB Q 8 : écart max 20 à 200 Hz | 7,67 dB | **0,09 dB** (32768 points) | ±1 dB |
| Notch 60 Hz Q 10 : creux au centre | −2,8 dB | **−150 dB** | ≤ −20 dB |
| Coupe-bas 30 Hz 48 dB/oct : écart à 20 Hz (−28,2 dB attendu) | 11,9 dB | **0,00 dB** | ±1 dB |
| Changement de FIR pendant la lecture | silence ≈ 55 ms | **aucun creux** (fondu 21 ms) | |

**Natural phase** (étagère 8 kHz +9 dB, cloche 100 Hz −8 dB Q 4, coupe-bas 30 Hz, cloche 16 kHz +6 dB) :
module et phase égaux à l'analogique à 0,001 dB / 0,01° près jusqu'à 18 kHz (44,1 à 192 kHz).

**EQ dynamique** (sinus 997 Hz d'amplitude variable, attaque 0,5 ms, plage −12 dB), raie parasite la
plus forte à f0 ± k·fs/32 : **−68,4 dB → −100,2 dB**. Fréquence d'une cloche +12 dB modulée sur un
sinus 3 kHz : **−63,1 dB → −108,8 dB**.

**Charge processeur** (Linux, compilation optimisée, 48 kHz) : zéro latence, 24 bandes stéréo dont 8
dynamiques : 2,3 % (1.3) → 3 à 4 % d'un cœur (2.0 : conceptions des bandes dynamiques mises en
cache tous les 0,25 dB). FIR : 0,8 % (8192 points) à 2,5 % (32768 points, matrice Mid/Side).

### Limites connues (à 44,1 / 48 kHz)

- Coupes très raides (48 à 96 dB/oct) près de Nyquist : jusqu'à 1,3 dB d'écart à 16 kHz, à 44,1 kHz
  (avant : jusqu'à 83 dB). Coupe-bas placée au-delà de 10 kHz, passe-bande très raides au-delà de
  10 kHz : écart de 1 à 4 dB (réglages sans usage courant).
- Étagères avec un Q très élevé (au-delà de 10) entre 3 et 10 kHz : moins précises (le creux et la
  bosse très étroits près de Nyquist ne se reproduisent pas avec un seul filtre du 2e ordre).
- Une cloche à 0,5 dB près jusqu'à 20 kHz : un filtre numérique a toujours une pente nulle à
  Nyquist ; l'analogique non. L'écart restant se loge entre 18 et 22 kHz.
- Modes FIR : l'EQ dynamique passe par une nouvelle conception du FIR (toutes les 25 ms environ) ;
  le détecteur lit le signal retardé de la latence moins 30 ms pour que l'action tombe à peu près
  au bon moment. Pour une dynamique rapide, Zero latency reste le bon choix.
- Linear phase en Low ou Medium avec une bande très grave et très étroite : la courbe réelle
  (affichée) est adoucie ; Auto choisit la longueur qui tient la précision.

### Choix faits (et pourquoi)

- Défaut de la résolution : **Auto** pour un nouveau plugin (précision garantie) ; **Medium** pour
  un projet 1.x (même longueur de FIR qu'avant, latence presque identique : +256 éch. de bloc).
- Natural phase prend toujours la longueur Auto : sa latence n'en dépend pas, seule la charge CPU.
- Le side-chain est choisi par bande (la place existait sous « Dynamic »).
- Bus side-chain désactivé par défaut ; « External » sans signal branché = détection interne.
- La page web Audio_Lab.html ne change pas : le plugin est la version « pro ».
  (Ce dépôt n'a pas de `tests/verify_dsp.py` : il n'y avait pas de comparaison au code web à retirer.)

### Tests

- `tests/quality_test.cpp` : 11 bancs chiffrés (ci-dessus), lancé aussi par la fabrication Mac.
- `tests/dsp_test.cpp` : réponse mesurée = affichée (21 réglages × 3 fréquences), Mid/Side,
  dynamique, convolution par partitions (latence, symétrie, réponse, changement de FIR sans trou,
  Natural phase).
- `tests/processor_test.cpp` : presets, annuler, A/B, état, latences (Auto, Low à Max, Natural),
  latence gardée pendant la lecture, bypass recalé, changement de mode sans clic, projets 1.x
  (résolution Medium, side-chain Internal, ancien format A/B), side-chain externe, mono, charge CPU.
- pluginval niveau 10 (Linux, VST3) : réussi.

## Rappel de l'interface (1.3)

Courbe 1000 × 520, analyseur avant / après, clavier de piano, poignées numérotées ; panneau flottant
de la bande sous la courbe (forme, pente, placement, Frequency / Gain / Q, Dynamic, Threshold,
Range, Attack, Release, et en 2.0 Internal / External) ; barre du bas : mode de phase, analyseur,
Advanced settings, Scale, Output, Auto gain.
