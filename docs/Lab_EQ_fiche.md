# Lab EQ (Audio Lab EQ) : fiche du plugin

Égaliseur paramétrique 24 bandes d'Audio Lab (Transversal Studio). Interface en anglais (fiche 14),
charte UI V2 (fiche 02). Fenêtre 1120 × 640, redimensionnable de 60 % à 200 % ; panneau
« Advanced settings » ouvert : 1120 × 714. Cette fiche décrit d'abord la version 3.0.0, puis la 2.0.0.

## 3.0.0 (10/11/2026) : précision mesurée et nouvelle interface

État au 10/11/2026 : terminée sur la branche `v3` du dépôt, fabriquée et validée sur GitHub
(Mac, Windows, Linux). Pas encore publiée ni installée : la publication attend l'accord de Ludovic.

Aucune écoute n'a été faite pendant ce travail. Tout ce qui suit est mesuré par le banc de tests.
Aucune supériorité sonore sur un autre égaliseur n'est affirmée ; le protocole d'écoute en aveugle
est dans la fiche 21.

### Décisions prises avec Ludovic

- Tout le monde passe au nouveau son, anciens projets compris. Un projet 1.x ou 2.0.0 rouvre avec
  les mêmes réglages, le même mode de phase et la même latence, mais la courbe est plus juste dans
  l'aigu (jusqu'à 0,4 dB de différence au-dessus de 10 kHz sur des réglages courants, davantage sur
  des étagères ou un tilt très résonants près de 20 kHz, que la 2.0.0 rendait mal) et l'EQ
  dynamique est plus précise dans le temps en Linear et en Natural.
- Les deux nouvelles formes sont ajoutées à la fin de la liste « Shape » (liste unique). Les
  réglages enregistrés ne bougent pas. Seule une automation de la forme déjà écrite par l'hôte en
  valeur normalisée serait relue de travers (Low cut lu comme High shelf, par exemple). Cas rare.
- Interface : piste B (bandeau fixe sous la courbe), niveaux de sortie dans la barre du bas.
- Remis à une 3.1 : dynamique spectrale, EQ match, collisions entre instances. Écartés : caractères
  de saturation, surround.

### Le son, avant et après

Banc `tests/quality_test.cpp` (34 sections, sans JUCE), lancé sur Mac à chaque fabrication ; un
échec bloque la publication.

Zero latency, écart maximal à l'égaliseur analogique de 20 Hz à 20 kHz. Le banc 3.0 est plus sévère
que celui de la 2.0.0 (étagères Q 4 et 10, coupes jusqu'à 20 kHz), d'où une colonne 2.0.0 plus
mauvaise que dans la section suivante.

| Forme | 44,1 kHz, 2.0.0 → 3.0 | 48 kHz | 96 kHz | 192 kHz |
|---|---|---|---|---|
| Cloche | 0,416 → 0,020 dB | 0,235 → 0,020 | 0,455 → 0,019 | 0,109 → 0,018 |
| Étagères, tilt | 13,6 → 0,056 | 19,4 → 0,019 | 0,359 → 0,018 | 0,077 → 0,014 |
| Notch | 0,568 → 0,016 | 0,629 → 0,013 | 0,192 → 0,015 | 0,005 → 0,002 |
| Coupe-bas | 2,54 → 0,022 | 2,10 → 0,020 | 1,32 → 0,015 | 0,116 → 0,015 |
| Coupe-haut | 1,33 → 0,020 | 0,600 → 0,020 | 0,430 → 0,019 | 0,126 → 0,015 |
| Passe-bande | 3,52 → 0,020 | 1,75 → 0,019 | 0,291 → 0,015 | 0,241 → 0,018 |

| Cible | 2.0.0 | 3.0 | Seuil du test |
|---|---|---|---|
| Écart à l'analogique, 24 000 réglages au hasard | non mesuré | 0,071 dB au pire | 0,1 dB |
| Même courbe à 44,1 / 48 / 96 / 192 kHz | 0,37 dB | 0,015 dB | 0,05 dB |
| Courbe affichée = courbe entendue (bandes fixes) | 0,0001 dB | 0,0001 dB ; 0,031 dB juste après un balayage | 0,05 dB |
| Bandes à 0 dB | bit à bit | bit à bit | 0 |
| EQ dynamique, raies parasites (Zero latency) | −71,2 dB | −167,9 dB | −110 dB |
| Automation d'une bande, raies parasites | −108,8 dB | −155,2 dB | −110 dB |
| Dix autres cas dynamiques, pire | non mesuré | −137,3 dB | −110 dB |
| Bande dynamique et automatisée à la fois | −69,3 dB | −89,3 dB | −85 dB (cible −110 non tenue) |
| EQ dynamique en Linear, erreur dans le temps (attaque 0,5 ms / 10 ms) | −29,7 / −20,4 dB | −68,7 / −62,7 dB | −60 dB |
| EQ dynamique en Linear, erreur de courbe jusqu'à ±18 dB | non mesuré | 0,07 dB | 0,1 dB |
| EQ dynamique en Natural, erreur dans le temps | non mesuré | −148,7 dB | −110 dB |
| Changement de mode ou de résolution : silence | 1,5 à 16 ms | 0 | 0 |
| Changement de mode : niveau sur bruit rose | −2 à −6 dB | −0,8 à +0,2 dB | −4,5 / +3 dB sur un essai |
| Gain automatique, écart de sonie sortie / entrée (bruit rose, 7 réglages) | −1,07 à +4,13 LU | 0,008 LU au pire | 0,1 LU |

Comment c'est fait, en clair :

- Zero latency : quand le filtre s'écarte de l'analogique de plus de 0,015 dB, la bande reçoit une à
  trois petites sections de correction, sans latence. Une bande loin de l'aigu garde son calcul
  d'avant et ne coûte rien de plus.
- EQ dynamique en Zero latency : le gain est suivi à chaque échantillon, à partir d'une table de
  filtres précalculés tous les 0,25 dB. Le son ne dépend plus de la taille des blocs de l'hôte.
- EQ dynamique en Natural : les bandes dynamiques sortent du filtre FIR et passent par le moteur
  Zero latency, juste après, alignées en temps.
- EQ dynamique en Linear : mélange de plusieurs filtres FIR à chaque échantillon (un filtre repère
  tous les 4 dB au plus). Jusqu'à 4 bandes dynamiques ; au-delà, les suivantes gardent l'ancienne
  méthode.
- Changement de mode : l'ancien et le nouveau chemin tournent ensemble pendant un fondu de 20 ms.
  La latence est annoncée à l'hôte 64 à 83 ms après le clic (4 ms en 2.0.0).
- Gain automatique : sonie mesurée en entrée et en sortie selon l'ITU-R BS.1770 (pondération K
  recalculée pour chaque fréquence d'échantillonnage : −23,000 LUFS à 48 kHz pour le sinus de
  référence, moins de 0,04 dB d'écart ailleurs). Le gain ne bouge pas pendant un silence et ne
  saute pas quand on bouge une bande. Convergence : jusqu'à 1,7 s sur un signal très coloré.
- Résolution Auto de la phase linéaire : elle vérifie l'écart réel du filtre et s'allonge tant qu'il
  dépasse 0,1 dB (1 dB toléré en 2.0.0). Elle choisit donc plus souvent un filtre long, avec plus
  de latence. Les latences des résolutions fixes ne changent pas.
- Pré-écho de la phase linéaire : mesuré sur le filtre réel et affiché dans Advanced settings, à
  côté de la latence (« Pre-echo: 170 ms above −60 dB »). Exemples à 48 kHz : cloche 3 kHz en Low,
  0,7 ms ; coupe-bas 80 Hz 24 dB/oct en Medium, 12,5 ms ; cloche 40 Hz Q 8 en Max, 160 ms ;
  Natural, 0 ms.

### Nouvelles fonctions

| Fonction | Ce qu'elle fait | Mesure |
|---|---|---|
| Flat tilt (forme) | Inclinaison à pente constante sur tout le spectre, 0 dB à la fréquence de la bande ; Gain = écart entre 20 kHz et 20 Hz | 0,04 dB de la droite idéale jusqu'à ±30 dB (0,08 dB à ±60) ; 0,101 dB en Linear Auto (seuil posé à 0,12) |
| All pass (forme) | Module plat, rotation de phase ; pente 6 dB/oct = ordre 1, sinon ordre 2 avec Q. Neutre en Linear (infobulle) | Module plat à 3·10⁻⁷ dB |
| Pente libre | De 6 à 96 dB/oct par pas de 0,1, pour les coupes (menu Slope, « Custom… ») | 0,09 dB d'écart au pire ; pente tenue à 0,5 dB/oct jusqu'à 48 dB/oct, 1 dB/oct au-delà |
| Brickwall | Coupe en mur, sans latence en Zero latency et Natural (elliptique d'ordre 12), vrai mur en Linear | Ondulation 0,03 dB ; −110 dB à un tiers d'octave |
| Seuil automatique | Le seuil suit le niveau moyen de la bande (2 s) + 3 dB ; Threshold est grisé (clic droit sur Threshold) | Même action à 0,06 dB près sur un signal 30 dB plus faible |
| Crête du spectre | Au survol d'un pic de l'analyseur, repère et fréquence ; un clic y pose une cloche Q 4 à 0 dB | +1,13 % à 110 Hz, +0,04 % à 1 kHz, −0,09 % à 6,3 kHz |
| Saisie au clavier | Double-clic (ou Entrée) sur un potentiomètre : « 2k », « 440 », « −3 », « A4 » | vue sur capture |

Nouveaux paramètres (aucun identifiant existant n'a changé ; 414 paramètres au total) :

| Identifiant | Nom | Plage | Défaut neutre |
|---|---|---|---|
| `b1_slopefree` … `b24_slopefree` | pente libre | 0 à 96 dB/oct | 0 = pente de la liste |
| `b1_brick` … `b24_brick` | Brickwall | Off / On | Off |
| `b1_thrauto` … `b24_thrauto` | seuil automatique | Off / On | Off |
| `b1_shape` … (existant) | forme | + Flat tilt (8), All pass (9) | inchangé |

Un projet 2.0.0 relu remet ces trois paramètres au neutre ; formes et pentes sont identiques
(test sur un état 2.0.0 reconstitué, pas sur un fichier produit par la 2.0.0).

### Interface : grille et proportions

La 3.0 reprend le système de mesure du Lab Comp 3.0.0 (fiche 06), adapté à 1120 × 640.

Ce qui n'allait pas en 2.0.0 : sept tailles de texte (7 à 14 px), des potentiomètres de 72 px, un
panneau flottant de 580 × 208 qui cachait le bas de la courbe, une colonne de 72 px à droite pour
l'échelle et les niveaux.

| Règle | Valeur |
|---|---|
| Module | 8 px ; tous les axes sur des pixels entiers |
| Marges de la fenêtre | 24 (3 modules) ; 16 entre éléments liés, 8 entre éléments d'un même groupe |
| Tailles de texte | 10, 12, 16 px (exception : noms de notes du clavier, 8 px, le clavier ne fait que 16 px de haut) |
| Rayons | 4, 8, 12, concentriques (segments : 8 dehors, 4 dedans, marge intérieure 4) |
| Traits | 1, 2, 3 |
| Hauteur des boutons, menus et de la barre du bas | 32 |

| Zone | Position et taille | Raison |
|---|---|---|
| Écran de la courbe | (24, 24), 1072 × 400 | toute la largeur utile ; 0 dB au centre ; 16 px par dB à ±12 dB (2 modules) |
| Échelle dB et bouton de plage | dans l'écran, calés à droite | plus de colonne perdue |
| Clavier | (24, 432), 1072 × 16 | 8 px sous l'écran, même axe des fréquences |
| Bandeau de bande | (24, 464), 1072 × 104 | fixe : rien ne masque la courbe ; deux rangées de 32 à y = 16 et 56 |
| Barre du bas | y = 584, hauteur 32 | mode de phase 320 (3 × 104 + 8), analyseur 176, Advanced settings, niveaux, Scale, Output, Auto gain |
| Advanced settings | (24, 632), 1072 × 58 | résolution (5 × 64 + 8), latence et pré-écho, side-chain |

| Élément | Mesures |
|---|---|
| Grand potentiomètre | arc 64 hors tout (rayon d'axe 30,5, trait 3), capuchon 46 (rapport 5/7, comme le Lab Comp), pas de 80 ; valeur 12 px remontée de 1 px, unité 10 px, nom 10 px en capitales 52 px sous l'axe, recentré de 0,4 px |
| Petit potentiomètre | 32 hors tout (rayon d'axe 14,5, trait 3), capuchon 20 ; nom et valeur 10 px à côté |
| Poignée de bande | 24 px (3 modules), numéro 12 px ; bande choisie : halo de 40 |
| Bandeau | identité (pastille 24, titre 16 px, trois boutons de 32, menus de 104) ; séparateur ; Frequency, Gain, Q ; séparateur ; Dynamic, side-chain, Threshold, Range ; Attack et Release en petits potentiomètres |

Gestes : glisser une poignée (Maj : fin), molette pour le Q, Alt + molette pour la plage dynamique,
double-clic sur une poignée pour l'activer ou la couper, clic droit pour le menu. Changement par
rapport à la 2.0.0 : le double-clic sur un potentiomètre ouvre la saisie au clavier ; la valeur par
défaut passe sur Alt + clic (ou Cmd + clic).

Captures de la 3.0.0 dans `docs/captures_3.0.0/` du dépôt (60 %, 100 %, 200 %, panneau ouvert et
fermé, clair et sombre). Les trois pistes de mise en page proposées, avec et sans grille, sont dans
le canevas de design du projet.

### Charge processeur

Mesurée sur Mac par la fabrication GitHub (Apple M1 virtualisé, 48 kHz stéréo, blocs de 512), en
pourcentage d'un cœur. Ce n'est pas le Mac de Ludovic, et ce n'est pas une comparaison avec Pro-Q 4 :
celle-ci se fait dans Logic (fiche 21, section 9).

| Cas | 3.0 sur Mac |
|---|---|
| Zero latency, 24 bandes dont 8 dynamiques | 2,8 % |
| Linear, 24 bandes, FIR 32768 points | 3,0 % |
| Natural, 24 bandes | 2,0 % |
| Linear 8192 points, 0 / 1 / 4 bandes dynamiques (fil audio) | 1,8 / 3,5 / 8,5 % |
| Linear 32768 points, 0 / 1 / 4 bandes dynamiques (fil audio) | 4,7 / 7,6 / 15,4 % |
| Natural, 0 / 1 / 4 bandes dynamiques (fil audio) | 2,2 / 2,9 / 3,3 % |
| Gain automatique en service | 0,35 % |

Sur la machine de calcul Linux, 2.0.0 contre 3.0 : Zero latency 24 bandes 3,8 → 4,2 % ; Linear Max
avec 4 bandes dynamiques, environ 46 % (4,4 % audio + 41 % de recalcul en arrière-plan) → 15,4 %,
désormais entièrement sur le fil audio.

Références publiques : FabFilter, Universal Audio et Waves ne publient ni chiffre de précision ni
chiffre de charge pour leurs égaliseurs. Seules les latences sont publiques (Pro-Q 4 en phase
linéaire à 44,1 kHz : 3072, 5120, 9216, 17408 et 66560 échantillons de Low à Maximum ; Lab EQ :
2304, 4352, 8448 et 16640 de Low à Max).

### Validation

- GitHub Actions, branche `v3`, fabrication 42901428180 du 10/11/2026 : tout réussi.
- Mac : banc qualité, tests du plugin, auval, pluginval niveau 10 sur VST3 et sur AU, ouverture de
  l'interface (pluginval niveau 5).
- Windows : pluginval niveau 10 sur VST3. Linux : tests DSP et tests du plugin.

### Limites connues de la 3.0.0

- Bandes dynamiques dans l'aigu : elles gardent le calcul de la 2.0.0 (jusqu'à 0,3 dB d'écart à
  l'analogique). Pour elles, la courbe affichée peut différer de l'entendu d'autant. Activer la
  dynamique sur une bande aiguë fait un petit saut de courbe, non mesuré.
- Bande à la fois dynamique et automatisée : −89 dB de parasites, pour une cible de −110.
- Linear, plus de 4 bandes dynamiques : les suivantes sont moins précises dans le temps.
- Linear Max avec 4 bandes dynamiques : 15 % d'un cœur sur le fil audio.
- Changement de mode : creux de −5,2 dB pendant 18 ms dans le pire cas (sinus 100 Hz, Linear Max
  vers Medium, chemins en opposition de phase).
- Résolution Auto : 30 réglages graves sur 152 restent hors de 0,1 dB même en Max (cloches et notch
  très étroits sous 100 Hz, coupe-bas 96 dB/oct à 20 ou 30 Hz ; pire cas : cloche 20 Hz Q 20,
  5,5 dB). La résonance dure plus longtemps que le filtre.
- Première conception d'une coupe très raide dans l'aigu : jusqu'à 0,5 ms de calcul sur le fil
  audio (risque de pic au chargement d'un preset plein de coupes raides, à vérifier sur Mac).
- Entre 20 kHz et Nyquist, à 44,1 / 48 kHz, la courbe n'est que faiblement contrainte.

### Ce qui n'a pas été testé

- Aucune écoute, aucun hôte réel (Logic, Pro Tools), aucun vrai projet 2.0.0 rouvert.
- AAX : pas fabriqué par GitHub ; à construire et à passer à la checklist de la fiche 08 sur le Mac.
- Gestes à la souris dans un vrai hôte ; menus Brickwall, Custom et seuil automatique non vus à
  l'écran ; conversion des notes (« A4 ») sans test automatique.
- Gain automatique sur de la musique réelle ; changement de mode avec des bandes dynamiques actives.
- Charge sur le Mac de Ludovic et comparaison avec Pro-Q 4.

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
