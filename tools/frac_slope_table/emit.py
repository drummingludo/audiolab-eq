# Table des coupes à pente libre, étape 3 sur 3 : vérifie toutes les pentes de 6,1 à 95,9 dB/oct et écrit l'en-tête.
import numpy as np, pickle, sys
import gen
import os
rows=pickle.load(open("tab.pkl" if os.path.exists("tab.pkl") and __name__=="__main__" else "rows.pkl","rb"))
# blocs dans l'ordre de génération, lignes triées par pente dans chaque bloc
blocks=[]; 
for r in rows:
    key=(r[1],r[2])
    if not blocks or blocks[-1][0]!=key: blocks.append((key,[]))
    blocks[-1][1].append(r)
tab=[]
for key,rs in blocks:
    rs=sorted(rs,key=lambda r:r[0]); tab+=rs
def lookup(t):
    for a,b in zip(tab[:-1],tab[1:]):
        if (a[1],a[2])!=(b[1],b[2]) or t<a[0] or t>b[0]: continue
        u=(t-a[0])/(b[0]-a[0]); return a[1],a[2],np.float32(a[3])+(np.float32(b[3])-np.float32(a[3]))*u
    return None
worst=[0,0,0]; miss=[]
RUN=__name__=="__main__"
for t in (range(61,960) if RUN else []):
    if t%60==0: continue
    r=lookup(t)
    if r is None: miss.append(t); continue
    M,J,th=r; n=t/60.0
    e24,e48,sl=gen.evaluate(np.float64(th),n,M,J)
    ideal=24/np.log2(((10**3.6-1)/(10**1.2-1))**(1/(2*n)))
    d=abs(sl-ideal)
    if e24>worst[0]: worst[0]=e24; w0=t
    if e48>worst[1]: worst[1]=e48; w1=t
    if d>worst[2]: worst[2]=d; w2=t
if RUN: print("manquants",miss); print("pire e24 %.4f (t=%d) e48 %.4f (t=%d) pente/idéal %.4f (t=%d)"%(worst[0],w0,worst[1],w1,worst[2],w2))
if __name__=="__main__" and len(sys.argv)>1:
    out=["// Audio Lab EQ — prototypes des coupes à pente libre (fichier généré, ne pas modifier à la main).",
"// Butterworth d'ordre fractionnaire n = pente / 6 : |H|² = 1/(1 + x^(2n)), coupure normalisée x = 1.",
"// Chaque nœud : pente t (dixièmes de dB/oct), M pôles (un pôle réel si M est impair, puis M/2 sections",
"// d'ordre 2), J paires pôle / zéro du premier ordre. Données : logarithmes naturels, dans l'ordre",
"// [ln w1 si M impair], (ln w, ln 1/Q) × M/2, (ln pôle, ln zéro) × J.",
"// Ajustement hors ligne : moindres carrés (Levenberg-Marquardt) sur l'écart en dB, de x = 0,02 jusqu'à",
"// −60 dB, poids 1 jusqu'à −36 dB puis décroissant ; de proche en proche à partir de la pente entière.",
"// Écart mesuré à la cible, toutes les pentes de 6,1 à 95,9 dB/oct par pas de 0,1 (nœuds interpolés compris) :",
"// %.3f dB au plus au-dessus de −24 dB, %.3f dB au plus au-dessus de −48 dB."%(worst[0],worst[1]),
"#pragma once","","namespace audiolab::eq::frac","{","struct Node { short t, M, J; int off; };","constexpr Node nodes[] = {"]
    data=[]; line=[]
    for r in tab:
        line.append("{ %d, %d, %d, %d }"%(r[0],r[1],r[2],len(data))); data+=[float(v) for v in r[3]]
        if len(line)==6: out.append("    "+", ".join(line)+","); line=[]
    if line: out.append("    "+", ".join(line)+",")
    out+=["};","constexpr int numNodes = %d;"%len(tab),"constexpr float data[] = {"]
    for i in range(0,len(data),8): out.append("    "+", ".join("%.7gf"%v if ('.' in "%.7g"%v or 'e' in "%.7g"%v) else "%.7g.0f"%v for v in data[i:i+8])+",")
    out+=["};","} // namespace audiolab::eq::frac",""]
    open(sys.argv[1],"w",encoding="utf-8").write("\n".join(out)); print(len(tab),"noeuds",len(data),"valeurs")
