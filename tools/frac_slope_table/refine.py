# Table des coupes à pente libre (Source/FracSlopeTable.h), étape 2 sur 3 : nœuds ajoutés là où l'interpolation ne suffit pas.
# Ordre : python3 gen.py ; python3 refine.py ; python3 emit.py ../../Source/FracSlopeTable.h (numpy et scipy requis, plusieurs minutes).
import numpy as np, pickle, gen, emit, sys
tab=emit.tab
for rnd in range(6):
    emit.tab=tab; added=0; new=[]
    for t in range(61,960):
        if t%60==0 or any(r[0]==t for r in tab): continue
        M,J,th=emit.lookup(t); n=t/60.0
        e24,e48,sl=gen.evaluate(np.float64(th),n,M,J)
        if e24>0.012 or e48>0.025:
            # voisins du même bloc
            idx=[i for i,(a,b) in enumerate(zip(tab[:-1],tab[1:])) if (a[1],a[2])==(b[1],b[2])==(M,J) and a[0]<t<b[0]][0]
            c=[gen.fitnode(n,M,J,np.float64(th)), gen.fitnode(n,M,J,tab[idx][3]), gen.fitnode(n,M,J,tab[idx+1][3])]
            ev=[gen.evaluate(x,n,M,J) for x in c]; b=min(range(3),key=lambda i:ev[i][1])
            new.append((idx,(t,M,J,c[b],ev[b]))); added+=1
    for idx,row in sorted(new,key=lambda x:-x[1][0]): tab.insert(idx+1,row)
    tab.sort(key=lambda r:0)  # (ordre conservé)
    # retri par bloc
    print("tour",rnd,"ajoutés",added,"total",len(tab)); sys.stdout.flush()
    if added==0: break
# vérifie que chaque bloc est trié
for a,b in zip(tab[:-1],tab[1:]):
    if (a[1],a[2])==(b[1],b[2]): assert a[0]<b[0],(a[0],b[0])
print("noeuds au-dessus de 0.03/0.05 :",[(r[0],round(r[4][0],3),round(r[4][1],3)) for r in tab if r[4][0]>0.03 or r[4][1]>0.05])
pickle.dump(tab,open("tab.pkl","wb"))
