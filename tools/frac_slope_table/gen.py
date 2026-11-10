# Table des coupes à pente libre (Source/FracSlopeTable.h), étape 1 sur 3 : ajustement des prototypes normalisés (voir refine.py pour l'ordre).
import numpy as np, sys, warnings
warnings.filterwarnings("ignore")
from scipy.optimize import least_squares
LN=10/np.log(10)
def model(th, M, J, x):
    i=0; P=np.zeros_like(x)
    if M%2:
        w=np.exp(th[i]); i+=1; P-=np.log1p((x/w)**2)
    for _ in range(M//2):
        w=np.exp(th[i]); k=np.exp(th[i+1]); i+=2
        u=(x/w)**2; P-=np.log((1-u)**2+k*k*u)
    for _ in range(J):
        p=np.exp(th[i]); z=np.exp(th[i+1]); i+=2
        P+=np.log1p((x/z)**2)-np.log1p((x/p)**2)
    return LN*P
def grid(n, floor_db=-60, N=200):
    xe=10**(-floor_db/(20*n)); x=np.exp(np.linspace(np.log(0.02),np.log(xe),N))
    T=-10*np.log10(1+x**(2*n)); wt=np.where(T>-36,1.0,np.maximum(0.03,1+(T+36)/25))
    return x,T,wt
def evaluate(th,n,M,J):
    xe=10**(60/(20*n)); xx=np.exp(np.linspace(np.log(0.02),np.log(xe),6000)); TT=-10*np.log10(1+xx**(2*n)); H=model(th,M,J,xx)
    e24=np.max(np.abs(H-TT)[TT>-24]); e48=np.max(np.abs(H-TT)[TT>-48])
    x12=np.interp(12,-H,xx); x36=np.interp(36,-H,xx)   # (H décroissant)
    i12=np.argmax(H<-12); i36=np.argmax(H<-36)
    f=lambda i,l: xx[i-1]*(xx[i]/xx[i-1])**((l-H[i-1])/(H[i]-H[i-1]))
    sl=24/np.log2(f(i36,-36)/f(i12,-12))
    return e24,e48,sl
def butter(M):
    th=[]
    if M%2: th+=[0.0]
    for q in range(M//2):
        ang=(2*q+1)*np.pi/(2*M) if M%2==0 else (q+1)*np.pi/M
        th+=[0.0,np.log(2*np.cos(ang))]
    return th
def fitnode(n,M,J,th0):
    x,T,wt=grid(n)
    r=least_squares(lambda t:(model(t,M,J,x)-T)*wt, th0, method='lm', max_nfev=6000, xtol=1e-13, ftol=1e-13)
    return r.x
def structure(n):
    if n<2: return 2,3
    M=int(np.floor(n+0.5))
    return M,(2 if n<7.5 else 0)
def interval_nodes(lo,hi,center):
    offs=[1,2,4,7,11,16,23,30]
    ns=set()
    for o in offs:
        for s in (-1,1):
            v=center+s*o/60.0
            if lo-1e-9<=v<=hi+1e-9: ns.add(round(v*60))
    return sorted(ns)
rows=[]   # (n60, M, J, th)
def march(center,lo,hi,M,J,dense=None):
    ns=dense if dense is not None else interval_nodes(lo,hi,center)
    up=[v for v in ns if v>center*60]; dn=[v for v in ns if v<center*60][::-1]
    for seq in (up,dn):
        th=None
        for v in seq:
            n=v/60.0
            cands=[]
            if th is not None: cands.append(fitnode(n,M,J,th))
            if th is None or True:
                xe=10**(60/(20*n)); st=butter(M)
                for j in range(J):
                    c=np.log(xe)*(j+0.7)/J; st+=[c+0.3,c+0.6] if n<center else [c+0.6,c+0.3]
                cands.append(fitnode(n,M,J,st))
                st2=butter(M)
                for j in range(J):
                    c=np.log(xe)*(j+0.4)/J; st2+=[c+0.2,c+0.5] if n<center else [c+0.5,c+0.2]
                cands.append(fitnode(n,M,J,st2))
            ev=[evaluate(c,n,M,J) for c in cands]
            # continuité d'abord : on garde la continuation si elle est à moins de 0,01 dB de la meilleure
            best=min(range(len(cands)),key=lambda i:ev[i][1])
            pick=0 if (th is not None and ev[0][1]<ev[best][1]+0.01) else best
            th=cands[pick]
            rows.append((v,M,J,th.copy(),ev[pick]))
            print("n=%.4f M%d J%d e24 %.4f e48 %.4f slope %+.3f (%d)"%(n,M,J,ev[pick][0],ev[pick][1],ev[pick][2]-6*n,pick)); sys.stdout.flush()
if __name__=="__main__":
    march(2.0,1.0,2.0,2,3,dense=[61,62,64,67,71,76,83,90,97,104,109,113,116,118,119])
    for m in range(2,17):
        M=m; lo=max(2.0,m-0.5); hi=min(16.0,m+0.5)
        if m<8: march(m,lo,hi,M,2)
        elif m==8:
            march(8,7.5,8.0,8,2); march(8,8.0,8.5,8,0)
        else: march(m,lo,hi,M,0)
    import pickle; pickle.dump(rows,open("rows.pkl","wb"))
