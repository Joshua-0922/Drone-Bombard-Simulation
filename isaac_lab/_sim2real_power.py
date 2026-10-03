"""Power analysis for the real-flight protocol (sim2real_strategy.md section 2.3): Monte-Carlo Mann-Whitney power
for per-arm n given CEP50 pairs, 2D Gaussian error with optional heavy tail. stdlib only:  python3 isaac_lab/_sim2real_power.py"""
import random, math
random.seed(1)
def rad(cep50, n, tail):
    s=cep50/1.1774
    out=[]
    for _ in range(n):
        r=math.hypot(random.gauss(0,s),random.gauss(0,s))
        if tail and random.random()<0.1: r*=1.6   # heavier tail: CEP90/CEP50 ~2
        out.append(r)
    return out
def mw_p(a,b):  # one-sided: a larger than b
    allv=sorted([(x,0) for x in a]+[(x,1) for x in b]); R=sum(i+1 for i,(x,g) in enumerate(allv) if g==0)
    n1,n2=len(a),len(b); U=R-n1*(n1+1)/2; mu=n1*n2/2; sd=math.sqrt(n1*n2*(n1+n2+1)/12)
    return (U-mu)/sd>1.645
for tail in (0,1):
  for c1,c2 in ((0.305,0.204),(0.30,0.25),(0.25,0.20)):
    row=[]
    for n in (10,20,30,40,60):
        k=sum(mw_p(rad(c1,n,tail),rad(c2,n,tail)) for _ in range(2000)); row.append(f"{n}:{k/2000:.2f}")
    print("tail" if tail else "rayl",c1,c2," ".join(row))
