import numpy as np, sys
d = np.load(sys.argv[1])
done, cause = d["done"], d["cause"]
CAUSES = ("success","crash","overspeed","bad_attitude","out_of_range","max_altitude","overshoot","stagnation","timeout")
lens, ang_last, z_last, dxy_last, dxy_min, angmax = [],[],[],[],[],[]
T,N = done.shape
for n in range(N):
    start=0
    for t in np.flatnonzero(done[:,n]):
        ci=int(cause[t,n])
        if CAUSES[ci-1]=="bad_attitude" and t-start>=1:
            sl=(slice(start,t+1),n)
            lens.append((t-start+1)*0.1)
            ang_last.append(float(d["ang_speed"][sl][-1]))
            angmax.append(float(d["ang_speed"][sl].max()))
            z_last.append(float(d["z"][sl][-1]))
            dxy_last.append(float(d["d_xy"][sl][-1]))
            dxy_min.append(float(d["d_xy"][sl].min()))
        start=int(t)+1
print(f"bad_attitude n={len(lens)}")
if lens:
    print(f"  ep length s : med={np.median(lens):.1f} p10={np.percentile(lens,10):.1f} p90={np.percentile(lens,90):.1f}")
    print(f"  last angspd : med={np.median(ang_last):.2f} p90={np.percentile(ang_last,90):.2f} (limit 2.0)")
    print(f"  max angspd  : med={np.median(angmax):.2f}")
    print(f"  last z      : med={np.median(z_last):.1f} p10={np.percentile(z_last,10):.1f} p90={np.percentile(z_last,90):.1f}")
    print(f"  last d_xy   : med={np.median(dxy_last):.2f}  d_xy min: med={np.median(dxy_min):.2f}")
slen=[]
for n in range(N):
    start=0
    for t in np.flatnonzero(done[:,n]):
        if CAUSES[int(cause[t,n])-1]=="success": slen.append((t-start+1)*0.1)
        start=int(t)+1
print(f"success n={len(slen)} ep length med={np.median(slen) if slen else float('nan'):.1f}s")
