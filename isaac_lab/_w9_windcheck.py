"""Wind statistics of --dump_sl files (10 Hz rows): python.sh _w9_windcheck.py a.npz b.npz ..."""
import sys, numpy as np
for p in sys.argv[1:]:
    d = np.load(p); w, ep = d["wind"], d["ep_len"]            # (T, N, 2), (T, N)
    segs = []
    for n in range(w.shape[1]):
        cuts = np.flatnonzero(np.diff(ep[:, n]) < 0) + 1
        segs += [s for s in np.split(w[:, n], cuts) if len(s) >= 31]
    w0 = np.stack([s[0] for s in segs]); sp0 = np.linalg.norm(w0, axis=1)
    allw = np.concatenate(segs); spa = np.linalg.norm(allw, axis=1)
    out = [f"{p.split('/')[-1]:14s} eps={len(segs)} len_med={np.median([len(s) for s in segs])/10:.1f}s",
           f"|w0| mean={sp0.mean():.2f} max={sp0.max():.2f} std_axis={w0.std():.3f}",
           f"all: std_axis={allw.std():.3f} |w| p50={np.median(spa):.2f} max={spa.max():.2f}"]
    for k in (1, 3, 10, 30):
        wk = np.stack([s[k] for s in segs])
        out.append(f"k={k/10:.1f}s rms(w-w0)={np.sqrt(((wk - w0) ** 2).mean()):.3f} corr={(wk * w0).mean() / np.sqrt((wk**2).mean() * (w0**2).mean()):.3f}")
    print(" | ".join(out))
