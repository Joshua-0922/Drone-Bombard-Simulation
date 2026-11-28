# release-time R2 of GRU vs hand-feature MLP on the same dumps, and how well each prediction tracks the true wind
import sys, numpy as np, torch, importlib.util
spec = importlib.util.spec_from_file_location("mu", "/tmp/rebuild/drone_bombard/math_utils.py"); mu = importlib.util.module_from_spec(spec); spec.loader.exec_module(mu)
def r2(p, y): return 1 - ((p - y) ** 2).sum() / ((y - y.mean(0)) ** 2).sum()
for t in ("s1", "s2", "s4"):
    g = torch.jit.load(f"/tmp/grus8/{t}/res_gru_S.pt").eval(); m = torch.jit.load(f"/tmp/abl9/fit/{t}/mlp_gi.pt").eval()
    PG, PM, Y, W = [], [], [], []
    for f in ("a", "b"):
        d = np.load(f"/tmp/grus8/{t}/v2_s1{f}.npz")
        obs = torch.as_tensor(d["obs"]).float(); T, N, _ = obs.shape
        el, att, miss, mn, wind = d["ep_len"], d["attached"] > 0.5, d["miss"], d["miss_n"], d["wind"]
        new = np.ones((T, N), bool); new[1:] = el[1:] <= el[:-1]
        h = torch.zeros(N, int(g.hidden_size)); acc = torch.zeros(N, mu.TILT_ACCUM_N); cnt = torch.zeros(N, 1); v0 = torch.zeros(N, 2)
        pg = np.zeros((T, N, 2), np.float32); pm = np.zeros((T, N, 2), np.float32)
        with torch.no_grad():
            for k in range(T):
                o = obs[k]; r = torch.as_tensor(new[k])
                h[r] = 0; acc[r] = 0; cnt[r] = 0; v0[r] = o[r][:, 6:8]
                dg, h = g(o, h); acc += mu.tilt_channels(o); cnt += 1
                x = torch.cat([o, acc / cnt, (o[:, 6:8] - v0) / cnt], -1)
                pg[k] = dg.numpy(); pm[k] = m(x).numpy()
        for n in range(N):
            st = np.flatnonzero(new[:, n]).tolist() + [T]
            for s, e in zip(st[:-1], st[1:]):
                if e < T and mn[e, n] > mn[s, n] and att[s:e, n].any():
                    k = s + np.flatnonzero(att[s:e, n])[-1]     # last carried frame = release
                    PG.append(pg[k, n]); PM.append(pm[k, n]); Y.append(miss[e, n]); W.append(wind[k, n][:2])
    PG, PM, Y, W = map(np.array, (PG, PM, Y, W))
    c = lambda p: np.mean([np.corrcoef(p[:, i], W[:, i])[0, 1] for i in (0, 1)])
    print(f"{t} n={len(Y)}  R2@release GRU {r2(PG, Y):.3f}  HAND {r2(PM, Y):.3f} | corr(pred, true wind) GRU {c(PG):.3f} HAND {c(PM):.3f} | corr(label, wind) {c(Y):.3f} | corr(GRU,HAND) {c.__call__(PG) and np.mean([np.corrcoef(PG[:, i], PM[:, i])[0, 1] for i in (0, 1)]):.3f}")
