"""Which channel group actually carries the drift? Leave-one-group-out refit.

The R2 ladder (obs -> +tilt -> +gi -> +wind) answers "does adding this help",
which is an ADD-one design and therefore order-dependent. This is the drop-one
complement: refit the full 38-channel model with one group removed and report
the loss. A group whose removal costs nothing is redundant no matter how well
it correlates with the label on its own.
"""
import sys
import numpy as np
import torch

I_VX, I_VY, I_ROLL, I_PITCH, I_SYAW, I_CYAW = 6, 7, 9, 10, 11, 12
I_PA0, I_PA1 = 21, 22

OBS, TILT, GI, DR, EPI = [], [], [], [], []
ep_base = 0
for f in sys.argv[1:]:
    d = np.load(f)
    obs, drift = d["obs"], d["drift"]
    ep_len, att = d["ep_len"], d["attached"]
    T, N = ep_len.shape
    new = np.ones((T, N), bool); new[1:] = ep_len[1:] <= ep_len[:-1]
    epi = np.cumsum(new, 0) - 1 + ep_base + np.arange(N) * T; ep_base += N * T
    roll, pitch = obs[..., I_ROLL], obs[..., I_PITCH]
    sy, cy = obs[..., I_SYAW], obs[..., I_CYAW]
    v = np.stack([roll, pitch, roll*cy, roll*sy, pitch*cy, pitch*sy,
                  obs[..., I_VX], obs[..., I_VY], obs[..., I_PA0], obs[..., I_PA1]], -1)
    vxy = np.stack([obs[..., I_VX], obs[..., I_VY]], -1)
    out = np.empty_like(v); gi = np.zeros((T, N, 2), np.float32)
    acc = np.zeros((N, 10), np.float32); cnt = np.zeros((N, 1), np.float32)
    v0 = np.zeros((N, 2), np.float32)
    for t in range(T):
        r = new[t]; acc[r] = 0; cnt[r] = 0; v0[r] = vxy[t][r]
        acc += v[t]; cnt += 1
        out[t] = acc / cnt; gi[t] = (vxy[t] - v0) / cnt
    m = att > 0.5
    OBS.append(obs[m]); TILT.append(out[m]); GI.append(gi[m])
    DR.append(drift[m]); EPI.append(epi[m])

X = np.concatenate([np.concatenate(OBS), np.concatenate(TILT),
                    np.concatenate(GI)], axis=1).astype(np.float32)
Y = np.concatenate(DR).astype(np.float32)
_, E = np.unique(np.concatenate(EPI), return_inverse=True)
n_ep = E.max() + 1

rng = np.random.default_rng(0)
te_ep = np.zeros(n_ep, bool); te_ep[rng.permutation(n_ep)[: n_ep // 3]] = True
te, tr = te_ep[E], ~te_ep[E]
ybar = Y[tr].mean(0)
Xs = ((X - X[tr].mean(0)) / (X[tr].std(0) + 1e-6)).astype(np.float32)
dev = "cuda" if torch.cuda.is_available() else "cpu"
print(f"frames {len(X)}  episodes {n_ep}  channels {X.shape[1]}")

# 38 = obs[0:26] + tilt[26:36] + gi[36:38]
GROUPS = {
    "기하 (표적·CCIP오차·낙하시간·고도)": [0, 1, 2, 3, 4, 5, 16, 18],
    "자세 (roll·pitch·yaw)":            [9, 10, 11, 12],
    "속도 (vx·vy·vz·speed_xy)":         [6, 7, 8, 17],
    "각속도":                            [13, 14, 15],
    "지령 (prev_action)":                [21, 22, 23, 24],
    "tilt 누적 (10ch)":                  list(range(26, 36)),
    "gi 접두가속 (2ch)":                 [36, 37],
}

def fit(cols):
    torch.manual_seed(0)
    net = torch.nn.Sequential(torch.nn.Linear(len(cols), 128), torch.nn.ELU(),
                              torch.nn.Linear(128, 128), torch.nn.ELU(),
                              torch.nn.Linear(128, 2)).to(dev)
    xt = torch.as_tensor(Xs[tr][:, cols], device=dev)
    yt = torch.as_tensor(Y[tr], device=dev)
    opt = torch.optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, 300)
    for _ in range(300):
        idx = torch.randperm(len(xt), device=dev)
        for i in range(0, len(idx), 4096):
            b = idx[i:i+4096]
            opt.zero_grad()
            torch.nn.functional.mse_loss(net(xt[b]), yt[b]).backward()
            opt.step()
        sch.step()
    with torch.no_grad():
        pr = net(torch.as_tensor(Xs[te][:, cols], device=dev)).cpu().numpy()
    return 1 - ((pr - Y[te])**2).sum() / ((Y[te] - ybar)**2).sum()

allc = list(range(38))
full = fit(allc)
print(f"\n전체 38채널  R2 = {full:.3f}\n")
print(f"{'뺀 그룹':36s} {'R2':>7s} {'손실 ΔR2':>10s}")
print("-" * 56)
rows = []
for name, cols in GROUPS.items():
    keep = [c for c in allc if c not in cols]
    r = fit(keep)
    rows.append((name, r, full - r))
for name, r, d in sorted(rows, key=lambda x: -x[2]):
    print(f"{name:36s} {r:7.3f} {d:10.3f}")
