"""L1-SL: can the observation predict the wind-only impact drift?

Target  delta* = real_impact - nominal_impact  (metres, 2-D) -- the exact
quantity a learned residual adds to the CCIP prediction.  A regressor that
cannot beat the constant predictor means the observation is blind to the wind,
and NO residual learner (supervised or RL) can do better than a fixed bias.

Four predictors, in increasing order of what they are allowed to see:

  constant          fits only the mean drift          -> R2 = 0 by construction
  linear(obs)       is the drift a linear read-out of the current observation?
  MLP(obs)          <- THE QUESTION: today's observation, arbitrary function
  MLP(obs + tilt)   plus a CAUSAL episode-running mean of attitude / tracking
                    error -- the proposed tilt channel, simulated offline.
                    If this jumps, L0 must be retrained with the channel.
  MLP(obs + wind)   the TRUE wind handed over. Not deployable; bounds what a
                    perfect wind estimator could ever buy, and separates
                    "the observation is blind" from "the drift is not wind".

Split is BY EPISODE: frames inside one episode share one wind draw, so a random
frame split leaks the label and inflates R2.
"""
import argparse
import numpy as np
import torch

# obs layout (see task_env._get_observations):
#  0-4   rel_x, rel_y, ccip_err_x, ccip_err_y, d_impact   (marker-gated)
#  5-20  -alt, vx, vy, vz, roll, pitch, sin_yaw, cos_yaw, wx, wy, wz,
#        t_fall, speed_xy, alt, attached, time_left
# 21-24  prev_action[:4]      25  detected
I_VX, I_VY, I_ROLL, I_PITCH, I_SYAW, I_CYAW = 6, 7, 9, 10, 11, 12
I_PA0, I_PA1 = 21, 22

p = argparse.ArgumentParser()
p.add_argument("npz", nargs="+")
p.add_argument("--mask", default="attached",
               choices=["attached", "predrop", "all", "last1", "last3", "last5", "last10"],
               help="Which frames the regression is scored on. 'attached' = every frame the "
                    "payload is carried, which OVER-WEIGHTS early high-altitude frames whose "
                    "drift is large and whose release decision never happens. 'lastK' = the K "
                    "frames before release, i.e. the ones the gate actually acts on.")
p.add_argument("--epochs", type=int, default=300)
p.add_argument("--hidden", type=int, default=128)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--export", default=None, metavar="PT",
               help="TorchScript-export the fitted regressor for play.py --sl_residual. "
                    "The module takes the RAW feature vector and returns METRES, so the "
                    "normalisation travels with the weights and cannot drift.")
p.add_argument("--export_arm", default="tilt", choices=["obs", "tilt"])
p.add_argument("--train_ep", type=int, default=0, metavar="N",
               help="Cap the TRAINING set at N episodes (0 = all). The held-out test "
                    "episodes are untouched, so R2 across different N is measured on the "
                    "same test set -- that is what makes the label-count curve readable.")
a = p.parse_args()


def tilt_features(obs, new):
    """Causal per-episode running mean of the channels that carry the wind.

    The instantaneous tilt is ambiguous -- a drone pitches to accelerate as well
    as to hold against wind -- but the manoeuvre part averages out over an
    episode while the wind part does not. Products with sin/cos yaw are included
    so the world-frame direction is recoverable by a LINEAR combination; the
    body-frame means alone are not enough when the heading changes.
    """
    roll, pitch = obs[..., I_ROLL], obs[..., I_PITCH]
    sy, cy = obs[..., I_SYAW], obs[..., I_CYAW]
    # Command and actual velocity are accumulated SEPARATELY, not as a
    # difference: they carry different normalisations (vx_scale 4.0 vs obs /10),
    # so a hard-coded difference fixes the wrong linear combination. Given both
    # means the network can form the right one.
    v = np.stack([
        roll, pitch,
        roll * cy, roll * sy, pitch * cy, pitch * sy,   # -> world-frame tilt
        obs[..., I_VX], obs[..., I_VY],                 # actual velocity
        obs[..., I_PA0], obs[..., I_PA1],               # commanded velocity
    ], axis=-1).astype(np.float32)
    T, N, C = v.shape
    out = np.empty_like(v)
    acc = np.zeros((N, C), np.float32)
    cnt = np.zeros((N, 1), np.float32)
    for t in range(T):                      # T ~ 700; explicit and obviously causal
        r = new[t]
        acc[r] = 0.0
        cnt[r] = 0.0
        acc += v[t]
        cnt += 1.0
        out[t] = acc / cnt
    return out


OBS, TILT, DRIFT, WIND, EPI = [], [], [], [], []
ep_base = 0
for f in a.npz:
    d = np.load(f)
    obs, drift, wind = d["obs"], d["drift"], d["wind"]
    ep_len, att, det = d["ep_len"], d["attached"], d["detected"]
    T, N = ep_len.shape
    new = np.ones((T, N), bool)
    new[1:] = ep_len[1:] <= ep_len[:-1]          # a reset happened in this slot
    epi = np.cumsum(new, axis=0) - 1 + ep_base + np.arange(N) * T
    ep_base += N * T
    tilt = tilt_features(obs, new)
    if a.mask.startswith("last"):
        k = int(a.mask[4:])
        carried = att > 0.5
        # release = the last carried frame in each slot-episode. Mark the K
        # frames ending there: a backward running count of remaining carried
        # frames within the episode.
        # `attached` is a PREFIX of each episode (once released it stays 0), so
        # counting carried frames backwards gives the distance to release.
        m = np.zeros_like(carried)
        left = np.zeros(N, np.int32)
        for t in range(T - 1, -1, -1):
            if t + 1 < T:
                left = np.where(new[t + 1], 0, left)   # this episode ends at t
            left = np.where(carried[t], left + 1, 0)
            m[t] = carried[t] & (left <= k)
    else:
        m = {"attached": att > 0.5,
             "predrop": (att > 0.5) & (det > 0.5),
             "all": np.ones_like(att, bool)}[a.mask]
    OBS.append(obs[m]); TILT.append(tilt[m]); DRIFT.append(drift[m])
    WIND.append(wind[m]); EPI.append(epi[m])

X = np.concatenate(OBS).astype(np.float32)
Tl = np.concatenate(TILT).astype(np.float32)
Y = np.concatenate(DRIFT).astype(np.float32)
W = np.concatenate(WIND).astype(np.float32)
_, E = np.unique(np.concatenate(EPI), return_inverse=True)
n_ep = E.max() + 1
mag = np.linalg.norm(Y, axis=1)
print(f"frames {len(X)}  episodes {n_ep}  obs_dim {X.shape[1]}  mask={a.mask}")
print(f"|drift| mean {mag.mean():.3f} m  p50 {np.median(mag):.3f}  "
      f"p90 {np.percentile(mag, 90):.3f}  max {mag.max():.3f}")
print(f"|wind|  mean {np.linalg.norm(W, axis=1).mean():.2f} m/s")

rng = np.random.default_rng(a.seed)
te_ep = np.zeros(n_ep, bool)
te_ep[rng.permutation(n_ep)[: max(1, n_ep // 3)]] = True
te, tr = te_ep[E], ~te_ep[E]
n_tr_ep = int((~te_ep).sum())
if a.train_ep:
    keep = np.zeros(n_ep, bool)
    keep[rng.permutation(np.flatnonzero(~te_ep))[: a.train_ep]] = True
    tr = keep[E]
    n_tr_ep = int(keep.sum())
print(f"train {tr.sum()} frames / {n_tr_ep} eps    "
      f"test {te.sum()} frames / {te_ep.sum()} eps")

ybar = Y[tr].mean(0)
dev = "cuda" if torch.cuda.is_available() else "cpu"


def std(M):
    return ((M - M[tr].mean(0)) / (M[tr].std(0) + 1e-6)).astype(np.float32)


def report(pred):
    sse = ((pred - Y[te]) ** 2).sum()
    sst = ((Y[te] - ybar) ** 2).sum()
    per = [1 - ((pred[:, k] - Y[te][:, k]) ** 2).sum()
           / ((Y[te][:, k] - ybar[k]) ** 2).sum() for k in (0, 1)]
    return 1 - sse / sst, per, float(np.sqrt(((pred - Y[te]) ** 2).sum(1).mean()))


class _Exported(torch.nn.Module):
    """raw features -> drift in METRES. Normalisation is baked in."""

    def __init__(self, net, mu, sd):
        super().__init__()
        self.net = net
        self.register_buffer("mu", torch.as_tensor(mu))
        self.register_buffer("sd", torch.as_tensor(sd))

    def forward(self, x):
        return self.net((x - self.mu) / self.sd)


def mlp(M, raw=None, export=None):
    torch.manual_seed(a.seed)
    net = torch.nn.Sequential(
        torch.nn.Linear(M.shape[1], a.hidden), torch.nn.ELU(),
        torch.nn.Linear(a.hidden, a.hidden), torch.nn.ELU(),
        torch.nn.Linear(a.hidden, 2)).to(dev)
    xt = torch.as_tensor(M[tr], device=dev)
    yt = torch.as_tensor(Y[tr], device=dev)
    opt = torch.optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    for _ in range(a.epochs):
        idx = torch.randperm(len(xt), device=dev)
        for i in range(0, len(idx), 4096):
            b = idx[i:i + 4096]
            opt.zero_grad()
            torch.nn.functional.mse_loss(net(xt[b]), yt[b]).backward()
            opt.step()
        sch.step()
    if export is not None and raw is not None:
        mod = _Exported(net.cpu().eval(), raw[tr].mean(0), raw[tr].std(0) + 1e-6)
        torch.jit.script(mod).save(export)
        print(f"[export] {export}  in={raw.shape[1]}  out=2 (metres)")
        net.to(dev)
    with torch.no_grad():
        return net(torch.as_tensor(M[te], device=dev)).cpu().numpy()


Xs = std(X)
Ws = std(np.c_[W, np.linalg.norm(W, axis=1, keepdims=True)].astype(np.float32))
Ts = std(Tl)

rows = [("constant (bias only)", np.tile(ybar, (te.sum(), 1)))]
A = np.c_[Xs[tr], np.ones(tr.sum(), np.float32)]
coef, *_ = np.linalg.lstsq(A, Y[tr], rcond=None)
rows.append(("linear(obs)", np.c_[Xs[te], np.ones(te.sum(), np.float32)] @ coef))
XT = np.c_[X, Tl].astype(np.float32)
rows.append(("MLP(obs)                <- today",
             mlp(Xs, raw=X, export=a.export if a.export_arm == "obs" else None)))
rows.append(("MLP(obs + tilt accum)   <- proposed",
             mlp(np.c_[Xs, Ts], raw=XT,
                 export=a.export if a.export_arm == "tilt" else None)))
rows.append(("MLP(obs + TRUE wind)    [ceiling]", mlp(np.c_[Xs, Ws])))

print(f"\n{'predictor':36s} {'R2':>7s} {'R2_x':>7s} {'R2_y':>7s} {'RMSE m':>8s}")
print("-" * 70)
for name, pred in rows:
    R, per, rm = report(pred)
    print(f"{name:36s} {R:7.3f} {per[0]:7.3f} {per[1]:7.3f} {rm:8.3f}")
print(f"\ntest |drift| RMS = {np.sqrt((Y[te] ** 2).sum(1).mean()):.3f} m")
