"""What did the L1-RL residual actually output, checkpoint by checkpoint?

Offline, no simulator: feed the SL dump (observation + TRUE wind-only drift per
step, /tmp/sl/dr1.5_*.npz) through the residual trunk of every saved
checkpoint and score it exactly like a regressor -- mean |output|, bias, R^2
against the true drift -- on the aiming frames and on the last 10 carried
frames (where the release happens). The supervised regressor (res_gi.pt) is
scored on the same frames for reference, and so is "output = 0" (L0).

  isaaclab.sh -p _probe_rl_residual.py --run <log_dir> --npz /tmp/sl/dr1.5_s1a.npz --sl /tmp/sl/res_gi.pt
"""
import argparse
import glob
import importlib.util
import os
import re

import numpy as np
import torch

p = argparse.ArgumentParser()
p.add_argument("--run", required=True, help="training log dir holding model_*.pt")
p.add_argument("--npz", nargs="+", required=True)
p.add_argument("--sl", default=None, help="TorchScript regressor for reference")
p.add_argument("--scale", type=float, default=2.0, help="residual.scale the run trained with")
a = p.parse_args()

_spec = importlib.util.spec_from_file_location(
    "residual_actor", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "drone_bombard", "residual_actor.py"))
ra = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ra)

I_VX, I_VY, I_ROLL, I_PITCH, I_SYAW, I_CYAW, I_PA0, I_PA1 = 6, 7, 9, 10, 11, 12, 21, 22


def tilt_gi(obs, new):
    """Same causal prefix means as _fit_sl_residual.tilt_features(want_gi=True)."""
    roll, pitch, sy, cy = obs[..., I_ROLL], obs[..., I_PITCH], obs[..., I_SYAW], obs[..., I_CYAW]
    v = np.stack([roll, pitch, roll * cy, roll * sy, pitch * cy, pitch * sy,
                  obs[..., I_VX], obs[..., I_VY], obs[..., I_PA0], obs[..., I_PA1]], -1).astype(np.float32)
    T, N, C = v.shape
    out, gi = np.empty_like(v), np.zeros((T, N, 2), np.float32)
    acc, cnt, v0 = np.zeros((N, C), np.float32), np.zeros((N, 1), np.float32), np.zeros((N, 2), np.float32)
    vxy = np.stack([obs[..., I_VX], obs[..., I_VY]], -1).astype(np.float32)
    for t in range(T):
        r = new[t]
        acc[r] = 0.0; cnt[r] = 0.0; v0[r] = vxy[t][r]
        acc += v[t]; cnt += 1.0
        out[t] = acc / cnt; gi[t] = (vxy[t] - v0) / cnt
    return np.concatenate([out, gi], -1)


X, Y, AIM, REL = [], [], [], []
for f in a.npz:
    d = np.load(f)
    obs, drift, ep_len, att, det = d["obs"], d["drift"], d["ep_len"], d["attached"], d["detected"]
    T, N = ep_len.shape
    new = np.ones((T, N), bool); new[1:] = ep_len[1:] <= ep_len[:-1]
    x = np.concatenate([obs, tilt_gi(obs, new)], -1)
    carried = att > 0.5
    last = np.zeros_like(carried); left = np.zeros(N, np.int32)
    for t in range(T - 1, -1, -1):          # distance to release, counted backwards
        left = np.where(new[t], 0, left)
        m = carried[t]
        last[t] = m & (left < 10)
        left = np.where(m, left + 1, left)
        left = np.where(new[t], 0, left)
    X.append(x.reshape(-1, x.shape[-1])); Y.append(drift.reshape(-1, 2))
    AIM.append((carried & (det > 0.5)).reshape(-1)); REL.append(last.reshape(-1))
X, Y = np.concatenate(X), np.concatenate(Y)
AIM, REL = np.concatenate(AIM), np.concatenate(REL)
print(f"frames {len(X)}  aiming {AIM.sum()}  release(last10) {REL.sum()}  "
      f"true drift |d| mean: aiming {np.linalg.norm(Y[AIM], axis=1).mean():.3f} m  "
      f"release {np.linalg.norm(Y[REL], axis=1).mean():.3f} m")


def score(pred, mask):
    y, q = Y[mask], pred[mask]
    sst = ((y - y.mean(0)) ** 2).sum()
    r2 = 1 - ((q - y) ** 2).sum() / sst
    mag = np.linalg.norm(q, axis=1).mean()
    bias = q.mean(0)
    # cosine between prediction and truth, on frames with a real drift
    big = np.linalg.norm(y, axis=1) > 0.1
    cos = (q[big] * y[big]).sum(1) / (np.linalg.norm(q[big], axis=1) * np.linalg.norm(y[big], axis=1) + 1e-9)
    return r2, mag, bias, np.nanmean(cos)


Xt = torch.as_tensor(X)
rows = [("output = 0 (L0)", np.zeros_like(Y))]
if a.sl:
    m = torch.jit.load(a.sl, map_location="cpu").eval()
    with torch.no_grad():
        rows.append(("SL res_gi.pt", m(Xt).numpy()))
cks = sorted(glob.glob(os.path.join(a.run, "model_*.pt")),
             key=lambda f: int(re.search(r"model_(\d+)", f)[1]) if re.search(r"model_(\d+)", f) else 10**9)
for ck in cks:
    sd = torch.load(ck, map_location="cpu", weights_only=False)["model_state_dict"]
    if not any(k.startswith("actor.residual.") for k in sd):
        continue
    net = ra.mlp_from_state_dict(sd, "actor.residual.").eval()
    with torch.no_grad():
        pred = torch.clamp(net(Xt), -1.0, 1.0).numpy() * a.scale   # metres, as _ccip applies it
    std = sd["std"][5:7].mean().item()
    rows.append((f"RL {os.path.basename(ck)[:-3]} (std {std:.3f})", pred))

print(f"\n{'arm':32s} | {'R2 aim':>7s} {'|out| aim':>9s} {'cos aim':>7s} | {'R2 rel':>7s} {'|out| rel':>9s} {'bias rel (x,y)':>16s} {'cos rel':>7s}")
for name, pred in rows:
    r2a, ma, _, ca = score(pred, AIM)
    r2r, mr, br, cr = score(pred, REL)
    print(f"{name:32s} | {r2a:7.3f} {ma:9.3f} {ca:7.2f} | {r2r:7.3f} {mr:9.3f} ({br[0]:+.3f},{br[1]:+.3f}) {cr:7.2f}")
