"""Why does an accurate-on-paper residual make the drop WORSE?

Hypothesis: the release gate takes the FIRST crossing of the corrected impact
prediction over the target. A prediction that is right ON AVERAGE but jittery
FROM STEP TO STEP crosses early -- the gate samples the extreme of the jitter,
not its mean. Accuracy (RMSE) and temporal smoothness are different properties
and only the oracle has both.

Measures, over the last K carried frames of each episode:
  RMSE(delta_hat, delta)                 how accurate
  step-to-step |delta[t]-delta[t-1]|     how smooth  (truth vs prediction)
and what an EMA over policy steps does to each.
"""
import sys
import numpy as np
import torch

npz, model = sys.argv[1:3]
K = 10
d = np.load(npz)
obs, drift, att = d["obs"], d["drift"], d["attached"]
ep_len = d["ep_len"]
T, N = ep_len.shape
new = np.ones((T, N), bool)
new[1:] = ep_len[1:] <= ep_len[:-1]

I = dict(VX=6, VY=7, ROLL=9, PITCH=10, SYAW=11, CYAW=12, PA0=21, PA1=22)
roll, pitch, sy, cy = (obs[..., I["ROLL"]], obs[..., I["PITCH"]],
                       obs[..., I["SYAW"]], obs[..., I["CYAW"]])
v = np.stack([roll, pitch, roll * cy, roll * sy, pitch * cy, pitch * sy,
              obs[..., I["VX"]], obs[..., I["VY"]],
              obs[..., I["PA0"]], obs[..., I["PA1"]]], -1).astype(np.float32)
tilt = np.empty_like(v)
acc = np.zeros((N, v.shape[-1]), np.float32); cnt = np.zeros((N, 1), np.float32)
for t in range(T):
    r = new[t]; acc[r] = 0; cnt[r] = 0
    acc += v[t]; cnt += 1; tilt[t] = acc / cnt

m = torch.jit.load(model, map_location="cpu").eval()
n_in = int(m.mu.numel())
X = np.concatenate([obs, tilt], -1) if n_in > obs.shape[-1] else obs
with torch.no_grad():
    pred = m(torch.as_tensor(X.reshape(-1, n_in))).numpy().reshape(T, N, 2)

# EMA over policy steps, reset per episode
for alpha in (1.0, 0.5, 0.3):
    ema = np.empty_like(pred)
    st = np.zeros((N, 2), np.float32)
    for t in range(T):
        st = np.where(new[t][:, None], pred[t], alpha * pred[t] + (1 - alpha) * st)
        ema[t] = st
    carried = att > 0.5
    sel = np.zeros_like(carried); left = np.zeros(N, np.int32)
    for t in range(T - 1, -1, -1):
        if t + 1 < T:
            left = np.where(new[t + 1], 0, left)
        left = np.where(carried[t], left + 1, 0)
        sel[t] = carried[t] & (left <= K)

    def jitter(a):
        """mean |a[t] - a[t-1]| over consecutive selected frames of one episode"""
        dd = np.linalg.norm(a[1:] - a[:-1], axis=-1)
        ok = sel[1:] & sel[:-1] & ~new[1:]
        return float(dd[ok].mean())

    err = np.linalg.norm(ema[sel] - drift[sel], axis=-1)
    print(f"alpha={alpha:<4}  RMSE {np.sqrt((err**2).mean()):.3f} m   "
          f"jitter(pred) {jitter(ema):.3f} m/step   "
          f"jitter(truth) {jitter(drift):.3f} m/step")
print(f"\n|drift| RMS over the same frames: "
      f"{np.sqrt((np.linalg.norm(drift[sel],axis=-1)**2).mean()):.3f} m")
