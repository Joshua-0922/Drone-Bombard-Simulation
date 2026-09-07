"""What does the residual regression actually correlate with?

R2 says how much of the drift is predictable; it does not say WHICH channel
carries it. This prints the per-channel Pearson correlations behind that R2.
"""
import numpy as np, sys

I_VX, I_VY, I_ROLL, I_PITCH, I_SYAW, I_CYAW = 6, 7, 9, 10, 11, 12
I_PA0, I_PA1 = 21, 22
NAME = ["rel_x", "rel_y", "ccip_err_x", "ccip_err_y", "d_impact",
        "-alt", "vx", "vy", "vz", "roll", "pitch", "sin_yaw", "cos_yaw",
        "angvel_x", "angvel_y", "angvel_z", "t_fall", "speed_xy", "alt",
        "attached", "time_left", "act_vx", "act_vy", "act_vz", "act_drop", "detected"]

OBS, TILT, GI, DR, W = [], [], [], [], []
for f in sys.argv[1:]:
    d = np.load(f)
    obs, drift, wind = d["obs"], d["drift"], d["wind"]
    ep_len, att = d["ep_len"], d["attached"]
    T, N = ep_len.shape
    new = np.ones((T, N), bool); new[1:] = ep_len[1:] <= ep_len[:-1]
    roll, pitch = obs[..., I_ROLL], obs[..., I_PITCH]
    sy, cy = obs[..., I_SYAW], obs[..., I_CYAW]
    v = np.stack([roll, pitch, roll*cy, roll*sy, pitch*cy, pitch*sy,
                  obs[..., I_VX], obs[..., I_VY], obs[..., I_PA0], obs[..., I_PA1]], -1)
    vxy = np.stack([obs[..., I_VX], obs[..., I_VY]], -1)
    out = np.empty_like(v); gi = np.zeros((T, N, 2), np.float32)
    acc = np.zeros((N, v.shape[-1]), np.float32); cnt = np.zeros((N, 1), np.float32)
    v0 = np.zeros((N, 2), np.float32)
    for t in range(T):
        r = new[t]; acc[r] = 0; cnt[r] = 0; v0[r] = vxy[t][r]
        acc += v[t]; cnt += 1
        out[t] = acc / cnt; gi[t] = (vxy[t] - v0) / cnt
    m = att > 0.5
    OBS.append(obs[m]); TILT.append(out[m]); GI.append(gi[m]); DR.append(drift[m]); W.append(wind[m])

X, Tl, G = np.concatenate(OBS), np.concatenate(TILT), np.concatenate(GI)
Y, Wd = np.concatenate(DR), np.concatenate(W)

def corr(a, b):
    a = a - a.mean(); b = b - b.mean()
    s = a.std() * b.std()
    return float((a * b).mean() / s) if s > 1e-12 else 0.0

print(f"frames {len(X)}")
print(f"|drift| mean {np.linalg.norm(Y,1 if False else None,axis=1).mean():.3f} m   "
      f"|wind| mean {np.linalg.norm(Wd[:,:2],axis=1).mean():.2f} m/s")

print("\n=== 바람 -> 드리프트 (물리 그 자체) ===")
for k, ax in enumerate("xy"):
    print(f"  corr(wind_{ax}, drift_{ax}) = {corr(Wd[:,k], Y[:,k]):+.3f}   "
          f"기울기 {np.polyfit(Wd[:,k], Y[:,k],1)[0]:+.4f} m per m/s")
print(f"  corr(|wind|, |drift|)      = {corr(np.linalg.norm(Wd[:,:2],axis=1), np.linalg.norm(Y,axis=1)):+.3f}")

print("\n=== 관측 26채널 vs 드리프트 (|corr| 상위 8) ===")
rows = [(NAME[i], corr(X[:,i], Y[:,0]), corr(X[:,i], Y[:,1])) for i in range(X.shape[1])]
for n, cx, cy_ in sorted(rows, key=lambda r: -max(abs(r[1]), abs(r[2])))[:8]:
    print(f"  {n:12s} drift_x {cx:+.3f}   drift_y {cy_:+.3f}")

print("\n=== tilt 누적 10채널 vs 드리프트 ===")
TN = ["roll_bar","pitch_bar","roll*cy","roll*sy","pitch*cy","pitch*sy",
      "vx_bar","vy_bar","act_vx_bar","act_vy_bar"]
for i, n in enumerate(TN):
    print(f"  {n:12s} drift_x {corr(Tl[:,i], Y[:,0]):+.3f}   drift_y {corr(Tl[:,i], Y[:,1]):+.3f}")

print("\n=== gi 2채널 (접두 평균 가속) vs 드리프트 ===")
for i, n in enumerate(["gi_ax", "gi_ay"]):
    print(f"  {n:12s} drift_x {corr(G[:,i], Y[:,0]):+.3f}   drift_y {corr(G[:,i], Y[:,1]):+.3f}")

print("\n=== tilt 누적 vs 바람 (환산표가 재는 것) ===")
for i in (2,3,4,5):
    print(f"  {TN[i]:12s} wind_x {corr(Tl[:,i], Wd[:,0]):+.3f}   wind_y {corr(Tl[:,i], Wd[:,1]):+.3f}")
