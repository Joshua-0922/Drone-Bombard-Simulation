"""Analyse a motor_step.py recording: command->PWM latency and rotor-speed step response.

    python analyze_motor_step.py data/run3_motor2 [--boot 30]

Method (short-window pitch tracking failed at this mic SNR, so the band power is used):
  * timeline: audio sample k at  median(cb_time - (block_index+1)*BLOCK/fs) - stream_latency + k/fs
    (Windows input latency is only known to ~+-20 ms, so dead times carry that offset error;
    time constants do not);
  * PWM step times on the host clock from SERVO_OUTPUT_RAW time_usec + min(host_rx - fc_time);
  * steady rotor speed from the blade-pass line (1 s windows, f0 + 2nd harmonic, 2-blade prop);
  * band power (120-2000 Hz) in a 1024-sample Hann window, averaged over all steps of a direction,
    mapped to rotor speed with loudness ~ rpm^k (k from the two steady levels);
  * fit of dead time + first-order lag, convolved with the analysis window; bootstrap over steps.
"""
import argparse, json, wave

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('prefix')
ap.add_argument('--boot', type=int, default=30)
args = ap.parse_args()

M = json.load(open(args.prefix + '_meta.json'))
with wave.open(args.prefix + '_audio.wav', 'rb') as w:
    fs = w.getframerate()
    a = np.frombuffer(w.readframes(w.getnframes()), '<i2').astype(np.float64) / 32767
B = M['block']
cb = np.array(M['block_cb_time']); nb = len(cb)
t = np.median(cb - (np.arange(nb) + 1) * B / fs) - M['stream_latency'] + np.arange(len(a)) / fs

pwm = np.array(M['pwm'], dtype=float)
tp = pwm[:, 1] * 1e-6 + np.min(pwm[:, 0] - pwm[:, 1] * 1e-6); v = pwm[:, 2]
cmds = [e[1] for e in M['events'] if e[0] == 'cmd']
steps = []
for i in np.where(np.diff(v) != 0)[0]:
    if abs(v[i + 1] - v[i]) > 100 and v[i] > 1200 and v[i + 1] > 1200:
        tm = (tp[i] + tp[i + 1]) / 2
        steps.append((tm, int(v[i]), int(v[i + 1]), tm - [c for c in cmds if c <= tm + 0.03][-1]))
lat_c = np.array([s[3] for s in steps]) * 1e3
print(f"motor {M.get('motor')}  battery {M['battery_mV']} mV  steps {len(steps)}  PWM {steps[0][1]}<->{steps[0][2]} us")
print(f"cmd->PWM: median {np.median(lat_c):.1f} ms (min {lat_c.min():.1f}, max {lat_c.max():.1f}), "
      f"PWM sample interval {np.median(np.diff(pwm[:, 1])) * 1e-3:.1f} ms")

def f0_seg(t0, t1):
    i = (t >= t0) & (t < t1); n = 1 << 18
    S = np.abs(np.fft.rfft(a[i] * np.hanning(i.sum()), n)); ff = np.fft.rfftfreq(n, 1 / fs)
    c = np.arange(120, 240, 0.1)
    return c[np.argmax(np.interp(c, ff, S) + np.interp(2 * c, ff, S))]
f_lo = np.median([f0_seg(s[0] - 1.1, s[0] - 0.1) for s in steps if s[2] > s[1]])
f_hi = np.median([f0_seg(s[0] - 1.1, s[0] - 0.1) for s in steps if s[2] < s[1]])
ratio = f_hi / f_lo
print(f"steady blade-pass {f_lo:.1f} / {f_hi:.1f} Hz -> {f_lo * 30:.0f} / {f_hi * 30:.0f} rpm (2-blade)")

X = np.fft.rfft(a); f = np.fft.rfftfreq(len(a), 1 / fs); X[(f < 120) | (f > 2000)] = 0
ab = np.fft.irfft(X, len(a))
N = 1024; win = np.hanning(N); win /= win.sum()
rel = np.arange(-0.25, 0.6, 48 / fs)
def env(t0):
    return np.array([np.sum(win * ab[(j := np.searchsorted(t, t0 + q) - N // 2):j + N] ** 2) for q in rel])

taus = np.arange(0.005, 0.30, 0.005)
offs = (np.arange(N)[::32] - N / 2) / fs; wk = win[::32] / win[::32].sum()
fitm = (rel > -0.15) & (rel < 0.5) & (np.arange(len(rel)) % 2 == 0)
tt = rel[fitm][:, None] + offs[None, :]
pre_m, post_m = rel < -0.08, (rel > 0.4) & (rel < 0.58)

def normalized(E, lo, k):
    rpm = (np.sqrt(E) / lo) ** (1 / k)
    return (rpm - rpm[pre_m].mean()) / (np.median(rpm[post_m]) - rpm[pre_m].mean())

def fit(y):
    best = (np.inf, 0.0, 0.0)
    for d in np.arange(-0.03, 0.12, 0.002):
        z = np.clip(tt - d, 0, None)[None]
        pred = (np.where(z > 0, 1 - np.exp(-z / taus[:, None, None]), 0) * wk).sum(2)
        e = ((pred - y[fitm]) ** 2).sum(1); j = np.argmin(e)
        if e[j] < best[0]:
            best = (e[j], d, taus[j])
    return best

rng = np.random.default_rng(0)
for key in ('UP', 'DOWN'):
    envs = [env(s[0]) for s in steps if (s[2] > s[1]) == (key == 'UP')]
    r = np.sqrt(np.mean(envs, axis=0))
    pre, post = np.median(r[pre_m]), np.median(r[post_m])
    lo, hi = (pre, post) if key == 'UP' else (post, pre)
    k = np.log(hi / lo) / np.log(ratio)
    y = normalized(np.mean(envs, axis=0), lo, k)
    sse, d, tau = fit(y)
    ys = np.convolve(y, np.ones(9) / 9, 'same')
    cross = lambda L: rel[np.where((rel > -0.03) & (ys >= L))[0][0]] * 1e3
    boot = [fit(normalized(np.mean([envs[i] for i in rng.integers(0, len(envs), len(envs))], axis=0), lo, k))[1:]
            for _ in range(args.boot)]
    bd, bt = np.array(boot).T * 1e3
    print(f"{key:4s} n={len(envs)} k={k:.2f} | dead {d * 1e3:.0f} ms (90% {np.percentile(bd, 5):.0f}..{np.percentile(bd, 95):.0f})"
          f"  tau {tau * 1e3:.0f} ms (90% {np.percentile(bt, 5):.0f}..{np.percentile(bt, 95):.0f})"
          f"  rmse {np.sqrt(sse / fitm.sum()):.2f} | t50 {cross(.5):.0f} ms  t90 {cross(.9):.0f} ms")
