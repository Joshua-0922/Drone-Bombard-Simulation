"""_extract_traincurve.py — pull per-iteration Episode_* stats out of an
rsl_rl console log and print windowed means (exp_014 R_alt judgment).

Usage: python3 _extract_traincurve.py <train.log> [win_start win_end]
"""

import re
import sys

KEYS = (
    "Episode_Termination/max_altitude",
    "Episode_Termination/success",
    "Episode_Termination/crash",
    "Episode_Termination/bad_attitude",
    "Episode_Termination/stagnation",
    "Episode_Termination/timeout",
    "Episode_Reward/rew_vision",
    "Episode_Metric/d_xy_min",
    "Episode_Metric/action_sat_frac",
)


def main(path, w0=300, w1=400):
    txt = open(path, errors="ignore").read()
    txt = re.sub(r"\x1b\[[0-9;]*m", "", txt)
    cur, series = None, {k: {} for k in KEYS}
    noise = {}
    for line in txt.splitlines():
        m = re.search(r"Learning iteration (\d+)/", line)
        if m:
            cur = int(m.group(1))
            continue
        if cur is None:
            continue
        m = re.search(r"Mean action noise std:\s*([\d.]+)", line)
        if m:
            noise[cur] = float(m.group(1))
        for k in KEYS:
            if k + ":" in line:
                m = re.search(re.escape(k) + r":\s*([-\d.]+)", line)
                if m:
                    series[k][cur] = float(m.group(1))

    n_iters = max(series[KEYS[0]].keys(), default=-1)
    print(f"iterations seen: 0..{n_iters}")

    def wmean(d, a, b):
        vals = [v for i, v in d.items() if a <= i <= b]
        return sum(vals) / len(vals) if vals else float("nan")

    print(f"\nwindow means, iters {w0}-{w1}:")
    for k in KEYS:
        print(f"  {k:<42} {wmean(series[k], w0, w1):+.4f}")
    if noise:
        print(f"  {'Mean action noise std':<42} {wmean(noise, w0, w1):+.4f}")

    # emergence trace for max_altitude in 50-iter buckets
    d = series["Episode_Termination/max_altitude"]
    print("\nmax_altitude by 50-iter bucket:")
    for a in range(0, (n_iters // 50 + 1) * 50, 50):
        print(f"  {a:>4}-{min(a + 49, n_iters):>4}: {wmean(d, a, a + 49):.4f}")


if __name__ == "__main__":
    args = sys.argv[1:]
    main(args[0], *(int(x) for x in args[1:3]))
