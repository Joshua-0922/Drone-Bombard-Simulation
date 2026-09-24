"""Paper figures for the adopted method (GRU-S), pooled over eval seeds {3000,4000,5000}.

  isaaclab.sh -p _fig_final.py --out /tmp/figs
Figures:
  fig1_speed_accuracy.png   delivery time vs CEP50 per arm (bar to CEP90), constant wind
  fig2_dr_sweep.png         CEP50 vs model-error scale: T2 (tuned), L0, ours
  fig3_wind.png             CEP50 vs wind condition: constant / realistic A / realistic B / stress tau ladder
Colours: validated categorical palette (dataviz reference), fixed per entity.
"""
import argparse, glob, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

p = argparse.ArgumentParser(); p.add_argument("--out", default="/tmp/figs"); a = p.parse_args()
os.makedirs(a.out, exist_ok=True)
SEEDS = {3000, 4000, 5000}
COL = {"ours": "#2a78d6", "T2": "#eb6834", "L0": "#1baf7a", "T0": "#e87ba4", "oracle": "#7a7975", "gi": "#4a3aa7"}
TXT, TXT2, GRID = "#0b0b0b", "#52514e", "#e6e5e1"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": TXT2, "axes.labelcolor": TXT, "xtick.color": TXT2,
                     "ytick.color": TXT2, "axes.spines.top": False, "axes.spines.right": False,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})


def pool(pattern):
    files = sorted(f for g in pattern.split(",") for f in glob.glob(g))
    assert files, pattern
    seeds = {int(f.rsplit("_s", 1)[1][:-5]) for f in files}
    assert seeds == SEEDS, (pattern, seeds)
    e, t, land = [], [], []
    for f in files:
        r = json.load(open(f))["episodes"]
        m = np.asarray(r["landed"], bool)
        e.append(np.asarray(r["release_impact_err"], float)[m]); t.append(np.asarray(r["deliver_time_s"], float)[m])
        land.append(m)
    e, t, land = np.concatenate(e), np.concatenate(t), np.concatenate(land)
    return dict(cep50=float(np.median(e)), cep90=float(np.percentile(e, 90)), t=float(t.mean()),
                deliv=float(land.mean()), s05=float((e <= 0.5).sum() / len(land)))


T0 = "/tmp/sweep3/T0hover_alt3.5_s3000.json,/tmp/t_reseed/T0hover_alt3.5_s*.json"
T2 = "/tmp/sweep3/T2p15_alt3.5_s3000.json,/tmp/t_reseed/T2p15_alt3.5_s*.json"
ARMS1 = [("T0 hover-drop", "T0", T0), ("T2 rule release", "T2", T2), ("L0 RL flight", "L0", "/tmp/sl_GI/L0_dr1.5_s*.json"),
         ("Ours (L0 + GRU residual)", "ours", "/tmp/gru/GRUS_E_tau0_s*.json"),
         ("Oracle (true wind)", "oracle", "/tmp/ou/ORCwindE_tau0_s*.json")]

# ---- fig 1: speed-accuracy plane ----
fig, ax = plt.subplots(figsize=(5.2, 3.6), dpi=200)
for name, key, pat in ARMS1:
    d = pool(pat)
    ax.plot([d["t"], d["t"]], [d["cep50"], d["cep90"]], color=COL[key], lw=2, alpha=0.5, solid_capstyle="round")
    ax.plot(d["t"], d["cep50"], "o", ms=8, color=COL[key], mec="#fcfcfb", mew=1.5)
    off = {"T0": (-6, 10, "right"), "T2": (8, -4, "left"), "L0": (10, 14, "left"),
           "ours": (10, -2, "left"), "oracle": (10, -16, "left")}[key]
    ax.annotate(name, (d["t"], d["cep50"]), xytext=off[:2], textcoords="offset points", ha=off[2],
                color=TXT, fontsize=8, fontweight="bold" if key == "ours" else "normal")
ax.set_xlabel("Delivery time [s]  (lower is faster)"); ax.set_ylabel("Landing error [m]  (dot = CEP50, bar to CEP90)")
ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True); ax.set_xlim(5.0, 9.6); ax.set_ylim(0, 0.95)
ax.set_title("Constant wind, model-error scale 1.5, n = 600 paired scenarios", color=TXT2, fontsize=8, loc="left")
fig.tight_layout(); fig.savefig(f"{a.out}/fig1_speed_accuracy.png"); plt.close(fig)

# ---- fig 2: DR sweep ----
DR = ["0", "0.5", "1.0", "1.5", "2.5"]
def sweep(fmt):
    return [pool(fmt.format(D=D))["cep50"] for D in DR]
series = [("T2 rule release (tuned)", "T2", lambda D: (f"/tmp/dr_axis/T2p15tuned_dr{D}_s*.json" if D not in ("1.5", "2.5")
           else (T2 if D == "1.5" else "/tmp/sweep3/T2p15_D_alt3.5_s3000.json,/tmp/t_reseed/T2p15_D_alt3.5_s*.json"))),
          ("L0 RL flight", "L0", lambda D: {"1.5": "/tmp/sl_GI/L0_dr1.5_s*.json", "2.5": "/tmp/sleval/L0_dr2.5_s*.json"}.get(D, f"/tmp/dr_axis/L0_dr{D}_s*.json")),
          ("Ours (L0 + GRU residual)", "ours", lambda D: f"/tmp/dr_axis/GRUS_dr{D}_s*.json" if D != "1.5" else "/tmp/gru/GRUS_E_tau0_s*.json")]
fig, ax = plt.subplots(figsize=(5.2, 3.4), dpi=200)
x = [float(D) for D in DR]
for name, key, fn in series:
    try:
        y = [pool(fn(D))["cep50"] for D in DR]
    except AssertionError as ex:
        print("skip", name, ex); continue
    ax.plot(x, y, "-o", color=COL[key], lw=2, ms=6, mec="#fcfcfb", mew=1.2, label=name)
    ax.annotate(name, (x[-1], y[-1]), xytext=(5, 0), textcoords="offset points", color=TXT, fontsize=8, va="center")
ax.axvline(1.5, color=GRID, lw=1, ls="--"); ax.text(1.5, 0.02, " training scale", color=TXT2, fontsize=7)
ax.set_xlabel("Model-error scale (wind, ballistic coefficient, release latency)"); ax.set_ylabel("CEP50 [m]")
ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True); ax.set_xlim(-0.1, 3.3); ax.set_ylim(0, 0.55)
ax.set_title("Dose-response: the residual helps only when there is model error to remove", color=TXT2, fontsize=8, loc="left")
fig.tight_layout(); fig.savefig(f"{a.out}/fig2_dr_sweep.png"); plt.close(fig)

# ---- fig 3: wind conditions ----
conds = [("constant", "/tmp/sl_GI/L0_dr1.5_s*.json", "/tmp/gru/GRUS_E_tau0_s*.json", "/tmp/ou/ORCwindE_tau0_s*.json"),
         ("real A\n20% gust\nτ=10 s", "/tmp/gust/L0_A_s*.json", "/tmp/gust/GRUS_A_s*.json", "/tmp/gust/ORC_A_s*.json"),
         ("real B\n30% gust\nτ=3 s", "/tmp/gust/L0_B_s*.json", "/tmp/gust/GRUS_B_s*.json", "/tmp/gust/ORC_B_s*.json")]
for T in ("10", "3", "1", "0.3"):
    conds.append((f"stress\ngust only\nτ={T} s", f"/tmp/ou/L0_tau{T}_s*.json", f"/tmp/gru/GRUS_E_tau{T}_s*.json", f"/tmp/ou/ORCwindE_tau{T}_s*.json"))
fig, ax = plt.subplots(figsize=(7.4, 3.6), dpi=200)
xs = np.arange(len(conds)); w = 0.27
for i, (name, key, idx) in enumerate([("L0 RL flight", "L0", 1), ("Ours (L0 + GRU residual)", "ours", 2), ("Oracle (true wind)", "oracle", 3)]):
    ys = [pool(c[idx])["cep50"] for c in conds]
    bars = ax.bar(xs + (i - 1) * w, ys, w * 0.92, color=COL[key], label=name, edgecolor="#fcfcfb", lw=0.8)
    for xi, yi in zip(xs, ys):
        ax.text(xi + (i - 1) * w, yi + 0.006, f"{yi:.2f}", ha="center", va="bottom", fontsize=6, color=TXT2)
ax.set_xticks(xs); ax.set_xticklabels([c[0] for c in conds], fontsize=7.5)
ax.set_ylabel("CEP50 [m]"); ax.grid(axis="y", color=GRID, lw=0.6); ax.set_axisbelow(True)
ax.axvline(2.5, color=GRID, lw=1, ls="--")
ax.legend(frameon=False, fontsize=7, loc="upper left")
ax.set_title("Wind conditions: realistic gusts keep the gain; gust-only stress beyond the fall time defeats even the oracle",
             color=TXT2, fontsize=7.5, loc="left")
fig.tight_layout(); fig.savefig(f"{a.out}/fig3_wind.png"); plt.close(fig)
print("figures written to", a.out)
