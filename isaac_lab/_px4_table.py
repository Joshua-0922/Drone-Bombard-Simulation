"""P-trained L0 / L0+GRU-S evaluated under the P loop vs the PX4 v1.15.4 velocity PID (no retraining).
stdlib only:  python3 isaac_lab/_px4_table.py ROOT   (ROOT holds px4e/, sl_GI/, gru/, gust/ eval dirs)
CIs: unpaired bootstrap of the CEP50 difference (episode order is not aligned across all files)."""
import json, random, statistics as st, sys
R = sys.argv[1]; SEEDS = (3000, 4000, 5000); rng = random.Random(0)
def load(fmt):
    e, l, c, lab = [], [], [], None
    for s in SEEDS:
        r = json.load(open(R + "/" + fmt.format(S=s))); ep = r["episodes"]; lab = r["cause_labels"]
        e += ep["release_impact_err"]; l += ep["landed"]; c += ep["cause"]
    return [x for x, k in zip(e, l) if k], l, c, lab
def q(v, f): v = sorted(v); return v[min(len(v) - 1, int(f * len(v)))]
def boot(a, b, n=2000):
    d = sorted(st.median(rng.choices(a, k=len(a))) - st.median(rng.choices(b, k=len(b))) for _ in range(n))
    return d[int(.025 * n)], d[int(.975 * n)]
P = {("L0", "tau0"): "sl_GI/L0_dr1.5_s{S}.json", ("GRUS", "tau0"): "gru/GRUS_E_tau0_s{S}.json",
     ("L0", "A"): "gust/L0_A_s{S}.json", ("GRUS", "A"): "gust/GRUS_A_s{S}.json",
     ("L0", "B"): "gust/L0_B_s{S}.json", ("GRUS", "B"): "gust/GRUS_B_s{S}.json"}
NAME = {"tau0": "정상 바람", "A": "현실 A (20%, τ10)", "B": "현실 B (30%, τ3)"}
def row(tag, cond):
    out = {}
    for arm in ("L0", "GRUS"):
        p = load(P[(arm, cond)]); x = load(f"px4e/{arm}_px4{tag}_{cond}_s{{S}}.json")
        out[arm] = (p, x)
    return out
def fmt(d):
    e, l, c, lab = d
    ba = c.count(lab.index("bad_attitude")) if "bad_attitude" in lab else 0
    return f"{st.median(e):.3f} / {q(e,.9):.3f} / {100*sum(x<=.5 for x in e)/len(l):.1f}% / 착지 {100*sum(l)/len(l):.1f}% / 자세종료 {ba}"
for tag, title in (("", "자세 각속도 한계 2 rad/s (기존 표와 동일)"), ("_av6", "자세 각속도 한계 6 rad/s")):
    print(f"\n### {title}\n")
    print("| 조건 | 팔 | P 제어 (학습 조건) | PX4 PID (재학습 없음) | ΔCEP50 PX4−P [95% CI] |\n|---|---|---|---|---|")
    for cond in ("tau0", "A", "B"):
        try: o = row(tag, cond)
        except FileNotFoundError: continue
        for arm in ("L0", "GRUS"):
            p, x = o[arm]; lo, hi = boot(x[0], p[0])
            print(f"| {NAME[cond]} | {'L0' if arm=='L0' else 'L0+GRU-S'} | {fmt(p)} | {fmt(x)} | {st.median(x[0])-st.median(p[0]):+.3f} [{lo:+.3f}, {hi:+.3f}] |")
        g, l0 = o["GRUS"][1][0], o["L0"][1][0]; lo, hi = boot(g, l0)
        print(f"| {NAME[cond]} | **본 방법 이득 (PX4)** | | {100*(st.median(g)-st.median(l0))/st.median(l0):+.0f}% | {st.median(g)-st.median(l0):+.3f} [{lo:+.3f}, {hi:+.3f}] |")
print("\n셀 = CEP50 / CEP90 [m] / succ@0.5 / 착지율 / 자세 종료 수, n = 600 (seed 3000·4000·5000 × 200)")
