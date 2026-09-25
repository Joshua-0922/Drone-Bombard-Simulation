"""Final-candidate comparison: GRU-C (consistency loss, no EMA) vs GRU-S + EMA 0.3 vs L0, every table condition.
Stdlib only; run on the host after `docker cp isaac-verify:/tmp <dir>` of the json dirs.
  python3 _final_table.py --root /path/with/{final_C,gru,gust,dr_axis,sl_R,ou,sl_GI}
Paired bootstrap on per-episode error (same seed => same scenarios)."""
import argparse, json, os, random, statistics as st
p = argparse.ArgumentParser(); p.add_argument("--root", required=True); p.add_argument("--boot", type=int, default=2000)
a = p.parse_args(); SEEDS = (3000, 4000, 5000)
COND = [  # name, GRUC file, GRUS+EMA reference, L0 reference   ({S} = seed)
    ("정상 (DR1.5)", "final_C/GRUC_tau0_s{S}", "gru/GRUS_E_tau0_s{S}", "sl_GI/L0_dr1.5_s{S}"),
    ("현실 A 20%/τ10", "final_C/GRUC_A_s{S}", "gust/GRUS_A_s{S}", "gust/L0_A_s{S}"),
    ("현실 B 30%/τ3", "final_C/GRUC_B_s{S}", "gust/GRUS_B_s{S}", "gust/L0_B_s{S}"),
    ("DR 0", "final_C/GRUC_dr0_s{S}", "dr_axis/GRUS_dr0_s{S}", "dr_axis/L0_dr0_s{S}"),
    ("DR 0.5", "final_C/GRUC_dr0.5_s{S}", "dr_axis/GRUS_dr0.5_s{S}", "dr_axis/L0_dr0.5_s{S}"),
    ("DR 1.0", "final_C/GRUC_dr1.0_s{S}", "dr_axis/GRUS_dr1.0_s{S}", "dr_axis/L0_dr1.0_s{S}"),
    ("DR 2.5", "final_C/GRUC_dr2.5_s{S}", "dr_axis/GRUS_dr2.5_s{S}", "sleval/L0_dr2.5_s{S}"),
    ("미지 사거리 26–30", "final_C/GRUC_range_s{S}", "sl_R/GRUS_dr1.5_s{S}", "sl_R/L0_dr1.5_s{S}"),
    ("스트레스 τ10", "final_C/GRUC_tau10_s{S}", "gru/GRUS_E_tau10_s{S}", "ou/L0_tau10_s{S}"),
    ("스트레스 τ3", "final_C/GRUC_tau3_s{S}", "gru/GRUS_E_tau3_s{S}", "ou/L0_tau3_s{S}"),
    ("스트레스 τ1", "final_C/GRUC_tau1_s{S}", "gru/GRUS_E_tau1_s{S}", "ou/L0_tau1_s{S}"),
    ("스트레스 τ0.3", "final_C/GRUC_tau0.3_s{S}", "gru/GRUS_E_tau0.3_s{S}", "ou/L0_tau0.3_s{S}"),
]

def load(fmt):
    err, landed, t = [], [], []
    for S in SEEDS:
        r = json.load(open(os.path.join(a.root, fmt.format(S=S) + ".json")))["episodes"]
        err += r["release_impact_err"]; landed += r["landed"]; t += r["deliver_time_s"]
    return err, landed, t

def q(v, f): v = sorted(v); return v[min(len(v) - 1, int(f * len(v)))]
def summ(err, landed, t):
    e = [x for x, l in zip(err, landed) if l]
    return dict(cep50=st.median(e), cep90=q(e, 0.9), s05=sum(x <= 0.5 for x in e) / len(landed),
                deliv=sum(landed) / len(landed), t=st.mean(x for x, l in zip(t, landed) if l))
def paired(ea, la, eb, lb):
    idx = [i for i in range(len(ea)) if la[i] and lb[i]]; rng = random.Random(0); d = []
    for _ in range(a.boot):
        s = [idx[rng.randrange(len(idx))] for _ in idx]
        d.append(st.median(ea[i] for i in s) - st.median(eb[i] for i in s))
    d.sort(); return st.median(ea[i] for i in idx) - st.median(eb[i] for i in idx), d[int(0.025 * a.boot)], d[int(0.975 * a.boot)]

print("| 조건 | L0 | GRU-S + EMA (현재) | GRU-C, EMA 없음 (후보) | ΔCEP50 후보−현재 [95% CI] | 유의 |")
print("|---|---|---|---|---|---|")
for name, fc, fs, fl in COND:
    try: C, Sr, L = load(fc), load(fs), load(fl)
    except FileNotFoundError as ex: print(f"| {name} | (missing: {ex.filename}) |"); continue
    c, s, l = summ(*C), summ(*Sr), summ(*L)
    d, lo, hi = paired(C[0], C[1], Sr[0], Sr[1])
    sig = "✓" if hi < 0 or lo > 0 else "–"
    f = lambda x: f"{x['cep50']:.3f} / {x['cep90']:.3f} / {100*x['s05']:.1f}"
    print(f"| {name} | {f(l)} | {f(s)} | {f(c)} | {d:+.3f} [{lo:+.3f}, {hi:+.3f}] | {sig} |")
print("\n셀 = CEP50 / CEP90 [m] / succ@0.5 [%], n = 600 (seed 3000·4000·5000 × 200)")
