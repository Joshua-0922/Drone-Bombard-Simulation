"""Table 1 for the paper: pool eval seeds per arm and print one markdown row each.

Arms are given on the command line as LABEL=GLOB so the scripted baselines
(/tmp/sweep*, /tmp/t_reseed) and the learned arms (/tmp/sleval, /tmp/sl_*) can be
mixed in a single table -- which is the whole point, since exp_027 flew the T
arms on eval seeds {1000,2000,3000} and exp_028 flew the L1 arms on
{3000,4000,5000}. Only a table whose every row is the SAME pooled seed set is
worth printing, so the seed set is asserted, not assumed.

  _agg_table1.py "T2@1.5 (3.5m)=/tmp/{sweep3,t_reseed}/T2p15_alt3.5_s*.json" ...

Error / time / geometry statistics are conditioned on DELIVERED episodes (the
payload actually landed); success and delivery rates are over all episodes.
"""
import glob, json, re, sys
import numpy as np

WANT = {3000, 4000, 5000}
rows, base = [], None

for spec in sys.argv[1:]:
    label, pat = spec.split("=", 1)
    files = sorted(f for g in pat.split(",") for f in glob.glob(g))
    seeds = {int(re.search(r"_s(\d+)\.json$", f)[1]) for f in files}
    assert seeds == WANT, f"{label}: seeds {sorted(seeds)} != {sorted(WANT)}"
    e, land, t, v, alt = [], [], [], [], []
    for f in files:
        r = json.load(open(f))["episodes"]
        m = np.asarray(r["landed"], bool)
        land.append(m)
        e.append(np.asarray(r["release_impact_err"], float)[m])
        t.append(np.asarray(r["deliver_time_s"], float)[m])
        v.append(np.asarray(r["release_speed_xy"], float)[m])
        alt.append(np.asarray(r["release_alt"], float)[m])
    e, land = np.concatenate(e), np.concatenate(land)
    t, v, alt = np.concatenate(t), np.concatenate(v), np.concatenate(alt)
    n = len(land)
    row = dict(label=label, n=n, dl=100 * land.mean(),
               t=t.mean(), tsd=t.std(), err=e.mean(), esd=e.std(),
               cep50=float(np.median(e)), cep90=float(np.percentile(e, 90)),
               s10=100 * (e <= 1.0).sum() / n, s05=100 * (e <= 0.5).sum() / n,
               v=v.mean(), alt=alt.mean())
    base = base or row
    row["d50"] = 100 * (row["cep50"] / base["cep50"] - 1)
    rows.append(row)

hdr = ("| 팔 | n | 배달률% | 배달시간 s | 착지오차 m | CEP50 | CEP90 | "
       "succ@1.0 | succ@0.5 | 투하 v | 고도 | ΔCEP50 |")
print(hdr)
print("|" + "---|" * (hdr.count("|") - 1))
for r in rows:
    d = "—" if r is base else f"{r['d50']:+.1f}%"
    print(f"| {r['label']} | {r['n']} | {r['dl']:.1f} | {r['t']:.2f}±{r['tsd']:.2f} | "
          f"{r['err']:.3f}±{r['esd']:.3f} | {r['cep50']:.3f} | {r['cep90']:.3f} | "
          f"{r['s10']:.2f}% | {r['s05']:.2f}% | {r['v']:.2f} | {r['alt']:.2f} | {d} |")
