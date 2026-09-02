"""Pool the evaluation seeds per arm and report the paper's metrics.

Arms are discovered from the filenames (`{arm}_dr{DR}_s{seed}.json`), so the
same script serves the main sweep, the unseen-R sweep, the policy-transfer
sweep and the label-count sweep without an edit.  `--wandb` pushes the table
as one job_type='eval' run -- play.py's own --wandb hook only covers the
unpaired path, and 30 separate runs would bury the comparison anyway.
"""
import argparse, glob, json, os, re
import numpy as np

# Preferred display order; anything else sorts after, alphabetically.
ORDER = ["L0", "SLobs", "SLtilt", "SLobsE", "SLtiltE", "ORCwind"]
LABEL = {"L0": "L0 (no residual)",
         "SLobs": "L1-SL obs            (raw)",
         "SLtilt": "L1-SL obs+tilt       (raw)",
         "SLobsE": "L1-SL obs        EMA 0.3",
         "SLtiltE": "L1-SL obs+tilt   EMA 0.3",
         "ORCwind": "oracle wind-only  [ceiling]"}

p = argparse.ArgumentParser()
p.add_argument("root", nargs="?", default="/tmp/sleval")
p.add_argument("--wandb", action="store_true")
p.add_argument("--wandb_name", default=None)
p.add_argument("--tag", default="", help="free-text note carried into the wandb config")
a = p.parse_args()

found = {}
for f in glob.glob(f"{a.root}/*_dr*_s*.json"):
    m = re.fullmatch(r"(.+)_dr([\d.]+)_s(\d+)\.json", os.path.basename(f))
    if m:
        found.setdefault((m[2], m[1]), []).append(f)

rows = []
for dr in sorted({k[0] for k in found}, key=float):
    arms = sorted({k[1] for k in found if k[0] == dr},
                  key=lambda x: (ORDER.index(x) if x in ORDER else len(ORDER), x))
    print(f"\n=== DR {dr}   (paired episodes, seeds pooled) ===")
    print(f"{'arm':30s} {'n':>5s} {'succ@1.0':>9s} {'succ@0.5':>9s} "
          f"{'CEP50':>7s} {'CEP90':>7s} {'err mean':>9s} {'deliver%':>9s}")
    base = None
    for arm in arms:
        err, deliv = [], []
        for f in sorted(found[(dr, arm)]):
            r = json.load(open(f))["episodes"]
            e = np.asarray(r["release_impact_err"], float)
            land = np.asarray(r["landed"], bool)
            err.append(e[land]); deliv.append(land)
        e = np.concatenate(err); land = np.concatenate(deliv)
        n = len(land)
        row = dict(dr=float(dr), arm=arm, n=n, cep50=float(np.median(e)),
                   cep90=float(np.percentile(e, 90)), mean=float(e.mean()),
                   s10=float((e <= 1.0).sum() / n), s05=float((e <= 0.5).sum() / n),
                   dl=float(land.mean()))
        base = base or row
        row["d50_pct"] = 100 * (row["cep50"] / base["cep50"] - 1)
        d50 = "" if row is base else f"  ({row['d50_pct']:+.1f}%)"
        print(f"{LABEL.get(arm, arm):30s} {n:5d} {100*row['s10']:8.2f}% {100*row['s05']:8.2f}% "
              f"{row['cep50']:7.3f} {row['cep90']:7.3f} {row['mean']:9.3f} "
              f"{100*row['dl']:8.2f}%{d50}")
        rows.append(row)

if a.wandb:
    import wandb
    cols = list(rows[0])
    run = wandb.init(project="drone-bombard-isaac", job_type="eval",
                     name=a.wandb_name or f"agg_{os.path.basename(a.root)}",
                     config={"root": a.root, "tag": a.tag})
    wandb.log({"eval/table": wandb.Table(columns=cols,
                                         data=[[r[c] for c in cols] for r in rows])})
    run.summary.update({f"{r['arm']}_dr{r['dr']}/cep50": r["cep50"] for r in rows})
    run.summary.update({f"{r['arm']}_dr{r['dr']}/succ05": r["s05"] for r in rows})
    wandb.finish()
    print(f"[wandb] logged {len(rows)} rows as run {run.name}")
