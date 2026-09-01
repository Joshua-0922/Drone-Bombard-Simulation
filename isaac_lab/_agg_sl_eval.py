"""Pool the three evaluation seeds per arm and report the paper's metrics."""
import glob, json, os, sys
import numpy as np

ARMS = [("L0", "L0 (no residual)"),
        ("SLobs", "L1-SL obs            (raw)"),
        ("SLtilt", "L1-SL obs+tilt       (raw)"),
        ("SLobsE", "L1-SL obs        EMA 0.3"),
        ("SLtiltE", "L1-SL obs+tilt   EMA 0.3"),
        ("ORCwind", "oracle wind-only  [ceiling]")]
root = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sleval"

for dr in ("1.5", "2.5"):
    print(f"\n=== DR {dr}   (3 eval seeds x 200 paired episodes) ===")
    print(f"{'arm':30s} {'n':>5s} {'succ@1.0':>9s} {'succ@0.5':>9s} "
          f"{'CEP50':>7s} {'CEP90':>7s} {'err mean':>9s} {'deliver%':>9s}")
    base = None
    for key, label in ARMS:
        err, ok, deliv = [], [], []
        for f in sorted(glob.glob(f"{root}/{key}_dr{dr}_s*.json")):
            d = json.load(open(f))
            r = d["episodes"]
            e = np.asarray(r["release_impact_err"], float)
            land = np.asarray(r["landed"], bool)
            err.append(e[land]); deliv.append(land)
        if not err:
            continue
        e = np.concatenate(err); land = np.concatenate(deliv)
        n = len(land)
        row = dict(cep50=np.median(e), cep90=np.percentile(e, 90), mean=e.mean(),
                   s10=(e <= 1.0).sum() / n, s05=(e <= 0.5).sum() / n,
                   dl=land.mean())
        if base is None:
            base = row
        d50 = "" if row is base else f"  ({100*(row['cep50']/base['cep50']-1):+.1f}%)"
        print(f"{label:30s} {n:5d} {100*row['s10']:8.2f}% {100*row['s05']:8.2f}% "
              f"{row['cep50']:7.3f} {row['cep90']:7.3f} {row['mean']:9.3f} "
              f"{100*row['dl']:8.2f}%{d50}")
