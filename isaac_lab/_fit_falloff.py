"""_fit_falloff.py — summarize yolo_eval.py --calibrate output per range bin
and fit the slant-range falloff parameters for DroneBombardVisionCfg.

Usage: /isaac-sim/python.sh _fit_falloff.py <calibration.csv>
Prints the per-bin table (detect rate, conf stats, pixel error) and a
suggested piecewise-linear fit for (a) the conf multiplier normalized to the
near-field mean and (b) the detection probability.
"""

import csv
import sys
from collections import defaultdict

import numpy as np


def main(path):
    by_range = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            by_range[float(row["slant_range_m"])].append(
                (row["detected"] == "True", float(row["conf"]),
                 float(row["du_px"]) if row["du_px"] != "nan" else np.nan,
                 float(row["dv_px"]) if row["dv_px"] != "nan" else np.nan)
            )

    print(f"{'slant_m':>8} {'n':>4} {'detect%':>8} {'conf_mean':>10} {'conf_std':>9} {'|duv|_px':>9}")
    ranges, det_rates, confs = [], [], []
    for r in sorted(by_range):
        rows = by_range[r]
        det = np.array([x[0] for x in rows])
        conf = np.array([x[1] for x in rows])[det] if det.any() else np.array([])
        duv = np.array([[x[2], x[3]] for x in rows])
        duv = duv[~np.isnan(duv).any(axis=1)]
        err = np.linalg.norm(duv, axis=1).mean() if len(duv) else float("nan")
        cm = conf.mean() if len(conf) else 0.0
        cs = conf.std() if len(conf) else 0.0
        print(f"{r:>8.1f} {len(rows):>4} {det.mean():>8.1%} {cm:>10.3f} {cs:>9.3f} {err:>9.1f}")
        ranges.append(r); det_rates.append(det.mean()); confs.append(cm)

    ranges = np.array(ranges); det_rates = np.array(det_rates); confs = np.array(confs)
    # normalize conf to the near-field plateau (mean of bins <= 9 m with detections)
    near = confs[(ranges <= 9.0) & (confs > 0)]
    if len(near) == 0:
        print("\nNO near-field detections — calibration scene is broken, do not fit.")
        return
    c0 = near.mean()
    print(f"\nnear-field conf plateau c0 = {c0:.3f}")
    print(f"{'slant_m':>8} {'conf/c0':>8} {'detect':>8}")
    for r, c, p in zip(ranges, confs, det_rates):
        print(f"{r:>8.1f} {c/c0:>8.2f} {p:>8.2f}")

    # least-squares linear fit on the falling section (conf/c0 < 0.98)
    g = np.clip(confs / c0, 0, None)
    fall = g < 0.98
    if fall.sum() >= 2:
        A = np.vstack([np.ones(fall.sum()), ranges[fall]]).T
        coef, *_ = np.linalg.lstsq(A, g[fall], rcond=None)
        print(f"\nconf multiplier fit (falling section): g(r) ~= {coef[0]:.3f} {coef[1]:+.4f}*r")
    fallp = det_rates < 0.98
    if fallp.sum() >= 2:
        A = np.vstack([np.ones(fallp.sum()), ranges[fallp]]).T
        coefp, *_ = np.linalg.lstsq(A, det_rates[fallp], rcond=None)
        print(f"detect prob fit  (falling section): p(r) ~= {coefp[0]:.3f} {coefp[1]:+.4f}*r")


if __name__ == "__main__":
    main(sys.argv[1])
