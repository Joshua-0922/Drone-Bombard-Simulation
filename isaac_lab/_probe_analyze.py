"""_probe_analyze.py — offline Probe A/B/C analysis of _probe_traj.py output.

Pure numpy, no isaaclab imports — run with any python that has numpy, e.g.
  docker exec isaac-verify /isaac-sim/python.sh \
      /workspace/drone-bombard/isaac_lab/_probe_analyze.py \
      /workspace/drone-bombard/isaac_lab/_probe_out/probeA_traj.npz

Probe A — termination census (re-derived from the npz cause codes).
Probe B — max_altitude episode shape classification:
            deliberate-climb vs loss-of-control vs ambiguous, plus
            approach-then-climb vs climb-from-spawn onset split.
Probe C — vision-reward counterfactual: re-score every episode's r3_vision
            stream with a slant-range confidence falloff
            g(slant) = clip(1.5 - slant/10, 0.1, 1.0)
            (the candidate curve from research/isaac_ppo_tuning_recommendations
            #1; the calibrated curve replaces it in A2 — the probe question is
            only "does climb income collapse", which is curve-shape robust).

All thresholds are stated inline next to their physical justification.
"""

import sys

import numpy as np

CAUSES = ("success", "crash", "overspeed", "bad_attitude", "out_of_range",
          "max_altitude", "overshoot", "stagnation", "timeout")
DT = 0.1               # policy step, 10 Hz
W_VISION = 1.5         # cfg.w_vision_center
VZ_CMD_MAX = 3.0       # vz_scale — max COMMANDABLE climb rate (m/s)
CLIMB_Z_BAND = 12.0    # above the 9-11 m spawn band -> unambiguous climb phase


def episodes_from(done, cause):
    """Yield (env, start, end, cause_idx) — end inclusive, done at end."""
    T, N = done.shape
    for n in range(N):
        start = 0
        for t in np.flatnonzero(done[:, n]):
            yield n, start, int(t), int(cause[t, n])
            start = int(t) + 1


def falloff(slant):
    return np.clip(1.5 - slant / 10.0, 0.1, 1.0)


def vision_income(d, eps_steps):
    """(orig, counterfactual) per-step r3_vision arrays for one episode."""
    u, v, conf = d["u_norm"][eps_steps], d["v_norm"][eps_steps], d["conf"][eps_steps]
    d_xy, z = d["d_xy"][eps_steps], d["z"][eps_steps]
    center = np.sqrt(u * u + v * v)
    prox = np.clip(1.0 - d_xy / 30.0, 0.0, None)
    base = np.where(conf > 0, W_VISION * np.clip(1.0 - center, 0.0, None) * conf * prox, 0.0)
    slant = np.sqrt(d_xy * d_xy + z * z)
    return base, base * falloff(slant)


def main(path):
    d = np.load(path)
    done, cause = d["done"], d["cause"]
    eps = list(episodes_from(done, cause))

    # ---------------- Probe A ----------------
    counts = {c: 0 for c in CAUSES}
    for _, _, _, ci in eps:
        counts[CAUSES[ci - 1]] += 1
    total = len(eps)
    print("=" * 72)
    print(f"PROBE A — termination census on KICK-FIXED plant ({total} episodes)")
    for c in CAUSES:
        if counts[c]:
            print(f"  {c:<14} {counts[c]:>4}  ({counts[c]/total:.1%})")

    # ---------------- Probe B ----------------
    print("=" * 72)
    print("PROBE B — max_altitude episode shape classification")
    cls = {"deliberate_climb": 0, "loss_of_control": 0, "ambiguous": 0}
    onset = {"approach_then_climb": 0, "climb_from_spawn": 0}
    stats = {"dur": [], "vz_med": [], "vz_max": [], "conf_frac": [],
             "center_med": [], "angspd_max": [], "dxy_at_onset": []}
    for n, s, e, ci in eps:
        if CAUSES[ci - 1] != "max_altitude" or e - s < 3:
            continue
        sl = (slice(s, e + 1), n)
        z, vz = d["z"][sl], d["vz"][sl]
        conf, u, v = d["conf"][sl], d["u_norm"][sl], d["v_norm"][sl]
        angspd, d_xy = d["ang_speed"][sl], d["d_xy"][sl]

        t_min = int(np.argmin(z))                    # climb onset = altitude minimum
        climb = slice(t_min, len(z))
        dur = (len(z) - t_min) * DT
        vz_med, vz_max = float(np.median(vz[climb])), float(vz[climb].max())
        det = conf[climb] > 0
        conf_frac = float(det.mean())
        center = np.sqrt(u[climb] ** 2 + v[climb] ** 2)
        center_med = float(np.median(center[det])) if det.any() else float("nan")
        angspd_max = float(angspd[climb].max())

        # loss-of-control: climb faster than the controller can even be
        # COMMANDED (vz > 4.5 = vz_scale + 50% margin), or a spike so short
        # it cannot be a strategy (<2 s from z-min to the 25 m ceiling), or
        # rotational blow-up flirting with the 2.0 rad/s termination limit.
        if vz_max > 1.5 * VZ_CMD_MAX or dur < 2.0 or angspd_max > 1.8:
            cls["loss_of_control"] += 1
        # deliberate climb: sustained (>=3 s), within commanded envelope,
        # target kept detected+centered while climbing — vision farming.
        elif dur >= 3.0 and vz_med > 0.3 and conf_frac >= 0.7 and center_med < 0.5:
            cls["deliberate_climb"] += 1
        else:
            cls["ambiguous"] += 1

        # onset: did it fly toward the target first (close >=30% of the
        # spawn distance before the climb) or climb straight from spawn?
        if t_min >= 20 and d_xy[t_min] < 0.7 * d_xy[0]:
            onset["approach_then_climb"] += 1
        else:
            onset["climb_from_spawn"] += 1

        stats["dur"].append(dur); stats["vz_med"].append(vz_med)
        stats["vz_max"].append(vz_max); stats["conf_frac"].append(conf_frac)
        stats["center_med"].append(center_med); stats["angspd_max"].append(angspd_max)
        stats["dxy_at_onset"].append(float(d_xy[t_min]))

    n_alt = sum(cls.values())
    print(f"  classified {n_alt} max_altitude episodes:")
    for k, v in cls.items():
        print(f"    {k:<22} {v:>4}  ({v/max(n_alt,1):.1%})")
    for k, v in onset.items():
        print(f"    {k:<22} {v:>4}  ({v/max(n_alt,1):.1%})")
    if n_alt:
        print("  climb-phase medians: "
              f"dur={np.median(stats['dur']):.1f}s vz_med={np.median(stats['vz_med']):.2f}m/s "
              f"vz_max={np.median(stats['vz_max']):.2f}m/s conf_frac={np.median(stats['conf_frac']):.2f} "
              f"center_med={np.nanmedian(stats['center_med']):.3f} "
              f"angspd_max={np.median(stats['angspd_max']):.2f}rad/s "
              f"d_xy@onset={np.median(stats['dxy_at_onset']):.2f}m")

    # ---------------- Probe C ----------------
    print("=" * 72)
    print("PROBE C — vision income counterfactual (slant-range falloff)")
    for group in ("max_altitude", "success"):
        tot_o, tot_c, hi_o, hi_c, cnt = 0.0, 0.0, 0.0, 0.0, 0
        for n, s, e, ci in eps:
            if CAUSES[ci - 1] != group or e - s < 3:
                continue
            sl = (slice(s, e + 1), n)
            base, cf = vision_income(d, sl)
            z = d["z"][sl]
            hi = z > CLIMB_Z_BAND
            tot_o += float(base.sum()); tot_c += float(cf.sum())
            hi_o += float(base[hi].sum()); hi_c += float(cf[hi].sum())
            cnt += 1
        if cnt == 0:
            print(f"  {group}: no episodes")
            continue
        print(f"  {group} ({cnt} eps): vision income/ep "
              f"orig={tot_o/cnt:+.1f} -> counterfactual={tot_c/cnt:+.1f} "
              f"(x{tot_c/max(tot_o,1e-9):.2f})")
        if hi_o > 0:
            print(f"    of which ABOVE z={CLIMB_Z_BAND:.0f}m: "
                  f"orig={hi_o/cnt:+.1f} -> cf={hi_c/cnt:+.1f} (x{hi_c/hi_o:.2f})")


if __name__ == "__main__":
    main(sys.argv[1])
