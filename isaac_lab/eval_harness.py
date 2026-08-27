"""Shared deterministic-evaluation harness for every drop arm (P0-2).

One collector + one metric definition + one JSON schema, used by BOTH the
learned arms (``play.py --policy``) and the rule-based arms
(``baseline_drop.py``), so Table 1 of the paper compares numbers that were
produced by identical code.

Why this exists (notes/research/paper_research_plan.md §3):

* **Paired evaluation.** Same-seed does NOT by itself give two arms the same
  episodes: the per-episode randomization is drawn inside ``_reset_idx``, and
  which envs reset on which step depends on the policy, so the RNG stream
  diverges as soon as two arms terminate differently. The fix here is
  structural rather than clever — run with ``num_envs == episodes`` and score
  only each env slot's FIRST episode. That first reset happens before any
  action is taken, so every arm sees a bit-identical set of initial conditions
  (handoff, marker, wind, drag, mass belief, gains, biases). This is the
  default (``paired=True``).
* **CEP.** The airdrop literature reports circular error probable, i.e. the
  50th/90th percentile of the miss distance, so the classical baselines and
  ours can be quoted in the same language. Computed over DELIVERED episodes,
  always alongside the delivery rate (a 0.1 m CEP at a 5% delivery rate is not
  a good system).
* **Agility.** ``deliver_time_s`` (episode start -> the event that scores the
  drop) is reported next to accuracy; without it a hover-then-drop policy looks
  optimal. ``feasible_window_s`` is the timing-robustness diagnostic.
* **Honest intervals.** Wilson score interval for rates, percentile bootstrap
  for error statistics. No single-seed point estimates without a spread.

The env is read through its terminal snapshot (``_last_final_snapshot``), never
through post-step tensors — DirectRLEnv resets done envs inside ``step()``, so
anything read afterwards is the NEXT episode's spawn state.
"""

from __future__ import annotations

import json
import math
import os

import torch

CAUSES = ("success", "crash", "overspeed", "bad_attitude", "out_of_range",
          "max_altitude", "overshoot", "stagnation", "timeout")
# Bucket for terminations that raise none of the flags above. In the
# land-terminal envs this is exactly "the payload landed outside the success
# radius" — the released-but-missed population, which must never be folded into
# any other bucket.
OTHER = "delivered_miss/other"
CAUSE_LABELS = CAUSES + (OTHER,)

# per-episode fields pulled from the env's terminal snapshot
_FIELDS = ("d_xy_min", "aim_err_min", "release_impact_err", "deliver_time_s",
           "feasible_window_s", "final_speed_xy", "wind_speed",
           # The parameters that ACTUALLY act on the payload's fall. The old
           # "drag_coef" column recorded a channel with no physics behind it
           # (it fed a dead analytic path) while omitting the ballistic
           # coefficient and release latency that decide where it lands --
           # which made a DR_SCALE sweep impossible to analyse.
           "payload_bc_scale", "release_delay")
_FLAGS = ("released", "landed", "success")


def _finite(t: torch.Tensor) -> torch.Tensor:
    return t[torch.isfinite(t)]


def _step(env, action):
    """Accept both step signatures: the raw gymnasium env returns
    ``(obs, rew, terminated, truncated, info)`` while ``RslRlVecEnvWrapper``
    collapses that to ``(obs, rew, dones, info)``. baseline_drop.py drives the
    env directly (no rsl_rl dependency) and play.py drives the wrapper, so the
    harness has to serve both."""
    out = env.step(action)
    if len(out) == 5:
        obs, rew, terminated, truncated, info = out
        return obs, rew, (terminated | truncated), info
    return out


def collect(env, act_fn, episodes: int, paired: bool = True, max_steps: int = 20000,
            verbose: bool = True) -> dict:
    """Roll out ``act_fn`` and return per-episode tensors.

    ``act_fn(obs, unwrapped_env) -> action``. With ``paired=True`` the run needs
    ``num_envs >= episodes`` and scores env slot k's first episode only."""
    u = env.unwrapped
    n = u.num_envs
    if paired and n < episodes:
        raise ValueError(
            f"paired evaluation needs num_envs >= episodes (got num_envs={n}, "
            f"episodes={episodes}). Re-run with --num_envs {episodes}.")

    rec = {k: [] for k in _FIELDS + _FLAGS}
    rec["cause"] = []
    slots = torch.zeros(n, dtype=torch.bool)   # paired: has this slot been scored?
    n_done = 0
    obs, _ = env.reset()

    for _ in range(max_steps):
        if paired and bool(slots.all()):
            break
        if not paired and n_done >= episodes:
            break
        with torch.inference_mode():
            action = act_fn(obs, u)
        obs, _rew, dones, _info = _step(env, action)
        if not bool(dones.any()):
            continue
        snap = getattr(u, "_last_final_snapshot", None)
        if snap is None:
            continue
        ids = snap["env_ids"].detach().cpu().long()
        keep = torch.ones(ids.numel(), dtype=torch.bool)
        if paired:
            keep = ~slots[ids]
            slots[ids] = True
        if not bool(keep.any()):
            continue

        f = snap["done_flags"]
        # Default to "other", NOT to CAUSES[0]: a land-terminal episode that
        # delivered but missed the success radius raises none of the CAUSES
        # flags, and a zero-init would silently bank it as a success.
        cause_idx = torch.full((ids.numel(),), len(CAUSES), dtype=torch.long)
        for i, c in enumerate(CAUSES):
            cause_idx = torch.where(f[c].detach().cpu().bool(),
                                    torch.full_like(cause_idx, i), cause_idx)
        rec["cause"].append(cause_idx[keep])
        rec["success"].append(f["success"].detach().cpu().bool()[keep])
        rec["released"].append(snap["released"].detach().cpu().bool()[keep])
        rec["landed"].append(snap["landed"].detach().cpu().bool()[keep])
        rec["d_xy_min"].append(snap["d_xy_min"].detach().cpu()[keep])
        rec["aim_err_min"].append(snap["aim_err_min"].detach().cpu()[keep])
        rec["release_impact_err"].append(snap["release_impact_err"].detach().cpu()[keep])
        rec["deliver_time_s"].append(snap["deliver_time_s"].detach().cpu()[keep])
        rec["feasible_window_s"].append(snap["feasible_window_s"].detach().cpu()[keep])
        rec["final_speed_xy"].append(
            torch.linalg.norm(snap["final_vel_xy"].detach().cpu(), dim=-1)[keep])
        rec["payload_bc_scale"].append(snap["payload_bc_scale"].detach().cpu()[keep])
        rec["release_delay"].append(snap["release_delay"].detach().cpu()[keep])
        rec["wind_speed"].append(
            torch.linalg.norm(snap["wind_xy"].detach().cpu(), dim=-1)[keep])
        n_done += int(keep.sum())
        if verbose and n_done % 100 < int(keep.sum()):
            print(f"  [eval] {n_done} episodes scored", flush=True)

    out = {k: (torch.cat(v) if v else torch.zeros(0)) for k, v in rec.items()}
    if not paired:  # trim the tail so every arm reports exactly `episodes`
        out = {k: v[:episodes] for k, v in out.items()}
    return out


def _wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval — correct near 0% and 100%, unlike normal-approx."""
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / d
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, centre - half), min(1.0, centre + half))


def _boot_ci(t: torch.Tensor, stat=torch.mean, iters: int = 2000, seed: int = 0) -> tuple:
    """Percentile bootstrap CI. Deterministic (own generator), so re-running the
    summary on the same records reproduces the same interval."""
    if t.numel() < 2:
        return (float("nan"), float("nan"))
    g = torch.Generator().manual_seed(seed)
    idx = torch.randint(0, t.numel(), (iters, t.numel()), generator=g)
    vals = stat(t[idx], dim=1)
    q = torch.quantile(vals, torch.tensor([0.025, 0.975]))
    return (float(q[0]), float(q[1]))


def _dist(t: torch.Tensor) -> dict:
    t = _finite(t)
    if t.numel() == 0:
        return {"n": 0}
    q = torch.quantile(t, torch.tensor([0.5, 0.9, 0.95]))
    return {"n": int(t.numel()), "mean": float(t.mean()), "std": float(t.std()) if t.numel() > 1 else 0.0,
            "med": float(q[0]), "p90": float(q[1]), "p95": float(q[2]), "max": float(t.max())}


def summarize(rec: dict, physical_payload: bool, success_radius: float) -> dict:
    """Metric block for one arm. ``physical_payload`` picks which flag counts as
    'the payload actually reached the ground' (v16/v19 land-terminal) versus
    'the release event resolved the drop' (analytic envs)."""
    n = int(rec["success"].numel())
    scored = rec["landed"] if physical_payload else rec["released"]
    err = rec["release_impact_err"][scored]

    n_succ = int(rec["success"].sum())
    lo, hi = _wilson(n_succ, n)
    out = {
        "episodes": n,
        "success_rate": n_succ / max(n, 1),
        "success_ci95": [lo, hi],
        "success_radius_m": success_radius,
        "delivery_rate": float(scored.float().mean()) if n else 0.0,
        "release_rate": float(rec["released"].float().mean()) if n else 0.0,
        "landing_error_m": _dist(err),
        # CEP = percentile of the miss distance over delivered episodes; quoted
        # WITH delivery_rate, never alone.
        "cep50_m": float(torch.quantile(err, 0.5)) if err.numel() else float("nan"),
        "cep90_m": float(torch.quantile(err, 0.9)) if err.numel() else float("nan"),
        "landing_error_mean_ci95": _boot_ci(err),
        "deliver_time_s": _dist(rec["deliver_time_s"][scored]),
        "feasible_window_s": _dist(rec["feasible_window_s"]),
        "d_xy_min_m": _dist(rec["d_xy_min"]),
        "aim_err_min_m": _dist(rec["aim_err_min"]),
        "final_speed_xy_mps": _dist(rec["final_speed_xy"]),
        "termination_causes": {c: int((rec["cause"] == i).sum()) for i, c in enumerate(CAUSE_LABELS)},
        # A release that never reached the ground inside the episode — a real
        # failure mode of late firing, and invisible if only release_rate is
        # quoted.
        "released_not_delivered": int((rec["released"] & ~scored).sum()),
    }
    return out


def print_summary(name: str, s: dict) -> None:
    le, dt, fw = s["landing_error_m"], s["deliver_time_s"], s["feasible_window_s"]
    ci = s["success_ci95"]
    print(f"\n===== {name} =====")
    print(f"  episodes            : {s['episodes']}")
    print(f"  success             : {s['success_rate']:.2%}  (95% CI {ci[0]:.2%}-{ci[1]:.2%}, "
          f"radius {s['success_radius_m']:.2f} m)")
    print(f"  delivery / release  : {s['delivery_rate']:.2%} / {s['release_rate']:.2%}")
    if le.get("n"):
        b = s["landing_error_mean_ci95"]
        print(f"  landing error [m]   : mean={le['mean']:.3f} (CI {b[0]:.3f}-{b[1]:.3f}) "
              f"med={le['med']:.3f} p90={le['p90']:.3f} max={le['max']:.3f}  n={le['n']}")
        print(f"  CEP50 / CEP90 [m]   : {s['cep50_m']:.3f} / {s['cep90_m']:.3f}")
    else:
        print("  landing error [m]   : n/a (nothing delivered)")
    if dt.get("n"):
        print(f"  delivery time [s]   : mean={dt['mean']:.2f} med={dt['med']:.2f} p90={dt['p90']:.2f}")
    if fw.get("n"):
        print(f"  release window [s]  : mean={fw['mean']:.2f} med={fw['med']:.2f} max={fw['max']:.2f}")
    causes = ", ".join(f"{c}={v}" for c, v in s["termination_causes"].items() if v)
    print(f"  terminations        : {causes}")
    if s.get("released_not_delivered"):
        print(f"  released, no impact : {s['released_not_delivered']}  (fired too late to land)")


def write_json(path: str, meta: dict, summary: dict, rec: dict) -> None:
    """One JSON per arm: meta + summary + the raw per-episode arrays (so a
    cross-arm paired test can be run offline without re-simulating)."""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    payload = {
        "meta": meta,
        "summary": summary,
        "episodes": {k: [None if (isinstance(x, float) and math.isinf(x)) else x
                         for x in v.tolist()]
                     for k, v in rec.items()},
        "cause_labels": list(CAUSE_LABELS),
    }
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=1)
    print(f"[eval] wrote {path}")


def run_and_report(env, act_fn, *, name: str, episodes: int, paired: bool,
                   meta: dict, out_json: str | None) -> dict:
    """collect -> summarize -> print -> (optional) JSON. The single entry point
    both play.py and baseline_drop.py call, so no arm can drift."""
    u = env.unwrapped
    rec = collect(env, act_fn, episodes, paired=paired)
    task_reward = getattr(u.cfg, "task_reward", None)
    radius = float(task_reward.success_radius if task_reward is not None
                   else getattr(u.cfg.phase_cfg, "success_impact_radius", 1.0))
    summary = summarize(rec, bool(getattr(u.cfg, "payload_physics_enabled", False)), radius)
    print_summary(name, summary)
    meta = dict(meta, arm=name, paired=paired, episodes_requested=episodes,
                num_envs=int(u.num_envs), success_radius_m=radius)
    if out_json:
        write_json(out_json, meta, summary, rec)
    return summary
