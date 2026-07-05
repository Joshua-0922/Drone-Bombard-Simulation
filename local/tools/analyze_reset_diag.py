"""RAD v1 학습 전 검증 — wandb 의 infra/reset_* metrics 분석.

목적: v8/v9a 학습 시 코드에 자동 측정된 reset_diag metric 으로
      RAD 의 fast reset 100% 시나리오 안전성 평가.

사용:
  python local/tools/analyze_reset_diag.py
  python local/tools/analyze_reset_diag.py --runs 96bokgae zjexq20k
  python local/tools/analyze_reset_diag.py --runs 96bokgae --verbose
"""
import argparse
import sys

try:
    import wandb
except ImportError:
    print("[ERROR] wandb 패키지 필요: pip install wandb", file=sys.stderr)
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("[ERROR] numpy 필요", file=sys.stderr)
    sys.exit(1)


ENTITY = "nayoonho0922-seoul-national-university"
PROJECT = "drone-bombard-sac"

# 분석 metric (drone_drop_env_rad.py / train_sac.py 의 InfraHealthMonitorCallback)
METRICS = [
    "infra/reset_pre_v",
    "infra/reset_pre_ang_v",
    "infra/reset_post_cruise_v",
    "infra/reset_post_cruise_ang_v",
    "infra/consecutive_fast_resets",
    "infra/used_full_restart",
    "infra/cruise_timeout_attempts",
    "infra/forced_restart_triggered",
]

# 판단 기준
THRESHOLDS = {
    "post_cruise_v_max": 1.5,           # m/s (cruise target ~1m/s + noise)
    "post_cruise_v_drift_per_100k": 0.1,  # m/s — slope, 0.1 이상이면 누설 의심
    "post_cruise_ang_v_max": 0.5,       # rad/s
    "cruise_timeout_sum": 5,             # 매 학습 총 5 미만 정상
    "fast_reset_cap_v8": 100,            # v8 cap
    "fast_reset_cap_rad": 50,            # RAD cap
}


def fetch_run_history(run_id, entity, project, metrics, verbose=False):
    """wandb run 의 history 받기. None 값 제외, step 별 dict 반환."""
    if verbose:
        print(f"  [wandb] Fetching {entity}/{project}/{run_id} ...", flush=True)
    api = wandb.Api()
    run = api.run(f"{entity}/{project}/{run_id}")

    # scan_history 가 generator. keys 명시하면 효율 ↑
    keys = ["_step"] + metrics
    history = []
    for row in run.scan_history(keys=keys, page_size=1000):
        # 모든 key 가 있고 None 아닌 것만
        if any(row.get(m) is not None for m in metrics):
            history.append(row)

    return run, history


def analyze_metric(history, metric, step_key="_step"):
    """metric 의 step별 값 추출 + 통계 + linear trend."""
    pairs = [(r.get(step_key), r.get(metric)) for r in history
             if r.get(metric) is not None and r.get(step_key) is not None]
    if len(pairs) < 5:
        return None
    steps = np.array([p[0] for p in pairs], dtype=np.float64)
    vals = np.array([p[1] for p in pairs], dtype=np.float64)
    # Linear trend (slope per step)
    try:
        slope, intercept = np.polyfit(steps, vals, 1)
    except Exception:
        slope, intercept = 0.0, vals.mean()
    return {
        "n": len(vals),
        "mean": float(vals.mean()),
        "std": float(vals.std()),
        "min": float(vals.min()),
        "max": float(vals.max()),
        "p50": float(np.percentile(vals, 50)),
        "p95": float(np.percentile(vals, 95)),
        "slope_per_step": float(slope),
        "slope_per_100k": float(slope) * 100_000,
        "first_step": int(steps[0]),
        "last_step": int(steps[-1]),
    }


def print_run_report(run_id, run_name, history, verbose=False):
    """단일 run 의 6 metric 분석 결과 print."""
    print(f"\n=== {run_id} ({run_name}) ===")
    print(f"  history rows with reset_diag: {len(history)}")
    if not history:
        print("  [WARN] reset_diag metric 없음 — InfraHealthMonitorCallback 미활성 학습")
        return None

    stats = {m: analyze_metric(history, m) for m in METRICS}

    # post_cruise_v: 핵심 metric
    pcv = stats.get("infra/reset_post_cruise_v")
    if pcv:
        verdict_v = []
        # max check
        if pcv["max"] > THRESHOLDS["post_cruise_v_max"]:
            verdict_v.append(f"❌ max {pcv['max']:.2f} > {THRESHOLDS['post_cruise_v_max']}")
        else:
            verdict_v.append(f"✅ max {pcv['max']:.2f} ≤ {THRESHOLDS['post_cruise_v_max']}")
        # drift (slope per 100k step)
        if pcv["slope_per_100k"] > THRESHOLDS["post_cruise_v_drift_per_100k"]:
            verdict_v.append(
                f"⚠️ drift +{pcv['slope_per_100k']:.3f}/100k > {THRESHOLDS['post_cruise_v_drift_per_100k']}"
            )
        else:
            verdict_v.append(f"✅ drift {pcv['slope_per_100k']:+.3f}/100k (안정)")

        print(f"  [핵심] post_cruise_v: mean={pcv['mean']:.3f}, "
              f"max={pcv['max']:.3f}, p95={pcv['p95']:.3f}")
        print(f"           → {' | '.join(verdict_v)}")

    pcav = stats.get("infra/reset_post_cruise_ang_v")
    if pcav:
        verdict_av = "✅" if pcav["max"] <= THRESHOLDS["post_cruise_ang_v_max"] else "❌"
        print(f"  post_cruise_ang_v: mean={pcav['mean']:.3f}, max={pcav['max']:.3f} "
              f"{verdict_av}")

    # pre_v vs post_cruise_v reduction
    prv = stats.get("infra/reset_pre_v")
    if prv and pcv and prv["mean"] > 0.1:
        reduction = 1.0 - pcv["mean"] / prv["mean"]
        verdict_r = "✅ set_pose effective" if reduction > 0.5 else "⚠️ set_pose weak"
        print(f"  pre_v mean={prv['mean']:.3f} → post_cruise_v mean={pcv['mean']:.3f} "
              f"(reduction {reduction*100:.0f}%) {verdict_r}")

    # consecutive_fast_resets
    cfr = stats.get("infra/consecutive_fast_resets")
    if cfr:
        print(f"  consecutive_fast_resets: max={cfr['max']:.0f}, p95={cfr['p95']:.0f}")
        # RAD 의 cap=50 시 발동 빈도 추정
        if cfr["max"] >= 50:
            print(f"    → RAD cap=50 발동 빈도: 약 {(cfr['p95'] / 50) * 100:.0f}% (p95 기준)")
        else:
            print("    → RAD cap=50 미발동 예상 (v8/v9a 의 max < 50)")

    # used_full_restart
    ufr = stats.get("infra/used_full_restart")
    if ufr:
        print(f"  used_full_restart: mean={ufr['mean']*100:.1f}% (full restart 발동 비율)")

    # cruise_timeout_attempts
    cta = stats.get("infra/cruise_timeout_attempts")
    if cta:
        verdict_t = "✅" if cta["mean"] < 0.1 else "⚠️"
        print(f"  cruise_timeout_attempts: mean={cta['mean']:.3f}, max={cta['max']:.0f} "
              f"{verdict_t}")

    # forced_restart
    fr = stats.get("infra/forced_restart_triggered")
    if fr:
        print(f"  forced_restart_triggered: mean={fr['mean']*100:.2f}%")

    if verbose:
        print("\n  --- full stats ---")
        for m in METRICS:
            s = stats.get(m)
            if s:
                print(f"    {m}: n={s['n']}, mean={s['mean']:.4f}, max={s['max']:.4f}, "
                      f"slope/100k={s['slope_per_100k']:+.4f}")

    return stats


def final_verdict(all_stats):
    """모든 run 의 분석 결과 종합 → RAD 학습 안전성 판단."""
    print("\n" + "=" * 60)
    print("=== RAD v1 학습 시작 안전성 verdict ===")
    print("=" * 60)

    issues = []
    recommendations = []

    # 모든 run 의 post_cruise_v 검사
    for run_id, stats in all_stats.items():
        if not stats:
            continue
        pcv = stats.get("infra/reset_post_cruise_v")
        if not pcv:
            continue
        if pcv["max"] > THRESHOLDS["post_cruise_v_max"]:
            issues.append(f"[{run_id}] post_cruise_v max {pcv['max']:.2f} 초과")
        if pcv["slope_per_100k"] > THRESHOLDS["post_cruise_v_drift_per_100k"]:
            issues.append(f"[{run_id}] post_cruise_v drift +{pcv['slope_per_100k']:.3f}/100k 누설 의심")

    # RAD specific
    rad_max_cfr = max(
        (s["infra/consecutive_fast_resets"]["max"]
         for s in all_stats.values()
         if s and s.get("infra/consecutive_fast_resets")),
        default=0,
    )
    if rad_max_cfr >= 50:
        rad_cap_hit_freq = (rad_max_cfr / 50) * 100
        recommendations.append(
            f"RAD cap=50: v8/v9a 의 max consecutive_fast_resets = {rad_max_cfr:.0f} → "
            f"RAD 에서 cap 발동 빈도 ~{rad_cap_hit_freq:.0f}% 추정. "
            f"학습 시간 영향 = +38s/100ep × {rad_cap_hit_freq:.0f}% ≈ "
            f"{38 * rad_cap_hit_freq / 100:.1f}s/ep overhead."
        )
    else:
        recommendations.append(
            f"RAD cap=50 미발동 예상 (v8/v9a max={rad_max_cfr:.0f} < 50)."
        )

    if not issues:
        print("\n✅ 모든 메트릭 정상. RAD 학습 시작 안전.")
    else:
        print(f"\n⚠️ 발견된 이슈 ({len(issues)} 건):")
        for issue in issues:
            print(f"  - {issue}")
        print("\n  RAD 학습 시작 전 root cause 분석 + fix 권장.")

    print("\n  RAD-specific 권장:")
    for rec in recommendations:
        print(f"  - {rec}")

    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--runs",
        nargs="+",
        default=["96bokgae", "zjexq20k"],
        help="wandb run id list (default: v8=96bokgae, v9a=zjexq20k)",
    )
    parser.add_argument("--entity", default=ENTITY)
    parser.add_argument("--project", default=PROJECT)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    print(f"[analyze_reset_diag] entity={args.entity}, project={args.project}")
    print(f"  runs: {args.runs}")
    print(f"  metrics: {len(METRICS)} ({', '.join(m.split('/')[-1] for m in METRICS)})")

    all_stats = {}
    for run_id in args.runs:
        try:
            run, history = fetch_run_history(
                run_id, args.entity, args.project, METRICS, args.verbose
            )
            stats = print_run_report(run_id, run.name, history, args.verbose)
            all_stats[run_id] = stats
        except wandb.errors.CommError as e:
            print(f"\n[ERROR] {run_id}: wandb 통신 실패 — {e}", file=sys.stderr)
            print(f"  → wandb login 또는 entity/project 확인", file=sys.stderr)
        except Exception as e:
            print(f"\n[ERROR] {run_id}: {type(e).__name__}: {e}", file=sys.stderr)

    final_verdict(all_stats)


if __name__ == "__main__":
    main()
