#!/usr/bin/env python3
"""
Representative Best Drop 분석 — 학습 종료 후 사후 분석.

정의:
    peak_episode = LAST episode E where rolling_100_success_rate(E) == max(all)
    peak_window  = episodes [E-99, E] (100 episodes)
    representative_top_3 = sorted(auto_success_drops in peak_window by drop_error)[:3]

규칙:
    - "auto drop 만" 카운트 (random/manual 제외)
    - drop 갯수 < 3 시 → "not measurable" (skip)
    - tie 시 LAST peak

입력:
    --output-log  : wandb run 의 output.log (rollout 별 success_rate 시계열)
    --index-csv   : drop_episodes/index.csv (모든 drops + drop_error + is_success)
    --success-replay : success_replay/{run_id}/ (auto+success 모델 파일들)
    --step-range  : "START,END" — 이 학습의 step 범위 (이전 run 의 entries 필터)
    --run-id      : WandB run id (로깅용)
    --run-name    : run name (로깅용)
    --output-json : 결과 저장 경로 (default: success-replay 폴더 안)

사용 예:
    python3 representative_best_analysis.py \\
        --output-log local/backups/phase1_final_round7_v3/wandb_run/files/output.log \\
        --index-csv local/backups/phase1_final_round7_v3/drop_episodes/index.csv \\
        --success-replay local/backups/phase1_final_round7_v3/success_replay_436xl0bb \\
        --step-range 385807,685807 \\
        --run-id 436xl0bb \\
        --run-name round7_v3_critic_stable
"""

import argparse
import csv
import json
import re
import sys
from pathlib import Path


def parse_rollouts(output_log_path: Path) -> list[dict]:
    """Parse output.log → rollouts with (success_rate, episodes, total_timesteps)."""
    rollouts = []
    content = output_log_path.read_text()
    # Split on rollout block separators
    blocks = re.split(r"-{4,}\n", content)
    for block in blocks:
        sr = re.search(r"success_rate\s+\|\s+([\d.]+)", block)
        ep = re.search(r"episodes\s+\|\s+(\d+)", block)
        ts = re.search(r"total_timesteps\s+\|\s+(\d+)", block)
        if sr and ep and ts:
            rollouts.append({
                "success_rate": float(sr.group(1)),
                "episodes": int(ep.group(1)),
                "total_timesteps": int(ts.group(1)),
            })
    return rollouts


def find_last_peak(rollouts: list[dict]) -> tuple[float, dict]:
    """Find max success_rate, return LAST occurrence."""
    if not rollouts:
        return 0.0, None
    max_sr = max(r["success_rate"] for r in rollouts)
    peaks = [r for r in rollouts if r["success_rate"] == max_sr]
    return max_sr, peaks[-1]  # last


def episode_to_step(rollouts: list[dict], target_episode: int) -> int:
    """Map episode count to closest preceding step."""
    sorted_rollouts = sorted(rollouts, key=lambda r: r["episodes"])
    closest_step = None
    for r in sorted_rollouts:
        if r["episodes"] >= target_episode:
            return r["total_timesteps"]
        closest_step = r["total_timesteps"]
    return closest_step or 0


def find_auto_success_in_window(
    index_csv: Path,
    success_replay_dir: Path,
    step_range: tuple[int, int],
    window_step_range: tuple[int, int],
) -> list[dict]:
    """Filter index.csv rows: in run's step range + in window + is_success=True
    + auto (검증: success_replay 폴더의 모델 파일 존재 = callback 이 auto+success 로 저장)
    """
    run_start, run_end = step_range
    win_start, win_end = window_step_range
    results = []

    with open(index_csv) as f:
        reader = csv.DictReader(f)
        # index.csv 의 컬럼 후보: filename, timestep, drop_error_m, is_success,
        # episode_reward, n_steps, (drop_trigger 추가될 예정)
        for row in reader:
            step = int(row["timestep"])
            if step < run_start or step > run_end:
                continue  # 다른 run 의 entry
            if not (win_start <= step <= win_end):
                continue  # window 밖
            is_success = row["is_success"].strip() == "True"
            if not is_success:
                continue

            # auto drop 검증 — drop_trigger 컬럼 있으면 그걸로, 없으면 success_replay 파일로 추정
            is_auto = False
            if "drop_trigger" in row:
                is_auto = row["drop_trigger"].strip() == "auto"
            else:
                # Fallback: success_replay 폴더에 같은 step 의 파일이 있으면 auto+success 였던 것
                matching = list(success_replay_dir.glob(f"success_step{step}_err*"))
                is_auto = bool(matching)

            if not is_auto:
                continue

            # 모델 파일 위치
            matching = list(success_replay_dir.glob(f"success_step{step}_err*"))
            model_path = str(matching[0]) if matching else None

            results.append({
                "step": step,
                "drop_error_m": float(row["drop_error_m"]),
                "model_filename": matching[0].name if matching else None,
                "model_path": model_path,
            })

    return results


def analyze(
    output_log: Path,
    index_csv: Path,
    success_replay_dir: Path,
    step_range: tuple[int, int],
    run_id: str,
    run_name: str,
) -> dict:
    """Main analysis. Returns manifest dict."""
    rollouts = parse_rollouts(output_log)
    print(f"Parsed {len(rollouts)} rollouts", file=sys.stderr)
    if not rollouts:
        return {
            "run_id": run_id,
            "run_name": run_name,
            "status": "error",
            "error": "No rollouts parsed from output.log",
        }

    print(f"  step range: {rollouts[0]['total_timesteps']:,}~{rollouts[-1]['total_timesteps']:,}",
          file=sys.stderr)
    print(f"  episode range: {rollouts[0]['episodes']:,}~{rollouts[-1]['episodes']:,}",
          file=sys.stderr)

    # Find peak (last occurrence)
    max_sr, peak = find_last_peak(rollouts)
    print(f"\nPeak success_rate: {max_sr}", file=sys.stderr)
    print(f"  At episode {peak['episodes']:,} (step {peak['total_timesteps']:,})",
          file=sys.stderr)

    # Window: last 100 episodes ending at peak
    window_ep_start = peak["episodes"] - 100
    window_ep_end = peak["episodes"]
    window_step_start = episode_to_step(rollouts, window_ep_start)
    window_step_end = peak["total_timesteps"]
    print(f"  Window episodes [{window_ep_start}, {window_ep_end}]", file=sys.stderr)
    print(f"  Window steps [{window_step_start:,}, {window_step_end:,}]", file=sys.stderr)

    # Find auto+success drops in window
    drops_in_window = find_auto_success_in_window(
        index_csv, success_replay_dir, step_range,
        (window_step_start, window_step_end),
    )
    print(f"\nAuto+success drops in peak window: {len(drops_in_window)}", file=sys.stderr)
    for d in drops_in_window:
        print(f"  step={d['step']:,}  err={d['drop_error_m']}m  {d['model_filename']}",
              file=sys.stderr)

    # Top 3 by drop_error
    sorted_drops = sorted(drops_in_window, key=lambda x: x["drop_error_m"])

    # 결정 C: drop < 3 → not measurable
    if len(sorted_drops) < 3:
        return {
            "run_id": run_id,
            "run_name": run_name,
            "status": "not_measurable",
            "reason": (f"Auto+success drops in peak window = {len(sorted_drops)} (< 3). "
                       f"Per decision C: skip representative_best measurement."),
            "step_range": list(step_range),
            "peak_success_rate": max_sr,
            "peak_episode": peak["episodes"],
            "peak_step": peak["total_timesteps"],
            "peak_window_episodes": [window_ep_start, window_ep_end],
            "peak_window_steps": [window_step_start, window_step_end],
            "auto_success_drops_in_window": len(sorted_drops),
            "drops_found_in_window": [
                {"step": d["step"], "drop_error_m": d["drop_error_m"]}
                for d in sorted_drops
            ],
            "interpretation": (
                "이 학습은 representative_top_3 측정 기준을 충족 못함. "
                "Peak success_rate window 안에 충분한 auto+success drop 이 없음. "
                "주로 random_drop 의 도움으로 success_rate 가 올라간 경우 발생."
            ),
        }

    top_3 = sorted_drops[:3]
    return {
        "run_id": run_id,
        "run_name": run_name,
        "status": "measured",
        "step_range": list(step_range),
        "peak_success_rate": max_sr,
        "peak_episode": peak["episodes"],
        "peak_step": peak["total_timesteps"],
        "peak_window_episodes": [window_ep_start, window_ep_end],
        "peak_window_steps": [window_step_start, window_step_end],
        "auto_success_drops_in_window": len(sorted_drops),
        "representative_top_3": [
            {
                "rank": i + 1,
                "drop_error_m": d["drop_error_m"],
                "step": d["step"],
                "model_filename": d["model_filename"],
                "model_path": d["model_path"],
            }
            for i, d in enumerate(top_3)
        ],
    }


def main():
    parser = argparse.ArgumentParser(
        description="Compute representative best drops from a finished training run.")
    parser.add_argument("--output-log", required=True, type=Path,
                        help="Path to wandb run files/output.log")
    parser.add_argument("--index-csv", required=True, type=Path,
                        help="Path to drop_episodes/index.csv")
    parser.add_argument("--success-replay", required=True, type=Path,
                        help="Path to success_replay/{run_id}/")
    parser.add_argument("--step-range", required=True,
                        help="Run's step range as 'START,END' (filter old runs)")
    parser.add_argument("--run-id", required=True, help="WandB run id")
    parser.add_argument("--run-name", required=True, help="WandB run name")
    parser.add_argument("--output-json", type=Path, default=None,
                        help="Output JSON path (default: success-replay/REPRESENTATIVE_BEST.json)")
    args = parser.parse_args()

    step_range = tuple(int(x) for x in args.step_range.split(","))

    manifest = analyze(
        args.output_log, args.index_csv, args.success_replay,
        step_range, args.run_id, args.run_name,
    )

    output_json = args.output_json or (args.success_replay / "REPRESENTATIVE_BEST.json")
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    print(f"\n=== Manifest saved to {output_json} ===")
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
