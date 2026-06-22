---
date: 2026-06-22
tags: [experiment, dry-run, vision, handoff, altitude, detection-gate, v13]
status: complete
wandb_run: uqy7lmny (v1, offline) / dryrun_alt10_gated (v2, offline) — WANDB_MODE=offline
type: experiments
---

# Exp 008 — 핸드오프 윈도우 확장 (고도↑ 시도 → 탐지 게이트가 진짜 레버)

> **목표:** "X마커가 너무 늦게(드론이 거의 머리 위) 탐지돼 RL 핸드오프 후 학습 윈도우가 짧다."
> 사용자 가설 = **순항 고도를 높이면**(5→10 m) 카메라 footprint가 넓어져 마커를 일찍 탐지 → 윈도우↑.
> **결과: 고도는 레버가 아니었다.** 진짜 병목은 `vision_callback`의 200 px 공간 필터 + confidence 게이트 부재.
> 관련: [[research/detection_gate_vs_altitude]] · [[research/rl_rules]] Rule 13 · [[experiments/exp_007_iyhfy5ps_v13_eval]]

---

## 설정

| 항목 | 값 |
|------|-----|
| 스크립트 | `ros2 run rl_navigation train_sac --config hyperparams_v13.yaml --timesteps 1500` |
| 체크포인트 dir | `/workspace/ros2_ws/rl_dryrun_alt10` (**격리** — 메인 `rl_checkpoints/sac_drop_preempt.zip` 미손상) |
| WandB | offline (geometry 검증용, 대시보드 오염 방지) |
| 로그 | `/tmp/dryrun_alt10.log`(v1) · `/tmp/dryrun_alt10_v2.log`(v2) · `/tmp/episode_0.log`(mission_manager VISION) |
| 빌드 | `--symlink-install` (src 편집 live) |

---

## v1 — 고도만 10 m (기존 200 px 필터, conf 게이트 없음)

`mission_manager_node.py` `target_altitude` 5.0→10.0, `start_drift_max` 5→10(가드 정합).

| Ep | 드론 pos (ENU) | 핸드오프 d_xy | conf | bbox | 판정 |
|----|----------------|---------------|------|------|------|
| 1 | (2.8, 3.0) — 순항 시작점 | 10.8 m | 0.00 | (0,0) | **spurious** → health gate reject |
| 2 | (2.1, 2.6) — 순항 시작점 | 11.6 m | 0.00 | (0,0) | **spurious** → reject |
| 3 | (8.7, 8.7) — 마커 부근 | **2.7 m** | **0.94** | (264,123) | 정상 lock |

- **clean 핸드오프 = d_xy 2.7 m** — 5 m 베이스라인(~3.2–3.8 m)과 사실상 동일. **고도를 올려도 핸드오프 거리 안 늘어남.**
- ep 1·2는 순항 *시작*에서 spurious CRUISE→TRACKING(conf=0.00, d_xy≈home→target ~11 m). 넓어진 FoV cone이 시작 즉시 X-like 지면 false positive를 잡음.
- health gate(10 m)가 spurious를 정확히 걸러냄(설계대로) → 그러나 6 retry 소진 → abort.

### 왜 고도가 안 통하나 (기하적으론 "통해야" 함)
1. **마커 apparent size 절반** (10 m vs 5 m) → YOLO가 가까워져야(~2.7 m) lock → 탐지가 더 일찍이 아님(오히려 동등/늦음).
2. **200 px 공간 필터**가 중심 근처만 허용 → 고도 무관하게 핸드오프를 "거의 머리 위"로 클립.

→ 결론: **레버는 고도가 아니라 탐지 게이트.** 사용자와 합의 후 "10 m 유지 + 트리거 수정"으로 전환.

---

## v2 — 10 m + conf 게이트(0.5) + 공간 필터 200→300 px

`vision_callback`에 두 게이트 도입(둘 다 ROS 파라미터):
- `min_detection_conf=0.5` — 약한 탐지(지면 FP) reject. 기존엔 `z>0`면 무조건 accept.
- `detection_pixel_radius=300` (200→300) — 실제 off-center 탐지를 더 일찍 accept(윈도우↑). conf 게이트가 넓어진 영역을 지켜주므로 안전.

| 지표 | v1 (200 px, 게이트 없음) | v2 (300 px + conf 0.5) |
|------|--------------------------|------------------------|
| clean 핸드오프 d_xy | 2.7 m | **5.0–5.2 m** (~2× 윈도우) |
| spurious 전이 | 3 ep 중 2 | **0** |
| health gate / EKF-drift 발동 | 2 | **0** |
| real 탐지 conf | 0.94 | 0.93 |

### `episode_0.log` VISION 게이트 증거 (결정타)
- **REJECT conf:** `z=0.45 / 0.41 / 0.31 / 0.29` (< 0.50) — v1에서 spurious를 일으킨 바로 그 순항-시작 FP들. 이제 트리거 전 차단.
- **REJECT dist:** `361 px(conf 0.89) / 356 px(conf 0.91)` — 진짜 마커지만 너무 off-center → 가까워질 때까지 hold(정상).
- **ACCEPT:** real 마커 conf 0.73–0.95, **264–293 px off-center 포함** (구 200 px면 reject됐을 탐지 = 윈도우 확장의 출처).
- **깨끗한 분리:** FP ≤ 0.45, real ≥ 0.73, 게이트 0.50이 그 사이 간격에 안착.

---

## 코드 변경 (전부 미커밋 — 사용자가 직접 커밋 예정)

| 파일 | 변경 |
|------|------|
| `mission_manager_node.py` | `target_altitude` 5→10 m; 신규 param `min_detection_conf=0.5`·`detection_pixel_radius=300`; `vision_callback` conf 게이트 + param radius + accept/reject 진단 로깅 |
| `hyperparams_v13.yaml` | `start_drift_max` 5→10 m |
| `drone_drop_env.py` | default `start_drift_max` 5→10; 하드코딩 step-1 drift 가드(5.0)를 `_cfg_start_drift_max`로 배선(둘이 함께 스케일); 주석 정합 |

## 튜닝 노브 (향후)
- `detection_pixel_radius` 300→~360이면 핸드오프 더 일찍(~6–7 m)이나 프레임 가장자리(코너 ~400 px) 근접 → edge 불안정 위험. **300→5 m가 안전 sweet spot.**
- `min_detection_conf` 0.5→0.6이면 FP 마진↑ (real ≥0.73이라 여유).

## 결론 & 다음
- ✅ **윈도우 ~2배(2.7→5.0 m), spurious 0, EKF-drift 0** — dry-run PASS.
- ⚠️ 이건 **보상 공식 변경이 아니라 기하+탐지 변경** → fresh start 필수는 아니나, 정책이 ~3.5 m 시작에서 학습됐으므로 초반 재적응 예상.
- **다음:** 사용자 커밋 후 fresh full training(또는 iyhfy5ps 재개) → 첫 롤아웃 핸드오프 d_xy ~5 m 정상 + success_rate 점검.

→ 규칙화: [[research/rl_rules]] Rule 13 / [[research/detection_gate_vs_altitude]]
