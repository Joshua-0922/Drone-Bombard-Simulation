# RL Training Pipeline — Project Log

> **Branch:** `main` | **VM 프리엠션 후 컨텍스트 복구용 로그**

---

# 1. Current State

**업데이트:** 2026-06-20

### 활성 학습

| 항목 | 값 |
|------|-----|
| Run name | `rl_yolo_v13_terminal_reward` (iyhfy5ps) — **2026-06-20 157.7K/500K에서 SIGTERM stop (eval 위해)** |
| 상태 | ⏸️ stop (plateau, ep_rew_mean ~100, success ~82%, target_lost 0). `sac_drop_preempt.zip`+70MB replay 보존 → 재개 가능 |
| 평가 결과 | deterministic eval: 유효 ep 1–3 100% 성공(reward 124>학습 ~100). ep 4–13 **EKF divergence 흡수 루프**(시작 상태 결함, 정책 아님). → [[experiments/exp_007_iyhfy5ps_v13_eval]] |
| 로그 | `/workspace/train_v13.log` (학습) · `/workspace/eval_v13.log` (평가) |
| Timesteps | 500,000 목표 (157.7K에서 중단) |
| 수정 config | `arm_bail_timeout: 10.0 → **20.0**` (06-17 armdiag) |
| ⚠️ 인시던트 | armdiag dry-run이 v13 30K 체크포인트 파괴(YAML 중복 키) → 30K 재개 불가, fresh 재시작. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
| 검증 | fresh run 초기 윈도우: **bail 0 / late-EKF 14–15.5s ×3 전부 회복** (구 10s면 bail) |
| ⚠️ OPEN 이슈 | YOLO `target_lost_rate` ~29% bimodal; teleport 후 EKF 13–16s 재수렴 자체 |

**처리량 진단(06-17):** v13(46y4xtiw)이 ~10h에 29.9K(6%)뿐 — fps≈0.83, ETA ~6.5일.
지배적 싱크 = `PX4 not armed after 10s` early-bail. armdiag dry-run(xgzum51v)으로
`pre_flight_checks_pass` 재수렴을 계측: **bimodal 0.0s(7/12) / 13–16s(5/12 ≈ 42%)**.
25s 창에서 bail 0 / SUCCESS 4 → v12의 10s 컷이 복구 직전 단두대질이었음 규명.
**Fix: `arm_bail_timeout` 10→20.** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / [[research/cruise_timeout_arming]] / Rule 11.



### 활성 보상 공식 (2026-03-22 패치, 미적용)

| Layer | 공식 | 파라미터 |
|-------|------|---------|
| R1 Safety | `−10` if alt < 2 m; `−8` if speed > 20 m/s | step > 20 이후 |
| R2 Stability | `−0.05 − 0.05‖ω‖² − 0.05‖Δa‖²` | w_time=0.05 (패치됨) |
| R3 Distance | `1.0 × (d_prev − d_xy)` | 선형, 포화 없음 |
| R3 Orient | `1.0 × cos θ × min(v_xy/2, 1)` | speed gate (안티-밀킹 패치) |
| R4 Drop | `50·exp(−5·d_err) [+100 jackpot if d≤0.1m]` | auto-drop at d_xy≤0.5m |
| Truncation | `−50` if step=500 and not dropped | 패치됨 |

### 학습 환경

| 파라미터 | 값 |
|---------|-----|
| Algorithm | SAC, `net_arch=[256,256]`, `device=cuda` |
| `num_envs` | 1 (Gazebo lockstep 병목으로 고정) |
| `total_timesteps` | 1,000,000 |
| RTF | **2** (RTF dry-run 결과: RTF=2 최적, avg 59.5 fps) |
| 예상 fps | **~60** |

### 체크포인트

- **보상 패치 전 마지막 정상:** `sac_drop_preempt.zip` (run `8otphxy8`, ~114K steps)
- **다음 학습:** Fresh start 필요 (보상 공식 변경으로 재개 금지)

---

# 2. Recent Progress

- **2026-06-20:** **v13 정책 평가 + 학습 stop.** iyhfy5ps가 157.7K(~32%)에서 **plateau**(ep_rew_mean ~100 80K부터 평탄, success ~82%, target_lost 0, fps≈0). eval을 위해 학습 **SIGTERM graceful stop**(preempt+70MB replay 보존, 재개 가능). deterministic eval(`sac_drop_preempt.zip`, 20-ep 요청/13 실행): **ep 1–3 전부 0.8m 성공(reward 126/114/132, step 41–72; 평균 124 > 학습 ~100) — 정책 양호.** **ep 4–13 전부 step1 EKF divergence(`d_xy≈11.9m`=home→target) → −15 truncation 흡수 루프** (연속 full-restart가 EKF 수렴 못 시킴; 카메라는 마커 봄=TRACKING OK이나 EKF position만 발산; 06-17 EKF 재수렴과 동일 뿌리, eval에서 누적 악화). **harness 결함 2종:** `evaluate.py`가 env 미emit 키 `info['drop_error_actual_m']` 의존 → miss-distance/CEP/drop-speed 전부 NaN; v13 env는 0.8m 성공원 종료(탄도 투하 미모델링) → CEP 비실재 → success-rate/step-to-reach로 평가해야. **다음:** 에피소드 시작 EKF↔카메라 health gate(drift면 retry) + evaluate.py 지표 교체 후 재평가. → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12
- **2026-06-17 (오후):** **v13 fresh 재시작(iyhfy5ps) + 인시던트.** 30K 재개를 시도했으나 armdiag dry-run이 **YAML 중복 `checkpoint_dir` 키**(격리 경로가 main에 덮임)로 메인 dir에서 실행되어 v13 30K 체크포인트 5개 삭제 + preempt를 599-step으로 덮음 → **30K 디스크 복구 불가**. 사용자 결정으로 **fresh 재시작**(arm_bail=20, 0→500K). 프로덕션 검증: 초기 윈도우 **bail 0, late-EKF 14.1/14.8/15.5s ×3 전부 회복**(구 10s면 30% bail). 재발방지: 파괴적 fresh-start 전 startup `Checkpoints:` 로그로 격리 검증. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]]
- **2026-06-17:** **arm_bail 처리량 병목 진단 & 수정.** v13(46y4xtiw)이 ~10h에 6%(29.9K)뿐, ETA ~6.5일 — 지배적 싱크가 `PX4 not armed after 10s` bail임을 확인. 컨트롤러에 `PREFLIGHT-PASS` dt 계측 추가 후 격리 dry-run(`hyperparams_v13_armdiag.yaml`, `arm_bail_timeout=25s`, offline). **결과: EKF 재수렴 bimodal — 0.0s(7/12) / 13–16s(5/12 ≈ 42%), 25s에서 bail 0 / SUCCESS 4.** v12의 10s 컷이 recoverable-with-time을 full-restart-only로 오판하고 복구 직전(3–6s 전) 단두대질했음. **Fix: `hyperparams_v13.yaml` arm_bail_timeout 10→20.** v13은 SIGTERM emergency save로 30K+리플레이 보존, 재개 가능. → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11
- **2026-06-15:** **Arming-rejection throughput fix.** `rl_yolo_v11_cam_fix`(k1uqgs8i) 분석 → 443 CRUISE 타임아웃의 근본 원인이 teleport 후 stale EKF arm 거부(28.2% NEVER ARMED, 전부 attempt 1/3)임을 규명. 수정 3종 적용: (#3) `/fmu/out/vehicle_command_ack` arm 거부 사유 로깅, (#2) `pre_flight_checks_pass` 게이팅, (#4) `arm_bail_timeout=10s` early-bail → 즉시 full infra restart. colcon build clean + dry-run(400 step, 0 타임아웃) 검증 후 fresh run `rl_yolo_v12_arm_fix`(500K) 기동. ⚠️ 정정: `cruise_poll_timeout`은 이미 20.0s(이전 "60s"는 fallback 기본값 오독). ⚠️ OPEN: YOLO target_lost_rate ~29% bimodal 미해결. → [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]]
- **2026-06-13:** v7 패치 적용 후 fresh run `rl_yolo_v7_drift_guard` (WandB: `7lhjy40o`) 시작. EKF drift guard (step1 d_xy>5m→truncate), proximity 4m→2.5m, penalty_target_lost -0.5→-0.1, stagnation_start_step 400→50.
- **2026-06-12:** Vision 기반 RL 학습 인프라 완성. EKF East 반전 버그 2종 수정. fresh run `rl_yolo` (WandB: `45l8vkw5`) 121K steps. **분석: target_lost_rate=1.0 원인 = EKF drift (dominant) + 카메라 FOV gap 3차진 이후 2.89m vs 시작 d_xy 3.5m).** run 폐기.
- **2026-04-16:** RTF dry-run 3종 완료 (RTF 1/2/4). **RTF=2 최적** (avg 59.5 fps, 61s/4Kstep). RTF=4는 Python 병목으로 역전. Exp 002 RTF=2로 결정.
- **2026-04-16:** WandB API key 영구 연결 (`/opt/drone-bombard/.wandb.env`, `--env-file` 방식). Docker image `drone-bombard-px4built:latest` — PX4 빌드 + 커스텀 airframes 4016-4019 포함.
- **2026-04-14:** Obsidian 연구 비서 시스템 초기화. `notes/` 구조 구축, CLAUDE.md + RL_Project_Log.md 간소화.
- **2026-03-22:** 보상 해킹 분석 → 4개 anti-milking 패치 적용 (학습 대기 중).
- **2026-03-20:** 선형 거리 보상 도입 (지수 포텐셜 교체), CRUISE retry, 3중 물리 폭발 방어.
- **2026-03-20:** Method A (1-World-4-Payload) 아키텍처 완성 및 dry-run 통과 (31 fps).
- **2026-03-19:** 자기관리 인프라 안정화 (z=0 스폰, COM_OF_LOSS_T=10s, fps=30-31).

---

# 3. Remaining Tasks (Next Steps)

- [x] **Vision 기반 RL 인프라 완성** — YOLO + SAC 시각 서보잉 파이프라인 구축
- [x] **EKF East 반전 버그 수정** — proximity target + RL env reward target 좌표 수정
- [x] **WandB `45l8vkw5` 100+ 에피소드 분석** → target_lost=1.0, EKF drift 확인 → 폐기
- [x] **EKF drift 방어 로직** — step 1에서 d_xy>5m이면 즉시 truncate (ekf_drift)
- [x] **fps 개선 — CRUISE 타임아웃 근본 원인 수정** — arm 게이팅(#2) + early-bail(#4). v12에서 효과 검증 중 → [[research/cruise_timeout_arming]]
- [x] **v12/v13 arm 처리량 재진단** — `ARM REJECTED` 0회(게이팅 작동), 실제 병목은 EKF 재수렴이 10s bail 초과(13–16s). **arm_bail 10→20s 적용** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11
- [x] **v13 재시작** — 30K 체크포인트 인시던트로 소실 → **fresh 재시작**(iyhfy5ps, arm_bail=20). 검증: bail 0, late-EKF 회복 확인.
- [ ] **v13(iyhfy5ps) 추세 점검** — 첫 롤아웃 후 success_rate 발생 + ep_len/env/ep_reward 추세 + 전체 bail율(구 ~21/h 대비)
- [ ] **(장기) teleport EKF 재수렴 단축** — 13–16s 재수렴 자체 줄이기(명시적 EKF reset 등). 타임아웃은 증상 완화일 뿐
- [ ] **⚠️ YOLO target_lost_rate ~29% bimodal 해결** — per-step 트리거가 전부/전무로 분리(악화 0.24→0.35). obs[9-11] zeroed + `-10` 페널티. 미해결 (이번 세션 범위 밖)
- [ ] **PX4 로그 /dev/null 리다이렉트** — `/tmp/px4_{i}.log` 100+ MB 증가 방지

---

# 4. Training History

> **전체 히스토리:** `notes/experiments/training_history.md`
> **개별 실험 노트:** `notes/experiments/exp_NNN_*.md`

최근 주요 runs:

| 날짜 | Run ID | Steps | 요약 |
|------|--------|-------|------|
| 2026-03-20 | 8otphxy8 | 114K | 선형 거리 보상 + CRUISE retry. 마지막 정상 베이스라인. |
| 2026-03-22 | — | — | 보상 패치 적용 (학습 없음). Fresh start 대기 중. |
| 2026-04-16 | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | RTF 1/2/4 dry-run. RTF=2 최적 (59.5 fps). |
| 2026-06-12 | esmtny0a | 33K | Vision SAC. proximity 버그로 128ep stagnation. 폐기. |
| 2026-06-12 | 45l8vkw5 | 121K | rl_yolo. target_lost=1.0 전구간. 원인: EKF drift + FOV gap. 폐기. |
| 2026-06-13 | 7lhjy40o | 진행 중 | rl_yolo_v7_drift_guard. EKF drift guard + proximity 2.5m + penalty_lost=-0.1. |
| 2026-06-14 | k1uqgs8i | ~42K | rl_yolo_v11_cam_fix. 학습 개선(env/ep_reward 20→54, 404 successes)이나 443 CRUISE 타임아웃으로 중단. |
| 2026-06-15 | rl_yolo_v12_arm_fix | 진행 중 | Arming-rejection throughput fix (arm 게이팅 + early-bail). dry-run 0 타임아웃 검증 후 fresh 기동. |
| 2026-06-17 | 46y4xtiw | ~30K (중단) | rl_yolo_v13_terminal_reward. ~10h에 6%뿐(fps≈0.83) — arm_bail 병목 진단 위해 graceful stop. 재개 대기. |
| 2026-06-17 | xgzum51v (offline) | 1000 (dry-run) | v13_armdiag. EKF 재수렴 bimodal(0s/13–16s) 계측 → arm_bail 10→20s 수정. Rule 11. ⚠️ 이 dry-run이 YAML 중복 키로 v13 30K 체크포인트 파괴. |
| 2026-06-17 | iyhfy5ps | 진행 중 (fresh 0→500K) | rl_yolo_v13_terminal_reward **fresh 재시작** (arm_bail=20). 30K 인시던트 후. 검증: bail 0, late-EKF 14–15.5s ×3 회복. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
