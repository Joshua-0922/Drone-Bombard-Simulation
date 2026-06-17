---
date: 2026-04-14
tags: [experiment, history, wandb, SAC]
status: active
type: reference
---

# 전체 학습 히스토리 (RL_Project_Log.md에서 이전)

> **Append-only.** 새 run은 아래 테이블에 추가하고 개별 `exp_NNN_*.md` 파일도 생성.

---

| 날짜 | Run ID | Steps | Mean Drop Error | 요약 |
|------|--------|-------|-----------------|------|
| 2026-03-12 | vekkz83a | 386 | — | 첫 run 중단; 0 FPS (60 s/episode RTF 고정). Phase 7 최적화 적용. |
| 2026-03-13 | vekkz83a | resumed | — | preempt + replay buffer 재개. RTF=0, headless. |
| 2026-03-17 | apax52d7 | ~3K | — | dartsim 크래시 수정, EKF yaw fix (COM_ARM_WO_GPS=1). ~25 steps/sec. |
| 2026-03-17 | 27mbu6qk | 49,242 | — | replay buffer 비호환 수정 후 재개. fps 2→4. ep_rew=-367. |
| 2026-03-18 | 9nbwg71r | 53,428 | — | Phase 12: OFFBOARD retry race 수정. fps 4→23. |
| 2026-03-18 | 2h1cvmer | 78,200 | — | Phase 13 초기: multi-instance 수정, stale shm, NaN obs 크래시. |
| 2026-03-18 | 53xx3o8u | 80,292+ | — | Phase 13 최종: PX4_GZ_MODEL_POSE 수정, EKF warmup 5s, RTF=1. **fps=12 안정.** |
| 2026-03-18 | dy97unuj | 546K–560K | — | **불량 run** — px4_msgs source 누락, 모든 에피소드 IDLE 고착. 종료. |
| 2026-03-18 | izf10080 | 564K–568K | — | **오염 버퍼 run** — ep_rew=-1,900. 종료. |
| 2026-03-18 | pbpqa0rp | 0 (fresh) | — | **Phase 1 Fresh Start.** manual drop 비활성, WandB callback 추가. ep_rew=-91 @6K. |
| 2026-03-19 | a9f6lk57 | — | — | **Debug run** — 자기관리 인프라 첫 시도. z=5 스폰 → ODE crash. 종료. |
| 2026-03-19 | 6dopfyjn | ~96K–102K | — | **Debug run** — world reset 제거. COM_OF_LOSS_T race 수정. |
| 2026-03-19 | nynxn6b5 | 102K+ | — | **자기관리 인프라 안정.** z=0 스폰, COM_OF_LOSS_T=10s. **fps=30-31.** |
| 2026-03-20 | ljbn3wfg | 8K (dry-run) | — | **Method A dry-run.** PX4_SIM_MODEL=gz_x500_bombard_r0, px4_msgs fix. **31 fps, 16 에피소드, 0 ODE.** |
| 2026-03-20 | cj3ytvq2 | 20K | — | **Method A 생산 학습 FRESH START.** num_envs=1, RTF=1, 33 fps. ep_rew=−545 @20K. |
| 2026-03-20 | mjfet61f | 52K | — | WandB 보상 모니터링 추가. **KILLED** — d_xy=1.98e11 물리 폭발. 버퍼 오염. |
| 2026-03-20 | 53samoqz | 95K | — | 폭발 복구. physics explosion guard 추가. **KILLED** — WandB 콜백 글리치 값 누산 버그. |
| 2026-03-20 | naf4zyhm | 104K | — | 3중 폭발 방어 적용. **KILLED** — `mean_rew_dist=0` (지수 포텐셜 포화 k1=1.0). |
| 2026-03-20 | 8otphxy8 | 114K | — | **선형 거리 보상 + CRUISE retry.** 마지막 정상 베이스라인. → [[experiments/exp_001_8otphxy8_linear_reward]] |
| 2026-03-22 | — | — | — | **보상 패치 적용 (학습 없음).** anti-milking, w_time 5×, truncation penalty. Fresh start 필요. → [[experiments/exp_002_reward_shaping_patches]] |
| 2026-04-16 | mtx7ud6o / x8jq9fsy / u8w3xn0w | 5500×3 (dry-run) | — | **RTF dry-run 비교.** RTF 1/2/4 순차 측정. **RTF=2 최적 (avg 59.5 fps, 61s/4Kstep).** RTF=4는 Python 병목으로 역전. → [[experiments/exp_003_rtf_dryrun]] |
| 2026-06-12 | esmtny0a | 33K (128 ep, stagnation) + 계속 중 | — | **Vision YOLO 접근 + EKF East 반전 버그 수정.** proximity_ned 버그([10,11]→[10,-11]) 발견·수정. 수정 전 128 에피소드 전체 stagnation. 수정 후 proximity d_xy=3.9m 작동 확인. → [[experiments/exp_004_rl_yolo_debug_vision]] |
| 2026-06-14 | k1uqgs8i | ~42K (17.6h, 중단) | best d_xy 0.68m | **rl_yolo_v11_cam_fix.** 학습 개선 중(env/ep_reward 20→54, reached_close 93%, 404 successes). 그러나 443 CRUISE 타임아웃(28.2% NEVER ARMED)으로 ~5.5h 낭비. SB3 ep_rew_mean 하락은 ep_len 붕괴(151→36.5)로 인한 착시. → [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] |
| 2026-06-15 | rl_yolo_v12_arm_fix | 진행 중 (500K 목표) | — | **Arming-rejection throughput fix.** teleport 후 stale EKF arm 거부 근본 원인 규명·수정 3종(arm_ack 로깅 #3, pre_flight_checks_pass 게이팅 #2, arm_bail_timeout=10s early-bail #4). dry-run 0 타임아웃 검증 후 fresh 기동. ⚠️ OPEN: YOLO target_lost_rate ~29% bimodal 미해결. → [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] / [[research/cruise_timeout_arming]] |
| 2026-06-16 | rl_yolo_v12_arm_fix (진단) | 93K 시점 | — | **정체 진단.** ep_rew≈-20.5(추세 없음), ep_len 40→16 감소, success **0회**, 그러나 d_xy~1.0m 일관 도달. 근본 원인 = **종단 overshoot 트랩**: 정하방 카메라(commit 24135e9) → 핸드오프 ~1m → overshoot 가드 step 1부터 무장 + 8 m/s 액추에이터로 0.5m 성공원 못 꿰. v11→v12 회귀(v11은 전방카메라로 활주로 확보, 404 successes). → [[research/terminal_overshoot_trap]] |
| 2026-06-16 | rl_yolo_v13_terminal_reward | **준비됨 (미실행)** | — | **종단 보상 재설계 (config-only, Fresh Start 필요).** overshoot_close_threshold 1.5→0.6, margin 1.5→2.0, penalty -20→-10, success_radius 0.5→0.8, action_vx/vy 8/5→4/3, w_proximity 0.3→0.6, proximity_radius 5.0→2.0. config: `hyperparams_v13.yaml`. → [[research/terminal_overshoot_trap]] |
| 2026-06-16 | rl_yolo_v13_terminal_reward (t0teou5w) | 300 (dry-run) | — | **dry-run PASS.** v12 SIGTERM 종료 + leftover gz/MicroXRCE 정리 후 fresh 기동. 에러/크래시/NaN 0, overshoot truncation 0. **SUCCESS 1회 (d_xy=0.79m≤0.8 @step64)** — v12는 success 0이었음. ep_len 16→64(트랩 제거로 정착 가능). arm-reject early-bail 1회(정상 처리). 300<learning_starts라 gradient update 없음 → 인프라·보상 기하 검증용. **다음: Fresh full training.** |
| 2026-06-17 | rl_yolo_v13_terminal_reward | 진행 중 (500K 목표) | — | **v13 Fresh full training 기동.** dry-run PASS 후 `--config hyperparams_v13.yaml --timesteps 500000` fresh start. `/workspace/train_v13.log`. 정상 startup(PX4 ready, training started, 에러 0). ⚠️ 확인 필요: 첫 롤아웃 후 success_rate 발생 + ep_len/env/ep_reward 추세. (OPEN: YOLO target_lost ~29%.) → [[research/terminal_overshoot_trap]] |
| 2026-06-17 | rl_yolo_v13_terminal_reward (46y4xtiw, 중단 @30K) | ~29.9K / ~10h, ep_rew≈95 | — | **처리량 진단·중단.** ~10h에 29.9K(6%)뿐 — fps≈0.83, ETA ~6.5일. 지배적 싱크 = `PX4 not armed after 10s` bail (210회/401 restart). 30K 체크포인트 + 리플레이(SIGTERM emergency save) 보존 후 graceful stop. → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] |
| 2026-06-17 | v13_armdiag_dryrun (xgzum51v, offline) | 1000 (조기 종료) | — | **arm_bail 진단 dry-run.** 컨트롤러에 `PREFLIGHT-PASS` dt 계측 추가 + `arm_bail_timeout=25s`. 결과: EKF 재수렴 **bimodal 0.0s(7) / 13–16s(5, ~42%)**, **bail 0 / SUCCESS 4**. → v12의 10s 컷이 복구 직전 단두대질이었음 규명. **Fix: `hyperparams_v13.yaml` arm_bail_timeout 10→20.** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / [[research/cruise_timeout_arming]] / [[research/rl_rules]] Rule 11 |
