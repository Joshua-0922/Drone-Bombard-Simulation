---
date: 2026-04-14
tags: [index, dashboard]
status: active
type: index
---

# 드론 정밀 투하 연구 — Obsidian 대시보드

> **프로젝트:** CCIP 기반 잔차 강화학습 활용 드론 정밀 투하
> **스택:** ROS 2 Humble · Docker · PX4 SITL · SAC (SB3) · Gazebo Harmonic · L4 GPU

---

## 현재 상태 (2026-07-03)

- **병행 트랙 — Isaac Lab migration (`feat/isaac-env-migration` 브랜치, `/opt/drone-bombard/isaac-worktree`):** **exp_013 — 첫 프로덕션 PPO 학습 완주 + 진단 완료 (2026-07-03).** 2048 envs×1000 iters(65.5M steps, 43분, wandb `wcjklw7a`) → **deterministic 200-ep eval = 36%**, d_xy_min 1.4m plateau. 시작 직후 **비전 사멸 버그**(env-origin 프레임 혼용, [[errors/err_20260703_vision_env_origin_frame]]) 발견·수정 후 재기동. 실패 원인 3종 규명: ①analytic conf 거리감쇠 누락→고도 상승 farming(max_alt 33%, Rule 17) ②farmer(+225)>finisher(+121) 보상 불균형(Rule 18a) ③noise_std 0.8→3.92 폭주(Rule 18b). **07-04 forensics 재정정: 리셋 속도킥은 프로세스당 1회(첫 물리 substep, m_eff=0.02504kg 계측 확정)로 실증 — 학습 오염 사실상 없음, max_alt 27-43%는 iter ~200 창발 학습된 attractor(비전 farming 1차 가설 복권), 36% 수치 유효.** entropy 실측: 실행 속도 궤적 σ-불변(vel-Δ σ3.9/det=1.01×) → 공짜 entropy 확정. **다음: exp_014 = conf 거리감쇠 + reward_success 300 + entropy_coef 0 + 킥 위생수정, fresh ([[research/exp014_ablation_protocol]] 참조).** 온보딩 문서 3종(`isaac_lab_reward_tuning`/`wandb_guide`/`experiment_workflow`) 신설. 상세: [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]].
- **메인 트랙 — SAC (jekyun 브랜치, Gazebo/PX4):**

## 현재 상태 (2026-07-01)

- **알고리즘:** SAC, `net_arch=[256,256]`, L4 GPU
- **현재 학습:** ▶️ `rl_yolo_v15_bc_stable` — **Fresh 0→300K 진행 중** (tmux `rl_train`, wandb online). wobble 교정(B+C 보상 댐핑 + LPF 0.4) 적용. Fresh Start가 v14 5체크포인트 삭제 → **`rl_checkpoints/v14_backup/`에 백업**(195K 포함).
- **이번 세션 (2026-07-01) — RL wobble 진단·교정:** 사용자 관찰(10 m 핸드오프 후 RL 인수하나 wobble). eval `deterministic=True`라 **탐험 노이즈 아님 = 학습된 bang-bang 정책.** LPF A/B: PX4 수신 속도명령 **jerk RMS 2.92→1.61(−45%)**, 평균 속력 1.13×(안 느려짐) → **smoothness-control 문제 확정.** 교정: (B) 근접-게이팅 속도 댐핑 `w_vel=0.15/R=4`, (C) `w_ang_vel 0.05→0.15`·`w_action_smooth 0.05→0.20`, 로직 LPF `velocity_lpf_alpha=0.4`(학습==배포). dry-run PASS → v15 fresh 기동. → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15
- **이전 학습:** ⏸️ `rl_yolo_v14_softreset` (byxyaf4d) — 196.5K/500K에서 SIGINT stop, `rl_checkpoints/v14_backup/`에 보존. v13(iyhfy5ps 157.7K)는 `rl_checkpoints/archive/v13_iyhfy5ps_157k_20260622`에 백업.
- **이번 세션 (2026-06-23) — v14 195K eval = 65%(13/20):** plateau(70K부터 reward 평탄) 확인 후 stop → clean 20-ep deterministic eval **65%** (v13 80% 대비 회귀). 실패 7 전부 final-approach stagnation(0.5–0.8m, 0.50m gate 직전), EKF 귀책 0. **Soft reset 장기검증 ✅**(3096 resets, soft ~91%, EKF bounded, no teleport) → Rule 14 검증완료. 회귀=정책 미성숙(39% budget). 비디오 3/3 success 캡처(`rl_eval_results/v14_195k_flight_annotated.mp4`+`_raw.mp4`). v14 commit 결정 대기. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- **이번 세션 (2026-06-22) — 리셋 처리량 ~3.9× (soft reset):**
  - 문제: v14 fps≈2, ETA ~2.5일. 에피소드마다 CRUISE timeout(~42s) + full restart(~22s).
  - **근본원인 규명:** teleport+disarm 후 PX4 **EKF 추정기 재수렴** 대기(`pre_flight_checks_pass=False`). fresh restart도 동일.
  - **EKF2_GPS_CHECK 0 A/B = 음성**(COM_ARM_WO_GPS라 GPS는 게이트 아님; 실제 게이트=EKF 수렴, 바이패스 param 없음). param/timeout 레버 고갈.
  - **Soft reset(teleport 회피) = 성공:** flyable이면 날아서 출발점 복귀 후 FSM만 재시작. **throughput 0.93→3.61/min(~3.9×), fps 2→9, reset 65s→11s, soft 100%(32/32), EKF d_xy 안정 4.5–5.8m(발산 없음).** ETA ~2.5일→~15h.
  - **코드 미커밋**, full run으로 장기 검증 중(EKF drift bounded? fallback율?).
  - 상세: [[experiments/exp_009_softreset_throughput]] / [[research/reset_throughput_bottleneck]] / Rule 14
- **이전 (2026-06-22) — 핸드오프 윈도우 확장:**
  - 사용자 요청: "X마커가 늦게(거의 머리 위) 탐지돼 RL 핸드오프 후 학습 윈도우가 짧다." 가설 = 순항 고도↑.
  - **고도만 10 m(v1 dry-run): 실패.** clean 핸드오프 여전히 d_xy 2.7 m(베이스라인 동급) + 순항-시작 spurious(conf=0.00, d_xy≈11 m) → health gate abort. 원인: 마커 apparent size 절반(YOLO 늦게 lock) + 200 px 필터가 핸드오프를 머리 위로 클립. **고도는 레버 아님.**
  - **10 m + 탐지 게이트 수정(v2): 성공.** `vision_callback` conf 게이트(`min_detection_conf=0.5`) + 공간 필터 200→300 px(`detection_pixel_radius`). **핸드오프 2.7→5.0 m(윈도우 ~2배), spurious 0, EKF-drift 0.** VISION 로그: FP conf 0.29–0.45 reject / real 0.73–0.95 accept.
  - **코드 변경 미커밋**(사용자 직접 커밋 예정). 보상 공식 아님(기하+탐지) → fresh 필수 아니나 초반 재적응 예상.
  - 상세: [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13
- **이전 세션 (2026-06-20) — v13 정책 평가:**
  - 학습 점검: 157.7K(~32%), **ep_rew_mean ~100 plateau(80K부터 평탄), success ~82%, target_lost 0**, fps≈0(restart 병목).
  - plateau 확인 → eval 위해 학습 SIGTERM stop → deterministic eval(`sac_drop_preempt.zip`, 20-ep 요청/13 실행).
  - **정책 양호:** 깨끗하게 시작한 ep 1–3 전부 0.8m 성공(reward 126/114/132, step 41–72; 평균 124 > 학습 ~100).
  - **eval EKF divergence 흡수 루프:** ep 4–13 전부 step1 `d_xy≈11.9m`(=home→target) → −15 truncation. 카메라는 마커 봄(TRACKING OK)이나 EKF position만 발산. 연속 full-restart가 EKF 수렴 못 시킴 → 자체 회복 불가. **정책 아님, 시작 상태 결함** (06-17 EKF 재수렴과 동일 뿌리).
  - **harness 결함 2종:** `evaluate.py` miss-distance/CEP=NaN(env 미emit 키); v13 env는 0.8m 종료(탄도 투하 없음) → CEP 비실재.
  - **다음:** 에피소드 시작 EKF↔카메라 health gate(drift면 retry) + `evaluate.py` 지표(success-rate/step) 교체 후 재평가.
  - 상세: [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12

### 이전 세션 (2026-06-17) — v13 처리량 병목 진단 & 수정
- **이번 세션 (2026-06-17) — v13 처리량 병목 진단 & 수정:**
  - v13(46y4xtiw) ~10h에 29.9K(6%)뿐, fps≈0.83, ETA ~6.5일. 지배적 싱크 = `PX4 not armed after 10s` bail.
  - **진단(armdiag dry-run, xgzum51v):** 컨트롤러에 `PREFLIGHT-PASS` dt 계측 추가 + `arm_bail_timeout=25s`.
    EKF 재수렴이 **bimodal — 0.0s(7/12) 또는 13–16s(5/12 ≈ 42%)**. 25s 창에서 **bail 0 / SUCCESS 4**.
  - **결론:** v12의 `arm_bail_timeout=10s`가 복구(13–16s) 직전에 멀쩡한 PX4를 단두대질 → full restart 강제.
    stuck-EKF는 *full-restart-only가 아니라 recoverable-with-time*.
  - **Fix:** `hyperparams_v13.yaml` `arm_bail_timeout: 10.0 → 20.0`.
  - **⚠️ 인시던트:** armdiag dry-run이 **YAML 중복 `checkpoint_dir` 키**로 메인 dir에서 실행 → v13 30K 체크포인트 파괴(복구 불가). → fresh 재시작(iyhfy5ps). [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]]
  - **프로덕션 검증(iyhfy5ps):** bail 0, late-EKF 14.1/14.8/15.5s ×3 전부 회복(구 10s면 bail).
  - 상세: [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / [[research/cruise_timeout_arming]] / Rule 11
  - ⚠️ **여전히 OPEN:** YOLO `target_lost_rate` ~29%; teleport 후 EKF 13–16s 재수렴 자체 (별도 과제)

### 이전 세션 (2026-06-16)
- **방식:** Vision 기반 — YOLO X마커 탐지 → TRACKING, RL이 시각 서보잉 학습
- **이번 세션 (2026-06-16) — v12 정체 진단 + v13 종단 보상 재설계:**
  - v12(93K): ep_rew≈-20.5(추세 없음), ep_len 40→16, success **0회**, d_xy~1.0m는 잘 도달
  - 근본 원인: **종단 overshoot 트랩** — 정하방 카메라(commit 24135e9)로 핸드오프가 ~1m가 되면서
    overshoot 가드가 step 1부터 무장 + 8 m/s 액추에이터가 0.5m 성공원을 지나침 → 매 에피소드 -20
  - v11→v12 회귀 (v11은 전방 카메라로 접근 활주로 확보 → 404 successes)
  - **v13 수정 (config-only, Fresh Start 필요):** overshoot threshold 1.5→0.6, success_radius 0.5→0.8,
    action vx/vy 8/5→4/3, w_proximity 0.3→0.6, radius 5→2 → `hyperparams_v13.yaml`
  - 상세: [[research/terminal_overshoot_trap]] / Rule 10
  - ⚠️ **여전히 OPEN:** YOLO `target_lost_rate` ~29% bimodal — 미해결
- **선행 (2026-06-15):** Arming throughput fix — teleport 후 stale EKF arm 거부 수정 → [[research/cruise_timeout_arming]]
- **Phase 1 계획:** CCIP 기반 자율 접근 비행 제어기 → [[research/phase1_plan]]

---

## 실험 현황

| # | Run ID | Steps | 상태 | 비고 |
|---|--------|-------|------|------|
| 001 | 8otphxy8 | 114K | ✅ 완료 | 선형 보상 + CRUISE retry |
| 002 | — | 0 | ✅ 완료 | 보상 패치 적용 (학습 없음) |
| 003 (dry-run) | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | ✅ 완료 | RTF 1/2/4 비교 → RTF=2 최적 |
| 004 | esmtny0a | 33K+ | ✅ 폐기 | Vision YOLO 접근 + EKF East 버그 수정 → [[experiments/exp_004_rl_yolo_debug_vision]] |
| 005 | rl_yolo_v12_arm_fix | 0→500K | ⛔ 중단 | Arming fix는 작동하나 종단 overshoot 트랩으로 success 0 → v13으로 대체 → [[research/terminal_overshoot_trap]] |
| 006 | rl_yolo_v13_terminal_reward (46y4xtiw→iyhfy5ps) | 0→500K | 🔄 fresh 재시작 | 종단 보상 재설계. 46y4xtiw 30K에서 처리량 진단 위해 stop → ⚠️ armdiag dry-run이 30K 체크포인트 파괴 → **fresh 재시작 iyhfy5ps (arm_bail=20)** → [[research/terminal_overshoot_trap]] / [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
| 006b (dry-run) | v13_armdiag_dryrun (xgzum51v) | 1000 | ✅ 완료 | arm_bail 진단: EKF 재수렴 bimodal(0s/13–16s), 25s에서 bail 0. **Fix: arm_bail_timeout 10→20** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11 |
| 007 (eval) | iyhfy5ps (157.7K 평가) | 13 ep | ✅ 완료 | deterministic eval. 유효 3-ep 100% 성공(reward 124>학습 ~100); ep 4–13 EKF divergence 흡수 루프(시작 상태 결함). harness: evaluate.py NaN, v13 탄도 투하 없음 → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12 |
| 008 (dry-run) | dryrun_alt10 (uqy7lmny / _gated, offline) | 1500×2 | ✅ 완료 | 핸드오프 윈도우↑. 고도만 10 m=실패(레버 아님), 10 m+탐지 게이트(conf 0.5 + 200→300 px)=성공(핸드오프 2.7→5.0 m, spurious 0). 미커밋 → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13 |
| 009 | EKF A/B + softreset (byxyaf4d) | proto+full | ✅ 완료 | 리셋 처리량 ~3.9×. EKF param A/B=음성, **soft reset(teleport 회피)=성공**(0.93→3.61 handoffs/min, fps 2→9, reset 65s→11s, soft 100%, EKF 안정). → [[experiments/exp_009_softreset_throughput]] / [[research/reset_throughput_bottleneck]] / Rule 14 |
| 010 (eval) | rl_yolo_v14_softreset (byxyaf4d, 195K) | 20 ep | ✅ 완료 | **195K eval = 65%(13/20)**, v13 80% 대비 회귀(실패 전부 final-approach stagnation). EKF 귀책 0. **Soft reset 장기검증 ✅**(3096 resets, soft ~91%, EKF bounded) → Rule 14 검증완료. 비디오 3/3 success 캡처. commit 결정 대기. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] |
| 011 | wobble A/B + v15_bc_stable | eval×2 + dry-run + 0→300K | ▶️ 학습 중 | **RL wobble = smoothness-control 문제 확정.** LPF A/B: 속도명령 jerk RMS 2.92→1.61(−45%), 평균 속력 불변. 교정(B 근접 속도 댐핑 + C smoothness↑ + LPF 0.4) dry-run PASS → v15 fresh 기동(v14 백업). → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15 |
| 012 | isaac_migration_phase2 (`feat/isaac-env-migration`) | 코드만 (미학습) | ✅ 코드 완료 | **Isaac Lab env+PPO 이식.** `isaac_lab/` 신설, v13/v15 obs·action·리워드·터미네이션 상수 그대로 포팅(parity 표 포함), SAC→PPO(rsl_rl), target/spawn 랜덤화 신규, vision=analytic+YOLO-eval 이원화. `pytest test_math.py` 29/29 통과. L4 VM 미기동 → env 스모크 미실행. → [[experiments/exp_012_isaac_migration_phase2]] / [[research/isaac_velocity_controller]] |
| 013 | exp013_v2_visionfix (wcjklw7a, Isaac PPO) | 65.5M steps (2048 envs×1000 iters) | ✅ 완료 | **Isaac 첫 완주 학습 + 200-ep deterministic eval = 36%.** v1은 비전 사멸 버그(env-origin 프레임 혼용)로 중단·수정 후 재기동. 곡선 plateau(iter ~700), d_xy_min 1.4m 정체. 실패 분해: max_altitude 33%(analytic conf 거리감쇠 누락 → 상승 farming, Rule 17) + crash 27%; farmer>finisher 보상 불균형(Rule 18a); noise_std 0.8→3.92 폭주(Rule 18b); **사후 --zero-actions FAIL(11.9m) → 리셋 속도킥이 run 전체 오염(§4d, 1차 용의자).** **다음: exp_014 = 0순위 킥 수정 → conf 거리감쇠+success 300+entropy 0, fresh.** → [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]] / [[errors/err_20260703_vision_env_origin_frame]] |

## 에러 현황

| 파일 | 상태 | 요약 |
|------|------|------|
| [[errors/err_20260703_vision_env_origin_frame]] | ✅ 해결 | Isaac `_update_vision` env-origin 프레임 혼용 → 벡터화 학습에서 비전 채널 완전 사멸(conf≡0). num_envs=1 검증으론 구조적으로 못 잡음 |
| [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] | ✅ 해결 | armdiag dry-run이 YAML 중복 키로 v13 30K 체크포인트 파괴. 재발방지: startup `Checkpoints:` 로그로 격리 검증 |
| [[errors/err_20260615_cruise-timeout-arming]] | ✅ 해결 | CRUISE 타임아웃 = teleport 후 PX4 arm 거부 (stale EKF) |
| [[errors/err_20260320_physics_explosion]] | ✅ 해결 | ODE 물리 폭발 3중 방어 |
| [[errors/err_20260319_ode_aabb_crash]] | ✅ 해결 | 드론 스폰 고도 ODE AABB 크래시 |

---

## 빠른 참조

| 주제 | 파일 |
|------|------|
| Guacamole/VNC 설정·접속 | `notes/Environment/README.md` |
| 자주 쓰는 명령어 | `notes/sessions/commands.md` |
| RL 규칙 | `notes/research/rl_rules.md` |
| VM 완전 복구 가이드 | `notes/Environment/README.md` |
| 보상 함수 설계 | `notes/research/reward_design.md` |
| 전체 시스템 아키텍처 (Gazebo/PX4/ROS2, `jekyun`) | `notes/research/system_overview.md` |
| 전체 시스템 아키텍처 (Isaac Lab, `feat/isaac-env-migration`) | `notes/research/isaac_lab_architecture.md` |
| Isaac Lab 다른 연구자용 온보딩 (보상/하이퍼파라미터/WandB/실험 절차) | `notes/research/isaac_lab_reward_tuning.md` · `notes/research/isaac_lab_wandb_guide.md` · `notes/research/isaac_lab_experiment_workflow.md` |

---

## 폴더 구조

| 폴더 | 용도 | 네이밍 규칙 |
|------|------|------------|
| `research/` | 이론·설계·아키텍처 | `{topic_slug}.md` |
| `experiments/` | 학습 실험 (WandB 연동) | `exp_{NNN}_{wandb_id}_{title}.md` |
| `errors/` | 에러 해결 기록 | `err_{YYYYMMDD}_{slug}.md` |
| `sessions/` | 세션별 작업 일지 | `session_{YYYY-MM-DD}.md` |
| `references/` | 논문·문서 메모 | `ref_{slug}.md` |
| `Environment/` | 인프라 설정 백업 | `{service}.{ext}` |

---

## 노트 인덱스

### 연구 (research/)
- [[research/vision_obs_refactor]] — Vision 기반 obs 리팩토링 (GPS 제거, YOLO 전환)
- [[research/phase1_plan]] — Phase 1 CCIP 기반 자율 접근 연구 계획 (8주, 5/8-6/30)
- [[research/reward_design]] — 4-layer 보상 함수 (LaTeX 수식)
- [[research/architecture]] — Method A (1-World-4-Payload) 아키텍처
- [[research/system_overview]] — 전체 시스템 (패키지, 토픽, 좌표계, 브리지) — `jekyun`(Gazebo/PX4/ROS2) 브랜치
- [[research/isaac_lab_architecture]] — Isaac Lab 전체 구조 (`isaac_lab/` 레이아웃, 데이터 흐름, Gazebo 대비 구조 차이) — `feat/isaac-env-migration` 브랜치
- [[research/rl_rules]] — RL 실험 규칙, WandB 메트릭, Known Failure Modes
- [[research/ekf_east_reversal]] — ⚠️ RETRACTED: "EKF East 반전"은 오진이었음 (실제 PX4 East=+Gazebo East). 정정: [[coordinate-frames]] / [[daily/daily_2026-06-14]]
- [[research/rtf_fps_analysis]] — RTF vs FPS 분석. RTF=2 최적, Python 루프 병목 규명
- [[research/cruise_timeout_arming]] — CRUISE 타임아웃 = teleport 후 EKF 재수렴 bimodal(0s/13–16s). **06-17 정정: v12의 10s 컷이 복구 직전 단두대질 → arm_bail 10→20s** (Rule 11).
- [[research/terminal_overshoot_trap]] — v12 종단 정체 = overshoot 해자 트랩 (정하방 카메라 핸드오프 ~1m). v13 보상 재설계.
- [[research/eval_terminal_env_metrics]] — v13 eval EKF divergence 흡수 루프 + evaluate.py 지표 비정합 (탄도 투하 없음 → CEP 비실재). 시작 health gate 필요 (Rule 12).
- [[research/detection_gate_vs_altitude]] — 핸드오프 윈도우의 진짜 레버 = 탐지 게이트(conf + 공간 필터), 고도 아님. 고도↑는 마커 가시성 깎아 역효과 (Rule 13).
- [[research/reset_throughput_bottleneck]] — 리셋 병목 = teleport 후 EKF 재수렴(param으론 못 고침). soft reset(teleport 회피)으로 ~3.9× (Rule 14).
- [[research/control_smoothness_wobble]] — RL 인수 후 wobble = smoothness-control 문제(정책, 탐험 아님). LPF+근접 속도댐핑+smoothness 가중 (Rule 15).
- [[research/isaac_velocity_controller]] — Isaac Lab 캐스케이드 속도 컨트롤러, PX4 게인 매핑. **PX4 대비 미검정**(게인 초기값) — 7-포인트 스텝응답 검정 계획.
- [[research/isaac_lab_reward_tuning]] — Isaac Lab 보상·하이퍼파라미터 레퍼런스 (다른 연구자용 온보딩). cfg 필드별 의미·튜닝 시 주의(overshoot moat, fresh-start 판단 등).
- [[research/isaac_lab_wandb_guide]] — Isaac Lab WandB 메트릭 가이드. `Episode_Termination/*`·`Episode_Reward/*` 등 신규 네임스페이스, Gazebo 트랙과 대조표. 첫 실 학습 전이라 rsl_rl 표준 키(§5)는 미검증 표시.
- [[research/isaac_lab_experiment_workflow]] — Isaac Lab 실험 실행 절차 (dry-run 사다리, fresh/resume 판단, WandB run 관리, 실험 로깅).
- [[research/isaac_ppo_tuning_recommendations]] — exp_013 결론: 무엇을 바꿔야 하는가 (conf 거리감쇠·reward_success 300·entropy 0 우선; 스폰 고도는 유지). Rule 17·18의 근거 문서.
- [[research/exp014_ablation_protocol]] — exp_014 ablation 설계(07-04): 킥 vs attractor 원인 분리 arms/임계값/무학습 probes. forensics로 킥은 프로세스당 1회로 실증(기각), attractor 1차 가설.

### 실험 (experiments/)
- [[experiments/training_history]] — 전체 WandB 학습 히스토리
- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 거리 보상 + CRUISE retry
- [[experiments/exp_002_reward_shaping_patches]] — 보상 패치 Fresh Training (대기 중)
- [[experiments/exp_003_rtf_dryrun]] — RTF 1/2/4 dry-run 비교. RTF=2 최적 확정.
- [[experiments/exp_004_rl_yolo_debug_vision]] — Vision YOLO TRACKING + EKF East 좌표 버그 수정.
- [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] — Arming-rejection throughput fix. v11 분석 + 수정 3종 + dry-run 검증.
- [[experiments/exp_006_xgzum51v_armdiag_dryrun]] — arm_bail 진단. EKF 재수렴 bimodal 계측 → 10s 컷이 진짜 병목. Fix: arm_bail 10→20s.
- [[experiments/exp_007_iyhfy5ps_v13_eval]] — v13 deterministic eval. 유효 3-ep 100% 성공(reward 124); ep 4–13 EKF divergence 흡수 루프. harness 지표 비정합 발견.
- [[experiments/exp_008_dryrun_alt10_handoff_window]] — 핸드오프 윈도우↑ dry-run. 고도↑ 실패→탐지 게이트 수정(conf+300 px)으로 핸드오프 2.7→5.0 m. 미커밋.
- [[experiments/exp_009_softreset_throughput]] — 리셋 처리량. EKF param A/B(음성) + soft reset 프로토(~3.9×, EKF 안정). full run 검증 중.
- [[experiments/exp_010_byxyaf4d_v14_195k_eval]] — v14 195K eval 65%(13/20, v13 회귀, final-approach stagnation) + soft reset 장기검증(3096 resets, Rule 14 완료) + 비디오 산출물.
- [[experiments/exp_011_wobble_lpf_reward_damping]] — RL wobble = smoothness-control 문제 확정. LPF A/B(jerk RMS −45%) + 보상 댐핑(B+C) → v15 fresh 기동.
- [[experiments/exp_012_isaac_migration_phase2]] — Isaac Lab env+PPO 이식. v13/v15 상수 parity 이식, `pytest test_math.py` 29/29 통과, L4 VM 미기동.
- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] — Isaac 첫 완주 PPO(65.5M steps) + eval 36%. 비전 사멸 버그 수정, 상승-farming attractor·보상 불균형·noise_std 폭주 규명 → Rule 17·18.

### 에러 (errors/)
- [[errors/err_20260703_vision_env_origin_frame]] — Isaac `_update_vision` world/env-local 프레임 혼용 → 벡터화 학습 비전 완전 사멸. "정확히 0.0000인 보상 성분 = 채널 사멸 신호" 규칙.
- [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] — armdiag dry-run이 YAML 중복 `checkpoint_dir` 키로 v13 30K 체크포인트 파괴. fresh-start 삭제 footgun + 격리 검증 규칙.
- [[errors/err_20260615_cruise-timeout-arming]] — CRUISE 타임아웃 = teleport 후 PX4 arm 거부 (stale EKF). arm 게이팅 + early-bail.
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 물리 폭발 3중 방어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 연구 일지 (daily/)
- [[daily/daily_2026-07-04]] — exp_013 forensics: 킥=프로세스당 1회 확정(m_eff 계측), max_alt=학습된 attractor 재귀속, entropy σ-불변 실측, eval 8ep 해명, exp_014 ablation 설계
- [[daily/daily_2026-07-03]] — Isaac Lab migration Phase 2: env+PPO 코드 이식, 워크트리 분리, test_math.py 29/29 통과, Rule 16 신규
- [[daily/daily_2026-06-23]] — v14 plateau stop @196.5K → 195K eval 65%(v13 회귀, final-approach stagnation) + soft reset 장기검증(Rule 14 완료) + 비디오 캡처
- [[daily/daily_2026-06-22]] — 핸드오프 윈도우 확장: 고도↑(10 m) 실패 → 탐지 게이트(conf 0.5 + 200→300 px)로 핸드오프 2.7→5.0 m
- [[daily/daily_2026-06-20]] — v13 정책 평가: 유효 ep 100% 성공(정책 양호) + eval EKF divergence 흡수 루프 + harness 지표 비정합
- [[daily/daily_2026-06-16]] — v12 정체 진단(종단 overshoot 트랩) + v13 종단 보상 재설계 prep
- [[daily/daily_2026-06-14]] — 06-12 이후 종합: 보상 함수 재설계(v9) + YOLO hold(v9b) + throughput 최적화(v9c·v9d)
- [[daily/daily_2026-06-12]] — EKF East 반전 수정 + proximity trigger 정상화 + rl_yolo fresh start
- [[daily/daily_2026-05-30]] — Isaac Sim migration Phase 1 인프라 구축 (Dockerfile·startup·deploy 재작성)
- [[daily/daily_2026-04-26]] — Vision obs 리팩토링 계획 + 카메라 파이프라인 복원
- [[daily/daily_2026-04-23]] — Spot VM 이전 완료 (startup.sh + watchdog CF + create_spot_vm.sh) + IP 변경 대응
- [[daily/daily_2026-04-17]] — Phase 1 코드 전체 구현 (변경 1-10, obs 15→17, CCIP auto-drop)
- [[daily/daily_2026-04-16]] — WandB 연결 + RTF dry-run 실험 + 인프라 고장 해결
- [[daily/daily_2026-04-14]] — Guacamole HTTPS + Obsidian 설치 + wikilink 정비

### 세션 (sessions/)
- [[sessions/session_2026-04-16]] — RTF dry-run, docker commit, airframe 수정
- [[sessions/session_2026-04-14]] — Obsidian 시스템 초기화 + 파일 간소화

### 환경 설정 (Environment/)
- [[Environment/README]] — VM 완전 복구 가이드
- [[Environment/docker-compose.yml]] — guacd + guacamole + postgres + nginx
- [[Environment/nginx.conf]] — WebSocket 리버스 프록시 설정
- [[Environment/vncserver.service]] — TigerVNC systemd 유닛
