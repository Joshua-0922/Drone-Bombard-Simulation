---
date: 2026-04-14
updated: 2026-09-25
tags: [index, dashboard]
status: active
type: index
---

# 드론 정밀 투하 연구 — Obsidian 대시보드

> **프로젝트:** 바람 속 정밀 투하 — 비행은 강화학습(L0), 조준 보정은 착탄점 공간의 잔차 지도학습(GRU-S + EMA)
> **스택:** Isaac Sim 5.1 · Isaac Lab 2.3 · rsl_rl PPO · GCP L4 · 컨테이너 `isaac-verify` · Obsidian + graphify
> **구식 노트**는 각 폴더의 `legacy/`에 있다(§legacy). 이 페이지는 현행 문서만 가리킨다.

---

## 현재 상태 (2026-09-25)

**본 방법 확정: L0 + GRU-S + EMA α=0.3.** 단일 방법 원칙 — 동률이거나 열세인 변형(잔차 RL, OU 학습, 실현 라벨, 분산 헤드, 시간 일관성 손실, 게이트 손실)은 논문·코드에서 제외하고 실험 노트에만 남겼다.

| 조건 (n=600) | L0 | **본 방법** | 오라클 |
|---|---|---|---|
| 정상 바람 DR 1.5 — CEP50 / CEP90 / succ@0.5 | 0.305 / 0.596 / 78.8% | **0.204 / 0.403 / 90.8%** | 0.188 |
| 현실 돌풍 A(20%, τ10) / B(30%, τ3) — CEP50 | 0.301 / 0.305 | **0.201 / 0.219** | 0.193 / 0.198 |
| DR 0 → 2.5 — 본 방법 vs L0 | | +8.8% → −39.4% | |

- 표·그림 완료: [[research/final_tables_v6]] · `notes/figures/fig{1,2,3}.png`
- EMA α는 민감도 스윕으로 확정(평탄 [0.1, 0.4], 무필터 +17%/+92%): [[experiments/exp_041_ema_alpha_sweep]]
- **다음:** 비전 절(YOLO 마커 보정·픽셀 양자화 팔) → 하드웨어 플랫폼 결정 → 본문 집필(개요 v6 §1~5) → sim2real(시뮬레이션 모델 그대로 onboard, T0/T2/L0/본 방법 실기 비교)

최근 일지: [[daily/daily_2026-09-25]] · [[daily/daily_2026-09-24]] · 세션: [[sessions/session_2026-09-24]]

---

## 빠른 참조

| 무엇 | 어디 |
|---|---|
| 논문 개요 | [[research/paper_outline_v6]] |
| 방법 설명 (5단계) | [[research/l1_sl_pipeline]] |
| 코드 위치 (함수·줄) | [[research/code_map]] |
| 파일별 역할·데이터 흐름 | [[research/isaac_lab_architecture]] |
| 실행 명령 | [[sessions/commands]] |
| 규칙 (Rule 1~46) | [[research/rl_rules]] |
| 실험 이력 허브 | [[experiments/training_history]] |
| 보상·MDP | [[research/reward_design]] |

---

## 폴더 구조

```
notes/
├── 00_index.md            이 문서
├── research/              연구 노트(현행)   ├── legacy/  Gazebo·이식·커리큘럼·팀원 트랙·구 계획서
├── experiments/           exp_025~ (논문 단계) ├── legacy/  exp_001~024
├── errors/                에러 해결 기록
├── daily/, sessions/      일지·세션 기록 (시간순, 이동하지 않음)
├── figures/               논문 그림
└── Environment/           VM·접속 가이드
```

---

## 노트 인덱스

### 연구 (research/)
- [[research/paper_outline_v6]] — 📄 **논문 개요 v6** — 바람 속 투하: 비행 RL + 착탄점 잔차 지도학습, 단일 방법(GRU-S + EMA), 절별 표·ablation·한계
- [[research/final_tables_v6]] — 📊 **최종 표 5개 + 그림 3장** — Table 1 · DR 스윕 · 바람 조건 · 미지 사거리 · ablation
- [[research/research_architecture]] — ⭐ **연구 아키텍처 v6 요약 블록(09-25)** + v4/v5 설계 기록 — 주장·비교 팔·버린 것·다음 단계
- [[research/l1_sl_pipeline]] — 📘 **본 방법 파이프라인(GRU-S)** — 수집·입력·학습·내보내기·주입 5단계, 성능, 채택하지 않은 시도, 제어기 의존성
- [[research/code_map]] — 🗺 **코드 지도** — τ의 뜻, 바람 모델 함수·줄, L0 학습·GRU 학습·평가 코드 위치, 결과 JSON 경로
- [[research/isaac_lab_architecture]] — 🧩 **`isaac_lab/` 파일별 역할과 데이터 흐름(09-25)**
- [[research/reward_design]] — 현재 보상 함수 + MDP 정의(관측 26 · 행동 7 · 종료), `TaskRewardCfg` 기준
- [[research/rl_rules]] — RL·잔차 실험 규칙 Rule 1~46 (누적)
- [[research/release_gate_jitter]] — 첫 교차 게이트 앞에서 예측 요동은 계통 편향이 된다 → EMA (Rule 38), α 민감도(§3b)
- [[research/residual_observability]] — 관측이 바람을 담고 있는가 — 지도 회귀 $R^2$ 사다리, 제어기 의존성 (Rule 37)
- [[research/residual_ceiling]] — 잔차의 상한선(오라클) 실측과 왜 착탄점 공간·지도학습인가 (§7)
- [[research/residual_policy_coupling]] — 잔차가 기저 정책에 결합된다 → 게인 무관 특징으로 해소 (Rule 39)
- [[research/residual_label_efficiency]] — 라벨 1,000개면 이득의 96% (Rule 40)
- [[research/residual_wind_stationarity]] — 잔차는 바람의 준정상성을 전제한다, 경계 τ≈3 s (Rule 42)
- [[research/residual_rl_exploration_collapse]] — 결과 공간 잔차의 PPO 탐험은 스스로 소멸 (Rule 44, 잔차 RL 불채택 근거)
- [[research/l1_rl_preflight]] — 잔차 RL 파일럿 전 수정 6건 (exp_032·033의 전제, 미채택 경로)
- [[research/paper_metrics]] — 논문 지표 확정: CEP50·CEP90·succ@0.5·배달 시간·배달률
- [[research/training_seed_protocol]] — 학습 시드 vs 평가 시드, 몇 개를 돌리나
- [[research/error_budget_l0]] — L0 착지 오차는 어디서 오는가
- [[research/t3_oracle_entrainment]] — 오라클 재정의(즉시 엔트레인먼트 가정 폐기), 지배 오차는 자기속도 항력 (Rule 31)
- [[research/ccip_vz_omission]] — CCIP 수직속도 누락 수정 — 잔차가 배우던 것은 공식 결손이었다 (Rule 30)
- [[research/reward_operating_point]] — `w_time`이 정하는 운용점(정지 투하 vs 통과 투하 무차별점)
- [[research/handoff_generalization_p0]] — 고정 초기조건은 표현을 암기시킨다, 랜덤화 축 분류 (Rule 27)
- [[research/agility_ceiling]] — 급기동 천장 4중 구조(인지 기하가 순항 속도를 못박음)
- [[research/related_work_survey]] — 선행연구 조사(CARP 계보·학습 투척·바람 하 RL·TossingBot), 겹치는 주장 없음
- [[research/sim2real_gap]] — 시뮬레이터에 없는 것(로터 유입류·모터 지연·게인 실측)과 실기 계획의 전제
- [[research/physical_payload_attach]] — 물리 페이로드 kinematic weld 패턴 (Rule 24)
- [[research/isaac_velocity_controller]] — 캐스케이드 속도 제어기, PX4 게인 매핑
- [[research/interim_report_outline]] · [[research/interim_report_isaac_1page]] · [[research/interim_report_isaac_draft]] — 중간보고서(09-07, GRU-S 확정 이전 수치)

### 실험 (experiments/) — 논문 단계
- [[experiments/training_history]] — 전체 학습·평가 히스토리 허브
- [[experiments/exp_041_ema_alpha_sweep]] — ✅ EMA α 민감도: [0.1, 0.4] 평탄, 무필터만 유의 악화 → **α=0.3 확정**
- [[experiments/exp_040_consistency_final]] — 🟰 시간 일관성 GRU 최종 비교, 12조건 전부 동률 → 본 방법 = GRU-S + EMA 유지
- [[experiments/exp_039_gate_aware_stage23]] — ⛔ 게이트를 손실에 결합: 악화 또는 유의차 없음
- [[experiments/exp_038_gate_aware_stage1]] — 평활 항(λ≥20)은 EMA 없이 동률 (원리상 등가)
- [[experiments/exp_037_relative_shrink_realistic_wind]] — ⭐⭐ 현실 바람(평균풍+돌풍)에서 이득 유지 −33%/−28% → **본 방법 = GRU-S**
- [[experiments/exp_036_uncertainty_head]] — 평균+분산 헤드: 스트레스 τ 손해 절반, 현실 조건 동률 (미채택)
- [[experiments/exp_035_realised_label]] — 실현 라벨: 스트레스 τ 개선, 정상 유지 (미채택)
- [[experiments/exp_034_learned_temporal_filter]] — ✅ 수제 누적 특징 → GRU 시간 필터 대체 (0.204 vs 0.209); OU 라벨 병목 (Rule 45)
- [[experiments/exp_033_tog7jqs5_l1rl_fixed_std_pilot]] — ⛔ 잔차 RL(std 고정): 반대 방향 성장 +18.2% (Rule 44)
- [[experiments/exp_032_icpj8p4r_l1rl_zero_pilot]] — ⛔ 잔차 RL(0 초기화): std 소멸, −1.9% (Rule 44)
- [[experiments/exp_031_ou_wind]] — 시변(OU) 바람 ablation 60 run — 준정상성 전제의 경계
- [[experiments/exp_030_gain_invariant_residual]] — ⭐⭐ 게인 무관 특징으로 정책 결합 해소 + Table 1 시드 정렬
- [[experiments/exp_029_l1_sl_generalization]] — 일반화 감사: 미지 사거리 ✅ / 정책 전이 ⛔ / 라벨 1,000개면 96%
- [[experiments/exp_028_l1_sl_pilot]] — ⭐ L1-SL 파일럿: PPO 없이 CEP50 −31.4%, EMA 발견 (Rule 38)
- [[experiments/exp_027_seen_unseen_3seed]] — seen/unseen + Table 1 3 seed + 잔차 상한선
- [[experiments/exp_026_release_rate_100hz]] — 릴리즈 판정 100 Hz + 성공 반경 0.5 m (T1≡T2)
- [[experiments/exp_025_dr_scale_sweep_gate]] — DR 스윕 유효성 게이트(무학습): 오라클 갭 단조 증가

### 에러 (errors/)
- [[errors/err_20260830_aim_reward_residual_inclusive]] — task_env의 dense aim 보상이 잔차 포함 오차를 먹고 있다
- [[errors/err_20260827_payload_drag_body_frame]] — err_20260827 — 페이로드 항력을 월드 프레임으로 계산해놓고 링크 프레임으로 전달
- [[errors/err_20260827_free_exit_termination]] — err_20260827 — 무료 탈출구: 벌하지 않는 종료 조건 + 탐지 뒤에 갇힌 접근 보상
- [[errors/err_20260823_ccip_vz_omission]] — err_20260823 — CCIP `ballistic_impact`가 수직속도를 누락
- [[errors/err_20260803_payload_landing_latch]] — 물리 페이로드 착지 래치가 "정지한 페이로드"를 영원히 놓친다
- [[errors/err_20260723_wandb_key_empty]] — err_20260723 — 컨테이너 baked-in WANDB_API_KEY 공백 → 학습 silent 실패
- [[errors/err_20260703_vision_env_origin_frame]] — Isaac Lab 비전 완전 사멸 — env-origin 좌표 프레임 혼용
- [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] — 인시던트 — armdiag dry-run이 v13 30K 체크포인트를 파괴 (YAML 중복 키 + fresh-start 삭제)
- [[errors/err_20260615_cruise-timeout-arming]] — Err — CRUISE 타임아웃 (Teleport 후 PX4 Arm 거부)
- [[errors/err_20260528_gz_timeout_recurrence_junsang]] — err 2026-05-28 — `gz model --list` TimeoutExpired (반복 인프라 크래시, #021)
- [[errors/err_20260520_spin_thread_recursive_reset]] — Spin Thread Death → Recursive reset() Crash
- [[errors/err_20260320_physics_explosion]] — Err — Gazebo 물리 폭발 (d_xy = 1.98×10¹¹ m)
- [[errors/err_20260319_ode_aabb_crash]] — Err — ODE AABB 크래시 (드론 스폰 고도)

### 연구 일지 (daily/)
- [[daily/daily_2026-09-25]] — 연구 일지 — 2026-09-25
- [[daily/daily_2026-09-24]] — 연구 일지 — 2026-09-24
- [[daily/daily_2026-09-14]] — 연구 일지 — 2026-09-14
- [[daily/daily_2026-09-13]] — 연구 일지 — 2026-09-13
- [[daily/daily_2026-09-07]] — 연구 일지 — 2026-09-07
- [[daily/daily_2026-09-02]] — 연구 일지 — 2026-09-02
- [[daily/daily_2026-09-01]] — 2026-09-01 — 잔차를 지도학습으로 돌려서 CEP를 31% 줄였다 (PPO 없이)
- [[daily/daily_2026-08-30]] — 연구 일지 — 2026-08-30
- [[daily/daily_2026-08-29]] — 연구 일지 — 2026-08-29
- [[daily/daily_2026-08-27]] — 연구 일지 — 2026-08-27
- [[daily/daily_2026-08-23]] — 연구 일지 — 2026-08-23
- [[daily/daily_2026-08-03]] — 연구 일지 — 2026-08-03
- [[daily/daily_2026-08-01]] — 연구 일지 — 2026-08-01
- [[daily/daily_2026-07-30]] — Daily — 2026-07-30
- [[daily/daily_2026-07-28_team_briefing]] — 팀 브리핑 — isaac_jk 브랜치 현황 (2026-07-28)
- [[daily/daily_2026-07-28]] — Daily 2026-07-28
- [[daily/daily_2026-07-23]] — 연구 일지 — 2026-07-23
- [[daily/daily_2026-07-16_junsang]] — 연구 일지 — 2026-07-16
- [[daily/daily_2026-07-15_junsang]] — 연구 일지 — 2026-07-15
- [[daily/daily_2026-07-13]] — 연구 일지 — 2026-07-13
- [[daily/daily_2026-07-07]] — 연구 일지 — 2026-07-07
- [[daily/daily_2026-07-06]] — Daily — 2026-07-06
- [[daily/daily_2026-07-05_gazebo_v15_regression]] — 연구 일지 — 2026-07-05
- [[daily/daily_2026-07-05]] — Daily 2026-07-05
- [[daily/daily_2026-07-04]] — 연구 일지 — 2026-07-04
- [[daily/daily_2026-07-03]] — 연구 일지 — 2026-07-03
- [[daily/daily_2026-07-01]] — 연구 일지 — 2026-07-01
- [[daily/daily_2026-06-23]] — 연구 일지 — 2026-06-23
- [[daily/daily_2026-06-22]] — 연구 일지 — 2026-06-22
- [[daily/daily_2026-06-20]] — 연구 일지 — 2026-06-20
- [[daily/daily_2026-06-17]] — Daily 2026-06-17 — v13 처리량 병목 진단 & arm_bail 수정
- [[daily/daily_2026-06-16]] — 연구 일지 — 2026-06-16
- [[daily/daily_2026-06-14]] — 연구 일지 — 2026-06-14 (06-12 이후 종합 요약)
- [[daily/daily_2026-06-12]] — 연구 일지 — 2026-06-12
- [[daily/daily_2026-06-09]] — 연구 일지 — 2026-06-09
- [[daily/daily_2026-06-05_junsang]] — 연구 일지 — 2026-06-05
- [[daily/daily_2026-06-03_junsang]] — 연구 일지 — 2026-06-03
- [[daily/daily_2026-05-31_junsang]] — 연구 일지 — 2026-05-31
- [[daily/daily_2026-05-30_vision-refactor]] — 연구 일지 — 2026-05-30 (Vision Refactor / junsang 브랜치)
- [[daily/daily_2026-05-30]] — 연구 일지 — 2026-05-30
- [[daily/daily_2026-04-23]] — 연구 일지 — 2026-04-23
- [[daily/daily_2026-04-17]] — 연구 일지 — 2026-04-17
- [[daily/daily_2026-04-16]] — 연구 일지 — 2026-04-16
- [[daily/daily_2026-04-14]] — 연구 일지 — 2026-04-14

### 세션 (sessions/)
- [[sessions/session_2026-09-24]] — 세션 — 2026-09-24 (UTC 04:50 ~ 23:30)
- [[sessions/session_2026-09-14]] — 세션 — 2026-09-13 15:00 ~ 2026-09-14 04:00 (UTC)
- [[sessions/session_2026-08-27]] — 세션 기록 — 2026-08-27
- [[sessions/session_2026-07-16_junsang]] — 세션 로그 — 2026-07-16
- [[sessions/session_2026-07-15_junsang]] — 세션 로그 — 2026-07-15
- [[sessions/session_2026-07-06]] — Session — 2026-07-06 (exp_017 Stage A + exp_018 Stage B)
- [[sessions/session_2026-07-05]] — 세션 2026-07-05 — plant 수정 실행 + inertia 대반전 + exp_014 A2/A0′
- [[sessions/session_2026-04-16]] — 세션 일지 — 2026-04-16
- [[sessions/session_2026-04-14]] — 세션 2026-04-14 — Obsidian 연구 비서 시스템 초기화
- [[sessions/commands]] — 자주 쓰는 명령어 모음 (Isaac Lab, 2026-09 기준)

### 환경 설정 (Environment/)
- [[Environment/vm_access_guide_junsang]] — L4 VM 접속 가이드 (Isaac 학습용) — 팀원용
- [[Environment/README]] — GCP VM 완전 복구 가이드 — Drone Bombard 환경

---

## legacy — 구식 노트 (참고용, 현행 아님)

**research/legacy/** (37편): Gazebo/PX4/ROS2·SAC 시기(`system_overview`, `architecture`, `phase1_plan`, `rtf_fps_analysis`, `cruise_timeout_arming`, `reset_throughput_bottleneck`, `detection_gate_vs_altitude`, `terminal_overshoot_trap`, `eval_terminal_env_metrics`, `control_smoothness_wobble`, `ekf_east_reversal`(철회)),
Isaac 이식·커리큘럼 시기(`isaac_lab_reward_tuning`, `isaac_lab_wandb_guide`, `isaac_lab_experiment_workflow`, `isaac_ppo_tuning_recommendations`, `isaac_inertia_ctrl_mismatch`, `exp014_ablation_protocol`, `phased_curriculum`, `curriculum_phase_convergence`, `ccip_release_decoupling`, `ccip_aim_reward_stageA`, `release_terminal_stageB`, `moving_target_models`),
팀원 트랙(`*_junsang`, `00_index_junsang`), 상위 문서로 대체된 계획서(`paper_research_plan`, `research_overview_for_paper`).
→ [[research/legacy/system_overview]] · [[research/legacy/phased_curriculum]] · [[research/legacy/00_index_junsang]]

**experiments/legacy/** (37편): exp_001~024 — Gazebo SAC 학습, Isaac 이식 첫 학습(exp_013), plant 수정(exp_014), 커리큘럼(exp_015), 릴리스 종단(exp_018), 물리 페이로드(exp_019·020), 이동 표적(exp_021), v20 env·P0(exp_022), Table 1 1차(exp_023·024).
→ [[experiments/legacy/exp_013_wcjklw7a_isaac_ppo_first_training]] · [[experiments/legacy/exp_022_p0_handoff_dyn_dr]]

규칙(Rule 1~29)과 학습 이력의 해당 행은 [[research/rl_rules]]·[[experiments/training_history]]에 그대로 남아 legacy 노트를 가리킨다.
