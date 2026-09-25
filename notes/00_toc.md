---
date: 2026-09-25
tags: [index, toc]
status: active
type: index
---

# notes 목차 — 어떤 파일에 어떤 내용이 있나

> 모든 노트의 전체 목록이다(자동 생성, 제목·날짜·상태). 무엇부터 읽을지는 대시보드 [[00_index]]의 "빠른 참조"를 본다.
> 현행 문서만 추리면: [[research/paper_outline_v6]](논문 개요) · [[research/l1_sl_pipeline]](방법) · [[research/final_tables_v6]](표·그림) · [[research/code_map]](코드 위치) · [[sessions/commands]](명령).
> 재생성: `python3 notes/_make_toc.py`.

## 루트 (1)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[00_index]] | 드론 정밀 투하 연구 — Obsidian 대시보드 | 2026-04-14 | active |

## research — 현행 연구 노트 (29)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[research/agility_ceiling]] | 급기동(dive-bomb)은 지금 코드에서 가능한가 — 천장 4중 구조 | 2026-08-27 | open |
| [[research/ccip_vz_omission]] | CCIP 수직속도 누락 — 잔차가 배우던 것은 바람이 아니라 공식 결손이었다 | 2026-08-23 | resolved |
| [[research/code_map]] | 코드 지도 — 바람 모델 · 강화학습(비행) · 지도학습(잔차) · 평가가 어느 파일에 있나 | 2026-09-25 | active |
| [[research/error_budget_l0]] | L0 오차 예산 — 착지 오차는 어디서 오는가 | 2026-08-30 | active |
| [[research/final_tables_v6]] | 최종 표·그림 v6 — 본 방법 = L0 + GRU 잔차 (GRU-S) | 2026-09-24 | active |
| [[research/handoff_generalization_p0]] | 고정 초기조건은 성능이 아니라 *표현*을 암기시킨다 (P0 발견) | 2026-08-03 | active |
| [[research/interim_report_isaac]] | 중간보고서 — Isaac Lab 파트 (개요 · 초고 · 요약본 통합) | 2026-09-07 | archived |
| [[research/isaac_lab_architecture]] | 시스템 구조 — isaac_lab/ 파일별 역할 (2026-09-25 기준) | 2026-07-03 | active |
| [[research/isaac_velocity_controller]] | Isaac Lab 속도 컨트롤러 — PX4 게인 매핑 + 검정 상태 | 2026-07-03 | active |
| [[research/l1_rl_preflight]] | L1-RL 사전점검 — 잔차 강화학습을 돌리기 전에 바꿔야 하는 것 | 2026-09-13 | applied |
| [[research/l1_sl_pipeline]] | L1-SL 파이프라인 — 본 방법(L0 + GRU-S + EMA)이 무엇을, 어떤 입력으로, 어디에 넣는가 | 2026-09-14 | active |
| [[research/paper_metrics]] | 논문 지표 확정 — 성공률을 헤드라인에서 내린다 | 2026-08-30 | decided |
| [[research/paper_outline_v6]] | 논문 개요 v6 — "바람 속에서 던지기: 비행은 강화학습, 조준 보정은 지도학습" | 2026-09-24 | active |
| [[research/physical_payload_attach]] | 물리 페이로드 부착/분리 — kinematic weld 패턴 | 2026-07-21 | active |
| [[research/related_work_survey]] | 선행연구 조사 — 우리는 어디에 서 있나 | 2026-08-27 | open |
| [[research/release_gate_jitter]] | 릴리즈 게이트는 첫 교차 판정이다 — 잔차는 정확한 것만으로 부족하고 매끄러워야 한다 | 2026-09-01 | active |
| [[research/research_architecture]] | 드론 정밀 투하 연구 — 최종 아키텍처 v6 | 2026-08-23 | active |
| [[research/residual_ceiling]] | 잔차 RL의 상한선 — 측정했다, 그리고 레시피 | 2026-08-30 | active |
| [[research/residual_label_efficiency]] | 라벨 몇 개가 필요한가 — 그리고 남은 간극은 데이터가 아니라 정보다 | 2026-09-02 | active |
| [[research/residual_observability]] | 관측이 바람을 담고 있는가 — 잔차의 지도학습 상한선 실측 | 2026-09-01 | decided |
| [[research/residual_policy_coupling]] | 잔차 회귀기는 물리가 아니라 정책에 결합되어 있다 | 2026-09-02 | resolved |
| [[research/residual_rl_exploration_collapse]] | 결과 공간 잔차의 PPO 탐험은 스스로를 지운다 — 첫 교차 게이트가 잡음을 벌하기 때문 | 2026-09-13 | active |
| [[research/residual_wind_stationarity]] | 잔차는 바람의 준정상성(quasi-stationarity)을 전제한다 — 그리고 그 경계를 쟀다 | 2026-09-07 | active |
| [[research/reward_design]] | 보상 함수와 MDP 정의 — 현재 (Isaac Lab task env) | 2026-03-22 | active |
| [[research/reward_operating_point]] | 보상이 정하는 운용점 — w_time이 속도-정확도 트레이드오프의 어디에 앉을지를 결정한다 | 2026-08-27 | open |
| [[research/rl_rules]] | RL 실험 & 디버깅 규칙 | 2026-04-14 | active |
| [[research/sim2real_gap]] | sim2real 대비 — 지금 시뮬레이터에 없는 것 | 2026-08-27 | open |
| [[research/t3_oracle_entrainment]] | T3 오라클이 상한선이 아니었다 — 즉시 엔트레인먼트 가정 | 2026-08-27 | resolved |
| [[research/training_seed_protocol]] | 학습 시드 vs 평가 시드 — 무엇을 몇 개나 돌려야 하나 | 2026-08-30 | active |

## research/legacy — 구식 연구 노트 (37)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[research/legacy/00_index_junsang]] | 🧭 junsang 개인 대시보드 | 2026-06-06 | active |
| [[research/legacy/architecture]] | Method A — 1-World-4-Payload 아키텍처 | 2026-03-20 | implemented |
| [[research/legacy/ccip_aim_reward_stageA]] | Stage A — 밀집 CCIP 조준오차 보상으로 release_rate 올리기 (exp_017) | 2026-07-06 | active |
| [[research/legacy/ccip_release_decoupling]] | CCIP 릴리스 디커플링 — success 100% vs drop_impact_error 4.59 m의 실체 | 2026-07-05 | active |
| [[research/legacy/control_smoothness_wobble]] | RL 인수 후 wobble = smoothness-control 문제 (진단·교정) | 2026-07-01 | active |
| [[research/legacy/cruise_timeout_arming]] | CRUISE 타임아웃의 근본 원인 — Teleport 후 PX4 Arm 거부 (Stale EKF) | 2026-06-15 | active |
| [[research/legacy/curriculum_phase_convergence]] | Phase 커리큘럼 실학습 수렴 특성 — warm-start 효과와 릴리스 갭 | 2026-07-13 | active |
| [[research/legacy/detection_gate_vs_altitude]] | 핸드오프 윈도우의 진짜 레버 = 탐지 게이트 (고도 아님) | 2026-06-22 | active |
| [[research/legacy/ekf_east_reversal]] | ⚠️ RETRACTED — "EKF East 축 반전"은 오진이었음 | 2026-06-12 | retracted |
| [[research/legacy/eval_terminal_env_metrics]] | v13 정책 평가 — EKF divergence 루프 & harness 지표 정합성 | 2026-06-20 | active |
| [[research/legacy/exp014_ablation_protocol]] | exp_014 ablation 설계 — max_altitude 33%의 원인 분리 (physics-kick vs climb-and-farm) | 2026-07-04 | executed |
| [[research/legacy/isaac_cruise_handoff_junsang]] | Isaac cruise 핸드오프 — reset 시 컨트롤러 setpoint seed | 2026-07-15 | active |
| [[research/legacy/isaac_expansion_roadmap_junsang]] | Isaac 모델 확장 로드맵 — CCIP residual 다음단계 개요 + 남은 축 정리 | 2026-07-15 | active |
| [[research/legacy/isaac_inertia_ctrl_mismatch]] | set_inertias는 solver에 전파된다 — exp_013은 rate loop이 ~1300× 저토크인 plant에서 학습됐다 | 2026-07-05 | confirmed |
| [[research/legacy/isaac_lab_experiment_workflow]] | Isaac Lab 실험 워크플로우 (feat/isaac-env-migration) — 신규 연구자 온보딩 | 2026-07-03 | active |
| [[research/legacy/isaac_lab_reward_tuning]] | Isaac Lab 보상·하이퍼파라미터 레퍼런스 (feat/isaac-env-migration) | 2026-07-03 | active |
| [[research/legacy/isaac_lab_wandb_guide]] | Isaac Lab WandB 메트릭 가이드 (feat/isaac-env-migration) | 2026-07-03 | active |
| [[research/legacy/isaac_model_intro_junsang]] | 드론 정밀 투하 모델 — 한 에피소드로 보는 소개 | 2026-07-23 | active |
| [[research/legacy/isaac_model_spec_junsang]] | Isaac 드론 모델 스펙 — 전체 파라미터 한눈에 | 2026-07-19 | active |
| [[research/legacy/isaac_ppo_tuning_recommendations]] | Isaac PPO 1차 학습 결론 — 무엇을 바꿔야 하는가 (exp_013 기반) | 2026-07-03 | active |
| [[research/legacy/isaac_v11_v13_design_guide_junsang]] | Isaac v11~v13 설계 가이드 — 기존 migration 모델과 무엇이 달라졌나 | 2026-07-15 | active |
| [[research/legacy/isaac_v18_curriculum_continuation_junsang]] | Isaac v18 커리큘럼 — 학습결과 백업 & Phase 3+ 재개 가이드 | 2026-07-18 | active |
| [[research/legacy/isaac_v19_collapse_nodrop_junsang]] | Isaac v19 붕괴 진단: no-drop reward local-optimum + A/B/D 처방 | 2026-07-23 | active |
| [[research/legacy/isaac_viz_tools_junsang]] | Isaac 시각화 & 검증 도구 (과제 2) | 2026-07-19 | active |
| [[research/legacy/moving_target_models]] | 이동 타겟 모션 모델(CV/CA/CT) + Singer-KF 타겟 트래커 | 2026-07-28 | implemented |
| [[research/legacy/paper_research_plan]] | 논문 연구 계획 — Vision-conditioned CCIP-Residual RL for Precision Free-Fall Payload Delivery | 2026-08-02 | active |
| [[research/legacy/phase1_plan]] | Phase 1: CCIP 기반 자율 접근 비행 제어기 — 상세 연구 계획 | 2026-04-17 | active |
| [[research/legacy/phase1_summary_junsang]] | Phase 1 종합 정리 — Round 1 ~ 7 v3, 그리고 redux | 2026-06-05 | active |
| [[research/legacy/phased_curriculum]] | Phase별 순차 커리큘럼 — 설계·수식·warm-start 원리 | 2026-07-05 | active |
| [[research/legacy/release_terminal_stageB]] | Stage B — 릴리스-종단 구조로 release_rate 5.5% → 100% (exp_018) | 2026-07-06 | active |
| [[research/legacy/research_overview_for_paper]] | 연구 전체 통합 개요 — 계보 · warm-start 체인 · 논문화 전략 | 2026-08-02 | active |
| [[research/legacy/reset_throughput_bottleneck]] | 리셋 처리량 병목 = teleport 후 EKF 재수렴 (soft reset로 회피) | 2026-06-22 | active |
| [[research/legacy/rtf_fps_analysis]] | RTF(Real-Time Factor)와 학습 FPS 관계 분석 | 2026-04-16 | active |
| [[research/legacy/sac_bounded_action_target_entropy_junsang]] | SAC Entropy 발산의 근본 원인 — Bounded Action + Target Entropy | 2026-06-03 | active |
| [[research/legacy/sac_reward_density_junsang]] | SAC Per-step Reward Density와 발산 | 2026-05-31 | active |
| [[research/legacy/system_overview]] | 시스템 전체 구조 (Gazebo/PX4/ROS2 — jekyun 브랜치) | 2026-04-14 | active |
| [[research/legacy/terminal_overshoot_trap]] | v12 종단 보상 트랩 (Terminal "Overshoot Moat") | 2026-06-16 | implemented |

## experiments — 논문 단계 실험 (exp_025~) (18)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[experiments/exp_025_dr_scale_sweep_gate]] | exp_025 — DR_SCALE 스윕 유효성 게이트 (무학습) | 2026-08-27 | done |
| [[experiments/exp_026_release_rate_100hz]] | exp_026 — 릴리즈 판정을 물리 주파수(100 Hz)로 + 성공 반경 0.5 m | 2026-08-27 | done |
| [[experiments/exp_027_seen_unseen_3seed]] | exp_027 — seen / unseen 비교 + Table 1 + 잔차 상한선 | 2026-08-30 | done |
| [[experiments/exp_028_l1_sl_pilot]] | exp_028 — L1-SL 파일럿: 잔차를 강화학습이 아니라 지도학습으로 | 2026-09-01 | done |
| [[experiments/exp_029_l1_sl_generalization]] | exp_029 — L1-SL 일반화 감사: 미지 사거리 · 정책 전이 · 라벨 수 | 2026-09-02 | done |
| [[experiments/exp_030_gain_invariant_residual]] | exp_030 — 게인 무관 특징으로 정책 결합 해소 + Table 1 시드 정렬 | 2026-09-07 | done |
| [[experiments/exp_031_ou_wind]] | exp_031 — 시변(OU) 바람 ablation: 잔차의 마지막 미검증 전제 | 2026-09-07 | done |
| [[experiments/exp_032_icpj8p4r_l1rl_zero_pilot]] | exp_032 — L1-RL 파일럿 (0 초기화): 종단 보상만으로 드리프트를 찾는가 → 못 찾았다 (308 iter에서 중단) | 2026-09-13 | done |
| [[experiments/exp_033_tog7jqs5_l1rl_fixed_std_pilot]] | exp_033 — L1-RL 파일럿 (탐험 폭 고정): 탐험을 지키면 잔차가 드리프트를 찾는가 → 반대 방향으로 갔다 (110 iter 중단) | 2026-09-14 | done |
| [[experiments/exp_034_learned_temporal_filter]] | exp_034 — 누적 특징을 학습된 시간 필터(GRU)로 대체: 정상 바람에서 같은가(✅), 시변 바람에서 나은가(⛔ 라벨이 병목) | 2026-09-14 | done |
| [[experiments/exp_035_realised_label]] | exp_035 — 실현 라벨: 정답을 "지금 바람"에서 "낙하 중 실제 바람 적분"으로 → 방향은 맞고(τ=10 −11%), τ ≤ 3은 아직 손해 | 2026-09-24 | done |
| [[experiments/exp_036_uncertainty_head]] | exp_036 — 불확실성 헤드: 평균+분산을 배우고 분산에 반비례해 물러선다 → τ ≥ 1 전 구간에서 학습 팔 최고, τ=10은 오라클+EMA 수준 | 2026-09-24 | done |
| [[experiments/exp_037_relative_shrink_realistic_wind]] | exp_037 — 상대 분산 수축 + 현실 바람(평균풍 + 돌풍) 평가 → 현실 돌풍에서는 모든 학습 팔이 이득을 유지하고, 정상 학습판 GRU-S가 최고 | 2026-09-24 | done |
| [[experiments/exp_038_gate_aware_stage1]] | exp_038 — 게이트 인지 학습 1단계: 평활 항이 EMA를 대체한다 (통과) | 2026-09-25 | done |
| [[experiments/exp_039_gate_aware_stage23]] | exp_039 — 게이트 인지 학습 2·3단계: 게이트를 손실에 결합해도 평활 항 이상은 없다 | 2026-09-25 | done |
| [[experiments/exp_040_consistency_final]] | exp_040 — 시간 일관성 GRU(GRU-C) 최종 재학습·전 조건 비교: 동률 → 본 방법은 GRU-S + EMA 유지 | 2026-09-25 | done |
| [[experiments/exp_041_ema_alpha_sweep]] | exp_041 — EMA α 민감도: 0.1~0.4 평탄, 0.5부터 악화, 무필터는 유의하게 나쁨 → α = 0.3 확정 | 2026-09-25 | done |
| [[experiments/training_history]] | 전체 학습 히스토리 (RL_Project_Log.md에서 이전) | 2026-04-14 | active |

## experiments/legacy — exp_001~024 (37)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[experiments/legacy/exp_001_8otphxy8_linear_reward]] | Exp 001 — 선형 거리 보상 + CRUISE Retry | 2026-03-20 | completed |
| [[experiments/legacy/exp_002_reward_shaping_patches]] | Exp 002 — 보상 함수 패치 적용 Fresh Training | 2026-03-22 | pending |
| [[experiments/legacy/exp_003_rtf_dryrun]] | Exp 003 — RTF Dry-Run 비교 (RTF 1 / 2 / 4) | 2026-04-16 | completed |
| [[experiments/legacy/exp_004_rl_yolo_debug_vision]] | Exp 004: rl_yolo_debug — Vision-Based TRACKING + EKF 좌표 버그 수정 | 2026-06-12 | in_progress |
| [[experiments/legacy/exp_004_round5_hover_junsang]] | Exp 004 — Hover Exploit 대응: Round 4 발산 → Round 5 Terminal Penalty | 2026-05-31 | in-progress |
| [[experiments/legacy/exp_005_phase1_redux_junsang]] | Exp 005 — Phase 1 redux: Task 재설계 + Curriculum 정밀화 | 2026-06-04 | in-progress |
| [[experiments/legacy/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] | Exp 005: rl_yolo_v12_arm_fix — Arming-Rejection Throughput Fix | 2026-06-15 | in_progress |
| [[experiments/legacy/exp_006_v11_dryrun_junsang]] | Exp 006 — Isaac v11 완화(relaxed) 테스트: 첫 통합 단일-phase dry-run 성공 | 2026-07-15 | done |
| [[experiments/legacy/exp_006_xgzum51v_armdiag_dryrun]] | exp_006 — armdiag dry-run: v12 arm-reject는 "복구 불가"가 아니라 "10s 컷이 너무 빨랐다" | 2026-06-17 | done |
| [[experiments/legacy/exp_007_iyhfy5ps_v13_eval]] | Exp 007 — v13 정책 평가 (deterministic eval) | 2026-06-20 | complete |
| [[experiments/legacy/exp_007_v12_random_marker_junsang]] | Exp 007 — Isaac v12: 첫 확장 (랜덤 marker 스폰) 일반화 검증 | 2026-07-15 | done |
| [[experiments/legacy/exp_008_dryrun_alt10_handoff_window]] | Exp 008 — 핸드오프 윈도우 확장 (고도↑ 시도 → 탐지 게이트가 진짜 레버) | 2026-06-22 | complete |
| [[experiments/legacy/exp_008_v13_partial_obs_junsang]] | Exp 008 — Isaac v13: 부분관측(blind cruise → 7m 탐지) 학습 | 2026-07-15 | done |
| [[experiments/legacy/exp_009_softreset_throughput]] | Exp 009 — 리셋 처리량: EKF param A/B (실패) → soft reset (성공 ~3.9×) | 2026-06-22 | active |
| [[experiments/legacy/exp_009_v14_ccip_residual_junsang]] | Exp 009 — Isaac v14: DR + CCIP 잔차학습 (Stage A, 바람 관측) | 2026-07-16 | done |
| [[experiments/legacy/exp_010_byxyaf4d_v14_195k_eval]] | Exp 010 — v14 (byxyaf4d) 195K 체크포인트 평가: 65% + soft reset 장기검증 | 2026-06-23 | done |
| [[experiments/legacy/exp_010_v15_airframe_wind_junsang]] | Exp 010 — Isaac v15: 바람이 드론 기체에 실제 작용 | 2026-07-16 | done |
| [[experiments/legacy/exp_011_v16_physical_drop_junsang]] | Exp 011 — Isaac v16: 실제 물리 payload drop (analytic → rigidbody) | 2026-07-16 | done |
| [[experiments/legacy/exp_011_wobble_lpf_reward_damping]] | exp_011 — 10m 핸드오프 후 RL wobble: LPF A/B 진단 + 보상 댐핑(B+C) | 2026-07-01 | active |
| [[experiments/legacy/exp_012_isaac_migration_phase2]] | exp_012 — Isaac Lab migration Phase 2: env + PPO 코드 이식 | 2026-07-03 | active |
| [[experiments/legacy/exp_012_v17_pixel_vision_junsang]] | Exp 012 — Isaac v17: 픽셀 양자화 vision (대략 위치 → 접근하며 정밀화) | 2026-07-16 | done |
| [[experiments/legacy/exp_013_v18_integration_curriculum_junsang]] | Exp 013 — Isaac v18: 능력 통합 (인지+물리) + 커리큘럼으로 데드락 해결 | 2026-07-18 | done |
| [[experiments/legacy/exp_013_wcjklw7a_isaac_ppo_first_training]] | exp_013 — Isaac Lab 첫 프로덕션 PPO 학습 (2048 envs, 1000 iters) | 2026-07-03 | complete |
| [[experiments/legacy/exp_014_A2_visionrange]] | exp_014 — plant 수정(질량/inertia/로터) + A2 비전 거리감쇠 | 2026-07-05 | done |
| [[experiments/legacy/exp_014_v19_full_integration_junsang]] | Exp 014 — Isaac v19: 전체 통합 (v18 + 실제 물리 drop) | 2026-07-19 | done |
| [[experiments/legacy/exp_015_phased_curriculum]] | exp_015 — Phase별 순차 커리큘럼 학습 (CCIP+Residual → 이동타겟) | 2026-07-05 | complete |
| [[experiments/legacy/exp_015_v19_abd_retrain_junsang]] | Exp 015 — Isaac v19 재학습: no-drop 붕괴 수정 (A+B+D) | 2026-07-23 | done |
| [[experiments/legacy/exp_016_ccip_release_reeval]] | exp_016 — CCIP 릴리스 referee 수정 + A2 200-ep 재평가 | 2026-07-05 | done |
| [[experiments/legacy/exp_016_v19_precision_landing_junsang]] | Exp 016 — Isaac v19 정밀도 향상: 연속 착지보상 | 2026-07-23 | done |
| [[experiments/legacy/exp_017_stageA_aim_reward]] | exp_017 — Stage A: 밀집 CCIP 조준오차 보상 (release_rate 개입 1차) | 2026-07-06 | done |
| [[experiments/legacy/exp_018_release_terminal]] | exp_018 — Stage B: 릴리스-종단 이벤트 + 보상 재탐색 | 2026-07-06 | done |
| [[experiments/legacy/exp_019_physical_payload]] | exp_019 — 물리 페이로드 attach/detach (kinematic weld) 구현·검증 | 2026-07-21 | done |
| [[experiments/legacy/exp_020_o5jn9xzk_payload_training]] | exp_020 — 물리 페이로드 부착 상태 첫 학습 + wandb 평가 figure 파이프라인 | 2026-07-23 | done |
| [[experiments/legacy/exp_021_v19_moving_target]] | exp_021 — v19 + 이동 타겟(CV/CT/CA) warm-start 학습 | 2026-07-30 | done |
| [[experiments/legacy/exp_022_p0_handoff_dyn_dr]] | exp_022 — P0: 핸드오프 랜덤화 + 동역학/센싱 DR (v20 env 신설) | 2026-08-03 | done |
| [[experiments/legacy/exp_023_table1_baselines]] | exp_023 — Table 1 1차: 규칙 기반 릴리스 베이스라인 vs 학습 정책 | 2026-08-03 | done |
| [[experiments/legacy/exp_024_v20_warmstart_failure]] | exp_024 — (a) 안 검증: v20(방위 랜덤) warm-start 학습은 과제가 아니라 페널티 회피로 수렴한다 | 2026-08-03 | done |

## errors — 에러 해결 기록 (13)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[errors/err_20260830_aim_reward_residual_inclusive]] | task_env의 dense aim 보상이 잔차 포함 오차를 먹고 있다 | 2026-08-30 | resolved |
| [[errors/err_20260827_payload_drag_body_frame]] | err_20260827 — 페이로드 항력을 월드 프레임으로 계산해놓고 링크 프레임으로 전달 | 2026-08-27 | resolved |
| [[errors/err_20260827_free_exit_termination]] | err_20260827 — 무료 탈출구: 벌하지 않는 종료 조건 + 탐지 뒤에 갇힌 접근 보상 | 2026-08-27 | resolved |
| [[errors/err_20260823_ccip_vz_omission]] | err_20260823 — CCIP ballistic_impact가 수직속도를 누락 | 2026-08-23 | resolved |
| [[errors/err_20260803_payload_landing_latch]] | 물리 페이로드 착지 래치가 "정지한 페이로드"를 영원히 놓친다 | 2026-08-03 | resolved |
| [[errors/err_20260723_wandb_key_empty]] | err_20260723 — 컨테이너 baked-in WANDB_API_KEY 공백 → 학습 silent 실패 | 2026-07-23 | resolved |
| [[errors/err_20260703_vision_env_origin_frame]] | Isaac Lab 비전 완전 사멸 — env-origin 좌표 프레임 혼용 | 2026-07-03 | resolved |
| [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] | 인시던트 — armdiag dry-run이 v13 30K 체크포인트를 파괴 (YAML 중복 키 + fresh-start 삭제) | 2026-06-17 | resolved |
| [[errors/err_20260615_cruise-timeout-arming]] | Err — CRUISE 타임아웃 (Teleport 후 PX4 Arm 거부) | 2026-06-15 | resolved |
| [[errors/err_20260528_gz_timeout_recurrence_junsang]] | err 2026-05-28 — gz model --list TimeoutExpired (반복 인프라 크래시, #021) | 2026-06-03 | mitigated |
| [[errors/err_20260520_spin_thread_recursive_reset]] | Spin Thread Death → Recursive reset() Crash | 2026-05-20 | resolved |
| [[errors/err_20260320_physics_explosion]] | Err — Gazebo 물리 폭발 (d_xy = 1.98×10¹¹ m) | 2026-03-20 | resolved |
| [[errors/err_20260319_ode_aabb_crash]] | Err — ODE AABB 크래시 (드론 스폰 고도) | 2026-03-19 | resolved |

## daily — 연구 일지 (44)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[daily/daily_2026-09-25]] | 연구 일지 — 2026-09-25 | 2026-09-25 | complete |
| [[daily/daily_2026-09-24]] | 연구 일지 — 2026-09-24 | 2026-09-24 | complete |
| [[daily/daily_2026-09-14]] | 연구 일지 — 2026-09-14 | 2026-09-14 | complete |
| [[daily/daily_2026-09-13]] | 연구 일지 — 2026-09-13 | 2026-09-13 | complete |
| [[daily/daily_2026-09-07]] | 연구 일지 — 2026-09-07 | 2026-09-07 | complete |
| [[daily/daily_2026-09-02]] | 연구 일지 — 2026-09-02 | 2026-09-02 | complete |
| [[daily/daily_2026-09-01]] | 2026-09-01 — 잔차를 지도학습으로 돌려서 CEP를 31% 줄였다 (PPO 없이) | 2026-09-01 | done |
| [[daily/daily_2026-08-30]] | 연구 일지 — 2026-08-30 | 2026-08-30 | complete |
| [[daily/daily_2026-08-29]] | 연구 일지 — 2026-08-29 | 2026-08-29 | complete |
| [[daily/daily_2026-08-27]] | 연구 일지 — 2026-08-27 | 2026-08-27 | complete |
| [[daily/daily_2026-08-23]] | 연구 일지 — 2026-08-23 | 2026-08-23 | complete |
| [[daily/daily_2026-08-03]] | 연구 일지 — 2026-08-03 | 2026-08-03 | complete |
| [[daily/daily_2026-08-01]] | 연구 일지 — 2026-08-01 | 2026-08-01 | complete |
| [[daily/daily_2026-07-30]] | Daily — 2026-07-30 | 2026-07-30 | done |
| [[daily/daily_2026-07-28_team_briefing]] | 팀 브리핑 — isaac_jk 브랜치 현황 (2026-07-28) | 2026-07-28 | active |
| [[daily/daily_2026-07-28]] | Daily 2026-07-28 | 2026-07-28 | done |
| [[daily/daily_2026-07-23]] | 연구 일지 — 2026-07-23 | 2026-07-23 | complete |
| [[daily/daily_2026-07-16_junsang]] | 연구 일지 — 2026-07-16 | 2026-07-16 | complete |
| [[daily/daily_2026-07-15_junsang]] | 연구 일지 — 2026-07-15 | 2026-07-15 | complete |
| [[daily/daily_2026-07-13]] | 연구 일지 — 2026-07-13 | 2026-07-13 | complete |
| [[daily/daily_2026-07-07]] | 연구 일지 — 2026-07-07 | 2026-07-07 | complete |
| [[daily/daily_2026-07-06]] | Daily — 2026-07-06 | 2026-07-06 | done |
| [[daily/daily_2026-07-05_gazebo_v15_regression]] | 연구 일지 — 2026-07-05 | 2026-07-05 | active |
| [[daily/daily_2026-07-05]] | Daily 2026-07-05 | 2026-07-05 | done |
| [[daily/daily_2026-07-04]] | 연구 일지 — 2026-07-04 | 2026-07-04 | complete |
| [[daily/daily_2026-07-03]] | 연구 일지 — 2026-07-03 | 2026-07-03 | complete |
| [[daily/daily_2026-07-01]] | 연구 일지 — 2026-07-01 | 2026-07-01 | complete |
| [[daily/daily_2026-06-23]] | 연구 일지 — 2026-06-23 | 2026-06-23 | complete |
| [[daily/daily_2026-06-22]] | 연구 일지 — 2026-06-22 | 2026-06-22 | complete |
| [[daily/daily_2026-06-20]] | 연구 일지 — 2026-06-20 | 2026-06-20 | complete |
| [[daily/daily_2026-06-17]] | Daily 2026-06-17 — v13 처리량 병목 진단 & arm_bail 수정 | 2026-06-17 | done |
| [[daily/daily_2026-06-16]] | 연구 일지 — 2026-06-16 | 2026-06-16 | complete |
| [[daily/daily_2026-06-14]] | 연구 일지 — 2026-06-14 (06-12 이후 종합 요약) | 2026-06-14 | complete |
| [[daily/daily_2026-06-12]] | 연구 일지 — 2026-06-12 | 2026-06-12 | complete |
| [[daily/daily_2026-06-09]] | 연구 일지 — 2026-06-09 | 2026-06-09 | complete |
| [[daily/daily_2026-06-05_junsang]] | 연구 일지 — 2026-06-05 | 2026-06-05 | complete |
| [[daily/daily_2026-06-03_junsang]] | 연구 일지 — 2026-06-03 | 2026-06-03 | complete |
| [[daily/daily_2026-05-31_junsang]] | 연구 일지 — 2026-05-31 | 2026-05-31 | complete |
| [[daily/daily_2026-05-30_vision-refactor]] | 연구 일지 — 2026-05-30 (Vision Refactor / junsang 브랜치) | 2026-05-30 | complete |
| [[daily/daily_2026-05-30]] | 연구 일지 — 2026-05-30 | 2026-05-30 | complete |
| [[daily/daily_2026-04-23]] | 연구 일지 — 2026-04-23 | 2026-04-23 | complete |
| [[daily/daily_2026-04-17]] | 연구 일지 — 2026-04-17 | 2026-04-17 | complete |
| [[daily/daily_2026-04-16]] | 연구 일지 — 2026-04-16 | 2026-04-16 | complete |
| [[daily/daily_2026-04-14]] | 연구 일지 — 2026-04-14 | 2026-04-14 | complete |

## sessions — 세션 기록·명령 (11)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[sessions/session_2026-09-25]] | 세션 — 2026-09-25 | 2026-09-25 | complete |
| [[sessions/session_2026-09-24]] | 세션 — 2026-09-24 (UTC 04:50 ~ 23:30) | 2026-09-24 | complete |
| [[sessions/session_2026-09-14]] | 세션 — 2026-09-13 15:00 ~ 2026-09-14 04:00 (UTC) | 2026-09-14 | complete |
| [[sessions/session_2026-08-27]] | 세션 기록 — 2026-08-27 | 2026-08-27 | complete |
| [[sessions/session_2026-07-16_junsang]] | 세션 로그 — 2026-07-16 | 2026-07-16 | complete |
| [[sessions/session_2026-07-15_junsang]] | 세션 로그 — 2026-07-15 | 2026-07-15 | complete |
| [[sessions/session_2026-07-06]] | Session — 2026-07-06 (exp_017 Stage A + exp_018 Stage B) | 2026-07-06 | done |
| [[sessions/session_2026-07-05]] | 세션 2026-07-05 — plant 수정 실행 + inertia 대반전 + exp_014 A2/A0′ | 2026-07-05 | done |
| [[sessions/session_2026-04-16]] | 세션 일지 — 2026-04-16 | 2026-04-16 | complete |
| [[sessions/session_2026-04-14]] | 세션 2026-04-14 — Obsidian 연구 비서 시스템 초기화 | 2026-04-14 | completed |
| [[sessions/commands]] | 자주 쓰는 명령어 모음 (Isaac Lab, 2026-09 기준) | 2026-04-16 | active |

## Environment — VM·접속 (2)

| 파일 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [[Environment/README]] | GCP VM 완전 복구 가이드 — Drone Bombard 환경 |  |  |
| [[Environment/vm_access_guide_junsang]] | L4 VM 접속 가이드 (Isaac 학습용) — 팀원용 | 2026-07-23 | active |

---

총 192편.
