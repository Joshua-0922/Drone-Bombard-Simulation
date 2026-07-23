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
| 2026-05-20 | 0rho5l9f | 5K (smoke) | 23.95m | SAC default baseline smoke. drops=1, ep_rew 평탄. |
| 2026-05-20 | pwkujvev | 10K (smoke) | 14.87m | H1+H2+H3+M1+M2 적용 (buffer 500k, gamma 0.995, gradient_steps 4, terminal scale↓, action 5d→4d). drops=1 (38%↓). |
| 2026-05-22 | um8txjvk | 200K | — | **N1=B v2 (w_impact=8) — 실패.** deterministic 0/5 drops, takeoff도 실패. 14m systematic offset 진단 → `jekyun_v2` branch 교체. |
| 2026-05-22 | tzbebmm4 | 1K (dry-run) | — | jekyun_v2 base (junsang_v2). Training complete, sim 정상 동작 확인. |
| 2026-05-22 | zn7xrm7e | 200K | 9.02m | **junsang_v2 본학습 — 실패.** critic 폭주 17K@63k, ent_coef 1.13 진동. 원인: M2(gradient_steps=4) + jekyun 큰 terminal reward 충돌. 14m offset은 해소(인프라 patch). |
| 2026-05-23 | w9flirvp | ~159K | ~20m | **junsang_v3 (M2 revert 4→1) — 실패.** SAC 안정성 회복했으나 정책 학습 안 됨 (mean_d_xy 20m 정체). env/reward 측 근본 문제로 판단. |
| 2026-05-23 | ujvpo8ry(5k) / krxfl97k(200k) | 200K | 12~14m | **junsang_v4 Tier 1 (P1~P11).** action scale 8 + rate limit 0.2 + curriculum 10m + bad_attitude 검사. success=0, inverted 44%. (숨은 버그: drop_attempt_bonus/truncation_penalty hardcoded 발견·수정) |
| 2026-05-24~25 | wdwoim34(19k crash) / xk7rw5e1(112k) | 112K | 13.15m | DropEpisodeRecorder 추가 + 재학습. success=0, 이전과 동일 양상. |
| 2026-05-25 | ruozrv5x | 150K | 14.02m (best 4.64) | **Round 1: #001+#002+#003+#006** — hybrid drop + 보상 스케일 축소 + 종료조건 정비. 432 drops, success 1건. d_xy 11.3m 정체. |
| 2026-05-26~30 | dbi74uif / z05fx7g9 | 150K | 19.09m (best 2.53) | **Round 2: gradient + max_steps 800 + 종료조건 진화.** success 16건 (16배↑). **post-success regression 발견.** (dbi74uif는 random_drop_start 150 너무 일러 접근 실패) |
| 2026-05-30~31 | q13hli0y(발산) / lidq3ydu(157k) | 157K (crash) | 22.42m (best 4.32) | **Round 3 (조합 C): PER(α0.6) + LR 1e-4 + Tau 0.002 + Sigmoid alt penalty + 4 안전장치.** success 8건 (Round 2 대비 2배 속도), 100~125k 최우수(avg 13.9m). PX4 로그 20GB 누적 → `gz model --list` timeout 크래시. → [[experiments/exp_004_round5_hover_junsang]] |
| 2026-05-31 | vo1l9wl6(14k 버그) / 4j46qwpk(146k) | 146K (발산) | — | **Round 4 (A+C): Hover per-step 차단** — w_heading 0.7→0.3, w_distance_penalty 0.03 신규. **학습 발산** — ent_coef 6.03, critic_loss 230K+. 원인: per-step density 축소 → reward sparsity↑ → SAC auto-entropy 양성 피드백. → [[research/sac_reward_density_junsang]] |
| 2026-05-31 | sdjytkpv | 300K (학습 중) | — | **Round 5: Hover Terminal Penalty** — Round 4 처방 복원(w_heading 0.7, distance_penalty 0) + episode 종료 시 -15 (sustained hover만, drop 시 제외). per-step density 보존 → SAC 안정성 유지. → [[experiments/exp_004_round5_hover_junsang]] |
| 2026-07-15 | v11_dryrun (tensorboard, Isaac/PPO) | 512env × 300iter | **착탄 ~0.75m** | **[Isaac 트랙 시작] v11 완화 테스트 dry-run — 성공.** 통합 단일 phase + 고정 marker(cruise 20m) + drop_signal + release envelope. ~iter88 수렴 → success 100%, release 100%, reward ~314. 버그: cruise 핸드오프 자세 폭주(step-1 즉사) → reset 시 컨트롤러 seed로 수정 (Rule 10). → [[experiments/exp_006_v11_dryrun_junsang]] |
| 2026-07-15 | v12_dryrun (tensorboard, Isaac/PPO) | 512env × 300iter | **착탄 ~0.75m** | **v11 첫 확장: 랜덤 marker — 성공.** (20,0) 중심 5m 원 면적균일 랜덤 스폰(방식 A, 드론 +X 순항→조향). marker_random 토글. ~iter49 수렴 → success 100%, release 100%, reward ~316. v11 학습이 다양한 위치 타겟에 일반화 확인. → [[experiments/exp_007_v12_random_marker_junsang]] |
| 2026-07-15 | v13_dryrun (tensorboard, Isaac/PPO) | 512env × 300iter | **착탄 ~0.8m** | **v13 부분관측 — 성공.** blind +X 순항 → 수평 7m 진입 시 marker 공개(연속·비latch), 미탐지 -0.2/step 페널티, marker 보상 detected 게이팅(위치 누출 방지), obs 25D. ~iter48 수렴 → success 100%, release 100%, reward ~290(blind 구간 페널티로 v11/v12보다↓). vision 징검다리. → [[experiments/exp_008_v13_partial_obs_junsang]] |
| 2026-07-16 | v14_dryrun / v14_nores (Isaac/PPO) | 512env × 300iter | **착탄 0.69 vs 0.82m** | **v14 DR+CCIP residual (Stage A).** wind N(0,1)·drag DR + residual action[5:7]±3m + 바람 관측(obs 27D). residual ON vs OFF: 착탄 0.69/0.82m, success 81.5/72.2%. residual 우세하나 격차 작음 = 바람이 payload에만 작용해 대조군도 비행보정으로 커버. 시드 1개. → [[experiments/exp_009_v14_ccip_residual_junsang]] |
| 2026-07-16 | v15_dryrun (wind 4.0, 중단) | 512env | **착탄 ~3m 정체** | **v15 바람이 기체에 작용.** wind-test 실측(OFF 0.01°→ON 3.5°) 검증. wind 4.0 dry-run 정체 원인=**residual(±3m) 포화**(드리프트 3.7~7.7m 초과). 대조군 release 붕괴. → wind_std 2.0으로 하향(재실행 X). → [[experiments/exp_010_v15_airframe_wind_junsang]] |
| 2026-07-16 | v16_dryrun (Isaac/PPO) | 512env × 250iter | **실제 착탄 sub-meter** | **v16 실제 물리 payload drop — 성공.** RigidObject 운반→투하→중력+drag 낙하→지면 착지 기반 학습(land-terminal). drop-test PASS(탄도곡선 \|dz\|0.10m). dry-run iter156 success 0.80·release 100%·실제 착탄 0.23~0.42m. analytic→물리 drop 전환 성공. → [[experiments/exp_011_v16_physical_drop_junsang]] |
| 2026-07-16 | v17_dryrun (Isaac/PPO) | 512env × 300iter | **착탄 ~0.75m** | **v17 픽셀 양자화 vision — 성공.** v13 위에 위치를 셀 중심으로 대략 판정(cell=0.15·slant, 멀면 애매·가까우면 정밀). 정책은 양자화값만 인지·success는 실제. iter48 success 0.83 → iter114 0.67~0.83·release 100%·착탄 ~0.75m. 애매한 픽셀로 접근→정밀투하 학습. 토글 가능. → [[experiments/exp_012_v17_pixel_vision_junsang]] |
| 2026-07-18 | v18_eased (통합 Phase1) | 512env × 300iter | **착탄 0.53m** | **v18 능력 통합(인지 v17 + 물리 v14/15) — Phase1 완화.** from-scratch로 다 켜니 **투하 데드락**(초기 랜덤 residual이 게이트 봉쇄 → release 0). 완화(gate 1.0→1.5·res 3→2·wind 2→1.5·k 0.15→0.12)로 부트스트랩 → success 1.0·release 100%·착탄 0.53m. → [[experiments/exp_013_v18_integration_curriculum_junsang]] |
| 2026-07-18 | v18_phase2 (통합 Phase2, warm-start) | resume model_200 × 200iter | **착탄 ~0.65m** | **v18 Phase2 hard 커리큘럼.** Phase1 모델 warm-start + hard 복원(gate 1.0·res 3.0·wind 2.0·k 0.15). 시작부터 success 0.87 → 최종 **success 1.0·release 100%·착탄 ~0.65m**, 데드락 없음. 커리큘럼이 통합 정석 해법 확인 (Rule 12). → [[experiments/exp_013_v18_integration_curriculum_junsang]] |
| 2026-07-19 | v19_warmstart (전체 통합+물리drop) | resume v18-P2 × 200iter | **실제 착탄 0.56m** | **v19 = v18 + 실제 물리 payload 낙하 (통합 완성).** land-terminal, 실제 착지 기반. v18 모델 warm-start → release 처음부터 100%, ~iter75 수렴 → success 1.0·실제 물리 착탄 0.56~0.67m. payload가 바람 속 실제 낙하해 명중. → [[experiments/exp_014_v19_full_integration_junsang]] |
| 2026-07-23 | v19_abd_full (붕괴수정 재학습) | resume v18-P2, iter300→600 × 1024env | **착탄 0.563m** | **v19 no-drop 붕괴 수정 (A+B+D).** 옛 v19(iter499)가 상주 CCIP shaping 수확으로 release 0% 붕괴 → A(포텐셜 shaping)+B(loiter 페널티)+D(best저장). 재학습: iter600까지 **release 96~100% 유지(붕괴 없음)**, best=iter599 **success 76.7%·착탄 0.563m**(옛 best 복원+success↑). 재적응 딥(350~450)에서 release 일시 0→회복(D 필요성 실증). → [[experiments/exp_015_v19_abd_retrain_junsang]] (Rule 13) |
| 2026-07-23 | v19_precise (정밀 착지보상 재학습) | resume v19-abd, iter599→899 × 1024env | **착탄 0.356m** | **v19 정밀도 향상 (연속 착지보상).** 성공존 내부 평평보상(0.1m·0.9m 동일 300)이 med_err를 0.56m로 정체 → 연속형 `300·exp(-2·err)+100`으로 교체 + best warm-start. best=iter875 **success 100%·release 100%·착탄 0.356m**(이전 0.563m 대비 37%↓, success 76.7→100%). → [[experiments/exp_016_v19_precision_landing_junsang]] |
