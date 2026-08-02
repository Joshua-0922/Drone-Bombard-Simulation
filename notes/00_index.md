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

## 현재 상태 (2026-08-02)

- **📄 논문 연구 계획 (앞으로의 설계도): [[research/paper_research_plan]]** — 문헌 105편 스윕 + `Learning to Throw`/`AeroThrow` 정독 기반. **비어 있는 칸 = 자유낙하·멀티로터·순항 릴리스·학습 트리거·유도레벨 잔차·탐지 조건화·바람 DR·이동표적의 조합.** 주장 3종(C1 유도레벨 잔차 / C2 좌표 비복원 탐지 조건화 / C3 릴리스=종단 구조 귀속)을 각각 최근접 선행에 대해 헷지한 문구로 고정. 표 7종 설계(T0~T5 베이스라인 · R0~R3 잔차 위치 · S0~S4 구조 vs 보상 · 보상 병리 · 충실도 사다리 · 인지 사다리 · 리드 L0~L5), 지표 5종(+CEP, 반송시간), 실행 순서 P0~P4, 리뷰어 공격 8종 선제 대응.
- **📄 연구 전체 통합 개요 (논문용): [[research/research_overview_for_paper]]** — 두 트랙(Gazebo/SAC, Isaac Lab) + 세 계보(base env 커리큘럼 / v-track 사다리 v11~v20 / 이동타겟)를 한 문서로 합침. 각 v가 추가한 기능·obs/action·warm-start 출처·결과 표, 체크포인트 혈통도, 확정 발견 12종(F1~F12), **이미 확보한 ablation 13축 + 추가로 필요한 실험 N1~N7**, 방법론 약점 7종(단일 시드·select_best 선택편향·rule-based 베이스라인 부재 등).
- **준상 v-track 연구노트 34개 `main` 복원 (2026-08-02).** 구 main 아카이빙 때 태그 `archive/main-pre-isaac_jk-promotion`에만 남아 있던 `notes/**/*_junsang.md`(v11~v19 1차 사료 + 모델 스펙 + 붕괴 진단 + SAC 초기 연구)를 전부 복원. 진입점: [[research/isaac_model_spec_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[research/isaac_v19_collapse_nodrop_junsang]] · [[00_index_junsang]]. ⚠️ `exp_006`~`exp_016` 번호가 제균 트랙과 충돌하니 인용 시 `_junsang` 접미사로 구분할 것.
- **`.gitignore` 정비 (2026-08-02).** `ros2_ws/` 접두사 누락으로 YOLO datasets/epoch 가중치 규칙이 무효였던 것 수정 + `rl_abtest_*`/`rl_dryrun_*`/`rl_softreset_test`(현재 untracked 148MB) 제외 → `git add .` 사고 방지. Obsidian 로컬 UI 상태(`workspace.json`)는 추적 해제. **이미 커밋된 367MB(datasets .jpg 173MB + epoch*.pt 180MB + ign_recording.mp4 14MB)는 `git rm --cached` 별도 판단 필요.**

## 현재 상태 (2026-08-01)

- **브랜치 정리: `main` = `isaac_jk`로 승격 (2026-08-01).** 그동안 `main`이 07-03 시점(`Isaac Lab migration Phase 1 skeleton`)에 정체돼 있었음 — `isaac_jk`가 실제 진행 중인 유일한 통합 브랜치임을 확인 후 승격. 구 `main`(junsang `_junsang` 연구노트 20여 개 + 초기 `isaac_lab_tasks/` 스켈레톤)은 태그 `archive/main-pre-isaac_jk-promotion`으로 보존. `Isaac-JS`(제균 개인 브랜치, 07-02 이후 Gazebo/SAC 트랙만 진행돼 Isaac Lab 코드 없음)는 고유 연구노트(`daily_2026-07-05_gazebo_v15_regression`/`daily_2026-07-07`, Rule 25/26)만 `isaac_jk`로 포팅 후 삭제. `Issac_JS`(junsang, 오타 아님)는 미변경 — 단, 세션 중 junsang이 새 커밋(v20 task 등록)을 푸시해 아직 `main` 미반영 상태. → [[daily/daily_2026-08-01]]
- **팀 브리핑(2026-07-28): [[daily/daily_2026-07-28_team_briefing]]** — isaac_jk 머지/이동타겟·KF 현황 + 팀원별 할 일
- **병행 트랙 — Isaac Lab: 이동 타겟(CV/CA/CT) + Singer-KF 트래커 구현 (2026-07-28).** `--target_motion {gm,cv,ca,ct}`·`--moving_target`·`--target_kf`(obs 14→21) 신설, 단위테스트 57/57 + 스모크 4종 PASS, KF 추적오차 0.09–0.12 m. **--target_kf는 14-dim ckpt warm-start 불가(Fresh Start).** → [[research/moving_target_models]]
- **병행 트랙 — Isaac Lab: exp020 시연 영상 파이프라인 + 렌더러 포렌식 (2026-07-24).** `record_episode.py` 재작성(metric-only referee로 발화→detach→실낙하 관측, trajectory NPZ 덤프) + `animate_episode.py`(matplotlib 3D/탑다운 mp4) + `sdg_dtype_patch.py`(annotator attach 차단 해제 핫패치). **실측 첫 데이터: CCIP pred err 0.111 m vs 실낙하 측정 miss 0.342 m** (seed 0, ×3 결정론 재현). **RTX 렌더 붕괴의 근본 원인 = 07-05 ultralytics pip이 교체한 numpy 2.4.6(omni ABI 파괴) — numpy 1.26.4 다운그레이드로 완전 해결, TiledCamera 진단도 같은 뿌리로 재해석.** 산출물: `logs/recordings/drone_bombard_exp020_payload_final.mp4`(실사 RTX 체이스캠) + `_anim.mp4`(matplotlib). → [[errors/err_20260724_isaac_render_frozen_fabric]]
- **병행 트랙 — Isaac Lab: exp_019 물리 페이로드 attach/detach 구현·검증 완료 (4/4 PASS, Rule 24).** 코드 전수 검토에서 결함 6종 발견(페이로드 물리 부재·`_payload_attached` 죽은 플래그·마커 env-0 게이팅·ctrl_mass 스칼라·릴리스=즉시 종단·CCIP vz 생략) → ①②③ 수정: 해석적 페이로드를 실제 per-env RigidObject(0.1 kg 실린더)로 교체. **GPU-복제 물리는 per-env 조인트 토폴로지 변경 불가 → kinematic weld 패턴**(부착 env만 매 physics step pose+vel write, CCIP 발화 → release_delay 카운트다운 → write 중단 = 자유낙하, z≤0.10 m 측정 착탄 래치). hover-drop 강제 릴리스 검증(8 envs, isaac-verify): 부착 추적 **1.1 mm**, 분리 0/8 잔류, 착탄 8/8, **측정 vs 해석적 CCIP |Δ| mean 0.012/max 0.021 m** — 물리↔해석 cm-parity 계측 증명. 보상·종단·referee bit-identical(순수 추가), `payload_impact_rate`/`payload_impact_err_measured_m` 지표 신설. 후속(별도 지시): 에피소드 착탄-연장, Phase-2 DR 힘 정합, vz 복원, per-env ctrl_mass. → [[experiments/exp_019_physical_payload]] / [[research/physical_payload_attach]] / Rule 24
- **(이전 2026-07-13) 병행 트랙 — Isaac Lab: exp_015 이어학습(2차) 완주 — P2/P3 extended 각 +2000 iters (P2_EXIT=0, P3_EXIT=0).** §7 baseline 체크포인트에서 페이즈별 단독 연장(`isaac-verify`, 2048 envs, `release_terminal` 미적용). **P2 ext(iter 1098→3097):** drop tail **2.87 m**(1차 2.91 m 대비 ~0.04 m 개선), best_min 0.008 m 스파이크, release_rate **0.33→0.01** 급락, success **0**. **P3 ext(iter 3097→5096):** drop tail **5.31 m**(1차 3.20 m **회귀**), reward 101.7→**74.5**, lead tail 평탄 0.35, success **0**. **0.8 m 돌파 ❌** — iter 예산 확대만으로는 릴리스-종단 명중 미형성(Rule 20f). exp_018 `release_terminal` 종단 재구조 필요 재확인. 산출물: `logs/exp015_cont/` (`exp015_phase{2,3}_ext_final.pt`, `summary_p{2,3}_ext.json`). → [[experiments/exp_015_phased_curriculum]] §8 / [[research/curriculum_phase_convergence]] §2(e) / Rule 20f
- **(이전 2026-07-12) 병행 트랙 — Isaac Lab: exp_015 실학습 — Phase 1→2→3 커리큘럼 첫 end-to-end 완주 (baseline, ORCH_EXIT=0).** 2048 envs·600/500/500 iters·seed 42·~65 min(~2.3 s/iter). plain `--phases 1,2,3`(release_terminal·w_aim 미사용 = exp_015 원본 릴리스 메커니즘). **Phase 1만 완전 수렴**: success 0.48→**1.00**, reward→107(exp_014 100% 재현). **Phase 2**(CCIP+Residual+DR): reward −0.8→**94.7** 회복, `drop_impact_error_m` **4.66→2.91 m ↓**, release_rate peak 0.98/tail 0.33(변동), success ~0(착탄 2.9 m ≫ 반경 0.8 m). **Phase 3**(이동타겟): reward→**102**, `lead_error_m` best **0.071 m**(tail 평탄 0.34), release 0.10, success ~0. warm-start 무손실 실증(페이즈 경계 reward 딥→빠른 회복, Rule 20e). **정직 평가: reward 우상향은 proximity 지배 — P2/P3 릴리스-종단 명중 능력은 베이스라인 500 iter로 미형성(추가 학습 또는 exp_018 release_terminal 구조 필요). exp_016/017의 "근접≠릴리스" 커리큘럼 스케일 재확인.** ckpt/로그/수렴그래프 host 영속화(`logs/exp015_orch/`). → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]] / Rule 20
- **(이전) 병행 트랙 — Isaac Lab: exp_018 Stage B — 릴리스-종단 구조로 release_rate 5.5% → 100% (Rule 23).** 근접 종단(d_xy≤0.8)을 릴리스-발화 종단으로 교체(`release_terminal` cfg, 실패 게이트·타임아웃 불변) + aim_err 보상 nominal-전용 고정(residual 채굴 차단 명문화). **종단 교체 단독(B0, 보상=Stage A v1 그대로): 학습 내 release_rate 23→99.6% 단조 상승**(Stage A의 단조 하락 반전 — Rule 22a 인과 확정), det 200-ep **100.00%**, drop err **0.125 m**(max 0.198), 호버-드롭 프로파일(종단 속도 med 0.11 m/s). 보상 스윕(단일 노브): w_aim 0/1.0/1.5 전부 100%(aim 항 사실상 잉여 — 자동 발화 referee가 노이즈를 +100 샘플러로 전환), knee 0.75만 근소 열화(98.5%). 적대 검증이 done-flag alias 버그 사전 발견(eval success 0% 위험, `.clone()` 수정, Rule 23d). 구조 요인 3종(잘림/노이즈/할인) 전부 해소·역전. **Stage C(DR+residual, 별도 지시) warm-start = `exp018_B0_final.pt`.** → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] / Rule 23
- **(이전) exp_017 Stage A — 밀집 CCIP 조준 보상(보상-변경-단독)은 release_rate를 못 올림 → 판정 (b)·Rule 22.** exp_016의 6% 갭에 대한 1차 개입: `w_aim·(1-tanh(aim_err/s))` dense 항(referee와 동일 aim_err, cfg 기본 off, 5-lens 적대 검증 후 학습). **HEAD 6-dim P1 기준선 신규 학습**(exp_015는 스모크만·exp_014 ckpt는 4-dim): 학습 내 release_rate 12→3.7% **단조 하락**(근접 최적화가 릴리스 능력을 능동 파괴), det 2.5%. v1(w=1/knee 0.5): det **5.5%**, aim_err_min med 1.15→**0.89 m**, speed 3.35→**2.72 m/s**(방향 실재, n=200 p≈0.13). v2(w=2/knee 1.0, 600 it): **회귀**(3.5%/1.10 m/3.45 m/s) — σ 1.55 폭등, 수동 소득화. 구조 원인 3종(γ-할인 완주 보너스·CCIP 노이즈 증폭 ×1.5 s·성공 조기 종단) → **릴리스는 Phase 2 종단 구조로**(w_aim은 Phase 2+에 그대로 이월 금지 — residual 채굴 경로). 체크포인트 3종 분리 보존(+호스트 `/opt/drone-bombard/checkpoints/exp017/`), farm 시그니처 0. → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] / Rule 22
- **(이전) exp_016 — "success 100% vs drop_impact_error 4.59 m" 디커플링 규명·수정 (Rule 21).** Phase 1엔 릴리스 트리거가 없었고(`DropCfg.release_tolerance=0.2` 정의만·미사용), `drop_impact_error_m`이 **d_xy-성공 종단 스냅샷**(잔여속도 ~3.0 m/s 포함)의 탄도 예측 = **속도 캐리 3.0×1.53 s ≈ 4.6 m**(투하 오차 아님). 수정: 스크립티드 CCIP referee(≤0.2 m 최초 충족 시 래치, **지표 전용 — 보상/종단 bit-identical**) + `drop_impact_error_m`=릴리스 시점 재정의(구 지표는 `_terminal_m` 보존) + `release_rate`/`aim_err_min` 신설. **A2 200-ep 재평가: 발화 시 0.137 m, 단 release_rate 6%(10 Hz)/11.5%(100 Hz) — CCIP 스윕 최근접 med 0.755 m ≈ d_xy_min 0.665 m(경로 cross-track 지배).** 근접 학습 정책은 릴리스 능력이 없다 → exp_015 Phase 2 몫. exp_013의 24 m도 동일 의미론(디커플링은 exp_012 도입부터 구조적). → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] / Rule 21
- **(이전) exp_015 — Phase별 순차 커리큘럼 코드 완료(미학습) (`feat/isaac-env-migration`).** 이미지 3단계(접근/nominal → CCIP+Residual/정지타겟 → 이동타겟)를 완전 구현. action 4→6(δx/δy CCIP residual), `phase` 단일 노브+파생 플래그, 릴리스 이벤트(nominal CCIP+δ 예측 트리거 → 실제 DR 낙하 착탄오차 터미널 보상), drag/wind 도메인랜덤화, Gauss-Markov 이동타겟+lead 보상, `train.py --phases 1,2,3` 서브프로세스 오케스트레이터(6-dim 고정 → `runner.load()` warm-start 무손실). **로컬 `py_compile` 12파일 통과**, `pytest`(+8 신규)·본 학습은 L4/컨테이너 대기(dev 박스 torch 없음). Phase 1 = exp_014 baseline과 동작 동일. → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20
- **(이전) exp_014 완료 — plant 수정 + 비전 거리감쇠 → deterministic 200-ep eval 100.0% (202/202), d_xy_min 0.665 m (exp_013: 36%/1.4 m).**
  - **plant 수정 3종** (커밋 `cd0c617`/`7d0e9b6`): ①속도킥 → 스폰타임 `UsdPhysics.MassAPI` authoring (`--zero-actions` 11.9 m FAIL→0.2 m PASS) ②로터 ±200 rad/s 리셋 재주입 제거 ③**inertia 대반전** — `set_inertias`는 solver에 전파되고 있었음(`_diag_inertia.py` 계측): **exp_013은 rate loop ~1300× 저토크 plant에서 학습**, 구 정책은 plant-overfit(재평가 bad_att 68%) → [[research/isaac_inertia_ctrl_mismatch]] / Rule 19 신설.
  - **exp_014 A2** (감쇠 ON, `v3qk07pg`): R_alt(300-400)=**0.0000**, success 99.85%, noise_std 0.80 안정(폭주 없음 — Rule 18b 재해석: σ 폭주도 plant 아티팩트). climb 창발(iter 150-199, 27.8%) 후 50 iter 내 완전 기각. **A0′** (감쇠 OFF 대조, `azoc1xp0`): R_alt 0.0365, success 96.5% → **지배 요인 = plant 일관성, 감쇠 = 잔여 꼬리 제거 + YOLO parity(유지)**. 상세: [[experiments/exp_014_A2_visionrange]].
  - 실 YOLO 캘리브레이션은 컨테이너 annotator 버그로 차단(하네스는 수리 완료, 커브는 분석값 — calibration-pending). reward_success·entropy_coef 불변(다음 페이즈, [[research/isaac_ppo_tuning_recommendations]]).
- (이전) **exp_013 — 첫 프로덕션 PPO 학습 완주 + 진단 완료 (2026-07-03).** 2048 envs×1000 iters(65.5M steps, 43분, wandb `wcjklw7a`) → **deterministic 200-ep eval = 36%**, d_xy_min 1.4m plateau. 시작 직후 **비전 사멸 버그**(env-origin 프레임 혼용, [[errors/err_20260703_vision_env_origin_frame]]) 발견·수정 후 재기동. 실패 원인 3종 규명: ①analytic conf 거리감쇠 누락→고도 상승 farming(max_alt 33%, Rule 17) ②farmer(+225)>finisher(+121) 보상 불균형(Rule 18a) ③noise_std 0.8→3.92 폭주(Rule 18b). **07-04 forensics 재정정: 리셋 속도킥은 프로세스당 1회(첫 물리 substep, m_eff=0.02504kg 계측 확정)로 실증 — 학습 오염 사실상 없음, max_alt 27-43%는 iter ~200 창발 학습된 attractor(비전 farming 1차 가설 복권), 36% 수치 유효.** entropy 실측: 실행 속도 궤적 σ-불변(vel-Δ σ3.9/det=1.01×) → 공짜 entropy 확정. **다음: exp_014 = conf 거리감쇠 + reward_success 300 + entropy_coef 0 + 킥 위생수정, fresh ([[research/exp014_ablation_protocol]] 참조).** 온보딩 문서 3종(`isaac_lab_reward_tuning`/`wandb_guide`/`experiment_workflow`) 신설. 상세: [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]].
- **메인 트랙 — SAC (jekyun 브랜치, Gazebo/PX4):**

## 현재 상태 (2026-07-05, Gazebo/SAC 트랙 마지막 갱신 — 이후 이 트랙은 미진행)

- **알고리즘:** SAC, `net_arch=[256,256]`, L4 GPU
- **현재 학습:** ⏸️ `rl_yolo_v15_bc_stable` — 반복 reset-recursion abort로 오토레쥼 서포바이저(`run_train_supervised.sh`) 추가 후 0→310K. **정정(07-05): 310K 정지는 계획된 stop이 아니라 07-03 GPU 드라이버 업그레이드(Isaac Lab용, 535→580)로 `drone-bombard-harmonic` 컨테이너가 깨지며 강제 중단된 것.** **사용자 관찰: 훈련된 에이전트가 X마커에 도달 못 함(v14 대비 회귀).** 원인 후보: (1) 근접-속도 댐핑(`w_vel`, `vel_damp_radius=3.0m`)이 v14의 final-approach stagnation 구간(0.5–1.2m)을 재타격, (2) crash-resume마다 replay buffer 초기화로 학습 불연속 — **둘 다 미확정, 인프라 정합 및 재평가 없이 이 트랙은 여기서 멈춤.** → [[daily/daily_2026-07-05_gazebo_v15_regression]] / [[daily/daily_2026-07-07]] / Rule 25 / Rule 26
- **이번 세션 (2026-07-01) — RL wobble 진단·교정:** 사용자 관찰(10 m 핸드오프 후 RL 인수하나 wobble). eval `deterministic=True`라 **탐험 노이즈 아님 = 학습된 bang-bang 정책.** LPF A/B: PX4 수신 속도명령 **jerk RMS 2.92→1.61(−45%)**, 평균 속력 1.13×(안 느려짐) → **smoothness-control 문제 확정.** 교정: (B) 근접-게이팅 속도 댐핑 `w_vel=0.15/R=4`, (C) `w_ang_vel 0.05→0.15`·`w_action_smooth 0.05→0.20`, 로직 LPF `velocity_lpf_alpha=0.4`(학습==배포). dry-run PASS → v15 fresh 기동. → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15
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
| 014 | exp014 A2 (v3qk07pg) + A0′ (azoc1xp0), Isaac PPO | 각 26.2M steps (2048 envs×400 iters) | ✅ 완료 | **plant 수정(킥·로터·inertia) + 비전 거리감쇠 → deterministic 200-ep eval = 100.0% (202/202)**, d_xy_min 0.665m. A2 R_alt=0.0000(창발-기각), A0′ 대조로 귀속 분리(지배=plant 일관성). → [[experiments/exp_014_A2_visionrange]] / [[research/isaac_inertia_ctrl_mismatch]] |
| 015 | isaac_phased_curriculum (`feat/isaac-env-migration`) | 코드만 (미학습) | ✅ 코드 완료 | **Phase별 순차 커리큘럼 구현.** 이미지 3단계(접근/nominal → CCIP+Residual/정지 → 이동타겟) 완전 구현: action 4→6(δ residual), phase 단일 노브+파생 플래그, 릴리스 이벤트(nominal CCIP+δ 트리거 → 실제 DR 낙하 착탄오차 터미널 보상), drag/wind DR, Gauss-Markov 이동타겟, lead 보상, `train.py --phases 1,2,3` 서브프로세스 오케스트레이터(6-dim 고정 → warm-start 무손실). `py_compile` 12파일 통과, pytest는 L4/컨테이너 대기. → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20 |
| 015-train | exp015 실학습 (baseline, Isaac PPO, tensorboard) | 2048 envs × 600/500/500 iters (~104M steps, ~65 min) | ✅ 완주 (ORCH_EXIT=0) | **첫 커리큘럼 end-to-end. Phase 1만 완전 수렴.** P1 success 0.48→1.00·reward→107(exp_014 재현); P2 reward −0.8→94.7·drop 4.66→2.91m↓·release peak 0.98/tail 0.33·success~0; P3 reward→102·lead best 0.071m(tail 평탄)·success~0. warm-start 무손실 실증(Rule 20e). reward 우상향=proximity 지배 — P2/P3 명중 능력 미형성(추가 학습 또는 exp_018 종단구조 필요). ckpt/로그/그래프 host 영속화. → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]] / Rule 20 |
| 015-cont | exp015 이어학습(2차) P2/P3 ext (+2000 iters each) | P2: 1098→3097 · P3: 3097→5096, 2048 envs, ~3h | ✅ 완주 (P2_EXIT=0, P3_EXIT=0) | **iter 예산 확대 검증 — 0.8m 돌파 ❌.** P2 ext: drop 2.87m(정체), release 0.33→0.01 급락, success 0. P3 ext: drop 5.31m(회귀), reward 74.5, success 0. 구조 개입(exp_018 release_terminal) 필요(Rule 20f). → [[experiments/exp_015_phased_curriculum]] §8 / [[research/curriculum_phase_convergence]] §2(e) |
| 016 | exp016_ccip_release_reeval (eval-only, A2 ckpt) | 200-ep deterministic eval | ✅ 완료 | **4.59 m 디커플링 규명 = 지표 의미론 버그(Rule 21).** 릴리스 트리거 부재 → 지표가 성공-종단 잔여속도(3.0 m/s)의 탄도 캐리 측정. CCIP referee(≤0.2 m) 수정 후: 발화 시 0.137 m, release_rate 6%/11.5%(10/100 Hz), aim_err_min med 0.755 m ≈ d_xy_min(cross-track 지배). 구 지표 4.649 m 재현 ✓. 보상/종단 bit-identical(지표 전용). → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] |
| 017 | exp017 Stage A (750gpldr/6z0gpnhy/fv5qqmtz, Isaac PPO) | 3 runs × 26–39M steps | ✅ 완료 — 판정 (b) | **밀집 CCIP 조준 보상(보상-변경-단독)은 release_rate 못 올림.** P1 6-dim 기준선 신규 학습(det 2.5%, 학습 내 12→3.7% 단조 하락 — 근접 최적화가 릴리스 능력 파괴). v1(w=1): det 5.5%·aim 0.89 m·speed 2.72(방향 실재, p≈0.13). v2(w=2/knee 1.0): 회귀. 원인 3종=γ-할인 완주 보너스·CCIP 노이즈 증폭·성공 조기 종단 → 릴리스=Phase 2 종단 구조 몫. ckpt 3종 분리 보존. → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] / Rule 22 |
| 018 | exp018 Stage B (xt0hrr1c/0ns10yso/4vaodj0o/kk06wsbx, Isaac PPO) | 4 runs × 400 iters (v1 warm-start) | ✅ 완료 | **릴리스-종단 구조 → det release_rate 100.00%, drop err 0.125 m.** 종단 교체 단독으로 5.5%→100%(학습 내 23→99.6% 단조 상승 — Rule 22a 인과 확정). aim 보상 노브 불감(w 0/1.0/1.5 전부 100%; knee 0.75만 98.5%) — 자동 발화 referee가 노이즈를 +100 샘플러로 전환. done-flag alias 버그 사전 수정(Rule 23d). 호버-드롭 수렴. Stage C warm-start=B0. → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] / Rule 23 |
| 020 | exp020 물리 페이로드 부착 학습 (o5jn9xzk/vryuc6mu, Isaac PPO) | 400 iters (B0 warm-start, 보상 bit-match) | ✅ 완료 | **물리 페이로드의 학습 비용 = 0 — det success/release 100.00%, drop err 0.169 m.** release_rate 첫 롤아웃부터 100%(재학습 과도기 없음), exp_019 parity의 학습-스케일 확증. σ 드리프트 1.41→1.71 모니터 대상. **wandb eval-figure 파이프라인 신설**(`play.py --wandb`). 컨테이너 빈 WANDB_API_KEY 함정 → `--env-file` 필수. → [[experiments/exp_020_o5jn9xzk_payload_training]] / [[errors/err_20260723_wandb_key_empty]] |
| 021 | exp021 v19 + 이동 타겟 CV/CT/CA (a6saa42b/29jqq1lu/ntumqwoz, Isaac PPO) | 3 runs × 1000 iters (준상 v19 precise 사본 warm-start) | ✅ 완료 (det eval 포함) | **이동 타겟 모션을 v19에 obs-보존 포팅 → warm-start 학습 3종.** `V11Env._step_moving_target()` + V11/V16/V19 wire, obs 28-D 불변(lossless 로드 실증). det 200-ep: cv success 44.5%/drop med 0.775 m · ct 33.8%/0.783 · ca 16.5%/1.063 — 리드 부재가 1차 병목(개선안 §3c). `--target_kf`+v-track은 명시적 에러(KF는 base env 전용). → [[experiments/exp_021_v19_moving_target]] / [[research/moving_target_models]] §5 |

## 에러 현황

| 파일 | 상태 | 요약 |
|------|------|------|
| [[errors/err_20260723_wandb_key_empty]] | ✅ 해결 | 컨테이너 baked-in `WANDB_API_KEY` 빈 값 → wandb 학습 silent 실패(isaaclab.sh exit 0 삼킴). `--env-file /opt/drone-bombard/.wandb.env` 필수 |
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
- [[research/paper_research_plan]] — **(08-02) 논문 연구 계획 — 문헌 지도·차별점 3종·표 7종(베이스라인/잔차 위치/구조 vs 보상/충실도/인지/리드)·실행 순서 P0~P4**
- [[research/research_overview_for_paper]] — **(08-02) 전 연구 통합 개요 — 계보·warm-start 체인·발견 F1~F12·ablation 설계(논문용)**
- [[research/vision_obs_refactor]] — Vision 기반 obs 리팩토링 (GPS 제거, YOLO 전환)
- [[research/phase1_plan]] — Phase 1 CCIP 기반 자율 접근 연구 계획 (8주, 5/8-6/30)
- [[research/reward_design]] — 4-layer 보상 함수 (LaTeX 수식)
- [[research/architecture]] — Method A (1-World-4-Payload) 아키텍처
- [[research/system_overview]] — 전체 시스템 (패키지, 토픽, 좌표계, 브리지) — `jekyun`(Gazebo/PX4/ROS2) 브랜치
- [[research/isaac_lab_architecture]] — Isaac Lab 전체 구조 (`isaac_lab/` 레이아웃, 데이터 흐름, Gazebo 대비 구조 차이) — `feat/isaac-env-migration` 브랜치
- [[research/rl_rules]] — RL 실험 규칙, WandB 메트릭, Known Failure Modes
- [[research/physical_payload_attach]] — **(07-21) 물리 페이로드 kinematic weld 패턴** — per-env 조인트 불가 제약, 분리=write 중단, 물리↔해석 parity 계측, 트레이드오프(팬텀 질량·DR 갈라짐). Rule 24.
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
- [[research/exp014_ablation_protocol]] — exp_014 ablation 설계(07-04): 킥 vs attractor 원인 분리 arms/임계값/무학습 probes. 07-05 실행 완료 — probe 수렴으로 A1 생략, A2+A0′ 실행.
- [[research/isaac_inertia_ctrl_mismatch]] — **(07-05) set_inertias는 solver에 전파된다(계측 ×1031) — exp_013은 rate loop ~1300× 저토크 plant에서 학습.** 구 정책 plant-overfit 실증(bad_att 68%), 런타임 물리 오버라이드 금지 + 토크 응답 게이트 (Rule 19).
- [[research/phased_curriculum]] — **(07-05) Phase별 순차 커리큘럼 설계·수식.** 6-dim 고정 action으로 warm-start 무손실, 릴리스/model-mismatch residual 보정, Gauss-Markov 이동타겟·lead, 서브프로세스 오케스트레이션 (Rule 20).
- [[research/ccip_release_decoupling]] — **(07-05) success 100% vs drop_impact_error 4.59 m의 실체 = 지표 의미론 버그.** 릴리스 트리거 부재 → 성공-종단 잔여속도 탄도 캐리(3.0 m/s×1.53 s). CCIP referee 수정 + aim_err_min 진단, 근접 성공 ≠ 릴리스 능력 (Rule 21).
- [[research/ccip_aim_reward_stageA]] — **(07-06) Stage A: 밀집 CCIP 조준 보상 실패 분석.** dense 사이드 보상은 γ-할인 완주 보너스·CCIP 노이즈 증폭(×1.5 s)·성공 조기 종단을 못 이김 — 릴리스는 종단 구조(Phase 2)로. 5-lens 사전 적대 검증·farm 경제 계산 포함 (Rule 22).
- [[research/release_terminal_stageB]] — **(07-06) Stage B: 릴리스-종단 구조로 5.5%→100%.** 종단 교체 단독이 지배 요인(Rule 22a 인과 확정); 자동 발화 referee가 탐험 노이즈를 발견 메커니즘으로 전환; aim shaping은 종단 구조에서 잉여; done-flag alias 함정 (Rule 23).
- [[research/curriculum_phase_convergence]] — **(07-12 baseline §7 + 07-13 이어학습 §8)** warm-start 무손실 실증, reward 우상향 ≠ 임무 능력, P2/P3 릴리스 명중은 500 iter로 미형성 — **+2000 iter 연장도 0.8m 미돌파(P2 정체, P3 회귀)**. 해법=exp_018 종단구조 (Rule 20e/f).

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
- [[experiments/exp_011_wobble_lpf_reward_damping]] — RL wobble = smoothness-control 문제 확정. LPF A/B(jerk RMS −45%) + 보상 댐핑(B+C) → v15 fresh 기동. **Postmortem(07-05): v15 310K에서 X마커 미도달 회귀 의심(미확정) — 댐핑 반경 재타격 또는 crash-resume buffer 초기화, Rule 25/26.**
- [[experiments/exp_012_isaac_migration_phase2]] — Isaac Lab env+PPO 이식. v13/v15 상수 parity 이식, `pytest test_math.py` 29/29 통과, L4 VM 미기동.
- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] — Isaac 첫 완주 PPO(65.5M steps) + eval 36%. 비전 사멸 버그 수정, 상승-farming attractor·보상 불균형·noise_std 폭주 규명 → Rule 17·18.
- [[experiments/exp_014_A2_visionrange]] — **(07-05) plant 수정(킥·로터·inertia) + 비전 거리감쇠 → eval 100.0%(202/202).** A2 R_alt=0.0000(창발-기각 시그니처), A0′ 대조로 귀속 분리(지배=plant 일관성, 감쇠=꼬리 제거+parity). noise_std 안정 → Rule 18b 재해석.
- [[experiments/exp_015_phased_curriculum]] — **(07-05 구현 + 07-12 §7 + 07-13 §8)** action 4→6 residual, 릴리스+DR, 이동타겟+lead. **§7: baseline 완주 — P1 success 1.00, P2 drop 2.91m, P3 lead best 0.071m. §8: P2/P3 각 +2000 iters — 0.8m 미돌파, P3 회귀** (Rule 20e/f).
- [[experiments/exp_016_ccip_release_reeval]] — **(07-05) 4.59 m 디커플링 규명·수정 + A2 200-ep 재평가.** 릴리스 트리거 부재(지표 의미론 버그) → CCIP referee 수정: 발화 시 0.137 m, release_rate 6%/11.5%, aim_err_min ≈ d_xy_min(cross-track 지배). Rule 21.
- [[experiments/exp_017_stageA_aim_reward]] — **(07-06) Stage A: 밀집 CCIP 조준 보상 3-run(기준선/w=1/w=2).** det release_rate 2.5→5.5→3.5% — 판정 (b) 정체. P1 6-dim 기준선·warm-start 체인(399→1399)·det eval 전표·ckpt 아티팩트 경로. Rule 22.
- [[experiments/exp_018_release_terminal]] — **(07-06) Stage B: 릴리스-종단 4-run(B0 종단만/B1 w↑/B2 knee↑/B3 aim 제거).** det 100/100/98.5/100%, drop err ~0.13 m, 학습 내 단조 상승. 호버-드롭 수렴, Stage C warm-start=B0. Rule 23.
- [[experiments/exp_019_physical_payload]] — **(07-21) 물리 페이로드 attach/detach — kinematic weld 구현·검증 4/4 PASS.** 결함 6종 발견·3종 수정, 측정 착탄 vs 해석적 CCIP |Δ| ≤ 0.021 m, 보상/종단 bit-identical. Rule 24.
- [[experiments/exp_020_o5jn9xzk_payload_training]] — **(07-23) 물리 페이로드 부착 첫 학습 — 학습 비용 0 확증.** B0 warm-start + 보상 bit-match, det 200-ep 100.00%/drop 0.169 m. σ 드리프트 1.41→1.71 관찰. `play.py --wandb` eval-figure 파이프라인 신설.
- [[experiments/exp_021_v19_moving_target]] — **(07-30) 이동 타겟(CV/CT/CA) v19 obs-보존 포팅 + 준상 v19 warm-start 학습 3종 완주.** obs 28-D 불변 lossless 로드 실증, 난이도 cv<ct<ca(ca reward 음수 잔존), det eval 후속. wandb a6saa42b/29jqq1lu/ntumqwoz.

### 에러 (errors/)
- [[errors/err_20260703_vision_env_origin_frame]] — Isaac `_update_vision` world/env-local 프레임 혼용 → 벡터화 학습 비전 완전 사멸. "정확히 0.0000인 보상 성분 = 채널 사멸 신호" 규칙.
- [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] — armdiag dry-run이 YAML 중복 `checkpoint_dir` 키로 v13 30K 체크포인트 파괴. fresh-start 삭제 footgun + 격리 검증 규칙.
- [[errors/err_20260615_cruise-timeout-arming]] — CRUISE 타임아웃 = teleport 후 PX4 arm 거부 (stale EKF). arm 게이팅 + early-bail.
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 물리 폭발 3중 방어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 연구 일지 (daily/)
- [[daily/daily_2026-08-01]] — 브랜치 정리: 10개 브랜치 계보 조사, push 용량초과 원인 규명(실수로 커밋된 SAC/영상 바이너리), `Isaac-JS` 고유 노트 `isaac_jk`로 포팅 후 삭제, `main`을 `isaac_jk`로 승격(구 main은 태그로 보존)
- [[daily/daily_2026-07-30]] — exp_021: 이동 타겟(CV/CT/CA) v19 포팅 + 준상 v19 warm-start(사본) 학습 3종 완주, wandb 3 runs
- [[daily/daily_2026-07-23]] — exp_020 물리 페이로드 부착 학습 완주(100%, 학습 비용 0) + wandb eval-figure 파이프라인 + wandb 키 공백 함정 해결
- [[daily/daily_2026-07-13]] — exp_015 이어학습(2차) P2/P3 ext 완주: iter 예산 확대로 0.8m 미돌파, P3 회귀, VM 종료
- [[daily/daily_2026-07-07]] — **(Gazebo/SAC 트랙, isaac_jk 분기 전 기록)** 보상 함수 전체 리뷰(10단계 변천사) + 차기 설계 제안(v16/Stage C) — 기록 전용, 코드 변경 없음
- [[daily/daily_2026-07-06]] — exp_017 Stage A(보상 단독 정체, Rule 22) + exp_018 Stage B(릴리스-종단 → 100%, Rule 23): 같은 날 문제 확정과 해결. 5-lens/4-lens 적대 검증, alias 버그 사전 수정
- [[daily/daily_2026-07-05]] — plant 수정 실행 + inertia 대반전(Rule 19) + exp_014 A2/A0′: eval 36%→100%, R_alt 0, noise_std 안정. 캘리브레이션은 이미지 버그로 차단
- [[daily/daily_2026-07-05_gazebo_v15_regression]] — **(Gazebo/SAC 트랙, isaac_jk 분기 전 기록)** v15(310K) X마커 미도달 회귀 진단 + `drone-bombard-harmonic` 컨테이너 GPU 드라이버 불일치(535 vs 580) 발견 — 원인 미확정, Rule 25/26
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
