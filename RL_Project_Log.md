# RL Training Pipeline — Project Log

> **Branch:** `main` | **VM 프리엠션 후 컨텍스트 복구용 로그**

---

# 1. Current State

**업데이트:** 2026-07-06

> **🔀 병행 트랙 (2026-07-06, Stage B): Isaac Lab exp_018 — 릴리스-종단 구조로 release_rate 5.5% → 100% (Rule 23)** —
> `feat/isaac-env-migration`. exp_017(판정 b) 직후 사용자 지시 실행: ①근접 종단(d_xy≤0.8) →
> **릴리스-발화 종단**(`DroneBombardEnvCfg.release_terminal`, 기본 False=레거시 bit-identical;
> 실패 게이트 7종·타임아웃 불변 — 적대 검증 byte-미접촉 확인) ②aim_err 보상 **nominal CCIP
> 전용** 명문화(트리거는 residual-포함 유지, 보상 양은 미포함 — reward-hacking 차단)
> ③Stage-A 보상 탐색 재실행(단일 노브·소폭). **결과(전 런 v1 warm-start, 400 iters, det
> 200-ep)**: **B0(xt0hrr1c, 보상=Stage A v1 그대로, 종단만 교체): 학습 내 release_rate
> 23→99.6% 단조 상승**(Stage A의 12→3.7% 단조 하락 정확히 반전 — 잘림이 지배 요인이었음을
> 인과 확정), **det 100.00%, drop err 0.125 m**(max 0.198), 호버-드롭 수렴(종단 속도 med
> 0.11 m/s). 스윕: B1(w=1.5) 100%, B3(w=0) 100% — **aim 항은 종단 구조에서 사실상 잉여**
> (자동 발화 referee가 탐험 노이즈를 +100 샘플러로 전환 — Stage A의 노이즈 증폭·γ-할인
> 문제 둘 다 역전), B2(knee 0.75)만 근소 열화(98.5%). farm 시그니처 0(ep_len 52→36 감소
> 수렴, stagnation/timeout/overshoot 전 구간 0). **4-lens 적대 검증이 done-flag alias 버그
> 사전 발견**(`success = _just_released` alias를 `_reset_idx`가 in-place clear → eval이
> success 0%로 보고할 뻔; `.clone()` 수정, 학습/wandb는 무영향) — Rule 23d. ckpt 4종:
> 컨테이너 `exp018_{B0,B1,B2,B3}_final.pt` + 호스트 `/opt/drone-bombard/checkpoints/exp018/`.
> **Stage C(DR drag/wind + residual, 별도 지시 대기) warm-start = `exp018_B0_final.pt`.**
> Phase-2 본선 이식 시 주의: release_tolerance 0.5(Phase 2 기본) vs 0.2(referee), 호버-드롭
> 프로파일의 바람 강건성은 미검(Phase 2 설계 의도 그 자체).
> → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] / Rule 23

> **(이전 2026-07-06, Stage A): Isaac Lab exp_017 — 밀집 CCIP 조준 보상, 판정 (b) 정체 (Rule 22)** —
> `feat/isaac-env-migration`. exp_016의 release_rate 6% 갭에 대한 **보상-변경-단독** 개입(사용자
> 제약: 종단/성공 게이트·entropy_coef·비전 캘리브레이션 불변, DR/residual은 Stage B로).
> **구현**: referee와 동일 aim_err의 dense 항 $w_{aim}(1-\tanh(e_{aim}/s))$ —
> `RewardCfg.w_aim`(기본 0.0=off, exp_014 parity)·`aim_reward_scale`, train.py `--w_aim` 주입,
> `math_utils.aim_error_reward`+테스트(40/40). **5-lens 사전 적대 검증**: parity(w=0
> bit-identical)/timing/plumbing(rsl_rl warm-start=N iters 추가 실행 실증)/physics(CCIP-hold
> v=d/T가 0.2 m 윈도 26–28 연속 스텝 유지 실현가능) 생존; pathology가 w=2 duty-cycle 펌프
> PV(~137–180 vs 완주 ~149, γ=0.995) 경고 → **w=1 헤지 개시** + farm 모니터.
> **결과(3 runs, warm-start 체인 399→1399, det 200-ep)**: ① P1 6-dim 기준선 신규
> 학습(750gpldr; exp_015는 스모크만·exp_014 ckpt는 4-dim): 학습 내 release_rate 12→3.7%
> **단조 하락**(근접 최적화가 릴리스 능력 능동 파괴), det 2.5%. ② v1(6z0gpnhy, w=1/knee
> 0.5 m): det **5.5%**, aim_err_min med 1.146→**0.889 m**, final_speed 3.35→**2.72 m/s** —
> 방향 실재(단 release_rate 차이는 n=200 p≈0.13), 학습 내 지표는 σ-지터에 가려 평탄(Rule
> 22c). ③ v2(fv5qqmtz, w=2/knee 1.0, 600 it): **회귀**(3.5%/1.096 m/3.45 m/s) — rew_aim
> 8×는 행동 불변의 수동 소득, σ 1.18→1.55. **원인 3종(구조적)**: γ-할인 +100이 모든 감속
> 벌함 · CCIP가 탐험 노이즈를 ×1.53 s 증폭(그래디언트 평탄화, entropy 불변 제약) · 성공
> 조기 종단이 조준 구간 제거. farm 시그니처 0. **결론: 릴리스는 dense 사이드 보상이 아니라
> Phase 2 릴리스-종단 구조로 학습(별도 지시 대기). w_aim은 Phase 2+ 그대로 이월 금지
> (residual 채굴 경로).** ckpt 3종 분리 보존: 컨테이너
> `/workspace/logs/isaac_lab/drone_bombard/exp017_{phase1,stageA,stageA2}_final.pt` + 호스트
> 백업 `/opt/drone-bombard/checkpoints/exp017/` — **차기 warm-start 소스 = stageA(v1) 권장**.
> → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] / Rule 22

> **(이전 2026-07-05 저녁): Isaac Lab exp_016 — "success 100% vs drop_impact_error 4.59 m" 디커플링 규명·수정 (Rule 21)** —
> `feat/isaac-env-migration`. **진단(적대 검증 5/5 확정)**: Phase 1엔 릴리스 트리거가 존재하지
> 않았고(`DropCfg.release_tolerance=0.2`는 정의만 되고 미사용 — v15 `drop_calculator_node`의
> CCIP ≤0.2 m 트리거 의도가 이식에서 소실), `drop_impact_error_m`은 **d_xy≤0.8 성공-종단
> 스냅샷**(잔여속도 포함)의 탄도 예측 = **속도 캐리** $v\cdot(\sqrt{2H/g}+0.1)$ = 3.0 m/s ×
> 1.53 s ≈ 4.6 m. `final_speed_xy` 계측(2.99 m/s)으로 산수 봉합. **수정**: 스크립티드 CCIP
> referee(매 policy step, |예측착탄−타겟|≤0.2 m ∧ alt>1 m 최초 충족 시 래치) — **지표 전용,
> 보상/종단/RNG 무접촉(학습 동역학 bit-identical)**. `drop_impact_error_m`=릴리스 시점 재정의,
> 구 지표 `drop_impact_error_terminal_m` 보존, `release_rate`/`aim_err_min_m`/`final_speed_xy`
> 신설. **A2 재평가(6407f8d 백포트, 200-ep deterministic)**: 구 지표 4.649 m 재현 ✓, **새 지표
> 발화 시 0.137 m(10 Hz)/0.172 m(100 Hz referee), 단 release_rate 6.0%/11.5%** — CCIP 스윕
> 최근접 med 0.755 m ≈ d_xy_min 0.665 m: **근접(d_xy) 보상으로 학습한 정책은 0.2 m 릴리스
> 윈도우를 거의 못 통과(cross-track 지배, 샘플링 아님)**. exp_013의 24 m도 동일 의미론(실패
> 지배 극단값) — 디커플링은 exp_012 지표 도입부터 구조적, plant 수정이 노출시킨 것. **진짜
> 투하 능력은 exp_015 Phase 2(릴리스 조건부 보상)가 학습해야 하며, 본 수정으로 Phase 1↔2
> `drop_impact_error_m` 의미론이 정렬됨.**
> → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] / Rule 21

> **(이전 2026-07-05): Isaac Lab exp_015 — Phase별 순차 커리큘럼 코드 완료 (미학습)** —
> `feat/isaac-env-migration`. 이미지의 3단계 커리큘럼(접근/nominal → CCIP+Residual/정지타겟 →
> 이동타겟)을 env + `train.py`로 **완전 구현**. ① **action 4→6** (`[0:4]` 속도 setpoint +
> `[4:6]` CCIP 착탄점 residual δx/δy) — 전 페이즈 6-dim 고정으로 `runner.load()` warm-start
> 무손실. ② **`phase`(1/2/3) 단일 노브** + 파생 플래그(residual/dr/moving_target/release)로
> 페이즈 동작 유도. ③ **릴리스 이벤트**: 온보드 nominal CCIP + 정책 residual 예측이 (lead)타겟
> tol 내면 투하 래치 → 실제 DR 물리(drag `U[0,0.15]`·wind `N(0,1.5)`) 낙하로 **진짜 착탄오차**
> 산출 → `w_impact·exp(-err/scale)` 터미널 보상. ④ **Gauss-Markov 이동타겟**(OU) + lead 보상
> (Phase 3). ⑤ **`train.py --phases 1,2,3`** 서브프로세스 오케스트레이터(각 페이즈 `model_final.pt`
> → 다음 `--resume` 체이닝; Isaac Sim 프로세스당 1 sim 제약). Phase 1은 미사용 dim zero-out으로
> **exp_014 접근 태스크와 동작 동일**(baseline 유효). **검증: 로컬 `py_compile` 12파일 통과;
> `pytest test_math.py`(+8 신규 = 38) 및 본 학습은 dev 박스 torch 부재로 L4/컨테이너 대기.**
> reward/DR/GM/lead 하이퍼파라미터는 초기 추정 → L4 dry-run 신호로 튜닝. 후속: 진짜 정책-학습
> lead 위해 obs에 타겟 속도 2채널(14→16) 추가 검토.
> → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20

> **(이전 2026-07-05): Isaac Lab exp_014 — plant 수정 + 비전 거리감쇠 → eval 100.0% (202/202)** —
> `feat/isaac-env-migration`. 확정 버그 2건 수정(속도킥→스폰타임 `UsdPhysics.MassAPI` authoring,
> 로터 ±200 rad/s 리셋 재주입 제거) + `--zero-actions` 게이트 PASS(11.9m→0.2m). 수정 검증 중
> **inertia 대반전 계측**: `set_inertias`는 solver에 전파되고 있었음 → **exp_013은 rate loop
> ~1300× 저토크 plant에서 학습된 것**(구 정책은 plant-overfit로 무효, **Rule 19** 신설,
> [[research/isaac_inertia_ctrl_mismatch]]). probe 3종(재평가/궤적판독/보상반사실) 수렴으로
> A1 생략 → A2(slant-range conf 감쇠) 400 iters: **R_alt=0.0000**(climb 창발 150-199 →
> 50 iter 내 기각), success 99.85%, **noise_std 0.80 안정(폭주 없음 — 18b 재해석: σ 폭주도
> plant 아티팩트)**. A0′(감쇠 OFF 대조): R_alt 0.0365, success 96.5% → 지배 요인=plant 일관성,
> 감쇠=꼬리 제거+YOLO parity(유지). **deterministic 200-ep eval = 100.0%, d_xy_min 0.665m,
> drop_impact_error 4.59m** (exp_013: 36%/1.4m/24m). 실 YOLO 캘리브레이션은 컨테이너
> annotator 버그로 차단(하네스 수리 완료, 커브는 분석값 calibration-pending).
> **reward_success·entropy_coef 불변(사용자 지시)** — 다음 페이즈에서 재평가(σ 안정이라
> entropy 0 근거 약화). 다음 후보: success_radius 0.8→0.5 커리큘럼, 임무 지표(drop error) 트랙.
> → [[experiments/exp_014_A2_visionrange]] / [[sessions/session_2026-07-05]]

> **(이전 2026-07-03) Isaac Lab exp_013 — 첫 프로덕션 PPO 학습 완주·진단 완료** —
> `feat/isaac-env-migration` 브랜치. 2048 envs×1000 iters(65.5M steps, 43분, wandb `wcjklw7a`)
> → **deterministic 200-ep eval = 36%**, d_xy_min 1.4m plateau(게이트 0.8 밖). 기동 직후
> **비전 사멸 버그**(`_update_vision` env-origin 프레임 혼용 → 벡터화 시 conf≡0; `yolo_eval.py`
> 동일) 발견·수정 후 재기동. 실패 3원인 규명: ①analytic conf 거리감쇠 누락→고도 상승
> farming(max_alt 33%, **Rule 17**) ②farmer(+225)>finisher(+121) 보상 불균형 — Gazebo v14
> final-approach stagnation과 동일 병인(**Rule 18a**) ③noise_std 0.8→3.92 폭주(**Rule 18b**).
> **사후 검증: --zero-actions FAIL(11.9m) — 리셋 속도킥 버그가 run 전체 오염(36%는 오염 plant 수치). 다음: exp_014 = 0순위 킥 수정 → conf 거리감쇠 + reward_success 300 + entropy_coef 0, fresh.** 온보딩 문서
> 3종(reward_tuning/wandb_guide/experiment_workflow) 신설. jekyun SAC 트랙 영향 없음.
> → [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]] /
> [[errors/err_20260703_vision_env_origin_frame]]
>
> (이전 상태 — Phase 2 코드 이식·parity·29/29 테스트: [[experiments/exp_012_isaac_migration_phase2]] /
> [[research/isaac_velocity_controller]] / Rule 16)

> **▶️ 활성 학습 (2026-07-01): `rl_yolo_v15_bc_stable`** — Fresh 0→300K 진행 중 (tmux `rl_train`, wandb online). **RL wobble 교정** 적용: eval `deterministic=True`라 wobble=학습된 bang-bang 정책(탐험 노이즈 아님). LPF A/B로 **PX4 수신 속도명령 jerk RMS 2.92→1.61(−45%), 평균 속력 불변** → smoothness-control 문제 확정. 교정 = (B) 근접-게이팅 속도 댐핑 `w_vel=0.15/vel_damp_radius=4` + (C) `w_ang_vel 0.05→0.15`·`w_action_smooth 0.05→0.20` + 로직 LPF `velocity_lpf_alpha=0.4`(학습==배포). dry-run PASS(크래시 0). **Fresh Start가 v14 5체크포인트 삭제 → `rl_checkpoints/v14_backup/`에 백업.** → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15

> **⚠️ 미커밋 변경 (2026-06-22~23, 검증 완료 — commit 결정 대기):**
> ① **핸드오프 윈도우 확장** — `target_altitude` 5→10 m + `vision_callback` 탐지 게이트(`min_detection_conf=0.5`, `detection_pixel_radius` 200→300) + `start_drift_max` 5→10. 핸드오프 2.7→5.0 m(~2배). → [[experiments/exp_008_dryrun_alt10_handoff_window]] / Rule 13
> ② **Soft reset (리셋 처리량 ~3.9×)** — `drone_drop_env.py`에 teleport 회피 리셋(`_try_soft_reset`) + `soft_reset_enabled: true`. **장기 검증 ✅: byxyaf4d 3096 resets, soft 성공 ~91%, EKF bounded, no teleport/no PX4 restart** → Rule 14 검증완료. → [[experiments/exp_009_softreset_throughput]] / [[experiments/exp_010_byxyaf4d_v14_195k_eval]] / Rule 14
> **⚠️ v14 정책 자체는 회귀(65%<80%):** byxyaf4d 195K eval = **65% (13/20)** vs v13 80%(16/20). 실패 전부 final-approach stagnation. 원인=미성숙(39% budget). **soft reset 인프라는 채택 가치 있으나 "v14를 baseline으로" 결정은 미정.** → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]

### 활성 학습

| 항목 | 값 |
|------|-----|
| Run name | `rl_yolo_v13_terminal_reward` (iyhfy5ps) — **2026-06-20 157.7K/500K에서 SIGTERM stop (eval 위해)** |
| 상태 | ⏸️ stop (plateau, ep_rew_mean ~100, success ~82%, target_lost 0). `sac_drop_preempt.zip`+70MB replay 보존 → 재개 가능 |
| 평가 결과 (확정 06-21) | **clean 20-ep eval: success 16/20 (80%), gate 0, EKF-drift 0, mean reward 134, mean closest 0.81m.** 4개 실패 전부 종단 stagnation(0.81–1.09m). 80%≈학습 ~82%. → [[experiments/exp_007_iyhfy5ps_v13_eval]] |
| 평가 fixes | health gate + YOLO 누수 fix + evaluate.py 재작성 (06-21, push fb69bb9). 발산 근본원인=YOLO 누수, fundamental EKF 버그 아님 |
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

- **2026-07-03 (Isaac Lab migration) — README 컨테이너 진입 절차 수정 + `play.py` 4-tuple 버그 fix.** 사용자가 혼자 재현 시도 시 실패 원인 규명: README의 `docker pull drone-bombard-isaac:latest`가 가리키는 이미지는 로컬에도 GCP Artifact Registry(`isaac-lab` 저장소, 0 items)에도 **존재하지 않음** — pull 대상이 없었음. 실제로는 `isaac-lab-local:580` 이미지로 띄운 `isaac-verify` 컨테이너가 이미 dev VM에 떠 있었음. README §5에 "이미 떠 있는 컨테이너 재사용" 절 신설 + non-root exec 시 root 소유 캐시(`/isaac-sim/kit/cache` 등)로 인한 `PermissionError` 크래시 및 chown 해결법 문서화 + `PYTHONUNBUFFERED=1` 팁(미설정 시 `simulation_app.close()`의 하드 종료로 마지막 PASS/FAIL print 유실) + README 전체의 `./isaaclab.sh`(존재하지 않는 상대경로, `/workspace/drone-bombard`에서 cwd 불일치) → `/workspace/isaaclab/isaaclab.sh` 절대경로로 일괄 수정(14곳). 검증 중 `isaac_lab/play.py`의 `run_zero_actions`/`run_scripted`/`run_policy`가 `RslRlVecEnvWrapper`(rsl_rl 4-tuple `obs,rew,dones,extras` API) 사용 중임에도 5-tuple(`obs,rew,terminated,truncated,info`) unpacking을 시도해 **항상 `ValueError`로 즉시 크래시**하던 버그 발견·수정. 수정 후 `--zero-actions`는 실행은 되나 altitude drift 12m/100 step로 FAIL(`verify_one_episode.py`는 동일 env로 148-step 안정 호버 PASS) — wrapper 경로 자체의 미해결 회귀 가능성, 후속 조사 필요. → [[isaac_container_access]] (Claude memory)
- **2026-07-03 (Isaac Lab migration) — 실제 실행 검증 통과 (VERIFY: PASS).** 사용자 요청으로 이 dev 박스에 `isaac-sim:5.1.0` pull → Isaac Lab v2.3.2+rsl_rl 설치 → `verify_one_episode.py`(신규 무학습 하네스)로 `Isaac-DroneBombard-Direct-v0` **1 에피소드 실제 실행**. env 구성·USD 씬(드론)·질량 오버라이드(2.173kg)·reset(obs (1,14))·**148스텝 안정 호버**(고도 유지, 중력보상)·obs/reward/termination 유한(NaN 0)·stagnation guard 정상 발동. **실행으로만 잡히는 env/컨트롤러 버그 5종 수정**(핵심: rate loop 토크에 관성항 누락 → ~46× 과토크 → 즉시 스핀아웃; `τ=I·(k_rate·rate_err)`로 수정). + 이미지 자체 버그 2종(dangling `_structures.py` 심링크, core isaaclab 미설치) Dockerfile 반영. **렌더링/GUI는 driver 535<580으로 이 박스에서 불가** — 물리/CUDA 정상, 시각화는 L4 VM 필요(사용자: headless 검증 수용). 커밋 `f2f1b1a`. → [[experiments/exp_012_isaac_migration_phase2]] §6b / [[research/isaac_velocity_controller]]
- **2026-07-03 (Isaac Lab migration, `feat/isaac-env-migration` 브랜치):** **Phase 2 — env+PPO 코드 이식 완료.** 별도 워크트리(`git worktree add /opt/drone-bombard/isaac-worktree feat/isaac-env-migration` + `git merge jekyun`, merge `940c88b`)에서 진행 — jekyun의 라이브 v15 학습(tmux `rl_train`) 방해 없음. `isaac_lab/` 신설: `math_utils.py`(action rate-limit/LPF, pinhole vision projection, hold-buffer, ballistic/CCIP, 3-layer reward, overshoot/stagnation guard — 순수 torch, isaaclab 무의존) + `drone_bombard_env.py`(DirectRLEnv, 위 math_utils를 isaaclab lifecycle에 연결) + `agents/rsl_rl_ppo_cfg.py` + `train.py`/`play.py`/`yolo_eval.py`. v13/v15 obs(14)·action(4)·reward·termination 상수 전부 parity 이식(표: [[experiments/exp_012_isaac_migration_phase2]]). SAC→PPO(rsl_rl), target/spawn 랜덤화 신규(Gazebo는 고정 타겟), vision=학습 시 analytic pinhole(YOLO 캘리브레이션 노이즈)+평가 시 실제 YOLOv8 이원화, drop=액션 아닌 스크립트 CCIP 메트릭(태스크 스코프 v15와 동일). Phase-2 훅 4종(CCIP residual, release 상수 cfg화, obs superset 고정, domain-rand 스텁) 비활성 wiring — Phase 1 출력 불변. **검증:** `pytest isaac_lab/tests/test_math.py` **29/29 통과**(drone-bombard-harmonic 컨테이너, torch 2.4.1, isaaclab 미설치 — 파일 경로 직접 로드로 패키지 `__init__` 우회). `py_compile` 전체 통과. L4 Spot VM 미기동 → env 스모크·실제 학습 미실행(README에 정확한 커맨드 문서화). 부수 발견: Gazebo `hyperparams_v13.yaml`의 `limit_tilt:0.26`는 코드에서 미사용인 죽은 설정(실제 게이트는 `limit_inverted_tilt=1.047` 기본값) — 이식 안 함. Overshoot guard가 success_radius=0.8에서 도달 불가능한 것은 Rule 10의 의도된 설계임을 Gazebo 소스로 재확인(버그 아님) — 비종단 진단 카운터만 신설. 신규 **Rule 16**(시뮬레이터 이식 시 plant/reward parity는 상수뿐 아니라 타이밍+메커니즘까지 검증). → [[experiments/exp_012_isaac_migration_phase2]] / [[research/isaac_velocity_controller]] / [[research/rl_rules]] Rule 16
- **2026-06-23:** **v14(byxyaf4d) plateau stop + 195K eval = 65% + soft reset 장기검증 통과.** ep_rew_mean이 70K부터 ~120–135 평탄(125K step 정체) 확인 → **196.5K/500K(~39%)에서 SIGINT graceful stop**(`sac_drop_195000_steps.zip` 보존, sim 점유 해제). **clean 20-ep deterministic eval = 성공 65%(13/20)** — v13 80%(16/20) 대비 **회귀**. 실패 7 전부 **final-approach stagnation**(~0.5–0.8m 접근 후 0.50m gate 직전 정체; min 0.52/0.63/0.70/0.79/0.79/1.09/1.19, ep18은 2cm차). 항법·탐지 아닌 **종말 거리 좁히기 약점.** EKF 귀책 실패 0(eval중 health-gate EKF divergence 2회 전부 full-restart self-heal 후 성공). **Soft reset 장기검증 ✅:** stop 직전 env `attempts=3096 success=2826 skipped=118`(soft ~91%, no teleport/no PX4 restart) → exp_009의 미해결 질문(학습 정책 fallback율·EKF drift bounded) **해소, Rule 14 검증완료.** **회귀 원인=정책 미성숙**(39% budget, terminal-tightening은 막판 sharpen 스킬; reward plateau는 거친 정책의 평탄). 추가 가설: 10m 고도(v13 5m)로 최종 하강↑. **비디오:** `record_flight.py`+3-ep evaluate 동시 → 3/3 success(0.48–0.50m) 캡처 → `rl_eval_results/v14_195k_flight_annotated.mp4`(2.1MB, YOLO 박스) + `_raw.mp4`(3.7MB). headless `gz sim -s`라 onboard annotated가 산출물. **v14 commit 결정 대기**(soft reset 인프라 채택 vs 65%<80% baseline 채택). → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] / Rule 14
- **2026-06-22 (오후):** **리셋 처리량 ~3.9× — soft reset로 teleport-EKF 병목 회피.** v14 fps≈2, ETA ~2.5일의 원인을 계측으로 확정: 에피소드마다 CRUISE timeout(~42s)+full restart(~22s)이고, 근본은 teleport+disarm 후 PX4 **EKF 추정기 재수렴** 대기(`ctrl_0.log: Delaying arm — pre_flight_checks_pass=False`). **fresh restart도 동일 timeout**(restart≈handoff). **EKF2_GPS_CHECK 0 A/B = 음성**(airframe가 이미 `COM_ARM_WO_GPS 1` → GPS는 게이트 아님; 실제 게이트=EKF validity/innovation 수렴, 바이패스 param 설계상 없음) → param/timeout 레버 고갈 규명. **해결 = soft reset**(`drone_drop_env.py _try_soft_reset`): 종료 시 flyable이면 disarm/teleport 없이 armed+airborne 유지하고 position setpoint로 출발점 복귀(controller 살려둬 offboard heartbeat 유지) → mission_manager FSM만 재시작 → 재핸드오프. EKF 미교란 → 재수렴 0. **프로토(9.2min/32회): throughput 0.93→3.61 handoffs/min(~3.9×), fps 2→9, reset 65s→11s, soft 성공 32/32(100%), EKF d_xy 안정 4.5–5.8m(발산 없음).** flyable 아니면 기존 teleport+restart fallback(downside bounded). full run `rl_yolo_v14_softreset`(byxyaf4d, fresh 500K, online) 기동 — 장기 EKF drift/fallback율 검증 중. **미커밋.** → [[experiments/exp_009_softreset_throughput]] / [[research/reset_throughput_bottleneck]] / Rule 14
- **2026-06-22:** **핸드오프 윈도우 확장 — 고도는 레버 아님, 탐지 게이트가 진짜 레버.** 사용자 요청("X마커가 늦게=거의 머리 위 탐지돼 RL 핸드오프 후 학습 윈도우가 짧다"). **시도 1(고도만 10 m):** `target_altitude` 5→10 + `start_drift_max` 5→10 정합. **실패** — clean 핸드오프 여전히 d_xy 2.7 m(베이스라인 동급), 3 ep 중 2가 순항-시작 spurious(conf=0.00, d_xy≈11 m, 넓어진 FoV의 X-like 지면 FP) → health gate 발동/abort. 원인: 마커 apparent size ∝ 1/고도(YOLO 늦게 lock) + `vision_callback` 200 px 필터가 핸드오프를 머리 위로 클립. **시도 2(10 m + 탐지 게이트):** `vision_callback`에 confidence 게이트(`min_detection_conf=0.5`; real 마커 conf 0.73–0.95 vs 지면 FP ≤0.45) + 공간 필터 200→300 px(`detection_pixel_radius`; real off-center 264–293 px 조기 accept). **dry-run PASS: 핸드오프 d_xy 2.7→5.0–5.2 m(윈도우 ~2배), spurious 0, EKF-drift 0, conf 0.93.** dry-run 격리(`--checkpoint-dir rl_dryrun_alt10` + offline)로 메인 체크포인트 보호. **코드 변경 미커밋**(사용자 직접 커밋 예정). 기하+탐지 변경이라 fresh 필수 아니나 핸드오프 3.5→5 m 변화로 정책 초반 재적응 예상. → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13
- **2026-06-21:** **eval 발산 흡수 루프 근본 원인 규명 & 수정 3종 (health gate + YOLO 누수 + evaluate.py).** 06-20 발견한 EKF↔camera 발산 루프의 진짜 원인 = **YOLO `xmarker_detector` 누수**: `_start_infra`가 fresh-start마다 YOLO 노드를 죽이지 않고 새로 spawn → 누적(검증 시 3개) → 충돌하는 pixel_coords 발행 → spurious CRUISE→TRACKING(conf=0.00) + EKF↔camera 불일치. **fundamental EKF 버그 아님** (clean slate에선 handoff 0.9m 정상). **수정:** ① `drone_drop_env.py` reset() step 8b **health gate**(`d_xy_prev>start_drift_max(5.0)`면 full restart+progressive settle 후 retry, max_retries(6) 초과 시 loop 대신 abort), ② fresh-start kill 리스트에 `xmarker_detector` 추가(누수 차단), ③ `evaluate.py` 재작성(success_rate/step-to-reach/closest d_xy from obs[12,13]/outcome breakdown; 죽은 `info['drop_error_actual_m']` NaN 의존 제거). config `hyperparams_v13.yaml`에 `start_drift_*` 추가. **Dry-run PASS(clean slate, 3/3 SUCCESS, gate 0회, mean reward 162, report NaN 없음).** `--symlink-install`이라 src 편집 live, rebuild 불필요. → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12
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

- [x] **(Isaac Lab, exp_015)** Phase별 순차 커리큘럼 **코드 구현 완료** — action 4→6 residual, phase 노브+파생 플래그, 릴리스 이벤트+DR 착탄오차 터미널 보상, Gauss-Markov 이동타겟+lead, `train.py --phases` 오케스트레이터. `py_compile` 12파일 통과. → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20
- [ ] **(Isaac Lab, exp_015)** L4/컨테이너에서 `pytest test_math.py`(+8 신규=38) 실행 → 페이즈별 2-iter 스모크(phase 1/2/3) → `--phases 1,2,3` dry-run(256 envs, 5,5,5) → 본 학습(2048 envs, 3000/2000/2000). PhaseCfg 하이퍼파라미터 dry-run 신호로 튜닝. → [[experiments/exp_015_phased_curriculum]] §5
- [ ] **(Isaac Lab migration, 병행 트랙)** L4 Spot VM 기동(`infra/deploy.sh` 빌드+push, `infra/startup.sh` 실행) → Cartpole 스모크 → `Isaac-DroneBombard-Direct-v0` env 스모크(2-iter) → `play.py --zero-actions/--scripted` 물리 검증. → [[experiments/exp_012_isaac_migration_phase2]]
- [ ] **(Isaac Lab migration)** PX4 속도-스텝응답 Gazebo 캡처 세션(`vel_logger_v2.py` 신규, 7-포인트) → Isaac 컨트롤러 게인 검정. 현재 미검정(구조 일치, 게인 초기값). → [[research/isaac_velocity_controller]]
- [ ] **(Isaac Lab migration)** `yolo_eval.py --calibrate` 첫 실행 → vision 캘리브레이션 v1(현재 v0=스펙 추정).
- [ ] **(Isaac Lab migration, 게이트 조건부)** `feat/isaac-env-migration`(이 워크트리)에서 `ros2_ws/`/`gazebo_models`/PX4 파일 정리 — **jekyun(라이브 SAC)는 대상 아님, 절대 미삭제.** 위 2개 항목(env 스모크 통과 + PX4 스텝응답 캡처) 완료 전까지 보류(사용자 확인, 2026-07-03). → [[experiments/exp_012_isaac_migration_phase2]] §8
- [x] **Vision 기반 RL 인프라 완성** — YOLO + SAC 시각 서보잉 파이프라인 구축
- [x] **EKF East 반전 버그 수정** — proximity target + RL env reward target 좌표 수정
- [x] **WandB `45l8vkw5` 100+ 에피소드 분석** → target_lost=1.0, EKF drift 확인 → 폐기
- [x] **EKF drift 방어 로직** — step 1에서 d_xy>5m이면 즉시 truncate (ekf_drift)
- [x] **fps 개선 — CRUISE 타임아웃 근본 원인 수정** — arm 게이팅(#2) + early-bail(#4). v12에서 효과 검증 중 → [[research/cruise_timeout_arming]]
- [x] **v12/v13 arm 처리량 재진단** — `ARM REJECTED` 0회(게이팅 작동), 실제 병목은 EKF 재수렴이 10s bail 초과(13–16s). **arm_bail 10→20s 적용** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11
- [x] **v13 재시작** — 30K 체크포인트 인시던트로 소실 → **fresh 재시작**(iyhfy5ps, arm_bail=20). 검증: bail 0, late-EKF 회복 확인.
- [x] **핸드오프 윈도우 확장** — 고도↑(레버 아님) 기각, **탐지 게이트 수정**(conf 0.5 + 공간필터 200→300 px)으로 핸드오프 2.7→5.0 m(~2배), spurious 0. dry-run PASS, **미커밋** → [[experiments/exp_008_dryrun_alt10_handoff_window]] / Rule 13
- [x] **리셋 처리량 병목 규명 + 수정** — 원인=teleport 후 EKF 재수렴(param 불가). **soft reset로 ~3.9×** 프로토 검증 → [[experiments/exp_009_softreset_throughput]] / Rule 14
- [x] **soft reset 장기 검증** — byxyaf4d 3096 resets에서 soft 성공 ~91%, EKF bounded, no teleport/no PX4 restart. **Rule 14 검증완료.** → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- [ ] **⚠️ v14 정책 회귀(65%<80%) 해소** — 195K eval 실패 전부 final-approach stagnation(미성숙 가설). 옵션: ① soft reset 켠 채 500K까지 재개/연장(미성숙 검증) ② 10m→5m 고도 A/B(종말 거리 가설) ③ 종말 보상 shaping 강화. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- [ ] **⚠️ v14 commit 결정 (사용자)** — soft reset/탐지게이트 인프라는 검증완료지만 정책 65%<80%. "v14를 validated baseline으로" 채택 여부 결정 필요. 윈도우 확장 + 탐지게이트 + soft reset 코드 일괄 미커밋 상태.
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
| 2026-06-22 | dryrun_alt10 (uqy7lmny/_gated, offline) | 1500×2 (dry-run) | **핸드오프 윈도우↑.** 고도만 10 m=실패(레버 아님; 마커 작아짐 + 200 px 필터 캡). 10 m+탐지 게이트(conf 0.5 + 200→300 px)=성공: 핸드오프 2.7→5.0 m(~2배), spurious 0. **미커밋.** → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13 |
| 2026-06-22 | EKF A/B + softreset proto (offline) | 1500+5000 | **리셋 처리량 ~3.9×.** 원인=teleport 후 EKF 재수렴(`pre_flight_checks_pass=False`). EKF2_GPS_CHECK 0 A/B=음성(COM_ARM_WO_GPS). **soft reset(teleport 회피)=성공**: 0.93→3.61 handoffs/min, fps 2→9, reset 65s→11s, soft 100%, EKF 안정. → [[experiments/exp_009_softreset_throughput]] / Rule 14 |
| 2026-06-22 | rl_yolo_v14_softreset (byxyaf4d) | 진행 중 (fresh 0→500K) | **full run — soft reset ON 장기 검증.** 10m + 탐지게이트 + soft reset 일괄, online. EKF drift bounded?/fallback율?/실제 ETA(~15h 예상) 확인 후 커밋. → [[experiments/exp_009_softreset_throughput]] |
| 2026-06-23 | rl_yolo_v14_softreset (byxyaf4d, stop @196.5K) | 196.5K/500K(~39%), reward plateau | **plateau stop + 195K eval = 65%(13/20).** v13 80% 대비 회귀(실패 전부 final-approach stagnation). EKF 귀책 0. **Soft reset 장기검증 ✅**(3096 resets, soft ~91%, EKF bounded) → Rule 14 검증완료. 비디오 3/3 success 캡처. 회귀=미성숙(39% budget). commit 결정 대기. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] |
| 2026-07-01 | rl_yolo_v15_bc_stable | 진행 중 (fresh 0→300K) | **wobble 교정(LPF+B+C) 적용 fresh run.** jerk RMS 2.92→1.61(−45%) A/B 확정 후 기동. → [[experiments/exp_011_wobble_lpf_reward_damping]] / Rule 15 |
| 2026-07-03 | isaac_migration_phase2 (`feat/isaac-env-migration`, 병행 트랙) | 0 (코드만) | **Isaac Lab env+PPO 이식.** v13/v15 parity, `pytest test_math.py` 29/29 통과, L4 VM 미기동. jekyun SAC 학습과 별개 브랜치/워크트리. → [[experiments/exp_012_isaac_migration_phase2]] |
| 2026-07-03 | exp013_v1_baseline (Isaac PPO, 병행 트랙, 중단 @iter 106) | ~7M steps | **비전 사멸 버그 발견·중단.** `rew_vision`≡0.0000 → `_update_vision` env-origin 프레임 혼용(2048-env grid에서 타겟 항상 프레임 밖). 수정+수치검증(visible 0%→63%). → [[errors/err_20260703_vision_env_origin_frame]] |
| 2026-07-03 | **exp013_v2_visionfix (wcjklw7a, Isaac PPO, 병행 트랙)** | 65.5M steps (1000 iters 완주) | **첫 완주 + deterministic 200-ep eval = 36%.** plateau @iter 700, d_xy_min 1.4m 정체. 실패: max_alt 33%(상승 farming, Rule 17)+crash 27%. farmer(+225)>finisher(+121) 불균형(Rule 18a), noise_std 0.8→3.92 폭주(Rule 18b). **사후 --zero-actions FAIL(11.9m): 리셋 속도킥 활성 — run 오염, max_alt 1차 용의자.** 다음=exp_014(0순위 킥 수정 → conf 거리감쇠+success 300+entropy 0, fresh). → [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]] |
| 2026-07-05 | **exp014 A2 (v3qk07pg) + A0′ (azoc1xp0), Isaac PPO, 병행 트랙** | 각 26.2M steps (400 iters) | **plant 수정 + 비전 거리감쇠 → deterministic 200-ep eval = 100.0% (202/202), d_xy_min 0.665m.** 수정: ①킥→스폰타임 MassAPI authoring(게이트 11.9m FAIL→0.2m PASS) ②로터 스핀 리셋 재주입 제거 ③**inertia 대반전**: `set_inertias`는 전파되고 있었음 — exp_013은 rate loop ~1300× 저토크 plant(Rule 19, 구 정책 plant-overfit로 무효). A2: R_alt=0.0000(climb 창발→기각 시그니처), noise_std 0.80 안정(폭주 없음). A0′ 대조: R_alt 0.0365 → 지배 요인=plant 일관성, 감쇠=꼬리 제거+YOLO parity(유지). 실 YOLO 캘리브레이션은 이미지 annotator 버그로 차단(하네스 수리 완료). reward_success·entropy 불변(다음 페이즈). → [[experiments/exp_014_A2_visionrange]] / [[research/isaac_inertia_ctrl_mismatch]] |
| 2026-07-05 | isaac_phased_curriculum (`feat/isaac-env-migration`, 병행 트랙) | 0 (코드만, 미학습) | **Phase별 순차 커리큘럼 구현.** 이미지 3단계(접근/nominal → CCIP+Residual/정지 → 이동타겟) 완전 구현: action 4→6(δ residual), phase 노브+파생 플래그, 릴리스 이벤트(nominal CCIP+δ 트리거 → 실제 DR 낙하 착탄오차 터미널 보상), drag/wind DR, Gauss-Markov 이동타겟+lead, `train.py --phases 1,2,3` 서브프로세스 오케스트레이터(6-dim 고정 → warm-start 무손실). Phase 1=exp_014 baseline 동작 동일. **`py_compile` 12파일 통과; `pytest`(+8 신규)·본 학습은 L4/컨테이너 대기.** → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20 |
| 2026-07-05 | **exp016 CCIP 릴리스 referee 재평가 (eval-only, A2 ckpt, 병행 트랙)** | 0 (200-ep deterministic eval) | **디커플링 규명: 4.59m = 지표 의미론 버그(릴리스 트리거 부재, 성공-종단 속도 캐리).** CCIP referee(≤0.2m) 수정 후: 발화 시 0.137m, release_rate 6%/11.5%(10/100Hz), aim_err_min med 0.755m ≈ d_xy_min(cross-track 지배). 구 지표 4.649m 재현. 보상/종단 bit-identical. Rule 21. → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] |
| 2026-07-06 | **exp017 Stage A — 밀집 CCIP 조준 보상 (750gpldr/6z0gpnhy/fv5qqmtz, Isaac PPO, 병행 트랙)** | 3 runs (P1 400 + v1 400 + v2 600 iters, 2048 envs) | **보상-변경-단독은 release_rate 못 올림 — 판정 (b), Rule 22.** det 200-ep: 기준선 2.5% → v1(w=1) 5.5%(aim 0.889m·speed 2.72, 방향 실재·p≈0.13) → v2(w=2/knee 1.0) 3.5% 회귀. P1 기준선 학습 내 12→3.7% 단조 하락(근접 최적화가 릴리스 능력 파괴). 원인=γ-할인 완주 보너스·CCIP 노이즈 증폭(×1.53s)·성공 조기 종단. 5-lens 사전 검증, farm 0. ckpt 3종 분리 보존(+호스트 백업) — 차기 warm-start=stageA(v1). → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] |
| 2026-07-06 | **exp018 Stage B — 릴리스-종단 이벤트 (xt0hrr1c/0ns10yso/4vaodj0o/kk06wsbx, Isaac PPO, 병행 트랙)** | 4 runs × 400 iters (전부 v1 warm-start) | **릴리스-종단 구조 → det release_rate 100.00%, drop err 0.125 m (Rule 23).** 종단 교체 단독(B0)으로 5.5%→100%(학습 내 23→99.6% 단조 상승 — Stage A 하락 반전, Rule 22a 인과 확정). aim 보상 노브 불감(w 0/1.5 100%, knee 0.75만 98.5%) — 자동 발화 referee가 노이즈를 +100 샘플러로 전환. done-flag alias 버그 사전 수정(eval success 0% 위험). 호버-드롭 수렴(종단 속도 0.11 m/s). **Stage C warm-start = exp018_B0_final.pt.** → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] |
