# RL Training Pipeline — Project Log

> **Branch:** `main` | **VM 프리엠션 후 컨텍스트 복구용 로그**

---

# 1. Current State

**업데이트:** 2026-07-05

> **⚠️ v15(`rl_yolo_v15_bc_stable`) — X마커 미도달 회귀 의심 (진단만, 미확정, 2026-07-05).** 반복 reset-recursion abort로 오토레쥼 서포바이저(`run_train_supervised.sh`) 재개 반복 후 0→310K, 2026-07-03 preempt(`sac_drop_310000_steps.zip`). 사용자 관찰: 훈련된 에이전트가 X마커에 도달 못 함(이전 v14 대비 회귀). **원인 후보 2건(미확정):** ① 근접-게이팅 속도 댐핑(`w_vel=0.08`, `vel_damp_radius=3.0m`)이 `success_radius=0.8m`보다 훨씬 넓어 v14의 기존 실패 구간(final-approach stagnation 0.5–1.2m)을 재타격 ② crash-resume마다 replay buffer 초기화(가중치만 복원)로 310K 스텝 수만큼의 연속 학습이 실제로는 없었을 가능성. **확정 전 필요:** `evaluate.py` 재실행(outcome breakdown) + `vel_logger.py` jerk 재측정. → [[daily/daily_2026-07-05]] / [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/rl_rules]] Rule 16
>
> **원 배경 (2026-07-01):** **RL wobble 교정** 적용: eval `deterministic=True`라 wobble=학습된 bang-bang 정책(탐험 노이즈 아님). LPF A/B로 **PX4 수신 속도명령 jerk RMS 2.92→1.61(−45%), 평균 속력 불변** → smoothness-control 문제 확정. 교정 = (B) 근접-게이팅 속도 댐핑 `w_vel=0.15/vel_damp_radius=4`(→ v13 base 이식 시 `0.08/3.0`으로 완화) + (C) `w_ang_vel 0.05→0.15`·`w_action_smooth 0.05→0.20` + 로직 LPF `velocity_lpf_alpha=0.4`(학습==배포). dry-run PASS(크래시 0). **Fresh Start가 v14 5체크포인트 삭제 → `rl_checkpoints/v14_backup/`에 백업.** → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15

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

- **2026-07-05:** **v15(310K) "X마커 미도달" 진단 (코드 변경 없음).** 사용자 관찰: wobble 교정판 모델이 이전 모델(v14)과 달리 X마커에 도달 못 함. 학습 이력 재구성 결과 v15는 잘못된 base config로 첫 크래시 후 수정, 이어서 **반복 reset-recursion abort**로 오토레쥼 서포바이저 추가, 0→310K에서 preempt — **그 이후 정식 eval이 한 번도 기록되지 않았음**을 확인. 원인 후보 2건(미확정): ① `vel_damp_radius=3.0m`가 `success_radius=0.8m`보다 넓어 근접-속도 댐핑(B)이 v14의 기존 final-approach stagnation 구간(0.5–1.2m)을 재타격했을 가능성 ② `run_train_supervised.sh`의 resume이 가중치만 복원하고 replay buffer는 매번 초기화 — 반복 abort가 있었다면 310K 스텝만큼의 연속 학습이 실제로는 없었을 가능성. 제안 변경안(우선순위순): evaluate.py 재실행으로 outcome breakdown 확인 → `vel_damp_radius` 축소(3.0→~1.0) 또는 `w_vel→0` → supervisor 재개 횟수 정량화 → (구조적) crash-resume 시 replay buffer도 보존. → [[daily/daily_2026-07-05]] / [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/rl_rules]] Rule 16
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

- [ ] **⚠️ v15 "X마커 미도달" 원인 확정** — `evaluate.py` 20+ep 재실행(outcome breakdown) + `vel_logger.py` jerk 재측정. stagnation 지배적이면 근접-속도 댐핑(w_vel/vel_damp_radius) 원인 확정. → [[daily/daily_2026-07-05]] / Rule 16
- [ ] **v15 회귀 수정안 적용** — `vel_damp_radius: 3.0→~1.0`(success_radius=0.8보다 살짝 크게) 또는 `w_vel→0`(fallback), stagnation 재발 여부 최우선 확인 후 fresh 300K 재학습 여부 결정.
- [ ] **supervisor 재개 횟수 정량화** — `/tmp/train_bc.log`의 `[SUPERVISOR] attempt N` 카운트 확인, wandb reward 곡선 불연속 패턴 대조.
- [ ] **(구조적, 낮은 우선순위) crash-resume 시 replay buffer 보존** — 현재 `run_train_supervised.sh` resume은 가중치만 복원. 반복 abort 재발 대비 SB3 체크포인트 콜백에 buffer 저장 추가 검토.
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
