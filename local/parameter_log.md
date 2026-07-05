# Parameter Log — Drone-Bombard-Simulation

> **목적**: 모든 parameter 의 의미 사전 (§1) + 시간순 변경/추가/제거 history (§3 이후).
> "이 parameter 가 뭐 하는 거지?" → §1 / "왜 언제 바뀌었지?" → §3 이후.
>
> **추가 규칙**: parameter 가 변경/추가/제거되거나 새 run 시작 시 §3 표에 한 줄 + §4 entry 한 블록 추가.

작성 시작: 2026-05-22
관련:
- [meeting_notes/meeting_notes_2026-05-20.txt](meeting_notes/meeting_notes_2026-05-20.txt)
- [meeting_notes/meeting_notes_2026-05-22.txt](meeting_notes/meeting_notes_2026-05-22.txt)
- [meeting_notes/meeting_notes_2026-05-23.txt](meeting_notes/meeting_notes_2026-05-23.txt)
- [design_review_2026-05-23.md](design_review_2026-05-23.md)
- [archive/change_inventory_pre_pull_2026-05-22.md](archive/change_inventory_pre_pull_2026-05-22.md)
- [../Drone-Bombard-Simulation/RL_analysis.md](../Drone-Bombard-Simulation/RL_analysis.md)

---

## 1. Parameter 의미 사전 (Glossary)

`hyperparams.yaml` 의 각 key 의미 + 영향 + trade-off.

### 1-1. `training:` — 학습 메타

| Key | 의미 | 영향 / Trade-off |
|---|---|---|
| `total_timesteps` | 학습 총 env step 수 (`env.step()` 호출 총 횟수) | 클수록 학습 길어짐. SAC 는 `gradient_steps` 배수만큼 network update 함 |
| `checkpoint_freq` | N step 마다 model `.zip` 저장 (CheckpointCallback) | 작을수록 보존 잦지만 디스크 사용↑ |
| `max_checkpoints_kept` | rolling 으로 마지막 N 개만 보관, 옛것 삭제 | 디스크 보호 |
| `checkpoint_dir` | 체크포인트/best_model/archive 폴더 root | — |
| `num_envs` | 병렬 환경 수 (SubprocVecEnv) | >1 이면 Gazebo lockstep overhead. 현재 1 권장 |
| `env_stagger_secs` | SubprocVecEnv instance 초기화 간격 (s) | num_envs>1 시 CPU spike 방지 |
| `eval_freq` | BestModelCallback 의 평가 주기 (env step) | smoke/본학습 분리 가능. 작을수록 best 갱신 잦음 |
| `eval_episodes` | episodes per eval (reserved for future EvalCallback) | 현재 BestModelCallback 은 rollout 의 ep_rew_mean 사용 |

### 1-2. `sac:` — Stable-Baselines3 SAC 알고리즘

| Key | 의미 | 영향 / Trade-off |
|---|---|---|
| `learning_rate` | actor/critic Adam optimizer LR | 표준 3e-4. 크면 발산, 작으면 학습 느림 |
| `buffer_size` | replay buffer 최대 크기 (transitions) | 작으면 옛 경험 사라져 catastrophic forgetting. 너무 크면 RAM↑ |
| `batch_size` | gradient step 당 mini-batch 크기 | 표준 256. 크면 안정성↑ but throughput↓ |
| `tau` | target network soft update rate (`τ·new + (1−τ)·target`) | 0.005 표준. 작을수록 안정 but 학습 느림 |
| `gamma` | discount factor. effective horizon ≈ `1/(1−γ)` step | 0.99 → ~100 step, 0.995 → ~200 step, 0.999 → ~1000 step. 크면 long-term credit 강화 but Q variance↑ |
| `learning_starts` | 이 step 까지 random action (gradient update 안 함) | 초기 replay 다양성 확보. 너무 짧으면 critic 발산 |
| `gradient_steps` | env step 당 network update 횟수 (UTD ratio) | 1=환경 step 마다 1 update. 4=4 update. 클수록 sample reuse↑ but critic 폭주 위험 |
| `net_arch` | actor/critic hidden layer 크기 list | [256,256] 표준. 크면 표현력↑ but 데이터 더 필요 |
| `device` | "cuda" / "cpu" | GPU 가속 |

### 1-3. `environment:` — env / 좌표 / 정규화

| Key | 의미 | 영향 / Trade-off |
|---|---|---|
| `target_enu_x`, `target_enu_y` | 타겟 위치 (ENU 좌표, m) | 학습 / 평가의 목표점 |
| `pos_scale` | position observation 정규화 scale (m) — `obs = pos / 50` | obs ∈ [-1,1] 유지. 큰 비행 영역 학습 시 늘림 |
| `vel_scale` | velocity obs 정규화 (m/s) | |
| `ang_vel_scale` | angular velocity obs 정규화 | |
| `action_vx_scale` | action[0] (-1~1) → vx 명령 변환 (m/s) | 크면 빠른 maneuver but 불안정 |
| `action_vy_scale`, `action_vz_scale` | y, z 방향 동일 | |
| `action_yaw_scale` | action[3] → yaw rate (rad/s) | 1.0 = ~57°/s. 너무 작으면 회전 느림 |
| `max_steps` | episode 최대 step (truncation 기준) | 길수록 episode 길어짐. 500 = ~25s at 20Hz |
| `min_altitude` | crash penalty 임계 고도 (m) | 너무 높으면 takeoff 어려움 |
| `min_altitude_start_step` | 이 step 이후부터 min_altitude 적용 | takeoff 유예 (보통 20 step ≈ 1초) |
| `obs_wait_timeout` | PX4 message 대기 timeout (s) | 작을수록 빨리 step 진행. PX4_SIM_SPEED_FACTOR 와 곱해서 wall-clock 결정 |
| `cruise_poll_timeout` | CRUISE 도달 대기 timeout (s) | 너무 짧으면 false negative |
| `use_vision` | 카메라 obs 사용 여부 (true/false) | RL 학습은 false (camera 없는 world). vision 미션은 true |

### 1-4. `reward:` — 4-Layer 보상 구조

**Layer 1 (안전, 매 step)**
| Key | 의미 |
|---|---|
| `penalty_crash` | altitude < min_altitude (after start_step) 시 negative reward |
| `penalty_overspeed` | speed > 20 m/s 시 negative reward |
| `penalty_target_lost` | vision confidence == 0 시 (use_vision 일 때만) |

**Layer 2 (안정/효율, 매 step)**
| Key | 의미 |
|---|---|
| `w_time` | per-step time penalty (시간 흐름 자체에 벌점) |
| `w_ang_vel` | `‖ω‖²` 제곱 penalty weight (기체 불안정도) |
| `w_action_smooth` | `‖a_t − a_{t-1}‖²` penalty (급격 조작 억제) |

**Layer 3 (접근, 매 step)** — `R3 = w_dist·(d_prev−d_now) + w_heading·cos(θ)·gate + w_impact·exp(−k_impact·d_impact)`
| Key | 의미 |
|---|---|
| `w_dist` | 타겟까지 거리 감소량 보상 weight (potential-based, 안정) |
| `w_heading` | 진행 방향과 타겟 방향 정렬 cos 보상 |
| `speed_gate_enabled` | `gate(v_xy) = min(v_xy/2, 1)` — 정지 상태에서 heading/impact reward 수집 차단 (anti-milking) |
| `w_impact` | CCIP 예측 d_impact 의 exp 보상 weight. **non-potential-based** → speed_gate 없으면 loitering hack 위험 |
| `k_impact` | r3_impact decay constant. 작을수록 멀리서도 신호 살림 (k=0.05 면 80m 에서 ~0.087, k=0.1 면 ~0.0017) |
| `k1_potential` | **UNUSED** — 옛 exp(-k1·d) potential 잔재 |

**Layer 4 (terminal, drop 시점)** — `R4 = drop_attempt_bonus + w_drop_base·exp(−k2·d_error) + r_success_jackpot·[d_error≤0.1] − penalty_instability`
| Key | 의미 |
|---|---|
| `auto_drop_threshold` | CCIP d_impact ≤ 이 값이면 자동 drop. 작을수록 정확 but drop 경험 빈도↓ |
| `drop_wait_timeout` | drop 후 actual landing 결과 대기 (s) |
| `drop_attempt_bonus` | drop 시도 자체에 부여 (jekyun v2 신규) — sparse → dense |
| `w_drop_base` | drop accuracy reward base weight. exp(-k2·d_error) 와 곱해짐 |
| `k2_precision` | drop precision decay. 작을수록 먼 거리도 의미있는 보상 (k2=5 면 2m drop ≈ 0, k2=0.3 면 2m drop ≈ 0.55) |
| `r_success_jackpot` | success_threshold 안 drop 시 추가 bonus |
| `success_threshold` | jackpot 거리 임계 (m) |
| `penalty_instability` | drop 시 |ω|>limit_ang_vel 또는 tilt>limit_tilt 면 negative |
| `limit_ang_vel`, `limit_tilt` | instability 판정 임계 |
| `truncation_penalty` | max_steps 초과 (timeout) 시 penalty. 너무 크면 "drop 안 하느니 빨리 죽기" |
| `invalid_drop_threshold` | drop_error 가 이 값 초과 시 invalid drop 판정 (v6 처방 C) |
| `invalid_drop_penalty` | invalid drop 시 추가 penalty. **v8: 50 → 0 (drop 회피 학습 차단)** |
| `hover_drop_block_threshold` | vel_xy < 이 값 면 publish_drop 무시 (v6 처방 A). **0 = 비활성** |
| `drop_angaccel_penalty_scale` | **v9a NEW** — drop 시점 직전 N step 의 max angular acceleration penalty scale. toss 의 급격한 pitch back 차단 |
| `drop_angaccel_window_n` | **v9a NEW** — ang_vel diff 계산용 window 길이 (step). E 측정 방법: 인접 step diff magnitude 의 max |
| `g` | 중력 (m/s²). CCIP 계산용 |

### 1-5. `wandb:` — 로깅

| Key | 의미 |
|---|---|
| `project` | wandb project 명 |
| `entity` | wandb team/org slug |
| `run_name` | run 표시명 (null = wandb 자동 생성) |
| `tags` | run tag list (검색용) |
| `save_model_artifact` | 최종 모델을 wandb artifact 로 업로드 |
| `log_freq` | WandbCallback gradient_save_freq (작을수록 로그 잦음, training loop block 위험) |
| `mode` | "online" / "offline". offline 은 별도 sync 스크립트 필요 |

### 1-6. 코드 측 callback (yaml 외)

| Key | 의미 |
|---|---|
| `env/mean_rew_impact` | rollout 평균 r3_impact (jekyun_v2 에는 없음, junsang 추가) |
| `env/mean_rew_dist/orient/ctrl/drop` | rollout 평균 각 layer reward |
| `env/drop_count` | per-rollout drop 수 (drop 없는 rollout 은 로깅 안 됨) |
| `env/drop_error_actual_m` | rollout 평균 실제 drop 오차 (m) |
| `env/mean_d_xy`, `mean_d_impact` | rollout 평균 거리 |
| **[2026-05-23 entry #13 이후 — 누적 count metric]** | (x축은 `env/total_episodes` 로 wandb 자동 매핑) |
| `env/total_episodes` | universal episode counter (x축 base) |
| `env/total_drop_terminated_count` | drop 발동 누적 (Layer 4) — 옛 `env/total_drop_count` 이름 변경 |
| `env/total_success_count` | d_error ≤ 0.5m 누적 |
| `env/total_jackpot_count` | d_error ≤ 0.1m 누적 (success_threshold) |
| `env/total_truncate_<reason>_count` | crash/overspeed/ang_vel/inverted/timeout 5종 누적 |
| `env/total_safety_violation_count` | crash + overspeed 합산 누적 |
| `env/total_physics_glitch_count` | physics 이상 누적 (NaN 등) — 옛 `physics_glitch_count` 이름 변경 |
| **[entry #13 에서 제거됨]** | env/success_rate, env/safety_violation_rate, env/truncate_*_rate, env/drop_terminated_rate (rate 8개) — 누적 count 로 대체 |

---

## 2. 현재 활성 설정 snapshot (2026-06-27 — v9a)

**Branch**: junsang
**Run name**: `phase1_redux_v9a_payload_dist_angaccel`
**WandB run**: `zjexq20k` (SIGTERM preempt @ step 313k)
**Base**: v8 warm start (`v8_peak_step217040_err0.87m.zip`) + fresh replay buffer
**Yaml 핵심 값** (기본값 외):

| Section.Key | 현재 값 | 출처 entry |
|---|---|---|
| training.total_timesteps | **100000** (override 시 400000) — v9a fine-tune | #28 |
| training.eval_freq | 10000 | #4 (L6) |
| training.eval_episodes | 3 | #4 (L6) |
| sac.buffer_size | 500000 | #4 (M1) |
| sac.gamma | 0.995 | #4 (H3) |
| sac.gradient_steps | 1 | #11 |
| sac.learning_rate | 1e-4 | Round 3 |
| sac.tau | 0.002 | Round 3 |
| sac.use_per | true | Round 3 |
| sac.per_alpha | 0.6 | Round 3 |
| sac.per_priority_max | 30.0 | Round 3 |
| sac.target_entropy | -15.0 | Round 7 |
| sac.ent_coef_hard_cap | 1.0 | Round 7 |
| sac.target_q_clip | 500.0 | Round 7 v3 |
| environment.target_enu_x | **4.0** | Phase 1 redux v1 |
| environment.target_enu_y | **3.0** | Phase 1 redux v1 |
| environment.pos_scale | **5.0** | Phase 1 redux v3 |
| environment.action_vx_scale | **3.0** | Phase 1 redux v3 |
| environment.action_vy_scale | **3.0** | Phase 1 redux v3 |
| environment.action_rate_limit | 0.2 | junsang_v4 P2 |
| environment.max_steps | 800 | Round 2 |
| environment.min_altitude | 3.0 | junsang_v4 P4 |
| environment.min_altitude_start_step | 1 | #002 |
| environment.ground_contact_altitude | 0.5 | #002 |
| environment.max_distance | **20.0** | Phase 1 redux v3 |
| environment.max_altitude | 50.0 | Round 3 |
| environment.max_consecutive_fast_resets | 100 | Round 7 v2 (#021 2차) |
| reward.auto_drop_threshold | **2.0** | Phase 1 redux v3 |
| reward.random_drop_start_step | 600 | Round 2 |
| reward.random_drop_prob | **0.0** | Phase 1 redux v1 |
| reward.hover_drop_block_threshold | **0.0 (비활성)** | v6→v8 |
| reward.invalid_drop_threshold | **95.0** | v8 |
| reward.invalid_drop_penalty | **0.0 (비활성)** | v8 |
| reward.drop_wait_timeout | **3.0** | v6 처방 B |
| reward.w_dist | **1.5** (1.0 → 1.5) | **v9a** |
| reward.w_heading | 0.7 | Round 5 |
| reward.w_distance_penalty | 0.0 | Round 5 |
| reward.w_impact | 0.4 | jekyun v3 base |
| reward.k_impact | 0.05 | jekyun v2 base |
| reward.k2_precision | 0.2 | Round 2 |
| reward.w_drop_base | 100.0 | jekyun v2 base |
| reward.drop_attempt_bonus | 30.0 | Round 2 |
| reward.k_drop_proximity | **0.4** | Phase 1 redux v3 |
| reward.w_prediction | **0.0 (비활성)** | v5 (SDF fix 후) |
| reward.k_prediction | 0.1 | Round 1 |
| reward.r_success_jackpot | 50.0 | Round 2 |
| reward.success_threshold | **2.0** | Phase 1 redux v3 |
| reward.jackpot_threshold | **0.3** | Phase 1 redux v2 |
| reward.penalty_instability | 50.0 | Round 1 |
| **reward.limit_ang_vel** | **10.0** (2.0 → 10.0) | **#27 (ang_vel fix)** |
| reward.alt_penalty_max | 50.0 | Round 3 |
| reward.alt_penalty_mid | 30.0 | Round 3 |
| reward.alt_penalty_k | 0.15 | Round 3 |
| **reward.drop_angaccel_penalty_scale** | **0.5 (NEW)** | **v9a #28** |
| **reward.drop_angaccel_window_n** | **5 (NEW)** | **v9a #28** |
| reward.hover_speed_threshold | 1.0 | Round 5 |
| reward.hover_consecutive_threshold | 200 | Round 5 |
| reward.penalty_hover | -15.0 | Round 5 |
| reward.hover_truncate_enabled | false | Round 5 |
| wandb.entity | nayoonho0922-seoul-national-university | |
| wandb.run_name | "phase1_redux_v9a_payload_dist_angaccel" | #28 |

**코드 변경 (v9a 최신)**:
- `drone_drop_env.py`:
  - `from collections import deque` 추가
  - `__init__`: `_cfg_drop_angaccel_penalty_scale`, `_cfg_drop_angaccel_window_n`, `_ang_vel_history = deque(maxlen=N+1)`
  - `reset()`: `_ang_vel_history.clear()`
  - `step()` 매 step: `_ang_vel_history.append(ang.copy())`
  - drop trigger 후 instability penalty 다음: max ang_accel penalty
  - reset() 의 `_start_infra()` 직후: `print('[GZ_SERVER_READY] reset_count=...')` marker (dgui 용)
- `train_sac.py`: 변경 없음 (v8 와 동일, resume 사용)
- `dds_topics.yaml` (PX4): `vehicle_angular_velocity` 주석 해제 + PX4 rebuild (#27)
- `x_marker_world.sdf`: `<camera_pose>` 변경 (dgui spawn 보기 위함)

**평가 도구 (NEW, 2026-06-22 ~)**:
- `local/scripts/evaluate_gui.py` — dgui (alias)
- `local/eval_config.yaml` — dgui config
- `ros2_ws/eval_models/` — 평가용 모델 모음 (bind mount sync)
- `local/eval_logs/` — 평가 결과 json 자동 저장

---

## 3. 변경/Run 요약 표 (시간순, 새 것이 아래)

| # | 일시 | Run name | Branch | 한 줄 요약 | 결과 |
|---|---|---|---|---|---|
| 1 | 2026-05-19 | — | junsang | 카메라 파이프라인 복구 (vision world 분리) | OK |
| 2 | 2026-05-19~20 | — | junsang | PX4 airframe rootfs cp (r0~r3) | OK |
| 3 | 2026-05-20 | 0rho5l9f | junsang | SAC 5k smoke (default 설정) | drops=1, error=23.95m |
| 4 | 2026-05-20 | pwkujvev | junsang | H1+H2+H3+M1+M2 + 10k smoke | drops=1, error=14.87m (38%↓) |
| 5 | 2026-05-21~22 | — | junsang | N1=D 200k 시도 ~37k 중단 | drop sparse 부각 |
| 6 | 2026-05-22 | — | junsang | N1=B v1 (w=5) ~16k | mean_rew_impact=0.567 |
| 7 | 2026-05-22 | um8txjvk | junsang | **N1=B v2 (w=8) 200k — 실패** | 0/5 drops, takeoff도 실패 |
| 8 | 2026-05-22 | — | → jekyun_v2 | Branch 교체 (14m offset 원인 진단) | jekyun_v2 base 채택 |
| 9 | 2026-05-22 | tzbebmm4 | jekyun_v2 (junsang_v2) | dry-run 1k | Training complete, sim 정상 |
| 10 | 2026-05-22 | zn7xrm7e | jekyun_v2 (junsang_v2) | 200k 본학습 (online) | **실패** — critic 폭주 (17K @ 63k), ent_coef 1.13, deterministic 정책 망가짐 |
| 11 | 2026-05-23 | w9flirvp | jekyun_v2 (junsang_v3) | M2 REVERTED (gradient_steps 4→1) | **실패** — SAC 안정성 회복했으나 정책 학습 안 됨 (mean_d_xy 20m 정체, 159k 까지) — env/reward 측 근본 문제 |
| 12 | 2026-05-23 | ujvpo8ry(5k), krxfl97k(200k) | jekyun_v2 (junsang_v4) | **Tier 1 (P1~P11) 적용** — action scale 8 + rate limit + early terminate + curriculum 10m + bad_attitude check | drop_error 12~14m 수렴, success=0, inverted 44% |
| 13 | 2026-05-23 | — | jekyun_v2 (junsang_v4) | WandB callback overhaul (rate → 누적 count) | src만 수정, install 미러는 #14 |
| 14 | 2026-05-24~25 | wdwoim34(19k crash), xk7rw5e1(112k kill) | jekyun_v2 (junsang_v4_150k) | DropEpisodeRecorder 추가 + 150k 재학습 | drop_error 13.15m, success=0, 이전과 동일 양상 |
| 15 | 2026-05-25 | ruozrv5x | jekyun_v2 (round1) | **Round 1: #001+#002+#003+#006** — hybrid drop + 보상 스케일 축소 + 종료 조건 정비 | 432 drops, best 4.64m, avg 14.02m, success 1건. d_xy 11.3m 정체 |
| 16 | 2026-05-26~30 | dbi74uif(첫시도), z05fx7g9(최종) | jekyun_v2/junsang (round2) | **Round 2: gradient + max800 + 종료조건 진화** | dbi74uif: 접근실패 (start_step 150 문제). z05fx7g9: success 16건(16배), best 2.53m, 평균 19.09m, post-success regression 발견 |
| 17 | 2026-05-30~31 | q13hli0y(발산), lidq3ydu(157k 크래시) | junsang (round3) | **Round 3 (조합 C): PER + LR 1e-4 + Tau 0.002 + Sigmoid alt + 4가지 안전장치** | 104 drops, success 8건 (Round 2 대비 2배 속도), PX4 로그 20GB 누적 → gz timeout 크래시 |
| 18 | 2026-05-31 | vo1l9wl6(14k 버그), 4j46qwpk(146k 발산) | junsang (round4) | **Round 4 (A+C): Hover per-step 차단** — w_heading 0.3, w_distance_penalty 0.03 | 학습 발산 — ent_coef 6.03 폭주, critic_loss 230k+. 원인: per-step density 축소 → reward sparsity → SAC auto-entropy 양성 피드백 |
| 19 | 2026-05-31 | sdjytkpv(65k pause), mnlr1zpe(148k 발산) | junsang (round5) | **Round 5: Hover terminal penalty** — Round 4 복원 + episode-end hover -15 | 또 발산 — ent_coef 6.16 (Round 4와 동일). 처방 무관, SAC 본질 문제 확인 |
| 20 | 2026-05-31 | bfv4la9a (162k 중단) | junsang (round6 v1) | **Round 6 v1: DampedEntropySAC (mean)** | Mean damping 작동 안 함 (ent_damping=1.0 유지). ent_coef 1.68. 체크포인트 95k 이후 저장 안 됨 (#020) |
| 21 | 2026-05-31~06-03 | 6b8bslmz (294k OOM) | junsang (round6 v2) | **Round 6 v2: Percentile damping + 체크포인트 정렬 fix** | ent_damping 작동, 95~195k 학습 성공 (success 4건, best 4.36m). 그러나 hard cap 2.0 갇힘 → critic 14M 폭주 → OOM |
| 22 | 2026-06-03 | iobwvcrm (학습 중) | junsang (round7) | **Round 7: target_entropy=-15 근본 처방 + hard cap 1.0** | (학습 중 — 150k) |
| 23-30 | 2026-06-04~07 | 다양 (Round 7 v3 ~ v4) | junsang | Round 7 v3 (685k 완성), kill_episode timeout, Phase 1 redux v1/v2/v3/v4 progression — 자세한 history 는 §4 #25~#33 + master.txt | best 1.32m → 0.809m → v5 fresh start (#023 SDF fix) |
| 33 | 2026-06-07 | (phase1 redux v5) | junsang | **v5 SDF fix** (#023): payload_{0~3}/model.sdf 에 `<dimensions>3</dimensions>` 추가, `w_prediction 20.0 → 0.0`, 옵션 A 코드 전부 제거. Fresh start | success ~16% (이전 false reward 위 학습 → 진짜 reward) |
| 34 | 2026-06-11~ | (phase1 redux v6) | junsang | **v6 hover drop 처방** (#022 fps + v5 invalid 50%): `drop_wait_timeout 10.0 → 3.0`. hover_drop_block_threshold 검토 후 폐기 (0 = 비활성) | success 11%, invalid drop 50% 여전 (실제 root cause = DetachableJoint plugin) |
| 35 | 2026-06-15~ | (phase1 redux v7) | junsang | **v7 safe attach (옵션 C)**: DetachableJoint reattach 후 detach silent fail 우회 시도 | **drop 0 — 학습 완전 실패**. invalid_drop_penalty 50 이 정책 drop 회피 학습 강제 |
| 36 | 2026-06-19~21 | **96bokgae** (303k) | junsang (phase1 redux v8) | **v8 no_invalid_penalty**: `invalid_drop_penalty 50 → 0`, `invalid_drop_threshold 50 → 95`. D1 처방 = 매 drop 후 `_kill_infra` (옵션 C 비활성) | **success 80.6%, jackpot 13, mean 1.85m, best 0.07m**. toss 전략 발견 (drone 이 marker 지나친 후 pitch back). 8,736 ep / 3,342 drops. backup 8.6 GB |
| 37 | 2026-06-22 | (v8 fix) | junsang | **ang_vel callback fix**: PX4 `dds_topics.yaml` 의 `vehicle_angular_velocity` 주석 제거 + PX4 rebuild. `limit_ang_vel: 2.0 → 10.0` (false crash 방지) | v8 학습 전체 obs[6:9]=0 이었음 (정책 영향 미미, 5 ep 평가 동일). v9a 의 drop_angaccel 의 prerequisite |
| 38 | 2026-06-26~27 | **zjexq20k** (313k, +17k) | junsang (v9a) | **v9a payload_dist + drop_angaccel**: `w_dist 1.0 → 1.5`, NEW `drop_angaccel_penalty_scale=0.5`, NEW `drop_angaccel_window_n=5`. v8 warm start + fresh replay buffer. SIGTERM stop @ 313k (의도 100k 의 17%) | 5 ep 평가: success 80% (v8 동일), mean 1.89m, **max ang_vel 2.10 rad/s (v8 2.5 대비 -16%)**. toss 그대로 유지 — fine-tune 17k 부족 |
| 39 | 2026-06-26~27 | **xzoz52cw** (313k → 432k) | junsang (v9a resume) | **v9a resume**: 313k preempt 에서 resume + replay buffer 자동 load. 의도 ~600k 까지, 사용자 SIGTERM 예정 | **CUDA error → container SIGKILL (137) @ step 432k**. preempt save 실패. rolling checkpoint 5k 마다 자동 저장 → `sac_drop_432806_steps.zip` 보존. drop_episodes +449 누적 |
| 40 | 2026-06-27 | (평가 only, 5 ep dgui) | — | **v9a step432k 평가** | 5 ep: drops 3, **success 2/3 = 66.7%**, mean 1.96m, **2 hover_timeout** ⚠️. EP5 max ang_vel 2.95 (outlier) — 정책 unstable. 추가 학습 (+119k) 이 정책 악화 시킴 |
| 41 | 2026-06-27 | (평가 only, 10 ep dgui) | — | **v8 + v9a 313k 10 ep 비교** — 통계 신뢰성 확인 | **v8**: 5/10 = 50%, mean 2.002m, 0 hover_timeout. **v9a 313k**: 3/9 = 33%, mean 2.006m, 1 hover_timeout. → 이전 5 ep 100% / 80% 는 표본 운. **v8 가 best baseline** (v9a 보다 success 17% ↑). 둘 다 mean ≈ 2.0m (success_threshold 2.0m 경계) |
| 42 | 2026-06-30 | (RAD v1 design only) | — (신규 framework, hyperparams_rad.yaml 예정) | **RAD v1 design 완료** — Relative + Approach (Phase 1) + Drop (Phase 2). 2 정책 hierarchical, obs 14d 상대좌표 (yaw-only body frame), spawn yaw ±90°, cruise 1m/s 가속, target (4,3,0), switch sphere d²≤20.5, z Hann reward (w_z=0.3, [0.5,7.5]), 7 final state 조건 (C1~C7) jackpot 총 +120, drop trigger d_impact≤1m, w_impact 1.0/k=0.1, sphere 벗어남 crash (d²>22 → −30), Phase 2 time/ang_vel/action_smooth 2× 강화, action_rate_limit 0.15, success_threshold 1.0m, max_distance 15, max_consecutive_fast_resets 50 | design 완료, 코드 작업 대기. 자세한 변경 = §4 #42 |
| 43 | 2026-06-30~07-02 | RAD v1 Phase 1 v1 (g8mvzniw) | — | **RAD v1 Phase 1 v1 (원본 학습)**: Original design 그대로 시작 | NaN abort at 62k step (actor forward NaN). 진단 여러 가설 (H1/H3/H5) 확답 못 함. Issue #028 최초 |
| 44 | 2026-07-02 | RAD v1 Phase 1 v2 | — | **v2 self-healing 도입** — A1: `clip_grad_norm_(actor, 20)` / `(critic, 200)`, B3: 매 gradient step 시 weight snapshot → step 후 NaN 감지 시 rollback. 상세 metric 대량 추가 (train/{q_target,q_current,q_pi}_{max,mean,min,std}, actor_loss_{q,ent}_term, log_prob_{mean,std}, reward_batch_{mean,max,min}, actor/critic weight/grad norm, nan_rollback_count, actor/critic_grad_clip_hit) | NaN abort at 171k step 재현. Self-healing 은 발동 안 함 (weight rollback 못 잡음). NaN 은 rollout action sampling 시 발생 |
| 45 | 2026-07-03 | RAD v1 Phase 1 v3 | — | **v3 curriculum + regression 도입**: 5-stage (intro/close/target/partial/full), advance 조건 강화 (window 5k→10k, threshold 0.9→0.95, min_stage 2k→10k), regression 신규 (window success<0.3 시 이전 stage 복귀, cooldown 20k), curriculum config 관련 yaml key 대량 추가 | Stage 1 정체 (75%). 그 후 발산 방향 진행. Curriculum 매커니즘 자체는 정상 작동 |
| 46 | 2026-07-03 | RAD v1 Phase 1 v4 | — | **v4 reward magnitude 축소**: `penalty_crash: -50 → -20`, `penalty_hover: -30 → -10`, `truncation_penalty: -15 → -5`, `target_q_clip: 500 → 200`, `max_consecutive_fast_resets: 50 → 500` (Phase 1 fast reset 정상) | Stage 1↔Stage 2 cycle 반복 (Advance-Regress 2회). Q_target_std 289→166 (43% 축소), q_clip_ratio 58%→41%. Stage 2 hover 65% 로 stuck |
| 47 | 2026-07-04 | RAD v1 Phase 1 v5 | — | **v5 stage2_close 완화**: switch_d² 25→30.25 (radius 5→5.5m), z_min 0.15→0.08, max_distance 40→50, limit_tilt 0.9→1.0 | 여전 Stage 2 hover 65% 로 실패. 갭 완화가 문제 아님 — 정책이 sphere entry 시도 자체 안 함 |
| 48 | 2026-07-04~05 | RAD v1 Phase 1 v6 | — | **v6 근본 접근**: `spawn_yaw_relative_range: ±90° → ±45°`, `penalty_hover: -10 → -30` (crash -20 대비 강 penalty), **info['initial_target_dist_3d/xy'], initial_pos_{x,y,z}, initial_speed_xy log 신규** (env callback rolling stats → WandB) | Stage 1 통과 (14k), Stage 2 통과 (28k, hover 65%→1% 극적 효과), **Stage 3 완전 실패 (0%)** cycle 반복. **Initial pos 실측으로 진짜 원인 확정**: target 거리 5.10m (spawn 근처). Stage1/2 는 spawn 이미 안 (trivial), Stage3 만 학습 필요 → Issue #029 |

---

## 4. 상세 entry (시간순, 새 것이 아래)

> **형식 규칙** (사용자 명시):
> - 변경: `<key>: <old> → <new>` + 이유
> - 추가: `NEW: <key> = <value>` + 이유
> - 제거: `REMOVED: <key>` + 이유

---

### #1. 2026-05-19 — 카메라 파이프라인 복구

**No yaml change** (코드/world 변경만).

**World/Launch 변경**:
- `gazebo_models/worlds/x_marker_world_vision.sdf` — NEW 파일 (base 의 복사 + Sensors+ogre2 추가)
- `drone_mission.launch.py` L196/204/321: `x_marker_world.sdf → x_marker_world_vision.sdf`

**이유**: base world 에 `gz::sim::systems::Sensors` 플러그인 의도적 제외 (RL 학습 안정성). vision 미션은 카메라 필요 → 분리.

---

### #2. 2026-05-19~20 — PX4 airframe rootfs 동기화

**No yaml change**.

**Container 측 변경**:
- ROMFS 의 `4016~4019_gz_x500_bombard_r{0,1,2,3}` → rootfs 로 cp

**이유**: 기본 PX4 빌드가 rootfs 에 4015 만 채움. 학습 코드는 `PX4_SIM_MODEL=gz_x500_bombard_r{instance_id}` 사용 → 누락된 airframe 으로 silent fail.

---

### #3. 2026-05-20 — SAC 5k smoke (default 설정)

WandB run: `0rho5l9f`
**No parameter change** — 기본값으로 baseline 측정.

**결과**:
- ep_rew_mean: -874 → -936 (평탄/악화)
- drops=1, drop_error=23.95m, mean_rew_drop=0
- safety_violation_rate=1.0, fps=22~37

---

### #4. 2026-05-20 — H1+H2+H3+M1+M2 적용 (10k smoke `pwkujvev`)

**변경**:
- `sac.buffer_size`: 100000 → 500000  (M1)
  - **이유**: 본학습 1M step 시 마지막 100k 만 남아 catastrophic forgetting. 초기 다양성 보존.
- `sac.gamma`: 0.99 → 0.995  (H3)
  - **이유**: effective horizon 100→200 step. 장기 maneuvering credit assignment 강화. H1 의 reward 평탄화와 같이 적용.
- `sac.gradient_steps`: 1 → 4  (M2)
  - **이유**: off-policy sample reuse. fps 22~37 환경에서 update 효율 향상.
- `reward.w_drop_base`: 50 → 20  (H1)
- `reward.r_success_jackpot`: 100 → 30  (H1)
- `reward.penalty_instability`: 50 → 15  (H1)
  - **(H1 묶음 이유)**: critic 이 step reward ±0.5 와 terminal ±100 spike 를 동시에 fit → variance 폭주. terminal scale 축소로 안정화.
- **action_space**: Box(5,) → Box(4,)  (H2, drone_drop_env.py 코드)
  - **이유**: action[4] manual drop trigger 가 CCIP auto-drop 과 충돌, gradient 거의 못 받음. CCIP only 정책으로 단순화.

**추가** (NEW):
- `training.eval_freq` = 10000  (L6, yaml 신규 키)
- `training.eval_episodes` = 3  (L6, yaml 신규 키)
  - **이유**: BestModelCallback 이 hardcoded 10_000 이었음. smoke/본학습 분리 가능하게 config 로 빼냄.

**결과**:
- ep_rew_mean: -1340→-823→-511 (단조 개선)
- drops=1, drop_error=14.87m (이전 23.95m 대비 38%↓)
- critic_loss 11.4→49.1 폭주→32.9 안정화 (M2 안전성 확인)
- 한계: drops=1 (sparse) → N1 안건으로 이행

---

### #5. 2026-05-21~22 — N1=D 200k 시도 (중단)

**No parameter change** — N1 옵션 D ("변수 추가 없이 1M 가서 봄") 채택.

**진행**: ~37k step.
**중단 이유**: drop_count=1 지속, drop sparse signal 문제 부각. `Downloads/N1_drop_sparse_AB방안.md` 분석으로 N1=B (가중치만 강화) 선도입 결정.

---

### #6. 2026-05-22 — N1=B v1 (~16k)

**변경**:
- `reward.w_impact`: 2.0 → 5.0
  - **이유**: drop sparse → 위치 신호 강화. potential-based 아니지만 가중치만 키워 저위험.
- `reward.k_impact`: 0.05 (유지)
  - **이유**: 별도 자료 권고 0.1 은 80m 에서 reward≈0.0017 (다른 항 대비 3자릿수 작음) — 본학습 초반 의미 없음. 0.05 유지.

**추가** (NEW, drone_drop_env.py 코드):
- `r3_impact` 에 `* speed_gate` 곱
  - **이유**: r3_impact 가 potential-based 아닌 단일 상태 함수 → loitering hack 위험. r3_orient 의 anti-milking 패턴 재사용.

**추가** (train_sac.py callback):
- `env/mean_rew_impact` 로깅 (3곳, callback 누락 수정)
  - **이유**: rew_impact 가 info dict 에 있지만 callback 수집 누락. N1=B 의 핵심 측정 지표라 필수.

**결과** (16k 부분 학습):
- mean_rew_impact: 0.567 (메커니즘 작동 확인, dry-run 의 0.00144 대비 ~400배)
- mean_rew_orient: -0.998 → +0.036 (드론이 타겟 방향 학습)

---

### #7. 2026-05-22 — **N1=B v2 — 200k 실패** (`um8txjvk`)

**변경** (#6 대비):
- `reward.w_impact`: 5.0 → **8.0**
- `reward.k_impact`: 0.05 → **0.03**
  - **이유**: v1 의 16k 데이터에서 메커니즘 작동 확인 → "위에서부터 줄이기" 전략으로 ②(w=8, k=0.03) 후보 채택. 후보 비교: ① w=5,k=0.025 (보수), ② w=8,k=0.03 (균형), ③ w=10,k=0.025 (공격적, terminal jackpot 침범 위험).

**추가** (NEW, train_sac.py):
- `env/total_drop_count` 누적 카운터 (3곳)
  - **이유**: env/drop_count 는 rollout-reset 됨 → wandb 차트가 "계속 1" 로 보이는 혼란. 누적 카운터로 stair-step 차트 가능.
  - **적용 시점**: Python 모듈 캐싱 때문에 현재 run 에는 영향 없음. 다음 학습부터 반영.

**Wandb mode 변경**:
- `wandb.mode`: offline → **online** (사용자 선택)
  - **이유**: incremental sync 스크립트 대신 직접. deadlock 위험 인지 후 진행.

**결과 (재앙)**:
- 200k step 완주 + `sac_drop_final.zip` 저장
- evaluate 5 epi (deterministic): **0/5 drops**, mean reward -6060
- trajectory: Target(11,10) → End(-16,23) 반대 방향 비행
- 사용자 GUI 관찰: "처음부터 바닥에 부딪히면서 튕기다 어디론가" → **takeoff 도 실패**
- 정책 완전 무용 → 학습 폐기

**진행 중 진단 (200k 도중)**:
- wandb API 로 19 drops 분석 → drop_error_actual_m 이 **17/19 회 정확히 14.8661m**
- CCIP 1m 예측 vs 실제 14.87m → 14m systematic offset (시뮬 측 구조적 결함 의심)

---

### #8. 2026-05-22 — Branch 교체 (junsang → jekyun_v2)

**No yaml change** — branch checkout 만.

**이유**:
- 14m offset 이 RL parameter 로 해결 불가 (sim 측 결함 의심)
- 사용자가 다른 branch (`origin/jekyun_v2`) 에서 정상 동작 확인
- jekyun_v2 의 commit 09ea871 의 **인프라 안정성 patch 4가지**:
  1. `_spin_loop` (resilient spin thread)
  2. `_reset_depth` recursion guard
  3. tiered CRUISE recovery
  4. `_kill_infra` pkill 폴백
- jekyun_v2 의 `RL_analysis.md` (431줄) 가 우리 200K 실패와 같은 실패 모드 (360K v2 mean_d_xy=19.9m, 안 움직임) 분석 + v3 처방 검증
- 14m offset 진짜 원인 추정: spin thread crash → callback freeze → mission_state freeze → CCIP stale data, 또는 infra reuse 옛 PX4 잔재 → 좌표 오염

**Backup**: `/home/juns/backup_pre_pull_2026-05-22/` (4.7GB, 8 카테고리)

---

### #9. 2026-05-22 — junsang_v2 셋업 + dry-run 1k (`tzbebmm4`)

**Base**: jekyun_v2 (commit 46bdb17). reward v3 + 인프라 patch 그대로 수용.

**Jekyun_v2 의 reward 측 활성 값** (우리가 안 건드림, 참고용):
- `auto_drop_threshold`: 2.0 → 4.0 (jekyun, drop 빈도↑)
- `w_dist`: 1.0 → 0.5 (jekyun)
- `w_heading`: 1.0 → 0.7 (jekyun v3)
- `w_impact`: 2.0 → 0.4 (jekyun v3, 우리 8.0 의 1/20)
- `k2_precision`: 5.0 → 0.3 (jekyun, 먼 거리도 보상)
- `w_drop_base`: 50.0 → 100.0 (jekyun)
- NEW `drop_attempt_bonus`: 150.0 (jekyun v3)
- NEW `truncation_penalty`: -80.0 (jekyun v3)

**Re-apply on jekyun_v2**:
- `sac.buffer_size`: 100000 → 500000 (우리 M1 재적용)
- `sac.gamma`: 0.99 → 0.995 (우리 H3 재적용)
- `sac.gradient_steps`: 1 → 4 (우리 M2 재적용)
- `training.eval_freq` = 10000 NEW (우리 L6 재적용)
- `training.eval_episodes` = 3 NEW
- `training.total_timesteps`: 1000000 → **1000** (dry-run 임시, 원래 200000)
  - **이유**: jekyun_v2 default 1M 이지만, 우리는 200k 우선 검증 (사용자 결정). dry-run 은 1k.
- `wandb.run_name`: "L4-AutoDrop-v2" → "junsang_v2"
- train_sac.py: `_step_rew_impact` callback, `_total_drop_count` counter, `eval_freq` from yaml

**채택 안 함** (REMOVED from history):
- H1 (terminal scale 축소) — jekyun v3 는 반대 방향
- H2 (action 5d→4d) — jekyun_v2 는 5d 유지
- N1=B v2 (w_impact=8) — jekyun 분석으로 per-step 강화 실패 모드 확정
- speed_gate 코드 patch — jekyun_v2 yaml 에 이미 `speed_gate_enabled: true`
- vision world 분리 (commit 29b1c9b) — RL 학습엔 무관 (보류)

**Dry-run 1k 결과** (`tzbebmm4`, offline):
- **Training complete** ✓ (sac_drop_final.zip 저장)
- PX4 ready 17초, RuntimeError/CRUISE timeout 없음 → sim 정상 동작
- env/mean_d_xy: 27.87m (learning_starts=1000 = total_timesteps 라 random 탐색만)
- env/total_drop_count: 0 (1k 짧음, drop 발생 안 함)
- env/mean_rew_impact: 0.099 (callback 정상 작동)
- env/safety_violation_rate: 1.0 (random 단계 정상)

**진단 patch (임시)**:
- `drone_drop_env.py` L1210: `px4_log = open(os.devnull, 'w')` → `open('/tmp/px4.log', 'w')`
  - **이유**: 5k 시도 시 CRUISE timeout 원인 진단용 (junsang 5/19 패턴 재사용)
- **REVERT 완료**: PX4 log 416MB 폭증 (pxh prompt spam), 진단 가치 없음 확인 후 원복 + log 삭제

**다음**: 200k 본학습 또는 5k 중간 검증.

---

### #10. 2026-05-22 — **200k 본학습 시작 (junsang_v2, online)**

WandB run: (진행 중, online mode)
**Base**: #9 (dry-run 1k 성공)

**변경**:
- `training.total_timesteps`: 1000 → **200000**
  - **이유**: dry-run 1k 성공 (Training complete, sim 정상 동작 확인) → 본학습 진입.

**환경**:
- `WANDB_MODE=online` (사용자 결정, deadlock 위험 인지 후 진행)
- 컨테이너 재시작 후 (이전 OOM exit 137 발생) — PX4 airframes rootfs 5개 정상 확인 후 진입

**결과**: (진행 중, 200k ≈ 7시간 예상)

**모니터링 지표**:
- `env/total_drop_count` (stair-step ↑)
- `env/mean_d_xy`, `env/mean_d_impact` (점진 ↓)
- `env/mean_rew_impact` (w_impact=0.4 라 작은 값)
- `env/drop_error_actual_m` (drop 발생 시)
- `env/safety_violation_rate` (learning_starts 후 ↓)
- `rollout/ep_rew_mean` (단조 또는 점진 개선)

**완주 후 결정 트리** (jekyun RL_analysis 의 예상치):
- `mean_d_xy ≤ 5m` → 정상 수렴, 추가 학습 또는 정밀화
- `mean_d_xy 5~15m` → 처방 가이드 §3 진단 매트릭스
- `mean_d_xy > 15m`, drops=0 → jekyun Stage 1 재조정 필요

**즉시 중단 조건**:
- RuntimeError "reset() recursive" → spin thread 또는 infra 측 문제 재발 → 로그 확인
- 30k step 까지 drops=0 → reward 설계 재검토

**결과 (200k 완주)** — WandB run `zn7xrm7e`, state=finished, **학습 실패**:

| 지표 | 값 | 평가 |
|---|---|---|
| `train/critic_loss` | **17,047** (200k 시점) | **발산** (정상 ~100) — 63k 시점 685→8,400 점프 후 회복 못함 |
| `train/actor_loss` | **-2,027** | **발산** (정상 -50~-200) |
| `train/ent_coef` | **1.113** | **통제 불능** — 시작 1.0 → 0.5 → **1.35 → 1.13 진동** (정상은 단조 감소 0.1~0.5) |
| `train/n_updates` | 795,996 | M2=4 정상 동작 (200k × 4) |
| `env/total_drop_count` | **106** | 의미있는 개선 (이전 N1=B v2 의 0 대비). drop_attempt_bonus=150 효과 ✓ |
| `env/drop_error_actual_m` (final) | 9.02m | 14m offset 없음 ✓ (인프라 patch 효과) |
| `env/mean_d_xy` (final) | 32.68m | 정책 망가짐 — target 에서 멀어짐 |
| `env/safety_violation_rate` | 1.0 | 마지막까지 crash 빈발 |
| Deterministic evaluate | **"전혀 학습되지 않음"** | 사용자 GUI 관찰 |

**Timeline (40 samples)**:
- Phase 1 (0~30k): critic 27→222 정상, d_xy 117→12 (target 접근), drops 1→15 ✓
- Phase 2 (30~60k): critic 222→657 증가 시작 ⚠
- **Phase 3 (63k 부근)**: critic 685 → **8,400** (10배 점프) 🚨
- Phase 4 (71k~200k): critic 5K~21K 진동, actor -1700~-2900, ent_coef 1.05~1.35 진동, d_xy 11~290m 진동

**원인 진단**:
- **M2 (gradient_steps=4)** 이 **jekyun v3 의 큰 terminal reward (drop_attempt_bonus 150 + w_drop_base 100 + jackpot 100 = 최대 +350, truncation -80)** 와 충돌
- critic 이 step reward (±0.5) 와 terminal spike (+250~350) 를 4배 빠르게 fit 하려다 발산
- **H3 (gamma 0.995)** 도 future reward 누적 효과 키워 Q variance ↑
- 63k 시점 = buffer 의 ~13% 시점에 학습 본격화되며 임계 도달

**Archive**: `/workspace/ros2_ws/rl_checkpoints/archive/junsang_v2_200k_failed_2026-05-22/` (22MB, 7 zip — 75k~95k + final + model)

---

### #11. 2026-05-23 — **junsang_v3: M2 REVERTED** (gradient_steps 4→1) — 단독 lever

**Base**: #10 (junsang_v2, 200k 학습 실패)

**변경**:
- `sac.gradient_steps`: 4 → **1**
  - **이유**: #10 에서 63k 시점 critic 685→8,400 점프 + 200k 까지 발산 지속. M2 만 단독 변경하여 critic 폭주 원인 검증 (one-lever-at-a-time 원칙).
- `wandb.run_name`: "junsang_v2" → "junsang_v3"
  - **이유**: 학습 구분 명확화.

**유지** (변경 안 함):
- `sac.buffer_size`: 500000 (M1 유지)
- `sac.gamma`: 0.995 (H3 유지)
- 모든 jekyun_v3 reward (drop_attempt_bonus 150 등)
- train_sac.py callback 들
- drone_drop_env.py: jekyun_v2 그대로

**검증 가설**:
- 만약 critic 안정 (loss < 300) → M2 가 critic 폭주의 단독 원인 확정
- 여전히 폭주 → terminal scale 또는 H3 가 추가 원인 (Stage 2 lever 필요)

**예상**:
- jekyun v1 (118K, gradient_steps=1, w_drop_base=50) 는 정상 수렴했음 → gradient_steps=1 이 그 안정성의 핵심 가능성
- M1 (큰 buffer) + H3 (큰 gamma) 만 추가는 SAC 측 효과 작음
- 200k 후 critic 안정 + drops 의미있게 증가 + d_xy 단조 감소 → 본격 학습 가능 환경 확인

**환경**:
- `WANDB_MODE=online`
- Container 안의 yaml + train_sac.py + drone_drop_env.py 모두 #10 과 동일 (gradient_steps 만 변경)

**상태**: 사용자 직접 실행 예정 (tmux 패턴). 명령은 #10 과 동일 (`ros2 run rl_navigation train_sac`).

---

### #12. 2026-05-23 — **junsang_v4: Tier 1 (P1~P11) 적용**

**Base**: #11 (junsang_v3, 정책 학습 실패)
**참조**: [design_review_2026-05-23.md](design_review_2026-05-23.md) (9회차 검토 완료)

**변경 — yaml (11 키)**:
- `environment.action_vx_scale`: 15.0 → **8.0**  (P1)
  - **이유**: 사용자 GUI "시작하자마자 튀어나가는" → 최대 수평 속도 제한
- `environment.action_rate_limit`: NEW → **0.2**  (P2)
  - **이유**: 사용자 "가속도 제한" → step 당 |Δa| ≤ 0.2 hard clip
- `environment.min_altitude`: 2.0 → **3.0**  (P4)
- `environment.min_altitude_start_step`: 20 → **10**  (P4)
  - **이유**: 사용자 "고도 최저선" → 상향 + 검사 빠르게
- `reward.auto_drop_threshold`: 4.0 → **10.0**  (P6)
  - **이유**: 사용자 "10m 안 일단 성공" → curriculum phase 1
- `reward.penalty_crash`: -10.0 → **-100.0**  (P5)
- `reward.penalty_overspeed`: -8.0 → **-50.0**  (P3)
- `reward.truncation_penalty`: -80.0 → **-30.0**  (P8)
  - **이유**: 사용자 "바닥 닿으면 안 됨" + timeout 압도 완화
- `reward.limit_inverted_tilt`: NEW → **1.047** (60°)  (P11)
- `reward.penalty_bad_attitude`: NEW → **-50.0**  (P11)
  - **이유**: 사용자 "절대 안 좋은 행동" (급격 회전/inverted) → step 별 검사 + truncate
- `wandb.run_name`: "junsang_v3" → **"junsang_v4"**

**변경 — drone_drop_env.py 코드**:
- cfg 로드 5 키 추가 (`_cfg_action_rate_limit`, `_cfg_limit_inverted_tilt`, `_cfg_penalty_bad_attitude`, `_cfg_drop_attempt_bonus`, `_cfg_truncation_penalty`)
  - **숨겨진 버그 발견**: `drop_attempt_bonus=150` 과 `truncation_penalty=-80` 이 코드에 **hardcoded** (L653, L695) — yaml 변경 효과 없었음. cfg 로드 + 코드 사용 변경 (이전 entry 의 yaml 변경이 사실상 무력했음)
- step() L572 직후: P2 action rate clip 추가
- step() L688 의 _compute_reward 호출 후: P11 safety 검사 + truncated 결정 (crash/overspeed/ang_vel/inverted)
- step() L692 의 max_steps timeout: `info['truncate_reason'] = 'timeout'` 추가

**변경 — train_sac.py callback**:
- `WandbMetricsCallback.__init__`: `_truncate_counts` dict + `_rollout_done_episodes` counter 추가
- `_on_step`: `info['truncate_reason']` 집계
- `_on_rollout_end`: 6 metric wandb 로깅 (`env/truncate_<crash|overspeed|ang_vel|inverted|timeout>_rate` + `env/drop_terminated_rate`)

**채택 안 함 (Tier 2 보류)**:
- P7 (per-step reward 강화) — orbit-milking 위험, Tier 1 결과 보고 결정
- action_vz_scale 축소
- limit_tilt 강화 (drop 시점만)
- wandb_watch.py 외부 모니터링

**Archive (이전 run)**: `/workspace/ros2_ws/rl_checkpoints/archive/junsang_v3_w9flirvp_failed_2026-05-23/` (7 zip, 22MB)

**Syntax 검증**: `python3 -c 'ast.parse(...)'` 양 파일 통과 ✓

**다음 단계**:
1. 5k dry-run (yaml total_timesteps 임시 5000) — WANDB_MODE=offline
2. 통과 기준 (정량): RuntimeError 없음, Training complete, total_drop_count≥1, safety_violation_rate<1.0, mean_d_xy 측정됨, ep_len_mean<500
   (entry #13 이후 metric 명: `env/total_drop_terminated_count` ≥ 1, `env/total_safety_violation_count` / `env/total_episodes` < 1.0)
3. 임계 적정성: `env/truncate_ang_vel_rate` < 0.5 (만약 ≥50% 이면 limit_ang_vel/action_rate_limit 완화 검토)
   (entry #13 이후: `env/total_truncate_ang_vel_count` / `env/total_episodes` < 0.5)
4. 답답함 점검: mean_d_xy 가 5k 동안 감소 (시작 50~100m → 끝 30~50m). 안 줄어들면 action_rate_limit 0.2→0.3
5. 5k 통과 → 사용자가 200k 본학습 (WANDB_MODE=online)

---

### #13. 2026-05-23 — **WandB callback metric overhaul (rate → 누적 count, episode x축)**

**Base**: #12 (junsang_v4 Tier 1, 현재 phase 1 진행 중)
**적용 시점**: **src 만 수정 — install 미러는 Phase 2 진입 시점**. Phase 1 학습 영향 0.

**문제**:
- rollout 의 episode 수 적을 때 rate metric (예: `env/truncate_crash_rate`) 가 0/N or N/N 형태 → wandb 차트가 0/1 binary 진동
- x축 = `time/total_timesteps` 이라 episode 길이 비균질 (54 step vs 500 step) → 추세 보기 어려움

**해결**:
- 모든 rate metric → 누적 count 로 변환 (monotonic stair-step)
- universal `env/total_episodes` counter 추가 (x축 기준)
- `wandb.define_metric('env/total_*', step_metric='env/total_episodes')` 자동 매핑

**제거** (rate 8개):
- `env/truncate_<crash|overspeed|ang_vel|inverted|timeout>_rate` (5개)
- `env/drop_terminated_rate`
- `env/success_rate`
- `env/safety_violation_rate`

**추가** (누적 count 9개):
- `env/total_episodes` (universal counter, x축)
- `env/total_drop_terminated_count` (기존 `total_drop_count` 의 이름 변경)
- `env/total_success_count` (d_error ≤ 0.5m 누적)
- `env/total_jackpot_count` (d_error ≤ 0.1m 누적, success_threshold)
- `env/total_truncate_<crash|overspeed|ang_vel|inverted|timeout>_count` (5개)
- `env/total_safety_violation_count` (crash + overspeed 합산)

**유지** (사용자 결정 #4):
- `env/drop_count` (per-rollout, 그대로)
- `env/mean_*` (mean metric, 그대로)
- `env/physics_glitch_count` (이미 누적)
- `train/*`, `rollout/*` (SAC/SB3 기본)

**코드 변경** (train_sac.py 의 WandbMetricsCallback):
- `__init__`: 누적 변수 추가 (`_total_episodes`, `_total_drop_terminated`, `_total_success_count`, `_total_jackpot_count`). 이전 `_safety_violation_count`, `_total_steps`, `_rollout_done_episodes` 제거 (rate 계산용이라 불요)
- `_on_step`: episode 종료 (done) 시 누적 카운터 갱신. truncate_counts 도 rollout reset 없이 누적.
- `_on_rollout_end`: rate 계산 코드 모두 제거 (`if self._rollout_done_episodes > 0` 블록 + `safety_violation_rate` 블록). 누적 log_dict 항목 추가.
- `main()` 의 `wandb.init()` 직후: `wandb.define_metric('env/total_*', step_metric='env/total_episodes')` 추가.

**검증**:
- syntax OK (ast.parse)
- src 18 occurrence 새 키 ✓
- install 0 occurrence (학습 영향 0) ✓

**Phase 2 진입 시점에 install 미러** ([drone_sim_tmux_training_guide.txt](guides/drone_sim_tmux_training_guide.txt) 의 phase 전환 절차 + [A_phased_curriculum_도입방안.md](A_phased_curriculum_도입방안.md) 부록 체크리스트 참조):

```bash
docker cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py \
  drone-bombard-harmonic:/workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py
```

---

### #14. 2026-05-24~25 — junsang_v4_150k 재학습 + DropEpisodeRecorder

WandB run: `wdwoim34` (19k에서 infra crash), `xk7rw5e1` (15k resume → 112k에서 사용자 kill)
**Base**: #12 (junsang_v4 Tier 1)

**추가** (코드):
- `train_sac.py`: `DropEpisodeRecorderCallback` — drop 에피소드마다 action/obs/reward 저장 (.npz)
- `replay_drop_episode.py`: 저장된 drop episode GUI 재생 스크립트
- `train_sac.py`: callback overhaul (#13) install 미러링 완료

**변경** (yaml):
- `training.total_timesteps`: 200000 → **150000**
- `wandb.run_name`: "junsang_v4" → **"junsang_v4_150k"**

**결과** (112k step, 1112 episodes):
- total_drops: 220, total_success: **0**
- drop_error: mean 13.15m, median 12.89m, min 2.32m
- inverted: 486 (**44%** — 최대 종료 원인)
- d_xy: 12.7m (최종), 14~90m 불안정 진동
- DropEpisodeRecorder: 275 episode 정상 저장

**결론**: 이전 학습(200k)과 동일한 양상. 파라미터가 아닌 구조적 문제 확인 → Round 1 결정.

---

### #15. 2026-05-25 — **Round 1: Issue #001+#002+#003+#006 동시 적용** (코드 수정 완료)

**Base**: #14 (junsang_v4_150k 결과 분석)
**참조**: issues/master.txt, issues/issue_001~010

**변경 — yaml (14 키)**:

  Issue #001 (보상 정밀도):
  - `reward.drop_attempt_bonus`: 150.0 → **30.0**
  - `reward.k2_precision`: 0.3 → **0.5**
  - `reward.r_success_jackpot`: 100.0 → **50.0**

  Issue #003 (스케일 축소):
  - `reward.penalty_crash`: -100.0 → **-50.0**
  - `reward.penalty_overspeed`: -50.0 → **-30.0**
  - `reward.penalty_bad_attitude`: -50.0 → **-30.0**
  - `reward.truncation_penalty`: -30.0 → **-15.0**

  Issue #006 (hybrid drop):
  - `reward.auto_drop_threshold`: 10.0 → **2.0**
  - NEW `reward.manual_drop_threshold` = **0.5**
  - NEW `reward.min_drop_step` = **10** (최초 50 → dry-run 후 10으로 조정)

  Issue #010 (prediction bonus, 보수적 도입):
  - NEW `reward.w_prediction` = **20.0**
  - NEW `reward.k_prediction` = **0.1**

  Issue #002 (종료 조건):
  - `environment.min_altitude_start_step`: 10 → **1**
  - NEW `environment.ground_contact_altitude` = **0.5**
  - NEW `environment.max_distance` = **100.0**
  - NEW `reward.penalty_out_of_range` = **-30.0**

  기타:
  - `wandb.run_name`: "junsang_v4_150k" → **"round1_hybrid_drop"**

**변경 — drone_drop_env.py**:
  - hybrid drop: `(action[4] > 0.5 and step >= 10) or (d_impact <= 2.0)` and not dropped
  - prediction_bonus: `w_prediction * exp(-k_prediction * |d_impact - actual_error|)`
  - ground contact: `altitude < 0.5m → crash` (무조건, step 무관)
  - out_of_range: `d_xy > 100m → truncate + penalty -30`
  - `info['drop_trigger']` = 'manual' / 'auto' 추가
  - default 값 전부 Round 1 값으로 정합

**변경 — train_sac.py**:
  - `_truncate_counts`에 `'out_of_range': 0` 추가
  - `_total_drop_manual` / `_total_drop_auto` 카운터 + wandb 로깅 추가

**보상 curve** (prediction gap=0 가정):
  d_error 0m=200(+50 jackpot), 1m=111, 2m=87, 5m=58, 10m=51, 15m=50

**추가 변경 (dry-run/학습 중 발견)**:
  - NEW `reward.k_drop_proximity` = **0.3**
    bonus = 30 * exp(-0.3 * d_xy). 즉시 drop 방지 — 가까이 가야 bonus 받음.
    d_xy=50m→0, 14m→0.5, 3m→12.2, 0m→30.
    (run 2zrnjipt에서 step 10마다 즉시 drop 패턴 발견 → 추가)

**추가 변경 (Issue #013 발견 후)**:
  - `reward.success_threshold`: 0.1 → **5.0** (total_success_count 중간 목표)
  - NEW `reward.jackpot_threshold` = **0.1** (total_jackpot_count 최종 목표, 분리)
  - SDF 수정: 8개 model.sdf `<topic>` → `<detach_topic>` (Critical #013)

**추가 변경 (Issue #014 해결 후)**:
  - x500_bombard/model.sdf: <include merge="true"> → **inline** 변환 (#014)
  - drone_drop_env.py: PX4_SIM_MODEL → **PX4_GZ_MODEL_NAME** (pre-spawn 연결)
  - drone_drop_env.py: world SDF에 드론 **pre-spawn inject** 로직 추가
  - drone_drop_env.py: Gazebo **paused 시작 → PX4 연결 후 unpause**
  - world SDF payload_0 z: 0.36(테스트 잔여) → **0.14** 원복

**추가 변경 (manual drop exploit 발견)**:
  - manual drop 비활성 (action[4] 무시)
  - NEW `reward.random_drop_start_step` = **150**
  - NEW `reward.random_drop_prob` = **0.005** (0.5%/step)
  - `reward.auto_drop_threshold`: 2.0 → **3.0**
  - `reward.manual_drop_threshold`, `reward.min_drop_step` 제거
  - train_sac.py: total_drop_manual → **total_drop_random**

**이전 시도**:
  vf5dgx55~r5zp3k1c — payload 미분리 (#013/#014)
  v6i3vy4p — manual drop "즉시 drop" exploit 발견 → random drop으로 전환
**추가 변경 (검증 방법 전환)**:
  - train_sac.py: DropEpisodeRecorderCallback에 **best drop 모델 저장** 추가
    auto drop 성공 시 best_drop_error 갱신되면 모델 .zip 저장
    저장 위치: drop_episodes/best_drops/best_err{X}m_step{Y}.zip
  - action replay(open loop) 폐기 → deterministic evaluate(closed loop)로 검증
    Issue #015: replay는 원래 궤적 재현 불가 (420 step 뒤 30m 차이)

**인프라 정리**:
  - 폴더 통합: local_dronebombard_simulation/ → Drone-Bombard-Simulation/local/
  - 디스크 정리: PX4 로그 33G + /tmp 9.5G + 빌드캐시 1.2G = 44G 회수
  - training guide에 PX4 로그 삭제 루틴 추가

**현재**: run ruozrv5x 150k 완주. random drop + auto drop + 실제 물리 payload 낙하.
  문서 위치: /home/juns/Drone-Bombard-Simulation/local/

---

### #16. 2026-05-26 — **Round 2: gradient 완만화 (접근+drop 보상 개선)**

WandB run: `(예정)`
**Base**: #15 (Round 1 — run ruozrv5x 150k 완주)

**Round 1 분석 결과**:
  432 drops, best 4.64m, avg 14.02m, success 1건 (0.2%)
  d_xy 최소값 평균 11.3m — auto_drop 3m 도달 0건
  10m 구간 drop 보상이 2.2점으로 noise에 묻혀 학습 신호 부재

**변경**:
- `reward.w_dist`: 0.5 → **1.0**
  - **이유**: 접근 보상 2배. 드론이 9~11m 정체 구간을 돌파하도록.
- `reward.k2_precision`: 0.5 → **0.2**
  - **이유**: precision 감쇠 완만하게. 10m에서 0.7→13.5 (학습 가능한 신호).
- `reward.k_drop_proximity`: 0.3 → **0.15**
  - **이유**: proximity 감쇠 완만하게. 10m에서 1.5→6.7.

**추가 변경 (Round 2 첫 시도 dbi74uif 분석 후)**:
- `environment.max_steps`: 500 → **800**
  - **이유**: Round 1 best가 420 step에 4.6m. 500은 접근 시간 부족.
- `reward.random_drop_start_step`: 150 → **600**
  - **이유**: step 150은 너무 이른 시점에 random drop 발동. 600 step 자유 접근 확보.
- NEW `environment.max_altitude` = **25.0**
  - **이유**: 고도 상한. 드론이 무한정 상승하며 시간 낭비 방지.
- NEW `reward.penalty_max_altitude` = **-30.0**
- NEW `environment.stagnation_window` = **200**
- NEW `environment.stagnation_min_progress` = **2.0**
- NEW `environment.stagnation_start_step` = **200**
  - **이유**: 200 step 동안 d_xy 2m 미감소 시 정체 판단. step 200 이후 체크.
- NEW `reward.penalty_stagnation` = **-15.0**
- WandB metric 22개 → **8개** 정리 (이름 변경 포함)
  - 상세: guides/wandb_metrics_guide.txt

**유지**:
- w_drop_base=100, drop_attempt_bonus=30, auto_drop_threshold=3.0
- random_drop_prob=0.005

**변경 전후 비교 (proximity + precision)**:
  15m:  0.4 →  8.2
  10m:  2.2 → 20.2
   8m:  4.5 → 29.2
   5m: 14.9 → 51.0
   3m: 34.5 → 74.0
   0m:  130 →  130

**Round 2 첫 시도 결과 (run dbi74uif, 150k)**:
  deterministic eval: d_xy 14~16m, 모두 random drop, 접근 실패
  원인: random_drop_start_step=150 → step 200에서 조기 drop → 접근 시간 부족

**결과**: (최종 학습 예정)

**결론 / 다음**: gradient 완만화 + 접근 시간 확보(max_steps 800, start_step 600)
  + 이탈 조기 차단(max_altitude, stagnation)으로 학습 효율 강화.

**추가 변경 (2026-05-30, 학습 진화 중)**:
- `environment.max_altitude` 제거 → drop 시점 고도 페널티로 대체
- NEW `reward.altitude_drop_threshold` = 15.0, `altitude_drop_w` = 1.0, `altitude_drop_k` = 0.15
  - **이유**: 비행 중 고도 강제할 필요 없음. drop 시 너무 높으면 수평 drift 큼.
- `environment.stagnation_*` 제거 → speed_gate가 이미 hover 보상 차단
- `truncate_reason 8종 wandb 카운터 추가` → 원인 추적
- `reset() pos_enu 초기화 추가` → n_steps=1 가짜 success 버그 fix

**Round 2 최종 결과 (run z05fx7g9, 150k)**:
  총 drop: 427건, 평균 19.09m, 중앙값 11.55m
  Best: 2.53m (Round 1 4.64m 대비 개선)
  Success (<5m): 16건 (Round 1 1건 대비 16배 증가)
  Deterministic eval: 3 epi 전부 crash, drop 0건

**발견된 문제**:
  1) Reset 버그 (n_steps=1 가짜 success 13건) — 해결
  2) Post-success regression — Issue #016, Round 3 처방

---

### #17. 2026-05-30 — **Round 3 (조합 C): PER + LR 1e-4 + Tau 0.002**

WandB run: `(예정)`
**Base**: #16 (Round 2 — z05fx7g9 결과 분석)

**문제 식별**:
  Issue #016 Post-success regression — real success 직후 5 epi 평균 19.72m,
  전체 평균 19.09m보다 나쁨. SAC sparse reward의 알려진 패턴.
  4가지 메커니즘:
    a) Critic overshoot
    b) Policy gradient over-correction
    c) Replay buffer 압도 (success 1/427)
    d) High entropy exploration

**변경**:
- `sac.learning_rate`: 3.0e-4 → **1.0e-4**
  - **이유**: 모든 update 점진적, critic overshoot 직접 완화
- `sac.tau`: 0.005 → **0.002**
  - **이유**: Target network 안정화, Q-value 발산 차단
- `Replay Buffer`: 표준 → **PrioritizedReplayBuffer (PER)**
  - alpha=0.6 (priority 강도)
  - beta=0.4 → 1.0 (importance sampling, schedule)
  - **이유**: TD-error 큰 success transition 자주 샘플링

**유지**:
  - 보상 구조 (Round 2)
  - max_steps 800, random_drop_start 600
  - Drop 시점 고도 페널티

**Pre-flight**:
  - Round 2 결과 백업 (archive/round2_z05fx7g9_2026-05-30/)
  - Reset 버그 fix 코드 베이스 사용

**Round 3 첫 시도 발산 (run q13hli0y, 30k 중단)**:
  step 17731 단일 epi reward -6.77e+9 발생 (≈ 드론 165m 고도에서 drop)
  원인: 지수 고도 페널티 무한대 폭주
  PER이 천문학적 priority 부여 → buffer 오염 → 학습 망가짐
  archive/round3_q13hli0y_FAILED_2026-05-30/ 보존

**Round 3 수정 (4가지 안전장치)**:
- `reward.alt_penalty_*` (NEW): Sigmoid 페널티 (지수 폐기)
  - alt_penalty_max=50.0, alt_penalty_mid=30.0, alt_penalty_k=0.15
  - penalty = -max * sigmoid(k * (alt - mid))
  - 출력 (-50, 0) 유계 → 폭주 불가
  - 거리별: 15m→-4.7, 20m→-11.7, 30m→-25, 50m→-47.5, 100m+→-50
  - **이유**: 음수 지수 4곳(proximity/precision/prediction/impact)은 본질
            적 유계라 안전. 양수 지수(고도)만 폭주 위험 → sigmoid로 교체.
- `environment.max_altitude` = 50.0 (NEW, 부활):
  - 비행 중 50m 초과 시 truncate
  - **이유**: 드론이 무한정 상승 차단 (sigmoid 페널티 외 추가 방어층)
- `reward.penalty_max_altitude` = -15.0 (NEW): timeout과 동일 페널티
  - **이유**: 0 페널티면 "climb-out" exploit 위험. -15는 가벼운 견제.
- `sac.per_priority_max` = 30.0 (NEW): PER priority 상한
  - priority = min(30, (|reward|+eps)^alpha)
  - **이유**: 이론 max reward 200 → priority 200^0.6 ≈ 24. cap 30은 충분.
- Hard cap [-200, +300] + warning (drone_drop_env.py step 끝):
  - reward 범위 밖이면 print warning + np.clip
  - **이유**: 방어적 안전망. 정상 학습엔 영향 없음. 발생 시 즉시 알림.

**유지**:
  - learning_rate=1e-4, tau=0.002, PER alpha=0.6 eps=0.1
  - 보상 구조 (Round 2)
  - max_steps 800, random_drop_start 600

**결과 (run lidq3ydu, 157k 크래시)**:
  - 총 drop 104건, 평균 22.42m, Best 4.32m, Success 8건 (7.7%)
  - PER + LR/Tau 효과: success 빈도 2배 가속 (Round 2: 16/150k vs Round 3: 8/100k)
  - Best episodes (107k, 108k, 124k): 장시간 비행 + 정밀 drop, reward 499~550
  - 100~125k 최우수 (avg 13.9m, success 3건)
  - Post-success regression 여전 (125~150k avg 35m)
  - Hover exploit 발생 (25~50k 45%, 100~125k 36%)
  - 4가지 안전장치 작동 (Hard cap 발동 없음, [WARN] 없음)

**크래시 원인**:
  - PX4 로그 20GB 누적 (.ulg 파일 2300+개)
  - Gazebo 응답 지연 → gz model --list 5초 timeout
  - reset() 중 예외 → 학습 abort
  - 마지막 체크포인트 95k (이후 60k 손실)

**결론 / 다음**:
  Round 3 PER + LR/Tau 처방 부분 검증 (success 2배 속도).
  Hover exploit 새로운 문제 식별 → Issue #017, Round 4 처방.
  PX4 로깅 비활성화로 인프라 안정성 확보.

---

### #18. 2026-05-31 — **Round 4 (A+C): Hover 차단**

WandB run: `vo1l9wl6` (학습 중)
**Base**: #17 (Round 3 — lidq3ydu 분석)

**문제 식별 (Issue #017)**:
  drop_error 13~16m 구간에 16건 (15.4%)
  14.87m = sqrt(11² + 10²) = spawn→target 거리 정확 매칭
  n_steps 600+ → 스폰에서 random_drop 대기 패턴
  원인: w_heading 0.7 × 600 step = +420 → hover가 안전한 수익원

**변경 (A + C 조합)**:
- `reward.w_heading`: 0.7 → **0.3** (Issue #017 A)
  - **이유**: hover의 주요 수입원 약화 (수익 57% 감소)
- NEW `reward.w_distance_penalty` = **0.03** (Issue #017 C)
  - **이유**: per-step penalty = -w * d_xy / 50
  - d_xy=15m → -0.009/step, 600 step → -5.4
  - 멀리 있을수록 비용 → hover 차단

**유지** (Round 3):
  - PER (alpha=0.6, eps=0.1, priority_max=30)
  - learning_rate=1e-4, tau=0.002
  - 4가지 안전장치 (sigmoid alt, max_alt truncate, PER cap, hard cap)
  - 보상 구조 (w_dist 1.0, k2 0.2, k_prox 0.15)
  - max_steps 800, random_drop_start 600

**인프라 변경**:
- PX4 로깅 비활성화: SDLOG_MODE 1 → -1
  - `/opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS`
  - **이유**: .ulg 누적 20GB → Gazebo timeout 크래시
  - 우리 RL 학습엔 .ulg 사용 안 함 → 손실 없음

**시뮬레이션 (300 step 예시)**:
  Hover 600 step at d_xy=15m: +183 (기존 +422, -57%)
  타겟 접근 400 step (15→5m): +179
  Success 500 step (15→3m): +232
  → success > hover ≈ 접근

**Round 4 결과**:
  vo1l9wl6 (14k 중단): reset 버그 부작용 — pos_enu z=0 → ground_contact 트리거
    Fix: drone_drop_env.py reset()에서 pos_enu = (0, 0, 5.0)
  4j46qwpk (146k 발산):
    - ent_coef 6.03 (정상 0.3~0.5의 12배)
    - critic_loss 230,000+ (정상 100~500의 500배)
    - ep_rew_mean -20, ep_len 감소 추세
    - 85 drops, 평균 25m, success 6건 (Round 3 8건/100k와 비슷)
    - 56~70k 구간 발산 시작 — 큰 양수(+187,+274) + 큰 음수(-184,-256) 교차

**발산 원인 분석 (정확한 메커니즘)**:
  1. w_heading 0.7→0.3 + distance_penalty 0.03 → per-step 보상 magnitude 감소
  2. Per-step / drop reward 비율: Round 3 1:300 → Round 4 1:700 (2배 sparse)
  3. Critic estimate variance 폭증
  4. Policy 집중 (한 가지 전략) → entropy ↓
  5. SAC auto-tuning "exploration 부족" 판단 → ent_coef ↑
  6. Bounded action space [-1,1] → entropy 못 올라감
  7. SAC 더 ent_coef ↑ → 양성 피드백 → 발산

**결론 / 다음**:
  Per-step 보상 density 변경은 SAC 발산 위험.
  Hover 차단은 다른 방법 필요 — Episode 종료 페널티로 시도 (Round 5).
  archive/round4_4j46qwpk_diverged_2026-05-31/

---

### #19. 2026-05-31 — **Round 5: Hover Terminal Penalty**

WandB run: `sdjytkpv` (학습 중)
**Base**: #18 (Round 4 발산 분석)

**문제 식별**:
  Round 4의 per-step hover 차단이 SAC auto-entropy 발산 트리거.
  Per-step 보상 density 보존하면서 hover 차단 필요.

**복원 (Round 4 변경 전면 되돌림)**:
- `reward.w_heading`: 0.3 → **0.7** (Round 3 수준)
- `reward.w_distance_penalty`: 0.03 → **0** (비활성)

**신규 (Episode 종료 페널티)**:
- NEW `reward.hover_speed_threshold` = **1.0** m/s
  - **이유**: 정지 판단 기준. 정상 비행 시 2~5 m/s 유지.
- NEW `reward.hover_consecutive_threshold` = **200** step
  - **이유**: episode max 800의 25% — 잠시 hover OK, 지속만 BAD
- NEW `reward.penalty_hover` = **-15.0**
  - **이유**: timeout과 동일 — 적당한 견제 (-50은 과함)

**메커니즘**:
  ```python
  # 매 step:
  speed_xy = sqrt(vx² + vy²)
  if speed_xy < 1.0:
      consecutive_still += 1
  else:
      consecutive_still = 0
  max_consecutive_still = max(...)

  # Episode 종료 시 (truncated AND not dropped):
  if max_consecutive_still > 200:
      reward -= 15
  ```

**핵심 장점**:
  - Per-step 보상 density 변화 없음 → SAC 안정성 유지
  - 1회 terminal 페널티 → 발산 모드 회피
  - Drop 시 제외 → 정밀 hover 정당화
  - 잠시 hover (50 step) OK → 학습 초반에도 무력화 안 함

**유지** (Round 3+4 안전장치):
  - PER (alpha=0.6, eps=0.1, priority_max=30)
  - learning_rate=1e-4, tau=0.002
  - Sigmoid alt penalty, max_alt truncate, hard cap
  - PX4 로깅 비활성

**Round 5 결과**:
  sdjytkpv (65k pause): ent_coef 1.30, ep_rew +13, success 12.5%
    - 표면적으로 좋아 보였으나 ent_coef는 단조 증가 중
  mnlr1zpe (resume from 65k, 148k 발산):
    - ent_coef 6.16 (Round 4와 동일 수준)
    - critic_loss 5,000+, ep_rew -37.9, success 12.5% → 5%
    - episode 짧아졌다 길어졌다 oscillation은 SAC 발산 중에도 발생

**핵심 발견**:
  Round 4 (per-step 처방)와 Round 5 (terminal 처방) 모두 동일 발산 패턴
  → Hover 처방 형태 무관
  → SAC + PER + sparse reward 환경 자체의 발산 모드 (Issue #019)
  archive/round5_diverged_2026-05-31/

**결론 / 다음**:
  Hover 처방 더 시도해도 SAC entropy 발산 우회 불가.
  SAC auto-entropy 자체를 통제 → Round 6 (DampedEntropySAC)

---

### #20. 2026-05-31 — **Round 6: DampedEntropySAC** (SAC entropy 발산 근본 처방)

WandB run: `bfv4la9a` (학습 중)
**Base**: #19 (Round 5 발산)

**문제 식별 (Issue #019)**:
  SAC + PER + bounded action space + sparse reward 환경:
  1. Policy 한 전략 집중 → log_prob ↑
  2. SAC: "exploration 부족" → ent_coef ↑
  3. Action clipped → 실제 entropy 못 늘어남
  4. ent_coef 무한 추격 → 발산

  사용자 통찰: "policy가 한 전략에 집중할수록 alpha 증가치를 줄이는"
  → SAC가 자연스러운 deterministic 수렴 막지 않도록

**변경 (코드, NEW DampedEntropySAC 클래스)**:
- `train_sac.py`: `class DampedEntropySAC(SAC)` 추가
  - Soft damping in train():
    ```python
    concentration = max(0, log_prob_mean + target_entropy)
    damping = ent_damping_threshold / (ent_damping_threshold + concentration)
    ent_coef_loss *= damping
    ```
  - Hard cap after optimizer step:
    ```python
    self.log_ent_coef.clamp_(max=log(ent_coef_hard_cap))
    ```
  - 새 metric: `train/ent_damping` (현재 damping factor 추적)
- SAC 대신 DampedEntropySAC 사용 (fresh + resume 모두)

**yaml 추가**:
- NEW `sac.ent_damping_threshold` = **5.0**
  - **이유**: concentration이 5일 때 damping=0.5 (절반 속도)
- NEW `sac.ent_coef_hard_cap` = **2.0**
  - **이유**: 절대 상한. log(2.0)으로 log_ent_coef clamp.

**유지** (Round 3+5):
  - PER (alpha=0.6, eps=0.1, priority_max=30)
  - learning_rate=1e-4, tau=0.002
  - 보상 구조 (w_heading 0.7, w_dist 1.0, k2 0.2, k_prox 0.15)
  - Hover terminal penalty (200 step, -15)
  - max_steps 800, random_drop_start 600
  - 4가지 안전장치 (sigmoid alt, max_alt truncate, PER cap, hard cap)
  - PX4 로깅 비활성

**예상 효과**:
  log_prob | concentration | damping | alpha 변화
  -5 (정상)| 0             | 1.00    | 정상 (정상 학습)
  0        | 5             | 0.50    | 절반 (둔화 시작)
  5        | 10            | 0.33    | 1/3 (더 둔화)
  10       | 15            | 0.25    | 1/4
  + 모든 경우 ent_coef ≤ 2.0 강제

  자연 수렴 허용 + 폭주 차단.

**Round 6 v1 결과 (run bfv4la9a, 162k 중단)**:
  Mean 기반 damping이 작동 안 함:
    - batch 256개 중 일부만 극단 집중
    - mean()이 5 미만이라 concentration = 0
    - ent_damping이 1.0 유지
    - 그러나 outlier가 큰 gradient → ent_coef 단조 증가
    - 1.0 → 1.46 → 1.68 (hard cap 2.0 직전)
  Hard cap이 막아주긴 함 — 학습 정체 (ep_rew -111 유지)
  또한 체크포인트 95k 이후 저장 안 됨 (Issue #020 발견)

**결론 / 다음**:
  Mean 기반 damping → Percentile 기반으로 수정 필요
  체크포인트 정렬 버그도 fix

---

### #21. 2026-05-31 — **Round 6 v2: Percentile Damping + Checkpoint fix**

WandB run: `6b8bslmz` (학습 중)
**Base**: #20 (Round 6 v1) — 95k 체크포인트에서 resume

**변경 (코드)**:
- `train_sac.py` DampedEntropySAC.train():
  - mean() → percentile(0.95) 사용
    ```python
    log_prob_q95 = th.quantile(log_prob.detach(), 0.95).item()
    concentration = max(0, log_prob_q95 - (-target_entropy))
    ```
  - **이유**: mean()은 outlier 못 감지 (PER 환경에서 특히)
- `train_sac.py` CleanupOldCheckpointsCallback (Issue #020):
  - 알파벳순 → step 번호 기반 정렬
    ```python
    files = sorted(files, key=lambda p: int(...step_num...))
    ```
  - **이유**: '100000'이 '95000'보다 알파벳상 앞 → 새 체크포인트 즉시 삭제 버그

**즉시 검증 (resume 직후)**:
  ent_coef: 1.08 (정상 범위)
  ent_damping: **0.527** (이전 1.0 → 작동 확인!)
  → log_prob 상위 5%가 집중 → damping 작동 → alpha 증가 둔화

**유지** (Round 6 v1):
  - DampedEntropySAC 클래스
  - ent_coef_hard_cap = 2.0
  - ent_damping_threshold = 5.0
  - PER, LR/Tau, hover terminal penalty, 4가지 안전장치, PX4 로깅 비활성

**결과**: (학습 중 — 95k에서 시작, 300k 목표)

**Round 6 v2 추가 변경 (학습 중)**:
- NEW: `SuccessReplay 시스템` (best_drops 대체)
  - 조건: `is_success` (drop_error ≤ 5m) AND `drop_trigger == 'auto'`
  - 이전 `best_drops`의 "best 갱신 시만" 조건 제거 — 모든 정밀 drop 보관
  - 저장 위치: `/workspace/ros2_ws/success_replay/{wandb_run_id}/`
    - host에 bind-mounted 상태로 접근 (container 비대 X)
    - local symlink: `local/success_replay → ../ros2_ws/success_replay`
  - 파일명: `success_step{N}_err{X.XX}m.zip`
  - **다음 학습부터 적용** (현재 실행 중 학습은 영향 X)
- 코드 변경: DropEpisodeRecorderCallback 내부 best_drops 로직 → success_replay 로직

**Round 6 v2 최종 결과 (294k에서 컨테이너 OOM 중단)**:
  학습 추이:
    95k~120k:  32 drops, avg 24m, success 1
    120k~145k: 34 drops, avg 35m
    145k~170k: 27 drops, avg 23m, success 2 (160k에 best 4.36m!)
    170k~195k: 26 drops, avg 22m (정점, ep_rew 양수)
    195k~220k: 12 drops → 학습 발산
    245k~270k: 10 drops, avg 53m (심각)
    270k+: critic_loss 14M → OOM
  자료:
    - Hard cap 2.0이 ent_coef 잡았으나 그 값에서 학습 망가짐
    - 160k 부근 best 모델 (best_err4.36m_step160625) 살아있음
    - 100k/150k/200k/250k milestone 살아있음
  archive/round6_v2_oom_2026-06-03/ (예정)

**결론 / 다음**:
  근본 원인 = bounded action + target_entropy=-5 부조화
  Round 7에서 target_entropy=-15로 근본 처방.

---

### #22. 2026-06-03 — **Round 7: target_entropy=-15 근본 처방**

WandB run: `iobwvcrm` (학습 중)
**Base**: #21 (Round 6 v2 OOM)

**근본 원인 분석 (Issue #019 v2)**:
  SAC 자동 entropy 튜닝 그라디언트:
    d_loss/d_log_alpha = -(log_prob + target_entropy)
    log_prob > -target_entropy 이면 alpha ↑

  Bounded action space (tanh squash):
    log_prob에 Jacobian 보정 자연스럽게 큰 양수
    log_prob > 5는 정상 학습 시에도 흔함
    target_entropy = -5 (default)면 → alpha 항상 ↑ 압력
    → hard cap에 영원히 갇힘 (Round 4/5/6 v1/6 v2 공통)

  해결책: target_entropy = -15
    log_prob > 15는 매우 극단적 (거의 발생 X)
    대부분 alpha ↓ 방향으로 gradient
    cap에 닿아도 자기 복구 가능

**변경**:
- `sac.target_entropy`: 'auto' (-5) → **-15.0**
  - **이유**: bounded action space 자연 entropy 범위 감안
  - default(-|A|)는 unbounded Gaussian 가정. squash 환경에 부적합.
- `sac.ent_coef_hard_cap`: 2.0 → **1.0**
  - **이유**: 2.0은 학습 불가능 수준 (entropy 보너스 7 >> reward 0.7)
  - 1.0이면 entropy 보너스 3.5로 학습 가능
  - 안전망으로서 더 효과적
- `training.total_timesteps`: 300000 → **150000**
  - **이유**: target_entropy fix 우선 검증. 잘 되면 resume으로 확장 가능.

**유지** (Round 6 v2):
  - DampedEntropySAC + percentile damping
  - PER (alpha=0.6, eps=0.1, priority_max=30)
  - learning_rate=1e-4, tau=0.002
  - SuccessReplay 시스템
  - 보상 구조 (Round 5 hover terminal penalty 포함)
  - 4가지 안전장치

**예상 효과**:
  Round 6 v2 vs Round 7:
    Round 6 v2: ent_coef 1.0→1.5→2.0 (cap 갇힘) → critic 14M
    Round 7:    ent_coef 0.3~0.5 안정 유지 (cap 안 닿음)

  자기 복구 가능성:
    이전: cap 도달 후 그 값에서 영원히 (target_entropy 압력)
    Round 7: cap 도달해도 다음 step에 자연 감소 (target_entropy fix)

**결과**: 14.9k 에서 #021 gz timeout crash (SAC 처방은 정상 작동).
  - ent_coef 안정 (0.29) ← target_entropy=-15 처방 검증됨
  - ep_rew_mean -33 → -25 (학습 진행)
  - 종료: gz model --list TimeoutExpired (Round 3, Round 6 v2 와 동일 버그)

---

### #23. 2026-06-03 — **Round 7 diag run + resilient v1 (#021 처방)**

WandB run: `9qocfk9y` (진단), `y6mxu5q2` (resilient v1)
**Base**: #22 (target_entropy=-15) + 진단 코드 + 1·2차 처방

**추가** (NEW):
- `env.max_consecutive_fast_resets` = **100**
  - **이유**: #021 gz timeout 누적 가설 차단.
  - drop 없이 fast-path teleport reset 이 100회 누적되면 강제 _kill_infra/_start_infra.
  - 100 episode 마다 ~30s 추가 → ~4% slowdown.
  - 0 = 비활성화 (기존 동작과 동일).
- 코드 측: drone_drop_env.py `_check_infra_healthy` 의 TimeoutExpired catch (1차 처방).
  - gz model --list 가 5s timeout 시 unhealthy 처리 → full restart 자동 진입.
- 코드 측: train_sac.py `InfraHealthMonitorCallback` (200 step 간격).
  - 진단 영구화. 다음 #021 발생 시 postmortem 데이터 자동 수집.
  - infra/gz_list_ms, gz_rss_mb, px4_rss_mb, reset_pre_v, post_cruise_v 등.

**진단 run 결과 (9qocfk9y, 39k 수동 SIGINT 중단)**:
  - **success 2건** (step 14995 err 3.63m, step 19807 err 3.82m)
    ← target_entropy=-15 정책 검증, SuccessReplay 첫 작동
  - 진단 데이터:
    - gz_list_ms ~470ms baseline (정상, degradation 신호 약함)
    - gz_rss 196MB 안정 (메모리 leak 신호 약함)
    - post_v 대체로 작음 (velocity 누적 가설 약함)
    - reset 16~390 범위에서 추세 없음
  - 결론: 누적 가설들 신호 미약. #021 는 trigger 못 됨 (intermittent 재확인).

**Resilient v1 시작 (y6mxu5q2)**:
  - success_step19807 에서 resume + 1·2차 처방 활성
  - 다음 crash 시 진단 100 sample 으로 가설 검증 자동

**결론 / 다음**:
  - SAC #019 처방 → 검증됨 (39k 진단 run)
  - 인프라 #021 처방 → 적용. 효과 검증은 다음 학습 또는 다음 crash 시.

---

### #24. 2026-06-04 — **Round 7 v3 critic-stable (Huber + target_q_clip + per-sample damping)**

WandB run: `436xl0bb` (round7_v3_critic_stable)
**Base**: #23 — preempt 에서 resume (385k → 685k)

**근본 원인 분석 (Round 7 v2 의 cap 도달)**:
  - critic_loss 가 68 → 17,100 (250x) 으로 첫 점프 → 이후 millions 까지 폭주
  - 폭주 → bootstrap target Q 값 inflation → 정책 행동 비정상 → log_prob 거대화
  - SAC: log_prob 크면 alpha ↑ 해석 → cap 도달
  - **critic 폭주가 entropy 발산의 원인** (반대 아님)

**변경**:
- 코드: `DampedEntropySAC.train()` 의 critic loss
  - **MSE → Huber loss (smooth_l1)**: gradient saturation 으로 outlier impact 차단
  - **target_q_values.clamp(±target_q_clip)**: bootstrap inflation 차단
- `sac.target_q_clip` = 500.0 (신규)
  - **이유**: reward 스케일 ±200 고려 ±500 으로 충분히 넓고 폭주 차단
- 코드: per-sample damping (q95 → element-wise)
  - **이유**: q95 단일 scalar 는 batch 전체 동일 damping → outlier 만 강하게 damped 가 더 정확

**유지**: target_entropy=-15, ent_coef_hard_cap=1.0, max_consecutive_fast_resets=100, 등 모두

**결과** (685k 자연 종료):
  - **best drop 1.32m** (이전 4.36m 갱신)
  - 6,055 episodes / 162 drops / 87 auto / **16 successes** / 0 jackpots
  - ent_coef: 1.0 cap (resume 시작 시) → 0.055 회복 ← **per-sample damping 효과 입증**
  - critic_loss: 17,800 (resume 시작) → ~35-40 stable ← **Huber + clip 효과 입증**
  - Forced restart 29회 정상 (1·2차 처방 작동)
  - #021 gz timeout crash 0회
  - fps 5 (cumulative since resume) — 짧은 episode + reset overhead 누적

**결론 / 다음**:
  - SAC + 인프라 처방 모두 검증 완료
  - Phase 1 마감 → 백업 (`local/backups/phase1_final_round7_v3/`, 609 MB)
  - Phase 2 검토 → Phase 1 redux 결정 (target relocation + random_drop=0)

---

### #25. 2026-06-05 — **kill_episode timeout 5s → 2s (Phase 2 speedup)**

WandB run: (Phase 1 redux 부터 적용)
**Base**: #24

**배경**:
  - Phase 1 종료 후 fps 5 분석 → 짧은 episode + reset overhead 가 원인
  - `_kill_episode` 의 `proc.wait(timeout=5)` 가 누적 비용
  - 50 step episode 의 경우 7.5s 중 5s 가 reset (66% 오버헤드)

**변경**:
- 코드: `drone_drop_env.py:_kill_episode` (line 1540)
  - `proc.wait(timeout=5)` → `proc.wait(timeout=2)`
  - **이유**: SIGTERM 응답 정상이면 1s 안에 종료. 2s 도 충분. 안 끝나면 SIGKILL escalation.
- `_kill_infra` 의 timeout=5 는 유지 (drop/forced restart 시만 호출, 영향 작음)

**예상 효과**:
  - 짧은 episode 케이스: ~50% reset 시간 감소 → fps 2-3배 증가
  - 정상 episode 도 약간 가속

---

### #26. 2026-06-05 — **Phase 1 redux: target (4,3) + random_drop=0**

WandB run: `phase1_redux_target43_norandomdrop` (진행 중, fresh start)
**Base**: #24 + #25

**배경 (Phase 1 종료 후 eval 진단)**:
  - 1.32m success snapshot 모델 5 EP eval: 0 drops, 모두 crash
  - milestone_600000 + A2 hybrid: 1 random drop / 5 EP
  - 결론: 정책이 random_drop 보조 없이 reliable auto_drop 못함
  - 14.87m 거리는 학습 어려움 (auto_drop_threshold 3m 도달 못함)

**변경**:
- `environment.target_enu_x`: 11.0 → **4.0**
- `environment.target_enu_y`: 10.0 → **3.0**
  - **이유**: spawn-to-target 14.87m → 5m, 정책이 auto_drop 학습 가능한 범위
- `reward.random_drop_prob`: 0.005 → **0.0**
  - **이유**: 정책 자체 drop 학습 강제. 학습 시그널은 적어도 진짜 정책 능력 측정 가능
- `wandb.run_name`: "phase1_redux_target43_norandomdrop"
- Gazebo world SDF: `x_marker_0` pose (11,10,0) → **(4,3,0)**
- 코드 fix: drop_calculator launch 의 `x_marker_x:=11.0` hardcoded → `:=cfg_target_x`
  - **이유**: 버그. 이전엔 환경 변경 시 drop_calculator 만 옛 값 보유 가능성

**유지**: target_entropy=-15, target_q_clip=500, ent_coef_hard_cap=1.0, per_alpha=0.6, kill_episode timeout=2s, 모든 처방

**Fresh start 이유**: 보상 분포 자체 변경 (target 위치 + random_drop). Replay buffer 의 모든 transitions 이 옛 값 기준 → resume 부적합.

**초기 결과** (step 1593):
  - fps 18 (이전 5 대비 **3.6배 가속**)
  - ep_len_mean 398 step (이전 50 step 대비 **8배 ↑**)
  - critic_loss 1.03 (매우 안정)
  - ent_coef 0.943 (fresh start default)
  - 예상 학습 시간: 약 4-5시간 (300k step)

**검증 가설**:
  1. closer target 으로 정책이 auto_drop 학습 가능?
  2. random_drop 없이 success_rate 의미 있게 증가?
  3. drop_error 분포가 bimodal 에서 unimodal 로?

**결론 / 다음**: 자연 종료 후 deterministic eval + stochastic eval 비교

---

### #27. 2026-06-05 — **Phase 1 redux v1 중간 결과 + v2 진입 (tight thresholds)**

WandB run: `ayi27a56` (v1, 89k 중단), `za9zxdh6` (v2, 진행 중)
**Base**: #26 (Phase 1 redux v1)

**Phase 1 redux v1 중간 결과 (89k 수동 중단)**:
  - 1,205 episodes, **830 drops** (모두 auto, random_drop=0 효과)
  - **799 successes (96.3% — 5m 기준)** ← 강력 학습 입증
  - best drop **0.809m** (Round 7 v3 의 1.32m 갱신)
  - drop_error 분포:
    < 5m: 96.2%, < 3m: 0.5%, < 2m: 0.1%, < 1m: 0.1%, < 0.3m: 0%
  - jackpots 0 (jackpot_threshold 0.1m 너무 빡빡)
  - 정책 deterministic 화: ent_coef 0.001 수준 (이전 0.055 대비 50배 작음)
  - critic_loss 3-7 안정
  - 학습 초기 fps 18, 중후반 drop 빈번 → _kill_infra 자주 호출 → fps 2 폭락
  - 예상 잔여 30시간 → 충분히 학습 입증 → 수동 중단

**교훈**:
  1. closer target (5m) + random_drop=0 조합 매우 효과적
  2. _kill_infra timeout 이 drop 빈번 시 fps 폭락 원인 (이전 분석 검증)
  3. 5m success 임계는 너무 헐거움 → 정밀화 단계 진입 필요
  4. jackpot 0.1m 는 도달 불가 → 늘려야 발화 가능

**Phase 1 redux v2 처방 (curriculum learning)**:

- `reward.auto_drop_threshold`: 3.0 → **1.0m**
  - **이유**: 정책이 더 정밀하게 접근해야 trigger. 0.809m 달성 가능 입증됨.
- `reward.success_threshold`: 5.0 → **1.0m**
  - **이유**: success 정의 자체를 정밀화. 96% (5m) → 새 기준 0.1% 예상.
- `reward.jackpot_threshold`: 0.1 → **0.3m**
  - **이유**: 0.1m 는 도달 불가능 (이전 0건). 0.3m 도 도전적이지만 best 0.809m 에서 추가 학습으로 가능.
- 코드: `drone_drop_env.py:_kill_infra` 의 `proc.wait(timeout=5)` → 2s
  - **이유**: drop 빈도 높을 때 fps 폭락의 주 원인. _kill_episode 와 일치.
- `wandb.run_name`: "phase1_redux_v2_tight_thresholds"

**유지**:
  - target_enu (4, 3), random_drop_prob 0.0
  - target_entropy=-15, target_q_clip=500, ent_coef_hard_cap=1.0, per_alpha=0.6
  - 모든 Phase 1 처방

**Resume vs Fresh 결정**:
  - **Resume from 89k preempt** (curriculum learning)
  - 이전 96% (5m) 정책 = 접근 능력 검증됨, 정밀화만 추가 학습
  - Replay buffer 의 stale reward 영향 작음 (100k step 후 자연 refresh)
  - 89k 학습 시간 보존

**목표 metrics**:
  - 새 1m 기준 success_rate > 50%
  - 첫 jackpot 발화 (drop_error < 0.3m)
  - best drop < 0.3m
  - fps 5-10 회복 (kill_infra timeout 효과)

**결론 / 다음**: 143k 에서 실패 → v3 로 전환 (Entry #31 참조)

---

### #28. 2026-06-05 — **current_success_streak metric 추가 (Callback only)**

WandB run: (다음 학습부터 적용)
**Base**: #27

**배경 — 사용자 요청 평가 도구**:
  - 학습 전체 누적 metric 보다 시점별 (point-in-time) 성능 측정 필요
  - SB3 의 `rollout/success_rate` 이미 rolling-100 (마지막 100 episodes 평균)
  - 추가로 "지금 진행 중인 연속 성공 횟수" 필요 — streak 정보

**연속 성공 가산점 (reward) 검토 — 기각**:
  - SAC 의 Markov 가정 위반: replay buffer 의 random sampling 시 streak 맥락 없음
  - Variance 증가, safe-play 편향, credit assignment 망가짐
  - obs 에 prev_episode_success 명시 추가 시 가능하지만 비용 큼
  - → reward 로는 적용 X. Metric only 로 한정.

**변경 (코드만, env 무수정)**:
- `train_sac.py:WandbMetricsCallback`:
  - `__init__`: `self._current_success_streak = 0`
  - `_on_step` (done block):
    - drop with is_success → streak += 1
    - drop without is_success → streak = 0 (실패로 리셋)
    - no drop in episode → streak = 0 (실패로 리셋)
  - `_on_rollout_end`: `log_dict['env/current_success_streak'] = self._current_success_streak`

**WandB 에서 보임**:
  - `env/current_success_streak` 시계열 chart
  - 0 → 1 → 2 → ... → 0 (실패 시 즉시) → ... 패턴

**적용 범위**:
  - 현재 진행 중인 v2 학습: **❌ 없음** (이미 메모리 로드된 옛 코드)
  - 다음 학습부터: **✓ 자동 적용**

**결론 / 다음**: 평가 도구만 추가. 학습 영향 X. v2 종료 후 다음 학습에서 활용.

---

### #29. 2026-06-05 — **Phase 1 redux v2 실패 분석 + 도구 강화**

WandB run: `za9zxdh6` (v2 failed 143k)
**Base**: #27 (v2 시작)

**v2 실패 결과** (143k 수동 중단):
  - Curriculum gap 너무 컸음: 5m success → 1m, 4× 정밀화 요구
  - 10 drops in 50k step (정상 학습의 1/100)
  - step 123k 이후 17k+ step 동안 0 drops → 정책이 drop 행동 잃음
  - success_rate metric = 0.9 (misleading — ep_info_buffer 의 v1 잔존)
  - fps 8 (예상 잔여 8.8h 효율 낮음)
  - ep_rew_mean -15 ~ -42 (음수, 학습 신호 좋지 않음)

**진단 결과**:
  - 임계 점프 (5m → 1m) 너무 가팔라 정책 적응 못함
  - 또한 pos_scale=50 등 거리 의존 파라미터들이 14m task 설계 그대로 → 5m 환경에 mis-scaled

**도구 강화 적용** (다음 학습부터):
  - DropEpisodeRecorderCallback: index.csv 헤더 + row 에 drop_trigger 컬럼 추가
    (이전: filename,timestep,drop_error_m,is_success,episode_reward,n_steps)
    (이후: 위 + drop_trigger)
  - RepresentativeBestCallback 신규 추가 (train_sac.py):
    * 매 rollout end 마다 REPRESENTATIVE_BEST.json 자동 갱신
    * peak success_rate window 안 top-3 auto+success drops
    * 결정 C: drop < 3 시 not_measurable
  - local/tools/representative_best_analysis.py 신규:
    * 사후 분석 스크립트
    * Round 7 v3 에 적용 → "not_measurable" 결과 보존
    * Round 7 v3 의 best 1.32m 이 isolated lucky drop 임을 입증

**보존**: rl_checkpoints/archive/phase1_redux_v2_failed_143k/
  - sac_drop_preempt.zip + replay.pkl
  - REPRESENTATIVE_BEST.json (실시간 callback 결과)
  - NOTE.md (실패 원인 분석)

**결론 / 다음**: v3 로 전환 — scale 처방 + 임계 완화 + fresh start

---

### #30. 2026-06-05 — **학습 데이터 정리 (~10 GB 절약)**

WandB run: N/A (housekeeping)

**배경**: 누적된 학습 데이터 12.7 GB. 대부분 superseded 라운드.

**삭제** (~10 GB):
  - local/backups/backup_pre_pull_2026-05-22 (4.6 GB)
    : 5월 22일 이전 repo+wandb snapshot, git/cloud 에 있어 redundant
  - ros2_ws/wandb/ 의 옛 runs (3 GB)
    : 모두 클라우드에 sync 됨, 로컬은 캐시
    : 보존만: 현재 진행 중 run
  - ros2_ws/rl_checkpoints/archive/ 옛 라운드 폴더 (340 MB)
    : round1~round6 디렉터리, 모두 Round 7 v3 가 갱신
    : milestone .zip 들 (Phase 1 백업에 final 있음)
  - ros2_ws/success_replay/ 의 옛 run 폴더들 (2.5 GB)
    : 모두 superseded
    : ayi27a56 (Phase 1 redux v1) 의 799 모델 → best 5 만 보존
  - drop_episodes/*.npz 의 옛 trajectory (55 MB)
    : index.csv 는 유지

**유지**:
  - local/backups/phase1_final_round7_v3/ (609 MB) — Phase 1 endpoint
  - rl_checkpoints/archive/phase1_redux_v1_pause_89k/ (86 MB) — v1 preempt
  - rl_checkpoints/archive/phase1_redux_v2_failed_143k/ (86 MB) — v2 실패 case study
  - success_replay/round6_v2_recovered/ (3 MB) — 역사 best
  - success_replay/ayi27a56_best5/ (16 MB) — v1 best 5
  - success_replay/za9zxdh6/ — v2 진행 데이터

**총 변화**: 12.7 GB → 1.9 GB

**결론 / 다음**: 디스크 깔끔. 다음 학습 시작 가능 상태.

---

### #33. 2026-06-07 — **Phase 1 redux v5: SDF dimensions=3 fix — 진짜 root cause**

WandB run: TBD (v5)
**Base**: Fresh start (v3/v4 preempt 안 씀 — 잘못된 reward 위에서 학습됐음)

**v3/v4 의 잘못된 가설 폐기**:
  v3 의 396 drops 의 "α 가설" (velocity inheritance 실패) 은 잘못된 진단.
  진짜 root cause: payload OdometryPublisher 의 `<dimensions>` 미지정
    → gz-sim8 default = 2 (2D mode)
    → payload odom 의 z 좌표 항상 0 publish
    → drop_calculator 가 payload_z ≤ 0.04 시 즉시 impact 처리
    → drop_error = drone 의 분리 시점 horizontal 위치 (= d_xy_at_trigger)
  상세는 issue_023_payload_odom_2d_bug.md 참조.

**검증**:
  - minimal_test mtest2: multi-model DetachableJoint detach 후 child v 보존 ratio = 101.5%
  - 학습 환경 측정: drone alt 4.58m 일 때 payload odom z = 0 (부착 중)
  - SDF fix 후: drone alt 4.06m → payload odom z = 4.053m ✓
  - test_v1 dry-run: payload first sample z=5.33m, 자유낙하 1.0s, transfer 103.1%

**변경 (코드)**:
  payload_{0,1,2,3}/model.sdf 의 OdometryPublisher:
    + <dimensions>3</dimensions>
    * <odom_publish_frequency>10 → 50</odom_publish_frequency>

**Option A 흔적 정리**:
  - drone_drop_env.py: wrench_pub publisher, EntityWrench/Wrench/Vector3/GzEntity 임포트,
    publish_drop 의 wrench publish, bridge config wrench topic, DROP_SKIP_WRENCH toggle 모두 제거
  - worlds/x_marker_world.sdf: ApplyLinkWrench plugin 제거
  - drop_calculator_node.py: /tmp/w_verify.csv 로깅 제거

**Hyperparams 변경**:
  - `w_prediction: 20.0 → 0.0` (fix 후 gap ≈ 0 이라 reward 무의미)
  - `run_name: phase1_redux_v5_sdf_fix`
  - 그 외 v3 hyperparams 유지 (auto_drop_threshold 2.0, random_drop_prob 0, scale 5.0, action 3.0)

**예상 효과**:
  - drop_error 가 진짜 ground impact 측정값
  - CCIP d_impact 가 진짜 prediction 신호
  - gap (drop_error - d_impact) ≈ 0 (air dynamics 작음)
  - 정책이 진짜로 d_impact 최소화 학습 가능

---

### #32. 2026-06-07 — **Phase 1 redux v4: Option A (ApplyLinkWrench) — 폐기 (잘못된 진단)**

⛔ **폐기 — Issue #023 참조.** 가설 자체가 잘못. 진짜 root cause 는 SDF dimensions 누락. 처방의 효과 16% 는 noise. 280k step 까지 학습됐지만 잘못된 reward 위. v5 fresh start 로 교체.

(이하 원본 entry 보존)


WandB run: TBD (v4)
**Base**: #31 (v3) + v3 preempt 254k

**v3 분석 결과 (CCIP gap root cause)**:
  v3 의 396 drops 분석으로 α 가설 (velocity inheritance 실패) 압도적 입증:
    - gap mean = 1.73m (정상 396), std 0.73m
    - correlation(gap, d_xy_trigger - d_impact) = 0.955
    - speed_xy 단독 R² = 0.643
    - 81.6% 의 drops 가 drop_error ≈ d_xy_at_trigger (±0.3m)
    - DetachableJoint 가 분리 시 payload velocity 0 으로 reset
    - payload 가 거의 straight-down fall

**처방 시도 history**:
  옵션 X (CCIP velocity 항 제거): hover-and-drop 만 가능, 학습 목표 위배 → 폐기
  옵션 W (VelocityControl + subprocess gz topic): latency 50-150ms → 실패
  옵션 A (ApplyLinkWrench + ros_gz_bridge + EntityWrench publish):
    - in-process publish (sub-ms latency)
    - 1 physics step impulse force (F = m × v / dt = 25 × v)
    - 자연 free fall 모방

**변경 (코드)**:
  - x_marker_world.sdf:
    + <plugin name="gz::sim::systems::ApplyLinkWrench"
              filename="gz-sim-apply-link-wrench-system"/>
  - drone_drop_env.py:
    + import EntityWrench, GzEntity from ros_gz_interfaces.msg
    + _RLBridgeNode 에 wrench_pub (EntityWrench)
    + Bridge config: /world/x_marker_world/wrench (ROS_TO_GZ)
    + publish_drop:
        wrench = EntityWrench(entity=GzEntity(name='payload_<iid>', type=LINK),
                               wrench=Wrench(force=Vector3(25*vx, 25*vy, 25*vz)))
        wrench_pub.publish(wrench)
        detach_pub.publish(Empty())
  - drop_calculator_node.py: w_verify.csv 에 payload 시계열 저장 (debugging)

**Option A mechanism 검증** (test_v1.py, 정책 무관):
  - mission_manager 를 TRACK state 로 강제 전환 (CRUISE 의 position cmd 비활성)
  - drone state 측정 → wrench publish → payload state 측정
  - 결과:
    drone speed at trigger:   0.48 m/s
    payload first sample:     0.54 m/s
    Velocity transfer ratio:  112.8% ← ✓ 작동
  - Drone 이 비행 명령 안 듣는 별도 문제 있지만 검증과 무관 (any v 측정해서 비교 가능)

**v4 학습 설정** (yaml):
  - run_name: phase1_redux_v4_option_a
  - 그 외 모든 v3 동일 (auto/success threshold 2.0, scale 처방 유지)
  - Resume from v3 preempt 254k
  - 옵션 A 만 다름 (publish_drop 에서 wrench 발생)

**검증 가설** (v3 baseline vs v4):
  - gap (drop_error - d_impact): v3=1.73m → v4 ≈ 0 (velocity 보정 작동)
  - correlation(gap, speed_xy): v3=0.955 → v4 ≈ 0 (gap 이 speed 와 무관해짐)
  - success_rate (2m): v3=1% → v4 가능 증가
  - drop_error 평균: v3=3.59m → v4 < 3.59m (velocity 가 drone-target 거리 보정)

**결론 / 다음**: 30+ drops 후 통계 비교. 효과 확인 시 정식 baseline.

---

### #31. 2026-06-05 — **Phase 1 redux v3: scale 처방 + 임계 완화 + fresh start**

WandB run: TBD (시작 중)
**Base**: #29 (v2 실패) + #28 (도구) + #30 (정리)

**근본 원인 분석 (v1/v2 의 hidden issue)**:
  Phase 1 redux target (4,3) = spawn-to-target 5m.
  그러나 거리 의존 파라미터들은 14.87m task 설계 그대로 유지:
    pos_scale=50 → obs 가 [-0.1, 0.1] 만 활용 → network 신호 약함
    action_vx=8 m/s → 5m 거리 0.625초 만에 통과 (overshoot)
    action_vy=5 m/s → 비슷
    max_distance=100 → 20× 과대
    k_drop_proximity=0.15 → 5m 환경에서 gradient 너무 완만

  v1 의 96% success 는 사실 운+task 단순화 덕분.
  v2 실패는 mis-scaled 환경에 정밀화 요구 가속 안 되어서.

**변경 (Scale 처방)**:
- `environment.pos_scale`: 50.0 → **5.0**
  - **이유**: 5m 거리에 맞춤. obs 범위 [-1,1] 활용.
- `environment.action_vx_scale`: 8.0 → **3.0** m/s
  - **이유**: 5m 거리에 정밀 제어 가능. 8 m/s 면 0.6초 통과.
- `environment.action_vy_scale`: 5.0 → **3.0** m/s
- `environment.max_distance`: 100.0 → **20.0** m
  - **이유**: 4× target 안전 마진, 명확성.
- `reward.k_drop_proximity`: 0.15 → **0.4**
  - **이유**: 5m 안에서 sharp gradient (가까울수록 강한 신호).

**변경 (임계 완화)**:
- `reward.auto_drop_threshold`: 1.0 → **2.0** m
  - **이유**: v2 의 1m 너무 빡빡 → curriculum gap 줄임.
- `reward.success_threshold`: 1.0 → **2.0** m
- `reward.jackpot_threshold`: 0.3 m 유지

**run_name**: `phase1_redux_v3_scaled_thresholds`
**Resume vs Fresh**: Fresh — replay buffer 의 scale 가정 변경됨, mis-scaled 정책 무효화.

**WandB metric 변경** (WandbMetricsCallback):
- 추가: `env/d_xy_outlier_ratio` (0-6m: 0, 6m+: 1 의 평균 — outlier 검출)
- 추가: `env/success_rate` (SB3 의 rollout/success_rate 를 env/ 에도 mirror)
- 작동: `env/current_success_streak` (v2 는 옛 코드 메모리 로드라 안 됐던 것)
- 제거: `env/total_truncate_*` 모두

**유지** (Phase 1 redux v3 핵심 처방):
  - SAC: target_entropy=-15, target_q_clip=500, ent_coef_hard_cap=1.0
  - PER: alpha=0.6
  - Critic: Huber + per-sample damping
  - 인프라: max_consecutive_fast_resets=100, _kill_episode/_infra timeout=2s
  - 환경: target_enu=(4,3), random_drop_prob=0

**초기 결과** (step 1405):
  - **ep_len_mean = 351 step** (v2 의 23 대비 15배 ↑) ← scale 처방 즉각 효과
  - **ep_rew_mean = +168** (v2 의 -42 와 정반대 — 양수 reward)
  - 드론 안정 비행 (action_vx 3 m/s 효과)
  - critic_loss 1.53 (안정)
  - 4 drops in 1.4k step (0.3%, 정상 시작)
  - fps 7 (첫 4 episode 표본 작아 신뢰성 낮음, 진행 후 회복 예상)

**검증 가설 (5가지)**:
  1. Scale 처방 → 학습 효율 + 정밀도 향상?
  2. 임계 2m → 적응 가능 curriculum 시작점?
  3. fps 15-25 회복?
  4. 새 도구들 정상 작동 (wandb 표시)?
  5. **Representative top 3 의 첫 의미 측정 가능?**

**결론 / 다음**: 학습 진행 중. 300k step → 약 3-5시간 예상 (fps 따라).

---

### #34. 2026-06-11 — **Phase 1 redux v6: hover drop 처방 (drop_wait timeout + hover_drop_block 검토)**

WandB run: `phase1_redux_v6_hover_drop_fix` (300k)
**Base**: #33 v5 (SDF dimensions=3 fix)

**v5 결과 → v6 처방 동기**:
  - v5 학습 success 16% (이전 baseline 갱신)
  - 그러나 invalid drop **50% 발생** — drop_calculator 가 정상 ground impact 받기 전 timeout
  - Issue #022: fps 6 정체 — reset overhead 70-80%

**변경**:
- `reward.drop_wait_timeout`: 10.0 → **3.0** s
  - **이유**: 정상 자유낙하 1초면 충분. 10s timeout 의 7s 가 invalid drop 비용 (fps 낭비). 70% 단축.
- `reward.hover_drop_block_threshold`: **검토 후 0 (비활성) 유지**
  - 가설: hover 중 drop 이 50% invalid 원인
  - 검토: 실측 시 hover drop 도 정상 ground impact. invalid 의 진짜 원인은 DetachableJoint plugin 의 reattach 후 detach silent fail (v7/v8 에서 확정)
  - 그래도 yaml key 는 추가 (config 화), 값 0 = 비활성

**결과**:
- success 11% (v5 의 16% 대비 약간 감소 — variance 안)
- invalid drop 50% 여전 (hover drop 가설 폐기)
- fps 약간 회복

**결론 / 다음**: invalid drop 의 진짜 원인 미해결 → v7 (옵션 C safe attach) 시도.

---

### #35. 2026-06-15 — **Phase 1 redux v7: safe attach (옵션 C) — drop 0 학습 실패**

WandB run: `phase1_redux_v7_safe_attach` (실패, ~37k)
**Base**: #34 v6

**옵션 C (safe drop reset path) 의도**:
  - drop 후 _kill_infra (38s) 대신 safe path (reattach 검토)
  - fps 회복 + DetachableJoint plugin 의 silent fail 우회 시도

**변경 (코드 + yaml)**:
- 옵션 C safe path 활성 (drone_drop_env.py)
- `reward.invalid_drop_penalty`: 50.0 유지 (drop_calculator timeout 회피 학습 의도)
- `reward.invalid_drop_threshold`: 50.0 유지

**결과 (재앙)**:
- **drop 0건** — 정책이 drop 행동 자체를 학습 회피
- 원인 분석: invalid_drop_penalty 50 의 신호가 강력 → 정책이 drop = 50% invalid (이전 v5/v6 경험) → drop 아예 안 함이 안전
- 옵션 C 의 reattach 자체도 silent fail (DetachableJoint plugin 의 본질 문제)

**결론 / 다음**:
- 옵션 C 폐기
- invalid_drop_penalty 의 정책 영향 분리: 0 으로 비활성 → v8
- 옵션 C 코드는 비활성 (D1 처방으로 매 drop 마다 _kill_infra 회귀)

---

### #36. 2026-06-19~21 — **Phase 1 redux v8: no invalid penalty — 80.6% success 달성**

WandB run: **`96bokgae`** (303k 자연 종료)
**Base**: #35 v7 (옵션 C 폐기 후 D1 회귀)

**핵심 진단** (v7 의 drop 0 결과 분석):
  - `invalid_drop_penalty: 50.0` 이 정책의 drop 회피 학습 강제
  - v5/v6 의 50% invalid drop 경험 → penalty 누적 → drop = 위험
  - 정책이 drop 안 함이 최적 결정 → 학습 실패

**변경**:
- `reward.invalid_drop_penalty`: 50.0 → **0.0** (비활성)
  - **이유**: 정책이 drop 회피 학습 차단. invalid 도 학습 신호로 받되 negative reward 0.
- `reward.invalid_drop_threshold`: 50.0 → **95.0**
  - **이유**: 99m default 만 잡음 (안전망). 실제 invalid 의 49% 가 50-95m 범위 였음 → 정상 drop 으로 분류.

**유지**:
- D1 처방 (옵션 C 폐기): 매 drop 후 `_kill_infra + _start_infra` (38s) → DetachableJoint silent fail 우회 (fresh SDF attach 보장)
- 그 외 v3 의 scale 처방 + v5 의 SDF fix 모두 그대로

**결과 (Phase 1 redux 의 진짜 baseline)**:
- **success 80.6%** (2,694 success / 3,342 drops)
- **jackpot 13건** (≤ 0.3m)
- mean drop_err **1.85m**
- **best drop 0.07m** (drop_0588_step157201_err0.07m.npz)
- 8,736 episodes, 303,801 timesteps
- backup 8.6 GB (모델 + replay 8.1GB + drop_episodes 28MB + wandb 421MB)

**정책 행동 분석** (사용자 GUI 관찰):
  - drone 이 marker (4, 3) 를 지나친 위치 (예: (5, 4)) 까지 비행
  - 그 위치에서 멈춤 + 몸을 marker 방향으로 기울임 (pitch back)
  - detach → payload 가 forward momentum 으로 marker 향해 toss
  - 모든 ep 마다 일관 — 정책 완전 수렴

**결론 / 다음**: Phase 1 redux 완성. toss 전략 본질 변경하려면 환경 / reward 추가 처방 필요 → v9 라운드.

---

### #37. 2026-06-22 — **ang_vel callback fix: PX4 dds_topics + limit_ang_vel**

WandB run: 별도 없음 (v8 fix only)
**Base**: #36 v8

**발견 동기**: v9 처방 (drop ang_vel penalty) 검증 위해 v8 의 drop_episodes 의 ang_vel obs 분석

**충격 결과**:
```
v8 학습 500 ep sample 의 obs[:, 6:9] (ang_vel 부분):
  mean: 0.000000
  max:  0.000000
  std:  0.000000
  >0 인 비율: 0.00%
```

→ **v8 학습 전체가 ang_vel obs 없이 진행됨에도 80% success 달성**.

**Root cause**:
`/opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` 의 `vehicle_angular_velocity` 가 PX4 default 에서 **주석 처리됨**:

```yaml
# Before
publications:
#  - topic: /fmu/out/vehicle_angular_velocity    ← 주석
#    type: px4_msgs::msg::VehicleAngularVelocity
  - topic: /fmu/out/vehicle_attitude            ← 활성
  - topic: /fmu/out/vehicle_local_position
  - topic: /fmu/out/vehicle_status
```

PX4 maintainers 가 의도적으로 disable (quaternion 의 derivative 라 redundant 판단).

**Fix**:
1. `dds_topics.yaml` 의 두 줄 uncomment
2. PX4 rebuild (`make px4_sitl_default`, ~15분)
3. `reward.limit_ang_vel`: **2.0 → 10.0**
   - **이유**: line 848 의 `omega_mag > limit_ang_vel` crash detection. 실측 max ang_vel = 4.89 rad/s (toss pitch back peak). false crash trigger 위험 → 10.0 완화.
4. install/share sync (hyperparams.yaml + drone_drop_env.py)

**검증 (v8 정책 5 ep 평가)**:
- 5/5 success ✓ (fix 전 5 ep 와 동일 success rate)
- mean err 1.85m (분포 안)
- 실측 max ang_vel: 2.1 ~ 2.7 rad/s
- drop 직전 ang_vel < 0.5 rad/s (drone 자연 안정화)

→ v8 정책 동작 영향 미미 (학습 시 항상 0 이라 weights 가 그 input 에 무지).

**Backup**:
- container: `/tmp/ang_vel_fix_backup/dds_topics.yaml`
- host: `local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml`

**결론 / 다음**: v9a 의 drop_angaccel penalty 의 prerequisite. 이제 ang_vel 측정 가능 → 처방 1 (drop_angaccel) 구현 가능.

---

### #38. 2026-06-26~27 — **Phase 1 redux v9a: payload_dist + drop_angaccel (v8 warm start fine-tune)**

WandB run: **`zjexq20k`** (SIGTERM stop @ step 313k = v8 + 17k)
**Base**: #36 v8 (warm start) + #37 (ang_vel fix prerequisite)

**사용자 처방 결정**:
1. **drop 시점 drone 의 각속도 변화 penalty** (사용자 제안 — toss 의 급격한 pitch back 차단)
2. **payload 가 target 향해 가까워지면 +reward, 멀어지면 -penalty** (사용자 제안)

**처방 1 (drop_angaccel) — 측정 방법 E**:
  drop trigger 시 직전 5 step 의 인접 ang_vel diff magnitude 의 max

```python
# drone_drop_env.py step() drop trigger 후:
hist = list(self._ang_vel_history)   # deque(maxlen=6)
max_ang_accel = 0
for i in range(1, len(hist)):
    accel = ||hist[i] - hist[i-1]||
    if accel > max_ang_accel: max_ang_accel = accel
reward -= drop_angaccel_penalty_scale * max_ang_accel
```

**처방 2 (payload distance) — 구현 분석**:
  사용자 의도 = "payload trajectory 의 monotonic 거리 감소 보상"
  분석 결과: 기존 `r3_dist = w_dist * (d_xy_prev - d_xy)` 와 본질적으로 동일 (attached 동안 drone == payload)
  → **w_dist scale 증가** (1.0 → 1.5) 로 구현 (옵션 A)
  → detach 후 payload tracking 은 ep 즉시 종료라 추적 안 됨 (옵션 B, C 폐기)

**변경 (hyperparams)**:
- `reward.w_dist`: 1.0 → **1.5**
  - **이유**: payload distance reward 강화 (+50%). marker 지나치는 시점의 penalty 도 50% 더 큼.
- `wandb.run_name`: "phase1_redux_v8_no_invalid_penalty" → **"phase1_redux_v9a_payload_dist_angaccel"**
- `wandb.tags`: + ["v9a", "payload_dist", "drop_angaccel"]
- `training.total_timesteps`: 300000 → 100000 (override 시 400000 = v8 의 303k + 100k)

**추가** (NEW yaml keys):
- `reward.drop_angaccel_penalty_scale` = **0.5**
  - **이유**: drop 시점 max ang_accel 평균 ~2 rad/s² × 0.5 = -1.0 per ep (terminal +30 의 ~3%). 명확한 영향, 발산 안전.
- `reward.drop_angaccel_window_n` = **5**
  - **이유**: drop 직전 5 step (fps 1-2 → 2-5초). toss 의 pitch back 동작 윈도우.

**추가** (NEW 코드, `drone_drop_env.py`):
- `from collections import deque`
- `__init__`: `_cfg_drop_angaccel_penalty_scale`, `_cfg_drop_angaccel_window_n`, `_ang_vel_history = deque(maxlen=N+1)`
- `reset()`: `_ang_vel_history.clear()`
- `step()` 매 step: `_ang_vel_history.append(ang.copy())`
- drop trigger 후 instability penalty 다음: max ang_accel 계산 + reward 차감 + `info['drop_max_ang_accel']`

**학습 명령**:
```bash
ros2 run rl_navigation train_sac \
    --resume /workspace/ros2_ws/eval_models/v8_peak_step217040_err0.87m.zip \
    --timesteps 400000
```

- warm start: v8_peak (`num_timesteps = 217,040`)
- fresh replay buffer: `_replay.pkl` 없음 → 자동 새 buffer (CLAUDE.md "보상 공식 변경 → Fresh Start" 준수)
- reset_num_timesteps=False → counter 유지, total 400k 까지 학습 = +183k step 의도 (실제 17k 만 진행)

**학습 진행 (5h, SIGTERM stop)**:
| 시점 | total_step | success_rate | ep_rew_mean | ep_len_mean | actor_loss |
|---|---|---|---|---|---|
| 시작 (18:41 UTC) | 217,040 | (v8 0.80) | — | — | — |
| 3시간 | 234,301 | 0.51 ⚠️ | -50 | 47 | 60-80 |
| 4시간 | 296,706 | 0.55 | -43 | 40 | 29-33 |
| 4.8시간 | 303,127 | 0.54 | -37 | **6.27** ⚠️ | 22-35 |
| 5시간 (SIGTERM) | **313k** | **0.58** | **-15.5** | **96.3** ✓ | 35.1 |

- 4.8h 시점에 ep_len 6.27 — 정책 발산 의심
- 5h 회복 (96.3) — fine-tune 적응 마무리

**평가 (dgui 5 ep, preempt step 313k)**:
```
EP1: 1.79m ✓
EP2: 1.81m ✓
EP3: 1.65m ✓
EP4: 1.98m ✓
EP5: 2.22m (margin)
success ≤2m:   4/5 = 80%
mean err:      1.888m
max ang_vel:   2.10 rad/s (평균)
```

**v8 vs v9a 비교**:

| 항목 | v8 | v9a | 변화 |
|---|---|---|---|
| success rate | 80% | 80% | 동일 |
| mean err | 1.852m | 1.888m | +0.036m (분포 안) |
| **max ang_vel** | **2.5 rad/s** | **2.10 rad/s** | **-16% ↓** ✓ |
| 정책 행동 | toss | **toss 유지** | 본질 동일 |

**결론**:
- **drop_angaccel penalty 효과 명확** (16% ang_vel 감소). 정책이 부드러운 drop 학습.
- **w_dist 1.5 효과 미미** — 누적 +1.4 (terminal +30 의 5%) 만으로는 정책 본질 행동 변경 어려움.
- **사용자 의도 (지나치는 현상 해결) 안 됨** — toss 전략 그대로 유지.
- 원인: 17k step 만 fine-tune (의도 100k 의 17%). v8 의 강한 toss prior 가 처방 흡수.

**Backup**:
- `eval_models/v9a_preempt_step313k.zip` (3.2 MB, 평가용)
- `rl_checkpoints/sac_drop_preempt.zip` + `_replay.pkl` (86 MB)
- 평가 결과: `local/eval_logs/eval_2026-06-27T00-36-50_v9a_preempt_step313k.json`

**다음 단계 후보**:
1. v9a 100k 까지 더 학습 (+~12시간) — 처방 효과 누적 가능
2. 두번째 처방 도입 (사용자 1, 2, 3 안건)
3. payload tracking 완전 구현 (Gazebo pose + ep 연장)
4. 환경 변경 (target randomize, cruise 비활성)
5. Fresh start with stronger shaping

---

### #39. 2026-06-26~27 — **v9a resume (xzoz52cw) — CUDA error 로 강제 종료**

WandB run: **`xzoz52cw`** (313k → 432k, +119k step)

**의도**: v9a 313k preempt 에서 resume + 추가 학습 (의도 ~600k 까지, 사용자 SIGTERM 예정)

**진행**:
- 시작: 2026-06-26 17:33 UTC, 7시간 18 분 진행
- replay buffer 자동 load (`sac_drop_preempt_replay.pkl` 86 MB)
- drop_episodes +449 추가 (4527 → 4976)

**종료 (실패)**:
- **CUDA error: unspecified launch failure** — checkpoint 저장 시 torch.save 의 GPU 오류
- container Exit (137) — SIGKILL
- preempt save 실패 (CUDA error 가 _emergency_save 호출 전 발생)
- **rolling checkpoint 5k 마다 자동 저장** → `sac_drop_432806_steps.zip` 보존

**결과**: `eval_models/v9a_step432806.zip` (3.2 MB)
**replay buffer 일관성**: 313k 시점 만 (432k 와 mismatch → resume 어려움)

---

### #40. 2026-06-27 — **v9a step432k 5 ep 평가 — 정책 악화**

평가 only. Base: v9a step432k

**dgui 5 ep, deterministic**:
```
EP1: 2.10m margin, steps 103, max_ang_vel 1.94
EP2: hover_timeout (drop X, d_xy 10m) ⚠️
EP3: 1.83m ✓, max_ang_vel 2.08
EP4: 1.94m ✓, max_ang_vel 2.11
EP5: hover_timeout + max_ang_vel 2.95 ⚠️ (정책 unstable)

drops: 3, success ≤2m: 2/3 = 66.7%, mean 1.96m
```

**비교**:
| 모델 | step | success | hover_timeout |
|---|---|---|---|
| v8 | 217k | 5/5 = 100% | 0 |
| v9a 313k | 313k | 4/5 = 80% | 0 |
| **v9a 432k** | 432k | **2/3 = 66.7%** | **2** ⚠️ |

**결론**: 추가 학습 (+119k) 이 정책 악화. 313k 가 더 안정. 그러나 5 ep 표본 작음 → 10 ep 평가 필요.

---

### #41. 2026-06-27 — **10 EP 통계 비교 (v8 vs v9a 313k) — 표본 운 의 영향 발견**

평가 only. 통계 신뢰성 확인 위해 양 모델 각 10 ep.

**v8_peak (10 ep)**:
```
EP 1~5:  1.93, 1.92, 1.87, 1.98, 1.74  → 5/5 ≤2m ✓
EP 6~10: 2.18, 2.08, 2.03, 2.18, 2.12  → 5/5 borderline ✗ (2.0~2.2m)
success ≤2m: 5/10 = 50%
mean: 2.002m, min 1.744, max 2.181, hover_timeout 0
```

**v9a 313k (10 ep)**:
```
EP 1~3:  1.99, 1.75, 1.85  → ✓
EP 4, 5: 2.01, 2.07  → ✗ borderline
EP 6:    hover_timeout (drop X) ⚠️
EP 7~10: 2.01, 2.11, 2.13, 2.13  → ✗ borderline
success ≤2m: 3/9 = 33%, drops 9/10
mean: 2.006m, hover_timeout 1
```

**핵심 발견**:
1. **이전 5 ep 100% (v8) / 80% (v9a) 는 표본 운** — 실제 50% / 33%
2. **v8 가 v9a 보다 success 17% ↑** (binomial 검정 의미 있음)
3. **mean err 둘 다 ≈ 2.0m** — 정책 정확도 자체는 비슷, threshold 2.0m 경계
4. **만약 success_threshold 가 2.1m 였다면** → v8 9/10 (90%), v9a 6/9 (67%)

**v9a 처방 최종 trade-off**:
- ✓ drop_angaccel 효과 명확 (ang_vel -16%)
- ✗ success rate 약간 ↓ (50% → 33%)
- ✗ 사용자 의도 (지나치는 현상 해결) 안 됨 — toss 그대로

**최종 결정 (사용자, 2026-06-27)**:
- **v8 을 best baseline** 으로 결정
- v9a 의 fine-tune 처방으로는 사용자 의도 달성 어려움
- **새 design framework** (2 단계 모드 분리, `design/two_stage_learning_plan.md`) 로 진행

---

### #42. 2026-06-30 — **RAD v1 design 완료** (Relative Approach Drop, 새 framework)

**Base**: v8 (`hyperparams.yaml`) 기준. RAD 는 v8/v9a 와 완전 다른 framework — Round/redux 시리즈의 patch 가 아닌 **새 framework 의 v1**.

**파일 분리 (옵션 A)**:
- `hyperparams_rad.yaml` (신규)
- `drone_drop_env_rad.py` (신규, Class: `DroneDropEnvRAD`)
- `train_sac_rad.py` (신규, Phase 1/2 분리 logic)
- `mission_manager_rad_node.py` (신규)
- `infra_rad.launch.py`, `episode_rad.launch.py` (신규)
- 기존 v8 파일 그대로 (왔다갔다 학습 가능)

**참조**: [design/rad_v1_design.md](design/rad_v1_design.md) — single source of truth.

---

**환경 (Phase 1 + Phase 2 공통)**:

- `environment.target_enu_x`: 4.0 (v8 그대로)
- `environment.target_enu_y`: 3.0 (v8 그대로)
- `environment.target_enu_z`: 5.0 → **0.0** (지면 marker 명시. v10a stage1 임시값 폐기)
  - **이유**: drop_calculator 가 z=0 지면에서 측정. target = (4, 3, 0) 통합
- `environment.pos_scale`: 5.0 → **10.0**
  - **이유**: v10 좌표 √50 ≈ 7m 대응 (obs ∈ [−1, 1] 유지)
- `environment.vel_scale`: 15.0 (v8 그대로)
- `environment.ang_vel_scale`: π (v8 그대로)
- `environment.action_vx_scale`, `vy`, `vz`: 3.0 (v8 그대로)
- `environment.action_yaw_scale`: 1.0 (v8 그대로)
- `environment.max_steps`: 800 (v8 그대로)
- `environment.min_altitude`: 3.0 → **0.5** (사실상 ground_contact_alt 와 통합, dead)
- `environment.ground_contact_altitude`: 0.5 (v8 그대로)
- `environment.max_distance`: 20.0 → **15.0**
  - **이유**: v10 좌표 √50 ≈ 7m 에 적정 마진 2× = 15m
- `environment.max_altitude`: 50.0 (v8 그대로)
- `environment.max_consecutive_fast_resets`: 100 → **50**
  - **이유**: Phase 1 drop 없음 → fast reset 비율 100% → GZ 누적 leak 조기 차단

**obs (14d, 상대좌표, yaw-only body frame)**:

- **REMOVED**: obs[0:3] 절대 pos_world, obs[9:11] pixel u/v, obs[13:14] rel_dx/dy (world frame)
- **NEW obs structure**:
  - `obs[0:3]` = (Δx_b, Δy_b, Δz_world) / POS_SCALE — target - drone, body frame
  - `obs[3:6]` = (vx_b, vy_b, vz_world) / VEL_SCALE — drone vel, body frame
  - `obs[6:9]` = (ω) / π (v8 그대로)
  - `obs[9:11]` = (roll, pitch) / π
  - `obs[11]` = payload_attached (Phase 1 dead = 1.0)
  - `obs[12]` = d_impact / POS_SCALE (Phase 1 dead = 0)
  - `obs[13]` = t_f / 10 (Phase 1 dead = 0)
  - **REMOVED**: yaw 절대값 (정책이 yaw-invariant 학습)
- 변환식: `[Δx_b; Δy_b] = R(yaw)^(-1) · [Δx_w; Δy_w]`, Δz_world 회전 안 함

**Spawn / Cruise (RAD only)**:

- **NEW**: `cruise.spawn_yaw_random_enabled` = true
- **NEW**: `cruise.spawn_yaw_relative_range` = [−π/2, +π/2] (relative to drone→target vector, uniform)
- **NEW**: `cruise.target_speed` = 1.0 m/s (head 방향 가속)
- **REMOVED**: `cruise_speed_x` (1.0), `cruise_speed_y` (−1.0) — 자동 이동 폐기

**Switch sphere / Final state (Phase 1)**:

- **NEW**: `phase1.switch_d_sq` = 20.5 (sphere d²≤20.5 진입 시 Phase 1 종료, radius √20.5 ≈ 4.53m)
- **NEW**: `phase1.terminal_floor` = 20 (sphere 진입 자체 보상)
- **NEW**: `phase1.terminal_w_each` = 50/7 ≈ 7.14 (각 final state 조건 만족당)
- **NEW**: `phase1.terminal_complete_bonus` = 50 (모든 조건 만족 jackpot)
- **NEW**: Final state 7 조건 임계값
  - `phase1.C1_z_min` = 3, `C1_z_max` = 5
  - `phase1.C2_v_xy_max` = 4
  - `phase1.C3_v_z_max` = 2
  - `phase1.C4_tilt_max` = 0.26
  - `phase1.C5_omega_max` = 2
  - `phase1.C6_yaw_err_max` = 1.047 (60°)
  - `phase1.C7_v_xy_min` = 0.3
- **REMOVED**: `stage1_R` (2.0), `stage1_only` (false), `stage1_reach_bonus` (100) — v10a stage1 임시 메커니즘 전체 폐기

**Reward (per-step)**:

- `reward.w_dist`: 1.0 (Phase 1 그대로) / 1.0 (Phase 2, v9a 의 1.5 폐기)
  - **거리 계산 변경**: `d_xy = √(x² + y²)` (v8) → `d_reward = √((x−target_x)² + (y−target_y)² + (z−4)²)` (RAD)
  - **이유**: z 포함, reference = (target_x, target_y, 4) — z=4 최소 안전 고도 위 1m
- `reward.w_heading`: 0.7 (v8 그대로, 수평만)
- **NEW**: `reward.w_z` = 0.3 (Hann raised cosine 가중치, **Phase 1 + Phase 2 모두 활성**)
- **NEW**: `reward.z_target` = 4.0 (Hann mu)
- **NEW**: `reward.z_half_range` = 3.5 (Hann window half-range. ⚠️ gaussian σ 가 아님)
- **NEW form**: `r_z = w_z × 0.5 × (1 + cos(π × (z − 4) / 3.5))` for z ∈ [0.5, 7.5], else 0
- `reward.w_impact`: 0.4 → **Phase 1: 0** (drop 없음), **Phase 2: 1.0** (강화)
- `reward.k_impact`: 0.05 → **0.1** (Phase 2, decay 강화)
- `reward.w_distance_penalty`: 0 (v8 그대로, dead)
- `reward.w_time` (hardcoded `-0.05`): **Phase 1: −0.05, Phase 2: −0.1** (2× 강화)
- `reward.w_ang_vel`: 0.05 → **Phase 1: 0.05, Phase 2: 0.1** (2× 강화)
- `reward.w_action_smooth`: 0.05 → **Phase 1: 0.05, Phase 2: 0.1** (2× 강화)
- `reward.speed_gate_enabled`: true (v8 그대로)

**Drop trigger (Phase 2)**:

- `reward.auto_drop_threshold`: 2.0 → **1.0** (CCIP 예측 miss < 1m 시 auto drop, 정밀도 강화)
- `reward.random_drop_prob`: 0 (v8 그대로)
- `reward.random_drop_start_step`: 600 (v8 그대로, dead)
- `reward.hover_drop_block_threshold`: 0 (v8 그대로, dead)
- Manual drop (action[4]): 비활성 (v8 그대로)

**Drop event reward (Phase 2)**:

- `reward.drop_attempt_bonus`: 30 (v8 그대로)
- `reward.k_drop_proximity`: 0.4 → **0.1** (약화)
  - **이유**: drone 이 target 위 정확히 가도록 압박 약화 → toss 학습 보존
- `reward.w_drop_base`: 100 (v8 그대로)
- `reward.k2_precision`: 0.2 (v8 그대로)
- `reward.r_success_jackpot`: 50 (v8 그대로)
- `reward.jackpot_threshold`: 0.3 (v8 그대로)
- `reward.success_threshold`: 2.0 → **1.0** (Phase 2)
  - **이유**: d_impact trigger 1m 과 정합. trigger 가 곧 success 정의
- `reward.alt_penalty_max`: 50 → **0** (폐기)
- `reward.alt_penalty_mid`: 30 → **0**
- `reward.alt_penalty_k`: 0.15 → **0**
  - **이유**: RAD 의 z=4 강제 정책 (Hann reward) + Phase 1 C1 조건 (z∈[3,5]) 으로 drop 고도 안전. sigmoid 발동 안 됨
- `reward.drop_angaccel_penalty_scale`: 0.5 (v9a, 그대로)
- `reward.drop_angaccel_window_n`: 5 (v9a, 그대로)
- `reward.invalid_drop_penalty`: 0 (v8 그대로, dead)
- `reward.invalid_drop_threshold`: 95 (v8 그대로)
- `reward.drop_wait_timeout`: 3.0 (v6, 그대로)
- `reward.w_prediction`: 0 (v5+, 그대로 dead)
- `reward.penalty_instability`: 50 (v8 그대로)

**경계 제약 (Phase 2)**:

- `environment.action_rate_limit`: 0.2 → **0.15 (Phase 2)** (Phase 1 은 0.2 유지)
  - **이유**: drop 직전 부드러움 강화

**Crash (Phase 2 신규)**:

- **NEW**: `phase2.sphere_escape_d_sq` = 22 (drone d² > 22 → ep terminate)
  - radius √22 ≈ 4.690m (sphere 4.527m + margin 0.163m)
  - **이유**: Phase 1 종단 distribution 보호 + 정책 발산 방지. margin 0.16m 로 학습 noise 흡수
- **NEW**: `phase2.sphere_escape_penalty` = −30

**Phase 2 max_steps**:

- **NEW**: `phase2.max_steps` = 200 (≈ 10 초)
  - **이유**: 사용자 원칙 "Phase 2 짧고 빠른 drop"

**기존 페널티 (변경 없음)**:

- `reward.penalty_crash`: −50 (v8 그대로, z<0.5 발동)
- `reward.penalty_overspeed`: −30 (v8 그대로)
- `reward.penalty_target_lost`: −10 (v8 그대로, vision dead)
- `reward.penalty_out_of_range`: −30 (v8 그대로, d>15m)
- `reward.penalty_max_altitude`: −15 (v8 그대로)
- `reward.penalty_hover`: −30 (v8 그대로)
- `reward.hover_speed_threshold`: 1.0 (v8 그대로)
- `reward.hover_consecutive_threshold`: 150 (v8 그대로)
- `reward.hover_truncate_enabled`: true (v8 그대로)
- `reward.truncation_penalty`: −15 (v8 그대로)
- `reward.penalty_bad_attitude`: −30 (v8 그대로)
- `reward.limit_ang_vel`: 10.0 (v8 그대로, ang_vel fix 후)
- `reward.limit_tilt`: 0.26 (v8 그대로)
- `reward.limit_inverted_tilt`: 1.047 (v8 그대로)

**SAC hyperparams (변경 없음)**:

- 전부 v8 그대로 (lr=1e-4, buffer_size=500k, batch=256, tau=0.002, gamma=0.995, learning_starts=1000, gradient_steps=1, net_arch [256,256], PER alpha=0.6 etc., DampedEntropySAC settings, target_entropy=−15.0, target_q_clip=500)

**학습 (Phase 1)**:

- **NEW**: `training.phase` = "phase1"
- `training.total_timesteps`: 600000 → **300000 floor + success gate**
- 종료 조건: step ≥ 300k AND 50k window success_rate ≥ 90% (success = sphere 진입 AND ∏ C1~C7 = 1)
- `training.checkpoint_dir`: `/workspace/ros2_ws/rl_checkpoints` → `/workspace/ros2_ws/rl_checkpoints_rad`

**학습 (Phase 2)**:

- **NEW**: `training.phase` = "phase2"
- **NEW**: `training.phase1_model_path` = `rl_checkpoints_rad/sac_phase1_final.zip`
- `training.total_timesteps`: → **150000 floor + success gate**
- 종료 조건: step ≥ 150k AND 50k window success_rate ≥ 90% (success = drop_error ≤ 1.0m)
- **NEW**: `training.phase1_rollout_max_steps` = 300 (Phase 2 ep 시작 시 Phase 1 정책 rollout 최대)
- Phase 1 rollout step 은 Phase 2 buffer 에 안 들어감 (학습 신호 오염 방지)

**Wandb**:

- `wandb.project`: `drone-bombard-sac` (그대로)
- `wandb.run_name`: `phase1_redux_v9a_payload_dist_angaccel` → **`rad_phase1_v1`** (Phase 1), **`rad_phase2_v1`** (Phase 2)
- `wandb.tags`: → **`["rad", "phase1", "v1"]`** 또는 **`["rad", "phase2", "v1"]`**

**결과**: design 완료. 학습 결과는 추후 entry #43+ 에서 기록 예정.

**결론 / 다음**:
- design 단계 완료. 코드 작성 (#134, #135) 대기
- 학습 전 Reset 잔존 속도 / EKF 누설 dgui smoke test (#129) 필요
- Phase 1 5k dry-run → 본학습 → 종료 → Phase 2 5k dry-run → 본학습 sequence

---

## 5. Entry 추가 template

```markdown
### #N. YYYY-MM-DD — <한 줄 제목>

WandB run: `<id>` (online/offline)
**Base**: <#N or branch>

**변경**:
- `<section>.<key>`: <old> → <new>
  - **이유**: ...

**추가** (NEW):
- `<section>.<key>` = <value>  (또는 코드 측 callback/메서드)
  - **이유**: ...

**제거** (REMOVED):
- `<section>.<key>`  (또는 폐기된 코드 patch)
  - **이유**: ...

**결과**:
- <metric>: <value>
- ...

**결론 / 다음**: ...
```

§2 (current snapshot) 와 §3 (요약 표) 도 함께 갱신.

---

끝.
