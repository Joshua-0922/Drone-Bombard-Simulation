# Change Inventory — Pre-Pull (2026-05-22)

> branch_pull_plan_2026-05-22.txt 의 Phase 1-3 산출물.
> 새 branch pull 전 우리(junsang branch + 호스트 측 자료)가 만진 모든 것을
> 한 문서에서 본다. Pull 후 분류 판단의 baseline.

작성: 2026-05-22 (200k 학습 완주 + evaluate 검증 직후)
참조: branch_pull_plan_2026-05-22.txt, meeting_notes_2026-05-20.txt, meeting_notes_2026-05-22.txt

---

## 0. 현재 상태 진단 (Pull 결정의 근거)

### 0-1. 200k N1=B v2 학습 결과 — **완전 실패**
- 학습 자체는 200k step 완주 (Training complete 로그 + sac_drop_final.zip 02:39 저장)
- Evaluate (5 episodes, deterministic): **0/5 drops**, mean reward -6060, miss = N/A
- Trajectory (top view, final episode): Start (3,1) → Target (11,10) 인데 End (-16,23) 로 **타겟 반대 방향 비행**
- **사용자 직접 GUI 관찰**: "드론이 처음부터 바닥에 부딪히면서 튕기면서 어디론가 갔다" → takeoff 도 실패
- 결론: 정책이 motor command 조차 학습 못함. 14m offset 만의 문제가 아닌 구조적 결함.

### 0-2. 14m systematic offset (wandb 분석에서 발견)
- 200k 도중 wandb API 로 19 drops 분석 → drop_error_actual_m 이 **17/19 회 정확히 14.8661m**
- CCIP 예측 d_impact ≈ 1m vs 실측 ~14.87m → 14m 시뮬레이션 구조적 offset
- 사용자가 다른 branch 에서 동일 코드 정상 동작 확인 → branch 교체 결정

### 0-3. Branch 후보 검토
- `junsang` (현재) — commit 29b1c9b, uncommitted 3 파일
- `jekyun` (local) — commit 22247d3, **junsang 보다 1-commit 옛 상태** (29b1c9b 없음)
- `main` (local) — 별도 검증 필요
- `remotes/origin/main` — 별도 검증 필요
- **⚠ Phase 2 진입 전 사용자 확인 필수**: "다른 branch" 가 jekyun 인지 main 인지 또는 원격 어딘가인지

---

## 1. 코드 변경 (uncommitted in junsang branch)

git diff HEAD 기준, 3 파일 / +46-20 줄.

### 1-1. `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` (+12-9)

| 라인 | 변경 | 라벨 | 의도 |
|---|---|---|---|
| L302 (docstring) | Action Box(5) → Box(4) docstring 갱신 | H2 (5/20) | action[4] manual drop 제거 명시 |
| L307-309 (docstring) | manual drop 설명 → H2 결정 주석 | H2 (5/20) | 제거 사유 기록 |
| L401 | `shape=(5,)` → `shape=(4,)` | H2 (5/20) | action_space 축소 |
| L415 | `np.zeros(5,...)` → `np.zeros(4,...)` | H2 (5/20) | action_prev shape |
| L451 | `np.zeros(5,...)` → `np.zeros(4,...)` | H2 (5/20) | reset() action_prev |
| L572-574 | manual_drop 주석 정리 + H2 주석 | H2 (5/20) | 옛 코드 제거 |
| L822-826 | r3_impact 에 `* speed_gate` + 주석 | N1=B v1 (5/22) | loitering hack 차단. r3_impact 가 potential-based 아니라 단일 상태 함수라 가중치 키울수록 위험 → r3_orient 의 speed_gate 패턴 재사용 |

### 1-2. `ros2_ws/src/rl_navigation/rl_navigation/train_sac.py` (+16-1)

| 라인 (신규) | 변경 | 라벨 | 의도 |
|---|---|---|---|
| L56 | `self._step_rew_impact: list = []` 추가 | N1=B v1 (5/22) | rew_impact callback 누락 수정 |
| L61 | `self._total_drop_count: int = 0` 추가 | N1=B v2 (5/22) | 누적 drop 카운터 (rollout-reset 되는 drop_count 보완) |
| L86 | rew_impact 수집 if-block | N1=B v1 (5/22) | _on_step 에서 info['rew_impact'] 수집 |
| L93 | `self._total_drop_count += 1` 추가 | N1=B v2 (5/22) | 누적 카운트 증가 |
| L118 | `('_step_rew_impact', 'env/mean_rew_impact')` 매핑 | N1=B v1 (5/22) | _on_rollout_end 매핑 |
| L134 | `log_dict['env/total_drop_count'] = self._total_drop_count` | N1=B v2 (5/22) | wandb 로깅 (stair-step 차트) |
| L157-161 | BestModelCallback docstring 보강 | L6 (5/20) | EvalCallback 도입 안 한 사유 기록 |
| L282 | `eval_freq = cfg_train.get('eval_freq', 10_000)` | L6 (5/20) | yaml 에서 eval_freq 로드 |
| L345 | `eval_freq=10_000` → `eval_freq=eval_freq` | L6 (5/20) | smoke/본학습 분리 가능 |

### 1-3. `ros2_ws/src/rl_navigation/config/hyperparams.yaml` (+18-10)

상세 표는 §2 (Config 변경) 참조.

---

## 2. Config 변경 — hyperparams.yaml

| Key | HEAD | 현재 | 라벨 | 날짜 | 의도 |
|---|---|---|---|---|---|
| `training.total_timesteps` | 1000000 | **200000** | M3 (5/20) | 본 학습 길이 |
| `training.eval_freq` | (없음) | 10000 | L6 (5/20) | deterministic eval 주기 |
| `training.eval_episodes` | (없음) | 3 | L6 (5/20) | episodes per eval |
| `sac.buffer_size` | 100000 | **500000** | M1 (5/20) | catastrophic forgetting 회피 |
| `sac.gamma` | 0.99 | **0.995** | H3 (5/20) | effective horizon ~100→200 |
| `sac.gradient_steps` | 1 | **4** | M2 (5/20) | off-policy sample reuse |
| `reward.w_impact` | 2.0 | **8.0** | N1=B v2 (5/22) | v1=5.0 → v2=8.0 격상 |
| `reward.k_impact` | 0.05 | **0.03** | N1=B v2 (5/22) | 멀리서 신호 살림 |
| `reward.w_drop_base` | 50.0 | **20.0** | H1 (5/20) | terminal spike 축소 |
| `reward.r_success_jackpot` | 100.0 | **30.0** | H1 (5/20) | terminal spike 축소 |
| `reward.penalty_instability` | 50.0 | **15.0** | H1 (5/20) | terminal spike 축소 |
| `wandb.entity` (comment) | "personal entities disabled" | "SNU. junsanglee64 free tier 만료" | 5/21 | 주석 보강만 (값 동일) |

**값 변경 11개 + 주석/문서화 변경 다수.**

이미 HEAD (29b1c9b) 시점에 적용된 것 (HEAD 와 현재 값이 같음):
- `reward.speed_gate_enabled: true`
- `reward.auto_drop_threshold: 2.0` (이전 5/19 이전 fa731ad commit 으로 적용)
- `wandb.entity: nayoonho0922-seoul-national-university` (5/21 적용 후 commit 됨)

---

## 3. World / Launch 변경 (이미 commit 29b1c9b 에 포함됨)

> 이건 git diff HEAD 에 안 잡힘 — 이미 commit 된 상태.
> 새 branch 가 commit 29b1c9b 를 기준으로 fork 됐다면 이미 있음. 아니면 재적용 필요.

| 파일 | 변경 | 라벨 | 날짜 |
|---|---|---|---|
| `gazebo_models/worlds/x_marker_world_vision.sdf` | 신규 생성 (Sensors+ogre2 추가) | 5/19 | vision 미션 전용 |
| `ros2_ws/src/mission_manager/launch/drone_mission.launch.py` L196/204/321 | world 경로 → vision sdf | 5/19 | vision 라우팅 |
| `gazebo_models/worlds/x_marker_world.sdf` (L59-63 주석) | 의도 설명 보강 | 5/20 | RL 학습 안정성 |

---

## 4. PX4 변경 (컨테이너 라이프타임만, persistent 아님)

| 변경 | 위치 | 라벨 | 영구화? |
|---|---|---|---|
| `gz_x500_bombard_r{0,1,2,3}` airframe ROMFS → rootfs cp | `/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/4016~4019` | L7 (5/19~20) | ❌ container 재생성 시 재적용 필요 |

메모리 `project_px4_airframe_sync.md` 에 영구 기록.

---

## 5. Untracked 파일 (git status)

repo 안에 있지만 git 에 추가 안 된 파일들:

| 파일 | 출처 | 처리 방향 |
|---|---|---|
| `ros2_ws/rl_eval_results/` | 2026-05-22 evaluate.py 산출물 (n1b_v2_200k 5 episodes) | 백업 대상 (현 정책 실패 증거물) |
| `ros2_ws/src/mission_manager/mission_manager/gps_target_publisher.py` | 우리가 만들지 않음 — 누군가 추가 | 새 branch 비교 시 확인 |
| `ros2_ws/src/rl_navigation/config/hyperparams.yaml.original` | yaml 백업 (시점 미상) | 백업 대상 |
| `ros2_ws/src/rl_navigation/config/hyperparams.yaml.team_backup` | team entity 이전 시 백업 | 백업 대상 |
| `ros2_ws/src/vision_detection/vision_detection/camera_stub.py` | 우리가 만들지 않음 — 누군가 추가 | 새 branch 비교 시 확인 |

---

## 6. 호스트 측 인프라 스크립트 (repo 외부)

| 파일 | 작성일 | 라벨 | 비고 |
|---|---|---|---|
| `/home/juns/wandb_incremental_sync.sh` | 5/20 | M (인프라) | v2 부터 online 모드라 사용 안 함, 보존만 |
| `/workspace/ros2_ws/start_infra_clean.sh` | 기존 | — | self-managed infra cleanup 패턴 |
| `/workspace/ros2_ws/train_managed.sh` | 기존 | — | 학습 wrapper |

---

## 7. 호스트 측 문서 / 가이드 (repo 외부)

| 파일 | 작성일 | 줄수 | 비고 |
|---|---|---|---|
| `/home/juns/meeting_notes_2026-05-20.txt` | 5/20-21 | 624 | H1/H2/H3/M1/M2/L4/L6/N1 결정 history |
| `/home/juns/meeting_notes_2026-05-22.txt` | 5/22 | 532 | N1=B 도입, v2 격상, 14m offset 발견 |
| `/home/juns/A_phased_curriculum_도입방안.md` | 5/22 | 324 | A 도입의 phased 절차 |
| `/home/juns/parameter_조절_의사결정_guide.md` | 5/22 v2 | 453 | 3D 진단 매트릭스 + 처방 |
| `/home/juns/Downloads/N1_drop_sparse_AB방안.md` | 5/21 | ? | A vs B trade-off |
| `/home/juns/drone_sim_fresh_training_start_guide.txt` | 5/20 | ? | 학습 실행 가이드 |
| `/home/juns/conversation_backup_2026-05-21.txt` | 5/21 | ~480KB | 이전 세션 전체 대화 |
| `/home/juns/branch_pull_plan_2026-05-22.txt` | 5/22 | 361 | 이 계획 문서 |
| `/home/juns/change_inventory_pre_pull_2026-05-22.md` | 5/22 | (이 문서) | Phase 1-3 산출물 |
| `/home/juns/eval_results/n1b_v2_200k_2026-05-22/` | 5/22 | 4 files | evaluate 산출물 (host 복사본) |

---

## 8. Claude 메모리 (auto-load)

`/home/juns/.claude/projects/-home-juns/memory/` — 6 파일:

| 파일 | 갱신일 | 타입 |
|---|---|---|
| `MEMORY.md` | 5/22 | 인덱스 |
| `feedback_local_only.md` | 5/20 | feedback (NEVER push to github) |
| `feedback_ros2_install_cache.md` | 5/20 | feedback (src/install sync) |
| `feedback_train_sac_graceful_kill.md` | 5/22 | feedback (SIGINT pattern) |
| `project_drone_sim.md` | 5/20 | project (컨테이너 구조) |
| `project_n1b_200k_training.md` | 5/22 | project (현재 학습 상태) |
| `project_px4_airframe_sync.md` | 5/20 | project (rootfs cp 필요) |

전부 백업 대상.

---

## 9. 컨테이너 측 archive (rl_checkpoints)

| 폴더 | 내용 |
|---|---|
| `archive/n1_d_baseline_2026-05-21/` | N1=D 베이스라인 (이전 학습 ~95k) |
| `archive/n1_b_w5_k05_2026-05-22/` | N1=B v1 16k preempt (mean_rew_impact=0.567 확인된 시점) |
| `archive/dry_run_2026-05-21/` | dry-run 산출물 |
| `archive/sac_drop_milestone_*` | 50k/100k/150k milestone (v2 학습 중 자동 archive) |
| `sac_drop_final.zip` (현재) | 200k N1=B v2 최종 — **사용자 검증 결과 정책 무용** |

전부 백업 대상.

---

## 10. ⚠ 발견된 충돌 / 주의사항

### 10-1. CLAUDE.md vs 메모리 충돌
- `/home/juns/Drone-Bombard-Simulation/CLAUDE.md` 의 "Auto-Logging & Git Sync (MANDATORY)" 가 git push 를 강제
- 우리 메모리 `feedback_local_only.md` 는 "Local-only — never push to GitHub"
- **사용자 명시 메모리가 우선** — CLAUDE.md 의 Auto-Logging 섹션 무시
- 이건 Phase 3 에서 CLAUDE.md 갱신 또는 메모리에 충돌 사실 명시 필요

### 10-2. CLAUDE.md 의 Obsidian / RL_Project_Log 관행
- `notes/` 폴더의 experiments/research/sessions/daily 노트 관행 정의
- 이번 5/19~22 작업에서는 따르지 않음 (대신 `/home/juns/*.txt` 와 `*.md` 가 그 역할)
- Phase 3 에서 정책 결정 필요: 따를지 / 폐기할지

### 10-3. Branch 식별 불확실성
- 사용자가 말한 "다른 branch" 가 `jekyun` 이면 — junsang 보다 1-commit 옛 (vision world 분리 안 됨). 14m offset 의 원인이 vision world commit 일 가능성?
- 또는 main / remotes/origin/main / 또는 원격에 새 branch
- **Phase 2 진입 전 사용자에게 확인 필수**

### 10-4. ROS2 install 캐시 함정
- src/ 편집 후 install/ 미러 필요 (메모리 `feedback_ros2_install_cache.md`)
- 새 branch pull 시 `colcon build --symlink-install` 또는 매 변경마다 docker cp 필요

### 10-5. PX4 rootfs 휘발성
- container 재생성 시 r0~r3 airframe 다시 cp 필요 (메모리 `project_px4_airframe_sync.md`)
- 새 branch 에서도 적용

---

## 11. Phase 2 분류 확정 (pull 후 작성)

**선택된 branch**: `origin/jekyun_v2` (commit 46bdb17) — 사용자 확인됨.
**공통 부모**: commit 22247d3 (junsang 도 jekyun_v2 도 여기서 fork).
**jekyun_v2 추가 commit 4개**: 09ea871 (안정성), 8ddb82a (reward v2), c38592a (reward v2 orbit fix), 46bdb17 (reward v3).

### 11-1. 우리(junsang) 변경 분류

| 변경 | 분류 | 결정 | 사유 |
|---|---|---|---|
| H1 (reward scale 50→20 등) | (B) | ❌ **폐기** | jekyun v3 는 반대 방향 (50→100), 우리 분석이 잘못된 방향 |
| H2 (action 5d→4d) | (B) | ❌ **폐기** | jekyun_v2 는 5d 유지, jekyun 의 검증된 설계 그대로 |
| H3 (gamma 0.99→0.995) | (B) | ✅ **재적용** | reward 와 무관한 학습론 개선 |
| M1 (buffer 100k→500k) | (B) | ✅ **재적용** | reward 와 무관 |
| M2 (gradient_steps 1→4) | (B) | ✅ **재적용** | reward 와 무관 |
| L6 (eval_freq config) | (B) | ✅ **재적용** | yaml + train_sac.py 수정 |
| N1=B v2 (w_impact=8 등) | (B) | ❌ **완전 폐기** | jekyun 의 RL_analysis 로 per-step 강화 실패 모드 확정 |
| speed_gate 코드 patch | (A) | — | jekyun_v2 yaml 에 `speed_gate_enabled: true` 이미 존재 |
| rew_impact callback | (B) | ✅ **재적용** | 측정 인프라, jekyun_v2 에 없음 |
| total_drop_count 카운터 | (B) | ✅ **재적용** | 측정 인프라 (stair-step 차트) |
| Vision world 분리 (29b1c9b) | (B) | ⏸ **보류** | RL 학습엔 무관, vision 모드 사용 시 필요 |
| PX4 airframe rootfs cp | (B) | ✅ **재적용 필요시** | 컨테이너 재생성 시마다 cp |

### 11-2. jekyun_v2 의 새 기능 (C — 모두 채택)

| 변경 | 채택 사유 |
|---|---|
| `_spin_loop` (resilient spin thread) | **14m offset 의 가장 유력한 원인 해결** — rclpy.spin() 예외 시 자동 재시작 |
| `_reset_depth` recursion guard | CRUISE timeout 무한 재시도 방지 |
| Tiered CRUISE recovery | fast path vs full restart 분리 — 잘못된 recovery 차단 |
| `_kill_infra` pkill 폴백 | **14m offset 의 또 다른 유력 원인 해결** — reused infra 의 옛 PX4 잔재 차단 |
| reward v3 hyperparameter | 360K 학습 실패 분석 후 jekyun 의 검증된 값 |
| `drop_attempt_bonus` 메커니즘 | sparse → dense, drop 시도 자체 보상 |
| `truncation_penalty` 명시 | jekyun v3 의 균형 설계 |
| RL_analysis.md (431줄) | jekyun 의 reward 설계 분석 문서 |
| notes/errors/err_20260520_*.md | spin thread crash 에러 분석 history |

---

## 12. Phase 3-1 실제 적용 결과

**Base**: jekyun_v2 (origin/jekyun_v2 동일).

**변경된 파일** (2 파일, +16-4 줄):

### 12-1. `hyperparams.yaml`

```diff
training:
+  total_timesteps: 200000   (1000000 → 200000, 200k 우선 검증)
+  eval_freq: 10000          (NEW, L6)
+  eval_episodes: 3          (NEW, L6)

sac:
+  buffer_size: 500000       (100000 → 500000, M1)
+  gamma: 0.995              (0.99 → 0.995, H3)
+  gradient_steps: 4         (1 → 4, M2)

wandb:
+  run_name: "junsang_v2"    ("L4-AutoDrop-v2" → "junsang_v2")
```

### 12-2. `train_sac.py`

- `_step_rew_impact` list 추가 (`__init__`)
- `_total_drop_count` counter 추가 (`__init__`)
- `_on_step` 에서 `rew_impact` 수집
- `_on_step` 의 drop 카운트 시점에 `_total_drop_count += 1`
- `_on_rollout_end` 의 매핑 테이블에 `(_step_rew_impact, env/mean_rew_impact)` 추가
- `_on_rollout_end` 에 `log_dict['env/total_drop_count'] = self._total_drop_count` (매 rollout)
- `main()` 에서 `eval_freq = cfg_train.get('eval_freq', 10_000)` 로 로드
- `BestModelCallback(eval_freq=eval_freq)` 로 전달

### 12-3. `drone_drop_env.py`

**변경 없음** — jekyun_v2 그대로. spin thread + reset guard + tiered recovery + infra kill 강화 모두 포함.

### 12-4. install/share 미러링

```
ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml ✓
ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py ✓
ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py ✓
```

---

## 13. Phase 3-2 갱신된 문서/메모리

- `/home/juns/meeting_notes_2026-05-22.txt` — PART 5 추가 (section 1-13 ~ 1-18)
- `/home/juns/change_inventory_pre_pull_2026-05-22.md` — 이 문서 (Phase 2/3 결과 반영)
- 메모리 4 개 변경:
    - `project_n1b_200k_training.md` — DEPRECATED 표시, 실패 결론
    - `project_junsang_v2_setup_2026-05-22.md` (NEW) — 현재 상태
    - `feedback_simulation_offset_root_cause.md` (NEW) — 14m offset 진단 방법론
    - `MEMORY.md` — 인덱스 갱신

남은 갱신 (학습 결과 본 후로 미룸):
- `/home/juns/parameter_조절_의사결정_guide.md` — jekyun reward 원칙 추가 + CCIP-Sim mismatch 케이스
- `/home/juns/A_phased_curriculum_도입방안.md` — jekyun_v3 base 에서 A 도입 시 차이점

---

## 14. 다음 단계 (Phase 3-3 — 새 학습 사이클)

1. **dry-run 5K** (yaml total_timesteps 임시 5000 → 학습 → 결과 확인 → 200000 복원)
    - 목표: jekyun RL_analysis 의 "mean_d_xy ≤ 5m" 도달 + silent crash 없음
    - 명령:
        ```
        export WANDB_MODE=offline    # dry-run 은 offline
        ros2 run rl_navigation train_sac
        ```
2. **본학습 200K** — dry-run 성공 시
3. **모니터링** (WandB):
    - `env/total_drop_count` (stair-step 증가)
    - `env/mean_rew_impact` (jekyun_v3: w_impact=0.4 이라 작은 값 예상)
    - `env/mean_d_xy`, `env/mean_d_impact`, `env/drop_error_actual_m`
4. **200K 완주 후 결정** ([[project_junsang_v2_setup_2026-05-22]] 참조)

---

끝.
