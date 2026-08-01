---
date: 2026-07-05
tags: [daily, log, wobble, regression, v15, stagnation, training-stability, gpu-driver, infra]
type: daily
status: active
---

# 연구 일지 — 2026-07-05

---

## 오늘 한 일

> v15_bc_stable(wobble 교정 모델)이 X마커에 도달하지 못한다는 사용자 관찰 진단(코드 변경 없음, 원인 분석만).

- v15 학습 이력 재구성: `45f0a63`(B+C+LPF 도입) → 잘못된 base config로 첫 크래시(41qq0tpd, `af8e2f3`로 수정) → **반복 reset-recursion abort**로 `run_train_supervised.sh` 오토레쥼 서포바이저 추가(`3d7e2b6`) → 0→310K, 2026-07-03 05:21 preempt(`sac_drop_preempt.zip`, `sac_drop_310000_steps.zip`).
- 현재 `hyperparams_v13.yaml`의 실제 wobble-fix 값 확인: `w_vel: 0.08`, `vel_damp_radius: 3.0`, `w_ang_vel: 0.15`, `w_action_smooth: 0.20`, `velocity_lpf_alpha: 0.4`.
- v15 310K 체크포인트에 대한 **공식 eval 기록이 전무함**을 확인 — exp_011의 "다음 확인" 항목(success ≥ v13 80%, jerk RMS 재측정)이 한 번도 완료되지 않은 채 preempt됨.
- `run_train_supervised.sh` 내부 주석에서 resume 시 **replay buffer가 매번 초기화**된다는 사실 확인.
- **진단 확정을 위해 `evaluate.py`를 `sac_drop_310000_steps.zip`(`hyperparams_v13.yaml`, 20-ep)로 실행 시도 → infra 자체가 죽어있어 실패.** 원인 규명 결과 **GPU 드라이버 불일치**(아래) 발견 — 원인 1/2 진단은 여전히 미확정 상태로 남음.

---

## 주요 결정 & 발견

> 원인 후보 2건 확정(둘 다 evaluate.py 재실행으로 검증 필요, 아직 미확정).

### 원인 1 — 근접-속도 댐핑(B)이 v14의 기존 취약점을 정확히 재타격

- v14(65%)의 지배적 실패 모드는 **final-approach stagnation** — 0.5–1.2 m 구간에서 정체(`exp_010`).
- wobble 교정으로 추가한 `r3_vel = -w_vel · speed_xy · max(0, 1 - d_xy/vel_damp_radius)`는 `vel_damp_radius=3.0m`로, `success_radius=0.8m`보다 훨씬 넓은 반경 전체에 걸쳐 "가까울수록 감속" 유인을 준다.
- 즉 **정체가 이미 벌어지던 바로 그 구간(0.5–1.2m)에 "더 느리게 가라"는 보상이 추가로 걸려 있음.** exp_011 자체에서도 "w_vel↔stagnation tension"으로 이미 예견됐던 리스크였고, 0.15→0.08로 완화만 했을 뿐 근본적으로 해소하지 않았음.
- `w_ang_vel 0.15`, `w_action_smooth 0.20` 상향도 방향은 같음: 종단에서 필요한 급격한 보정 기동(마지막 gap을 closing하는 액션)에 추가 비용을 물려, wobble은 줄이되 "끝까지 파고드는" 유인도 같이 줄였을 가능성.

### 원인 2 — 반복 크래시 + resume 시 replay buffer 초기화로 학습 불연속

- `run_train_supervised.sh` 주석: "Periodic checkpoints save weights only (no replay buffer), so each resume restarts SAC's buffer but keeps the learned policy."
- 서포바이저가 추가된 이유 자체가 "recurring reset-recursion abort"(반복 발생)였음 — 즉 이 런은 한 번이 아니라 여러 번 중단·재개를 거쳤을 가능성이 높음.
- crash마다 replay buffer가 비워지면 SAC은 매번 정책은 유지한 채 경험을 처음부터 다시 채워야 함 → 최종 스텝 카운트(310K)가 시사하는 것보다 **실질적으로 수렴에 쓰인 연속 학습량은 적을 수 있음**.
- 몇 번 재개됐는지는 아직 정량화 안 됨(로그 미확인) — 다음 확인 필요.

### 아직 확정 안 된 것

- 두 원인 중 무엇이 지배적인지, 혹은 둘 다인지는 **evaluate.py 재실행(outcome breakdown) 없이는 확정 불가**. 실패가 대부분 `stagnation`이면 원인 1이 지배적, `timeout`/불규칙 패턴이 섞여 있으면 원인 2(학습 불연속)도 유의미하게 기여했을 가능성.

### ⚠️ 새로 발견 — `drone-bombard-harmonic` 컨테이너 GPU 드라이버 불일치로 인프라 자체가 죽어 있음

**진단 확정 위해 `evaluate.py --model rl_checkpoints/sac_drop_310000_steps.zip --episodes 20 --config hyperparams_v13.yaml`를 실행했으나, 인프라가 PX4 position data를 못 받아 CRUISE timeout → full restart ×3 → forced reset을 반복(v15 학습 때와 동일한 reset-recursion 패턴).** 원인 규명:

- `nvidia-smi`(컨테이너 내부): `Failed to initialize NVML: Driver/library version mismatch` — 컨테이너 NVML lib **535.309**, 호스트 드라이버 **580.159.03**(Isaac Lab용으로 업그레이드됨). `torch.cuda.is_available()` → `False`.
- `gz sim -s -r --headless-rendering ...`를 단독 실행 → **45초간 로그 한 줄도 안 찍힘**(정상이면 즉시 시작 배너 출력), `gz topic -l`은 `timeout 8` 래핑에도 **2분간 응답 없이 행(hang)** — Gazebo 코어(gz-transport) 자체가 정상 기동을 못 하고 있음. CUDA(추론용)만이 아니라 **gz sim의 헤드리스 GPU 렌더링(Ogre2/EGL) 경로 자체가 깨졌을 가능성**이 높음.
- **기존 가정 정정:** v15가 310K에서 "preempt"된 것을 지난 세션(07-01)엔 정상적인 사용자 stop으로 기록했으나, Isaac 트랙 메모리 노트에 따르면 **07-03 GPU 드라이버 업그레이드 도중 학습이 강제 중단됐음**이 이미 기록돼 있었다(`project_state.md`: "v15_bc_stable 학습은 07-03 driver 업그레이드 때 중단됨"). 즉 310K 정지는 계획된 eval-용 stop이 아니라 **드라이버 업그레이드가 컨테이너를 깨뜨려 강제로 죽은 것** — 그 이후로 지금까지 이 컨테이너로는 Gazebo 기반 학습/eval이 전혀 검증되지 않았다.
- **결론:** 지금 이 문제는 wobble 보상 shaping과는 **완전히 별개의, 더 우선순위 높은 차단 이슈**다. 인프라(드라이버/컨테이너)가 고쳐지기 전엔 v15의 실제 성능(원인 1/2 어느 쪽이 맞는지)을 확인할 방법이 없다.
- **사용자 결정:** 드라이버/컨테이너 정합은 사용자가 직접 처리하기로 함(Isaac 트랙과의 GPU 공유 문제이므로). Claude는 컨테이너 이미지·드라이버 변경을 보류하고, 인프라 복구 확인 후 eval 재시도.

---

## 코드 변경 사항

없음 — 이번 세션은 진단만 수행. 아래 "내일 할 일"이 다음 세션에서 적용할 변경안.

---

## 문제 & 해결

| 문제 | 해결 여부 | 메모 |
|------|----------|------|
| v15(310K) 훈련된 에이전트가 X마커에 도달 못 함 | ❌ 미해결(원인 후보 2건 규명, 확정 전) | 원인 1: `w_vel`(근접 속도 댐핑)이 v14 stagnation 구간(0.5–1.2m)을 재타격. 원인 2: 반복 crash-resume에서 replay buffer 초기화로 학습 불연속. 확정 필요 |
| v15 310K 체크포인트 정식 eval 누락 | ⚠️ open | exp_011의 "다음 확인" 항목이 preempt 후 한 번도 실행 안 됨 |
| supervisor resume가 replay buffer 안 지킴 | ⚠️ open (설계상 한계) | `run_train_supervised.sh`가 `--resume`으로 가중치만 복원 — 향후 crash-resume 안정성 위해 buffer도 저장/복원 고려 |
| **`drone-bombard-harmonic` 컨테이너 GPU 드라이버 불일치로 gz sim 기동 불가** | ❌ 미해결 (사용자가 직접 처리 예정) | 호스트 드라이버 580 vs 컨테이너 NVML/CUDA lib 535(Isaac Lab 업그레이드 여파). `evaluate.py` 실행 시도가 v15 학습과 동일한 reset-recursion 패턴으로 실패 → **v15의 310K 정지는 계획된 eval-stop이 아니라 이 드라이버 업그레이드에 의한 강제 중단이었음**(정정). 이게 풀리기 전엔 원인 1/2 확정도, 어떤 Gazebo 기반 학습/eval도 불가 |

---

## 내일 할 일 — 다음 학습 개선을 위한 변경 제안 (우선순위 순)

0. **[ ] (차단 이슈, 최우선) `drone-bombard-harmonic` 컨테이너 GPU 드라이버/CUDA 정합** — 사용자가 직접 처리. 호스트 드라이버 580에 맞게 컨테이너 NVML/CUDA userspace lib 갱신하거나 Isaac 트랙과 GPU/드라이버 사용 조율. 이게 끝나야 아래 1번(eval)이 실행 가능.

1. **[ ] 진단 확정: v15 310K(또는 preempt) 체크포인트로 `evaluate.py` 20+ep 재실행**
   - outcome breakdown(success/stagnation/timeout) + closest d_xy 분포 확인.
   - `vel_logger.py`로 jerk RMS 재측정(wobble이 실제로 줄었는지 확인 — 교정 자체가 무효화된 건 아닌지 분리해서 봐야 함).
   - 실패가 `stagnation` 지배적이면 원인 1 확정, 아니라면 원인 2 쪽에 무게.

2. **[ ] 원인 1 수정: 근접-속도 댐핑을 종단 접근 반경 밖으로 빼거나 제거**
   - `vel_damp_radius: 3.0 → ~1.0` (success_radius 0.8m보다 살짝 큰 정도로 축소) — "멀리서는 자유, 아주 가까이서만 감속"이라는 원래 의도를 실제로 구현.
   - 또는 fallback으로 `w_vel: 0.08 → 0.0`(완전 제거)하고 wobble 억제는 C(smoothness 가중)+LPF만으로 감당되는지 우선 확인. C+LPF만으로 jerk RMS가 충분히 낮다면 B는 애초에 불필요한 리스크였다는 뜻.
   - 어느 쪽이든 **stagnation 재발 여부를 최우선으로 보면서 튜닝** — v14가 이미 겪은 실패 모드이므로 "이번엔 안 그러겠지"가 아니라 명시적으로 검증.

3. **[ ] 학습 연속성 확인: supervisor 재개 횟수 정량화**
   - `/tmp/train_bc.log`(컨테이너 내부, `[SUPERVISOR] attempt N`) grep으로 실제 재개 횟수 확인.
   - wandb run history에서 reward/step 곡선에 불연속(급락 후 재상승) 패턴이 있는지 확인 — 있다면 매 resume이 실제로 policy를 훼손했다는 방증.

4. **[ ] (구조적, 우선순위 낮음) crash-resume 시 replay buffer도 보존하도록 체크포인트 로직 개선**
   - 지금은 가중치만 저장 → 향후 같은 종류의 반복 크래시가 나도 학습 연속성이 깨지지 않도록. `save_replay_buffer=True` 옵션 등 SB3 체크포인트 콜백 확인.
   - 이번 회귀의 근본 수정이라기보다, "reset-recursion abort" 자체가 재발할 걸 가정한 방어책.

5. **[ ] 위 2번 변경 적용 시 fresh start 여부 결정**
   - `w_vel`/`vel_damp_radius` 변경은 보상 공식 변경이므로 원칙적으로 fresh 권장(CLAUDE.md 규칙).
   - 다만 빠른 신호 확인을 위해, 먼저 현재 체크포인트에서 짧은 dry-run(1–2K step) A/B로 stagnation 완화 여부만 gate-check 하고, 통과하면 fresh 300K로 정식 재학습.

---

## 관련 노트

- [[experiments/exp_011_wobble_lpf_reward_damping]] — 오늘 진단이 확장하는 원 실험 노트
- [[research/control_smoothness_wobble]] — wobble 진단 방법론
- [[research/rl_rules]] Rule 15(wobble) / Rule 25(오늘 추가 — 종단 반경 내 보상 shaping 리스크)
- [[research/terminal_overshoot_trap]] / [[experiments/exp_010_byxyaf4d_v14_195k_eval]] — v14 final-approach stagnation 원 진단
- [[experiments/training_history]] · [[00_index]]
