---
date: 2026-09-13
tags: [research, residual, l1, rl, ppo, preflight, audit]
status: active
type: research
---

# L1-RL 사전점검 — 잔차 강화학습을 돌리기 전에 바꿔야 하는 것

> **한 줄.** 지금 코드로 `train.py --resume L0 --zero_init_residual --freeze_nominal`을
> 돌리면 **세 가지 이유로 L1-SL과 비교할 수 없는 run**이 나온다: ① 조준 보상이 잔차를
> 먹는다(보상 해킹 통로, 08-30 미수정), ② RL 잔차의 입력이 26채널이라 L1-SL(38채널)보다
> **정보가 적다** — 같은 정보량의 SL 팔은 이미 L0에게 진다, ③ 잔차 채널에 평활이 없어
> 첫 교차 게이트의 요동 편향(Rule 38)을 RL만 떠안는다. 셋 다 "RL이냐 SL이냐"와 무관한
> 이유로 RL 팔을 지게 만든다. 코드 감사만 했고 **아무것도 고치지 않았다.**

관련: [[research/residual_ceiling]] §7.4·§8 · [[research/research_architecture]] §7.6.6·§8.4 ·
[[errors/err_20260830_aim_reward_residual_inclusive]] · [[research/release_gate_jitter]] ·
[[research/residual_observability]] · [[experiments/exp_030_gain_invariant_residual]] ·
[[research/rl_rules]] (Rule 29·38·39·40·41)

---

## 0. 비교의 목적을 먼저 고정한다

[[research/research_architecture]] §7.6.6이 남긴 질문은 하나다:

> **종단 보상(실제 착지 오차)이 드리프트 예측(MSE) 위에 더할 것이 있는가.**

이 질문에 답하려면 두 팔이 **학습 신호만** 달라야 한다. 나머지는 전부 같아야 한다:

| 조건 | L1-SL gi (확보됨) | L1-RL (목표) |
|---|---|---|
| 비행·투하 결정 | 동결 L0 seed 1 | **동결 L0 seed 1** |
| 잔차 입력 | obs 26 + tilt 누적 10 + gi 2 = **38** | **38** (같은 정의) |
| 잔차 망 | 38→128→128→2, ELU | **같은 구조** (SL 가중치로 초기화 가능) |
| 잔차 권한 `residual.scale` | **2.0 m** (`oracle_scale`) | **2.0 m** |
| 시간 평활 | EMA α=0.3 | **EMA α=0.3** |
| 학습 신호 | MSE on $\delta^\* = x_{real}-x_{nominal}$ | **PPO on 종단 보상** |
| 평가 | seed {3000,4000,5000}×200 paired, DR 1.5, 사거리 18–22 | **동일** |

오른쪽 열의 굵은 항목 중 **현재 코드가 만족하는 것은 하나도 없다.** 아래는 그 목록이다.

---

## 1. 필수 수정 — 이것 없이는 run이 무효

### 1.1 ⛔ 조준 포텐셜이 잔차 포함 오차를 먹는다 (08-30 발견, 미수정)

`task_env.py`: `_get_dones` L919 `_, d_impact, _ = self._ccip(pos, vel)` → `self._d_impact`
→ `_get_rewards` L1053 `aim = w_ccip * (exp(-k d_impact) - exp(-k d_impact_prev))`.
`_ccip` L669는 `residual.enabled`면 `apply_ccip_residual`을 적용한다.

즉 **잔차를 표적 쪽으로 내밀기만 하면 조준 보상(+30 상한)을 받는다.** base env
`drone_bombard_env.py` L1822에는 exp_018의 가드가 있는데 08-27 재구축 때 빠졌다.

**처방.** `_ccip`에서 공칭 예측 거리를 따로 캐시(`_d_impact_nominal`)하고
`_get_rewards`의 `aim`과 `_reset_idx` L882의 `_d_impact_prev` 초기값이 그것을 쓴다.
**게이트(`release_gate`)·교차 판정(`crossed`)·관측 채널은 잔차 포함 그대로** —
릴리즈 결정을 편향시키는 것이 잔차의 일이다.

부수 확인: `gate_reward`(1.0/step, 잔차 포함 게이트 개방 시)는 `w_loiter`(1.0×연속 스텝)가
두 번째 스텝부터 상쇄하므로 farming 가치가 없다. `drop_signal_reward`(20, 사건 1회)는
투하 확정이 동결 정책 몫이라 잔차가 만들 수 없다. **조준 항 하나만 고치면 된다.**

### 1.2 ⛔ RL 잔차의 입력이 L1-SL보다 12채널 적다 — 정보 불공정

RL 잔차는 정책 MLP의 출력이라 입력이 관측 26채널뿐이다. L1-SL의 이득은
tilt 누적 10 + gi 2채널에서 나온다 ([[experiments/exp_030_gain_invariant_residual]] 채널 분해:
누적 12채널이 $R^2$ 손실의 87%).

**같은 정보량의 SL 팔은 이미 L0에게 진다** ([[experiments/exp_028_l1_sl_pilot]] §4):

| 팔 (DR 1.5, n=600) | CEP50 | vs L0 |
|---|---|---|
| L0 | 0.305 | — |
| SL **obs만**, EMA 0.3 | 0.329 | **+7.8% (순손실)** |
| SL obs+tilt+gi, EMA 0.3 | 0.209 | −31.6% |

→ 26채널 RL 잔차는 학습 방식과 무관하게 **정보 상한이 $R^2$ 0.44**이고, 그 수준에서
결과 공간 잔차는 손해다. 이대로 돌리면 "RL이 안 된다"가 아니라 "정보를 덜 줬다"를 재는 것이다.

**처방.** `task_env._get_observations`에 누적 12채널을 **맨 뒤에** 붙이는 옵션
(`cfg.accum_obs`, observation_space 26 → 38). 정의는 `play.py::_SLResidual` /
`_fit_sl_residual.tilt_features`와 **비트 동일**해야 한다(이미 한 번 갈라진 적이 있다 —
`play.py` L407 주석). 리셋은 `_reset_idx`, $v_0$는 리셋 후 첫 관측에서 래치.
`obs[:26]`은 그대로이므로 L0 체크포인트와의 호환은 §1.4의 구조로 확보한다.

### 1.3 ⛔ 잔차 채널에 평활이 없다 — Rule 38을 RL만 떠안는다

L1-SL은 `play.py --sl_ema 0.3`으로 주입 전에 EMA를 건다. RL 잔차는 `_pre_physics_step`
L705에서 그대로 `_ccip`에 들어간다. 학습 중에는 탐험 잡음(σ 초기 0.8 × scale)이
그대로 게이트를 흔들고, 첫 교차 판정은 그 요동의 극값을 표집해 **계통적으로 이른 릴리즈**를
만든다 ([[research/release_gate_jitter]]). 이것은 [[research/research_architecture]] §7.6.6이
예고한 대가 ①이고, 평활 없이는 RL 팔이 지는 이유가 정보인지 매끄러움인지 분리할 수 없다.

**처방.** `ResidualCfg.ema_alpha`(기본 1.0 = 끔)를 두고 `_pre_physics_step`에서
잔차 채널에 EMA 적용(에피소드 시작 시 원값으로 리셋 — `_ResidualEMA`와 같은 규칙).
학습·평가 모두 같은 값(0.3). L1-SL 평가는 기존대로 `--sl_ema 0.3` + env EMA 끔
(두 경로가 같은 규칙이므로 등가; 겹쳐 걸면 이중 평활).

### 1.4 ⛔ 동결 팔에 잔차 트렁크가 없다 — `--freeze_nominal`의 용량 결함 + §1.2와 충돌

`train.py::_prepare_residual_head`는 트렁크 전체를 얼리고 **출력층의 잔차 행만** 학습시킨다
([[research/residual_ceiling]] §5가 이미 지적). §1.2로 입력을 38로 늘려도 **첫 층의 새 열이
얼어 있으면 잔차는 누적 채널을 볼 수 없다.** 비동결로 풀면 비행 정책이 바뀌어 L1-SL과의
귀속("오직 잔차")이 깨진다.

**처방 — 별도 잔차망 (Johannink식, [[research/residual_ceiling]] §6.2).**
행동 = `[nominal(obs[:26])[0:5], residual_mlp(obs[:38])]`. nominal은 L0 체크포인트에서
로드해 동결, residual_mlp는 38→128→128→2 ELU. rsl_rl 3.1.2(컨테이너 실측 버전;
`train.py` 주석의 "v2.3.2"는 낡았다)에서는 `runner.load()` 뒤 `runner.alg.policy.actor`를
합성 모듈로 교체하고 **`runner.alg.optimizer`를 다시 만든다**(PPO가 생성자에서
`Adam(policy.parameters())`를 잡아두므로). 크리틱은 26→38 입력을 0열로 넓혀 L0 값함수를
초기값으로 쓴다. `play.py::_load_policy`도 같은 합성 모듈을 만들어야 L1-RL 체크포인트를
읽는다 → 빌더를 `drone_bombard/` 안에 한 곳에 둔다.

이 구조의 부산물: **잔차망 구조가 SL 회귀기와 같으므로 `res_gi.pt`의 가중치로 초기화할 수
있다**(정규화 `mu/sd`는 첫 층에 흡수). 그러면 §0의 질문에 가장 직접적인 팔이 생긴다 —
*"L1-SL에서 출발해 PPO로 미세조정하면 더 좋아지는가"* ([[research/residual_ceiling]] §7.3 Stage 2).

### 1.5 ⚠️ 감사 중 새로 발견 — 동결 행이 Adam 모멘텀으로 움직인다

`runner.load(path)`는 기본 `load_optimizer=True`라 L0의 Adam 상태(1·2차 모멘트)를 복원한다.
`--freeze_nominal`은 출력층 nominal 행의 **그래디언트를 0으로 마스킹**할 뿐 파라미터를
옵티마이저에서 빼지 않으므로, grad가 0이어도 Adam은 복원된 `exp_avg`로 스텝을 밟는다
(`p.grad is None`일 때만 건너뛴다). 크기는 lr × Σβ₁ᵏ × m/√v ≈ 수 10⁻³ 정도로 작지만
**"nominal 출력 bit-identical"이라는 test_residual_head의 보장이 실제 run에서는 성립하지
않는다** — 테스트는 상태가 빈 새 Adam으로 검사한다. 트렁크는 `requires_grad=False`라
grad가 None이므로 안전하다.

**처방.** L1 warm-start에서는 `runner.load(path, load_optimizer=False)`. §1.4 구조로 가면
nominal이 옵티마이저 밖에 있으므로 자동 해결되지만, 현행 경로를 쓸 경우 필수.
부수: `load()`가 `current_learning_iteration`을 1000으로 복원하므로 wandb x축이 1000부터 시작한다.

### 1.6 ⛔ 평가 경로에 학습 잔차의 권한을 맞출 스위치가 없다

`play.py` L759는 `--sl_residual`/`--oracle_residual`일 때만 `residual.scale = oracle_scale`(2.0)로
올린다. L1-RL 정책은 자기 잔차를 쓰므로 그 분기를 타지 않아 **1.0 m 권한으로 평가**된다 —
L1-SL(2.0)과 클램프가 다르다(DR 2.5·외삽 행에서 특히). `--residual_scale` 인자(학습·평가 공통)와
`--accum_obs`가 필요하다. 평가 명령은 `_gi_headline.sh`의 조건을 그대로 복제한다.

---

## 2. 파라미터 결정 (공정성 조건)

| 항목 | 값 | 근거 |
|---|---|---|
| `residual.scale` | **2.0** (학습·평가) | L1-SL·오라클과 같은 클램프. 08-29의 "2.0이 크다"는 논거는 무작위 초기화 전제였고, 0 초기화/SL 초기화면 소멸 |
| `w_residual` | **0** (본 팔) | SL에는 수축 항이 없다. §1.1 수정 후 남는 farming 통로가 없으므로 정당화 근거도 사라짐. 0.1 팔은 ablation |
| 잔차 dims 탐험 σ 초기값 | **0.2 이하** (`--residual_init_std`) | 0.8 × 2.0 m = 1.6 m RMS는 T2 착탄 오차의 2배. EMA 0.3이 √(α/(2−α)) ≈ 0.42배로 줄여도 0.67 m |
| 초기화 | 두 팔: **0 초기화** / **`res_gi.pt` 초기화** | 전자 = "RL이 찾는가", 후자 = "종단 보상이 더하는가"(§0의 질문) |
| 반복 수 | 500 iter 우선, 2048 envs | L0 1000 iter = **4 h 16 m 실측**(model_0 03:50 → final 08:06). 잔차만 학습이면 500 iter까지 CEP50 −15% 미달 시 중단([[research/residual_ceiling]] §8) |
| 조기 중단 | Rule 29 | 보상↑·과제지표 평탄·σ 단조↑ 200~300 iter |
| 판정 지표 | CEP50 **·CEP90·succ@0.5·succ@1.0**·배달시간·투하 v | Rule 40 — CEP50 하나로 판정 금지 |

> 관측 채널 2–4(CCIP 오차·착탄 오차 크기)는 **잔차 포함**이라 동결 L0도 잔차가 켜지면
> 관측이 달라져 비행이 미세하게 바뀐다. L1-SL 결과도 같은 조건이었으므로(배달률·추락 동일,
> [[research/release_gate_jitter]] §1) 비교에는 영향이 없다. 한 줄 caveat으로만 적는다.

---

## 3. 인프라 사전점검 (2026-09-13 실측)

| 항목 | 상태 |
|---|---|
| `isaac-verify` 컨테이너 | 6일 전 Exited(137) → `docker start`로 복구 ✅. `/tmp/rebuild` 보존 |
| `/tmp/rebuild` ↔ repo `isaac_lab/` | **전 파일 md5 일치** ✅ (수정 후 `docker cp` 필수) |
| L0 체크포인트 | seed 1 `/tmp/l0b/.../model_final.pt`, seed 2 `/tmp/l0_s2/...` ✅. **seed 3은 model_50까지만** — 미완 |
| SL 산출물 | `/tmp/sl/res_gi.pt`(본 방법) · `res_tilt*.pt` · 덤프 4종 ✅ |
| GPU | L4, 0 MiB 사용, 고아 프로세스 없음 ✅ |
| wandb | `docker start`만 했으므로 키 없음 → `docker exec --env-file /opt/drone-bombard/.wandb.env` 필수 (train.py 사전검사가 막는다) |
| rsl_rl | **3.1.2** (`train.py` 주석은 2.3.2). `runner.alg.policy`·`policy.std`·`act_inference` 가정은 유효 |

---

## 4. 문서 갱신 대상 (수정과 함께)

- [[errors/err_20260830_aim_reward_residual_inclusive]] status open → resolved
- [[research/reward_design]] §2.4 첫 행 · [[research/research_architecture]] §8.4·§9.1(L1-RL 정의를
  "별도 잔차망 + 동결 L0"로) · [[research/residual_ceiling]] §8("새 코드 0줄" 계획은 폐기)
- `tests/test_residual_head.py` — 합성 모듈 기준으로 재작성 + §1.5의 옵티마이저 상태 검사 추가
- `RL_Project_Log.md` §3 5·8번

---

## 5. 실행 순서

1. §1.1 → §1.3 → §1.2 → §1.4/1.5/1.6 순으로 수정, `docker cp`, 단위테스트
2. **등가 검사 1회**: `--accum_obs`로 한 롤아웃 돌려 `obs[:, 26:38]`이 `_SLResidual`의 누적과
   비트 동일한지 assert (GPU 1분)
3. **dry-run**: 0 초기화 팔 3 iter → `Episode_Reward/rew_aim_pot`이 L0와 같은 자릿수인지,
   `residual_mag` 로그가 0 근처에서 출발하는지
4. 파일럿 2팔 × 500 iter (0 초기화 / SL 초기화), seed 1
5. 평가 `_gi_headline.sh` 조건 복제 → `_agg_table1.py`(시드셋 assert)로 L0 · L1-SL gi · L1-RL × 2 · 오라클 한 표
