---
date: 2026-08-03
tags: [experiment, isaac-lab, baseline, p0, table1, ccip, aerothrow, eval-harness, cep]
status: done
type: experiment
wandb_run: "none (deterministic paired evals)"
---

# exp_023 — Table 1 1차: 규칙 기반 릴리스 베이스라인 vs 학습 정책

> [[research/paper_research_plan]] §5 **P0-2**(평가 하네스) + **P0-6**(무학습 베이스라인) 실행 +
> §4 Table 1의 **첫 실측**. 논문에서 가장 큰 구멍이었던 "RL이 왜 필요한가"에 처음으로 수치를 준다.

## 1. 도구 (P0-2)

### `isaac_lab/eval_harness.py` — 모든 arm이 공유하는 단일 평가 경로

`play.py --policy`(학습 arm)와 `baseline_drop.py`(규칙 arm)가 **같은 collector·같은 지표 정의·같은
JSON 스키마**를 쓴다. 표의 행끼리 코드가 달라 생기는 드리프트가 원천 차단된다.

- **Paired 평가**: 같은 시드만으로는 두 arm이 같은 에피소드를 보지 못한다 — 에피소드 랜덤화는
  `_reset_idx`에서 뽑히고 **어느 env가 언제 리셋되는지는 정책에 따라 달라져** RNG 스트림이 갈라진다.
  해결은 영리한 방법이 아니라 구조적인 것: **`num_envs == episodes`로 돌리고 각 env 슬롯의
  *첫 번째* 에피소드만 채점한다.** 첫 리셋은 어떤 행동보다도 먼저 일어나므로 모든 arm이
  **bit-identical한 초기조건 집합**(핸드오프·마커·바람·항력·질량신념·게인·bias)을 본다.
- **CEP50/CEP90**: 착탄 오차의 50/90 분위 — 에어드롭 문헌의 표준 언어. 항상 **delivery_rate와 함께**
  보고한다(5% 투하율의 0.1 m CEP는 좋은 시스템이 아니다).
- **T_deliver**(에피소드 시작 → 채점 이벤트) + **feasible release window**(발화가 허용된 구간 길이).
- **정직한 구간**: 비율은 Wilson 구간, 오차는 부트스트랩(고정 generator라 재현 가능).
- **`released_not_delivered`** 카운터 — 이게 §2의 버그를 잡아냈다.

### `isaac_lab/baseline_drop.py` — T0~T3 (P0-6)

네 arm이 **같은 접근 컨트롤러**(블라인드 순항 → 지각된 표적으로 P 제어 → 릴리스 고도대로 하강)를
공유하고 **릴리스 규칙만** 다르다. 모두 **동일한 릴리스 엔벨로프**를 통과해야 한다(게이트 완화 없음).

| arm | 규칙 | 사용 정보 |
|---|---|---|
| **T0** `hover` | 표적 위에서 정지 후 투하 | 지각 표적만 |
| **T1** `ccip` | CCIP 오차가 릴리스 반경에 들어온 **첫 순간** 발화 | 지각 표적만 |
| **T2** `argmin` | 수평선 내 예측 상태 전부의 탄착오차를 평가, **argmin이 현재 스텝일 때** 발화 (AeroThrow Alg.1 포팅) | 지각 표적만 |
| **T3** `oracle` | T2 + **참 바람/항력으로 계산한 해석적 잔차**를 잔차 채널로 방출 + 그만큼 upwind 접근 | **특권 정보** |

T3는 "학습 잔차가 발견하려는 바로 그 보정"을 해석적으로 준 것 = **잔차 메커니즘의 고전적 상한**.
잔차 권한 ±`residual_scale`도 그대로 적용되므로 드리프트가 그 이상이면 학습 잔차와 **똑같이 포화**한다.

## 2. 도중 발견 — 물리 페이로드 착지 래치 버그

하네스의 `released_not_delivered`가 처음부터 큰 값을 보고했다(released 75% / delivered 31%).
추적 결과 **정지한 페이로드는 착지 판정을 영원히 통과하지 못한다**(실린더 half-height 0.03 m > 판정면 0.0).
32-env 강제 릴리스 프로브에서 **래치 0/32**, 도달 최소 z가 정확히 0.0300 m.
`payload_land_eps = 0.10`(exp_019가 parity 검증한 값)으로 수정 → **32/32**.
**v19 정책 @ v19 성공률이 91.0% → 100.00%로 정정**되었다.
→ [[errors/err_20260803_payload_landing_latch]] · [[research/rl_rules]] Rule 28.

> 베이스라인을 만들지 않았다면 이 버그는 계속 "정책 실패"로 오독되었을 것이다 —
> 무학습 arm은 정책과 무관한 기준점이라 계측 결함을 드러낸다.

## 3. Table 1 (1차 실측)

**프로토콜:** `Isaac-DroneBombard-V19-Direct-v0`(= 정책이 학습된 분포), **paired 200 ep**,
seed **1000**(학습 시드 42/43/44와 분리), num_envs 200, deterministic.
성공 반경 1.0 m, 착탄은 **실제 물리 낙하**로 채점.

| # | arm | 학습 | success (95% CI) | delivery | CEP50 | CEP90 | T_deliver med | 비고 |
|---|---|---|---|---|---|---|---|---|
| T0 | hover-drop | ✗ | 91.50% (86.8–94.6) | 97.0% | 0.358 m | 0.733 m | **7.90 s** | 정확하지만 **가장 느림** |
| T1 | CCIP 임계 발화 | ✗ | 6.50% (3.8–10.8) | 97.0% | 1.606 m | 2.164 m | 5.50 s | 첫 허용 순간 발화 |
| T2 | predictive argmin | ✗ | 35.50% (29.2–42.4) | 97.0% | 1.097 m | 1.603 m | 5.80 s | AeroThrow Alg.1 |
| T3 | wind-oracle residual | ✗ (**특권**) | 47.50% (40.7–54.4) | 97.0% | 1.010 m | 1.836 m | 5.90 s | 참 바람 사용 |
| **T5** | **ours (RL)** | ✓ | **100.00%** (98.1–100) | **100%** | **0.370 m** | **0.679 m** | 5.95 s | v19 precise |

### 읽는 법

- **T1 → T2 (+29.0 pp, CEP −32%)**: 고전 타이밍 정교화의 순수 이득. AeroThrow의 기여를 우리 env에서 재현.
- **T2 → T3 (+12.0 pp, CEP −8%)**: **참 바람을 알려줘도** 여기까지다. 잔차 권한 ±2 m와
  게이트가 상한을 만든다.
- **T3 → T5 (+52.5 pp, CEP −63%)**: **학습 정책이 특권 정보 고전 arm을 성공률 2.1배·CEP 2.7배로 이긴다.**
  이것이 "RL이 무엇을 사는가"의 첫 정량 답이다.
- **T0의 존재 이유**: 호버-드롭은 CEP 0.358 m로 **T5와 대등한 정확도**를 낸다 — 대신 **33% 느리다**
  (7.90 vs 5.95 s). 즉 **T5의 대(對)호버 우위는 정확도가 아니라 민첩성**이고,
  **대(對)순항-릴리스(T1–T3) 우위는 정확도**다. 이 구분 없이 "RL이 더 정확하다"고 쓰면 T0에 반박당한다.

### ⚠️ 정직성 메모 (논문에 그대로 써야 할 것)

1. **T5도 사실상 호버-드롭에 수렴해 있다**(exp_018 이래의 알려진 성질, 종단 속도 ~0.4 m/s).
   T5의 5.95 s는 "더 빨리 호버 지점에 도달한다"이지 "순항 중 투하한다"가 아니다.
   → **P2의 체공시간/반송시간 페널티**가 이 축을 열어야 진짜 속도-정확도 Pareto가 생긴다.
2. 베이스라인 접근 컨트롤러는 P 제어 + 고정 게인이다. 더 좋은 고전 컨트롤러(NMPC)라면 T1–T3가
   올라갈 여지가 있다 — **"고전이 원리적으로 불가능하다"가 아니라 "동일 비행 능력 하에서
   릴리스 규칙만 바꾼 비교"**로 서술할 것.
3. T3의 잔차는 게이트가 검사하는 양(예측 착탄점)을 **실제 착탄점과 일치**시키는 방향(+drift)이어야 한다.
   부호를 반대로 두면(−drift) 게이트가 허구를 검사하게 되어 47.5% → 7.0%로 무너진다(실측).
   같은 부호 규약이 학습 잔차에도 적용된다.
4. 단일 시드·단일 체크포인트다. 다중 시드는 P1-8.

## 4. 재현

```bash
# 학습 arm
play.py --task Isaac-DroneBombard-V19-Direct-v0 --policy <ckpt> \
        --episodes 200 --num_envs 200 --seed 1000 --paired_eval \
        --arm_name T5_ours_v19 --out-json /workspace/eval/T5_policy_v19.json
# 규칙 arm (arm ∈ hover|ccip|argmin|oracle)
baseline_drop.py --arm argmin --task Isaac-DroneBombard-V19-Direct-v0 \
        --episodes 200 --num_envs 200 --seed 1000 --out-json /workspace/eval/argmin_v19.json
```

산출물: 호스트 `/opt/drone-bombard/eval/p0/*.json` (arm별 meta + summary + **에피소드별 원자료**,
오프라인 paired 검정용).

## 5. 다음

- P2에서 **T4**(잔차 없는 학습 arm) 추가 → T4→T5가 잔차의 순수 이득이 된다(현재는 T3와 T5 사이에 공백).
- 같은 표를 **v20 분포**에서 재측정(§ [[experiments/exp_022_p0_handoff_dyn_dr]] 결정 후).
- 다중 시드(P1-8) + held-out 체크포인트 선택.

## 관련

- [[research/paper_research_plan]] §4 Table 1 · §3 프로토콜
- [[experiments/exp_022_p0_handoff_dyn_dr]] — 같은 하네스로 측정한 분포 전이 4조건
- [[errors/err_20260803_payload_landing_latch]] — 이 실험이 잡아낸 계측 버그
- [[research/rl_rules]] Rule 28
