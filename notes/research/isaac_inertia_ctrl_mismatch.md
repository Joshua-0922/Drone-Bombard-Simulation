---
date: 2026-07-05
tags: [research, isaac-lab, physx, controller, inertia, forensics]
status: confirmed
type: research
---

# set_inertias는 solver에 전파된다 — exp_013은 rate loop이 ~1300× 저토크인 plant에서 학습됐다

> **2026-07-05 `_diag_inertia.py` 계측 (`/workspace/logs/_diag_inertia2.log`).**
> 07-04 포렌식과 구 코드 주석의 **"set_inertias는 sim에 전파 안 됨" 주장은 오류**였다.
> 이 발견은 exp_013 plant의 실체와 [[research/rl_rules]] Rule 15 wobble의 Isaac 측
> 대응 원인을 다시 쓴다.

## 1. 계측 방법과 결과

순수 바디 토크 $\tau_x = 5\times10^{-5}\,\mathrm{N\cdot m}$를 free-fall 중 25 substep
인가, $I_{est} = \tau/\alpha$ (각속도 램프 선형 피팅):

| Phase | 조작 | $I_{est}$ | view Ixx |
|---|---|---|---|
| P1 | as-built (스폰 authoring, native I) | 2.115e-5 | 1.657e-5 |
| P2 | `set_masses(body→0.025)` | 2.099e-5 | 1.657e-5 |
| P3 | `set_masses(body→2.17)` 복원 | 2.115e-5 | 1.657e-5 |
| P4 | `set_inertias(x500 0.0217)` | **2.181e-2** | 2.170e-2 |

(P1 2.1e-5 > authored 1.66e-5는 프롭 링크 4개의 기여 ~2.9e-6 포함 — 정합.)

- **set_masses는 inertia를 건드리지 않는다** (P2/P3: 질량 87× 변화에도 I 불변).
- **set_inertias는 solver에 전파된다** (P4: ×1031). — 기존 주장 정정.

## 2. exp_013 plant의 실체 (재구성)

구 `_apply_body_mass_override()`는 `set_masses(2.17)` + `set_inertias(0.0217)`를
init에서 호출했고, 컨트롤러는 **override 이전에** `get_inertias()`를 읽어
$I_{ctrl} = 1.66\times10^{-5}$로 토크를 사이징했다. 07-05 계측으로 solver는
$I_{body} = 0.0217$를 실제로 받았음이 확인됨. 따라서:

$$\dot\omega = \frac{I_{ctrl}}{I_{body}} k_{rate}\, \omega_{err} \approx \frac{18}{1309}\,\omega_{err} \approx 0.014\,\omega_{err}$$

**rate loop이 ~1300× 저토크** — 자세 응답이 사실상 마비된 plant였다. 파생 증상:

- 자세가 물리적으로 빨리 못 움직임 → `bad_attitude` 종단이 거의 불가능 (eval 1/200)
- 굼뜬 자세 → 속도 명령을 크게·오래 걸어야 기동 → rail-riding 액션 평균
  (raw |a|=2.6, 77% 포화)과 wobble 스타일의 Isaac 측 원인 후보
- hover는 무사 (토크 ≈ 0 지점이라 미스매치 무증상) → `--zero-actions` 게이트가
  이 버그를 못 잡았던 이유

## 3. 구 정책의 plant-overfit 실증 (probe 재평가)

exp013_v2 `model_final.pt`을 일관 plant에서 deterministic 재평가 (200 ep):

| plant 변형 | success | bad_attitude | max_altitude | crash |
|---|---|---|---|---|
| 구 plant (exp_013 eval) | 36% | 0.5% | 33% | 27% |
| 수정 plant (native I authoring) | 7.9% | **67.8%** | 24.3% | 0% |
| 수정 plant + 프롭스핀 복원 (A/B) | 7.9% | **65.3%** | 27% | 0% |
| 최종 plant (x500 I authoring) | 8.5% | **69.0%** | 22.5% | 0% |

- bad_attitude는 에피소드 중앙값 **1.5 s**에 `|ω|>2.0` 한계로 발화 (last angspd med
  1.77, max med 1.86) — 구 정책은 자세 한계 바로 밑에서 사는 정책이었고, 응답이
  설계 수준으로 돌아온 plant에선 같은 명령이 즉시 한계를 넘는다.
- **프롭스핀 A/B가 로터 수정을 무관으로 실증** (65% ≈ 68%): 행동 변화는 inertia
  일관성 변화 단독 기인.

## 4. 적용 규칙

1. **런타임 물리 프로퍼티 오버라이드 금지** — 스폰타임 USD authoring
   (`_author_body_mass_props`)만 사용. 런타임 뷰 API는 (a) 1-substep 지연 소비
   (속도킥, [[isaac_mass_override_reset_bug]]), (b) 뷰 캐시·solver·컨트롤러 3자
   불일치를 만든다.
2. **컨트롤러가 읽는 물리량 == solver 물리량 검증을 게이트에 추가** — hover
   게이트는 토크 ≈ 0이라 못 잡는다. `_diag_inertia.py` 재사용 (토크 인가 →
   $I_{est}$ vs `get_inertias` 비교).
3. plant를 바꾸는 수정 후에는 **구 체크포인트 성능이 무효** — fresh start 필수
   (이번 케이스: 정책이 plant 동역학 자체에 overfit).

## 관련

- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] §4d — 속도킥 포렌식 (07-04)
- [[experiments/exp_014_A2_visionrange]] — 이 plant에서의 첫 fresh 학습
- [[research/isaac_ppo_tuning_recommendations]] — 우선순위 재배열 근거
- [[research/rl_rules]] — Rule 19 (본 발견의 규칙화)
- [[isaac_mass_override_reset_bug]] (Claude memory) — 07-05 정정 반영 필요
