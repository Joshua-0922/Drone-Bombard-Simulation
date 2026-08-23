---
date: 2026-08-23
tags: [research, ccip, ballistics, model-error, residual, paper]
status: resolved
type: research
---

# CCIP 수직속도 누락 — 잔차가 배우던 것은 바람이 아니라 공식 결손이었다

> **한 줄**: `ballistic_impact`가 $t=\sqrt{2H/g}$(정지 투하 특수화)를 쓰는 동안 물리
> 페이로드는 드론의 실제 $v_z$를 상속받아 낙하했다. 릴리즈 엔벨로프 안에서 이 누락이
> **모델 오차의 약 70%**를 차지했고, 이는 논문 주 주장("CCIP는 정확한데 항력·바람이
> 틀리게 만든다")을 거짓으로 만든다.

관련: [[research/research_architecture]] · [[errors/err_20260823_ccip_vz_omission]] ·
[[research/ccip_release_decoupling]] · [[experiments/exp_019_physical_payload]] ·
[[research/rl_rules]] (Rule 30)

---

## 1. 무엇이 틀렸나

`isaac_lab/drone_bombard/math_utils.py`

```python
# BEFORE (2026-08-23 이전)
t_fall = torch.sqrt(torch.clamp(2.0 * altitude / gravity, min=0.0))
```

docstring은 이 특수화를 **의도적**이라 밝히고 있었다 —
*"exact parity with drop_calculator_node's `t = (vz + sqrt(vz^2 + 2gH))/g` formula
**specialised to release-from-rest-vertically** (vz~=0 at release, matching the Gazebo
referee which triggers at/near success)"*.

즉 **Gazebo 시절에는 타당한 가정**이었다. 릴리즈가 성공 근방(호버)에서만 발생했으므로
$v_z\approx0$이 참이었다.

## 2. 언제 거짓이 되었나

| 시점 | 사건 | 결과 |
|---|---|---|
| v16 (exp_019) | 물리 페이로드(RigidObject) 도입 | 페이로드가 `write_root_pose_to_sim`으로 드론의 **실제 선속도를 상속**받아 분리됨 (`drone_bombard_env.py:962`) |
| v19 | 릴리즈 엔벨로프가 `release_max_vz = 3.0` 허용 | $v_z\neq0$ 릴리즈가 정상 경로가 됨 |
| — | 예측기는 그대로 $v_z=0$ 가정 | **체계적 오차 발생** |

exp_019 노트는 이를 **후속작업 #3 "CCIP vz 항 복원"**으로 이미 기록했고,
[[research/ccip_release_decoupling]] §4도
*"vz 미반영은 예측·실측 양쪽에 동일 적용되어 상호 일관 — **실 페이로드 바디 도입 시
full-vz 공식으로**"* 라고 조건부 유효성을 명시했다.
**물리 페이로드는 도입되었고, 공식은 고쳐지지 않았다.**

## 3. 크기 — 오차 분해

릴리즈 엔벨로프에서 샘플링한 상태(고도 3~8 m, 수평속도 ≤5 m/s, $|v_z|\le3$ m/s,
바람 $N(0,1.5)$ cap 5, 탄도계수 ±20%)로 두 모델을 오프라인 적분하여 비교:

| 오차원 | p50 (cruise 구간) | p90 |
|---|---|---|
| **$v_z$ 누락** | **0.547 m (~70%)** | 1.231 m |
| 바람 | 0.197 m | 0.497 m |
| 항력 | 0.120 m | 0.351 m |
| **합계** | **0.761 m** | 1.612 m |

hover-drop 구간(정책이 실제로 수렴했던 영역)에서는 총 p50 0.290 m — $v_z\approx0$이라
누락 항이 작다. **즉 이 버그는 "기동하며 투하할수록" 커진다.**

### 폐쇄형 확인

$$t = \frac{v_z + \sqrt{v_z^2 + 2gH}}{g}, \qquad v_z = \text{ENU UP-positive}$$

| $H$ | $v_z$ | $t_{naive}$ | $t_{true}$ | $\Delta t$ | 오차 @6 m/s |
|---|---|---|---|---|---|
| 8 m | 0 | 1.277 | 1.277 | 0 | 0 |
| 8 m | −1 | 1.277 | 1.179 | −0.098 | 0.59 m |
| 8 m | −2 | 1.277 | 1.089 | −0.188 | 1.13 m |
| **8 m** | **−3** | 1.277 | 1.007 | **−0.270** | **1.62 m** |
| 8 m | +2 | 1.277 | 1.497 | +0.220 | 1.32 m |

**부호 주의**: 하강($v_z<0$) 중이면 낙하시간이 **짧아지므로** 구 공식은 착탄점을
실제보다 **멀리** 예측한다(overshoot). 초기 분석에서 이 부호를 반대로 계산했다가
정정했다 — $v_z$를 하향 양수로 두면 부호와 크기가 모두 달라진다.

## 4. 왜 이것이 논문을 무너뜨리나

주 주장 v2 문안:

> *"CCIP는 정확한 닫힌 형태 해를 제공하지만 항력·바람·질량 불확실성으로 인한 모델
> 오차를 포착하지 못한다."*

실제로는 **CCIP가 스스로 알고 있는 상태 변수($v_z$)를 안 쓰고 있었다.** 잔차가 배우던
것의 70%는 "계측 불가능한 외란"이 아니라 **"내 공식의 결손"**이다. 리뷰어가
`ballistic_impact` 한 줄만 봐도 잡힌다.

부수 피해: **T3 "wind-oracle" 베이스라인도 이 공식으로 보정량을 계산**했으므로,
오라클의 특권 정보 우위가 공식 오차에 묻혀 있었다.

## 5. 수정

```python
# AFTER
def _time_to_fall(altitude, vel_z, gravity):
    h = torch.clamp(altitude, min=0.0)
    disc = torch.sqrt(vel_z * vel_z + 2.0 * gravity * h)
    return torch.clamp((vel_z + disc) / gravity, min=0.0)
```

- `ballistic_impact` / `predict_impact_nominal` / `time_to_fall` **모두 `vel_z`를 필수
  인자로** 승격 — 기본값을 주지 않은 것은 의도적이다. 모든 호출부가 컴파일 단계에서
  깨지므로 "조용히 옛 특수화 유지"가 불가능하다.
- 호출부 11곳 갱신: base env 4, v-track 5, `baseline_drop.py` 3(T2 horizon roll-out 포함), 진단 1
- $v_z=0$에서 구 공식으로 **정확히 환원**됨(테스트로 고정)

### 남은 근사 (의도적 미수정)

`release_delay`는 수평 캐리 $(v+w)\cdot\tau$만 더하고 고도 변화를 반영하지 않는다.
더 근본적으로 **시뮬레이터에는 실제 릴리즈 지연이 없다**(`v11_env.py:1182`에서 즉시 분리).
기본값 0.1 s는 5 m/s에서 **약 0.5 m의 유령 캐리**를 예측에 주입한다.
→ 별도 결정 사항. [[research/research_architecture]] §2 B5.

## 6. 파급

| 영향 | 조치 |
|---|---|
| v11~v20 체크포인트 전량 | plant/predictor 변경 → **무효** (Rule 19c). 어차피 폐기 결정됨 |
| T0~T3 Table 1 수치 | **재측정 필요** (T2 horizon, T3 오라클 모두 이 공식 사용) |
| 논문 §I / §III-A 문안 | "계측 가능한 상태에 대해서는 정확"으로 한정 |
| 새 ablation 축 | **$v_z$ 항 on/off** — 버그를 기여로 전환 (그림 8: 모델 오차 분해 막대) |

## 7. 교훈 → [[research/rl_rules]] Rule 30

**해석 모델의 "의도적 특수화"는 그 전제가 참인 동안만 유효하다. 전제를 만든 조건이
바뀌면 특수화는 주석이 달린 채로 버그가 된다.** 여기서는 docstring이 전제를 정확히
적어두었고(*"vz~=0 at release"*), 후속작업으로도 등록되어 있었으나, 전제를 깨뜨린
커밋(v16 물리 페이로드)이 그 링크를 따라가지 않았다.

**적용 규칙**: 특수화 가정을 명시한 함수는 그 가정을 **실행 시점에 검증 가능한 형태로
남기거나**(assert / 진단 로깅), 가정을 깨뜨릴 수 있는 기능 플래그(`payload_physics_enabled`,
`release_max_vz`)에서 **역참조 주석**을 걸어라. 주석만으로는 안 걸린다.
