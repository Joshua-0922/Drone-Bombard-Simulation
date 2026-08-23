---
date: 2026-08-23
tags: [error, ccip, ballistics, math_utils, resolved]
status: resolved
type: error
---

# err_20260823 — CCIP `ballistic_impact`가 수직속도를 누락

관련: [[research/ccip_vz_omission]] (원인 분석) · [[research/research_architecture]] §2 B1

---

## 증상

버그가 **런타임 에러를 내지 않았다.** 조용히 틀린 착탄점을 예측했고, 학습은
그 오차를 잔차로 흡수하며 정상적으로 수렴했다. 발견 경로는 실패가 아니라
**논문 주장과 코드의 대조**였다.

관측 가능했던 신호(사후 판독):
- `drop_impact_error` 계열 지표가 hover 수렴 정책에서는 작고, cruise-release
  베이스라인(T1/T2)에서는 크게 벌어짐 — T1 6.5% / T2 35.5% vs T0 hover 91.5%
- 즉 "기동하며 투하할수록 나빠지는" 패턴이 이미 Table 1에 찍혀 있었으나
  perception/제어 난이도로 해석되었음

## 원인

`isaac_lab/drone_bombard/math_utils.py` — `ballistic_impact()`

```python
t_fall = torch.sqrt(torch.clamp(2.0 * altitude / gravity, min=0.0))   # t = sqrt(2H/g)
```

정식은 $t=(v_z+\sqrt{v_z^2+2gH})/g$. $v_z=0$ 특수화는 **Gazebo 시절 릴리즈가 호버
근방에서만 일어났을 때 참**이었고, docstring에 그 전제가 명시되어 있었다.

전제를 깬 커밋: **v16(exp_019) 물리 페이로드 도입** — 페이로드가 드론의 실제 선속도를
상속받아 낙하하기 시작. v19가 `release_max_vz=3.0`을 허용하며 $v_z\neq0$ 릴리즈가
정상 경로가 됨. 예측기는 갱신되지 않음.

이미 [[experiments/exp_019_physical_payload]] 후속 #3, [[research/ccip_release_decoupling]] §4에
**해야 할 일로 기록되어 있었으나 이행되지 않았다.**

## 크기

| 오차원 | p50 |
|---|---|
| $v_z$ 누락 | **0.547 m (~70%)** |
| 바람 | 0.197 m |
| 항력 | 0.120 m |

$v_z=-3$ m/s, $H=8$ m, 수평 6 m/s → **1.62 m overshoot**.

## 해결

`vel_z`를 `ballistic_impact` / `predict_impact_nominal` / `time_to_fall`의
**필수 인자로 승격**. 기본값을 주지 않아 모든 호출부가 강제로 갱신되도록 함
(조용한 레거시 경로 차단).

```python
def _time_to_fall(altitude, vel_z, gravity):
    h = torch.clamp(altitude, min=0.0)
    disc = torch.sqrt(vel_z * vel_z + 2.0 * gravity * h)
    return torch.clamp((vel_z + disc) / gravity, min=0.0)
```

**갱신한 호출부 11곳**

| 파일 | 위치 |
|---|---|
| `drone_bombard/math_utils.py` | `ballistic_impact`, `time_to_fall`, `predict_impact_nominal` |
| `drone_bombard/drone_bombard_env.py` | `_evaluate_scripted_release_metric`, `_evaluate_release` ×2, `_predicted_impact_from_snapshot` (+ 스냅샷에 `final_vel_z` 추가) |
| `drone_bombard/v11_env.py` | `V11._ccip`, `V11._get_dones`, `V14._ccip`, `V17._ccip`, `V18._ccip` |
| `baseline_drop.py` | `_predict_series`(T2 horizon), `_oracle_residual`(T3), T1 트리거 |
| `_test_payload_drop.py` | 예측 호출 (⚠️ 이 파일은 다른 이유로도 stale — `_detach_countdown` 등 HEAD에 없음) |

## 검증

| 검사 | 결과 |
|---|---|
| 단위 테스트 `tests/` 전체 | ✅ **69 passed** |
| 신규 회귀 테스트 3종 | ✅ 폐쇄형 일치 / $v_z=0$ 환원 / 1.62 m 크기 고정 |
| smoke `--v19` (물리 페이로드 + 잔차 + DR) | ✅ 2 iter 완주 |
| smoke `--phase 1` (base, 스크립트 referee) | ✅ 2 iter 완주 |
| smoke `--phase 2` (base, release+residual+lead) | ✅ 2 iter 완주 |

## 함정 메모

1. **부호를 두 번 틀렸다.** $v_z$를 하향 양수로 두고 $(v_z+\sqrt{\cdot})/g$를 쓰면
   방향과 크기가 모두 틀린다. 코드 규약은 **ENU UP-positive**이므로 하강은 $v_z<0$이고
   낙하시간이 **짧아진다**(구 공식은 overshoot).
2. **`train.py --task Isaac-DroneBombard-V19-Direct-v0`는 동작하지 않는다.**
   base `DroneBombardEnvCfg`가 넘어가 `AttributeError: 'DroneBombardEnvCfg' object has
   no attribute 'cruise_dir_deg'`. v-track은 **`--v19` / `--v20` 플래그**를 써야 함.
3. 호스트에 torch 없음 → 단위테스트는 `isaac-verify` 컨테이너에서
   `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 isaaclab.sh -p -m pytest ... -p no:cacheprovider`.
   컨테이너는 `/opt/drone-bombard/isaac-worktree`를 마운트하므로 주 워크트리 파일은
   `docker cp`로 옮겨서 검증했다.

## 후속 (미이행)

- [ ] **T0~T3 Table 1 재측정** — T2 horizon과 T3 오라클이 모두 이 공식 사용
- [ ] `release_delay`가 예측 상수일 뿐 실제 지연이 아닌 문제 (§2 B5)
- [ ] 페이로드 항력 프레임 버그 `is_global=True` (§2 B2)
- [ ] T3 오라클 재정의 — 해석식 바람항이 3~7배 과보정 (§2 B3)
