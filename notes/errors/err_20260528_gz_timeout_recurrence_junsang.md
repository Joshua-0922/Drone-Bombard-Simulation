---
date: 2026-06-03
updated: 2026-06-05
tags: [error, infra, gazebo, timeout, reset, intermittent, issue-021]
status: mitigated
type: error
owner: junsang
---

# err 2026-05-28 — `gz model --list` TimeoutExpired (반복 인프라 크래시, #021)

> **상태:** 1·2차 처방으로 **완화**(Round 7 v3 685k에서 crash 0회). 근본 원인 미확정.
> **출처:** `local/issues/issue_021_gz_timeout_recurrence.md`, `local/design/model_history.md`

---

## 증상

학습 중 `reset()` 안에서 다음 예외가 전파되어 **학습 전체가 종료**:

```
subprocess.TimeoutExpired: Command '['gz', 'model', '--list']' timed out after 5.0 seconds
```

- 발생 위치: `drone_drop_env.py:_check_infra_healthy()` (line ~1106) → `subprocess.run(['gz','model','--list'], timeout=5.0)`
- 호출 경로: `reset()` → `_wait_for_cruise()` → CRUISE timeout → `_check_infra_healthy` → TimeoutExpired

---

## 7일간 4 run 크래시 + 오진단 정정

| Run ID | 날짜 | crash step | 당시 (틀린) 분석 |
|--------|------|-----------:|------------------|
| e1xgwhiw | 2026-05-28 | 70,569 | (분석 없음 — 첫 발생) |
| lidq3ydu | 2026-05-30 | 157,882 | **"PX4 log 20GB"** ❌ |
| 6b8bslmz | 2026-05-31 | 295,087 | **"OOM at 294k"** ❌ |
| iobwvcrm | 2026-06-02 | 14,907 | "인프라 timeout" ✅ |

> **중요 교훈:** Round 3·6 v2의 종료 원인을 각각 "PX4 로그 20GB", "컨테이너 OOM(exit 137)"으로 기록했으나, **2026-06-03 재분석에서 모두 #021(동일 gz timeout)이었음**이 밝혀짐. → 증상이 다르게 보여도 같은 근본 버그일 수 있다.

같은 기간 crash 없이 완주한 run도 있음(z05fx7g9, 4j46qwpk 등) → **intermittent failure**(간헐적, 재현 어려움).

---

## 가설 (다음 crash 시 검증용 — 미확정)

1. **Residual velocity 누적** — `gz.msgs.Pose`엔 velocity 필드 없음 → set_pose가 위치만 reset, velocity 보존 누적
2. **Gazebo internal state corruption** — entity registry / joint constraint cache가 시간 따라 corrupt → `gz model --list`가 lock에서 hang
3. **PX4 EKF 누적 손상** — pre_v 큰 episode 누적 → EKF 공분산 NaN/inf → CRUISE 도달 실패 → healthcheck 빈도↑ → hang 순간에 걸림
4. **순간적 trigger event** — 특정 sequence(drop 중 set_pose) deadlock, 누적 무관

---

## 처방

### 1차 (적용) — 예외 catch로 학습 보호
`_check_infra_healthy`가 `TimeoutExpired`를 catch하고 `False` 반환 → 한 번의 hang으로 학습이 죽지 않고 **full restart 회복 경로** 자동 진입.

```python
try:
    result = subprocess.run(['gz', 'model', '--list'], ..., timeout=5.0)
except subprocess.TimeoutExpired:
    return False   # → 회복 경로 진입
```

### 2차 (적용) — 누적 차단
drop 없이 fast-path reset이 N회(default 100) 연속되면 **강제 full restart**.

- `max_consecutive_fast_resets = 100` (hyperparams.yaml)
- 비용: 100 episode마다 30~60s → ~4% slowdown

### 3차 (미적용)
근본 원인 가설 확인 후 결정. intermittent라 언제 재현될지 모름 → 현재는 1·2차로 충분.

---

## 진단 instrumentation (postmortem 자동 수집)

`InfraHealthMonitorCallback`(200 step 간격)이 wandb에 로깅:
`infra/gz_list_ms`(정상~470ms), `gz_rss_mb`(~196MB), `px4_rss_mb`(~19MB), `reset_pre_v`, `reset_post_cruise_v`, `consecutive_fast_resets`, `forced_restart_triggered` 등.

**다음 crash 시 분석:** gz_list_ms 증가→가설2 / rss 증가→메모리leak / pre_v 단조증가→가설1 / cruise_timeout_attempts 잦음→가설3 / 무신호→가설4.

---

## 효과 검증 (Round 7 v3, 685k 자연 종료)

| 지표 | 시작(385k) | 종료(685k) | 추세 |
|------|-----------|-----------|------|
| #021 crash | (대비) | **0회** | ✅ 해결 |
| Forced restart | 0 | 29 | 정상 트리거 |
| gz_ms | 480ms | 480ms | 안정 (degradation 없음) |
| gz_rss | 196MB | 197MB | +1MB (leak 신호 없음) |
| post_v | 다양 | 다양 | 누적 신호 약함 |

→ velocity 누적·메모리 leak 가설 약화. 근본 원인은 여전히 미확정이나 **1·2차 처방으로 학습 안정성 확보**. 2026-06-05 Phase 1 마감 시 영구 코드 통합.

---

## 관련 노트

- [[research/sac_bounded_action_target_entropy_junsang]] — 같은 라운드들에서 동시 발생한 SAC 발산 (#019)
- [[errors/err_20260520_spin_thread_recursive_reset]] — 같은 reset/infra 계열 에러
- [[daily/daily_2026-06-03_junsang]] — #021 처방 결정 당일
- [[daily/daily_2026-06-05_junsang]] — Phase 1 마감 시 처방 검증
- [[00_index_junsang]]
- local: `issues/issue_021_gz_timeout_recurrence`
