================================================================================
 Issue #021 — gz model --list TimeoutExpired (recurring infra crash)
 발견  : 2026-06-03
 상태  : 1·2차 처방 적용 (Round 7 resilient v1), 근본 원인 미확정
 영향  : 7일간 4 run crash (e1xgwhiw, lidq3ydu, 6b8bslmz, iobwvcrm)
================================================================================


## 증상

학습 중 reset() 안에서 `subprocess.TimeoutExpired: Command '['gz', 'model', '--list']' timed out after 5.0 seconds` 가 발생하여 Python 예외 전파, 학습 전체 종료.

크래시 발생 위치 (`drone_drop_env.py` line 1106):
    _check_infra_healthy() → subprocess.run(['gz', 'model', '--list'], timeout=5.0)

호출 경로 (reset() 안):
    _wait_for_cruise() → CRUISE timeout → _check_infra_healthy → TimeoutExpired

## 이력

| Run ID      | 날짜       | crash step | 이전 문서의 잘못된 분석 |
|-------------|------------|-----------:|--------------------------|
| e1xgwhiw    | 2026-05-28 |     70,569 | (분석 없음 — 첫 발생)    |
| lidq3ydu    | 2026-05-30 |    157,882 | "PX4 log 20GB"  (틀림)   |
| 6b8bslmz    | 2026-05-31 |    295,087 | "OOM at 294k"   (틀림)   |
| iobwvcrm    | 2026-06-02 |     14,907 | "인프라 timeout" (정확)  |

같은 기간 crash 없이 완주한 run: z05fx7g9, 4j46qwpk, mnlr1zpe, bfv4la9a → **intermittent failure**.

## 가설 (다음 crash 시 검증할 후보)

1. **Residual velocity 누적**
   `gz.msgs.Pose` 에는 velocity 필드 없음 → set_pose 는 위치만 reset. 매 episode 종료
   velocity 가 보존됨. PX4 EKF 가 흡수하지만 누적이 어딘가 손상시킬 가능성.

2. **Gazebo internal state corruption**
   메모리 RSS 안 늘어도 logical state (entity registry, joint constraint cache)
   가 시간 따라 corrupt. `gz model --list` 가 entity registry lock 에서 hang.

3. **PX4 EKF 누적 손상**
   pre_v 큰 episode 가 누적되면 EKF 공분산 행렬이 NaN/inf. CRUISE 도달 못함 → 
   _check_infra_healthy 호출 빈도 증가 → Gazebo 가 우연히 hang 한 순간 잡힘.

4. **순간적 trigger event**
   특정 sequence (e.g. drop 도중 set_pose) 가 deadlock 일으킴.
   누적 무관, 운에 의해 trigger.

## 진단 데이터 (postmortem 자동 수집)

InfraHealthMonitor 콜백 (200 step 간격) + reset_diag 가 wandb 에 로깅:

* infra/gz_list_ms — gz model --list 응답 시간 (정상 ~470ms, baseline)
* infra/gz_rss_mb — Gazebo RSS (정상 ~196MB)
* infra/px4_rss_mb — PX4 RSS (정상 ~19MB)
* infra/reset_pre_v / pre_ang_v — 직전 episode 종료 velocity
* infra/reset_post_cruise_v / post_ang_v — CRUISE 도달 시점 velocity
* infra/reset_idx — 누적 reset 횟수
* infra/reset_prev_dropped — 직전 drop 발생 여부
* infra/used_full_restart — 이번 reset 이 full restart 였는지
* infra/forced_restart_triggered — 2차 처방으로 강제 restart
* infra/consecutive_fast_resets — drop 없이 누적된 fast-reset 횟수
* infra/cruise_timeout_attempts — CRUISE 시도 retry 횟수

**다음 crash 시 분석 절차:**
1. WandB run 의 마지막 100 sample 추출
2. gz_list_ms 가 시간 따라 증가 → 가설 2 (Gazebo deadlock)
3. gz_rss_mb / px4_rss_mb 증가 → 메모리 leak (다른 원인)
4. pre_v / post_cruise_v 가 단조증가 → 가설 1 (velocity 누적)
5. cruise_timeout_attempts 가 자주 1+ → 가설 3 (PX4 EKF 손상)
6. 위 어디에도 신호 없음 → 가설 4 (순간 trigger)

## 처방

### 1차 처방 (적용 — Round 7 resilient v1)
`_check_infra_healthy` 가 `subprocess.TimeoutExpired` 를 catch 하고 False 반환.
한 번의 gz hang 으로 학습 전체가 죽지 않음. 회복 경로 (full restart) 자동 진입.

위치: drone_drop_env.py:_check_infra_healthy
```
try:
    result = subprocess.run(['gz', 'model', '--list'], ..., timeout=5.0)
except subprocess.TimeoutExpired:
    if rclpy.ok():
        self._node.get_logger().warning('[RL Env] gz model --list timed out — ...')
    return False
```

### 2차 처방 (적용 — Round 7 resilient v1)
누적 차단: drop 없이 fast-path reset 이 N 회 (default 100) 연속되면 강제 full restart.

위치: drone_drop_env.py
* __init__: self._consecutive_fast_resets = 0,
            self._cfg_max_consecutive_fast_resets = cfg_env.get(..., 100)
* reset(): _forced_restart = (self._consecutive_fast_resets >= cap)
           if _prev_dropped or _forced_restart:
               kill_infra + start_infra; counter = 0
           else:
               _gz_reset_poses(); counter += 1

설정: hyperparams.yaml env.max_consecutive_fast_resets (default 100)

비용: 100 episode 마다 30~60s 추가 → ~4% slowdown 예상

### 3차 처방 (미적용 — 근본 원인 미확정)
가설 확인 후 결정.

## 관련 코드
* drone_drop_env.py:_check_infra_healthy (line 1093~)
* drone_drop_env.py:_gz_reset_poses (line 1535~)
* drone_drop_env.py:reset (line 471~)
* train_sac.py:InfraHealthMonitorCallback (line 394~)

## 관련 issue
* issue_017 (hover exploit) — fast-reset 빈도와 연관 가능
* issue_019 (SAC entropy divergence) — 같은 라운드들에서 동시 발생, Round 6 v2 "OOM" 잘못된 기록 정정 필요

## 진행 메모
* 2026-06-03: 1차/2차 처방 적용. round7_resilient_v1 시작. 다음 crash 시 위 진단 데이터로 가설 검증.
* 2026-06-04: Round 7 v3 (critic-stable, run 436xl0bb) 진행. 처방 효과 입증:
  - 685k step 자연 종료까지 #021 gz timeout crash **0회**
  - Forced restart 29회 정상 작동 (100 fast-reset 누적마다)
  - gz_ms 베이스라인 ~480ms 안정 유지 (degradation 없음)
  - gz_rss 197 MB 안정 (메모리 leak 신호 없음)
  - post_v 대체로 작음 (velocity 누적 가설 약화)
* 2026-06-05: Phase 1 마감. 1·2차 처방 영구 코드 통합 결정.
  근본 원인 가설 검증은 다음 #021 crash 시 (intermittent failure 이므로 언제 일어날지 모름).
  현재 시점에서 학습 안정성 유지가 더 중요 → 1·2차 처방만으로 충분.

## 효과 검증 데이터 (Round 7 v3, 685k step)

| 지표 | 시작 (385k preempt resume) | 종료 (685k 자연) | 추세 |
|---|---|---|---|
| gz_ms 평균 | 480ms | 480ms | 안정 |
| gz_rss 평균 | 196 MB | 197 MB | +1 MB only |
| px4_rss | 19-20 MB | 19-20 MB | 안정 |
| Forced restart | 0 | 29 | 정상 트리거 |
| #021 crash | (대비) | 0 | 해결 |
| post_v 대부분 | 다양 | 다양 | 누적 신호 X |
