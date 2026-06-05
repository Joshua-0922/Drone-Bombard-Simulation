# Phase 2 진입 전 — 무엇을, 왜, 어떻게 (검토 + 최종 계획)

> **목적**: Phase 1 종료를 앞두고, Phase 2 에서 시도할 만한 변경들에 대해
> 한 번에 모아서 진지하게 고찰. 메커니즘 → 기대 → 위험 → resume 가능성 검토
> 끝에 **실제로 무엇을 어떤 순서로 시도할지 최종 계획** 도출.
>
> 영구 plan 이 아니라 **이 시점의 정리 + 결정 문서**. 실험 결과 따라 갱신.

작성일: 2026-06-04
Phase 1 마감: Round 7 v3 (run `436xl0bb`, ~685k step) 자연 종료 직후
이 문서 구성:
  1~2장: 베이스라인 정량 + Phase 2 의 의도
  3~5장: **고려한 모든 후보들의 메커니즘과 trade-off**
  6장: **최종 실험 큐** (실제 시도 순서)
  7~9장: 평가/예산/백업 절차
  10장: Phase 3 안내

---

## 1. 배경

### Phase 1 종료 시점 베이스라인 (Round 7 v3)

| 영역 | 상태 |
|---|---|
| **SAC entropy 발산** | ✅ 해결 (per-sample damping + target_entropy=-15) |
| **Critic 폭주** | ✅ 해결 (Huber + target_q_clip=500) |
| **인프라 #021 gz timeout** | ✅ 해결 (1·2차 처방, forced restart 19+회 정상) |
| **성능** | success_rate 10%, best drop 1.32m, median drop_error 15m |

### Phase 1 정량 측정 (685k 종료 시점 예정)

```
정책 능력:    best 1.32m → 이미 입증
정밀도 분포:  bimodal — 11% < 5m, 89% 멀리 빗나감
공통 약점:    median 15m (정확한 접근 일관성 부족)
```

### Phase 2 의 정체성

> "베이스라인의 SAC + 인프라 안정성 유지 + 정밀도 점진 향상"

- 보상 공식의 큰 변경 X (그건 Phase 3)
- 단일 파라미터 격리 테스트 (인과 명확)
- Resumable 우선 (빠른 iteration), 필요 시 fresh start

---

## 2. 목표

| 지표 | 현재 (베이스라인) | Phase 2 목표 |
|---|---|---|
| success_rate | 10% | **20%+** |
| median drop_error | 15m | **< 10m** |
| < 3m 비율 | 4% | **10%+** |
| best drop | 1.32m | < 1.0m |
| 학습 안정성 | 유지 | 유지 |

---

## 3. 방법론 — 통제된 단일 변수 실험

### 원칙

1. **한 번에 한 파라미터만 변경**
2. **베이스라인과 동일한 출발점** (preempt resume)
3. **공정한 비교를 위한 정량 평가**: deterministic eval 10 episodes
4. **위험도 낮은 것부터** (resume 가능, 영향 작은 것)

### 흐름

```
[Phase 1 baseline 685k 모델]
       ↓
[deterministic eval 10ep → 평균 drop_error 측정] ← baseline metric
       ↓
   실험 #1 (jackpot 1m)
       ↓
[100k resume 학습 → deterministic eval] ← variant metric
       ↓
   비교 → 효과 판정
       ↓
   실험 #2 (다음 후보)
       ↓
   ... 반복
```

---

## 4. 실험 큐 — Phase 2a (Resumable)

### 실험 #1: `jackpot_threshold` 0.1 → 1.0

**우선순위**: 🥇 최우선

**메커니즘:**
- 현재 0.1m (10cm) — 학습 전체에서 jackpot **0회** 발화
- 1.0m 으로 늘리면 정밀 drop 시 실제 발화 가능 (best 1.32m 직전까지 도달)
- `r_success_jackpot` (50 reward) 가 정책에 신호 추가

**가설:**
- 정밀 drop 에 큰 보상 → 정책이 그쪽으로 강하게 끌림
- 1.5m 까지 떨어지는 drop 들이 더 자주 발생할 것

**예상 효과:**
- 단기 (50k): drop_error 분포의 좌측 꼬리 (< 2m) 가 두꺼워짐
- 중기 (100k): success_rate 10% → 15-18%

**최악 시나리오:**
- jackpot 발화가 여전히 드물어 효과 미미 (가장 가능성 큼)
- 또는 critic 분산 약간 증가 (영향 < 1% transitions)

**감지/모니터링:**
- `env/total_jackpot` 발화 횟수
- `critic_loss` 추세 (35-50 유지 시 OK, 500+ 시 회귀)
- `env/drop_error` 평균/중앙값

**Resume vs Fresh:** ✅ Resume 안전 (옛 buffer 의 jackpot transition 0개)

**종료 조건:**
- 100k step 진행 OR success_rate 명백한 변화
- 만약 50k 내 jackpot 발화 0회 → threshold 더 늘림 (1.5m)

---

### 실험 #2: `target_entropy` -15 → -20

**우선순위**: 🥈 2번째

**조건:** 실험 #1 종료 후, ent_coef 가 여전히 0.1-0.2 수준이면 진행. cap 근처면 보류.

**메커니즘:**
- SAC entropy target 을 더 음의 방향으로 → 더 deterministic 정책 허용
- 현재 ent_coef 0.12 가 더 작아질 수 있음 (예: 0.05)

**가설:**
- 정책의 액션 분산 감소 → 일관된 drop 의사결정
- bimodal 분포의 분산 줄임 → median 개선

**예상 효과:**
- ent_coef 0.12 → 0.05 정도
- drop_error 분산 감소
- success_rate 약간 상승

**최악 시나리오:**
- exploration 완전 죽음 → local optimum 정착
- best 1.32m 가 한계가 됨, 그 이상 학습 안 됨

**감지:**
- `ent_coef` 추세
- `actor_loss` 변화
- drop_error 분산 (std)

**Resume vs Fresh:** ✅ Resume 안전 (SAC 하이퍼파라미터만)

**종료 조건:**
- 100k step OR success_rate 변화 명확
- ent_coef 가 floor (예 0.001) 도달 시 정지

---

### 실험 #3: `random_drop_prob` 0.005 → 0.001

**우선순위**: 🥉 3번째

**조건:** 실험 #1, #2 결과 보고 진행

**메커니즘:**
- 현재 episode 당 ~63% random drop 발생 → 89% far-miss 의 주범 가설
- 1/5 로 줄임 → 노이즈 감소

**가설:**
- bimodal 분포의 우측 꼬리 (> 15m) 축소
- 정책 driven drops 비율 증가

**예상 효과:**
- 평균 drop_error 감소 (random noise 빠지면서)
- drops/1k step 약간 감소

**최악 시나리오:**
- drop 빈도 폭락 → policy 가 drop 자체를 잊음
- success_rate 폭락

**감지:**
- `env/total_drops`, drops/1k step
- `env/total_auto_drops` 비율 증가 여부

**Resume vs Fresh:** ✅ Resume 안전 (trigger 빈도만 변경)

**종료 조건:**
- 100k step OR drop 빈도 < 0.3/1k 시 즉시 복원

---

### 실험 #4 (선택): `target_q_clip` 500 → 1000

**우선순위**: 보류, 베이스라인 critic 안정성 봐서 결정

**조건:** 다른 실험 진행 중 critic_loss 가 일관되게 < 100 이면 시도. 100+ 면 보류.

**메커니즘:**
- 현재 critic_loss 35-50 → 500 clip 이 거의 안 닿음
- 1000 으로 완화 → 더 풍부한 Q-value 학습 가능

**가설:**
- 정밀 success 의 Q-value 가 더 정확히 학습
- 정책 gradient 가 더 sharp 해짐

**최악:**
- critic 다시 폭주 → ent_coef 발산
- 이전 처방 효과 일부 회귀

---

## 5. Phase 2b (Fresh Start) — 필요 시

베이스라인 + Phase 2a 로 충분치 않으면 진행.

### 후보 #5: `w_heading` 0.7 → 0.9
- 매 step dense reward → buffer 거의 전체 stale
- Fresh start 150k

### 후보 #6: `w_distance_penalty` 0.0 → 0.05
- 매 step dense reward
- Round 5 에서 비활성화한 이유 점검 후 시도

---

## 6. 평가 메트릭

### 학습 중 (continuous)
- `env/total_drops`, drops/1k step
- `env/total_auto_drops`, auto vs random 비율
- `env/total_success`, `env/total_jackpot`
- `env/drop_error` 평균
- `train/ent_coef`, `ent_damping`, `critic_loss`
- `infra/*` (안정성 유지 확인)

### Deterministic Eval (실험 전후 비교)
```
10 episodes, 같은 시드/같은 환경
metric:
  - 평균 drop_error
  - 중앙값 drop_error
  - < 5m, < 3m, < 1m 비율
  - episode 종료 사유 분포
```

---

## 7. 시간 예산

| 단계 | 시간 |
|---|---|
| Phase 1 자연 종료 + 백업 | 2~3h |
| 베이스라인 deterministic eval | 30분 |
| 실험 #1 (jackpot 1m) | ~2h 학습 + 30분 eval |
| 실험 #2 (target_entropy) | ~2h + 30분 |
| 실험 #3 (random_drop) | ~2h + 30분 |
| (선택) 실험 #4 | ~2h + 30분 |
| 종합 분석 + 문서화 | 1h |
| **합계** | **~12-15h** (분산 가능) |

---

## 8. Phase 2 종료 조건

다음 중 하나 충족:
1. **성공**: success_rate ≥ 20% 안정, median drop_error < 10m
2. **수렴**: 모든 Phase 2a 실험 완료, 더 이상 의미 있는 개선 없음 → Phase 3 권고
3. **회귀**: 실험으로 베이스라인보다 나빠진 경우 → 베이스라인 복원, Phase 3 직행

---

## 9. Phase 1 데이터 백업 절차

Round 7 v3 자연 종료 직후 실행. 모든 자료 archive 로 영구 보존.

### 9.1 모델 / 체크포인트

```bash
mkdir -p /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/

# 최종 모델 (학습 완료 시 자동 저장)
docker cp drone-bombard-harmonic:/workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/

# Preempt save (학습 중간 안전 저장)
docker cp drone-bombard-harmonic:/workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/
docker cp drone-bombard-harmonic:/workspace/ros2_ws/rl_checkpoints/sac_drop_preempt_replay.pkl \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/

# Rolling checkpoints (마지막 5개)
docker cp drone-bombard-harmonic:/workspace/ros2_ws/rl_checkpoints/. \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/rl_checkpoints_snapshot/
```

### 9.2 SuccessReplay (학습 중 발화한 성공 모델들)

```bash
docker cp drone-bombard-harmonic:/workspace/ros2_ws/success_replay/436xl0bb \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/success_replay/
```

### 9.3 Drop Episodes (모든 drop 의 trajectory)

```bash
docker cp drone-bombard-harmonic:/workspace/ros2_ws/rl_checkpoints/drop_episodes \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/
```

### 9.4 WandB 데이터 (로컬 캐시)

```bash
docker cp drone-bombard-harmonic:/workspace/ros2_ws/wandb/run-20260603_214418-436xl0bb \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/wandb_run/
```

### 9.5 설정 + 코드 snapshot

```bash
cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/config/hyperparams.yaml \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/hyperparams.yaml

cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/train_sac.py

cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py \
    /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/drone_drop_env.py
```

### 9.6 백업 검증

```bash
ls -la /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/
du -sh /home/juns/Drone-Bombard-Simulation/local/backups/phase1_final_round7_v3/
# 예상: 모델 zip 3MB + replay pkl 83MB + drop_episodes ~100MB + wandb ~10MB
# 총 ~200MB
```

### 9.7 README 작성

`local/backups/phase1_final_round7_v3/README.md` 에 다음 기록:
- 학습 종료 시점 (날짜, step)
- 최종 metric (success_rate, drop_error 통계)
- Deterministic eval 결과 10 episodes
- Phase 2 로 이어지는 시작점 명시
- 적용된 처방 요약 (Round 7 v3 모든 변경)

---

## 10. Phase 3 (참고용 — 별도 계획 시점)

Phase 2 가 더 이상 의미 있는 개선 못 가져오면 Phase 3 진입.

**Phase 3 후보 항목**:
- 4-layer hierarchical reward 구조 자체 재검토
- CCIP prediction 통합 방식 개선
- Terminal physics 시점 변경
- Vision-based observation 도입 (Issue #018)
- PPO 등 알고리즘 변경 시도

이건 Phase 2 결과 본 후 별도 design_review 문서로 작성.

---

## 관련 문서

- `local/design/design_review.md` — 최종 설계 요약
- `local/issues/master.txt` — 안건 현황
- `local/issues/issue_021_gz_timeout_recurrence.md` — Round 7 인프라 처방
- `local/parameter_log.md` — 파라미터 변경 이력
- `local/guides/post_training_verification_guide.txt` — Deterministic eval 절차
- `local/backups/phase1_final_round7_v3/` — Phase 1 종료 백업 (자연 종료 후)
