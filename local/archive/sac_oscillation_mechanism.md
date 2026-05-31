# SAC Sparse Reward 환경에서의 학습 Oscillation 메커니즘

> 작성일: 2026-05-31
> 작성 맥락: Round 4 (run 4j46qwpk) 초반 2k~4.6k 구간에서 관찰된 critic_loss spike + ep_len 일시 하락 + 회복 패턴 분석
> 분류: 참고 자료 (학습 진단 시 활용)

---

## 관찰 데이터 (Round 4, run 4j46qwpk)

```
total_timesteps  critic_loss  ep_len_mean  ep_rew_mean  ent_coef
   1910             208          99           -40         0.919
   2375              78          96           -45         0.892
   2698              55          84           -43         0.874
   2706             357          75           -43         0.874   ← spike
   2710             241          67           -43         0.873
   4665              30         106           -34         0.727   ← 회복
```

**패턴:**
1. 2k 부근 안정 학습 (ep_len 99)
2. 2.7k 부근 critic_loss 7배 폭증 (78 → 357)
3. ep_len 단조 감소 (99 → 67)
4. 4.6k에 회복 (critic_loss 30, ep_len 106, ep_rew 개선)

---

## 메커니즘 분해 (4단계)

### Stage 1: Drop event spike

```
배경:
  - Drop은 sparse event (수백~수천 step 중 1번)
  - 보상 magnitude 큼 (+50 ~ +500)
  - Per-step 보상은 작음 (-1 ~ +1)

발생:
  2.7k 부근 random_drop 또는 auto_drop 발동
  큰 보상 transition이 buffer에 추가됨
  Critic은 이 state-action 페어의 Q-value를 업데이트

영향:
  Bellman target = r + γ * Q(s', a')
  큰 r → Q(s, a) 추정값 급증
  Critic loss = (Q_target - Q_predicted)²
  예측이 따라잡기 전엔 loss 폭증 → 357
```

### Stage 2: PER 증폭

```
PER (Prioritized Experience Replay):
  priority = (|TD-error| + ε)^α
  큰 TD-error transition을 자주 sampling

대형 drop transition:
  TD-error 큼 → priority 높음
  매 batch에서 자주 선택됨
  Critic의 그 영역 학습이 집중적

부작용:
  단일 transition이 critic을 한 방향으로 강하게 끌고감
  Policy gradient도 따라감
  → 다른 transition들의 학습이 상대적으로 약해짐
```

### Stage 3: Policy over-correction

```
Actor update:
  L_actor = -E[Q(s, a)] + α * H(π)
  Q 값이 큰 state-action 페어에서 policy 확률 증가

영향:
  큰 보상 받은 행동을 따라 하려 함
  하지만:
    - 비슷한 state에서 그 행동이 항상 좋은 건 아님
    - 다른 행동 (이전엔 잘 통하던) 무시
  → 정책 변동 (variance ↑)
  → 일부 에피소드 짧게 종료 (crash 등)
  → ep_len 감소

수치적 예시:
  이전: policy가 일관되게 hover + 가끔 success → ep_len 99
  Spike 후: policy가 success 행동 따라 하려다 실수 → crash 늘어남
           → ep_len 67
```

### Stage 4: 자동 회복

```
SAC의 안정화 메커니즘:

  a) Target network (tau)
     - Q_target은 천천히 갱신
     - 즉각적인 critic 변화에 영향받지 않음
     - 우리 tau=0.002 → 매우 안정적
     - Stage 1의 spike도 천천히 흡수

  b) Auto entropy tuning (ent_coef)
     - Policy 변동성 측정
     - 너무 deterministic이면 entropy↑ (탐색 강화)
     - 너무 stochastic이면 entropy↓ (활용 강화)
     - Stage 3 후 자연스럽게 균형 찾음

  c) Replay buffer 다양성
     - 짧은 episode 데이터가 buffer에 들어감
     - Critic이 "이 행동은 짧게 끝난다 = 안 좋다" 학습
     - Policy gradient가 반대 방향 유도
     - 다른 좋은 transition들도 다시 학습 기회

결과:
  4.6k 시점: critic_loss 30, ep_len 106, ep_rew -34
  Spike 이전보다 약간 더 나아진 상태
```

---

## SAC + Sparse Reward의 본질적 oscillation

이 패턴은 **버그가 아니라 알고리즘 특성**입니다.

### 왜 본질적인가

```
Sparse reward = drop event가 드물게 발생
→ 학습 신호의 90%+는 작은 per-step
→ 가끔 큰 episode-end 보상이 dominant

Bellman backup의 비선형 효과:
  Q(s, a) ← r + γ * max_a' Q(s', a')
  γ=0.995 → effective horizon ~200 step
  Drop 보상이 ~200 step 거슬러 올라가며 모든 transition에 영향

결과:
  단일 drop transition이 critic 전체에 파급
  oscillation은 피할 수 없음
```

### Round 3에서 본 동일 패턴

```
100k~125k: avg 13.9m, success 3건 ← 최우수
125k~150k: avg 35.5m, success 0건 ← 발산

이것도 동일한 메커니즘:
  100~125k에 큰 success (reward 499~550)
  → critic 큰 업데이트
  → policy over-correction
  → 125~150k에 발산
```

---

## 진짜 발산 vs 정상 oscillation 구별

### 정상 (회복 가능)
- critic_loss spike 후 다시 100 이하 (수 batch 안에)
- ep_len 짧아진 후 다시 길어짐 (수십~수백 step 안에)
- ep_rew_mean 장기 추세가 개선
- ent_coef 안정 또는 점진 감소
- **Round 4 4j46qwpk 4.6k 회복 패턴**

### 진짜 발산 (회복 불가)
- critic_loss 1000+ 도달 후 더 증가
- ep_len 1로 수렴, 회복 안 됨
- ep_rew_mean 더 음수로 깊어짐
- ent_coef 1.0 이상으로 폭주
- **Round 3 q13hli0y (-6.77e+9 outlier)**
- **Round 2 후반 hover 정착 패턴**

---

## 완화 방법

### 이미 적용된 (Round 3+)

```
1. LR 축소 (3e-4 → 1e-4)
   - 모든 update 점진적
   - Spike 진폭 감소

2. tau 축소 (0.005 → 0.002)
   - Target network 천천히 갱신
   - Critic 안정성 ↑

3. PER priority 상한 (30)
   - Extreme outlier가 sampling 독점 방지

4. Hard cap [-200, +300]
   - 단일 step reward 폭주 차단
```

### 추가 가능 (Round 5+)

```
5. Gradient clipping
   - Critic/actor gradient norm을 max 10 등으로 제한
   - 큰 update 자체를 방지

6. Distributional RL
   - Q-value 분포 학습 (평균 대신)
   - Outlier에 더 강건

7. CQL (Conservative Q-Learning)
   - Q-value overestimation 직접 페널티
   - 보수적 학습

8. EMA Policy
   - 추론 시 policy weights의 EMA 사용
   - Oscillation 평균화
```

---

## 학습 모니터링 권고

### Spike 발견 시 (정상)

```
관찰:
  critic_loss > 200 spike

행동:
  1. 다음 50~100 batch 지켜봄
  2. 회복되는지 확인 (loss 100 이하로)
  3. ep_len 추세 확인
  
판단:
  100 batch 안에 회복 → 정상 oscillation
  계속 증가 → 발산 의심
```

### 발산 의심 시

```
관찰:
  critic_loss 1000+ 지속
  ep_len 1로 수렴
  ent_coef 폭주

행동:
  1. 학습 중단
  2. 마지막 안정 체크포인트 확인
  3. 원인 분석 (drop_episodes, reward outlier 등)
  4. 처방 후 fresh start
```

---

## 부록: 우리 환경의 특수성

### Drop 보상 magnitude

```
이론 최대 single drop reward:
  proximity bonus (30) + precision (100) + prediction (20) + jackpot (50)
  = +200

평균 per-step reward:
  approach + heading + impact + penalties = ~0.5

차이: ~400배
```

이 큰 차이가 oscillation의 주요 원인. 보상 스케일을 더 균등하게 만들면 oscillation 감소.

### 우리가 만든 안전망

```
정상 oscillation 보장:
  - LR 1e-4, tau 0.002 (느린 업데이트)
  - PER cap 30 (priority 폭주 차단)
  - Hard cap ±300 (reward 폭주 차단)
  - Sigmoid altitude penalty (지수 폭주 차단)
  - max_altitude truncate (드론 무한 상승 차단)

이 안전망 덕분에 진짜 발산은 거의 발생 안 함.
하지만 정상 oscillation은 SAC 특성상 피할 수 없음.
```

---

## 결론

Round 4 4j46qwpk의 2.7k~4.6k 구간 패턴은:
- **정상 SAC oscillation**
- 회복 메커니즘 작동 확인됨
- 학습 추세 자체는 개선 중
- 우려할 사항 아님

장기 학습 추세에 집중. 단기 fluctuation은 정상.
