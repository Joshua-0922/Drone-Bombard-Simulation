---
date: 2026-04-16
tags: [research, rtf, fps, sim-speed, bottleneck, PX4, Gazebo]
status: active
type: research
---

# RTF(Real-Time Factor)와 학습 FPS 관계 분석

> **실험 출처:** [[experiments/exp_003_rtf_dryrun]]
> **결론:** RTF=2 최적. Python RL 루프가 bottleneck이므로 RTF를 무한정 올려도 FPS는 증가하지 않음.

---

## 실험 결과

| RTF | `PX4_SIM_SPEED_FACTOR` | avg fps | 4000 steps 소요 | 상대 속도 |
|-----|----------------------|---------|----------------|---------|
| 1 | 1 | 40.5 | 89s | 1.0× (기준) |
| **2** | **2** | **59.5** | **61s** | **1.47× ✅** |
| 4 | 4 | 51.5 | 70s | 1.27× ❌ (역전) |

- **RTF=2** → RTF=1 대비 fps **+47%**, 벽시계 시간 **−32%**
- **RTF=4** → RTF=2 대비 fps **−14%**, 벽시계 시간 **+15%** (역전)

---

## 원인 분석: Python RL 루프 병목

```
RTF 올릴 때 일어나는 일
  Gazebo + PX4 물리 스텝 빈도: RTF배 증가
         ↓
  obs publish 빈도: RTF배 증가
         ↓
  Python train_sac 루프: 변하지 않음 (CPU 연산 상한)
         ↓
  obs_wait_timeout(0.02s) 초과 스텝 증가
         ↓
  실질 step당 대기 시간 증가 → FPS 하락
```

RTF=4 이상에서 시뮬레이터가 파이썬보다 빠르게 달려 **관측값을 버리거나** `obs_wait_timeout` 이후 강제 진행하게 됨.

### 시스템 구성 요소별 병목 계층

```
[Gazebo Physics] → [PX4 SITL] → [ROS2 bridge] → [Python RL loop] → [GPU SAC update]
         시뮬레이터 측 ──────────────────────────────── Python 측 ──────────────────
              RTF로 가속 가능                          가속 불가 (CPU 바운드)
```

현재 병목: **Python RL loop** (obs 처리, action 계산, SB3 내부 로직)

---

## 적용 규칙

### RTF 선택 기준

| RTF | 권장 여부 | 이유 |
|-----|---------|------|
| 1 | 디버깅 전용 | 시뮬 동작 직접 관측 가능 |
| **2** | **기본 학습 ✅** | 최적 fps, 안정적 |
| 4 | 사용 금지 | fps 역전, obs 유실 위험 |
| >4 | 사용 금지 | 더 심각한 병목 예상 |

### 향후 RTF 상한 향상 조건

RTF > 2로 이득을 얻으려면 Python 루프를 먼저 최적화해야 함:

1. **obs 처리 C 확장화:** Cython 또는 numpy vectorize
2. **PyTorch AMP (mixed precision):** GPU SAC update 속도 향상
3. **비동기 obs 수집:** actor/collector 분리 (SB3 기본 구조 변경 필요)

---

## WandB 확인 방법

RTF dry-run 3종 비교:
- `dryrun-RTF1-2026-04-16` → `mtx7ud6o`
- `dryrun-RTF2-2026-04-16` → `x8jq9fsy`
- `dryrun-RTF4-2026-04-16` → `u8w3xn0w`

WandB 그룹 `rtf-comparison`에서 `time/fps` 메트릭 비교로 재현 가능.

---

## 관련 노트

- [[experiments/exp_003_rtf_dryrun]] — 원본 실험 기록
- [[research/rl_rules]] — Rule 7: RTF 선택 규칙
- [[research/system_overview]] — 시스템 전체 구조
