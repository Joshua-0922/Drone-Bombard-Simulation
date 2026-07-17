---
date: 2026-07-16
tags: [experiment, isaac, v16, physical-payload, rigidbody, drop, land-terminal, drop-test]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-16_*_v16_dryrun)"
owner: junsang
---

# Exp 011 — Isaac v16: 실제 물리 payload drop (analytic → rigidbody)

> **목적 (이번 주 마무리 과제 1):** 지금까지 drop은 **수식(`ballistic_impact`)으로 착탄점을 계산**하는
> analytic 방식 — 실제로 떨어지는 물체가 없었음. v16은 **실제 RigidObject가 떨어져 지면에 닿은 지점**으로
> 학습. 미래에 시변 바람 등에서 낙하 궤적이 실제로 바뀌므로 필수.
> **출처:** branch `Issac_JS`, `DroneBombardV16Env/Cfg` + base `payload_physics_enabled` 훅, 백업 태그 `week1-v15-analytic`

---

## 설계 (v12 기반 — 물리 drop 메커니즘 격리)

| 요소 | 방식 |
|------|------|
| payload | env마다 `RigidObject`(실린더 0.1kg) → `scene.rigid_objects["payload"]` |
| 운반 | `_step_payload_physics`(물리 100Hz): 매 스텝 드론 아래로 pose/vel 덮어씀(kinematic carry) |
| 투하 | drop 발화 → 덮어쓰기 중단, 드론 속도 상속, 자유낙하 + **바람 2차 항력**(`payload_phys_drag_k`) |
| 착지 | payload local z ≤ 지면 → 실제 착탄 xy 기록 |
| 종료 | **release-terminal 폐기 → payload 착지가 종료.** 투하 후 낙하 관측까지 지속 |
| 투하 후 드론 | **호버 고정(옵션 A)** — 비행 실패조건 gate off (freeze 트랜지언트가 착지 선점 방지) |
| 보상 | 터미널 = **실제 착탄 거리** (analytic은 obs/조준 힌트로만 = "예측→실측") |

base 훅 `payload_physics_enabled`(기본 False) → v11~v15 무손상.

---

## 검증

### ① `play.py --drop-test` (물리 배선 수치 검증)
강제 투하 후 공중 궤적을 drag-free 탄도곡선과 비교(착지 전, 리셋 간섭 없음):

| drag_k | 결과 |
|--------|------|
| 0.02 (초기) | ❌ FAIL — \|dz\|0.55m (종단속도 7m/s, 항력 과대) |
| **0.005 (수정, 전면적 기준)** | ✅ **PASS — 방출 9.86m·forward 4m/s → 하강(6.82m)·\|dz\|0.10m·\|dxy\|0.29m** |

→ 운반→방출→속도상속→중력+drag 낙하→forward 던지기 **배선 정확**.

### ② dry-run (실제 착지 기반 학습)
| iter | success | release | 실제 착탄(`drop_impact_error_m`) |
|------|---------|---------|------|
| 71 | 0.20 | 1.00 | 0.64 m |
| 113 | 0.67 | 1.00 | 0.23 m |
| **156** | **0.80** | 1.00 | **sub-meter** |

→ 정책이 **실제 낙하 payload를 타겟 근처에 착지**시키도록 학습. 물리 drop+착지+보상 **end-to-end 작동 확인.**

---

## 결론 & 다음
- **analytic drop → 실제 물리 drop 전환 성공.** payload가 실제 rigidbody로 떨어지고, 지면 착지 지점으로 학습.
- ⚠️ **영상으로는 아직 안 봄** — drop-test/메트릭은 수치 검증. 낙하 장면 시각 확인은 **과제 2(시각 도구)** 에서.
- 다음: 과제 2(카메라·marker·바닥 개선) → v16 낙하 직접 시청 / 시변 바람 결합.
- 관련: [[experiments/exp_010_v15_airframe_wind_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[experiments/training_history]]
