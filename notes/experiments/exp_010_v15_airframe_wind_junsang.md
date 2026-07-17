---
date: 2026-07-16
tags: [experiment, isaac, v15, wind, airframe, residual-saturation, wind-test]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-16_*_v15_dryrun / v15_nores_dryrun)"
owner: junsang
---

# Exp 010 — Isaac v15: 바람이 드론 기체에 실제 작용

> **목적:** v14의 한계(바람이 payload 탄도에만 작용, 드론은 무풍 비행) 해소. 바람이 **기체에 2차
> 항력**으로 작용해 정지유지에 tilt가 들고, "비행 보정만으로 바람을 이기는" 경로를 차단.
> **출처:** branch `Issac_JS`, `DroneBombardV15Cfg` + base `wind_force_enabled` 훅, [[experiments/exp_009_v14_ccip_residual_junsang]]

---

## 구현

컨트롤러(`_run_velocity_controller`)에 추력과 함께 **상대기류 2차 항력** 추가:
`v_air = wind − v_drone`, `F = k·|v_air|·v_air` (`wind_drag_k=0.06`). `wind_force_enabled` 훅(기본 off → v11~v14 무손상).

### 실측 검증 `play.py --wind-test` (수치로 확인)
5 m/s 강제풍에서 정지비행 시 기울기:
- **wind_force OFF: 0.01°** (v14 방식 — 바람이 기체에 전혀 작용 안 함 실증)
- **wind_force ON: 3.50°** (해석 예측 atan(k·5²/mg)=4.02°와 근접 — 실측이 약간 낮은 건 하류로 0.33m/s 밀리며 상대기류 감소)
→ **PASS: 바람이 실제로 기체를 민다.**

---

## dry-run — wind_std=4.0 (첫 시도): 정체 → 원인 규명

| iter | success | 착탄 (ON) | 착탄 (OFF) |
|------|---------|-----------|------------|
| ~150 | ~0 | ~3.2 m 정체 | ~3.0 m (release 0 붕괴) |

**둘 다 착탄 ~3m 정체·success≈0.** 로컬 진단으로 원인 발견:

> **residual(±3m) 포화.** wind_std=4.0에선 payload 드리프트가 **3.7~7.7m**로 residual 범위(±3m)를
> **초과** → residual이 물리적으로 다 못 메움. (v14 wind 1.5에선 드리프트<2m라 충분했음.)
> 또한 대조군(residual off)은 **release 0으로 붕괴(착탄 11m)** — 바람이 기체를 흔드니 "비행 보정"만으론
> 부족 → **바람이 실제 작용할 때 residual의 가치가 v14보다 훨씬 큼**을 시사.

→ 규칙화: **residual_scale는 드리프트 크기를 커버해야 함.** → [[research/rl_rules]]

---

## 조치: wind_std 4.0 → 2.0

drift가 residual 범위에 들어오도록 하향 (cap 5, obs scale 6):
- 드리프트 중앙값 **2.4m < residual 3m** (전형 경우 보정 가능), 강풍 tail 33.8%만 일부 포화.
- 바람은 여전히 기체를 ~1~2° 기울임.
- **재실행은 안 함(세팅만)** — 사용자 판단으로 다음으로 넘어감.

## 남은 것 / 다음
- [ ] **v15 dry-run(wind 2.0) 미실행** — 다음에 residual on/off 대조.
- 향후 현실화: 시변 바람(돌풍/난류), 수직풍 → v16 물리 payload와 결합 시 낙하궤적 실제 변화.
- 관련: [[experiments/exp_009_v14_ccip_residual_junsang]] · [[experiments/exp_011_v16_physical_drop_junsang]] · [[research/isaac_expansion_roadmap_junsang]]
