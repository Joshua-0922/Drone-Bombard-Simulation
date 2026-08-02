---
date: 2026-07-15
tags: [experiment, isaac, v12, expansion, random-marker, generalization, drop-signal]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-15_*_v12_dryrun)"
owner: junsang
---

# Exp 007 — Isaac v12: 첫 확장 (랜덤 marker 스폰) 일반화 검증

> **목적:** v11(고정 marker)에서 학습된 drop 정책이 **다양한 위치의 타겟에 일반화**되는지 확인.
> v11의 첫 확장 축 = "고정 marker → 랜덤 marker" (한 지오메트리 암기가 아닌지 검증).
> **출처:** branch `Issac_JS`, `isaac_lab/drone_bombard/v11_env.py` (marker_random 토글), [[experiments/exp_006_v11_dryrun_junsang]]

---

## 설계 (삭제 없이 토글)

v11 env를 그대로 두고 **`marker_random` cfg 플래그**만 추가:

| 항목 | 값 |
|------|-----|
| marker 스폰 | (20, 0) 점 **중심 반경 5m 원(disk) 안 면적-균일 랜덤** (`r=5·√U`, `θ=U(0,2π)`) |
| 드론 스폰 | (0,0,10m) 고정, **+X로 순항 시작** (방식 A) → marker가 옆(최대 ±5m≈±14°)이면 **조향**해서 정렬 |
| 나머지 | drop_signal·release envelope·cruise 핸드오프·nominal 물리 = **v11과 동일** |

- `DroneBombardV12Cfg(DroneBombardV11Cfg)`: `marker_random=True`, `marker_spawn_radius=5.0`
- 태스크 `Isaac-DroneBombard-V12-Direct-v0` (env 클래스는 v11 재사용), `train.py --v12`
- 로컬 검증: py_compile OK, pytest 45 passed, disk 샘플링 정확(|offset|≤5, 평균 3.34=2R/3 면적균일).

---

## 결과 (VM L4, js-v11 컨테이너)

| 단계 | 설정 | 결과 |
|------|------|------|
| smoke | 16env / 3iter | ✅ env 구성·랜덤 marker 동작·에피소드 ~18스텝(step-1 즉사 없음)·EXIT 0 |
| **dry-run** | **512env / 300iter** | ✅ **~iter 49 수렴 시작** |

**dry-run 수렴 지표 (iter 113~):**
- `success` = **1.00 (100%)**
- `release_rate` = **1.00 (100%)**
- `drop_impact_error_terminal` = **~0.75 m** (v11 0.43m 대비 약간↑ — 랜덤 위치라 조향 필요, 여전히 radius 1.0 이내)
- `Mean reward` = **~316**

→ **v11의 학습이 랜덤 위치 타겟에 성공적으로 일반화.** (20,0)±5m 원 어디에 marker가 생겨도 드론이 순항하며 조향·정렬·투하해 100% 성공. 한 지오메트리 암기가 아니었음이 확인됨.

---

## 다음 확장 축 (순서 예정)

- [ ] **A로 난이도↑**: 방위 원뿔 ±30~45°로 확대 (더 큰 조향) / 거리·고도 범위 확대
- [ ] wind / domain randomization (+ CCIP residual `action[5]`) — wind trap 주의
- [ ] 이동 타겟 (moving target)
- [ ] vision 복원 (카메라 인지)
- 관련: [[experiments/exp_006_v11_dryrun_junsang]], [[research/isaac_cruise_handoff_junsang]], [[experiments/training_history]]
