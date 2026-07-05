---
date: 2026-07-04
updated: 2026-07-05
tags: [research, isaac-lab, ablation, experiment-design, ppo, vision]
status: executed
type: research
---

> **2026-07-05 실행 결과** ([[experiments/exp_014_A2_visionrange]]): cheap probes
> P1/P2/P3 실행 → 세 probe 수렴(§4 예측대로) → **A1 arm 생략, A2 직행**. 단,
> 실행 중 **inertia 대발견**([[research/isaac_inertia_ctrl_mismatch]])으로 이 문서의
> 전제 두 가지가 수정됨: ① "A1 diff에서 inertia는 native 유지"는 무의미했음 —
> exp_013 solver inertia는 처음부터 x500 0.0217이었다(전파 안 된 것은 컨트롤러의
> 읽기 쪽). ② P1(구 정책 × 수정 plant)은 "불변이면 킥-무관"을 예측했으나 실제로는
> **정책의 plant-overfit**(bad_attitude 68%)이 지배 — 그럼에도 climb 잔존(22-27%,
> +3 m/s 명령 rail)으로 물리 배제 결론은 그대로 성립. A0′(수정 plant, 감쇠 없음)가
> 새 기준선으로 필요해짐 — A2 붕괴 확인 시 실행.

# exp_014 ablation 설계 — max_altitude 33%의 원인 분리 (physics-kick vs climb-and-farm)

> **설계만, 미실행.** 근거: [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] §4d
> (07-04 forensics) + 학습 커브 창발 분석. **forensics가 이미 사전 확률을 크게 기울였다:**
> 킥은 프로세스당 1회(첫 물리 substep)라 지속 27-43% max_alt rate의 원인일 수 없고,
> 그 rate는 iter ~200에서 창발(0-199 구간 ~0%)한 학습된 행동이다. 따라서 이 ablation의
> 목적은 "동률 후보 분리"가 아니라 **잔여 불확실성 제거 + attractor 가설(비전 거리감쇠
> 누락)의 인과 확증**이다.

---

## 0. 사전 확률 (forensics 결과 반영)

| 가설 | 사전 판정 | 근거 |
|---|---|---|
| (A) 물리 킥이 max_alt 33% 유발 | **사실상 기각** | 킥=프로세스당 1회 실증(`_diag_kick.py`); 창발-지속 패턴(iter 200~)과 양립 불가; 학습 노출 ~0.3% 에피소드 |
| (B) 비전 climb-farming attractor | **1차 가설** | 기하 실증(u_n∝x/z, conf 거리무관) + 창발 구간 rew_vision 고유지(17.3/16.6) + 실YOLO에선 불가능한 정책 |
| (C) 제3원인 (noise_std 폭주의 vz 랜덤워크 등) | 보조 가설 | noise_std는 0.89→3.78 단조 — 창발(200)과 타이밍 불일치(그 시점 1.3), 단독 설명 부족 |

## 1. Arms (각각 정확한 diff)

| Arm | Diff (이것만 변경) | 목적 |
|---|---|---|
| **A0** 베이스라인 | 없음 — exp_013 데이터(wcjklw7a) 재사용. 추가로 seed 43 재학습 1회(400 iters)로 seed 분산 추정 | 임계값 산정용 분산 |
| **A1** 킥 수정만 | `__init__` 말미에 zero-wrench sim step 1회 추가(t2first 실증 경로 — 최소 diff) **또는** 스폰타임 `mass_props`(질량만; **inertia는 native 유지** — x500 inertia baking은 회전 plant 변경 confound). 다른 것 일절 불변, seed 42 | (A) 최종 기각 확인. 예상: max_alt rate 변화 없음(±seed 노이즈) |
| **A2** 비전 거리감쇠만 | `_update_vision`에 conf·탐지확률 slant-range 감쇠(킥은 그대로 둠), seed 42 | (B) 인과 확증. 예상: max_alt 붕괴 |
| **A3** (조건부) 둘 다 | A1+A2 | A1/A2가 모두 중간값일 때만 |

## 2. Run 길이·예산

- **400 iterations/arm** (≈17분 GPU @29K steps/s). 근거: max_alt 창발이 iter ~200
  (50-iter 윈도우 기준 200-249에서 0.333; 0-199는 ≤0.088) → 창발 확인에 2× 마진.
- 판정 윈도우: **iters 300-400의 `Episode_Termination/max_altitude` 평균** (R_alt).
  A0 기준 R_alt(300-400) ≈ 0.35-0.43.
- 각 arm 종료 후 deterministic eval 200-ep(±4분) — **env당 첫 에피소드는 통계에서 제외**
  (A1 미적용 arm의 프로세스-시작 킥 아티팩트 배제; 32 envs면 최대 32ep 제외).
- 총 GPU: A0(seed43)+A1+A2 = 3×(17+4)분 ≈ **63분**; A3 필요 시 +21분.
- 통계 노이즈: iteration당 종료 에피소드 ~600 → 100-iter 윈도우 binomial SE ≈ 0.2%p로
  무시 가능; **지배 불확실성은 seed 분산** — A0-seed43과 exp_013(seed42)의 R_alt 차가
  추정치(예상 ±5-8%p).

## 3. 판정 임계값 (decision tree)

R_alt := arm의 iters 300-400 max_altitude 종단율, B := 베이스라인 R_alt (~0.39), σ_s := seed 분산 추정.

1. **A1 R_alt ∈ B ± max(0.08, 2σ_s)** → 킥 무관 확정 (예상 결과). A1이 B보다 0.15+ 낮으면
   forensics와 모순 → forensics 재검토(이 경우 A1의 diff가 의도치 않게 plant를 바꿨는지 먼저 확인).
2. **A2 R_alt < 0.10** → attractor 인과 확증 (**B 확정**). A2에서 `Episode_Reward/rew_vision`이
   함께 하락하고 success가 유지/상승하는지 동반 확인(감쇠가 비전 서보잉까지 죽였으면 success도
   붕괴 — 그 경우 감쇠 커브가 과함).
3. **A2 R_alt ∈ [0.10, 0.25]** → 부분 기여 → A3 실행: A3 < 0.05면 (B)+(A1 diff의 잔여효과)로
   정리; A3도 ≥0.10이면 **제3원인 탐색** — 우선 후보 (C): entropy_coef=0 변형 1개 추가
   (400 iters)로 noise_std 고정 후 재측정.
4. success rate는 부차 판정: A2에서 R_alt가 무너졌는데 success가 36%에서 정체면
   max_alt가 crash/stagnation으로 **전이**한 것 — 종단 분포 전체를 비교할 것 (attractor
   제거≠완주 학습; 완주는 reward_success 300의 몫, [[research/isaac_ppo_tuning_recommendations]] #2).

## 4. 무학습 cheap probes (ablation 전에 즉시 실행 가능, 각 ~3-5분)

| Probe | 방법 | 결론 가능 범위 |
|---|---|---|
| **P1** 기존 정책 × 킥-수정 plant | exp_013 `model_final.pt`를 A1 plant에서 200-ep eval | max_alt rate가 (첫 에피소드 제외 기준) 불변이면 킥-무관의 **무학습 확증**. 단, 학습된 정책의 행동 원인까지는 못 밝힘 |
| **P2** 고도 궤적 판독 | eval 중 max_alt 종단 에피소드의 z(t) 기록: 스폰 직후부터 단조 상승(전략적 climb)인지, 접근 실패 후 상승 전환인지 | climb의 "의도성" 분류 — B 가설이면 타겟을 프레임에 유지한 채 상승하는 패턴(u,v 중앙 근처 + z↑)이 관찰돼야 |
| **P3** 보상 반사실 | 기록된 climb 궤적에 감쇠-conf를 오프라인 적용해 r3_vision 재계산 | "감쇠가 있었으면 climb 수익이 사라졌는가" 정량화. env 재실행 불필요, 수 분 |

P1+P2가 B 가설과 정합하면 ablation 자체를 A2 단독으로 축소해도 무방(GPU 42분 → 21분).

## 관련

- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] §4d — forensics 근거
- [[research/isaac_ppo_tuning_recommendations]] — exp_014 본 변경 목록
- [[isaac_mass_override_reset_bug]] (Claude memory) — 킥 메커니즘 확정 기록
- [[research/rl_rules]] Rule 17 — attractor 원리
