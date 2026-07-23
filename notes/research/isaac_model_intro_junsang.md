---
date: 2026-07-23
tags: [research, isaac, intro, overview, walkthrough, onboarding]
type: research
status: active
owner: junsang
---

# 드론 정밀 투하 모델 — 한 에피소드로 보는 소개

> **이 문서는?** 우리 최종 통합 모델(**v19**)이 **한 에피소드 동안 무슨 일을 겪는지**를 이야기처럼
> 따라가는 소개서. 각 단계마다 **① 무슨 일이 일어나나 ② 그걸 어떻게 해냈나 📁 직접 찾아볼 위치**를 붙임.
> 스펙 수치는 [[research/isaac_model_spec_junsang]], 확장 이력은 [[research/isaac_expansion_roadmap_junsang]].

---

## 한눈에

> **바람 부는 환경에서, 처음엔 안 보이는 랜덤 위치의 표적을, 순항하다 가까워지면 인지하고,
> 탄도를 바람에 맞춰 보정한 뒤, 고도를 낮춰 실제 payload를 투하해 명중시킨다.**
> 학습: PPO(rsl_rl), 관측 **28D** → 행동 **7D**. 현재 best `v19_precise`(iter875): **명중률 100%, 실제 착탄 0.356m**.

코드: `Issac_JS/isaac_lab/drone_bombard/` (`v11_env.py` = 버전별 로직, `drone_bombard_env.py` = 물리/컨트롤러, `math_utils.py` = 게이트·탄도).

---

## 에피소드 흐름

### 1. 드론이 스폰된다 — 이미 순항 중인 상태로
- **무슨 일:** 드론이 고도 **10 m**에서 **+X 방향 4 m/s로 이미 날고 있는 상태**로 등장. 정지 상태가 아니라 "순항 중 인계(handoff)" 지점.
- **어떻게:** 스폰 시 위치·속도를 cruise로 세팅. 단, 여기서 함정 하나 — 컨트롤러 setpoint를 0으로 리셋하면 첫 스텝에 "속도 4→0" 급명령이 들어가 **자세 폭주로 즉사**함. 그래서 **리셋 때 컨트롤러를 cruise 속도로 seed**해 부드럽게 이어받게 함.
- 📁 `v11_env.py:_reset_idx`(스폰·컨트롤러 seed, ~L416/L444) · [[research/isaac_cruise_handoff_junsang]] (Rule 10)

### 2. 표적이 랜덤으로 생기고, 바람이 분다
- **무슨 일:** 표적(marker)이 정면 (20 m, 0) 중심 **반경 5 m 원 안 랜덤 위치**에 생성. 동시에 이번 에피소드용 **바람(wind_xy)과 항력(drag)이 랜덤 샘플**됨 → 매판 환경이 다름.
- **어떻게:** 표적은 면적 균일 랜덤 스폰(방식 A). 바람/항력은 **에피소드마다 뽑는 도메인 randomization(DR)** — 정책이 특정 바람에 과적합하지 않고 일반화하도록. 바람은 나중에 payload와 **기체에도 실제 힘**으로 작용.
- 📁 `v11_env.py:_reset_idx`(marker random) · `drone_bombard_env.py:_run_velocity_controller`(기체 바람) · [[experiments/exp_007_v12_random_marker_junsang]] · [[experiments/exp_010_v15_airframe_wind_junsang]]

### 3. 드론이 순항한다 — 아직 표적은 "안 보임"
- **무슨 일:** 표적이 어디 있는지 **모른 채** +X로 계속 순항. 관측(obs)에서 표적 위치 채널이 **가려져(0으로 마스킹)** 있음.
- **어떻게:** "부분관측" — 표적과 **수평거리 ≤ 7 m**가 되기 전엔 위치 obs를 마스킹하고 `detected=0`. 못 찾는 동안 매 스텝 작은 페널티(−0.2)를 줘서 **가만있지 말고 탐색하도록** 유도.
- 📁 `v11_env.py:V13Env._get_observations`(reveal 마스킹, ~L776) · [[experiments/exp_008_v13_partial_obs_junsang]]

### 4. 인지할 수 있게 되면 — 조준·학습이 본격 시작
- **무슨 일:** 드론이 표적 **7 m 이내**로 들어오면 표적이 "보이기" 시작 → 여기서부터 조준·투하 판단.
- **어떻게 (인지):** 진짜 카메라 대신 **픽셀 양자화** 프록시 — 표적 위치를 정확히가 아니라 **셀 중심으로 스냅**. 셀 크기 = 0.12·(드론↔표적 거리) → **멀면 셀 커서 애매, 가까울수록 정밀**. 그래서 "애매하게 보고 다가가며 다듬는" 실제 비전 감각을 흉내.
- 📁 `v11_env.py:_quantize_target` · [[experiments/exp_012_v17_pixel_vision_junsang]] · 스펙 §8 [[research/isaac_model_spec_junsang]]

### 5. 바람에 맞춰 탄도를 보정한다 (CCIP residual)
- **무슨 일:** "지금 떨어뜨리면 어디 맞나"를 계산(CCIP)하되, **바람 때문에 흘러갈 양을 미리 당겨 보정**.
- **어떻게:** 무바람 탄도 예측 $p_{nom}$ 에 **정책이 낸 보정 $a_{[5:7]}$**(±2~3 m)을 더한 $p_{corr}$ 를 씀. 정책은 obs로 받은 바람값을 보고 "이만큼 흘러가니 이만큼 미리 조준"을 학습. 조준·게이트·보상 모두 이 **보정된 예측** 기준, **명중 판정만 실제 물리**.
- 📁 `v11_env.py:_ccip` · 스펙 §9 · [[experiments/exp_009_v14_ccip_residual_junsang]] (Rule 11: residual이 드리프트보다 작으면 포화)

### 6. 고도를 살짝 낮춰 투하 조건을 연다 (release 게이트)
- **무슨 일:** 10 m 순항 고도에서 **고도를 낮춰** 투하 가능 구간으로 들어가고, 조준·속도·자세가 다 맞으면 **투하**.
- **어떻게:** **release envelope 게이트** — ⓐ 보정 예측 착탄오차 ≤ 1.5 m ⓑ **고도 3~8 m** ⓒ 속도·수직속도·기울기·각속도가 한계 이내 — 이 전부를 만족해야 게이트가 열림. 그래서 드론은 자연히 **8 m 아래로 내려오며(=살짝 하강)** 조건을 맞춘 뒤, 정책의 **투하 신호(action[4]>0.5)**가 나면 실제로 투하.
- 📁 `math_utils.py:release_gate` · `v11_env.py:V19Env._get_dones`(fire 판정, ~L646) · 스펙 §3

### 7. payload가 실제로 분리·낙하해 명중한다
- **무슨 일:** 투하 순간 payload가 **드론에서 분리** → **바람 속을 자유낙하**(궤적이 휨) → 지면 착지. 드론은 그 자리 호버. **payload가 착지하면 에피소드 종료**, 명중 여부는 **실제 착지점**으로 판정.
- **어떻게:** payload는 시각효과가 아니라 **진짜 RigidObject**. 부착 중엔 드론에 매달려 이동, 투하되면 중력+바람 항력으로 100 Hz 물리 낙하 → 착지 좌표 기록. (analytic 수식이 아니라 실제 물리라 "진짜로 떨어짐".)
- 📁 `drone_bombard_env.py:_step_payload_physics` · [[experiments/exp_011_v16_physical_drop_junsang]] · [[experiments/exp_014_v19_full_integration_junsang]]

---

## 그래서 학습은 무엇을 배우나 (한 줄)
매 스텝 **관측(28D: 상대위치·속도·자세·CCIP오차·바람·detected 등)** 을 보고 **행동(7D: 속도4·투하신호1·탄도보정2)** 을 내며, **보상**으로 "가까이·정확히 조준하고, 조건 맞으면 투하해, 실제로 명중"을 강화학습(PPO).

**보상 설계의 두 급소 (직접 겪고 고친 것):**
- **투하를 아예 안 하게 붕괴** → 조준 유지에 상주 보상을 주면 "완벽 조준 + 안 던지고 호버"가 이득이 됨 → shaping을 **"유지가 아니라 개선에만"**(포텐셜/차분형)으로 바꿔 해결. → [[research/isaac_v19_collapse_nodrop_junsang]] (Rule 13)
- **정밀도 정체(0.56 m)** → 성공존 안에서 보상이 평평하면 더 정밀할 이유가 없음 → **연속 착지보상**으로 0 m까지 계속 당김 → **0.356 m·명중률 100%**. → [[experiments/exp_016_v19_precision_landing_junsang]]

---

## 더 보기
- 전체 파라미터·수식: [[research/isaac_model_spec_junsang]]
- 능력 확장 이력·다음 계획: [[research/isaac_expansion_roadmap_junsang]]
- 실험 이력: [[experiments/training_history]] · 규칙: [[research/rl_rules]] · 대시보드: [[00_index]]
