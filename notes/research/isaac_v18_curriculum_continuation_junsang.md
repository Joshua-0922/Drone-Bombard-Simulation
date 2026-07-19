---
date: 2026-07-18
tags: [research, isaac, v18, curriculum, checkpoint, continuation, backup]
type: research
status: active
owner: junsang
---

# Isaac v18 커리큘럼 — 학습결과 백업 & Phase 3+ 재개 가이드

> **목적:** v18 통합 커리큘럼(Phase 1→2)의 **학습결과를 백업**하고, **Phase 3,4,5...를 이어서**
> 할 수 있도록 체크포인트 위치·재개 명령을 남긴다.
> 관련: [[experiments/exp_013_v18_integration_curriculum_junsang]]

---

## 현재까지 (커리큘럼 사다리)

| Phase | 설정 | 체크포인트 (VM: js-v11 컨테이너 내부) | 결과 |
|-------|------|----------------------------------------|------|
| **1 (완화)** | gate 1.5·res 2.0·wind 1.5·k 0.12 | `/workspace/logs/isaac_lab/drone_bombard/drone_bombard_ppo/2026-07-18_07-18-36_v18_eased/model_200.pt` | success 1.0, 착탄 0.53m |
| **2 (hard)** | gate 1.0·res 3.0·wind 2.0·k 0.15 | `.../2026-07-18_*_v18_phase2/model_*.pt` (최신 주기저장, ~model_300) | success 1.0, 착탄 ~0.65m |

> ⚠️ **체크포인트 물리 백업 상태:** 현재 **VM(js-v11 컨테이너 writable layer)에만** 존재
> (VM 정지해도 컨테이너 파일은 보존됨). **다음 VM 세션에서 노트북 `~/`으로 회수**해 durable 백업 예정.
> 코드는 `origin/Issac_JS` + 태그로 백업됨(재현 가능).

---

## Phase 3+ 재개 방법

**패턴:** 이전 phase의 model을 `--resume`으로 이어받고, 새 난이도/기능을 켠다.

```bash
# 예: Phase 3 = Phase 2 모델에서 이어, 더 어렵게 or 새 기능(+물리 drop 등)
train.py --v18 --v18_hard \
  --resume <Phase 2 model_*.pt> \
  --num_envs 512 --max_iterations 200 --run_name v18_phase3 ...
```

- **차원 동일**(obs 28D·action 7D)이면 warm-start clean 로드. 기능 추가로 obs/action 차원이 바뀌면 부분로드 필요.
- Phase 3 후보: (a) 난이도 더↑(바람·픽셀 애매성) (b) **+물리 drop(v16)** 통합 (단 obs/action 재설계) (c) 이동 타겟.

## 핵심 교훈 (재개 시 명심)
- **from-scratch로 다 켜면 데드락**(초기 랜덤 residual이 게이트 봉쇄) → 반드시 **완화 Phase에서 부트스트랩 후 warm-start로 난이도↑**. → [[research/rl_rules]] Rule 12.
- 새 능력 추가 시에도 "성공경험을 한 번은 하게" 완화부터.

---

## 관련 노트
- [[experiments/exp_013_v18_integration_curriculum_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[research/rl_rules]] · [[experiments/training_history]] · [[00_index]]
