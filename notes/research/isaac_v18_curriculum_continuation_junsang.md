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

| 단계 | 설정 | 체크포인트 / 백업 | 결과 |
|-------|------|----------------------------------------|------|
| **v18 P1 (완화)** | gate 1.5·res 2.0·wind 1.5·k 0.12 | `.../2026-07-18_07-18-36_v18_eased/model_200.pt` | success 1.0, 착탄 0.53m |
| **v18 P2 (hard)** | gate 1.0·res 3.0·wind 2.0·k 0.15 | `.../2026-07-18_*_v18_phase2/model_300.pt` (노트북 `~/v18_backup/`) | success 1.0, 착탄 ~0.65m |
| **v19 (통합+물리drop)** | +`payload_physics_enabled` | v18-P2 warm-start; ⚠️ iter499 **붕괴**(release 0)·iter375 유실 | (붕괴, 폐기) |
| **v19_abd (붕괴수정 A+B+D)** | +포텐셜 shaping·loiter·best저장 | `2026-07-23_02-31-51_v19_abd_full/model_best.pt`(iter599) · 노트북 `~/v19_backup/v19_abd_run/` | success 76.7%·착탄 0.563m |
| **v19_precise (연속착탄) ⭐현재 best** | +`precise_landing_reward`·k_land 2.0 | `2026-07-23_07-58-11_v19_precise/model_best.pt`(iter875) · 노트북 `~/v19_backup/v19_precise_run/` · VM `~/v19_precise_backup/run/`·`v19_precise_best.pt` | **success 100%·release 100%·착탄 0.356m** |

> ✅ **백업 durable 완료(2026-07-23):** v19_abd·v19_precise run 전체(각 15 ckpt)를 **노트북 `~/v19_backup/`** 에 회수 완료. v18-P2는 `~/v18_backup/`. 코드 `origin/Issac_JS`(붕괴수정 `2f2bf9b`, 정밀 `09a53a2`).
> **다음 재개 출발점 = `v19_precise` best(iter875).**

---

## 재개 방법 (검증된 명령)

**패턴:** best model을 `--resume`으로 이어받고, 새 난이도/기능/보상을 켠다. obs 28D·action 7D 동일하면 clean warm-start.

```bash
# 실제 사용한 정밀화 재학습 (v19_abd best → 연속 착지보상):
train.py --v19 --resume /workspace/drone-bombard/v19_best.pt \
  --num_envs 1024 --max_iterations 300 --seed 42 --headless \
  --logger tensorboard --run_name v19_precise
# 이후 best 선정 (실제착탄 success 기준):
select_best_checkpoint.py --task Isaac-DroneBombard-V19-Direct-v0 \
  --log_dir <run_dir> --num_envs 64 --episodes 150 --headless
```
> ⚠️ VM `~/wt-js`는 **git 아님(코드 복사 dir)** → 코드 변경 시 `gcloud compute scp`로 파일 반영 후 학습.

**다음 후보:** (a) release_radius 1.5→1.0 커리큘럼(정밀↑) (b) k_landing 추가 상향 (c) 다중시드 견고화 (d) 이동 타겟 (e) 실제 핀홀 카메라/YOLO vision (f) sim-to-real.

## 핵심 교훈 (재개 시 명심)
- **from-scratch로 다 켜면 데드락**(초기 랜덤 residual이 게이트 봉쇄) → **완화 Phase 부트스트랩 후 warm-start로 난이도↑** (Rule 12).
- **shaping은 "개선"에만**(포텐셜/차분형). 상주형은 수렴 후 no-drop 붕괴 유발 (Rule 13). → [[research/isaac_v19_collapse_nodrop_junsang]]
- **보상 바꾸면 재적응 딥**(release가 잠깐 0까지)→회복. **조기종료 금지 + best-checkpoint 저장**(save_interval 25 + `select_best_checkpoint.py`). log_dir 절대 비우지 말 것(iter375 유실 재발 방지).
- **정밀도는 명시적으로 보상에 넣어라**: 성공존 내부까지 연속 gradient(평평보상 금지) → 착탄 0.56→0.36m.

---

## 관련 노트
- [[experiments/exp_013_v18_integration_curriculum_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[research/rl_rules]] · [[experiments/training_history]] · [[00_index]]
