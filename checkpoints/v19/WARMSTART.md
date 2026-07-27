# v19 Warm-Start Checkpoints

v19 (staged integration #3: perception + physics + **real physical payload drop**)
학습 결과를 남이 이어서(warm-start / resume) 쓸 수 있도록 공유하는 체크포인트입니다.

## 무엇이 들어있나

| 폴더 | run | 만든 커밋 | 성능 (best 기준) |
|------|-----|-----------|------------------|
| `precise/` | v19_precise (정밀도 push) | `09a53a2` | release 100%, success ~0.66, drop_err ~0.39 m |
| `abd/`     | v19_abd (collapse fix A+B+D) | `2f2bf9b` | release 100%, success ~0.42, drop_err ~0.70 m |

각 폴더:
- `model_final.pt` — 옵티마이저 상태 + iteration 포함 → **이어학습(resume)용**
- `model_best.pt`  — 평가 최고 체크포인트 → **새 실험의 시드 / 추론용**

환경: task `Isaac-DroneBombard-V19-Direct-v0`, obs **28-D**, action **7-D** (모든 체크포인트 동일).
코드 체인: **v18 → abd(`2f2bf9b`) → precise(`09a53a2`)**. obs/action/네트워크가 전부 동일해
서로 무손실 로드됩니다(보상 토글만 차이).

## 이어학습 (resume — 옵티마이저까지 그대로)

```bash
# 반드시 학습 당시 코드로 맞추기 (예: precise)
git checkout 09a53a2

# source 순서 필수: root -> workspace (틀리면 px4_msgs import 에러로 silent crash)
source /root/ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

python isaac_lab/train.py \
  --task Isaac-DroneBombard-V19-Direct-v0 \
  --resume ./checkpoints/v19/precise/model_final.pt
```

## 새 실험의 시드로만 쓰기 (weights만 물려받고 새로 학습)

```bash
python isaac_lab/train.py \
  --task Isaac-DroneBombard-V19-Direct-v0 \
  --resume ./checkpoints/v19/precise/model_best.pt
```

## 주의사항

- **`model_final` vs `model_best`**: 이어학습은 `model_final`(옵티마이저·iter 포함), 시드/추론은 `model_best`.
- **보상 공식을 바꾸면**: PPO는 on-policy라 weight warm-start 자체는 OK지만, value function이 새 보상 스케일에 재적응해야 하므로 초반 value_loss가 튑니다(정상). replay 재사용 개념은 없음.
- **커밋을 맞추세요**: 다른 커밋의 env cfg로 로드하면 obs/action 차원이 어긋나 로드 실패할 수 있습니다. 표의 커밋으로 `git checkout` 후 resume.
- resume 로직: `train.py --resume <ckpt>` → `runner.load(...)` (rsl_rl `OnPolicyRunner`, 모델+옵티마이저 무손실 로드).
