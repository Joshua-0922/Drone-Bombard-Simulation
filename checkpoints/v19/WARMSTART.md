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

---

## 검토용 GUI로 정책 보기 (play.py --show)

공유된 체크포인트를 GUI에서 재생해 비행/투하를 눈으로 검토할 수 있다. `--show`가
**타깃 비컨·디스크 + 페이로드 마커 + 근접 체이스 카메라**를 켠다.

```bash
# 컨테이너 내부에서 (Isaac Sim 런타임 필요)
# ⚠️ play.py 기본 --task는 v13/v15라, v19를 보려면 task를 반드시 지정할 것
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py \
  --task Isaac-DroneBombard-V19-Direct-v0 \
  --policy ./checkpoints/v19/precise/model_best.pt \
  --show --num_envs 4 --episodes 10
```

옵션:
- `--policy <ckpt>` : 재생할 정책 체크포인트 (검토는 `model_best.pt` 권장)
- `--show`          : 시각 보조(비컨/마커) + 체이스 카메라 ON
- `--wind-test --wind-speed 5` : +X 방향 고정 바람으로 외란 반응 확인
- `--num_envs` / `--episodes` : 동시에 볼 드론 수 / 재생 에피소드 수

### 화면을 보는 방법 (Isaac Sim은 웹앱처럼 링크 라이브가 안 됨)
- **GPU + 드라이버 ≥ 580 필요** (RTX 렌더러).
- **공용 VM(모니터 없음)**: `ssh -X`(X11 포워딩, README §5) 또는 Isaac Sim WebRTC 라이브스트리밍.
- **자기 GPU 머신**: 이미지 빌드 후 로컬 디스플레이/X11로 실행.
- **GPU/Isaac Sim이 없는 사람**: GUI 대신 영상으로 공유 —
  ```bash
  /workspace/isaaclab/isaaclab.sh -p isaac_lab/record_episode.py --headless --enable_cameras \
    --task Isaac-DroneBombard-V19-Direct-v0 \
    --policy ./checkpoints/v19/precise/model_best.pt --video_length 250
  ```
  → 생성된 mp4를 공유하면 상대는 별도 런타임 없이 검토 가능.
