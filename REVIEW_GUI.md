# 검토용 GUI (play.py) 사용법

학습된 **아무 정책 체크포인트든** GUI에서 재생해 비행/조준/투하를 눈으로 검토하는 방법.
특정 버전(v19)이나 warm-start와 무관하게, 모든 버전에 공통으로 적용된다.

## 실행

```bash
# 컨테이너 내부에서 (Isaac Sim 런타임 필요)
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py \
  --task <TASK_ID> \
  --policy <path/to/model.pt> \
  --show --num_envs 4 --episodes 10
```

- `--policy <ckpt>` : 재생할 체크포인트(.pt). 검토는 보통 `model_best.pt`.
- `--show`          : 시각 보조(타깃 비컨/디스크 + 페이로드 마커) + 근접 체이스 카메라 ON.
- `--wind-test --wind-speed 5` : +X 방향 고정 바람으로 외란 반응 확인.
- `--num_envs` / `--episodes`  : 동시에 볼 드론 수 / 재생 에피소드 수.
- 그 외: `--zero-actions`(호버 sanity), `--scripted`, `--step-response`.

## ⚠️ 버전 ↔ TASK_ID 를 반드시 맞출 것

`--policy`의 체크포인트를 만든 **환경 버전과 같은 `--task`** 를 줘야 obs/action 차원이
맞아 로드된다. play.py 기본값은 `Isaac-DroneBombard-Direct-v0`(v13/v15 계열)이므로,
다른 버전을 볼 때는 **항상 명시**해야 한다.

| 학습 버전 | `--task` id |
|-----------|-------------|
| v11 | `Isaac-DroneBombard-V11-Direct-v0` |
| v12 | `Isaac-DroneBombard-V12-Direct-v0` |
| v13 | `Isaac-DroneBombard-V13-Direct-v0` |
| v14 | `Isaac-DroneBombard-V14-Direct-v0` |
| v15 | `Isaac-DroneBombard-V15-Direct-v0` |
| v16 | `Isaac-DroneBombard-V16-Direct-v0` |
| v17 | `Isaac-DroneBombard-V17-Direct-v0` |
| v18 | `Isaac-DroneBombard-V18-Direct-v0` |
| v19 | `Isaac-DroneBombard-V19-Direct-v0` |
| (기본/레거시 v13·v15) | `Isaac-DroneBombard-Direct-v0` |

예 — 공유된 v19 체크포인트 검토:
```bash
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py \
  --task Isaac-DroneBombard-V19-Direct-v0 \
  --policy ./checkpoints/v19/precise/model_best.pt --show
```

## 화면을 보는 방법 (Isaac Sim은 웹앱처럼 링크 라이브 불가)

- **GPU + 드라이버 ≥ 580 필요** (RTX 렌더러).
- **공용 VM(모니터 없음)**: `ssh -X`(X11 포워딩, README §5) 또는 Isaac Sim WebRTC 라이브스트리밍.
- **자기 GPU 머신**: 이미지 빌드 후 로컬 디스플레이/X11로 실행.
- **GPU/Isaac Sim이 없는 사람**: GUI 대신 영상으로 공유 —
  ```bash
  /workspace/isaaclab/isaaclab.sh -p isaac_lab/record_episode.py --headless --enable_cameras \
    --task <TASK_ID> --policy <path/to/model.pt> --video_length 250
  ```
  → 생성된 mp4를 공유하면 상대는 별도 런타임 없이 검토 가능.

## 관련
- 공유된 v19 체크포인트: [`checkpoints/v19/`](checkpoints/v19/) · [`WARMSTART.md`](checkpoints/v19/WARMSTART.md)
