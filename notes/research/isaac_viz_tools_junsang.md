---
date: 2026-07-19
tags: [research, isaac, visualization, livestream, play, task2, markers]
type: research
status: active
owner: junsang
---

# Isaac 시각화 & 검증 도구 (과제 2)

> **목적:** 학습 결과를 **직접 눈으로** 보기 위한 시각/검증 도구 정리. play.py 모드 + 마커 +
> 라이브스트림 + 시행착오.
> 관련: [[experiments/exp_014_v19_full_integration_junsang]]

---

## play.py 검증/시청 모드 (체계화)

| 모드 | 용도 |
|------|------|
| `--policy CKPT` | 학습 모델 롤아웃 + 에피소드별 success/착탄 통계 |
| `--show` | 시청용: 마커(타겟 비콘·payload) ON + **근접 chase 카메라** (`show_markers`, 학습 땐 off로 비용 0) |
| `--wind-test` | v15 기체 바람 실측(무풍 hover에서 tilt: OFF 0.01°→ON 3.5°, atan(k·v²/mg) 검증) |
| `--drop-test` | v16 물리 payload가 탄도곡선대로 낙하하는지 공중궤적 검증 |
| `--zero-actions`/`--scripted`/`--step-response` | 기존 sanity |
| `--livestream 1` | WebRTC 스트림 → Isaac Sim WebRTC Streaming Client 접속(server IP, 49100/47998) |

## 마커 (visual, show_markers일 때만)
- **타겟**: 지면 성공존 원판(r1.0, 빨강) + **수직 비콘 기둥**(h6m, 마젠타) — 옆에서도 보이게
- **payload**: analytic(v11~v18)은 시각 마커(작은 원통) / **물리(v16/v19)는 마커 끄고** RigidObject 자체 표시
- **payload 하이라이트**(물리): 실물이 10cm로 너무 작아 → **큰 노란 구슬(r0.3, emissive)이 실물 추적** (물리 무변경)
- chase 카메라: `origin_type=asset_root, asset_name=robot`, eye (-4.5,-3.5,2.2) (멀다는 피드백 반영해 근접)

## 라이브스트림 접속
1. VM 켜기 → `js-v11-gui`(--network host, PUBLIC_IP=외부IP) 컨테이너
2. `play.py --task <V##> --policy <ckpt> --num_envs 1 --episodes 100000 --show --livestream 1`
3. **Isaac Sim WebRTC Streaming Client** → Server `136.113.193.83`, signaling 49100, media 47998
4. GCP 방화벽 `allow-isaac-sim` (49100·8211·47998) 개방됨

---

## 🐞 시행착오 (반복 조정)
| 증상 | 원인 | 해결 |
|------|------|------|
| 영상 안 옴 | `--livestream 2`(private) → publicEndpoint 미설정 | **`--livestream 1`(public)** + `PUBLIC_IP` env |
| 드론 너무 작게/멀리 | chase eye (-8,-8,4) 멂 | 근접 (-4.5,-3.5,2.2) |
| 격자만 보이고 드론 안 보임 | 기본 카메라가 지면 향함 | chase 카메라(드론 추적) |
| 노란 원기둥 2개 | 물리 payload + 시각 마커 **중복** | 물리 payload면 시각 마커 끔 |
| payload 분리 안 보임 | 실물 10cm **너무 작음** | **큰 노란 하이라이트 구슬** 추적 |
| 흰 바닥 눈부셔 안 보임 | 솔리드 흰색 + 이중 조명 | **격자 복귀** + 태양광 그림자 (솔리드 폐기) |
| 스트림에 옛 버그화면 | play.py **프로세스 2개** 49100 충돌 | 전부 kill 후 단일 재시작 |

## 교훈
- **payload 시각화 = 크기가 관건**(실물이 작으면 하이라이트 마커 필요).
- **바닥은 격자가 낫다**(솔리드는 참조 없어 오히려 안 보임). 그림자가 깊이 단서로 유용.
- analytic drop(v11~v18)은 **payload가 안 날아감**(release-terminal) → 물리 drop(v16/v19)이어야 낙하 시각화 가능.

## 업데이트 (2026-07-23) — GUI 합격 ✅
- **노란 하이라이트 구슬 제거** → 실물 payload(주황 원통) 그대로 표시(공으로 확대 금지).
- **카메라 payload 추적**(`viewer.asset_name="payload"`): 분리 후 낙하하는 payload를 지면까지 따라감 → 분리·명중이 또렷.
- **투하 안 보이던 원인 = 붕괴한 v19 체크포인트**(release 0)였음 → v18/정밀 best 모델로 교체하니 실제 명중 시청됨. (모델 문제였지 시각화 문제 아님)
- 정밀 best(v19_precise iter875, 착탄 0.356m)로 **타이트한 명중** 라이브 확인 완료.

## 남은 것
- 그림자/조명 미세조정(선택). 시각화는 사실상 완료.
- 관련: [[experiments/exp_016_v19_precision_landing_junsang]] · [[experiments/exp_015_v19_abd_retrain_junsang]] · [[experiments/exp_014_v19_full_integration_junsang]] · [[00_index]]
