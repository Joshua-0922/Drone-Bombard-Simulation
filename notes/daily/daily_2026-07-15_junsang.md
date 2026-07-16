---
date: 2026-07-15
tags: [daily, log, isaac, v11, v12, v13, expansion]
type: daily
status: complete
---

# 연구 일지 — 2026-07-15

> Gazebo→Isaac 마이그레이션 첫 학습. 완화 baseline(v11)부터 랜덤 marker(v12)·부분관측(v13)까지
> 한 축씩 확장하며 전부 100% success 달성. + 라이브 GUI 시청, 디스크 진단.

---

## 오늘 한 일

- **Isaac v11 완화 테스트 모델 설계·구현** — 단일 통합 phase, 고정 marker(정면 20m), 정책 drop_signal + release envelope, vision 제거, DR/residual/이동타겟은 inert hook으로 보존(삭제 없이 토글).
- **v11 검증** (VM L4): smoke(16env) → dry-run(512env·300iter) → ~iter88 수렴 **success 100%·착탄 0.43m**.
- **v12 첫 확장(랜덤 marker)** — (20,0) 중심 5m 원 면적균일 랜덤. dry-run ~iter49 수렴 **success 100%·착탄 ~0.72m**. 일반화 확인.
- **v13 부분관측** — blind +X 순항 → 수평 7m 진입 시 marker 공개(연속·비latch) + 미탐지 페널티 + 보상 게이팅. dry-run ~iter48 수렴 **success 100%·착탄 ~0.8m**.
- **결정론적 검증** — `play.py --policy model_final.pt` 20 에피소드 → success 100%, 착탄 **14cm**(탐험 노이즈 없어 학습곡선보다 정밀).
- **라이브 GUI 시청** — Isaac WebRTC 라이브스트림(livestream 1) + chase 카메라로 드론 비행 실시간 확인.
- **tensorboard 곡선 회수·시각화**, **노트북 디스크 full 진단**, **실험 기록 exp_006/007/008 + 허브 3곳 갱신**.

---

## 주요 결정 & 발견

- **cruise 핸드오프 = 컨트롤러 seed 필요** (신규 규칙): 움직이는 상태로 spawn 시 reset에서 `_v_filt`/`_prev_action`을 cruise 속도로 seed 안 하면 첫 스텝 자세 폭주(bad_attitude) 즉사. → `[[research/rl_rules]]` Rule 10, `[[research/isaac_cruise_handoff_junsang]]`.
- **확장은 "삭제 없이 토글"** — `marker_random`, `reveal_radius` 등 cfg 플래그로 v11 무손상 확장(v12/v13는 cfg/서브클래스만).
- **랜덤 스폰 방식**: 부채꼴 아님. (20,0) 중심 **반경 5m 원** 면적균일(`r=R√U`). v13 reveal 반경 7m = disk 5m + 여유 2m.
- **v13 부분관측 핵심**: 미탐지 시 marker 의존 보상 전부 0(게이팅) → **보상이 숨은 위치를 누출하지 않게** 해야 "진짜 blind". 미탐지 페널티로 재진입 강제.
- **현재 vision 구조 파악**: down-camera **핀홀 픽셀 투영**(u,v,conf), footprint ≈ 고도×0.58(측면). 복원 시 "안 보이는 타겟 접근" 과제로 급증 → 마지막 확장 축.
- **VM 사용 방식**: 공용 isaac-worktree 무손상 + 우리 소유 `wt-js` 복사본 + 전용 컨테이너(js-v11 / js-v11-gui, `--user root`). 실험 후 VM 정지로 과금 중단.

---

## 코드 변경 사항

| 파일 | 변경 내용 |
|------|----------|
| `drone_bombard/v11_env.py` | V11(완화)·V12(marker_random 토글)·V13(부분관측 env: obs 마스킹+detected 플래그, 보상 게이팅+미탐지 페널티) |
| `drone_bombard/math_utils.py` | `release_gate` (release envelope 판정) + 유닛테스트 |
| `drone_bombard/__init__.py` | 태스크 V11/V12/V13 등록 |
| `train.py` | `--v11_test` / `--v12` / `--v13` 플래그·분기 |
| `play.py` | chase 카메라(viewer origin_type=asset_root) |

> 커밋: `af8cfad`(v11) · `38c66c2`(v11 cruise-seed 수정) · `5a4f85d`(v12) · `795be85`(v13). 전부 `origin/Issac_JS` push 완료.

---

## 문제 & 해결

| 문제 | 해결 여부 | 메모 |
|------|----------|------|
| v11 첫 스텝 `bad_attitude` 즉사 (episode length 1) | ✅ | reset에서 컨트롤러를 cruise setpoint로 seed (Rule 10) |
| VM 잦은 stockout | ✅ | 요청 시마다 1회 재시도 → 여러 번 만에 확보 (정상 패턴) |
| 노트북 디스크 100% full | ✅ | Isaac 학습 아님 — Docker(`rac2026_px4_sim` 82GB 컨테이너 + 빌드캐시 45GB)가 범인. 사용자가 정리(70G 확보) |
| WebRTC 영상 안 옴 | ✅ | livestream 2(private)→**1(public)** + `PUBLIC_IP` env |
| 스트림에 드론 안 보임(격자만) | ✅ | 기본 카메라가 지면 향함 → **chase 카메라**(드론 추적) |
| payload 안 보임 / 지면 bounce 우려 | ✅ | analytic payload(물리 없음) + release-terminal 리셋 텔레포트 — 둘 다 정상 |
| notes 쓰기 ENOSPC | ✅ | 디스크 확보 후 exp_006~008 작성 |

---

## 내일 할 일

- [ ] 다음 확장 축 선택: (a) 난이도↑(reveal 5m·페널티 튜닝) / (b) **진짜 vision**(핀홀 카메라 u,v,conf) / (c) wind·DR+residual / (d) 이동 타겟
- [ ] (원하면) 실험 노트 `Main-notes` git 커밋/push
- [ ] 필요 시 v13 tensorboard 전체 곡선 회수·시각화

---

## 관련 노트

- [[experiments/exp_006_v11_dryrun_junsang]] — v11 완화 dry-run
- [[experiments/exp_007_v12_random_marker_junsang]] — v12 랜덤 marker
- [[experiments/exp_008_v13_partial_obs_junsang]] — v13 부분관측
- [[research/isaac_cruise_handoff_junsang]] — cruise 핸드오프 seed (Rule 10)
- [[experiments/training_history]] · [[research/rl_rules]] · [[00_index]]
