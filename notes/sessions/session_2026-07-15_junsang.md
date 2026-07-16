---
date: 2026-07-15
tags: [session, isaac, v11, v12, v13, migration]
type: session
status: complete
---

# 세션 로그 — 2026-07-15

> Isaac 트랙 첫 학습 세션. 완화 baseline(v11) → 랜덤 marker(v12) → 부분관측(v13)까지
> 한 축씩 확장·검증. 상세는 [[daily/daily_2026-07-15_junsang]].

---

## 세션 흐름

1. **v11 완화 모델 설계·구현** (단일 phase · 고정 marker · 정책 drop_signal · vision 제거) → 격리 브랜치 `Issac_JS` + VM 격리 환경(wt-js 복사본 + 전용 컨테이너).
2. **v11 검증**: smoke → dry-run 512env → **success 100%, 착탄 0.43m**. (버그: cruise 핸드오프 step-1 즉사 → 컨트롤러 seed 수정, Rule 10.)
3. **결정론적 검증** `play.py --policy` → success 100%, 착탄 14cm.
4. **라이브 GUI** — WebRTC 라이브스트림(livestream 1 + PUBLIC_IP) + chase 카메라로 비행 실시간 확인.
5. **v12 (랜덤 marker)** 구현·검증 → **success 100%, 착탄 ~0.72m** (일반화 확인).
6. **v13 (부분관측: blind→7m reveal)** 구현·검증 → **success 100%, 착탄 ~0.8m**.
7. **문서화**: exp_006/007/008 + 허브 3곳 + daily + session + 비교 가이드.

---

## 이 세션 산출물

- **코드** (`origin/Issac_JS` push 완료, HEAD `795be85`): v11/v12/v13 env·cfg·태스크·train 플래그·play chase 카메라·release_gate.
- **실험 노트**: [[experiments/exp_006_v11_dryrun_junsang]] · [[experiments/exp_007_v12_random_marker_junsang]] · [[experiments/exp_008_v13_partial_obs_junsang]]
- **규칙/발견**: [[research/isaac_cruise_handoff_junsang]] (Rule 10), [[research/isaac_v11_v13_design_guide_junsang]] (기존 migration 대비 변경점 가이드)
- **체크포인트**: `model_final*.pt` — VM 디스크(js-v11 컨테이너)에만 보존.

## 다음 세션 시작점

- 확장 3축(완화→랜덤→부분관측) 전부 100% 검증됨.
- 다음 후보: 난이도↑(reveal 5m) / **진짜 vision 복원**(핀홀 카메라) / wind·DR+residual / 이동 타겟.
- (선택) `Main-notes` 실험 노트 git 커밋/push — 현재 파일 저장만 됨.

---

## 관련 노트

- [[daily/daily_2026-07-15_junsang]] · [[experiments/training_history]] · [[research/rl_rules]] · [[00_index]]
