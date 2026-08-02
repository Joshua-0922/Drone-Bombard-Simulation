---
date: 2026-07-16
tags: [session, isaac, v14, v15, v16, physical-drop]
type: session
status: complete
---

# 세션 로그 — 2026-07-16

> DR+residual(v14) → 기체 바람(v15) → 실제 물리 drop(v16). 이번 주 목표 달성 + 마무리 과제 1 완료.
> 상세는 [[daily/daily_2026-07-16_junsang]].

---

## 세션 흐름

1. **v14** DR+CCIP residual(Stage A, 바람 관측) 구현 → residual on/off 대조 dry-run → residual 우세하나 격차 작음(바람이 payload에만 작용).
2. **v15** 바람을 드론 기체에 작용(2차 항력) → `--wind-test`로 실측 검증(0.01°→3.5°) → wind 4.0 정체 원인=residual 포화(Rule 11) → wind 2.0 튜닝.
3. **v16** 실제 물리 payload drop(RigidObject) — 마무리 과제 1 → `--drop-test` PASS → dry-run success 0.80·실제 착탄 sub-meter.
4. **백업** 태그 `week1-v15-analytic` + **문서 전면 업데이트**.

## 이 세션 산출물

- **코드** (`origin/Issac_JS`): v14/v15/v16 env·cfg·train 플래그, `--wind-test`/`--drop-test` 검증도구. 태그 `week1-v15-analytic`.
- **실험 노트**: [[experiments/exp_009_v14_ccip_residual_junsang]] · [[experiments/exp_010_v15_airframe_wind_junsang]] · [[experiments/exp_011_v16_physical_drop_junsang]]
- **규칙**: [[research/rl_rules]] Rule 11 (residual 포화).

## 다음 세션 시작점

- **이번 주 마무리 과제**: 과제1(물리 drop) ✅ 완료 / **과제2(시각·검증 도구) 미착수**.
- v16 낙하 영상 확인은 과제 2 후.
- 밀린 것: v15 dry-run(wind 2.0), 다중 시드.

---

## 관련 노트

- [[daily/daily_2026-07-16_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[experiments/training_history]] · [[00_index]]
