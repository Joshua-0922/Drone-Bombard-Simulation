---
date: 2026-07-16
tags: [daily, log, isaac, v14, v15, v16, ccip-residual, wind, physical-drop]
type: daily
status: complete
---

# 연구 일지 — 2026-07-16

> DR+CCIP residual(v14) → 바람이 기체에 작용(v15) → **실제 물리 payload drop(v16)** 까지.
> 이번 주 학습 목표 달성 + 마무리 과제 1(물리 drop) 완료.

---

## 오늘 한 일

- **v14 (DR + CCIP residual, Stage A)** 구현·검증: wind/drag DR + residual action[5:7]±3m + 바람 관측(obs 27D) + 대조군(`--v14_no_residual`). residual ON vs OFF 대조 dry-run.
- **v15 (바람이 드론 기체에 작용)** 구현·실측검증: 상대기류 2차 항력을 컨트롤러에 추가. `play.py --wind-test`로 OFF 0.01°→ON 3.5° 확인. wind 4.0 dry-run 정체 → residual 포화 진단 → wind 2.0 튜닝.
- **v16 (실제 물리 payload drop)** 구현·검증 (마무리 과제 1): RigidObject 운반→투하→중력+drag 낙하→착지 기반 학습. `--drop-test` PASS, dry-run success 0.80·실제 착탄 sub-meter.
- **백업 태그** `week1-v15-analytic` 생성(analytic 라인 복귀점).
- 문서 전면 업데이트(exp_009/010/011 + 허브 + Rule 11 + roadmap).

---

## 주요 결정 & 발견

- **v14 residual 격차가 작았던 이유** = 바람이 payload 탄도에만 작용, 드론은 무풍 → 대조군도 "비행 보정"으로 커버. → v15로 바람을 기체에 작용시킴.
- **Rule 11 (residual 포화)**: DR 세기를 키우면 드리프트가 residual_scale(±3m)을 초과해 포화 → 착탄 정체. wind 4.0(드리프트 3.7~7.7m)에서 관측 → wind 2.0으로 하향. → [[research/rl_rules]]
- **v16 drag 보정**: `payload_phys_drag_k` 0.02→0.005 (전면적 기준). 0.02는 종단속도 7m/s로 과대 → drop-test FAIL. 0.005로 PASS.
- **물리 drop 설계**: release-terminal 폐기 → **payload 착지가 종료**, 투하 후 드론 호버(옵션 A), 실패조건 gate off. analytic은 obs 힌트로 유지("예측→실측").

---

## 코드 변경 사항

| 파일 | 변경 |
|------|------|
| `drone_bombard_env.py` | `wind_force_enabled` 훅(v15 기체 바람), `payload_physics_enabled` 훅(v16 RigidObject·carry·drag·착지) |
| `v11_env.py` | V14Cfg/Env(DR+residual), V15Cfg(기체 바람+wind 2.0 튜닝), V16Cfg/Env(land-terminal·실제착탄 보상) |
| `__init__.py`, `train.py` | V14/V15/V16 등록·`--v14/--v15/--v16`·`--v14_no_residual` |
| `play.py` | `--wind-test`(기체 바람 실측), `--drop-test`(물리 payload 검증) |

> 커밋: v14/v15/wind튜닝/v16/drag보정 다수, 전부 `origin/Issac_JS` push. 태그 `week1-v15-analytic`.

---

## 문제 & 해결

| 문제 | 해결 | 메모 |
|------|------|------|
| v14 residual 효과 작음 | ✅(해석) | 바람이 payload에만 작용 → v15로 기체 작용 |
| v15 wind 4.0 착탄 ~3m 정체 | ✅ | residual 포화(드리프트>±3m) → wind 2.0 하향 (Rule 11) |
| v16 drop-test FAIL(\|dz\|0.55m) | ✅ | payload drag 과대(0.02) → 0.005로, PASS |
| VM 잦은 stockout | ✅ | 요청마다 1회 재시도(최대 13회 만에 확보) |

---

## 내일 할 일

- [ ] **과제 2 (시각/검증 도구 개선)** — 카메라 근접·marker 가시화·바닥 개선·play.py 체계화
- [ ] v16 낙하를 **영상으로 직접 확인**(과제 2 후)
- [ ] (선택) v15 dry-run(wind 2.0) 실행 / 다중 시드

---

## 관련 노트

- [[experiments/exp_009_v14_ccip_residual_junsang]] · [[experiments/exp_010_v15_airframe_wind_junsang]] · [[experiments/exp_011_v16_physical_drop_junsang]]
- [[research/isaac_expansion_roadmap_junsang]] · [[research/rl_rules]] · [[daily/daily_2026-07-15_junsang]] · [[00_index]]
