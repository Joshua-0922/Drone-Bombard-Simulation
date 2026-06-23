---
date: 2026-06-23
tags: [experiment, eval, soft-reset, v14, regression, final-approach, video]
status: done
wandb_run: byxyaf4d (rl_yolo_v14_softreset, stopped @196.5K/500K ~39%)
type: experiments
---

# Exp 010 — v14 (byxyaf4d) 195K 체크포인트 평가: 65% + soft reset 장기검증

> **요약:** soft-reset full run을 plateau 확인 후 SIGINT stop(체크포인트 보존), 195K 체크포인트로 clean 20-ep
> deterministic eval. **성공률 65% (13/20)** — v13의 80%(16/20) 대비 **회귀**. 단 회귀는 정책 미성숙(39% budget)
> 탓이고, **soft reset 메커니즘 자체는 장기 검증 통과**(3096 resets, EKF bounded).
> 관련: [[experiments/exp_009_softreset_throughput]] · [[research/reset_throughput_bottleneck]] Rule 14 · [[experiments/exp_007_iyhfy5ps_v13_eval]]

---

## 학습 종료 상황 (byxyaf4d)

- **plateau 확인 후 stop.** ep_rew_mean: ~6 → (18K dip −22, 탐색) → 70K에서 ~120 도달 → **70K~196K 내내 ~120–135 평탄**(125K step 동안 학습 정체). ep_len_mean 150 → ~40–50(목표 도달 빨라짐).
- **196.5K/500K (~39%)에서 SIGINT graceful stop.** 체크포인트 `sac_drop_195000_steps.zip` 디스크 보존(손실 0). 리셋 free 위해 sim 점유 해제(단일 gz/px4 인스턴스).

## Soft reset 장기 검증 (Rule 14 미해결 질문 해소) ✅

stop 직전 env 통계: **attempts=3096 success=2826 skipped=118 (soft 성공 ~91%, no teleport / no PX4 restart).**
- exp_009의 32회(100%)가 학습 정책에서도 유지되는지가 미해결이었음 → **수천 회에서 ~91% soft 성공**, fallback ~9%만 teleport. exp_009 한계 #1(학습된 공격적 정책이 fallback↑) = **bounded 확인**.
- EKF drift: 학습 내내 health gate(10m) 안쪽 유지, 발산 루프 없음. 한계 #2(d_xy 누적) = **장기 run에서 bounded 확인.**
- **→ soft reset은 production-ready. Rule 14 검증 완료.**

## Eval 결과 (20-ep, deterministic, `sac_drop_195000_steps.zip`)

| 지표 | 값 |
|------|-----|
| **성공률** | **0.650 (13/20)** |
| outcome | success 13 / **stagnation 7** / EKF-drift·gate fail **0** |
| mean steps-to-success | 260.9 (median 247) |
| mean closest d_xy | 0.584 m |
| best closest d_xy | 0.478 m |
| mean final d_xy | 0.670 m |
| 성공 반경(gate) | ≤ 0.50 m |
| mean ep reward | −491 (±201) |

- **실패 모드 100% 균일 = final-approach stagnation.** 7 실패 전부 ~0.5–0.8 m까지 접근 후 0.50 m gate 직전 정체(실패 min: 0.52 / 0.63 / 0.70 / 0.79 / 0.79 / 1.09 / 1.19). 항법·탐지 실패 아님 — **종말 거리 좁히기(terminal tightening) 약점.** ep 18은 2 cm 차(min 0.52).
- **EKF 건강:** eval 중 health-gate EKF↔camera divergence 2회(ep4 전, ep19 전) → 둘 다 full infra restart + 8s settle로 **self-heal, 직후 에피소드 성공.** 정책 귀책 EKF 실패 0.

## v13 대비 회귀 분석

| | v13 (iyhfy5ps) | v14 (byxyaf4d) |
|--|----------------|----------------|
| 20-ep clean eval | **80% (16/20)** | **65% (13/20)** |
| 체크포인트 | ~157.7K (~32%) | 195K (~39%) |
| reward env | 동일 terminal | 동일 terminal |
| 변경점 | — | soft reset + 10m + 탐지게이트 |

- v14 정책 변경은 **리셋 처리량(soft reset) + 핸드오프 윈도우(10m·탐지게이트)** 뿐 — 보상 미변경이라 정책이 *나빠질* 이유는 구조적으로 없음.
- **가장 유력한 원인: 미성숙.** 39% budget에서 stop, terminal-tightening은 마지막에 sharpen되는 스킬 — 70K 이후 reward plateau는 *거친* 정책의 평탄이고 종말 정밀도는 아직 안 올라옴. 추가 가설: 10m 고도(v13은 5m)에서 최종 하강 거리가 길어져 종말 정밀 요구↑.

## 비디오 산출물 (3/3 success 캡처)

`record_flight.py`(240s) + 3-ep evaluate 동시 구동 → **3 에피소드 전부 success(0.48–0.50 m)**.
- `ros2_ws/rl_eval_results/v14_195k_flight_annotated.mp4` (2.1 MB, YOLO 박스+상태 오버레이)
- `ros2_ws/rl_eval_results/v14_195k_flight_raw.mp4` (3.7 MB, raw onboard)
- 640×480, ftyp+moov 검증(재생 가능). headless `gz sim -s`라 3rd-person GUI 그랩 불가 → onboard annotated가 표준 산출물.

## 다음 / 결정 대기
1. **v14 코드 commit 여부 = 사용자 결정.** soft reset/탐지게이트는 검증 완료(인프라 개선)지만 65%<80%라 "v14를 validated baseline으로" 채택은 미정.
2. **회귀 해소안:** ① soft reset 켠 채 500K까지 재개/연장(미성숙 가설 검증) ② 10m→5m 고도 환원 A/B(종말 거리 가설) ③ 성공 반경 0.50 m 유지 시 종말 보상 shaping 강화 검토.

→ 규칙: [[research/rl_rules]] Rule 14(검증완료) / [[research/reset_throughput_bottleneck]]
