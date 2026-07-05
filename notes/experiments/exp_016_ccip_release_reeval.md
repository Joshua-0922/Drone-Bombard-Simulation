---
date: 2026-07-05
tags: [experiment, isaac-lab, ccip, release, metrics, eval]
status: done
type: experiment
wandb_run: N/A (eval-only; A2 체크포인트 = v3qk07pg model_final.pt)
---

# exp_016 — CCIP 릴리스 referee 수정 + A2 200-ep 재평가

> exp_014 A2의 "success 100% vs drop_impact_error 4.59 m" 디커플링 규명·수정.
> 원인 분석: [[research/ccip_release_decoupling]]. **결론: 릴리스 트리거가
> 존재하지 않았고(위치 기반 성공 종단에서 지표 측정), 4.59 m는 투하 오차가
> 아니라 종단 잔여속도의 탄도 캐리였다.**

관련: [[experiments/exp_014_A2_visionrange]] · [[experiments/exp_015_phased_curriculum]] · [[research/rl_rules]] Rule 21 · [[experiments/training_history]]

## 1. 설정

- **정책**: exp_014 A2 `model_final.pt` (4-dim, 커밋 `6407f8d` 시대) — HEAD는
  6-dim(exp_015)이라 로드 불가 → `git archive 6407f8d`로 eval 전용 트리 구성,
  referee 지표만 백포트(컨테이너 로컬 `/workspace/eval014`, 워킹트리 무오염).
- **referee**: 매 policy step(10 Hz) + 물리 스텝(100 Hz) 쌍둥이 버퍼,
  $|\hat{p}_{impact} - p_{target}| \le 0.2$ m ∧ alt > 1 m 최초 충족 시 래치.
  **rollout 동역학 bit-identical**(액션/보상/종단 무접촉) — 지표 버퍼만 추가.
- 명령: `play.py --policy ... --num_envs 4 --episodes 200 --headless`
  (`isaac-verify`, L4).

## 2. 결과 (200 ep)

| 지표 | exp_014 보고 | 본 재평가 |
|---|---|---|
| success | 100.0% (202/202) | **100.0% (200/200)** |
| d_xy_min | 0.665 m | 0.660 m |
| drop_impact_error (구=terminal) | 4.59 m | **4.649 m (재현)** |
| **drop_impact_error (신=@release)** | — | **0.137 m (10 Hz) / 0.172 m (100 Hz)** |
| **release_rate @0.2 m** | — | **6.0% (10 Hz) / 11.5% (100 Hz)** |
| aim_err_min (CCIP 최근접) | — | med 0.776 (10 Hz) / 0.755 m (100 Hz) |
| final_speed_xy @종단 | — | 2.989 m/s |

산수 봉합: $2.99 \times (\sqrt{2 \cdot 10/9.81} + 0.1) = 4.57$ m ≈ 4.65 m 관측
(±d_xy 슬랙). 디커플링 완전 설명.

## 3. 판정

1. **버그였나?** — 예(지표 의미론 버그). 릴리스 설계 의도(v15
   `drop_calculator_node`, tol 0.2 m)는 CCIP 오차 트리거인데, Phase-1 이식에서
   트리거가 사라지고 지표만 "성공 종단 스냅샷"에서 측정됨.
   `DropCfg.release_tolerance`는 정의만 되고 미사용이었다.
2. **수정 후 수치** — 릴리스 발화 시 0.137 m(≤0.2 by construction). 단
   **발화율 6%**: 접근 정책의 CCIP 스윕 최근접(~0.75 m) ≈ 경로 cross-track
   오차(d_xy_min 0.665 m)라 0.2 m 윈도우를 거의 통과하지 못함. **exp_014
   정책의 "실질 투하 하한"은 ~0.75 m(중앙값)이며, 0.2 m 달성은 릴리스 조건부
   보상(exp_015 Phase 2)의 몫.**
3. **exp_013 비교** — 동일 지표 의미론(exp_012 커밋 `80a5cd9` 도입, exp_013
   커밋 `8a0a096` 동일). 24 m는 실패 지배(max_alt 33%가 25 m 천장·crash 27%·
   overspeed ~20 m/s 종단의 거대 캐리). **디커플링은 지표 도입부터 구조적
   내재 — plant 수정이 만든 것이 아니라 100% success가 노출시킨 것.**
   (Gazebo 시대 [[research/eval_terminal_env_metrics]]도 "위치 성공은 투하
   모델 없음"을 이미 문서화.)
4. **action_sat_frac 0.46 연관(#3)** — 릴리스 자체가 없으므로 "릴리스 순간
   saturation" 질문은 성립 불가로 해소. latency 갭도 0(해석적 릴리스,
   mechanism delay 0.1 s는 예측식 내부 반영).

## 4. 코드 반영 (HEAD, feat/isaac-env-migration)

- `drone_bombard_env.py`: Phase-1 스크립티드 CCIP referee
  (`_evaluate_scripted_release_metric`, 지표 전용) + `_aim_err_min` 전 페이즈
  추적 + 로깅 재정의(`drop_impact_error_m`=릴리스 시점,
  `drop_impact_error_terminal_m`=구 지표 보존, `release_rate`,
  `aim_err_min_m`, `final_speed_xy`).
- `play.py`: per-episode 분포 통계(스냅샷 누적, mean/med/p90/max).
- 보상·종단·액션 경로 불변 — Phase-1 학습 동역학 bit-identical,
  Phase 2/3 로직 무접촉(`aim_err_min` 추적만 추가).

## 5. 후속

- [ ] exp_015 Phase 1 재학습 시 `aim_err_min` 곡선 관찰(릴리스-가능성 수렴 신호)
- [ ] Phase 2 dry-run에서 tol 0.5 m·10 Hz 윈도우-스킵 여부 `aim_err_min`으로 감시
- [ ] 실 페이로드 바디 도입 시 full-vz 공식 업그레이드 (문서화됨, 범위 밖)
