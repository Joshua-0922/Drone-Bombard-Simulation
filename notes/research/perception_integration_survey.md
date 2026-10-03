---
date: 2026-10-03
tags: [research, related-work, perception, yolo, sim2real, vision]
status: active
type: research
---

# 선행연구 조사 — 표적 위치를 참값에서 검출기(YOLO)로 바꿀 때, 학습을 다시 하는가

> **질문.** 투하·투척 드론 연구에서 (1) 처음에는 표적 위치를 참값으로 주고 학습한 뒤, (2) 검출기로 표적을 찾는 단계에서
> 정책을 **처음부터 다시 학습**하는가, 아니면 **학습된 정책에 검출 결과를 넣어 시험에서만** 쓰는가? 그리고 그것을 **시뮬레이션에서도** 하는가?
>
> **답(요약).** 조사한 연구 중 실제 검출기를 학습 루프에 넣은 것은 없다. 공통 구조는
> **"정책은 참값(또는 참값을 투영한 값 + 잡음)으로 학습 → 검출기는 배포 단계에서 그 입력 자리를 채움"** 이다.
> 차이는 검출 오차를 학습에 어떻게 반영하느냐뿐이다: 무시(AeroThrow), 단순 잡음(Learning to Throw), 실측 오차 모델로 **미세조정**(Swift).

관련: [[research/related_work_survey]] · [[research/paper_outline_v6]] §6 · [[research/sim2real_gap]] · [[00_index]]

---

## 1. 논문별 정리

| 논문 | 표적 위치를 학습에서 어떻게 주나 | "비전" 단계에서 정책은 | 실제 검출기 | 시뮬레이션 평가 | 실기 |
|---|---|---|---|---|---|
| **Learning to Throw** (Zhai … Scaramuzza, arXiv 2606.27603, 2026) | 상태 정책: 표적-화물 오프셋 **참값** | 같은 PPO 파이프라인으로 **처음부터 다시 학습**한 변형. 관측 = 표적 주변 0.25 m 사각형 꼭짓점 + 화물 키포인트를 카메라로 투영한 정규화 좌표 | **없음.** 실기에서도 모션캡처 자세를 가상 카메라로 투영한 HIL 키포인트 사용: *"allowing us to focus on testing the control performance without the need for an object detection module"* | 함 | 함(zero-shot). 2 m에서 상태 0.105 m vs 비전 0.101 m |
| **AeroThrow** (Li … Lyu, RA-L 2025, arXiv 2507.13903) | 표적 좌표 **주어진 값**(인지 없음) | 해당 없음 (학습 정책이 아님, NMPC) | 없음. 상태는 모션캡처 + IMU EKF | 함(상세 없음) | 함 |
| **Swift** (Kaufmann … Scaramuzza, Nature 2023) | 다음 게이트의 상대 자세(꼭짓점 12차원) **참값**으로 학습. 검출기는 학습 중 돌리지 않음 | 실기 50초 비행으로 **인지(오도메트리) 오차를 가우시안 과정으로, 동역학 오차를 k-NN으로 적합** → 그 오차를 시뮬레이션에 넣고 **정책을 미세조정** | **있음**(CNN 게이트 검출기 + VIO), 배포 시에만 | 함 | 함 |
| **Bootstrapping RL with Imitation** (Xing … Scaramuzza, CoRL 2024) | 교사 정책: 특권 상태 | 교사 → 학생(시각 입력) **증류** 후 RL 미세조정 | 시각 특징(학습 표현) | 함 | 함 |
| **TossingBot** (Zeng et al., RSS 2019 / T-RO 2020) | — | 이미지에서 파지·투척 잔차를 실기에서 직접 학습 | 이미지 end-to-end | 일부 | 실기 학습 |

## 2. 공통 패턴 — 세 가지 방식

| 방식 | 학습 | 배포 | 대표 | 비용 |
|---|---|---|---|---|
| **A. 그대로 꽂기** | 참값 | 검출기가 같은 입력 자리를 채움, 재학습 없음 | 대부분의 모듈형 시스템 | 0. 검출 오차에 대한 강건성은 보장 안 됨 |
| **B. 잡음 모델로 학습/미세조정** | 참값 + 검출기 오차 모델(가우시안, 양자화, 탈락, 지연) | 실제 검출기 | Learning to Throw(단순 잡음), **Swift(실측 오차 모델로 미세조정)** | 오차 모델 측정 + 미세조정 1회 |
| **C. 다른 관측으로 처음부터 / 증류** | 투영 키포인트·이미지 | 같은 관측 | Learning to Throw 비전 변형, Xing 2024 | 정책 전체 재학습 |

어느 연구도 **렌더링 + 실제 YOLO 추론을 학습 루프 안에서** 돌리지 않는다(병렬 2048 환경에서 비용이 맞지 않음). 실제 검출기는 **배포(실기) 또는 소수 환경 평가**에서만 쓰인다.

## 3. 우리 프로젝트에 대한 함의

- **학습은 참값 유지가 표준이다.** 주 결과(Table 1~5)를 참값 표적으로 낸 설계는 선행연구와 같은 구조이고, 정당화가 따로 필요 없다. Learning to Throw조차 실기에서 검출기를 쓰지 않았다.
- **"YOLO 절"의 표준 구성은 B 방식이다.**
  1. 시뮬레이션 렌더 + 실제 YOLO로 검출 오차를 **거리·각도별로 측정**(우리 `yolo_eval.py --calibrate`가 원래 이 목적).
  2. 그 오차 모델(편향, 분산, 탈락률, 지연)을 학습 환경의 표적 관측에 주입.
  3. 기존 L0를 **재학습 없이** 평가(A) → 손실이 크면 오차 모델 위에서 **미세조정**(Swift 방식) → 잔차 GRU도 그 비행으로 재적합.
  4. 소수 환경(≤8)에서 **YOLO를 루프에 넣은 닫힌 루프 평가**로 오차 모델이 맞는지 검증.
- **우리가 더할 수 있는 것.** 실제 검출기를 루프에 넣은 결과는 Learning to Throw도 future work로 남겼다. 시뮬레이션에서라도 YOLO-in-the-loop 수치를 내면 그 빈칸을 채운다.
- **코드 현황(확인 필요 사항).** `PerceptionCfg.pixel_quantize`(픽셀 양자화 잡음, 기본 OFF)는 B 방식의 가장 단순한 형태로 이미 있다. `yolo_eval.py`는 **구 환경 `Isaac-DroneBombard-Direct-v0`(관측 14)** 기준이라 현행 과제 환경(`Task-v0`, 관측 26)에 맞게 옮겨야 하고, numpy 2.x 오염 이력(cv2 빌드) 때문에 실행 전 환경 점검이 필요하다([[research/isaac_lab_architecture]]).

## 출처
- Learning to Throw: https://arxiv.org/abs/2606.27603 (HTML v1 §IV-C, Table VI)
- AeroThrow: https://arxiv.org/abs/2507.13903 (§V)
- Swift: https://www.nature.com/articles/s41586-023-06419-4 (Methods: residual models, fine-tuning)
- Bootstrapping RL with Imitation: https://arxiv.org/abs/2403.12203
