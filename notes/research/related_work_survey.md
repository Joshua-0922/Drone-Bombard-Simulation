---
date: 2026-08-27
tags: [research, related-work, survey, positioning, novelty, paper]
status: open
type: research
---

# 선행연구 조사 — 우리는 어디에 서 있나

> **결론 먼저.** 2026-06-25에 **Scaramuzza 그룹(UZH RPG)이 거의 같은 문제를 발표했다**
> (arXiv 2606.27603, 두 달 전). 실기 검증, 비전 변형, 오픈소스까지 포함되어 있다.
> **정확도 경쟁으로도 민첩성 경쟁으로도 우리는 진다.**
>
> 그런데 그 논문이 **"future work"로 명시한 세 항목이 정확히 우리가 이미 구현한 것들이다** —
> 항력 모델링, 페이로드 공력 특성 랜덤화, 이동 중 투하.
> **경쟁이 아니라 후속으로 서면 자리가 있다.**

관련: [[experiments/exp_026_release_rate_100hz]] · [[research/agility_ceiling]] ·
[[research/research_architecture]] §10

---

## 1. 지형도 — 세 갈래

### 1.1 고전 정밀 공중투하 (CARP / 탄도풍)

우리 CCIP·T1·T2가 속한 계보. **군용 정밀투하에서 수십 년 된 성숙 분야**이고,
"바람을 추정해 투하점을 계산한다"는 아이디어는 여기서 이미 표준이다.

| 논문 | 핵심 |
|---|---|
| Yakimenko, *Autonomous Parachute-Based Precision Delivery Systems* (2016, 단행본) | CARP(Computed Air Release Point)와 **탄도풍(ballistic wind)** 개념의 정본 |
| Mathisen, Grindheim, **Johansen**, "Approach methods for autonomous precision aerial drop from a small UAV" (IFAC 2017) | 소형 UAV 투하의 접근 기동 + 바람 고려 릴리즈 상태 계산. **우리 T1/T2와 가장 가까운 고전 baseline** |
| Yang & Jeon, "Recursive path planning and **wind field estimation** for precision airdrop" (JGCD 2019) | 비행 중 바람장을 재귀 추정해 투하점 갱신. **"바람을 관측으로 줄 것인가" 논쟁의 근거 문헌** |
| Scheirey et al., "Low-Cost, High-Precision Two-Stage Airdrop Model" (AIAA 2026) | CARP/HARP + 2,000 ft 단위 탄도풍 테이블. 최신 |

> **함의:** "바람을 알면 잘 던진다"는 **새롭지 않다.** 우리 T3 오라클이 하는 일은
> 이 분야가 이미 하는 일이다. 새로울 수 있는 것은 **"바람을 모를 때 학습이 얼마나
> 회복하는가"** 쪽이다.

### 1.2 공중 투척 / 표적 릴리즈 (직접 경쟁 영역)

| 논문 | 방식 | 성능 |
|---|---|---|
| Foehn, Falanga, Kuppuswamy, Tedrake, **Scaramuzza**, RSS 2017 | 상보성 제약 궤적 최적화, 케이블 페이로드 스윙업 후 릴리즈 | 모델 기반 |
| Cao, Fang, Liang, T-ASE 2025 | 시간최적 투척 궤적 + 해석적 warm start | 모델 기반 |
| **AeroThrow** (Li, Chen, Lin, Ye, Lyu), **RA-L 2025** | 능동 관절(aerial manipulator) + NMPC + 계층적 외란 보상. **릴리즈 타이밍 민감도를 하드웨어로 낮춤** | 우리 **T2의 원본** |
| ⭐ **Zhai, Raimondi, Ren, Geles, Armati, Xing, Scaramuzza**, arXiv 2606.27603, **2026-06-25** | **RL 정책이 CTBR + 릴리즈를 end-to-end 출력**. Isaac Lab + Warp + PhysX 하이브리드 시뮬레이터 | **§2 참조** |

### 1.3 잔차 물리 / 던지기 학습 (방법론 계보)

| 논문 | 핵심 |
|---|---|
| **TossingBot** (Zeng, Song, Lee, Rodriguez, Funkhouser), 2019 | **"Residual Physics"의 원조.** 물리 시뮬레이터가 낸 제어 파라미터 위에 신경망이 **잔차**를 얹음. 85% 투척 정확도 |
| Ma, Liu, Qu, **Hutter**, "Learning Accurate Whole-body Throwing with **High-frequency Residual Policy**" (2025) | 공칭 추종 정책 + **고주파 잔차 정책** + 최적화 모듈. 6 m 표적에 평균 **0.28 m** |
| Liu, Da Costa, Billard, "Learning to Throw-Flip" (2025) | 물리 기반 자유비행 모델 + 회귀 학습으로 미모델링 효과 보정. **데이터 동화로 샘플 복잡도 40% 감소** |
| Liu, Nayak, Billard, T-ASE 2022 | 모바일 매니퓰레이터 투척, 평면 문제로 축약 |

> **함의:** **"해석 모델 + 학습 잔차"는 확립된 패러다임이다.** 우리 방법의 신규성은
> "잔차를 쓴다"가 아니라 **"잔차를 어디에 주입하는가"**(탄착 공간)에 있어야 한다.
> Ma/Hutter는 **고주파 잔차**를, TossingBot은 **제어 파라미터 잔차**를 쓴다.
> **탄착점 잔차는 아직 보지 못했다** — 여기가 유일하게 방어 가능한 방법론적 차별점이다.

---

## 2. ⭐ 직접 경쟁 논문 정밀 분석 — Scaramuzza 2026

**"Learning to Throw: Agile and Accurate Cable-Suspended Payload Delivery with a Quadrotor"**
(arXiv 2606.27603, 2026-06-25)

### 그들이 한 것

- 쿼드로터(0.21 kg, **T/W 6.8**) + 33 cm 케이블 + 테니스공. 서보 릴리즈 기구
- **하이브리드 시뮬레이터**: system-identified 해석적 쿼드로터 모델(Warp) ↔ PhysX 로프
  articulation(15 링크). **6차원 wrench 하나로만 결합.** 500 Hz
- PPO, **비대칭 actor-critic**, 50 Hz **CTBR** + 릴리즈 래치. obs 35차원
- 릴리즈 지연 **0.11 s를 실제로 구현** (우리 0.22 s와 같은 처방)
- **zero-shot 실기 배치**

### 그들의 수치

| 표적 | 착지 오차 | 투척 시간 |
|---|---|---|
| 1.5 m | **0.082 ± 0.035 m** | 1.090 s |
| 2.0 m | **0.105 ± 0.099 m** | 1.154 s |
| 2.5 m (OOD) | **0.133 ± 0.082 m** | 1.188 s |

MPC baseline 대비 착지 오차 **−50%**, 투척 시간 **−30%**. **민첩성-정확도 Pareto front
바깥**에 위치(실기 vs 시뮬 baseline인데도).
비전 정책(키포인트 기반)도 상태 기반과 **동등**(0.101 m).

### ⭐⭐ 그들이 "한계 / future work"로 명시한 것

> *"These results are obtained under three simplifying assumptions that also outline
> future work: targets are **static and on a fixed ground plane**, the post-release
> trajectory is predicted using a **drag-free ballistic model**, and the payload is
> **fixed in mass and aerodynamic properties**. Extending the target distribution to
> moving platforms, **randomizing payload properties with online adaptation**, and
> **modeling post-release aerodynamics** would broaden the approach to **delivery on
> the move**, varied cargo, and longer, more energetic throws."*

**세 항목 모두 우리가 이미 구현했다.**

| 그들의 future work | 우리 상태 |
|---|---|
| post-release aerodynamics 모델링 | ✅ `payload_drag_coef_nominal`, `integrate_payload_impact` |
| payload 공력 특성 랜덤화 | ✅ A그룹 `payload_bc_rel` (다만 실측 기여 6%) |
| delivery **on the move** | ✅ 순항 핸드오프 20 m 진입, 정지 없음 |
| moving platforms | ✅ CV/CT/CA 이동 타겟 포팅 완료 (exp_021) |
| 바람 | ❌ **그들 논문에 바람이 아예 없다** ← 우리만 있음 |

### 결정적 차이 — 우리는 다른 체제(regime)에 있다

| | Scaramuzza 2026 | 우리 |
|---|---|---|
| 표적 거리 | **1.5–2.5 m** | **18–22 m** |
| 전체 기동 시간 | **1.1 s** | **~7 s** |
| 릴리즈 고도 | ~1 m | 3–8 m |
| 낙하 시간 | ~0.45 s | **~1.0 s** |
| 외란 | 질량 ±5%, 케이블 길이 ±6% | **바람 N(0,1.5) + 탄도계수 ±20% + 릴리즈 지연 N(0.22,0.05)** |
| 탄도 모델 | **무항력** | 항력 포함 |
| 부착 | 케이블 서스펜션(스윙 활용) | 강체 마운트 |

**낙하 시간이 0.45 s면 항력이 실제로 무시 가능하다** — 그들이 무항력 모델을 쓸 수
있는 이유는 게으름이 아니라 체제가 다르기 때문이다. 1.0 s 낙하 + 바람에서는 무시할 수 없다.

⚠️ **심사자는 0.082 m와 0.378 m를 나란히 놓고 볼 것이다.** 정규화 없이는 진다.
반드시 **거리 대비 오차**(그들 5.3–5.5%, 우리 1.9%)나 **외란 조건 명시**와 함께 제시할 것.

---

## 3. 남아 있는 빈틈 — 실제로 새로운 것은 무엇인가

### 3.1 ✅ 방어 가능 — 탄착 공간 잔차 (주입 지점)

TossingBot은 **제어 파라미터**에, Ma/Hutter는 **추종 명령**에 잔차를 준다.
**예측된 탄착점 자체에 잔차를 주는 것은 조사 범위에서 발견하지 못했다.**

이 주입 지점의 성질:
- 해석 모델이 루프 안에 남는다 → 릴리즈 게이트·조준 목표가 계속 물리적 의미를 가짐
- 잔차의 단위가 **미터**다 → 크기를 직접 해석 가능("학습이 0.3 m를 보정했다")
- end-to-end 정책에서는 **불가능한 진단**이다

### 3.2 ✅✅ 가장 강한 자산 — 회복 가능성의 정량화

**이것이 실제로 아무도 안 한 것이다.**

문헌의 ablation은 전부 "구성요소 X를 빼면 오차가 2.4–3.7배" 형태다(Scaramuzza Table V).
이건 **sim-to-real 기여도** 측정이지 **회복 가능성** 측정이 아니다.

우리가 가진 것:

| DR_SCALE | 비특권(T2) | 오라클(T3) | **갭 = 원리상 회복 가능분** |
|---|---|---|---|
| 0.0 | 0.570 | 0.573 | −0.003 |
| 0.5 | 0.599 | 0.577 | 0.022 |
| 1.0 | 0.646 | 0.511 | 0.136 |
| 1.5 | 0.815 | 0.491 | 0.325 |

> **"모델 오차 크기의 함수로서, 착지 오차 중 학습이 원리상 회복 가능한 비율은 얼마인가"**
> — 이 질문에 답한 논문을 찾지 못했다. 오라클 상한선을 **구성적으로 정확하게**
> (플랜트와 동일한 적분으로) 정의한 것도 마찬가지다.

이건 **실기 없이도 성립하는 기여**다. 우리 조건에서 유일하게 그렇다.

### 3.3 ✅ 부분적 — 바람

Scaramuzza 논문에 바람이 없다. 고전 airdrop 문헌에는 있지만 **학습이 없다.**
"바람을 부분적으로만 추론 가능한 상태에서 학습하는" 조합은 비어 있다.
단 이건 **증분적**이지 단독 기여가 되기는 약하다.

### 3.4 ✅ 릴리즈 판정 주파수

exp_026의 T1≡T2 결과("AeroThrow 이점의 전부가 타이밍 해상도였다")는 **AeroThrow에 대한
직접적인 반증적 관찰**이다. AeroThrow는 하드웨어(능동 관절)로 타이밍 민감도를 낮췄는데,
우리는 **소프트웨어 판정 주파수만으로 같은 효과**를 얻는다. 작지만 명확한 발견.

### 3.5 ❌ 방어 불가

| 주장 | 왜 안 되나 |
|---|---|
| "RL이 모델 기반보다 정확하다" | Scaramuzza가 실기로 이미 함 |
| "민첩한 투하" | T/W 6.8 + CTBR 실기 vs 우리 T/W 2.0 시뮬. 상대가 안 됨 |
| "비전 기반 투하" | 그들이 키포인트 비전으로 동등 성능 이미 보임 |
| "Isaac Lab 시뮬레이터 기여" | 그들이 오픈소스 공개 예정 |

---

## 4. 수준 진단 — 솔직하게

### 현재 스코프(시뮬레이션 전용, Crazyflie 모델, 실기 없음)

| 목표 | 가능성 |
|---|---|
| RA-L / ICRA / IROS **정식 논문** | ⚠️ **실기 없이는 어렵다.** 이 분야(aerial manipulation)는 실기 검증이 사실상 필수이고, 직접 경쟁 논문이 두 달 전에 zero-shot 실기를 냈다 |
| IROS / ICRA **워크숍**, 국내 학술지, 석사 논문 | ✅ 충분 |
| RA-L **재도전** (§3.2를 주 기여로 재구성 시) | ⚠️ 가능성 있음. 단 "방법 논문"이 아니라 **"분석 논문"**으로 프레이밍해야 함 |

### 등급을 올리는 두 경로

**경로 A — 실기 (정공법, 비용 큼).** Crazyflie급이 아니라 T/W 3+ 기체 + 릴리즈 기구 +
모션캡처 또는 RTK. 바깥 바람에서 실험하면 **Scaramuzza가 못 한 조건**이 되어 차별화까지 됨.
다만 이건 몇 달짜리 하드웨어 작업이다.

**경로 B — 분석 논문으로 재프레이밍 (권장, 지금 자산으로 가능).**

> 제목 방향: *"How much of aerial delivery error can learning recover?
> A model-error sweep with a constructive oracle bound."*
>
> 주장: "우리 RL이 더 잘 던진다"(❌ 진다) 대신
> **"공중 투하 오차를 성분별로 분해하고, 각 성분이 학습으로 회복 가능한지를
> 모델 오차 크기의 함수로 정량화한다"**(✅ 아무도 안 함).
>
> 그 안에서 탄착 공간 잔차는 **회복을 실행하는 도구**이지 주장 자체가 아니다.
> 실기가 없어도 성립하는 이유: 오라클 갭은 **시뮬레이터 안에서 정의되는 양**이기 때문.

경로 B의 부수 이점: exp_025·exp_026이 **이미 그 논문의 그림 2개와 표 3개**다.

---

## 5. 즉시 해야 할 일

- [ ] **Scaramuzza 2606.27603을 관련연구로 정식 인용하고 체제 차이를 표로 명시** (§2 마지막 표)
- [ ] AeroThrow(T2)를 "RA-L 2025, 능동 관절+NMPC"로 정확히 인용 — 현재 코드 주석은 출처 불명확
- [ ] TossingBot / Ma-Hutter를 "잔차 주입 지점" 비교표로 인용 (§3.1)
- [ ] 고전 airdrop(Yakimenko / Mathisen-Johansen / Yang-Jeon)을 **T1/T2의 출처**로 인용 —
      현재 T1/T2가 "우리가 만든 baseline"처럼 보이는데 실은 **문헌 재현**이다. 그렇게 쓰는 편이 강하다
- [ ] **정규화 지표 결정** — 거리 대비 오차 또는 낙하시간 대비. 미정규화 비교는 반드시 진다
- [ ] 경로 A / B 결정
