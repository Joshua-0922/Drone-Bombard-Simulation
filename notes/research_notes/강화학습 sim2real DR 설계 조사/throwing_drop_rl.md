# Learning-based throwing, aerial payload release, and aerial manipulation with sim-to-real (2015–2026): per-paper fact sheet

Scope note: about 25 tool calls. Primary arXiv/ar5iv full text was read for: Learning to Throw (Zhai 2026), AeroThrow (Li 2025), TossingBot, MC-PILOT (Turcato 2025), HEAP excavator throwing (Werner 2024), Kasaei ICRA 2023, FLARE (Cao 2025/26), and Ghadirzadeh DPPT 2017. Searches for RL fixed-wing airdrop, RL precision airdrop and RL parachute release found **no paper with real-world transfer**. They are listed as gaps.

## Q1. Per-paper fact sheet (task, platform, release DR, object DR, wind, sysID, reward, residual, real accuracy)

### Takeaway
Only one aerial paper is learning-based with real transfer: Zhai et al. 2026, "Learning to Throw". It throws a tennis ball on a 33 cm rope from a 0.21 kg quadrotor. It uses a **fixed** 0.11 s release lag, **does not randomize payload mass or drag**, has **no wind**, and reaches 0.08–0.13 m mean landing error at 1.5–2.5 m range indoors. AeroThrow (Li et al. 2025) is **NMPC + delta-arm manipulator + disturbance observer, not RL and not rule-based**. Its real error is about 1.5–10 cm. No surveyed paper covers outdoor wind, release from altitude of 5 m or more, or randomized payload drag. Our setup is beyond all of them on those axes.

### Cited Findings

**A. "Learning to Throw: Agile and Accurate Cable-Suspended Payload Delivery with a Quadrotor": Zhai, Raimondi, Ren, Geles, Armati, Xing, Scaramuzza (UZH RPG), arXiv:2606.27603, 25 Jun 2026, no venue stated.** This is the paper our notes cite as "Learning to Throw". The obs-noise values 0.01/0.001 match it. — [arXiv html](https://arxiv.org/html/2606.27603v1)
- Platform: quadrotor 0.21 kg, T/W 6.8, CTBR (collective thrust + body rate) low-level control. Payload: a tennis ball on a 33 cm rope with an end loop. The rope is simulated as 15 rigid segments joined by universal joints. — [arXiv](https://arxiv.org/html/2606.27603v1)
- Release mechanism: a 3D-printed shaft runs through the rope loop. A servo lever retracts it on a binary command from the FC. The policy's latch fires at ρ>0.95. Release is delayed by an **identified, fixed 0.11 s actuation lag that is not randomized**. That 0.11 s lag is the only release sysID reported. — [arXiv](https://arxiv.org/html/2606.27603v1)
- Simulator: Isaac Lab. A Warp quadrotor model (actuation-delay queue, low-level controller, motor, battery, thrust/torque map, drag and rotor inflow) is coupled with PhysX TGS for the rope and payload. Physics runs at 500 Hz and the policy at 50 Hz. **Payload aerodynamics are not modelled.** Impact is predicted ballistically without drag. — [arXiv](https://arxiv.org/html/2606.27603v1)
- DR (all uniform):
  - quadrotor mass ±5%
  - initial position ±0.5/0.5/0.3 m, initial velocity ±0.5/0.5/0.3 m/s
  - attitude ±20°, body rate ±30°/s
  - tether length: 6 variants spanning ±6%
  - cable joint offsets ±15° (rates ±15°/s)

  Obs noise is Gaussian: per-step std 0.01 plus per-episode bias std 0.001. Actuator noise: per-step std 0.025, bias std 0.02. **Payload mass, drag, thrust map and actuator latency are fixed to measured values.** No wind. — [arXiv](https://arxiv.org/html/2606.27603v1)
- Training: PPO with asymmetric actor-critic, 1500 iterations (~2×10⁸ steps), ~1 h on an RTX 5090. 5-D action (thrust, 3 body rates, release latch). Goal sampled uniformly within a 2 m disk. Episodes are 5 s. — [arXiv](https://arxiv.org/html/2606.27603v1)
- Reward:
  - **Release term:** $500\exp(-2.5\,\|\hat x_{imp}-x_g\|)$, an exponential (Laplacian-type) kernel on the *predicted* impact point. There is no success radius. The term is sparse in time and dense in error.
  - Time-of-flight penalty: 2.0 per second.
  - Proximity guard: 100, with a 0.25 m margin.
  - Carry-phase terms: small kinetic penalties (1e-3 to 3e-3), command magnitude and rate penalties, and a carry-time cost of 2.0 s⁻¹.
  - Termination penalty: 50.

  — [arXiv](https://arxiv.org/html/2606.27603v1)
- Real-world result: zero-shot, 10 throws per target, landing error mean ± std:
  - 1.5 m: 0.082±0.035 m
  - 2.0 m: 0.105±0.099 m
  - 2.5 m (out of distribution): 0.133±0.082 m

  Throw durations were 1.09–1.19 s. The TO+MPC baseline is an *idealized sim* run with no delay and no noise. It scored 0.127/0.167/0.285 m. Success rate and CEP are not reported. — [arXiv](https://arxiv.org/html/2606.27603v1)
- Ablations on hardware, 2 m target, 5 throws each. Removing one sim component multiplies error by 2.4–3.7×:
  - no quadrotor aero: 0.342 m
  - no motor model: 0.254 m
  - simplified low-level controller: 0.386 m (with a lateral bias)
  - rigid cable: 0.306 m

  A sensitivity sweep at 2 m shows landing error rising 0.027 → 0.413 m as release-velocity error grows 0.4 → 14.3 cm/s. — [arXiv](https://arxiv.org/html/2606.27603v1)
- Dominant error source: release-state error, made worse by low quadrotor model fidelity. Stated limitations: no drag, fixed payload mass and aerodynamics, static targets. — [arXiv](https://arxiv.org/html/2606.27603v1)

**B. AeroThrow: "An Autonomous Aerial Throwing System for Precise Payload Delivery": Ziliang Li, Hongming Chen, Yiyang Lin, Biyu Ye, Ximin Lyu, arXiv:2507.13903, Jul 2025.** — [arXiv abs](https://arxiv.org/abs/2507.13903)
- Method: **NMPC** with hierarchical disturbance compensation: a nonlinear disturbance observer (NDOB, 50 Hz Butterworth) plus INDI in the angular-rate loop. A delta-arm manipulator offsets UAV tracking error. **It is not RL. It is also not "rule-based" in the sense our notes use.** — [arXiv html](https://arxiv.org/html/2507.13903)
- Platform: ~1.59 kg quadrotor with delta arm (3× XL430 servos), Jetson Orin NX, NxtPX4v2 FC, 6S battery, NOKOV mocap with EKF. Payload: 200 g, treated as unknown mass, held by an electromagnet. — [arXiv html](https://arxiv.org/html/2507.13903)
- Release timing: the planner imposes continuous landing-point constraints over a window τ, which gives a *feasible release interval* instead of a single instant. NMPC re-plans t_r online. Actuator delay is handled through the stopping condition. **No delay value, distribution or sysID is reported.** At release the horizontal NDOB estimate jumps from −1.96 N to 0. — [arXiv html](https://arxiv.org/html/2507.13903)
- Real landing error (cm, RMSE/max):
  - full NDOB+INDI: 5.9/8.9 at v_r=4.9 m/s, 5.2/7.2 at 2.2 m/s, 1.5/2.0 at 2.5 m/s
  - plain NMPC: 15.4–22.0 RMSE at the same speeds
  - nominal vs proposed release timing (mean/max): 76.7/83.8 vs 10.1/12.3 at v_r=4.3 m/s

  The comparison used 10 throws per method. Heights and distances are not reported. NDOB alone improves concentration by about 54%. — [arXiv html](https://arxiv.org/html/2507.13903)
- Error sources named: planner reference error, tracking error, **gripper (electromagnet) actuation delay**, and the abrupt parameter change at release. — [arXiv html](https://arxiv.org/html/2507.13903)

**C. TossingBot: Zeng, Song, Lee, Rodriguez, Funkhouser, RSS 2019 (journal version T-RO 2020 per the paper's own text; venue not re-verified), arXiv:1903.11239.** — [ar5iv](https://ar5iv.labs.arxiv.org/html/1903.11239)
- Residual physics:
  - Ballistic point-mass, gravity-only release speed: $\|\hat v\|=\sqrt{a(p_x^2+p_y^2)/(r_z-p_z-\sqrt{p_x^2+p_y^2})}$.
  - Release geometry is fixed: 45°, cd=0.76 m, ch=0.02 m on the real robot.
  - The network predicts a per-grasp residual δ, so $\|v\|=\|\hat v\|+\delta$.
  - Labels are self-supervised from the observed landing point, $\bar\delta=\|v_{xy}\|-\|\hat v_{xy}\|$, with Huber loss.
  - Drag along the velocity direction is left to δ.

  — [ar5iv](https://ar5iv.labs.arxiv.org/html/1903.11239)
- Real throwing accuracy into 25×15 cm boxes. Seen and unseen objects:
  - residual physics: 84.7% / 82.3%
  - physics only: 61.3% / 58.5%
  - direct regression: 54.2% / 52.0%
  - human baseline (unseen): 80.1%

  Unseen box locations: residual 83.9% vs physics 62.2% vs regression 32.7%. Throughput: 514 mean picks per hour. — [ar5iv](https://ar5iv.labs.arxiv.org/html/1903.11239)
- Release timing is not discussed as an error source. Named error sources: drag on light objects, grasp offset from the CoM (centripetal effects), and object not immobilized in the gripper. — [ar5iv](https://ar5iv.labs.arxiv.org/html/1903.11239)

**D. Ghadirzadeh et al., "Deep Predictive Policy Training using RL", arXiv:1703.00727 (IROS 2017 per a course slide; unverified from the paper itself).** — [ar5iv](https://ar5iv.labs.arxiv.org/html/1703.00727)
- Platform: PR2 7-DoF arm, joint velocity commands at 10 Hz, trajectory of T=20 steps. **Release timing is not described.**
- Reward is qualitative and terminal: +2 hit, +1 close, −1 far.
- Data: 180 real attempts (~1 h). Mean deterministic reward was 1.58 (0.83 with unknown distractors). No metric accuracy is reported.

— [ar5iv](https://ar5iv.labs.arxiv.org/html/1703.00727)

**E. MC-PILOT, "Data efficient Robotic Object Throwing with Model-Based RL": Turcato, Giacomuzzo, Terreran, Allegro, Carli, Dalla Libera (Padova), arXiv:2502.05595, 2025.** It extends an earlier MC-PILOT conference paper; that venue was not found. — [arXiv html](https://arxiv.org/html/2502.05595v1)
- Platform: Franka Panda. Only the release velocity is learned.
- **Release delay is modelled explicitly as uniform, $t_d\sim U(a,a+b)$.** (a, b) are fitted by Bayesian optimization over a GP of the Monte Carlo landing error, with bounds a∈[−0.3,0.3] and b∈[0,0.01]. A rough 0.24 s gripper compensation is applied first.
- Fitted real values:
  - rubber ball: a=0.259±0.057 s, b=0.009 s
  - tennis ball: a=0.258 s, b=0.008 s
  - cube: a=0.265 s, b=0.002 s

  In other words, the delay is about 0.26 s with jitter width ≤ 10 ms.
- Cost: saturating $c=1-\exp(-\|p-P\|^2_{\Sigma_c})$ on horizontal error only, with ℓc=0.1.
- Real result: 100% bin accuracy with 3 objects. The baseline was about 50% on the furthest targets. Target radius is 0.1 m (the text says "0.1 mm", but the captions give 0.1 m).
- Error sources: **unmodelled air drag** (the ballistic model overestimates range at medium and high speed), **non-deterministic gripper delay** (the gripper runs on a separate thread), and fingertip elasticity, which makes the delay object-dependent.
- In simulation, dropping the delay model lowered accuracy.

— [arXiv html](https://arxiv.org/html/2502.05595v1)

**F. "Dynamic Throwing with Robotic Material Handling Machines": Werner, Nan, Eyschen, Spinelli, Yang, Hutter (ETH RSL), arXiv:2405.19001, 2024.** — [arXiv html](https://arxiv.org/html/2405.19001v2)
- Platform: 12 t HEAP excavator with a gripper that has 2 passive DoF. Training: PPO in Isaac Gym, 8192 environments, ~1.7 h.
- **Release-delay sysID:** an IMU was dropped and command-to-free-fall timed. Result: **258 ms mean, 15 ms std, n=39**. In simulation it is applied as a constant delay, not randomized.
- Release trigger: gripper command >0.9. A forced release occurs with 5% probability near the target.
- DR covers "identified parameters", observation noise and controller disturbances. Ranges are not published.
- Reward: improvement in minimum 2D/3D distance, plus $\exp(-d_{3D}^2)$, plus action-rate and magnitude penalties. A terminal reward at impact is based on the final 3D error.
- Real result: 9 m target, 10 throws per policy, basketball:
  - 2D policy: downrange 8.92±0.37 m, crossrange 0.05±0.30 m
  - 3D policy: 8.90±0.42 m, −0.14±0.72 m
- Error sources: cabin vibration and drift, grasp variation, release-timing sensitivity (larger for 3D throws because of the tangential velocity), and grasp-strength-dependent delay.

— [arXiv html](https://arxiv.org/html/2405.19001v2)

**G. Kasaei & Kasaei, "Throwing Objects into A Moving Basket While Avoiding Obstacles", ICRA 2023, arXiv:2210.00609.** — [ar5iv](https://ar5iv.labs.arxiv.org/html/2210.00609); [venue per search](https://arxiv.org/pdf/2210.00609)
- Platform: dual UR5e, trained in Gazebo with SAC and DDPG. The 4-D action modulates a throwing kernel: shoulder start and end, duration, and **release time as a learned action**.
- Reward: +1 if the object lands inside the basket radius (basket 0.30×0.30×0.15 m); otherwise −distance; −10 for obstacle collision. One throw per episode.
- Randomization: goal pose, obstacle x position ±5 cm, basket motion.
- Real result: zero-shot, 20 trials per object, SAC:
  - static basket: 90%
  - moving basket: 85%
  - with obstacle: 80%
- Named error sources: **gripper command delay through the controller and network stack**, tracking lag, and unstable grasps.

— [ar5iv](https://ar5iv.labs.arxiv.org/html/2210.00609)

**H. FLARE, "Agile Flights for Quadrotor Cable-Suspended Payload System via RL": Cao, Zhou, Wang, Li (Zhejiang U.), arXiv:2508.09797v2, Jan 2026.** — [arXiv html](https://arxiv.org/html/2508.09797)
- Platform: 305 g quadrotor (T/W 3.5) with a 70 g slung payload. Betaflight FC, inference at 100 Hz. **There is no release.**
- DR: payload initial angle ±10° and target sampling. A robustness test applied ±50% mass and length, but the training ranges are not stated. Latency is not modelled.
- Reward: squared-distance progress (λ=10), smoothness 1e-4, tilt penalty 3, crash penalty 10.
- Real payload-targeting error: 0.12 m average at a 1.5 m target.

— [arXiv html](https://arxiv.org/html/2508.09797)

**I. Older non-RL aerial throw and catch work (pre-2015 or context only).** ETH Flying Machine Arena demos of cooperative net-based ball throwing used iterative learning of the aim. No accuracy numbers were found. — [Robohub](https://robohub.org/?p=2359); [PopSci](https://www.popsci.com/technology/article/2012-10/swiss-quadrotors-learn-throw-and-catch)

**J. Fixed-wing and parachute airdrop.**
- An NTNU paper on precision aerial drop from fixed-wing UAVs (non-RL) computes the release state onboard. It notes that wind can invalidate a precomputed release state. — [NTNU drop.pdf](https://torarnj.folk.ntnu.no/drop.pdf)
- A KAIST deep-RL parafoil precision landing paper (AIAA 2022) is simulation-only per the search snippet and was unverified. — [ResearchGate](https://www.researchgate.net/publication/357601158_Hazard_Avoidance_Landing_of_Loaded_Parafoil_using_Deep_Reinforcement_Learning)

Controller stacks for all eight papers are in the "## Controller stacks" section at the end.

Relevance to us (velocity-command policy over PX4): none of these papers uses a velocity-setpoint policy on PX4. Zhai's CTBR-over-FC pipeline is the closest, and its LLC ablation (2.6–3.7× error) agrees with our Rule 47: the GRU is coupled to the controller, so the PX4 PID must be modelled in sim.

### Inferences
- **Correction for our notes:** AeroThrow is model-based NMPC with a disturbance observer and an actuated arm, not a "rule-based" throw. If our paper outline calls it rule-based, or attributes to it "robustness bought by degrading to place", re-check against the paper. Its release-window method is a planning trick (a window of feasible release times), not a heuristic rule. The claim attributed to the original authors in §V-A was not verified here.
- **Release latency is never randomized in any learning paper found.** Every one uses a single identified constant: Zhai 0.11 s, Werner 0.258 s, or the MC-PILOT fitted uniform window of about 0.26 s ± ≤10 ms. Our N(0.22, 0.05) s is much wider than any measured jitter: Werner's std was 15 ms and MC-PILOT's width was ≤10 ms. That is conservative, but it may cost accuracy. A practical plan: measure our servo/hook delay with the IMU-drop method (Werner), then shrink σ to the measured value plus margin.
- **No paper randomizes payload drag.** Zhai and TossingBot both ignore or residualize it, and MC-PILOT names drag as its main unmodelled error. Our ±20% k/m DR plus a GRU residual over CCIP is the TossingBot residual-physics idea moved to the time-series and aerial case. TossingBot's +23 percentage points over physics-only is the strongest precedent for the residual.
- **Reward shapes that worked:**
  - Zhai: exponential kernel on predicted impact distance, λ=2.5 m⁻¹ (scale 0.4 m), large weight 500, plus a time-of-flight penalty.
  - MC-PILOT: Gaussian saturating cost with ℓc=0.1.
  - Werner: $\exp(-d^2)$ plus progress terms.
  - Kasaei: sparse +1 in radius, else −distance.

  The dense kernel with a scale near the target tolerance is the dominant pattern. A pure binary success bonus appears only in single-step bandit-style settings (Kasaei, Ghadirzadeh).
- **Achievable real accuracy is about 0.05–0.15 m mean error at 1.5–9 m range, indoors or with no wind** (Zhai, AeroThrow, FLARE, MC-PILOT). Werner's outdoor excavator spread is 0.3–0.7 m std at 9 m. Our expected outdoor CEP50 of 0.32–0.36 m sits between these and is plausible.

### Gaps
- No RL or learned airdrop paper (quadrotor, fixed-wing or parachute) with real-world transfer, wind DR and drop altitude of 5 m or more was found. That makes our setting novel, but also without a direct baseline.
- AeroThrow release heights and distances, and whether its "nominal" baseline is rule-based, are not reported in the text read.
- Zhai 2026 does not report payload mass, success rate or CEP. Neither TossingBot nor Ghadirzadeh models release timing.
- The MC-PILOT conference version venue was not found. The TossingBot T-RO 2020 and Ghadirzadeh IROS 2017 venues are from secondary or own-text mentions only.
- Requested topics not found or not checked within the call budget: "Learning to throw arbitrary objects" by Bianchi; Mellinger/Kumar ballistic work; RL slung-load papers with release beyond FLARE (ASTER, arXiv:2603.10715, was found but not read).

## Q2. How was release latency and its randomness modelled (distribution type)?

### Takeaway
Measured constants dominate. Only MC-PILOT uses an explicit distribution: uniform U(a, a+b), with a≈0.26 s and b≤0.01 s fitted by Bayesian optimization. Nobody uses a Gaussian, and nobody randomizes widely.

### Cited Findings
- Zhai 2026: a fixed identified 0.11 s servo lag, not randomized. — [arXiv](https://arxiv.org/html/2606.27603v1)
- Werner 2024: IMU-drop sysID gave 258±15 ms (n=39), applied as a constant delay. — [arXiv](https://arxiv.org/html/2405.19001v2)
- MC-PILOT 2025: $t_d\sim U(a,a+b)$, BO-fitted per object, a=0.258–0.265 s, b=0.002–0.009 s. The delay is object-dependent because of fingertip elasticity. — [arXiv](https://arxiv.org/html/2502.05595v1)
- AeroThrow 2025: no delay value. The feasible release window plus online NMPC re-timing absorbs the timing error. Using its timing method instead of nominal release cut error from 76.7 to 10.1 cm at 4.3 m/s. — [arXiv](https://arxiv.org/html/2507.13903)
- Kasaei 2023: release time is a learned action. Gripper command delay is named as an error source but not modelled. — [ar5iv](https://ar5iv.labs.arxiv.org/html/2210.00609)

### Inferences
- The AeroThrow result shows that at 4.3 m/s, timing error is the largest single term (about 7× reduction once handled). At our 2–6 m/s, a 50 ms σ means 0.1–0.3 m of along-track error per σ. Narrowing σ through sysID is likely worth more than any reward tweak.

### Gaps
- No paper reports the delay distribution *shape* from a large sample. Werner gives only mean and std.

## Q3. Which reward formulations for hitting a target worked best?

### Takeaway
Dense, saturating distance kernels on the (predicted) impact point, with scale matched to the tolerance, combined with a time or flight penalty. No head-to-head comparison of kernel types exists in these papers.

### Cited Findings
- Zhai: $500e^{-2.5d}$ on the predicted impact point at release, −2.0 per second of flight time, carry-time cost 2.0 s⁻¹. — [arXiv](https://arxiv.org/html/2606.27603v1)
- MC-PILOT: $1-\exp(-d^2/\ell_c)$, ℓc=0.1, horizontal error only. — [arXiv](https://arxiv.org/html/2502.05595v1)
- Werner: Δ(min distance) progress term plus $\exp(-d_{3D}^2)$ plus a terminal error reward. — [arXiv](https://arxiv.org/html/2405.19001v2)
- Kasaei: +1 if within the basket radius, else −distance, −10 for collision. SAC beat DDPG and BC. — [ar5iv](https://ar5iv.labs.arxiv.org/html/2210.00609)
- Ghadirzadeh: tiered terminal +2/+1/−1. — [ar5iv](https://ar5iv.labs.arxiv.org/html/1703.00727)
- FLARE: squared-distance progress, weight 10. — [arXiv](https://arxiv.org/html/2508.09797)

### Inferences
- Zhai's design is the closest analogue to ours: an RL quadrotor computing reward at release from the predicted impact. Its time-of-flight and carry-time penalties play the same role as our w_time.

### Gaps
- None of the papers ablates kernel shape or success bonus against distance terms.

## Q4. What real accuracy was achieved and what error sources dominated?

### Takeaway
Real-world accuracy ranges:
- aerial, indoors with mocap: about 2–13 cm mean
- arm into boxes and bins: 80–100% success
- outdoor excavator: 0.3–0.7 m std at 9 m

The recurring dominant error sources are release-state and timing error, unmodelled drag, grasp or attachment variation, and low model fidelity in the flying platform.

### Cited Findings
- Zhai: 0.082–0.133 m. Main error is release-state error, and model fidelity is critical (2.4–3.7× worse without it). — [arXiv](https://arxiv.org/html/2606.27603v1)
- AeroThrow: 1.5–10 cm RMSE. Main errors are tracking, gripper delay and the parameter jump at release. — [arXiv](https://arxiv.org/html/2507.13903)
- TossingBot: 84.7% / 82.3% into 25×15 cm boxes. Main errors are drag, CoM offset and contact. — [ar5iv](https://ar5iv.labs.arxiv.org/html/1903.11239)
- MC-PILOT: 100% within r=0.1 m with 3 objects. Main errors are drag and non-deterministic delay. — [arXiv](https://arxiv.org/html/2502.05595v1)
- Werner: crossrange std 0.30 m (2D) and 0.72 m (3D) at 9 m. Main errors are vibration, grasp and release timing. — [arXiv](https://arxiv.org/html/2405.19001v2)
- Kasaei: 80–90% success. Main errors are gripper command delay and tracking lag. — [ar5iv](https://ar5iv.labs.arxiv.org/html/2210.00609)
- FLARE: 0.12 m payload targeting error, no release. — [arXiv](https://arxiv.org/html/2508.09797)

### Inferences
- All the aerial numbers come from indoor flights with mocap and no wind. Outdoors on GNSS/EKF state estimation, expect that state-estimate error plus wind will dominate over the timing and drag errors these papers report.

### Gaps
- No outdoor learned aerial-release accuracy numbers were found.

## Controller stacks

"Controller not specified" means the full text I read does not state it. [UNVERIFIED] marks a claim that comes from a secondary source or an inference, not the paper text.

| Paper | Policy output | Low-level controller & rate | Modelled identically in sim? | Source |
|---|---|---|---|---|
| Zhai et al. 2026, Learning to Throw | CTBR: collective thrust + 3 body rates + binary release latch, 50 Hz | CTBR low-level controller on the flight controller. Firmware (PX4/Betaflight) and rate: controller not specified | Yes, explicitly modelled in sim: low-level controller, motor model, battery, thrust map, actuation-delay queue, at 500 Hz physics with 10 substeps. Hardware ablation with a simplified LLC: 0.386 vs 0.105 m | [arXiv 2606.27603](https://arxiv.org/html/2606.27603v1) |
| Li et al. 2025, AeroThrow | No learned policy. NMPC tracks the throw trajectory and re-plans release time t_r; delta-arm joint commands go to 3× DYNAMIXEL XL430 servos | NxtPX4v2 FC (PX4-family hardware, [UNVERIFIED] firmware). INDI in the angular-velocity inner loop + NDOB external-force observer (50 Hz Butterworth filter). NMPC and INDI loop rates: controller not specified | Simulation experiments reported; whether sim uses the identical stack: controller not specified | [arXiv 2507.13903](https://arxiv.org/html/2507.13903) |
| Zeng et al. 2019/2020, TossingBot | Grasp pixel and angle + residual δ on ballistic release speed (fixed 45°, fixed release position) | UR5 arm, vendor controller [UNVERIFIED]; motion-primitive execution: controller not specified, rate not stated | Controller not specified | [ar5iv 1903.11239](https://ar5iv.labs.arxiv.org/html/1903.11239) |
| Ghadirzadeh et al. 2017, DPPT | Joint-velocity command trajectory, T=20 steps at 10 Hz; release timing not described | PR2 per-joint velocity PID (PID rate not stated) | Behaviour model trained on samples from the Gazebo simulator; PID fidelity in sim: controller not specified | [ar5iv 1703.00727](https://ar5iv.labs.arxiv.org/html/1703.00727) |
| Turcato et al. 2025, MC-PILOT | Release velocity only (release position and direction fixed by geometry); gripper open command with about 0.24 s compensation | Franka Panda: controller not specified (gripper runs on a separate thread) | Controller not specified; release delay modelled as U(a,a+b) | [arXiv 2502.05595](https://arxiv.org/html/2502.05595v1) |
| Werner et al. 2024, HEAP throwing | Commands to 4 upper-body joint actuators + gripper release command (release when >0.9) | Joint velocity controller over a torque-based actuator model; rate not specified | Yes: Isaac Gym models the joint velocity controller, with disturbances injected into it and velocity-command randomization; release delay is a constant 258 ms | [arXiv 2405.19001](https://arxiv.org/html/2405.19001v2) |
| Kasaei & Kasaei 2023 | 4 throwing-kernel parameters: initial and final shoulder joint values, trajectory duration, release time; one action per throw | UR5e + Robotiq 2F-140. Controller not specified; gripper commands go through the robot controller and network (named delay source) | Gazebo/ODE built to match the real setup; identical controller: controller not specified | [ar5iv 2210.00609](https://ar5iv.labs.arxiv.org/html/2210.00609) |
| Cao et al. 2025/26, FLARE | Policy output type: controller not specified in the text read; onboard inference at 100 Hz (LibTorch) | Betaflight FC; rate not stated | Sim timestep 0.01 s; FC modelling: controller not specified | [arXiv 2508.09797](https://arxiv.org/html/2508.09797) |

Relevance to us: none of the eight papers uses a velocity-setpoint policy over the PX4 position/velocity PID. The closest is Zhai: a CTBR policy over the FC, with the LLC modelled in sim. Its LLC ablation shows a 3.7× error increase when the controller is simplified, which matches our Rule 47 (the GRU residual is coupled to the controller, so PX4 PID must be modelled in training).
