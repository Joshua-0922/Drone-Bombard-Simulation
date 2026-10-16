---
date: 2026-10-16
tags: [research, sim2real, domain-randomization, quadrotor, rl, literature]
status: draft
type: research
---

# Quadrotor RL sim-to-real transfer (2015–2026): DR, SysID, reward, action space, gap

Each paper entry has (a) DR, (b) SysID, (c) reward, (d) action, (e) gap, and **(f) controller stack**.

Method note: every paper was read through arXiv HTML, ar5iv, or PMC full text with WebFetch, which returns a summary rather than the verbatim text. The numbers below come from those summaries and are cited by paper and section. Anything that came only from an abstract or a search snippet is marked **[abstract only]** or **[unverified]**. PDF text extraction was not possible in this environment (no pdftotext or pypdf), so tables that appear only as images or supplements could not be checked.

## Q1. Per-paper fact sheet: (a) DR, (b) SysID and actuator model, (c) reward, (d) action space, (e) sim-to-real gap and fix

### Takeaway
Policies that transferred zero-shot fall into two groups. One group uses wide uniform per-episode DR around a crude model: Molchanov 2019, Kaufmann 2022, Zhang 2024 "extreme adaptation". The other uses careful SysID with little or no DR (Eschmann 2023, Ferede 2023/24, "Learning to Throw" 2026), or adds residual models fitted to real data (Swift 2023, Ferede 2024). In every work, DR is sampled uniformly once per episode. No reviewed paper reported Gaussian or heavy-tailed physics DR, and none used a DR-range curriculum. The curricula found were on reward weights or on the trajectory distribution.

### Cited Findings

**Controller-stack summary table** (MC = whether the low-level controller is modelled in sim)

| Work | Policy output | Low-level controller under policy | Rates | MC | DR style | Transfer |
|---|---|---|---|---|---|---|
| Hwangbo 2017 | rotor thrusts | none (PD attitude helper in training only) | 7 µs per step; HW rate n/r | n/a | none; initial states only | zero-shot, minor error |
| Molchanov 2019 | normalized rotor thrusts | none | policy 100 Hz / dyn 200 Hz | n/a (1st-order lag + OU noise) | uniform per-trajectory: T/W, τ_m, geometry, ±20% | zero-shot, 3 quads |
| Kaufmann 2022 | LV / CTBR / SRT | **controller not specified** ("full control stack" / "low-level controller w/ IMU") | 50 Hz sim; HW n/r | not stated | uniform ±30% m, J; drag | CTBR best; SRT not deployable |
| Swift 2023 | CTBR | **Betaflight PID** + ESC | VIO 100 Hz, cam 30 Hz, inference 8 ms | **yes** (+ battery) | residual k-NN + GP from 50 s real | zero-shot fails; fine-tuned wins |
| Song 2023 | CTBR | **controller not specified** (higher-rate low-level; Agilicious) | 100 Hz | not stated (delay only) | thrust-map + drag, ranges n/r | zero-shot, 108 km/h |
| Eschmann 2023 | RPM setpoints | none | sim 100 Hz | n/a (LPF τ = 0.15 s) | essentially none; critic disturbances | zero-shot; delay essential |
| DATT 2023 | CTBR | **Crazyflie firmware PI rate** | 50 Hz radio | no (1st-order lag k = 0.4) | force disturbance ±3.5 m/s² + Brownian | zero-shot w/ L1 |
| Neural-Fly 2022 | (not RL) | n/r | n/r | n/r | — | wind tunnel ≤ 12.1 m/s |
| Ferede 2023 (G&CNet) | normalized RPM | none | n/r | n/a (τ = 0.06 s) | moment disturbances | gap fixed by moment input |
| Ferede 2024 E2E / INDI | RPM / CTBR | none / **INDI** | sim 100 Hz | n/a / partial (τ = 0.03 s lag) | uniform disturbances | E2E sim advantage mostly lost |
| Zhang 2024 | motor speeds | PixRacer FCU as motor pass-through | 500 Hz | n/a (τ = 0.01 s) | coupled size factor + ±20% + mid-episode | zero-shot, 2 quads |
| Learning to Throw 2026 | CTBR + release latch | Betaflight CTBR (ablation wording) | 50 Hz policy, 500 Hz sim | **yes** (identified + cmd queue) | narrow: mass ±5%, cable ±6%, noise | zero-shot, 0.08–0.13 m |
| Aerial Gym 2025 | velocity+yaw-rate / motor thrust | **ArduPilot** / **PX4 module** (VOXL 2) | 250 Hz inference (motor task) | not stated | n/r | zero-shot |
| OmniDrones 2023 | throttle / vel / rate / att | built-in sim controllers only | ~60 Hz (dt = 0.016) | — | configurable, no defaults | no real tests |

n/r = not retrieved. Sources are cited in the per-paper (f) bullets below. Aerial Gym rows: [arXiv 2503.01471, Sec. V-A, V-C](https://arxiv.org/html/2503.01471). OmniDrones: [arXiv 2309.12825, Sec. III-D, IV](https://arxiv.org/html/2309.12825).

**Hwangbo et al. 2017, RA-L, "Control of a Quadrotor with RL" (Hummingbird)**
- (f) Controller: the policy outputs **rotor thrusts** directly, so no low-level tracking controller sits under it on hardware. In training only, a small PD attitude controller runs alongside the policy. The policy evaluates in 7 µs per step; the hardware control rate was not retrieved. In sim, thrusts act directly on the rigid body, so no underlying controller needs modelling. — [ar5iv 1707.05110, Sec. III-A, IV-B](https://ar5iv.labs.arxiv.org/html/1707.05110)
- (d) The policy maps state directly to 4 rotor thrusts. A small PD attitude controller runs alongside the policy during training. — [ar5iv 1707.05110, Sec. I, III-A, IV-B](https://ar5iv.labs.arxiv.org/html/1707.05110)
- (a/b) The simulator is a noiseless floating body: no drag, negative thrust clipped, diagonal inertia that was not measured, 0.01 s step. **No DR**; only initial states are randomized. — [same, Sec. IV-A/B](https://ar5iv.labs.arxiv.org/html/1707.05110)
- (c) Cost per step = 4e-3‖p‖ + 2e-4‖a‖ + 3e-4‖ω‖ + 5e-4‖v‖, with γ = 0.99. — [same, Eq. 9](https://ar5iv.labs.arxiv.org/html/1707.05110)
- (e) The policy was flown on hardware "without further optimization" and recovered from upside-down throws at about 5 m/s. Gaps the authors list: asymmetric motor dynamics (rotors "accelerate relatively fast but decelerate quite slowly"), ground effect, battery level and weight distribution, wireless delay, and state-estimation error. They propose no fix. — [same, Sec. V–VI](https://ar5iv.labs.arxiv.org/html/1707.05110)

**Molchanov et al. 2019, IROS, "Sim-to-(Multi)-Real" (Crazyflie 2.0 plus 73 g and 124 g custom quads)**
- (f) Controller: the policy outputs **normalized rotor thrusts** with no low-level controller underneath (it replaces the attitude and position stack). Policy at 100 Hz, dynamics at 200 Hz in sim. In sim, the rotor command passes through the first-order lag plus OU noise, so no PID is modelled. Whether it ran onboard or offboard: not verified. — [ar5iv 1903.04628, Sec. IV-B, IV-E](https://ar5iv.labs.arxiv.org/html/1903.04628)
- (a) DR is sampled **per trajectory**, uniform (Table I): body width U(0.05, 0.2) m, motor time constant T U(0.1, 0.2) s (nominal 0.15), T/W U(1.8, 2.5) (nominal 1.9), torque-to-thrust U(0.005, 0.02) (nominal 0.006). The mass entry did not parse ("≤5", probably a percentage) **[unverified]**. In the geometry-based approach, mass comes from component densities and inertia from the parallel-axis theorem. A second scheme randomizes ±X% around nominal; ±20% gave the best multi-quad results. — [ar5iv 1903.04628, Sec. V-A, Table I, Table IV](https://ar5iv.labs.arxiv.org/html/1903.04628)
- (b) Motor lag is a discrete first-order low-pass filter (Eq. 7) with settling time T ≥ 4dt. Motor noise is a discretized Ornstein–Uhlenbeck process (Eq. 9); θ and σ are not given. Thrust is f = f_max(û′+ε)², with f_max = 0.25·g·m·r_t2w. Drag is neglected. Sensor noise is zero-mean Gaussian on position, attitude and velocity, with magnitudes "roughly estimated" from logs of the drone at rest; gyro noise follows RotorS. Dynamics run at 200 Hz, the policy at 100 Hz. — [same, Sec. IV-B to IV-E](https://ar5iv.labs.arxiv.org/html/1903.04628)
- (c) Cost = (‖e_p‖ + α_v‖e_v‖ + α_ω‖e_ω‖ + α_a‖a‖ + α_R·angle(R))·dt, with α_ω = 0.1, α_a = 0.05, α_R = α_v = 0. α_ω = 0 or 1.0 failed to train. α_ω = 0.05 did not take off on hardware. α_ω = 0.25 was best in simulation but raised real position error. — [same, Eq. 14, Table III](https://ar5iv.labs.arxiv.org/html/1903.04628)
- (d) Normalized per-rotor thrust f̂ ∈ [0,1]⁴; network 2×64 tanh; PPO. — [same, Sec. IV-B, V-B](https://ar5iv.labs.arxiv.org/html/1903.04628)
- (e) Real trajectory-tracking error: Mellinger 0.11 m; policy with motor lag 0.19 m; without motor delay 0.21 m; without noise 0.24 m. On the 124 g quad, the Crazyflie-trained network gave e_t 0.47 m against 0.04 m for Mellinger. — [same, Sec. VI-B, Table IV](https://ar5iv.labs.arxiv.org/html/1903.04628)

**Kaufmann et al. 2022, ICRA, "A Benchmark Comparison of Learned Control Policies for Agile Quadrotor Flight" (0.768 kg race quad)**
- (f) Controller: the policy outputs one of **LV**, **CTBR**, or **SRT** (rotor thrusts). LV is tracked by an unnamed "full control stack with access to accurate state estimation"; CTBR by an unnamed "low-level controller using measurements from an inertial sensor"; SRT has no inner loop. Policy at 50 Hz in sim; real-world rate and inner-loop rate not stated. The paper does not say whether the LV or CTBR tracking controllers were modelled in sim. **Controller not specified** (name and type). — [ar5iv 2202.10796, Sec. II, IV](https://ar5iv.labs.arxiv.org/html/2202.10796)
- (a) Uniform per-rollout DR (Table II): mass ±30%, inertia ±30%, gravity ±0.4 m/s², drag k_vx and k_vy ±0.3, k_vz ±0.15, plus Gaussian observation noise. The thrust-constant entries read ±0.0 even though the text says thrust is randomized; the paper leaves this unresolved **[unverified]**. — [ar5iv 2202.10796, Sec. IV-B, Table II](https://ar5iv.labs.arxiv.org/html/2202.10796)
- (b) First-order motor model (k_mot not stated); symplectic Euler at 1 ms. The model-mismatch test uses NeuroBEM with 20 ms delay. The real system had about 60 ms communication latency. — [same, Sec. III-B, V-A/B](https://ar5iv.labs.arxiv.org/html/2202.10796)
- (c) Reward = −quadratic tracking error (Q = diag{0.1×3, 0.02×9, 0.002×3, 0.01×3}) − action penalty R = 0.001 per action, and −500 on crash. — [same, Eq. 5](https://ar5iv.labs.arxiv.org/html/2202.10796)
- (d/e) **This is the main action-space result.** Single-rotor thrust (SRT) is close to CTBR in nominal simulation, but under model mismatch it crashed on RaceA/B/C and Split-S, and it "could not be deployed" on hardware because of its sensitivity to delay. CTBR tolerated 60 ms of delay while hovering without noticeable loss, though the same delay crashed racing. CTBR transferred and reached 3 g and more than 45 km/h. Linear velocity (LV) also transferred but was clearly worse on agile maneuvers. CTBR position error, BEM simulation vs. real: hover 0.6 vs 4.4 cm; RandC 2.6 vs 47.8 cm. The authors attribute the residual gap to battery voltage, the thrust map, and mass-distribution torque imbalance. — [same, Fig. 3, Tables V–VII, Sec. V-B](https://ar5iv.labs.arxiv.org/html/2202.10796)

**Kaufmann et al. 2023, Nature, "Swift" (champion-level drone racing)**
- (f) Controller: the policy outputs **CTBR**. Underneath is the onboard **Betaflight PID**, and the ESC feeds the motors. **The same Betaflight PID, ESC, and battery behaviour are modelled in sim**, predicting motor commands within 1%. Policy inference takes 8 ms on the CPU, with VIO at 100 Hz and camera at 30 Hz; the policy command rate and Betaflight loop rate were not retrieved. — [PMC10468397, Methods "Quadrotor simulation", "Drone hardware configuration"](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/)
- (b) Quadratic propeller lift and drag. Aerodynamics are a grey-box polynomial in body velocity and mean squared motor speed, fitted to mocap flight data. Motors are first order (k_mot). The Betaflight PID and ESC are modeled, predicting motor commands within 1% error. A grey-box battery model maps power draw to voltage. — [PMC10468397, Methods "Quadrotor simulation"](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/)
- (e) The fix is residual models fitted to real data. Dynamics residual: k-NN regression with k = 5 over (state, commanded thrust), about 800–1,000 samples. Perception residual: nine 1D Gaussian processes with RBF-mixture kernels, sampled from the posterior during fine-tuning. The real data is **3 rollouts, about 50 s of flight**. Fine-tuning used 2e7 steps after 1e8 pretraining steps, updating policy weights only. Zero-shot and DR baselines did not complete a single lap under the shifted dynamics. — [same, Methods "Residual dynamics model", "Residual observation model", "Fine-tuning", "Simulation results"](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/)
- (c/d) Action is CTBR (mass-normalized collective thrust plus body rates). r = r_prog + r_perc + r_cmd − r_crash, where r_cmd penalizes body rates and action change, and r_crash = 5. The weights λ1–λ5 are in Extended Data Table 1a, which was **not retrieved**. The observation includes the previous action. Sensorimotor latency is 40 ms. — [same, Methods "Observations, actions and rewards", Discussion](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/)

**Song et al. 2023, Science Robotics, "Reaching the Limit in Autonomous Racing: OC vs RL"**
- (f) Controller: the policy outputs **CTBR**, tracked by "a low-level controller that operates at a higher frequency". Deployment uses the Agilicious framework, but the paper does not say whether Agilicious provides that loop. Policy at 100 Hz; inner-loop rate not stated. Sim modelling of the low-level controller is not stated; only the measured system delay is simulated. **Controller not specified.** — [arXiv 2310.10943 HTML, Materials and Methods](https://arxiv.org/html/2310.10943)
- (a) Thrust-map and drag coefficients are randomized to mimic battery sag and aerodynamics. Ranges are **not given** in the text. The measured real delay is simulated in training (value not stated). Simulation tests apply a 40 ms delay to every method. — [arXiv 2310.10943 HTML, Results "RL vs OC", "Sim-to-real"](https://arxiv.org/html/2310.10943)
- (c/d) Action is CTBR with tanh outputs, MLP 2×256. Reward = gate progress − 0.01‖ω‖, −10 on collision, +10 on finish. — [same, Materials and Methods, Eq. 5](https://arxiv.org/html/2310.10943)
- (e) Training uses a simple rigid-body model in Flightmare; NeuroBEM is used only for testing. Under the realistic model, success was trajectory tracking 0%, contouring control 20%, RL 100%. On hardware the 6S drone reached 108 km/h and 12.58 g. The authors conclude RL wins because it optimizes a better **objective**, not because it is a better optimizer. — [same, Fig. 2C, Fig. 5, Discussion](https://arxiv.org/html/2310.10943)

**Eschmann, Albani, Loianno 2023/24, "Learning to Fly in Seconds" (Crazyflie, RLtools, TD3)**
- (f) Controller: the policy outputs **4 RPM setpoints** and replaces the Crazyflie attitude and position controllers, so no tracking controller sits underneath. Motor and ESC response are modelled only as a first-order LPF with τ = 0.15 s. Sim at 100 Hz; the hardware policy rate was not retrieved **[unverified]**. — [arXiv 2311.13081 HTML, Sec. IV](https://arxiv.org/html/2311.13081)
- (a) The summary reports the paper as training "without domain randomization" for sim-to-real, while the critic receives force and torque disturbances sampled at each episode start (asymmetric actor-critic), plus observation noise. Exact ranges are in a supplementary PDF that was **not retrieved**. — [arXiv 2311.13081 HTML, Sec. I/III/IV](https://arxiv.org/html/2311.13081)
- (b) Rotor delay is a first-order low-pass filter with **τ = 0.15 s**, chosen empirically from a typical range of 0.05–0.25 s, so an action affects the state 5–25 control steps later. Simulation runs at 100 Hz. — [same, Sec. IV](https://arxiv.org/html/2311.13081)
- (c) Reward = −(position, tilt 1−q_w², velocity, angular rate, action-deviation costs) + a survival constant. Curriculum: every 100k steps, the reward weights are multiplied by factors C_p up to C_target, and exploration noise decays on the same schedule. — [same, Sec. IV](https://arxiv.org/html/2311.13081)
- (d) Actions are 4 RPM setpoints. The actor observation includes an action history of N_H × 4. — [same](https://arxiv.org/html/2311.13081)
- (e) Ablation on the real Crazyflie, 10 seeds (Table II): **without simulated rotor delay, 0/10 usable policies**. Without action history: 5/10 at 300k steps and 0/10 on trajectory tracking. Without the curriculum: 1/10 at 300k steps. Without AAC: 6/10. Observation noise and disturbances were classed "minor". Real Lissajous RMSE at fast speed (3 m/s, 0.9 g): 0.24 m, against 0.36 m for the geometric controller. — [same, Sec. VI, Tables II–III](https://arxiv.org/html/2311.13081)

**Huang et al. 2023, CoRL, "DATT: Deep Adaptive Trajectory Tracking" (Crazyflie 2.1, 40 g, T/W just under 2)**
- (f) Controller: the policy outputs **CTBR** (desired total thrust plus body rates). Underneath is the **Crazyflie firmware PI attitude-rate controller**, followed by the actuation matrix to motor thrusts. Commands are sent at 50 Hz over 2.4 GHz radio through Crazyswarm, with mocap state at 50 Hz. **In sim the PI is not modelled explicitly**; it is replaced by a first-order lag (k = 0.4) on ω and fΣ. Gains not given. — [arXiv 2310.09053 HTML, Sec. 3.1, 4.1–4.2](https://arxiv.org/html/2310.09053)
- (a) Physical parameters are **not** randomized. The disturbance is a 3D force with uniform random direction and magnitude in [−3.5, 3.5] m/s², set at episode start, then evolving as a Brownian random walk d ← d + ε, ε ~ N(0, 0.01·I·dt). The reference trajectory is held fixed for the first 2.5M of 20M steps (a curriculum). — [arXiv 2310.09053 HTML, Sec. 4.1](https://arxiv.org/html/2310.09053)
- (b) A first-order lag with k = 0.4 is applied to the commanded ω and fΣ, chosen for the hardware. Mocap and commands run at 50 Hz over radio. — [same, Sec. 4.1–4.2](https://arxiv.org/html/2310.09053)
- (c/d) Action is CTBR. Cost = ‖p − p_d‖ + 0.5‖ψ‖ + 0.1‖v‖. An L1-adaptive disturbance estimate d̂ is a policy input. — [same, Sec. 3.1, 3.3](https://arxiv.org/html/2310.09053)
- (e) A learned RMA estimator was competitive in simulation (0.055 vs 0.062 m) but degraded on hardware from domain shift. Replacing it with closed-loop L1 estimation fixed this. Real errors with fans and a drag plate: 0.095 m on smooth trajectories and 0.161 m on infeasible ones. — [same, Tables 2 and 4, Sec. 5](https://arxiv.org/html/2310.09053)

**O'Connell et al. 2022, Science Robotics, "Neural-Fly" (not RL)**
- (f) Controller: not RL. A nonlinear tracking controller is augmented with a learned aerodynamic-force term and adapted online. Its output layer, the autopilot underneath, and the rates were **not retrieved (abstract only)**. — [arXiv 2205.06908](https://arxiv.org/abs/2205.06908)
- About 12 minutes of flight data trains the DAIML wind-invariant basis, with online composite adaptation of the linear coefficients. Tested in the Caltech Real Weather Wind Tunnel at up to 12.1 m/s (43.6 km/h). — [arXiv 2205.06908 abstract](https://arxiv.org/abs/2205.06908) **[abstract only]**

**Ferede et al. 2023, "End-to-end neural network based optimal quadcopter control" (G&CNet, imitation of optimal control rather than RL)**
- (f) Controller: the network outputs **4 normalized RPM commands**, so no low-level controller sits underneath (end-to-end guidance and control). Sim models motors as a first-order delay (τ = 0.06 s), with no PID modelled. Rate **not specified** in the retrieved text. — [arXiv 2304.13460 HTML, Sec. 2.1, 4.1.1](https://arxiv.org/html/2304.13460)
- (b) SysID by linear regression on flight logs (Table 1): thrust and drag following Svacha et al.; inertia from Sun et al. (0.000906 / 0.001242 / 0.002054 kg·m²); **motor time constant τ = 0.06 s**. Action is 4 normalized RPMs. — [arXiv 2304.13460 HTML, Sec. 2.1, Table 1](https://arxiv.org/html/2304.13460)
- (e) In real flight the drone drifted and a low-frequency pitch-moment mismatch appeared. Fix: feed the estimated external moments to the network (measured minus modeled moment, 8 Hz Butterworth), trained with sampled M_x, M_y ∈ [−0.04, 0.04] and M_z ∈ [−0.01, 0.01] N·m. — [same, Sec. 2.4, 4.1.2–4.2.2](https://arxiv.org/html/2304.13460)

**Ferede et al. 2024, ICRA, "End-to-end RL for Time-Optimal Quadcopter Flight"**
- (f) Controller, two variants. E2E: **RPM** commands with no inner loop. INDI: **CTBR** (p, q, r ∈ [−3, 3], thrust ∈ [0, 15]) tracked by an onboard **INDI** controller, modelled in sim as first-order rate and thrust delays (τ = 0.03 s) rather than the INDI law itself. Sim step 0.01 s; hardware rates were not retrieved. — [arXiv 2311.16948 HTML, Sec. II-B, II-C](https://arxiv.org/html/2311.16948)
- (b) The end-to-end model is nominal physics plus residual MLPs for thrust and moment (1 hidden layer of 32), fitted to flight data. In the INDI variant, rate and thrust delays are τ = 0.03 s, with drag d_x = 0.34 and d_y = 0.43 identified from flight. — [arXiv 2311.16948 HTML, Sec. II-B1, II-C1](https://arxiv.org/html/2311.16948)
- (a) There is no DR as such. Initial states and external disturbances are sampled uniformly: M_x, M_y ±0.03, M_z ±0.01, F_z ±0.5. Onboard-measured force and moment are policy inputs. — [same, Sec. II-B3](https://arxiv.org/html/2311.16948)
- (c/e) Reward = progress + gate bonus (10 − 10‖·‖) − 10 on collision, with no smoothness term. Action is RPM. The simulated advantage of end-to-end RPM over INDI/CTBR was 1.39 s per lap and shrank to 0.05 s in real flight (end-to-end 15.75 s sim vs 17.35 s real). The authors note end-to-end control is more sensitive to model error. — [same, Sec. IV, Table I](https://arxiv.org/html/2311.16948)

**Zhang et al. 2024/25, "A Learning-based Quadcopter Controller with Extreme Adaptation" (one policy for 267 g and 985 g quads)**
- (f) Controller: the policy outputs **motor speeds** at 500 Hz on a Qualcomm RB5, with a **PixRacer FCU** and off-the-shelf ESCs underneath. The FCU acts as a motor-command pass-through; its internal rate and attitude loops are not used by the policy. Sim models motors as first-order with τ = 0.01 s. Torque and thrust references come from a rate law K = diag(20, 20, 4) inside the reward, not from an onboard controller. — [arXiv 2409.12949 HTML, Sec. II-C, III](https://arxiv.org/html/2409.12949)
- (a) Parameters are coupled through a single size factor c ~ U[0,1]: arm 0.046–0.200 m, mass ∝ l³ (0.226–0.950 kg), inertia ∝ l⁵, drag ∝ l², C_F exponential in c, maximum motor speed 800–8044 rad/s. On top of this: per-rotor effectiveness U[0.7, 1.3], payload 18–40% of mass offset ±50% of the arm, ±20% uniform noise after scaling, and mass, inertia and CoM **re-randomized at a random time within the episode**. Motor τ is fixed at 0.01 s. Measured latency is 5 ms. Tested out to 16× the training range. — [arXiv 2409.12949 HTML, Sec. II-E, Table II, Eqs. 13–17](https://arxiv.org/html/2409.12949)
- (c/d) Actions are motor speeds. Reward = −‖a_t − a_{t−1}‖ + survival + thrust- and torque-tracking terms (weights not listed). IL→RL blending uses α = e^(−0.001·epoch). A 1D-CNN adaptation module reads 100 state-action pairs at 500 Hz. — [same, Sec. II-B to II-D](https://arxiv.org/html/2409.12949)
- (e) Sim–real Pearson correlation r = 0.652. Survived 20% single-motor thrust loss, a 20% off-centre payload, and wind up to 3.5 m/s. — [same, Sec. V, Table V](https://arxiv.org/html/2409.12949)

**Zhai, …, Scaramuzza 2026, "Learning to Throw: Cable-Suspended Payload Delivery" (UZH RPG; arXiv June 2026, no venue stated). This is the work closest to ours.**
- (f) Controller: the policy outputs **CTBR plus a release latch** at 50 Hz. Underneath is a low-level CTBR controller, called "the identified Betaflight CTBR controller" in the ablation (the paper does not name it explicitly for hardware). **It is modelled in sim** as an identified controller block that maps body rates to motor-speed setpoints, preceded by a command queue for delay (Agilicious-based). Sim runs at 500 Hz with each action held for 10 substeps. Inner-loop rate and FC board **not specified**. Replacing it with a geometric body-rate controller raised real error from 0.105 to 0.386 m. — [arXiv 2606.27603 HTML, Sec. III-B, Table V](https://arxiv.org/html/2606.27603)
- (a) DR, all uniform: mass ±5%; cable length ±6% (6 variants); cable tilt offsets ±15° with initial rates ±15°/s; initial-state ranges. Observation noise is per-step std 0.01 plus per-episode bias std 0.001. Command (thrust, body-rate) noise is per-step std 0.025 plus bias std 0.02. **Payload mass, drag, thrust map and actuator latency are deliberately NOT randomized; they are fixed to measured values.** — [arXiv 2606.27603 HTML, Sec. III-C](https://arxiv.org/html/2606.27603)
- (b) SysID of the standalone quadrotor, reused unchanged with the payload: command queue delay τ_d, low-level controller model, first-order motor τ_mot, battery model, thrust/torque–speed map, drag and rotor inflow. The release command is delayed by the **identified 0.11 s actuation lag**. — [same, Sec. III-B, Table I](https://arxiv.org/html/2606.27603)
- (c) Carry phase: kinetic penalties (−1e-3 on v and ω, −3e-3 on position term), body-rate command penalty 0.05, command-rate L1/L2 penalty c_Δ = (0.15, 0.025, 0.025, 0.025) with η = 0.2 and β = 0.5, carry-time cost 2.0 /s. Release: 500·exp(−2.5‖x̂_imp − x_g‖) using a drag-free parabola as the impact predictor, time-of-flight penalty 2.0, proximity penalty 100 (margin 0.25 m). Termination −50. — [same, Eqs. 1–4, Table II](https://arxiv.org/html/2606.27603)
- (d) Action is CTBR plus a release latch (release when ρ > 0.95), at 50 Hz. — [same](https://arxiv.org/html/2606.27603)
- (e) Real landing error was 0.082 m (1.5 m target) and 0.105 m (2.0 m). Hardware ablations at the 2.0 m target: removing the aerodynamics model gave 0.342 m, removing the motor model 0.254 m, a simplified controller model 0.386 m, a rigid cable 0.306 m. **Fidelity of the controller model mattered most.** — [same, Tables III, V](https://arxiv.org/html/2606.27603)

**Simulator frameworks**
- (f) Controllers: Aerial Gym ran velocity/yaw-rate policies on **ArduPilot** with ROVIO VIO, and a motor-thrust policy as a **PX4 module** on a VOXL 2 at 250 Hz. Whether the ArduPilot or PX4 loops were modelled in sim is **not stated**. OmniDrones offers rotor/velocity/rate/attitude controllers in sim only, with no hardware. — [Aerial Gym Sec. V](https://arxiv.org/html/2503.01471), [OmniDrones Sec. III-D](https://arxiv.org/html/2309.12825)
- OmniDrones (2023): DR can cover mass and inertia, rotor force constant and motor gain, wind and drag (all at startup or at runtime), and configuration (startup only). **No default numeric ranges** are given in the paper. The default action is per-motor throttle, with velocity, rate (CTBR) and attitude modes available. Direct and rate control worked best; velocity control was "insufficient for fine-grained tasks". No sim-to-real results are reported. dt = 0.016 s. — [arXiv 2309.12825 HTML, Sec. III-C/D, IV-C, Table II](https://arxiv.org/html/2309.12825)
- Aerial Gym (2025): the motor is first order with **separate rise and fall time constants** τ_inc and τ_dec (Eqs. 8–9), addressing the asymmetry Hwangbo observed. IMU noise and bias-walk presets exist for VN-100 and BMI085. Numeric DR defaults are not in the paper. Velocity/yaw-rate policies were deployed zero-shot on an ArduPilot quad with VIO. A motor-level policy on a VOXL 2 with PX4 reached 0.09 m error, with real per-motor commands diverging (attributed to asymmetric mass or motor constants). — [arXiv 2503.01471 HTML, Sec. III-C4, III-D, V-A to V-C](https://arxiv.org/html/2503.01471)

**Other 2025–26 payload and wind works (search snippets only, papers not opened)** **[unverified]**
- FLARE (RA-L 2026): zero-shot RL for agile flight with a cable-suspended payload. — [arXiv 2508.09797](https://arxiv.org/html/2508.09797v2)
- ASTER (2026): inverted flight with a cable-suspended payload. — [arXiv 2603.10715](https://arxiv.org/html/2603.10715)
- GustPilot (2026): hierarchical DRL plus INDI; wind-field configuration randomized at each reset. — [arXiv 2603.19966](https://arxiv.org/pdf/2603.19966)
- A 2026 adaptive outer-loop RL preprint argues that DR alone produces "overly conservative, hyper-reactive" policies. — [arXiv 2605.16015](https://arxiv.org/pdf/2605.16015)

### Inferences
- The observation and command noise in our current DR (0.01 per step plus 0.001 bias; 0.025 plus 0.02) is numerically identical to Learning to Throw Sec. III-C. That paper pairs this noise with a SysID'd command delay, motor τ, and controller model, which our DR lacks.
- Our velocity-command L0 on PX4 offboard sits at the "LV" level in Kaufmann 2022's taxonomy. LV transferred to hardware there, and it is the least sensitive to delay and motor dynamics. The cost is less agility, which matters little at 2–6 m/s.
- Controller stack: no reviewed hardware RL work used a **PX4 offboard velocity** loop underneath the policy. The nearest are Aerial Gym (ArduPilot velocity, zero-shot, controller modelling not stated) and Kaufmann 2022 LV ("full control stack", unnamed). The two works that model the real inner loop explicitly (Swift with Betaflight, Learning to Throw with an identified Betaflight CTBR controller) are also the ones that show its fidelity matters on hardware.
- Several groups treat a mismatch in the low-level controller model as the dominant gap: Learning to Throw (0.386 m without it), Swift (Betaflight and ESC modelled), and our own exp_042 GRU collapse under the PX4 PID. This supports the plan to train on the identified PX4 PID.

### Gaps
- Exact ranges from the Eschmann supplement, the RLtools config, and Swift Extended Data Table 1 (reward λs) were not retrieved.
- Song 2023 does not give its DR ranges or the training delay value.
- The Molchanov mass range did not parse.
- The FLARE, ASTER and GustPilot DR tables were not read.

## Q2. Which DR or modelling components did ablations show mattered most?

### Takeaway
Actuator and command dynamics come first: motor or rotor delay, command latency, and the low-level controller model. Eschmann's 0/10 without rotor delay is the starkest result. Noise and DR breadth give moderate gains. Randomizing T/W alone did not explain Molchanov's gains.

### Cited Findings
- Without a simulated rotor delay, 0/10 policies flew. Without action history, trajectory tracking dropped to 0/10. Observation noise and disturbances were minor. — [Eschmann, Sec. VI, Table II](https://arxiv.org/html/2311.13081)
- Removing motor delay degraded Molchanov's policy only slightly (0.19 → 0.21 m); removing noise hurt more (→ 0.24 m). ±20% DR beat full-range DR in accuracy, while full range was more consistent across platforms. T/W randomization alone did not explain the gains. — [Molchanov, Sec. VI-B, VII, Table IV](https://ar5iv.labs.arxiv.org/html/1903.04628)
- SRT policies could not be deployed because of delay sensitivity; CTBR tolerated 60 ms in hover. — [Kaufmann 2022, Fig. 3, Sec. V-B](https://ar5iv.labs.arxiv.org/html/2202.10796)
- Real-world ablation, 2.0 m target error: no controller model 0.386 m, no aerodynamics 0.342 m, rigid cable 0.306 m, no motor model 0.254 m, against 0.105 m for the full model. — [Learning to Throw, Table V](https://arxiv.org/html/2606.27603)
- Zero-shot and DR baselines failed to complete a lap under dynamics shift; residual models from 50 s of real data fixed this. — [Swift, Methods "Simulation results"](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/)

### Inferences
- In our current DR, the missing pieces line up with the factors these ablations rank highest: no motor lag, no command latency, and fixed T/W. Release latency, which we do randomize, is a payload-side analogue of Learning to Throw's identified 0.11 s.

### Gaps
- No reviewed paper ablated wind DR distribution shape on hardware.

## Q3. How were latency and motor dynamics handled?

### Takeaway
Almost always as a first-order lag on rotor speed or on the commanded CTBR, with τ from 0.01 to 0.15 s. Delay is either measured and simulated as a fixed queue, or stress-tested (0–100 ms). Only Aerial Gym models the rise/fall asymmetry.

### Cited Findings
| Work | Lag model | Value | Delay |
|---|---|---|---|
| Molchanov 2019 | 1st-order on rotor command, DR | T U(0.1, 0.2) s, nominal 0.15 | — |
| Eschmann 2023 | 1st-order LPF on RPM | 0.15 s (range 0.05–0.25) | via action history |
| Ferede 2023 | 1st-order motor | 0.06 s (SysID) | — |
| Ferede 2024 (INDI) | 1st-order on rate and thrust | 0.03 s (SysID) | — |
| DATT 2023 | 1st-order on ω and fΣ | k = 0.4 | 50 Hz radio |
| Zhang 2024 | 1st-order motor | 0.01 s fixed | 5 ms measured |
| Kaufmann 2022 | 1st-order k_mot | not stated | sweep 0–100 ms; real ≈ 60 ms |
| Song 2023 | — | — | measured delay simulated; 40 ms in tests |
| Swift 2023 | 1st-order k_mot + Betaflight/ESC + battery model | not retrieved | 40 ms sensorimotor |
| Learning to Throw 2026 | command queue τ_d + 1st-order τ_mot + controller model | release lag 0.11 s | identified |
| Aerial Gym 2025 | 1st-order, τ_inc ≠ τ_dec | not stated | — |

Sources: [Molchanov](https://ar5iv.labs.arxiv.org/html/1903.04628), [Eschmann](https://arxiv.org/html/2311.13081), [Ferede 2023](https://arxiv.org/html/2304.13460), [Ferede 2024](https://arxiv.org/html/2311.16948), [DATT](https://arxiv.org/html/2310.09053), [Zhang 2024](https://arxiv.org/html/2409.12949), [Kaufmann 2022](https://ar5iv.labs.arxiv.org/html/2202.10796), [Song](https://arxiv.org/html/2310.10943), [Swift](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/), [Learning to Throw](https://arxiv.org/html/2606.27603), [Aerial Gym](https://arxiv.org/html/2503.01471)

### Inferences
- For a PX4 velocity-command interface, the analogue would be a command-queue delay plus a first-order lag on achieved velocity or attitude, identified from offboard step logs. This would follow Learning to Throw or DATT rather than per-rotor lags.

### Gaps
- None of the reviewed papers randomized latency with a Gaussian distribution. Most either fixed it at the measured value or swept it in evaluation.

## Q4. Non-uniform distributions, heavy tails, or progressive DR scaling?

### Takeaway
None were found for physical parameters. All reported physics DR is uniform and per episode. The non-uniform elements are noise processes (Gaussian per step, OU motor noise, Brownian disturbance), a log-like coupling of parameters through one size factor, intra-episode re-randomization, and curricula on reward weights or references rather than DR width.

### Cited Findings
- Molchanov: OU motor noise and Gaussian sensor noise. Their first approach mentions Gaussian sampling around nominal "in some cases", but this could not be verified from the summary. — [Sec. IV-B/D, V-A](https://ar5iv.labs.arxiv.org/html/1903.04628)
- DATT: Brownian random-walk force disturbance, N(0, 0.01·dt) per step; trajectory curriculum held for 2.5M steps. — [Sec. 4.1](https://arxiv.org/html/2310.09053)
- Zhang 2024: parameters coupled through one U[0,1] size factor (C_F exponential in c), plus mid-episode re-randomization of mass, inertia and CoM. Evaluation extends to 16× the training range; no training-time range curriculum is described. — [Sec. II-E](https://arxiv.org/html/2409.12949)
- Eschmann: multiplicative reward-weight curriculum every 100k steps; removing it cut success to 1/10. — [Sec. IV, VI](https://arxiv.org/html/2311.13081)
- Learning to Throw: per-step Gaussian noise plus per-episode Gaussian bias on observations and commands. — [Sec. III-C](https://arxiv.org/html/2606.27603)

### Inferences
- Our use of N(0, 1.5) for mean wind and N(0.22, 0.05) for release latency already goes beyond the uniform-only practice in this literature. Neither is contradicted, but neither is directly supported by a hardware ablation in these papers.

### Gaps
- A targeted search for RL quadrotor works with Dryden or von Kármán gust models transferred to hardware returned nothing; no confirmed example was found.
