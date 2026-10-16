---
date: 2026-10-16
tags: [sim2real, domain-randomization, system-identification, literature, legged, manipulation]
status: draft
type: research
---

# DR and system identification practice in landmark legged/manipulation sim-to-real RL papers (2017–2025)

Method note: primary text was read through ar5iv HTML renders of the arXiv papers and raw GitHub configs. A small model summarised each fetch, so every number below is as extracted. Items marked **[UNVERIFIED]** are ambiguous in the extraction or could not be checked against the paper. Supplementary tables I could not reach (Haarnoja 2024 Table S3, Radosavovic 2024 supplement, OpenAI 2019 appendix) are listed under Gaps.

## Q1. Per-paper fact sheet: (a) DR, (b) pre-RL sim setup / SysID, (c) reward, (d) noise and latency, (e) ablations

### Takeaway
Two clear patterns. (1) The papers with the largest real-world ablation effects (Peng 2018, OpenAI 2018, Tan 2018, Hwangbo 2019) all say that **latency/timestep and actuator modelling mattered more than mass or friction ranges**. Removing timestep randomization cut real success from 0.89 to 0.29 (Peng). Without an actuator network ANYmal could not take a single step (Hwangbo). (2) In the Isaac-Gym era (2021+), DR sets are narrow: friction, added mass, CoM, motor strength ±10%, pushes, and a delay range. The authors put the work into identifying the invariant parts, such as actuator dynamics and measured latency, instead of widening ranges.

### Cited Findings

#### Tobin et al. 2017, "Domain Randomization for Transferring DNNs from Simulation to the Real World" (visual DR, object localisation)
- (a) Randomised per training image: 0–10 distractor objects with random shape, position and texture; textures of table, floor, skybox and robot; camera position inside a 10×5×10 cm box, orientation up to 0.1 rad per axis, FOV up to 5% scaling; number, pose and specularity of lights; type and amount of image noise — [Tobin 2017, §IV](https://arxiv.org/abs/1703.06907)
- (e) Real localisation error was about 1.5 cm on average, 1.3±0.6 cm for the object alone (Table II). With fewer than 1,000 unique textures (at 10k images), performance degraded significantly. Removing distractors was the most damaging change: error rose to 7.2±4.5 cm with distractors and 7.4±5.3 cm under occlusion. Camera randomization helped a little (2.0 cm without it vs 1.3 cm with it). Image noise had a negligible effect on accuracy (1.4 vs 1.3 cm) but helped convergence. ImageNet pretraining was not essential — [Tobin 2017, §V ablations](https://arxiv.org/abs/1703.06907)

#### Peng et al. 2018 (ICRA), "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization" (Fetch arm puck pushing)
- (a) Table I, all **sampled once per episode**: link mass 0.25–4× (log-uniform); joint damping 0.2–20× (log); puck mass 0.1–0.4 kg (uniform); puck friction 0.1–5 (log); puck damping 0.01–0.2 Ns/m (uniform); table height 0.73–0.77 m (uniform); controller gains 0.5–2× (log); action-timestep rate λ 125–1000 s⁻¹ (uniform) — [Peng 2018, Table I](https://arxiv.org/abs/1710.06537)
- (d) Latency model: Δt = Δt₀ + Exp(λ) with Δt₀ = 0.04 s. Δt is **resampled every step** and λ is fixed per episode. Observation noise is Gaussian, zero mean, with σ = 5% of the running std of each feature, added every step. Exploration action noise σ = 0.01 rad — [Peng 2018, §V](https://arxiv.org/abs/1710.06537)
- (c) Sparse binary reward (0 inside the goal, −1 otherwise) with HER (k = 0.8). Real success means the puck ends within 0.07 m — [Peng 2018](https://arxiv.org/abs/1710.06537)
- (e) Real-robot LSTM ablation (Table III): all randomization 0.89±0.06 (28 trials); fixed action timestep 0.29±0.11; no observation noise 0.25±0.12; fixed link mass 0.64±0.10; fixed puck friction 0.48±0.10. The authors conclude that **latency and sensor noise matter most** — [Peng 2018, Table III](https://arxiv.org/abs/1710.06537)

#### Tan et al. 2018 (RSS), "Sim-to-Real: Learning Agile Locomotion for Quadruped Robots" (Minitaur, PyBullet)
- (b) SysID steps: disassemble the robot and measure link lengths; weigh each link and find its CoM, then write these into the URDF; estimate inertia assuming uniform density; measure motor friction. Actuator model: ideal DC motor (τ = K_t·I, back-EMF) driven by a PD servo. Because the linear torque–current relation failed on hardware (the robot sank or could not lift its feet), it was replaced by a **piecewise-linear torque–current curve**. Bullet's built-in position control was overdamped — [Tan 2018, §V-A](https://arxiv.org/abs/1804.10332)
- (d) Latency was measured with a one-step PWM spike and by timing the motor response: 3 ms for the microcontroller PD loop and 15–19 ms for the TX2 controller. It is simulated by keeping an observation history and **linearly interpolating at t − latency**. The control rate is about 150–200 Hz and varies, because the TX2 has no real-time OS — [Tan 2018, §III, §V-A](https://arxiv.org/abs/1804.10332)
- (a) Table I: mass 80–120%; motor friction 0–0.05 Nm; inertia 50–150%; motor strength 80–120%; control step 3–20 ms; latency 0–40 ms; battery voltage 14.0–16.8 V; contact friction 0.5–1.25; IMU bias ±0.05 rad; IMU noise std 0–0.05 rad — [Tan 2018, Table I](https://arxiv.org/abs/1804.10332)
- (c) r = (p_n − p_{n−1})·d − w·Δt·|τ·q̇|, with w = 0.008, described as untuned and robust over a wide range. Episodes end after 1000 steps or at base tilt > 0.5 rad — [Tan 2018, §IV-C](https://arxiv.org/abs/1804.10332)
- (e) **Without the actuator model or the latency model, policies failed on the robot, and random perturbations alone could not close the gap.** The 4-D IMU-only observation transferred better than the 12-D one (which adds motor angles). The 12-D policy was better in sim but worse on the robot. Sim vs real speeds: trot 0.50 vs 0.60 m/s, gallop 1.34 vs 1.18 m/s. Exact return numbers appear only in figures — [Tan 2018, §VI](https://arxiv.org/abs/1804.10332)

#### Hwangbo et al. 2019 (Science Robotics), "Learning agile and dynamic motor skills for legged robots" (ANYmal)
- (b) **Actuator network**: an MLP with 3×32 hidden units and softsign activation. Input is joint position error and joint velocity at t, t−0.01 s and t−0.02 s. Training data was over 1M samples at 400 Hz from sine foot trajectories (5–10 cm, 1–25 Hz) plus manual disturbances, collected in under 4 minutes. Validation RMS was 0.740 Nm vs 3.55 Nm for the ideal model. On test data from the locomotion policy it was 0.966 Nm vs 5.74 Nm. Rigid-body inertia comes from CAD, with an expected error of about 20% from cabling — [Hwangbo 2019, Methods](https://arxiv.org/abs/1901.08652)
- (a) 30 ANYmal model instances with randomized parameters: CoM U(±2 cm), link mass U(±15%), joint positions U(±2 cm) — [Hwangbo 2019](https://arxiv.org/abs/1901.08652)
- (d) Training-only uniform observation noise: joint velocity ±0.5 rad/s, base linear velocity ±0.08 m/s, base angular velocity ±0.16. No explicit latency term is used: delay is absorbed by the actuator network's 20 ms history — [Hwangbo 2019](https://arxiv.org/abs/1901.08652)
- (c) Locomotion cost terms (× Δt): angular velocity 6; linear velocity 10 and 4; torque 0.005; joint speed 0.03; foot clearance 0.1; foot slip 2.0; orientation 0.4; smoothness 0.5. **Reward curriculum**: k_c starts at 0.3 and updates as k ← k^0.997, scaling every cost except the task term. Policy runs at 200 Hz with kp = 50 and kd = 0.1 — [Hwangbo 2019](https://arxiv.org/abs/1901.08652)
- (e) With the ideal or the analytical actuator model, the robot **could not take a single step**. The analytical model was tuned for more than a week without success. The learned controller tracked at 0.143 m/s / 0.174 rad/s error, vs 0.231 / 0.278 for the prior model-based controller — [Hwangbo 2019](https://arxiv.org/abs/1901.08652)

#### Yu, Kumar, Turk 2019 (IROS), "Sim-to-Real Transfer for Biped Locomotion" (Darwin OP2, arXiv 1903.01390)
- (b) Pre-SysID fits a 14-dim parameter subset: per-joint-group scaling of a neural-network PD actuator, a torque limit, foot friction and torso CoM. The DR bounds are the min/max of the subset fits, **expanded by 10% on each side**, then sampled uniformly. Post-SysID uses Bayesian optimisation over the policy's latent μ with 25 hardware trials (under 15 minutes) — [Yu 2019](https://arxiv.org/abs/1903.01390)
- (d) Control frequency randomized per rollout in [25, 33] Hz. Orientation bias U(±0.3). Motor position noise N(0, 0.01). — [Yu 2019](https://arxiv.org/abs/1903.01390)
- (e) Their "Robust" plain-DR baseline over the same range was **more conservative and walked less far** than the system-ID-conditioned policy. The authors state that "domain randomization technique alone [is] insufficient" — [Yu 2019](https://arxiv.org/abs/1903.01390)

#### Xie et al. 2019 (CoRL), "Learning Locomotion Skills for Cassie: Iterative Design and Sim-to-Real"
- (b) MuJoCo Cassie model with reflected motor inertia and **empirically measured sensor/actuator noise and delay**. Sim runs the **same state-estimator and network interface code as the hardware** (EKF at 2 kHz), instead of giving the policy ground-truth state — [Xie 2019](https://arxiv.org/abs/1903.09537)
- (a) **No dynamics randomization.** Robustness was checked in sim with +20% pelvis mass and 50 N × 0.2 s pushes every 3 s. The authors attribute robustness to stochastic policies at a low control rate (30 ms / about 33 Hz) and note DR can make policies overly conservative — [Xie 2019](https://arxiv.org/abs/1903.09537)
- (d) Fixed Gaussian action noise with std exp(−2) ≈ 0.1 rad. The network link adds 1–2 ms on hardware — [Xie 2019](https://arxiv.org/abs/1903.09537)
- (e) Iterative fixes were reward edits plus distillation, not DR changes. Policy A (compressed) worked on hardware but the pelvis shook; a stable-pelvis reward fixed it (B). A joint-acceleration term gave softer contact (E). Foot lift was raised to 20 cm (F). An asymmetric step length appeared only on hardware and its cause was never found — [Xie 2019](https://arxiv.org/abs/1903.09537)

#### OpenAI et al. 2018, "Learning Dexterous In-Hand Manipulation" (Dactyl)
- (a) Table 1, **sampled per episode**: object dimensions U[0.95, 1.05]×; object and link masses U[0.5, 1.5]×; surface friction U[0.7, 1.3]×; joint damping **log-uniform** [0.3, 3.0]×; actuator P-gain log-uniform [0.75, 1.5]×; joint limits additive N(0, 0.15 rad); gravity vector additive N(0, 0.4 m/s²) per axis — [OpenAI 2018, Table 1](https://arxiv.org/abs/1808.00177)
- (d) Observation noise has **two parts: correlated (sampled once per episode, i.e. a bias) and uncorrelated (per step)**. Fingertip position 1 mm / 2 mm; object position 5 mm / 1 mm; object orientation 0.1 / 0.1 rad. Noise goes to policy inputs only, not to the value network. Action noise: 5% uncorrelated additive, 1.5% correlated additive, 1.5% uncorrelated multiplicative. **Action delay**: per episode, each actuator is delayed by one environment step (about 80 ms) with probability 0.5. Backlash slack is sampled per episode from N(calibrated value, 0.1). Timing: each env step is 10 MuJoCo steps of Δt = 8 ms + Exp(λ), with λ ~ U[1250, 10000] per episode — [OpenAI 2018, Tables 7–8, §4](https://arxiv.org/abs/1808.00177)
- (d) Unmodeled-effect forces: a per-episode probability p ~ log-U[0.1%, 10%] of applying a Gaussian force each step, with σ = 1 m/s² × object mass, **decaying by 0.99 per 80 ms** (a correlated, impulse-like disturbance). Marker dropout probability 0.2/s for 1 s — [OpenAI 2018](https://arxiv.org/abs/1808.00177)
- (c) r_t = d_t − d_{t+1} (orientation-angle progress); +5 on reaching a goal (within 0.4 rad); −20 on a drop — [OpenAI 2018](https://arxiv.org/abs/1808.00177)
- (e) Real consecutive rotations (Table 4, median): all randomizations 13; **none 0**; no observation noise 8.5; **no physics randomization 2; no unmodeled effects 2**. For vision: all 11.5, no observation noise 3.5. Training cost was about 3 simulated years without DR vs about 100 years with full DR (50 h vs 1.5 h wall-clock) — [OpenAI 2018, Table 4](https://arxiv.org/abs/1808.00177)

#### OpenAI et al. 2019, "Solving Rubik's Cube with a Robot Hand" (ADR)
- (a) ADR: each parameter is uniform on [φᴸ, φᴴ]. Each iteration pins one random parameter to a boundary and records performance there into a buffer of size m. When the buffer fills, its mean is compared against thresholds t_L < t_H: at or above t_H the range expands by Δ, at or below t_L it shrinks by Δ. **Initial ranges are the calibrated values (zero width).** Entropy is H = (1/d)Σ log(φᴴ − φᴸ), in nats per dimension — [OpenAI 2019, §5](https://arxiv.org/abs/1910.07113)
- (a) Categories: simulator physics; custom physics (action latency, motor backlash); "adversarial" random-network perturbations; observation noise (correlated per episode plus uncorrelated per step); vision — [OpenAI 2019](https://arxiv.org/abs/1910.07113)
- (e) Block task, real robot: manual DR mean 2.7±1.1 successes; ADR (Large) 13.3±3.6; ADR (XXL) 32.0±6.4. Rubik's cube: manual DR 1.8±0.4; ADR XXL 26.8±4.9 (60% half-scramble, 20% full scramble). **Real performance rose monotonically with ADR entropy** (manual about −0.569 npd vs XXL 0.479 npd). They hypothesise that an LSTM trained over an ADR distribution performs implicit meta-learning — [OpenAI 2019, Tables](https://arxiv.org/abs/1910.07113)

#### Lee et al. 2020 (Science Robotics), "Learning quadrupedal locomotion over challenging terrain" (ANYmal, teacher–student)
- (a) Friction drawn per terrain type from a Gaussian: hills N(0.7, 0.2), slippery hills N(0.3, 0.1), steps/stairs N(0.7, 0.2), clipped at ≥ 0.1. External disturbance forces are applied. Mass, CoM and motor are randomized and exposed to the teacher as privileged information, but **no ranges appear in the main text** — [Lee 2020](https://arxiv.org/abs/2010.11251)
- (b) Per-robot actuator network with input [joint error, velocity] at t, t−0.01, t−0.02. RaiSim simulator — [Lee 2020](https://arxiv.org/abs/2010.11251)
- (c) Teacher reward weights: linear velocity 0.05, angular velocity 0.05, base motion 0.04, foot clearance 0.01, body collision 0.02, target smoothness 0.025, torque 2e-5. Adaptive terrain curriculum: a particle filter keeps terrains with traversability in [0.5, 0.9] — [Lee 2020](https://arxiv.org/abs/2010.11251)
- (e) A TCN student with 100-step (2 s) history beat TCN-20 on steps. Under a 50 N × 5 s push, TCN-100 deviated 35.5% less than TCN-1. Training TCN-20 directly with RL (no privileged teacher) failed. A GRU student landed between TCN-20 and TCN-100. Uniform terrain sampling gave a lower reward plateau. A real 10 kg payload (22.7% of body mass, never simulated) still allowed 13.4 cm steps — [Lee 2020](https://arxiv.org/abs/2010.11251)

#### Kumar et al. 2021 (RSS), "RMA: Rapid Motor Adaptation" (A1, RaiSim)
- (a) Table I, train / test ranges: friction [0.05, 4.5] / [0.04, 6.0]; Kp [50, 60] / [45, 65]; Kd [0.4, 0.8] / [0.3, 0.9]; payload [0, 6] / [0, 7] kg; CoM [−0.15, 0.15] / [−0.18, 0.18] (units as printed; the cm label is doubtful **[UNVERIFIED]**); motor strength [0.90, 1.10] / [0.88, 1.22]; **re-sample probability per step 0.004 (train) / 0.01 (test)**. Parameters are re-drawn within an episode, not only at reset — [Kumar 2021, Table I](https://arxiv.org/abs/2107.04034)
- (c) Reward scales: forward min(vx, 0.35) ×20; lateral movement and rotation ×21; work 0.002; ground impact 0.02; smoothness 0.001; action magnitude 0.07; joint speed 0.002; orientation 1.5; z-acceleration 2.0; foot slip 0.8. **Penalty curriculum**: coefficients start small (k₀ = 0.03, per the supplement) and grow exponentially, so the robot does not learn to stand still. Mass, friction and motor-strength ranges are also widened linearly over training (a DR curriculum). Base policy runs at 100 Hz; the adaptation module at 10 Hz over 50 steps (0.5 s) of history. PD Kp = 55, Kd = 0.8 — [Kumar 2021, §III–IV](https://arxiv.org/abs/2107.04034)
- (e) Sim Table II success rate: Robust DR 62.4%; SysID 56.5%; RMA without adaptation 52.1%; **RMA 73.5%**; privileged expert 76.2%. No measured latency is reported — [Kumar 2021, Table II](https://arxiv.org/abs/2107.04034)

#### Rudin et al. 2022 (CoRL 2021), "Learning to Walk in Minutes Using Massively Parallel Deep RL" (legged_gym, Isaac Gym)
- (a) Paper: friction U[0.5, 1.25] per robot; base pushed by up to ±1 m/s in xy every 10 s. Observation noise is **uniform** within: joint position ±0.01 rad, joint velocity ±1.5 rad/s, base linear velocity ±0.01 m/s (the repo default is 0.1 **[conflict]**), angular velocity ±0.2 rad/s, projected gravity ±0.05, height scan ±0.1 m — [Rudin 2022, §IV](https://arxiv.org/abs/2109.11978)
- legged_gym repo defaults (`LeggedRobotCfg`): `friction_range=[0.5,1.25]`, `randomize_base_mass=False`, `added_mass_range=[-1,1]`, `push_interval_s=15`, `max_push_vel_xy=1.0`. Noise scales dof_pos 0.01, dof_vel 1.5, lin_vel 0.1, ang_vel 0.2, gravity 0.05, height 0.1, `noise_level=1.0`. dt 0.005 s × decimation 4 gives 50 Hz. action_scale 0.5. Reward scales: tracking_lin_vel 1.0, tracking_ang_vel 0.5, lin_vel_z −2.0, ang_vel_xy −0.05, torques −1e-5, dof_acc −2.5e-7, feet_air_time 1.0, collision −1, action_rate −0.01. No latency or delay randomization in the base config — [legged_gym legged_robot_config.py](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py)
- (c) Paper reward (× dt): linear velocity tracking 1, angular velocity tracking 0.5, vertical velocity −4, roll/pitch rate −0.05, joint motion −0.001, torque −2e-5, **action rate −0.25**, collisions −0.001, feet air time 2. These **differ from the repo defaults** (action_rate −0.01, lin_vel_z −2, air time 1) — [Rudin 2022](https://arxiv.org/abs/2109.11978)
- (b,d) ANYmal used an actuator network (LSTM on current measurements only; adding history did not help). The sim step could not go below 0.005 s because the actuator network became unstable. Game-inspired terrain curriculum: a robot is promoted when it walks past the terrain border and demoted when it covers less than half the commanded distance; robots that clear the top level are sent back to random levels — [Rudin 2022](https://arxiv.org/abs/2109.11978)
- (e) Timeout bootstrapping improved reward by about 10–20%. Best throughput was 2048–4096 robots with 100k–200k-sample batches; too many robots relative to the horizon hurt. On hardware, robustness fell at high speed, so the command cap was cut to 0.6 m/s — [Rudin 2022](https://arxiv.org/abs/2109.11978)
- Unitree's humanoid fork (`unitree_rl_gym`, G1): friction [0.1, 1.25]; added mass [−1, 3] kg (enabled); pushes every **5 s** up to 1.5 m/s; Kp hip 100 / knee 150 / ankle 40; Kd 2 / 4 / 2; action_scale 0.25; decimation 4. Rewards include base_height −10, orientation −1, alive 0.15, feet_swing_height −20, contact 0.18, dof_pos_limits −5 — [unitree_rl_gym g1_config.py](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/legged_gym/envs/g1/g1_config.py)

#### Miki et al. 2022 (Science Robotics), "Learning robust perceptive locomotion for quadrupedal robots in the wild"
- (a) Masses, initial state, external forces and torques are randomized per episode, and foot friction is occasionally set low. No numeric ranges appear in the main text — [Miki 2022](https://arxiv.org/abs/2201.08117)
- (d) **Structured exteroceptive noise model**: lateral shifts of scan points, height perturbations and outliers, applied at three scopes: per point and per foot (every step), and per episode (constant). A mapping condition is drawn at the start and middle of each trajectory: nominal 60%, offset 30% (drift, deformable ground), noisy 10% (near-total loss). Noise magnitude follows a curriculum, ramped linearly from epoch 20 to full strength at epoch 100. The nominal parameter vector ⟨0.004, 0.005, 0.01, 0.04, 0.03, 0.05, 0.1⟩ maps to components ambiguously **[UNVERIFIED]** — [Miki 2022](https://arxiv.org/abs/2201.08117)
- (c) r = 0.75(r_lv + r_av + r_lvo) + r_b + 0.003 r_fc + 0.1 r_co + 0.001 r_j + 0.08 r_jc + 0.003 r_s + 1e-6 r_τ + 0.003 r_slip. **DR and some reward terms are scaled by a curriculum c_{k+1} = c_k^0.98** — [Miki 2022](https://arxiv.org/abs/2201.08117)
- (e) The gated GRU belief encoder matched the teacher better than an ungated one (rough terrain MSE 0.690 vs 0.746 under small noise). This is an imitation metric, not a measured transfer effect — [Miki 2022](https://arxiv.org/abs/2201.08117)

#### Margolis & Agrawal 2022 (CoRL), "Walk These Ways" (Mini Cheetah / Go1)
- (a) Table 5: payload −1 to 3 kg; motor strength 90–110%; joint calibration ±0.02 rad; friction 0.40–1.00; restitution 0–1; gravity offset ±1.0 (the source prints the unit as "m", which is probably m/s² **[UNVERIFIED unit]**). Training is on flat ground only — [Margolis 2022, Table 5](https://arxiv.org/abs/2212.03238)
- (b,d) About 20 ms of latency, identified on hardware and modelled as a **constant action delay**. An actuator network maps PD error to torque. The authors prefer identifying invariant properties over wide randomization, to avoid conservative behaviour (**[UNVERIFIED wording]**: this point came from a summarised fetch). Kp = 20, Kd = 0.5, 50 Hz — [Margolis 2022](https://arxiv.org/abs/2212.03238)
- (c) r = r_task · exp(0.02 · r_aux). Task: xy velocity 0.02, yaw 0.01. Gait auxiliaries: swing force −0.08, stance velocity −0.08, height −0.2, pitch −0.1, Raibert footswing −0.2, swing height −0.6. Regularisers: z velocity −4e-4, roll/pitch rate −2e-5, foot slip −8e-4, collision −0.02, joint limits −0.2, torque −2e-5, joint velocity −2e-5, joint acceleration −5e-9, action smoothing (first and second order) −2e-3 each. **Grid-adaptive command curriculum**: vx starts at ±1 and grows to ±3 m/s, yaw rate from ±1 to ±5 rad/s, in 0.5-wide bins; a bin expands once its tracking reward passes 0.8 (xy) or 0.7 (yaw) — [Margolis 2022, Table 1](https://arxiv.org/abs/2212.03238)

#### Haarnoja et al. 2024 (Science Robotics), "Learning agile soccer skills for a bipedal robot with deep RL" (OP3)
- (b) Servo SysID fitted to sinusoidal motor tests: damping 1.084 Nm/(rad/s), armature 0.045 kg·m², friction 0.03, max torque 4.1 Nm, Kp 21.1. The real servos control voltage, so the model is approximate; **position control masked the mismatch, while current/torque control left a gap too large for zero-shot transfer** — [Haarnoja 2024](https://arxiv.org/abs/2304.13653)
- (a) DR, per episode: floor friction 0.5–1.0; joint offsets ±2.9°; IMU orientation up to 2°; IMU position up to 5 mm; torso mass up to 0.5 kg at a random location. Observation delay 10–50 ms. Impulse perturbations of 5–15 (units as printed) for 0.05–0.15 s every 1–3 s. The authors **deliberately randomized only a few axes because excess randomization yields conservative policies** — [Haarnoja 2024](https://arxiv.org/abs/2304.13653)
- (d) Policy at 40 Hz (25 ms), which often missed the deadline on hardware. An exponential action filter u_t = 0.8·u_{t−1} + 0.2·a_t. A 5-step history of proprioception — [Haarnoja 2024](https://arxiv.org/abs/2304.13653)
- (e) "Zero-shot transfer did not work for agents trained without domain randomization and perturbations": robots fell every 1–2 steps. Real vs sim gaps: walking 13% faster, turning 11% slower, get-up 28% longer, kick 5% slower; set-piece scoring 58% real vs 70% sim — [Haarnoja 2024](https://arxiv.org/abs/2304.13653)

#### Zhuang et al. 2023 (CoRL), "Robot Parkour Learning" (A1/Go1, Isaac Gym)
- (a) Table 9: added mass [1, 3] kg; CoM x [−0.05, 0.15], y ±0.1, z ±0.05 m; friction [0.5, 1.0]; motor strength [0.9, 1.1]; **depth latency [0.2, 0.26] s; proprioception latency [0.0375, 0.0475] s**; camera position ±1 cm (x); pitch 0–5°; FOV 85–88° — [Zhuang 2023, Table 9](https://arxiv.org/abs/2309.05665)
- (d) Depth runs at 10 Hz, asynchronously relative to the 50 Hz policy. Visual delay was calibrated on the real robot. Sim depth gets clipping, Gaussian noise and artefacts; real depth gets clipping, hole filling and spatial/temporal filters. Kp = 50, Kd = 1, with targets clipped so torque stays within ±25 Nm — [Zhuang 2023](https://arxiv.org/abs/2309.05665)
- (c) r = forward + energy + alive (2). **Soft-dynamics curriculum**: obstacles start penetrable with a penetration penalty; difficulty s ∈ [0, 1] moves in steps of 0.05 — [Zhuang 2023](https://arxiv.org/abs/2309.05665)

#### Siekmann et al. 2021 (ICRA), "Sim-to-Real Learning of All Common Bipedal Gaits via Periodic Reward Composition" (Cassie)
- (a) Table I, uniform, per rollout: joint damping [0.3, 4.0]×; joint mass [0.5, 1.5]×; ground friction [0.35, 1.1]; ground slope ±0.03 rad; encoder offset ±0.05 rad. Policy execution rate is also randomized (range not given). Policy at 40 Hz, PD at 2 kHz. No DR ablation is reported — [Siekmann 2021, Table I](https://arxiv.org/abs/2011.01387)

#### Radosavovic et al. 2024 (Science Robotics), "Real-world humanoid locomotion with RL" (Digit, transformer)
- (a,d) Dynamics, control-parameter, terrain and delay randomization plus observation noise are used, and a combination of dynamics, terrain and delay randomization is credited for transfer. **All ranges are in the supplement and were not retrieved.** Policy at 50 Hz, PD at 1 kHz. The policy also outputs PD gains for 8 leg joints. Closed kinematic chains are modelled with a stiff virtual spring — [Radosavovic 2024](https://arxiv.org/abs/2303.03381)
- (e) The transformer beat MLP, TCN and LSTM; longer context helped (16 steps used) — [Radosavovic 2024](https://arxiv.org/abs/2303.03381)

#### Fu et al. 2024, "HumanPlus" (Unitree H1)
- (a) Table 2, uniform: base payload ±3 kg; end-effector payload 0–0.5 kg; base CoM ±0.1; motor strength [0.8, 1.1]; friction [0.3, 0.9]; **control delay [0.02, 0.04] s**. Policy at 50 Hz, PD at 1 kHz, context of 8 steps. No ablation of the low-level controller — [HumanPlus 2024, Table 2](https://arxiv.org/abs/2406.10454)

#### Controller stack summary (policy output → low-level controller, rates, sim fidelity)
"controller not specified" means the part was not stated in the text I read (the supplements were not retrieved). Each row's sources are the paper links in its fact sheet above.

| Paper | Policy output | Low-level controller (gains) | Policy rate / inner-loop rate | Same model in sim? | Key DR / latency |
|---|---|---|---|---|---|
| Tobin 2017 | Object position estimate (perception only, no learned controller) | controller not specified | n/a | n/a | visual DR only |
| Peng 2018 (Fetch) | Joint-space targets (inferred from 0.01 rad action noise **[UNVERIFIED]**) | Robot joint controller, gains randomized 0.5–2× (log); Kp/Kd values not specified | about 25 Hz (Δt₀ = 0.04 s + Exp(λ)) / inner rate not specified | Gains randomized rather than identified | Δt per step, mass 0.25–4× |
| Tan 2018 (Minitaur) | Motor angle targets (leg pose) | Microcontroller PD servo → PWM; sim uses DC motor + back-EMF + **piecewise-linear torque–current**; Kp/Kd values not extracted | 150–200 Hz (variable) / servo loop has 3 ms latency | Yes: identified motor model + measured latency (interpolated at t − latency) | latency 0–40 ms, control step 3–20 ms |
| Hwangbo 2019 (ANYmal) | Joint position targets | ANYdrive SEA modelled by **actuator net** (3×32 softsign, 20 ms history); PD kp 50, kd 0.1 with zero target velocity | 200 Hz (locomotion), 100 Hz (recovery) / actuator data logged at 400 Hz | Yes: the actuator net replaces the analytical model | CoM ±2 cm, mass ±15%, no explicit delay |
| Yu 2019 (Darwin OP2) | Joint position targets | Servo position control modelled as a **NN-PD actuator**; hardware PID (PWM-level) gains not reproduced in sim | 33 Hz (randomized 25–33 Hz in training) | Partially: NN-PD fitted by SysID, real PID not used | SysID bounds ±10% |
| Xie 2019 (Cassie) | PD targets | Embedded joint PD at 2 kHz (gains not reported); EKF at 2 kHz | about 33 Hz (30 ms) / 2 kHz | Yes: same estimator and network-interface code, measured noise/delay | no DR |
| OpenAI 2018 (Dactyl) | Joint position targets for the Shadow hand (discretized, from memory **[UNVERIFIED]**) | Hand position controller with randomized P-gain log-U[0.75, 1.5]; inner rate not specified | about 12.5 Hz (80 ms env step) / 8 ms sim step + Exp(λ) | Partly: backlash model with calibrated slack, gains randomized | 1-step delay with probability 0.5, correlated + white noise |
| OpenAI 2019 (Rubik's) | controller not specified (in the portion read; presumed the same hand stack **[UNVERIFIED]**) | controller not specified | controller not specified | ADR from calibrated values | ADR incl. latency, backlash |
| Lee 2020 (ANYmal) | Foot-trajectory modulation over a 1.25 Hz periodic leg pattern, resolved to joint targets (PMTG-style, from memory **[UNVERIFIED]**) | ANYdrive via **actuator net** (input at t, t−0.01, t−0.02) | "400 Hz" per extraction, conflicting with the 50 Hz history **[UNVERIFIED]** / controller rate not specified | Yes: per-robot actuator net | friction per terrain N(·), ranges not in main text |
| Kumar 2021 RMA (A1) | Joint position targets | Joint PD; real Kp 55, Kd 0.8; sim randomizes Kp [50, 60], Kd [0.4, 0.8]; PD rate not specified | 100 Hz base / 10 Hz adaptation module | Gains randomized around real values; no latency model | 0.004/step re-sample |
| Rudin 2022 legged_gym | Joint position targets (action_scale 0.5, `control_type='P'`) | Sim PD per robot config (example stiffness 10–15 N·m/rad, damping 1.0–1.5); ANYmal uses an LSTM actuator net instead | 50 Hz / PD at 200 Hz (dt 0.005 × decimation 4) | Unitree-style robots use an ideal PD; ANYmal uses the actuator net | friction, pushes, no delay |
| unitree_rl_gym G1 | Joint position targets (action_scale 0.25) | PD Kp hip 100 / knee 150 / ankle 40, Kd 2 / 4 / 2 | 50 Hz assumed from the inherited dt 0.005 × 4 **[UNVERIFIED]** | Ideal PD, no actuator model in config | friction 0.1–1.25, mass −1–3 kg |
| Miki 2022 (ANYmal) | controller not specified in text read (likely the Lee 2020 structure **[UNVERIFIED]**) | Actuator net (Hwangbo); gains not specified | controller not specified | Yes: actuator net | structured height-map noise |
| Margolis 2022 WTW (Go1) | Joint position targets (0 = nominal pose) | PD Kp 20, Kd 0.5 plus an actuator net; PD rate not specified | 50 Hz | Yes: actuator net + constant about 20 ms delay | narrow DR, flat ground |
| Haarnoja 2024 (OP3) | Joint position targets, filtered u = 0.8u⁻ + 0.2a | Servo position control; fitted Kp 21.1, damping 1.084, armature 0.045, friction 0.03, τ_max 4.1 Nm; servo rate not specified | 40 Hz (deadline often missed) | Yes: SysID'd position-actuator model. **Torque/current control did not transfer** | obs delay 10–50 ms, few DR axes |
| Zhuang 2023 Parkour (A1) | Joint position targets (scale 0.4 hip, 0.6 thigh/knee) | PD Kp 50, Kd 1; targets clipped so torque stays within ±25 Nm | 50 Hz / PD about 1 kHz | Ideal PD; latencies randomized | proprio delay 37.5–47.5 ms, depth 200–260 ms |
| Siekmann 2021 (Cassie) | PD targets | Joint PD at 2 kHz; gains not specified | 40 Hz (rate randomized, range not given) / 2 kHz | Same structure in sim; damping randomized | damping 0.3–4×, encoder offset |
| Radosavovic 2024 (Digit) | PD setpoints for 16 joints **plus learned PD gains for 8 leg joints**; toes on fixed default PD | Joint PD at 1 kHz | 50 Hz / 1 kHz | Delay and dynamics randomized; values in the supplement | controller gains not specified numerically |
| HumanPlus 2024 (H1) | Joint position setpoints | PD at 1 kHz; gains not reported | 50 Hz / 1 kHz | Ideal PD + control delay | delay 20–40 ms |

Pattern: all of the legged and humanoid works output **position targets into a high-rate PD or servo loop** (200 Hz–2 kHz) with the policy at 25–200 Hz. Fidelity of that inner loop is handled either by identification (actuator net or a fitted servo model) or by randomizing gains around measured values. The one documented torque/current-control attempt (Haarnoja) failed to transfer zero-shot.

### Inferences
- Across papers, the strongest real-world ablation evidence favours **(i) actuator or low-level controller fidelity and (ii) latency/timestep randomization** over wide inertial DR. For a PX4 velocity loop, the analogue of the actuator network is the identified closed-loop velocity response (PX4 PID plus ESC/motor lag). The analogue of the timestep randomization is the offboard command-to-response delay.
- Typical modern DR widths are friction about 0.5–1.25, mass ±10–20% or a small added mass, motor strength ±10%, and pushes. The width is set to cover measured uncertainty, not to be maximal. ADR is the only line of work showing that "wider is monotonically better", and it used a recurrent policy with about 100 simulated years of data.
- Haarnoja's finding that position control masked the actuator mismatch while torque control did not suggests a velocity-command interface has the same effect. Wrapping the policy in a high-bandwidth inner loop reduces sensitivity to motor-model errors.

### Gaps
- Haarnoja 2024 reward weights (Table S3), the delay distribution, and whether delay is per-episode or per-step: supplement not retrieved.
- Radosavovic 2024 DR, noise and delay table: supplement only, not retrieved.
- OpenAI 2019 ADR hyperparameters (p_b, m, t_L, t_H, Δ) and final per-parameter ranges: in the appendix beyond the portion read.
- Lee 2020 numeric mass, CoM and motor ranges and noise magnitudes: in the supplement, not retrieved. The extraction also reported a "400 Hz policy", which conflicts with a 2 s / 100-step history (50 Hz) **[UNVERIFIED]**.
- Booster/Unitree humanoid sim2real papers (2024–2025) beyond the `unitree_rl_gym` defaults were not covered, due to the tool-call budget.

## Q2. Which works scaled DR progressively, or argued that too-wide DR hurts (conservatism)?

### Takeaway
Progressive DR is common: ADR (automatic, from zero width), RMA (linear range curriculum plus penalty curriculum), Miki (an exponential curriculum on DR, rewards and noise), and Hwangbo (a penalty curriculum). Explicit warnings about conservatism come from Tan 2018, Yu 2019, Xie 2019, Margolis 2022 and Haarnoja 2024. ADR is the counterexample: wider ranges kept helping real performance.

### Cited Findings
- Tan 2018 chose DR ranges "carefully to prevent learning overly conservative running gaits". Randomized policies had lower mean return and lower variance, a robustness/optimality trade-off, and the authors advise using DR only when necessary — [Tan 2018](https://arxiv.org/abs/1804.10332)
- Yu 2019: a plain-DR "Robust" policy was more conservative and walked less far than a SysID-conditioned policy over the same range — [Yu 2019](https://arxiv.org/abs/1903.01390)
- Xie 2019: no DR at all, with transfer obtained via accurate measured noise/delay and the shared estimator code; DR is noted as making policies overly conservative — [Xie 2019](https://arxiv.org/abs/1903.09537)
- Haarnoja 2024: randomized only a few axes because too much randomization gives a conservative policy — [Haarnoja 2024](https://arxiv.org/abs/2304.13653)
- Margolis 2022: identifies latency and actuator dynamics rather than randomizing widely (**[UNVERIFIED wording]**) — [Margolis 2022](https://arxiv.org/abs/2212.03238)
- RMA: DR difficulty (mass, friction, motor strength) rises linearly, and penalty coefficients start at k₀ = 0.03 and grow exponentially to avoid a "stand still" optimum. Plain DR ("Robust") reached 62.4% vs 73.5% for adaptive RMA, i.e. wide DR without adaptation leaves performance on the table — [Kumar 2021](https://arxiv.org/abs/2107.04034)
- Hwangbo 2019: cost curriculum k ← k^0.997 starting at 0.3 — [Hwangbo 2019](https://arxiv.org/abs/1901.08652). Miki 2022: c_{k+1} = c_k^0.98 on DR and reward terms, plus linear noise ramping — [Miki 2022](https://arxiv.org/abs/2201.08117)
- ADR: starts at the calibrated values and expands a range only when boundary performance ≥ t_H. Real Rubik's success rose from 1.8 (manual DR) to 26.8 (ADR XXL) — [OpenAI 2019](https://arxiv.org/abs/1910.07113)

### Inferences
- A reactive feedback policy (MLP, no memory) pays for wide DR with conservatism. Memory or adaptation (LSTM in ADR, an RMA encoder, a TCN or GRU student) lets wider DR pay off. This supports keeping L0's DR moderate and letting a history-based residual (GRU) absorb variation.
- Ramping penalty terms is a recurring fix for "learn to stand still or hover" local optima.

### Gaps
- No paper found that sweeps DR width systematically on real hardware for a fixed architecture, apart from ADR's entropy-vs-performance curve.

## Q3. How was actuator latency and command delay modelled?

### Takeaway
There are four patterns. Measure-and-fix (Tan: interpolate at t − latency; Margolis: constant about 20 ms; Xie: measured). Randomize a delay range around the measurement (Tan 0–40 ms; Haarnoja 10–50 ms observation delay; Zhuang proprioception 37.5–47.5 ms and depth 200–260 ms; HumanPlus 20–40 ms). Stochastic per-step timestep (Peng and OpenAI: Δt₀ + Exp(λ), λ per episode). Implicit through actuator-network history (Hwangbo, Lee).

### Cited Findings
- Tan 2018: latency measured with a PWM spike (3 ms servo, 15–19 ms TX2), randomized 0–40 ms with the control step at 3–20 ms, implemented by interpolating the observation history at t − latency — [Tan 2018](https://arxiv.org/abs/1804.10332)
- Peng 2018: Δt ~ 0.04 s + Exp(λ), λ ~ U[125, 1000] per episode, resampled per step. Removing it dropped real success from 0.89 to 0.29 — [Peng 2018](https://arxiv.org/abs/1710.06537)
- OpenAI 2018: each actuator is delayed by one 80 ms step with probability 0.5 per episode, and the sim step is 8 ms + Exp(λ) with λ ~ U[1250, 10000] — [OpenAI 2018](https://arxiv.org/abs/1808.00177)
- Haarnoja 2024: 10–50 ms observation delay, a 40 Hz policy, and a 0.8/0.2 exponential action filter — [Haarnoja 2024](https://arxiv.org/abs/2304.13653)
- Zhuang 2023: separate latency ranges per modality (proprioception 0.0375–0.0475 s, depth 0.2–0.26 s), with the depth delay calibrated on the robot — [Zhuang 2023](https://arxiv.org/abs/2309.05665)
- HumanPlus: control delay U[0.02, 0.04] s — [HumanPlus 2024](https://arxiv.org/abs/2406.10454)
- Hwangbo 2019 / Lee 2020: no explicit delay; the actuator network's 3-sample (20 ms) history absorbs it — [Hwangbo 2019](https://arxiv.org/abs/1901.08652), [Lee 2020](https://arxiv.org/abs/2010.11251)
- legged_gym base config has no delay randomization by default — [legged_gym config](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py)
- Yu 2019 and Siekmann 2021 randomize the policy rate itself (25–33 Hz; range not given for Siekmann) — [Yu 2019](https://arxiv.org/abs/1903.01390), [Siekmann 2021](https://arxiv.org/abs/2011.01387)

### Inferences
- Common practice is to measure the end-to-end delay on hardware first, then randomize roughly 0.5×–2× around it. Separate delays per sensor stream are worth modelling when rates differ (camera vs IMU), which parallels the GPS/EKF vs offboard-command paths on PX4.

### Gaps
- Whether Haarnoja's delay is sampled per episode or per step was not confirmed.

## Q4. Per-step vs per-episode randomization, and correlated noise

### Takeaway
Physics parameters are almost always per episode. Exceptions: RMA re-samples within an episode with probability 0.004 per step, and Peng and OpenAI resample the timestep per step. OpenAI 2018 is the clearest correlated-noise design: per-episode bias plus per-step white noise for both observations and actions, and exponentially decaying random forces. Miki applies elevation noise at per-step and per-episode scopes. No paper in this set used OU or Gauss–Markov noise explicitly.

### Cited Findings
- Peng 2018: dynamics are sampled per episode, Δt per step, and observation noise per step (σ = 5% of the feature's running std) — [Peng 2018](https://arxiv.org/abs/1710.06537)
- OpenAI 2018: physics per episode; observation and action noise split into correlated (per episode) and uncorrelated (per step) parts; random object forces decay by 0.99 per 80 ms, with the force probability set per episode — [OpenAI 2018](https://arxiv.org/abs/1808.00177)
- RMA: environment parameters are re-sampled with probability 0.004 per step in training (0.01 at test), so dynamics change mid-episode and the adaptation module must track them — [Kumar 2021](https://arxiv.org/abs/2107.04034)
- Miki 2022: height-scan noise at per-point and per-foot (each step) and per-episode scopes, with a mapping-failure mode re-drawn at the start and middle of each trajectory — [Miki 2022](https://arxiv.org/abs/2201.08117)
- Haarnoja 2024: friction, offsets, IMU pose and torso mass per episode, plus random impulses every 1–3 s — [Haarnoja 2024](https://arxiv.org/abs/2304.13653)
- legged_gym: friction per robot at creation and uniform observation noise each step — [Rudin 2022](https://arxiv.org/abs/2109.11978)

### Inferences
- The OpenAI split into per-episode bias plus per-step white noise maps directly onto drone sensors: GPS or EKF position bias plus jitter, and wind as a mean plus gust. An exponentially decaying random force is a cheap alternative to an OU process for gusts.
- RMA-style mid-episode re-sampling is the legged analogue of time-varying wind. It is only useful when the policy has memory.

### Gaps
- No landmark legged or manipulation paper found here uses OU or Gauss–Markov noise by name. Such usage may exist in aerial-robotics papers, which are outside this scope.
