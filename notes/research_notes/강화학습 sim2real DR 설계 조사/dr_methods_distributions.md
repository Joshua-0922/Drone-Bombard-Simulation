---
date: 2026-10-16
tags: [research, sim2real, domain-randomization, wind, latency]
status: draft
type: research
---

# DR distribution and scale design, sim-param inference, and drone disturbance models (2015–2026)

Legend: **[V]** = checked against a source fetched in this session. **[UNVERIFIED]** = from prior knowledge. It is linked to the canonical paper but I did not re-read the page this session, so check it before quoting in the paper.

## Q1. Which approaches beat fixed hand-tuned uniform DR in sim-to-real, and by how much?

### Takeaway
The cleanest head-to-head numbers come from DORAEMON (ICLR 2024). On a real Panda push task it gets 60% success, against 0% for a wide fixed uniform DR, 26.7% for AutoDR (ADR) and 46.7% for LSDR. Too-wide fixed DR failed completely there, which is direct evidence of the conservatism/over-regularisation failure. Methods that use real data (SimOpt, BayesSim, DROPO, NPDR, BayRn) improve on hand-tuned ranges in their own papers. The 2022 offline-vs-online benchmark gives no single winner: online methods are limited by the current policy, and offline methods fail when open-loop replay diverges.

### Cited Findings
**(a) Adaptive / automatic DR scale**
- DORAEMON (Tiboni, Klink, Peters, Tommasi, D'Eramo, Chalvatzaki; ICLR 2024) maximises the entropy of the DR distribution under a constraint that in-distribution success probability stays ≥ α. It needs no real data and widens the range only while the policy still succeeds [V] — [arXiv 2311.01885](https://arxiv.org/abs/2311.01885)
- DORAEMON PandaPush (17-D randomisation; success = box within 3 cm). Sim2Real over 30 rollouts [V] — [arXiv html v2, Table 1](https://arxiv.org/html/2311.01885v2):
  - No-DR: 13.33%, 8.08±6.28 cm
  - Fixed-DR: 0%, 23.26±2.25 cm
  - LSDR: 46.67%, 7.38±8.69 cm
  - AutoDR: 26.67%, 4.17±2.07 cm
  - DORAEMON: 60%, 2.68±1.01 cm
  - Sim2Sim over 10,000 rollouts: 15.06 / 0.02 / 37.77 / 30.45 / 66.57%.
- The DORAEMON authors say Fixed-DR on a wide uniform range was "unable to learn any meaningful behavior". The abstract says excessive randomisation "notoriously leads to overly conservative policies". The introduction says it leads to "over-regularized policies that are no longer able to reach optimal performance" [V] — [arXiv html v2](https://arxiv.org/html/2311.01885v2)
- DORAEMON implementation details [V] — [arXiv html v2](https://arxiv.org/html/2311.01885v2):
  - Independent univariate **Beta** distributions per parameter, initialised at Beta(100,100); a Gaussian variant also works (App. A.4).
  - Success threshold α = 0.5 in all sim2sim runs.
  - KL trust region ε ∈ {0.005, 0.01, 0.05}, tuned per environment.
  - Reported to beat AutoDR "in all environments" (sim2sim figures only).
- OpenAI ADR (Rubik's cube, arXiv 1910.07113): environments get progressively harder, and ADR-trained policies and vision estimators "transfer much better" than manual DR. Memory-augmented (LSTM) policies show "signs of emergent meta-learning" at test time [V, abstract] — [arXiv 1910.07113](https://arxiv.org/abs/1910.07113)
  - Further details [UNVERIFIED]: real cube solved 60% of the time on average and 20% on the hardest scrambles. ADR raises a parameter's boundary when performance at that boundary exceeds t_H and lowers it below t_L.
- Active Domain Randomization (Mehta, Diaz, Golemo, Pal, Paull; CoRL 2019) [UNVERIFIED]:
  - Learns which randomisation instances are informative, using SVPG particles and a discriminator reward that measures how distinguishable a randomised rollout is from the reference environment.
  - Reported lower variance and better generalisation than uniform DR on LunarLander, Pusher and ErgoReacher, plus a real ErgoReacher.
- LSDR ("Learning Domain Randomization Distributions for Training Robust Locomotion Policies", Mozifian, Gamboa Higuera, Meger, Dudek; IROS 2020): learns the DR distribution to maximise worst-case/robust performance and is DORAEMON's LSDR baseline [UNVERIFIED attribution] — numbers above [V] via [DORAEMON Table 1](https://arxiv.org/html/2311.01885v2)

**(b) Sim-parameter inference from real data**
- DROPO (Tiboni, Arndt, Kyrki; Robotics and Autonomous Systems 166, 2023, art. 104432) [V] — [arXiv 2201.08434](https://www.arxiv.org/abs/2201.08434), [Aalto record](https://aaltodoc.aalto.fi/items/c66c7c07-c9df-44e7-96df-1b6579d78112/full):
  - Fits a DR distribution (mean and variance) to a small offline set of real trajectories by maximising the likelihood of the real transitions under simulation.
  - Recovers parameter distributions, and also finds a distribution that compensates for an unmodelled phenomenon.
  - Succeeded in two zero-shot sim2real transfers and beat prior methods.
- Online vs. offline adaptive DR benchmark (Tiboni, Arndt, Averta, Kyrki, Tommasi; arXiv 2206.14661, 2022) compared SimOpt, BayRn, DROID and DROPO [V] — [arXiv 2206.14661](https://arxiv.org/abs/2206.14661):
  - Online methods "are limited by the quality of the currently learned policy".
  - Offline methods may fail when real trajectories are replayed open-loop in simulation.
  - Numbers are not in the abstract.
- A 2025 paper (ICLR 2026) frames offline DR as maximum-likelihood estimation over a distribution of simulator parameters and notes that ODR theory was largely unexplored [V, search snippet] — [arXiv 2506.10133](https://arxiv.org/pdf/2506.10133v2)
- SimOpt (Chebotar et al., ICRA 2019): alternates policy training with updates to a Gaussian DR distribution that minimise the discrepancy between real and simulated observation trajectories. Real tasks were swing-peg-in-hole and drawer opening, using a few real rollouts per iteration [UNVERIFIED]
- BayesSim (Ramos, Possas, Fox; RSS 2019): likelihood-free inference (mixture-density network) of a posterior over simulator parameters from real trajectory summaries. DR is then sampled from that posterior [UNVERIFIED]
- NPDR (Muratore et al., "Neural Posterior Domain Randomization", CoRL 2021) and BayRn (Muratore et al., ICRA 2021) [UNVERIFIED]:
  - NPDR does SBI with normalising-flow posteriors.
  - BayRn uses Bayesian optimisation over DR distribution parameters, with real-world return as the objective.
- Swift (Kaufmann et al., Nature 620, 2023) [V] — [PMC10468397](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/):
  - Real-data residuals: about 50 s of flight from 3 rollouts (800–1,000 dynamics samples).
  - Perception residuals are stochastic (nine 1-D GPs). Dynamics residuals are "largely deterministic" (kNN, k=5, predicting residual acceleration).
  - Policy fine-tuned 2×10⁷ steps in the augmented simulator, after 1×10⁸ initial steps.
  - Under simulated domain shift, zero-shot, plain-DR and time-optimal baselines completed **no laps**. Swift completed the track in every setting.
  - A second fine-tune iteration changed gate-centre distance by only 0.09±0.08 m.
  - Race record: won 5/9, 4/7 and 6/9 against three champions.
- Eschmann, Albani, Loianno (arXiv 2404.07837) [V] — [arXiv 2404.07837](https://arxiv.org/html/2404.07837v2):
  - Data-driven SysID (MAP) from about 1 minute of normal flight logs, with no thrust stand. It jointly identifies inertia, thrust curve, torque coefficient and **motor time constant**.
  - An RL policy trained on the identified 3.35 kg quad flew in gusty wind up to 8 m/s at speeds up to 5 m/s.

**(c) Adversarial / worst case**
- RARL (Pinto, Davidson, Sukthankar, Gupta; ICML 2017) [V, abstract] — [arXiv 1703.02702](https://arxiv.org/abs/1703.02702):
  - Zero-sum game against a learned adversary that applies disturbance forces, motivated by H∞.
  - Robust to train/test differences in friction and mass, and outperforms the baseline even without the adversary.
  - MuJoCo only (InvertedPendulum, HalfCheetah, Swimmer, Hopper, Walker2d). The abstract gives no numbers.

### Inferences
- Several sources agree on one point: a fixed range set too wide is dangerous. Adaptive methods win mainly because they **avoid over-wide ranges**, not because they are wider. Our single global scale knob behaves like a crude ADR or curriculum. A DORAEMON-style rule is a principled replacement: grow each parameter's range independently while success@0.5 m stays ≥ α.
- For us, a DROPO/SimOpt-style fit to real F450 logs (motor τ, mass, drag, latency) is the natural next step once flight logs exist. The Eschmann MAP method needs only about 1 minute of normal flight, which matches our planned "manual flight first" step.
- Swift's split is a useful template for the GRU: deterministic residuals → regressors; stochastic residuals → sampled noise models.

### Gaps
- The full ADR numbers (manual DR vs. ADR on real block reorientation) and the ADR thresholds were not extracted. The PDF was not text-readable through the fetch tool.
- I did not fetch exact numbers for SimOpt, BayesSim, NPDR, BayRn, Active DR, Self-paced DR (Klink et al.), GDR or Real2Sim2Real.
- I found no study comparing DR methods on **quadrotor wind** specifically.

## Q2. Theory and evidence on history-dependent (recurrent) policies under DR

### Takeaway
Chen, Hu, Jin, Li, Wang ("Understanding Domain Randomization for Sim-to-real Transfer", arXiv 2110.03239; I believe ICLR 2022, unverified) model DR as a latent-MDP / POMDP problem. The DR-optimal policy is **history-dependent**, and the sim-to-real gap is sublinear in horizon H only because memory lets the policy implicitly identify the environment. ADR also reported emergent meta-learning in LSTM policies. This directly supports a recurrent (GRU) residual that infers wind from history.

### Cited Findings
- Setup: the simulator is a set 𝒰 of MDPs, and π*_DR maximises expected value under a uniform ν over 𝒰. π*_DR is history-dependent [V] — [ar5iv 2110.03239](https://ar5iv.labs.arxiv.org/html/2110.03239). Assumptions:
  - A1: communicating MDPs with diameter ≤ D. Without it the gap can be Ω(H) (Prop. 1).
  - A2: δ-separation between models.
  - A3: L-Lipschitz smoothness near the real MDP.
- Bounds [V] — [ar5iv 2110.03239](https://ar5iv.labs.arxiv.org/html/2110.03239):
  - Thm 1, finite separated class of size M: Gap = O(D·M³·log(MH)·log²(SMH/δ)/δ⁴), which is poly-logarithmic in H.
  - Thm 2, finite class without separation: O(D√(M³H log(MH))).
  - Thm 3 lower bound: Ω(D√(MH)) for any history-dependent policy.
  - Thm 4, infinite class: O(D√(d_e H log(H·𝒩))/ν(𝒞_{ℳ*,ε}) + Lε). Here ν(𝒞_{ℳ*,ε}) is the probability that the DR distribution samples a model within ε of the real one.
  - Prop. 2: the gap can be Ω(H) for large classes without smoothness.
- The paper says memory and randomisation are both critical, and that without memory the policy cannot identify the environment and cannot reach sublinear worst-case loss [V] — [ar5iv 2110.03239](https://ar5iv.labs.arxiv.org/html/2110.03239)
- ADR: memory-augmented policies trained under ADR show emergent meta-learning at test time [V] — [arXiv 1910.07113](https://arxiv.org/abs/1910.07113)
- Peng et al. 2018 ("Sim-to-Real Transfer of Robotic Control with Dynamics Randomization", ICRA): LSTM policies trained with dynamics randomisation transferred a pushing task better than feed-forward policies [UNVERIFIED]

### Inferences
- In Thm 4, the gap scales with 1/ν(𝒞_{ℳ*,ε}). If the DR distribution puts little mass near the real system, the bound degrades. This is a theoretical argument **against needlessly wide or flat DR** and **for centring DR on SysID estimates** (the DROPO/BayesSim style).
- Our finding that the GRU is coupled to the controller fits this view. The GRU implicitly identifies the closed-loop "environment", and the controller is part of that environment. Swapping PX4 PID changes ℳ*, so the GRU must be re-fit or the controller must be inside 𝒰.
- The separation assumption (A2) suggests DR parameters that do not change observations cannot be identified from history. Randomising them only adds conservatism.

### Gaps
- I did not find an explicit formal lower bound for memoryless policies in the text read. The claim is stated in prose.

## Q3. Distribution families for positive scale parameters, time-correlated disturbances, and drone disturbance numbers

### Takeaway
I found no controlled study showing that one family (uniform, Gaussian, log-uniform, Beta) transfers better than the others in general. Common practice:
- uniform multiplicative factors for mass and friction (about 0.5–1.5×)
- log-uniform for parameters spanning orders of magnitude
- Beta in DORAEMON, because it is bounded and its entropy is tractable

Arguments from first principles and theory favour **bounded, positive, SysID-centred** distributions: log-normal/log-uniform or Beta on [lo, hi], rather than a Gaussian clamped at 0.

For wind near the ground, the MIL Dryden low-altitude model gives about 37 m (h = 5 m) and 67 m (h = 10 m) for the longitudinal length scale. At a 3 m/s mean wind that is a correlation time of about 12–22 s, an order of magnitude above a 1–1.3 s fall.

### Cited Findings
**Distribution families**
- Practice in recent papers [V] — [TRANSIC 2405.10315](https://arxiv.org/pdf/2405.10315), [arXiv 2609.13761](https://arxiv.org/pdf/2609.13761), [claru guide](https://claru.ai/guides/how-to-setup-domain-randomization-pipeline):
  - TRANSIC uses U(0.5,1.5) for mass scale and friction.
  - An in-hand manipulation paper uses friction U[0.5,2.0] and **LogU[1,4]** for inertia multipliers.
  - Community and vendor guides say uniform is the default and log-uniform suits parameters spanning orders of magnitude. These are not controlled comparisons.
- DORAEMON uses independent Beta per parameter: bounded support, and entropy can be computed in closed form [V] — [arXiv html v2](https://arxiv.org/html/2311.01885v2)
- DROPO and SimOpt fit Gaussian means and variances to data, so the resulting DR is SysID-centred [V for DROPO] — [arXiv 2201.08434](https://www.arxiv.org/abs/2201.08434)

**Drone wind (MIL-F-8785C / MIL-HDBK-1797 low altitude, h < 1000 ft, h in ft)** [V] — [MathWorks Dryden block](https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html); [arXiv 1908.00372](https://arxiv.org/pdf/1908.00372)
- Length scales: Lu = Lv = h/(0.177+0.000823h)^1.2 and Lw = h (8785C). In 1797, 2Lw = h and Lu = 2Lv.
- Intensities: σw = 0.1·W20 and σu/σw = σv/σw = 1/(0.177+0.000823h)^0.4.
- W20 (wind at 20 ft): light 15 kt, moderate 30 kt, severe 45 kt.
- Forming filters: Hu(s) = σu√(2Lu/(πV))/(1+(Lu/V)s). Hv and Hw are second-order with a lead term (1+√3(L/V)s), so they are **not** first-order Gauss–Markov. Hu is first-order.
- My evaluation of these formulas (my calculation):

  | h | Lu | σu/σw | Lw (8785C) |
  |---|----|-------|------------|
  | 5 m | ≈ 120 ft ≈ 37 m | ≈ 1.94 | 5 m |
  | 10 m | ≈ 221 ft ≈ 67 m | ≈ 1.89 | 10 m |

**Surface-layer statistics**
- Ratios of short-period to hourly wind speed at 10 m over open terrain: about **1.52 for 3 s gusts** and 1.1 for 600 s [V, lecture table] — [SNU OCW wind properties](https://ocw.snu.ac.kr/sites/default/files/NOTE/1~2%EC%A3%BC%EC%B0%A8_Part%20I%20Wind%20Properties_v14.pdf)
- σu/u* ≈ 2.2 (Beljaars 1987), as used in a hurricane-records study [V, search snippet] — [AMS confex pdf](https://ams.confex.com/ams/pdfpapers/36949.pdf)
- Open-terrain reference roughness is z0 = 0.03 m at 10 m; the lecture gives an open-terrain range of 0.015–0.15 m [V, snippet] — [AMS confex pdf](https://ams.confex.com/ams/pdfpapers/36949.pdf)
- The gust factor depends on surface roughness and stability, with stability mattering more over rough surfaces [V, snippet] — [DTU Orbit, wind-gust parametrisations](https://orbit.dtu.dk/en/publications/windgust-parametrizations-at-heights-relevant-for-wind-energy-a-s/)
- Turbulence intensity I_v(z) = k_I/(c0·ln(z/z0)) with k_I = 1 (Eurocode EN 1991-1-4) [UNVERIFIED]. With z0 = 0.03 m this gives I ≈ 0.17 at 10 m and ≈ 0.20 at 5 m.

**Motor lag**
- Crazyflie 2.1 (27 g): T_m = 0.072 s identified from flight data. The manufacturer step response gives about 0.073 s and Förster reported 0.065 s.
- 3.35 kg quad: "much smaller" T_m; no number is given [V] — [arXiv 2404.07837](https://arxiv.org/html/2404.07837v2)

**Latency**
- A PX4 + Micro XRCE-DDS + ROS 2 latency study compared XRCE-DDS with MAVROS/MAVLink and reports XRCE-DDS is better [V, snippet only] — [Sensors, PMC10674650](https://pmc.ncbi.nlm.nih.gov/articles/PMC10674650/). The numbers were not retrievable (captcha).
- End-to-end ROS 2 latency depends strongly on the DDS middleware [V, snippet] — [Kronauer et al. 2021](https://www.barkhauseninstitut.org/fileadmin/user_upload/Publikationen/2021/2021_Kronauer_Latency.pdf)
- A probabilistic ROS 2 DDS latency model over lossy wireless links exists [V, snippet] — [arXiv 2508.10413](https://arxiv.org/pdf/2508.10413v1)

**Residual approaches**
- Swift's stochastic/deterministic residual split [V] — [PMC10468397](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/)
- TossingBot (Zeng et al., RSS 2019 / T-RO 2020): a learned residual on top of a ballistic physics release-velocity estimate, reporting about 85% throwing accuracy into boxes and over 500 mean picks per hour [UNVERIFIED]
- Residual RL (Johannink et al. ICRA 2019; Silver et al. 2018) [UNVERIFIED]

### Inferences
**Latency.** N(0.22, 0.05) clamped at 0 is the wrong shape:
- Real latency is strictly positive and has a hard floor (transport plus servo actuation time).
- It is right-skewed, with occasional long stalls from scheduling and DDS.
- Clamping a Gaussian at 0 puts a point mass at an impossible value (P(<0) ≈ 5e-6 here, so it is harmless in practice, but the shape is still wrong in the right tail).

Proposal: a **shifted log-normal or shifted gamma**, d = d_min + LogN(μ, σ), fitted to measured release delays. Optionally mix in a small-probability heavy-tail component (e.g. 2–5% from a wider log-normal) to cover stalls. This is an inference; I found no paper measuring PX4/uXRCE release-latency histograms.

**Positive scale parameters (mass, gains, drag, motor τ).** Sample multiplicatively:
- log-uniform or log-normal around the SysID value: symmetric in ratio, so ×0.5 and ×2 are equally likely
- or Beta on [lo, hi] if a DORAEMON-style entropy rule will tune it

This avoids negative or zero values and the asymmetry of additive Gaussians. By Chen Thm 4, mass near the true value (ν(𝒞_{ℳ*,ε})) matters, so heavy-tailed families should be reserved for genuinely rare events (stalls, strong gusts), not used as the bulk shape.

**Gust correlation vs. fall time** (supports our earlier finding):
- Fall time from 5–8 m is about 1.0–1.3 s, with √(2h/g) = 1.01 s at 5 m.
- Dryden Lu/U at h ≈ 5–10 m with U = 3 m/s gives about 12–22 s, using Taylor's frozen-turbulence assumption with mean wind instead of airspeed. Dryden is ill-posed as airspeed V → 0 near hover, so this is approximate.
- Under the MIL model, the longitudinal gust seen during the fall is therefore **nearly constant**. That favours a history-based estimator (GRU) that tracks the current wind.
- The vertical component has a much shorter scale (Lw ≈ h, so about 2–3 s at 3 m/s; half that under 1797) and is second-order. If our Gauss–Markov τ is much shorter than about 5 s for the horizontal components, it is likely pessimistic relative to MIL, and it lies in the "residual hurts" regime we found.
- Recommendation: draw τ_u from a range centred on Lu/U (log-uniform, for example over [0.3, 3]× nominal). Give the vertical component its own Dryden second-order filter, instead of a single isotropic τ.

**Intensity.** Scale σu with the mean wind (σu ≈ I·U, I ≈ 0.15–0.25 at 5–10 m), not as an independent Gaussian. Then a 3 s gust factor of about 1.5 emerges naturally. This links the "capped Gaussian mean" and the gust amplitude through one physical parameter.

**Too-wide wind DR.** DORAEMON shows that a fixed range set too wide can destroy learning entirely. Prefer a success-constrained expansion over manually raising one global scale.

### Gaps
- No primary measured data on PX4/uXRCE-DDS/ROS 2 latency distribution shape (percentiles, tails) was retrieved. Measure on the F450 + RPi 5 stack instead.
- I found no controlled empirical comparison of uniform, Gaussian, log-uniform, Beta and heavy-tailed DR families for sim2real transfer. The family recommendations above are inferences.
- MIL-HDBK-1797 itself was not read directly; the formulas come from MathWorks and arXiv 1908.00372. MIL-HDBK-1797 does not define the low-altitude model for hover or near-zero airspeed.
- No motor τ number was found for 2212/2216-class 920 KV motors on a 450 frame. A rough guess of 0.02–0.05 s for that size is UNVERIFIED; identify it from logs with the Eschmann MAP method.
- I found no measured 5–10 m gust correlation times for the actual site (SNU field). Logging an anemometer at about 1–10 Hz would give site-specific τ and I directly.
- Not covered: Self-paced DR, GDR, DROID, Real2Sim2Real, and the OU vs. random-walk vs. per-episode-constant comparison (no direct source found).

## Controller stacks

Checked 2026-10-16 against arXiv HTML / ar5iv / PMC text. "controller not specified" = the text I read does not name the low-level controller. **[UNVERIFIED]** = not confirmed this session (PDF not machine-readable, or not fetched).

| paper | policy output | low-level controller & rate | modelled in sim? | source |
|---|---|---|---|---|
| DORAEMON (ICLR 2024), real Franka PandaPush | Normalised **joint accelerations** in [−1,1] per joint, at 50 Hz | Each output is turned into a 20 ms joint trajectory, tracked by a **PD + feed-forward** term at 1000 Hz. Gains not given. Real-robot controller (libfranka or vendor impedance) **not specified**. | Partly. The PD + FF tracking is described in the task spec, and "we reproduce the real setup in simulation with MuJoCo". The paper never says the real robot runs the identical controller. | [arXiv 2311.01885 v2 App.](https://arxiv.org/html/2311.01885v2) |
| Swift (Nature 2023), racing quad | **CTBR**: mass-normalised collective thrust + body rates | **Betaflight** PID rate controller + BLHeli32 ESC. Rate not given in the text read; inference takes 8 ms on CPU. Policy rate [UNVERIFIED]. | **Yes, explicitly**: "an accurate model of both the low-level controller and the motor speed controller". Motor commands predicted with <1% error. | [PMC10468397](https://pmc.ncbi.nlm.nih.gov/articles/PMC10468397/) |
| Eschmann, Albani, Loianno (arXiv 2404.07837), 3.35 kg quad | **Motor RPM setpoints**, direct end-to-end (not CTBR) | Flight firmware and rate **controller not specified** (PX4 is mentioned only as background and for the log format). | Motors are modelled as first-order lag with the identified time constant. No attitude or rate controller sits between the policy and the motors in sim. | [arXiv 2404.07837 v2](https://arxiv.org/html/2404.07837v2) |
| ADR / Rubik's cube (OpenAI 2019), Shadow hand | **Discretised relative joint-position changes** (categorical, 20 coords), at 12.5 Hz (80 ms steps) | **Controller not specified** in the Rubik paper itself (inherits the Dactyl stack, below). Torque limits hand-tuned. | Partly. MuJoCo position actuators with randomised `actuator_gainprm` and force range. Action latency, action delay, noise and motor backlash are added as custom physics. Tendons partially modelled. | [ar5iv 1910.07113](https://ar5iv.labs.arxiv.org/html/1910.07113) |
| Dactyl (OpenAI 2018, arXiv 1808.00177), the predecessor hardware | 20-D relative desired joint angles, 11 bins each, every 80 ms | **C++ real-time PD controller** on a separate machine. The text gives conflicting rates: "roughly 1 kHz" (§2.1) vs "every 5 ms" (App. B.4). | **Not identical.** Sim uses a proportional law, torque = P·(target − current). No D term is stated; joint damping is randomised instead. | [ar5iv 1808.00177](https://ar5iv.labs.arxiv.org/html/1808.00177) |
| SimOpt (Chebotar et al., ICRA 2019): Franka drawer opening, ABB YuMi swing-peg | Drawer: **7 joint-velocity commands** + gripper. YuMi: **not specified**. | **Controller not specified**, and no rate is given. | Implicitly. NVIDIA Flex with randomised 7-D joint compliance, damping and **action scaling**. Action scaling was among the parameters that moved most, which suggests the controller mismatch was absorbed by DR. | [ar5iv 1810.05687](https://ar5iv.labs.arxiv.org/html/1810.05687) |
| BayesSim (Ramos, Possas, Fox; RSS 2019) | n/a | **No real-robot experiment** in the arXiv version. "Real" data are simulator rollouts with ground-truth parameters (Fetch Push/Slide, Hopper, classic control). | n/a | [ar5iv 1906.01728](https://ar5iv.labs.arxiv.org/html/1906.01728) |
| DROPO (Tiboni et al., RAS 2023): Kuka LWR4+ hockey puck, Franka Panda push | Hockey: **latent action of a VAE trajectory generator**. Panda push: action space not specified. | Hockey: a position controller; real gains **not specified**. Push: "a position controller", details **not specified**. Real data at 10 ms per transition (~100 Hz). | Hockey: **yes**. The simulated position-controller **gains are randomised DR parameters** in MuJoCo. Push: sim controller not described (only box mass, friction and CoM randomised). | [ar5iv 2201.08434](https://ar5iv.labs.arxiv.org/html/2201.08434) |
| Active DR (Mehta et al., CoRL 2019), real Poppy Ergo Jr (ErgoReacher) | **Not specified** (actions clipped to [−1,1]) | Servo controller (Dynamixel) and rate **not specified** | Partly. Bullet sim with randomised joint damping and max torque ("gains and maximum torques for each joint"). How the gains enter the controller is not stated. | [ar5iv 1904.04762](https://ar5iv.labs.arxiv.org/html/1904.04762) |
| BayRn (Muratore et al., ICRA 2021), real Quanser Furuta pendulum | **Motor voltage** at 100 Hz, so no low-level controller is used | None. Voltage is applied directly (a separate controller only resets the device between rollouts). | **Yes**. The voltage→torque motor model (motor constant, back-EMF) is in sim, apparently at nominal values. | [ar5iv 2003.02471](https://ar5iv.labs.arxiv.org/html/2003.02471) |
| BayRn (same paper), Barrett WAM ball-in-a-cup | **Joint-angle deltas** at 500 Hz | The WAM's **feed-forward controller**; gains not specified | Not specified | [ar5iv 2003.02471](https://ar5iv.labs.arxiv.org/html/2003.02471) |
| NPDR (Muratore et al., CoRL 2021) | [UNVERIFIED] Likely the Quanser Furuta pendulum with a motor-voltage action, as in BayRn. The PDF was not text-readable. | [UNVERIFIED] | [UNVERIFIED] | [PMLR v164](https://proceedings.mlr.press/v164/muratore22a.html) |
| Peng et al. 2018 (ICRA), Fetch puck pushing | **Relative joint-position targets** (7-D) at a 0.04 s nominal step | A "position controller" (structure not described) | **Yes, with gains randomised 0.5–2×**. The action interval is randomised as Δt₀ + Exp(λ) to model controller latency. | [ar5iv 1710.06537](https://ar5iv.labs.arxiv.org/html/1710.06537) |
| TRANSIC (Jiang et al., CoRL 2024), Franka | Student and residual: **joint positions**. Teacher: end-effector pose via **OSC**, sim only. | Real: **Deoxys joint-position controller at 1000 Hz**. Sim: 60 Hz control, 2 substeps. | Yes for the student. The authors switched from OSC to joint position specifically because "a low-level action space such as the joint position ensures consistent deployment in simulation and real hardware". | [arXiv 2405.10315](https://arxiv.org/html/2405.10315) |
| RARL (Pinto et al., ICML 2017) | n/a | **No real robot** (MuJoCo only) | n/a | [arXiv 1703.02702](https://arxiv.org/abs/1703.02702) |
| LSDR (Mozifian et al., IROS 2020) | [UNVERIFIED] | [UNVERIFIED] Not fetched; it may be sim-only locomotion. | [UNVERIFIED] | — |
| TossingBot (Zeng et al.) | [UNVERIFIED] Grasp and release-velocity residual on a UR5 | [UNVERIFIED] | [UNVERIFIED] | — |
| Chen et al. (arXiv 2110.03239) | n/a: theory only | n/a | n/a | [ar5iv 2110.03239](https://ar5iv.labs.arxiv.org/html/2110.03239) |

### Inferences (controller stacks)
- Only **Swift** (CTBR → Betaflight, controller modelled to <1% error), **TRANSIC** (same joint position controller in sim and real), **BayRn Furuta** (no controller; motor model in sim) and **Peng 2018 / DROPO hockey** (controller gains randomised) treat the controller as part of the simulated plant. Everyone else either leaves it unstated or absorbs it with "action scaling" or gain DR (SimOpt, Active DR, ADR).
- TRANSIC's switch from OSC to joint-position control "for consistent deployment" and Dactyl's PD-real vs P-sim mismatch are both examples of our Rule 47 (the GRU is coupled to the controller). Our A plan (train against an exact PX4 velocity PID port) is the Swift pattern applied one level higher: velocity setpoints → PX4 instead of CTBR → Betaflight.
- When the controller cannot be ported exactly, randomise its gains around the identified values (0.5–2× as in Peng 2018), like any other positive scale parameter.
