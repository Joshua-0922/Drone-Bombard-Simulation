"""Pure-torch unit tests for drone_bombard/math_utils.py.

Runs WITHOUT isaaclab installed — this repo's dev box has no Isaac Sim
(GPU driver 535 < the 580 Isaac Sim 5.1.0 requires), so this file must be
importable/runnable with just torch + pytest. To avoid triggering
``drone_bombard/__init__.py`` (which imports isaaclab/isaaclab_rl at import
time via gym.register), math_utils.py is loaded directly by file path
rather than via ``from drone_bombard.math_utils import ...``.

Run: pytest isaac_lab/tests/test_math.py -v
"""

import importlib.util
import math
import os
import types

import pytest
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
_MATH_UTILS_PATH = os.path.join(_HERE, "..", "drone_bombard", "math_utils.py")

_spec = importlib.util.spec_from_file_location("math_utils", _MATH_UTILS_PATH)
mu = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mu)


# =====================================================================
# Action pipeline: rate limit + LPF
# =====================================================================

def test_rate_limit_clamp():
    prev = torch.tensor([[0.0, 0.0, 0.0, 0.0]])
    action = torch.tensor([[1.0, -1.0, 0.05, 0.3]])
    out = mu.rate_limit_action(action, prev, 0.2)
    torch.testing.assert_close(out, torch.tensor([[0.2, -0.2, 0.05, 0.2]]))


def test_lpf_discrete_step_response_matches_1_minus_0_6_pow_k():
    """alpha=0.4: v_filt_k = 1 - 0.6^k for a constant unit step from rest."""
    cmd = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    v_filt = torch.zeros(1, 4)
    snap = torch.zeros(1, dtype=torch.bool)
    expected = [1 - 0.6 ** k for k in range(1, 6)]
    for k, exp in enumerate(expected, start=1):
        v_filt = mu.lpf_step(cmd, v_filt, 0.4, snap)
        assert v_filt[0, 0].item() == pytest.approx(exp, abs=1e-6), f"tick {k}"
    # y(100ms) = y_2 (2 ticks @ 20Hz = 100ms) = 0.64
    assert expected[1] == pytest.approx(0.64, abs=1e-9)


def test_lpf_state_continuity_across_policy_step_boundary():
    """2 policy steps = 4 LPF ticks (2 ticks/policy step @ decimation=10,
    lpf_tick_period_steps=5, 100Hz physics -> 20Hz LPF). Held constant
    command across both steps must give y_4 = 1 - 0.6^4 = 0.8704 exactly,
    i.e. the filter state persists across the policy-step boundary rather
    than resetting."""
    cmd = torch.tensor([[2.0, 0.0, 0.0, 0.0]])  # arbitrary nonzero command
    v_filt = torch.zeros(1, 4)
    snap = torch.zeros(1, dtype=torch.bool)
    # policy step 1: 2 ticks
    for _ in range(2):
        v_filt = mu.lpf_step(cmd, v_filt, 0.4, snap)
    # policy step 2: 2 more ticks (no reset in between -> continuity)
    for _ in range(2):
        v_filt = mu.lpf_step(cmd, v_filt, 0.4, snap)
    expected = 2.0 * (1 - 0.6 ** 4)
    assert v_filt[0, 0].item() == pytest.approx(expected, abs=1e-6)
    assert expected == pytest.approx(2.0 * 0.8704, abs=1e-9)


def test_lpf_per_env_reset_isolation_and_snap():
    """Resetting env 0 must not perturb env 1's filter state, and env 0
    must snap to the new command on its next post-reset tick (no ramp)."""
    cmd = torch.tensor([[1.0, 0, 0, 0], [1.0, 0, 0, 0]])
    v_filt = torch.zeros(2, 4)
    snap = torch.zeros(2, dtype=torch.bool)

    for _ in range(3):
        v_filt = mu.lpf_step(cmd, v_filt, 0.4, snap)
    env1_state_before_reset = v_filt[1].clone()
    assert v_filt[0, 0].item() == pytest.approx(1 - 0.6 ** 3, abs=1e-6)

    # Reset env 0 only: new command, snap flag set for env 0.
    cmd_after_reset = torch.tensor([[5.0, 0, 0, 0], [1.0, 0, 0, 0]])
    snap_after_reset = torch.tensor([True, False])
    v_filt = mu.lpf_step(cmd_after_reset, v_filt, 0.4, snap_after_reset)

    assert v_filt[0, 0].item() == pytest.approx(5.0, abs=1e-6), "env 0 must snap to new command"
    # env 1 (never reset) must continue its own uninterrupted EMA trajectory:
    # one more normal tick on top of its pre-reset state = 1-0.6^4, NOT
    # frozen at env1_state_before_reset (that would mean env0's reset call
    # incorrectly skipped env1's update too).
    assert v_filt[1, 0].item() == pytest.approx(1 - 0.6 ** 4, abs=1e-6)
    assert v_filt[1, 0].item() != pytest.approx(env1_state_before_reset[0].item(), abs=1e-6)


# =====================================================================
# Vision: pinhole projection + hold buffer
# =====================================================================

def _fx():
    h_fov = 1.047
    cx = 320.0
    return cx / math.tan(h_fov / 2.0)


def test_pinhole_straight_down_centered_when_target_directly_below():
    pos = torch.tensor([[0.0, 0.0, 10.0]])
    quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]])  # identity, level
    target_xy = torch.tensor([[0.0, 0.0]])
    u, v, visible = mu.project_target_pinhole(pos, quat, target_xy, _fx(), _fx(), 320.0, 240.0, 640, 480, 0.1, 100.0)
    assert u.item() == pytest.approx(320.0, abs=1e-4)
    assert v.item() == pytest.approx(240.0, abs=1e-4)
    assert bool(visible.item())


def test_pinhole_world_y_offset_shifts_u_by_55_4px():
    """Level drone at (0,0,10), target offset 1m along world +Y -> maps to
    body +Y (x_c) under the chosen camera-axis convention -> Delta u =
    fx*(1/10) px (fx = cx/tan(h_fov/2) ~= 554.4, derived by the same
    formula the env uses in DroneBombardVisionCfg.__post_init__ — computed
    here via _fx() rather than a hand-rounded literal, since h_fov=1.047 is
    not exactly 60 degrees so tan(h_fov/2) != tan(30deg) exactly)."""
    pos = torch.tensor([[0.0, 0.0, 10.0]])
    quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    target_xy = torch.tensor([[0.0, 1.0]])
    fx = _fx()
    u, v, visible = mu.project_target_pinhole(pos, quat, target_xy, fx, fx, 320.0, 240.0, 640, 480, 0.1, 100.0)
    assert fx == pytest.approx(554.4, abs=0.5)  # sanity band around the known ~60deg-FOV value
    assert (u.item() - 320.0) == pytest.approx(fx / 10.0, abs=1e-3)
    assert v.item() == pytest.approx(240.0, abs=1e-4)
    assert bool(visible.item())


def test_pinhole_world_x_offset_shifts_v_not_u():
    pos = torch.tensor([[0.0, 0.0, 10.0]])
    quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    target_xy = torch.tensor([[1.0, 0.0]])
    fx = _fx()
    u, v, visible = mu.project_target_pinhole(pos, quat, target_xy, fx, fx, 320.0, 240.0, 640, 480, 0.1, 100.0)
    assert u.item() == pytest.approx(320.0, abs=1e-4)
    assert (v.item() - 240.0) == pytest.approx(fx / 10.0, abs=1e-3)


def test_pinhole_not_visible_when_target_behind_or_out_of_frame():
    # Drone below the target (z_c would be negative -> not visible)
    pos = torch.tensor([[0.0, 0.0, -5.0]])
    quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    target_xy = torch.tensor([[0.0, 0.0]])
    fx = _fx()
    _, _, visible = mu.project_target_pinhole(pos, quat, target_xy, fx, fx, 320.0, 240.0, 640, 480, 0.1, 100.0)
    assert not bool(visible.item())

    # Target far to the side, out of the 640-wide frame at 10m altitude
    pos2 = torch.tensor([[0.0, 0.0, 10.0]])
    target_xy2 = torch.tensor([[0.0, 50.0]])
    _, _, visible2 = mu.project_target_pinhole(pos2, quat, target_xy2, fx, fx, 320.0, 240.0, 640, 480, 0.1, 100.0)
    assert not bool(visible2.item())


def test_quat_apply_inverse_pure_yaw_90deg():
    """90 deg yaw (about world/body +Z) should rotate world +X into body -Y
    (standard right-handed rotation, w,x,y,z convention)."""
    half = math.pi / 4.0
    q = torch.tensor([[math.cos(half), 0.0, 0.0, math.sin(half)]])
    v = torch.tensor([[1.0, 0.0, 0.0]])
    out = mu.quat_apply_inverse_pure(q, v)
    torch.testing.assert_close(out, torch.tensor([[0.0, -1.0, 0.0]]), atol=1e-6, rtol=1e-6)


def test_hold_buffer_countdown_and_expiry():
    hold_frames = 10
    held = torch.zeros(1, 3)
    remaining = torch.zeros(1, dtype=torch.long)

    fresh = torch.tensor([[100.0, 200.0, 0.9]])
    detected = torch.tensor([True])
    u, v, c, held, remaining = mu.hold_buffer_update(detected, fresh, held, remaining, hold_frames)
    assert (u.item(), v.item(), c.item()) == (100.0, 200.0, pytest.approx(0.9))
    assert remaining.item() == hold_frames

    # Miss for hold_frames-1 ticks: still shows the held detection each time,
    # countdown decrements by exactly 1 per miss.
    miss = torch.zeros(1, 3)
    not_detected = torch.tensor([False])
    for expected_remaining in range(hold_frames - 1, -1, -1):
        u, v, c, held, remaining = mu.hold_buffer_update(not_detected, miss, held, remaining, hold_frames)
        assert remaining.item() == expected_remaining
        assert (u.item(), v.item()) == (100.0, 200.0), "must keep showing held detection while remaining > 0"

    # One more miss after remaining hit 0 -> zeros (expired).
    u, v, c, held, remaining = mu.hold_buffer_update(not_detected, miss, held, remaining, hold_frames)
    assert (u.item(), v.item(), c.item()) == (0.0, 0.0, 0.0)


def test_hold_buffer_fresh_detection_resets_countdown():
    hold_frames = 10
    held = torch.zeros(1, 3)
    remaining = torch.zeros(1, dtype=torch.long)
    detected = torch.tensor([True])
    fresh1 = torch.tensor([[10.0, 20.0, 0.8]])
    _, _, _, held, remaining = mu.hold_buffer_update(detected, fresh1, held, remaining, hold_frames)
    # miss a few ticks, countdown decays
    miss = torch.zeros(1, 3)
    not_detected = torch.tensor([False])
    for _ in range(4):
        _, _, _, held, remaining = mu.hold_buffer_update(not_detected, miss, held, remaining, hold_frames)
    assert remaining.item() == hold_frames - 4
    # a fresh detection resets the countdown to hold_frames
    fresh2 = torch.tensor([[30.0, 40.0, 0.95]])
    u, v, c, held, remaining = mu.hold_buffer_update(detected, fresh2, held, remaining, hold_frames)
    assert remaining.item() == hold_frames
    assert (u.item(), v.item()) == (30.0, 40.0)


# =====================================================================
# Ballistic / CCIP hook parity (Phase-1 bit-identical to hook-free reference)
# =====================================================================

def test_ballistic_impact_matches_drop_free_closed_form():
    pos_xy = torch.tensor([[5.0, 0.0]])
    vel_xy = torch.tensor([[2.0, 0.0]])
    altitude = torch.tensor([10.0])
    release_delay, gravity = 0.1, 9.81
    zero_drag = torch.zeros(1)
    zero_wind = torch.zeros(1, 2)

    impact = mu.ballistic_impact(pos_xy, vel_xy, altitude, release_delay, gravity, zero_drag, zero_wind)

    t_fall_ref = math.sqrt(2.0 * 10.0 / 9.81)
    impact_ref = pos_xy + vel_xy * (t_fall_ref + release_delay)
    torch.testing.assert_close(impact, impact_ref, atol=1e-6, rtol=1e-6)


def test_ballistic_impact_responds_to_release_delay_and_gravity_params():
    """The function must actually use the passed-in release_delay/gravity
    (not silently fall back to a hardcoded default) — this is the
    functional guarantee behind moving these into DropCfg instead of
    inline literals: changing the argument changes the output."""
    pos_xy = torch.tensor([[0.0, 0.0]])
    vel_xy = torch.tensor([[1.0, 0.0]])
    altitude = torch.tensor([10.0])
    zero_drag = torch.zeros(1)
    zero_wind = torch.zeros(1, 2)

    impact_a = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.1, 9.81, zero_drag, zero_wind)
    impact_b = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.9, 9.81, zero_drag, zero_wind)
    impact_c = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.1, 3.0, zero_drag, zero_wind)

    assert not torch.allclose(impact_a, impact_b)
    assert not torch.allclose(impact_a, impact_c)


def test_ballistic_impact_wind_is_true_zero_noop_in_phase1():
    pos_xy = torch.tensor([[0.0, 0.0]])
    vel_xy = torch.tensor([[1.0, 0.0]])
    altitude = torch.tensor([10.0])
    zero_drag = torch.zeros(1)
    zero_wind = torch.zeros(1, 2)
    nonzero_wind = torch.tensor([[3.0, 0.0]])

    impact_no_wind = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.1, 9.81, zero_drag, zero_wind)
    impact_with_wind = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.1, 9.81, zero_drag, nonzero_wind)
    assert not torch.allclose(impact_no_wind, impact_with_wind), "nonzero wind must change the result"

    t_fall_ref = math.sqrt(2.0 * 10.0 / 9.81)
    impact_ref_zero_wind = pos_xy + vel_xy * (t_fall_ref + 0.1)
    torch.testing.assert_close(impact_no_wind, impact_ref_zero_wind, atol=1e-6, rtol=1e-6)


def test_ballistic_impact_batched_no_broadcast_bug():
    """Regression: with N != 1 and N != 2 envs, the time factor must
    broadcast against the [N,2] horizontal vectors. A bare [N] (missing
    unsqueeze) throws 'size of tensor a (2) must match tensor b (N)'."""
    for n in (3, 5, 255, 256):
        pos_xy = torch.randn(n, 2)
        vel_xy = torch.randn(n, 2)
        altitude = torch.rand(n) * 10 + 1
        zero_drag = torch.zeros(n)
        zero_wind = torch.zeros(n, 2)
        impact = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.1, 9.81, zero_drag, zero_wind)
        assert impact.shape == (n, 2)
        # per-row equals the scalar closed form
        for i in range(min(n, 4)):
            t = math.sqrt(2.0 * altitude[i].item() / 9.81)
            expected = pos_xy[i] + vel_xy[i] * (t + 0.1)
            torch.testing.assert_close(impact[i], expected, atol=1e-5, rtol=1e-5)


def test_ccip_residual_is_zero_stub_no_params():
    obs = torch.randn(8, 14)
    residual = mu.ccip_residual(obs)
    assert residual.shape == (8, 2)
    assert torch.equal(residual, torch.zeros(8, 2))
    assert isinstance(mu.ccip_residual, types.FunctionType), "must be a pure function, not an nn.Module"


# =====================================================================
# Phased-curriculum helpers: nominal CCIP, residual, GM target, rewards
# =====================================================================

def test_time_to_fall_closed_form_and_clamp():
    alt = torch.tensor([10.0, 0.0, -3.0])
    t = mu.time_to_fall(alt, 9.81)
    assert t[0].item() == pytest.approx(math.sqrt(2.0 * 10.0 / 9.81), abs=1e-6)
    assert t[1].item() == pytest.approx(0.0, abs=1e-9)
    assert t[2].item() == pytest.approx(0.0, abs=1e-9), "below-ground altitude clamps, no NaN"


def test_predict_impact_nominal_matches_drag_free_ballistic():
    pos_xy = torch.tensor([[5.0, -2.0], [0.0, 0.0]])
    vel_xy = torch.tensor([[2.0, 1.0], [-1.0, 3.0]])
    altitude = torch.tensor([10.0, 6.0])
    pred = mu.predict_impact_nominal(pos_xy, vel_xy, altitude, 0.1, 9.81)
    ref = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.1, 9.81, torch.zeros(2), torch.zeros(2, 2))
    torch.testing.assert_close(pred, ref, atol=1e-6, rtol=1e-6)


def test_apply_ccip_residual_adds_scaled_delta():
    pred = torch.tensor([[5.0, 5.0], [0.0, 0.0]])
    residual = torch.tensor([[1.0, -0.5], [0.0, 1.0]])
    out = mu.apply_ccip_residual(pred, residual, scale=2.0)
    torch.testing.assert_close(out, torch.tensor([[7.0, 4.0], [0.0, 2.0]]), atol=1e-6, rtol=1e-6)
    # zero residual is a true no-op
    out0 = mu.apply_ccip_residual(pred, torch.zeros_like(residual), scale=2.0)
    torch.testing.assert_close(out0, pred, atol=1e-6, rtol=1e-6)


def test_step_target_velocity_mean_reversion_and_noise():
    vel = torch.tensor([[1.0, -2.0]])
    theta, sigma, dt = 0.5, 0.0, 0.1
    # zero noise -> pure mean reversion: v <- (1 - theta*dt)*v
    out = mu.step_target_velocity(vel, theta, sigma, dt, torch.zeros_like(vel))
    torch.testing.assert_close(out, (1.0 - theta * dt) * vel, atol=1e-6, rtol=1e-6)
    # nonzero noise adds sigma*sqrt(dt)*noise
    noise = torch.tensor([[1.0, 1.0]])
    out2 = mu.step_target_velocity(vel, theta, 2.0, dt, noise)
    expected = (1.0 - theta * dt) * vel + 2.0 * math.sqrt(dt) * noise
    torch.testing.assert_close(out2, expected, atol=1e-6, rtol=1e-6)


def test_step_target_velocity_zero_theta_zero_sigma_is_constant():
    vel = torch.tensor([[3.0, -1.0], [0.0, 2.0]])
    out = mu.step_target_velocity(vel, 0.0, 0.0, 0.1, torch.randn_like(vel))
    torch.testing.assert_close(out, vel, atol=1e-6, rtol=1e-6)


def test_impact_terminal_reward_monotone_decreasing_and_saturation():
    err = torch.tensor([0.0, 1.0, 5.0])
    r = mu.impact_terminal_reward(err, w_impact=100.0, reward_scale=1.0)
    assert r[0].item() == pytest.approx(100.0, abs=1e-6), "perfect hit saturates to w_impact"
    assert r[1].item() == pytest.approx(100.0 * math.exp(-1.0), abs=1e-5)
    assert r[0].item() > r[1].item() > r[2].item(), "monotonically decreasing in miss distance"


def test_lead_prediction_reward_perfect_and_decay():
    pred = torch.tensor([[10.0, 0.0], [10.0, 0.0]])
    actual = torch.tensor([[10.0, 0.0], [13.0, 4.0]])  # second: 5m off
    r = mu.lead_prediction_reward(pred, actual, w_lead=10.0, reward_scale=5.0)
    assert r[0].item() == pytest.approx(10.0, abs=1e-6), "perfect lead saturates to w_lead"
    assert r[1].item() == pytest.approx(10.0 * math.exp(-5.0 / 5.0), abs=1e-5)


def test_ballistic_impact_drag_branch_changes_result_when_nonzero():
    """Phase-2 model mismatch: nonzero drag must move the impact vs the
    drag-free nominal prediction (the gap the residual learns to close)."""
    pos_xy = torch.tensor([[0.0, 0.0]])
    vel_xy = torch.tensor([[3.0, 0.0]])
    altitude = torch.tensor([10.0])
    nominal = mu.predict_impact_nominal(pos_xy, vel_xy, altitude, 0.1, 9.81)
    with_drag = mu.ballistic_impact(pos_xy, vel_xy, altitude, 0.1, 9.81, torch.tensor([0.2]), torch.zeros(1, 2))
    assert not torch.allclose(nominal, with_drag), "nonzero drag must change the impact point"


# =====================================================================
# Guards: overshoot dormancy/reachability, stagnation
# =====================================================================

# Authoritative v13/v15 constants (mirrored from DroneBombardRewardCfg /
# DroneBombardTerminationCfg — kept here as raw floats so this file stays
# isaaclab-free).
_SUCCESS_RADIUS = 0.8
_OVERSHOOT_CLOSE_THRESHOLD = 0.6
_OVERSHOOT_FLYTHROUGH_RADIUS = 1.2
_OVERSHOOT_MARGIN = 2.0


def test_overshoot_terminating_guard_dormant_by_construction_at_success_radius():
    """The terminating overshoot guard's arm radius (0.6) is strictly below
    the success radius (0.8): since success triggers at d_xy<=0.8 before
    d_xy_min could ever sample below 0.6, the guard cannot fire while
    success_radius=0.8. This mirrors the Gazebo source's own dormancy
    (drone_drop_env.py:775-835) — Rule 10's deliberate design, not a bug."""
    assert _OVERSHOOT_CLOSE_THRESHOLD < _SUCCESS_RADIUS


def test_overshoot_flythrough_diagnostic_reachable_independent_of_success():
    """The non-terminating diagnostic counter's arm radius (1.2) is above
    success_radius (0.8), so it CAN fire (grazing passes that cross the
    success disk between sampled steps) without needing the curriculum to
    tighten success_radius first."""
    assert _OVERSHOOT_FLYTHROUGH_RADIUS > _SUCCESS_RADIUS


def test_overshoot_guard_fires_on_recede_past_close_approach():
    d_xy_min = torch.tensor([0.5, 0.9, 0.5])
    d_xy_now = torch.tensor([3.0, 3.0, 2.0])  # first: fires; second: min never got close; third: within margin
    fired = mu.overshoot_guard(d_xy_now, d_xy_min, _OVERSHOOT_CLOSE_THRESHOLD, _OVERSHOOT_MARGIN)
    assert fired.tolist() == [True, False, False]


def test_stagnation_guard_fires_only_after_window_with_low_progress():
    window, min_progress = 150, 1.0
    # before window: never fires regardless of progress
    assert not bool(mu.stagnation_guard(torch.tensor([5.0]), torch.tensor([5.0]), torch.tensor([100]), window, min_progress).item())
    # at/after window, insufficient progress -> fires
    assert bool(mu.stagnation_guard(torch.tensor([4.5]), torch.tensor([5.0]), torch.tensor([150]), window, min_progress).item())
    # at/after window, sufficient progress -> does not fire
    assert not bool(mu.stagnation_guard(torch.tensor([2.0]), torch.tensor([5.0]), torch.tensor([150]), window, min_progress).item())


def test_stagnation_guard_worst_case_7m_spawn_has_margin():
    """A real approach from the widened 7m spawn needs >=0.067 m/s average
    to clear the 15s/150-step window (1.0m / 15s); any approach speed
    >=0.1 m/s (a very slow crawl) clears it with >=50% margin — see the
    migration plan's guard-behavior analysis."""
    window, min_progress = 150, 1.0
    slow_progress_15s = 0.1 * 15.0  # 1.5 m progress at a "very slow" 0.1 m/s
    fired = mu.stagnation_guard(
        torch.tensor([7.0 - slow_progress_15s]), torch.tensor([7.0]), torch.tensor([150]), window, min_progress,
    )
    assert not bool(fired.item())


# =====================================================================
# Reward formula — hand-computed scenarios, each isolating one term
# =====================================================================

class _RewardCfg:
    w_dist = 2.0
    w_heading = 0.0
    speed_gate_enabled = True
    w_proximity = 0.6
    proximity_radius = 2.0
    w_vision_center = 1.5
    w_time = 0.05
    w_ang_vel = 0.15
    w_action_smooth = 0.20
    w_vel = 0.08
    vel_damp_radius = 3.0


def _reward(d_xy, d_xy_prev, pos_xy, target_xy, vel_xy, ang_vel, delta_action, u_norm, v_norm, conf):
    r, breakdown = mu.compute_reward(
        torch.tensor([d_xy]), torch.tensor([d_xy_prev]), torch.tensor([pos_xy]), torch.tensor([target_xy]),
        torch.tensor([vel_xy]), torch.tensor([ang_vel]), torch.tensor([delta_action]),
        torch.tensor([u_norm]), torch.tensor([v_norm]), torch.tensor([conf]), _RewardCfg,
    )
    return r.item(), {k: v.item() for k, v in breakdown.items()}


def test_reward_neutral_hover_is_time_penalty_only():
    r, _ = _reward(5.0, 5.0, [0.0, 0.0], [5.0, 0.0], [0.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 0.0)
    assert r == pytest.approx(-0.05, abs=1e-6)


def test_reward_distance_gradient_isolated():
    # d_xy 5 -> 4 (moved 1m closer); far from proximity/vel-damp ranges; no vision.
    r, bd = _reward(4.0, 5.0, [0.0, 0.0], [4.0, 0.0], [1.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 0.0)
    assert bd["rew_dist"] == pytest.approx(2.0, abs=1e-6)
    assert r == pytest.approx(-0.05 + 2.0, abs=1e-6)


def test_reward_proximity_bonus_isolated():
    # d_xy stays at 1.0 (no motion): proximity = 0.6*(1-1/2) = 0.3
    r, bd = _reward(1.0, 1.0, [0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 0.0)
    assert bd["rew_proximity"] == pytest.approx(0.3, abs=1e-6)
    assert r == pytest.approx(-0.05 + 0.3, abs=1e-6)


def test_reward_ang_vel_penalty_isolated():
    # d_xy=10 (outside proximity_radius=2 and vel_damp_radius=3), no motion.
    r, bd = _reward(10.0, 10.0, [0.0, 0.0], [10.0, 0.0], [0.0, 0.0], [1.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 0.0)
    assert bd["rew_ctrl"] == pytest.approx(-0.05 - 0.15 * 1.0, abs=1e-6)
    assert r == pytest.approx(-0.05 - 0.15, abs=1e-6)


def test_reward_action_smoothness_penalty_isolated():
    r, bd = _reward(10.0, 10.0, [0.0, 0.0], [10.0, 0.0], [0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0)
    assert bd["rew_ctrl"] == pytest.approx(-0.05 - 0.20 * 1.0, abs=1e-6)
    assert r == pytest.approx(-0.05 - 0.20, abs=1e-6)


def test_reward_velocity_damping_isolated():
    # d_xy=2.5: inside vel_damp_radius(3) -> near_factor=1-2.5/3=0.16667;
    # outside proximity_radius(2) -> proximity term is 0.
    r, bd = _reward(2.5, 2.5, [0.0, 0.0], [2.5, 0.0], [2.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 0.0)
    expected_vel = -0.08 * 2.0 * (1.0 - 2.5 / 3.0)
    assert bd["rew_vel"] == pytest.approx(expected_vel, abs=1e-6)
    assert bd["rew_proximity"] == pytest.approx(0.0, abs=1e-6)


def test_reward_vision_centering_isolated():
    # centered (u=v=0), conf=1.0, d_xy=1.0 (proximity_factor = 1-1/30).
    r, bd = _reward(1.0, 1.0, [0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 1.0)
    expected_vision = 1.5 * 1.0 * 1.0 * (1.0 - 1.0 / 30.0)
    assert bd["rew_vision"] == pytest.approx(expected_vision, abs=1e-6)


def test_reward_vision_zero_when_conf_zero():
    r, bd = _reward(1.0, 1.0, [0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 0.0)
    assert bd["rew_vision"] == pytest.approx(0.0, abs=1e-6)


def test_reward_heading_disabled_contributes_zero_even_with_motion():
    """w_heading=0.0 in v13/v15 — must contribute exactly zero regardless of
    how well-aligned the velocity is with the target bearing."""
    r, bd = _reward(4.0, 4.0, [0.0, 0.0], [4.0, 0.0], [1.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 4, 0.0, 0.0, 0.0)
    assert bd["rew_orient"] == pytest.approx(0.0, abs=1e-9)


def test_aim_error_reward_mapping():
    aim_err = torch.tensor([0.0, 0.5, 2.0])
    r = mu.aim_error_reward(aim_err, 2.0, 0.5)
    assert r[0].item() == pytest.approx(2.0, abs=1e-6)  # perfect aim -> full weight
    assert r[1].item() == pytest.approx(2.0 * (1.0 - math.tanh(1.0)), abs=1e-6)
    assert r[2].item() < 0.1  # ~0 beyond ~3*scale
    # monotonically decreasing in aim error
    assert r[0] > r[1] > r[2]


def test_aim_error_reward_off_and_sentinel():
    aim_err = torch.tensor([0.0, 0.5, float("inf")])
    # w_aim=0 -> exact zeros (Phase-1 reward parity default)
    assert torch.equal(mu.aim_error_reward(aim_err, 0.0, 0.5), torch.zeros(3))
    # inf sentinel (pre-first-evaluation) -> exactly 0 even with the term on
    r = mu.aim_error_reward(aim_err, 2.0, 0.5)
    assert r[2].item() == 0.0
    assert torch.isfinite(r).all()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
