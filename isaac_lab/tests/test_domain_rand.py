"""Pure-torch unit tests for drone_bombard/mdp/domain_rand.py (P0 samplers).

Same constraint as test_math.py: must run WITHOUT isaaclab installed, so the
module is loaded by file path instead of through ``drone_bombard.mdp`` (whose
package __init__ chain pulls in isaaclab via gym.register).

The property that matters most here is the OFF path: every sampler must return
its identity value AND consume no RNG, otherwise enabling the P0 knobs would
silently shift the random stream of every pre-P0 version.

Run: pytest isaac_lab/tests/test_domain_rand.py -v
"""

import importlib.util
import os

import pytest
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
_DR_PATH = os.path.join(_HERE, "..", "drone_bombard", "mdp", "domain_rand.py")

_spec = importlib.util.spec_from_file_location("domain_rand", _DR_PATH)
dr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dr)

DEV = torch.device("cpu")


# =====================================================================
# sample_uniform
# =====================================================================

def test_uniform_stays_in_range_and_varies():
    torch.manual_seed(0)
    out = dr.sample_uniform(2.0, 6.0, 4096, DEV)
    assert out.shape == (4096,)
    assert float(out.min()) >= 2.0 and float(out.max()) <= 6.0
    assert float(out.std()) > 0.5          # actually spread, not a constant
    assert abs(float(out.mean()) - 4.0) < 0.1


def test_uniform_collapsed_range_is_exact_constant_and_draws_no_rng():
    torch.manual_seed(0)
    before = torch.rand(1)
    torch.manual_seed(0)
    out = dr.sample_uniform(4.0, 4.0, 32, DEV)
    after = torch.rand(1)
    assert torch.equal(out, torch.full((32,), 4.0))
    torch.testing.assert_close(before, after)  # RNG stream untouched


# =====================================================================
# sample_scale — multiplicative plant/controller mismatch
# =====================================================================

def test_scale_disabled_is_exactly_one_and_draws_no_rng():
    torch.manual_seed(7)
    before = torch.rand(1)
    torch.manual_seed(7)
    out = dr.sample_scale(64, DEV, rel=0.05, enabled=False)
    after = torch.rand(1)
    assert torch.equal(out, torch.ones(64))
    torch.testing.assert_close(before, after)


def test_scale_zero_rel_is_identity_even_when_enabled():
    out = dr.sample_scale(16, DEV, rel=0.0, enabled=True)
    assert torch.equal(out, torch.ones(16))


def test_scale_spans_the_requested_band():
    torch.manual_seed(1)
    rel = 0.05
    out = dr.sample_scale(8192, DEV, rel=rel, enabled=True)
    assert float(out.min()) >= 1.0 - rel
    assert float(out.max()) <= 1.0 + rel
    assert abs(float(out.mean()) - 1.0) < 0.005          # centred on nominal
    assert float(out.max()) > 1.0 + 0.9 * rel            # reaches the edges
    assert float(out.min()) < 1.0 - 0.9 * rel


# =====================================================================
# sample_bias — the per-episode (slow) half of the noise model
# =====================================================================

def test_bias_disabled_is_zero_and_draws_no_rng():
    torch.manual_seed(3)
    before = torch.rand(1)
    torch.manual_seed(3)
    out = dr.sample_bias(8, 28, DEV, std=0.01, enabled=False)
    after = torch.rand(1)
    assert torch.equal(out, torch.zeros(8, 28))
    torch.testing.assert_close(before, after)


def test_bias_shape_and_scale():
    torch.manual_seed(2)
    std = 0.02
    out = dr.sample_bias(4096, 28, DEV, std=std, enabled=True)
    assert out.shape == (4096, 28)
    assert abs(float(out.std()) - std) < 0.1 * std
    # per-env constant: distinct envs get distinct biases (this is what makes it
    # a calibration offset rather than white noise)
    assert not torch.allclose(out[0], out[1])


def test_bias_zero_std_is_identity_even_when_enabled():
    out = dr.sample_bias(4, 6, DEV, std=0.0, enabled=True)
    assert torch.equal(out, torch.zeros(4, 6))


# =====================================================================
# The equivalence the payload-mass DR relies on:
#   free-flight trajectory depends on (k, m) only through k/m
# =====================================================================

def test_ballistic_coefficient_scaling_equals_mass_scaling():
    """Integrating m*dv/dt = m*g + k*|v|v with (k*s, m) gives the same path as
    (k, m/s) — which is why _payload_bc_scale can stand in for payload mass
    without a runtime PhysX mass write."""

    def fall(k, m, steps=200, dt=0.01):
        v = torch.zeros(3)
        p = torch.tensor([0.0, 0.0, 10.0])
        g = torch.tensor([0.0, 0.0, -9.81])
        for _ in range(steps):
            v_air = -v
            a = g + (k / m) * v_air.norm() * v_air
            v = v + a * dt
            p = p + v * dt
        return p

    s = 1.3
    torch.testing.assert_close(fall(0.005 * s, 0.1), fall(0.005, 0.1 / s))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
