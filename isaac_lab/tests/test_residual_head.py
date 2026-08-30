"""Unit test for train.py's L1 residual-head surgery (zero-init + freeze).

Runs WITHOUT isaaclab — train.py imports isaaclab at module scope, so the two
helpers under test are loaded by source extraction rather than by importing the
module. Keeping the test independent of Isaac Sim is what makes it runnable on
the dev box (see test_math.py's header for why).

What is actually being checked: that "freeze the nominal" really freezes it.
A residual arm whose velocity/drop outputs kept training would not be
"L0 + residual" at all, and the whole L0-vs-L1 attribution would be void — but
nothing in the training logs would show it.

Run: pytest isaac_lab/tests/test_residual_head.py -v
"""

import ast
import os
import types

import pytest
import torch
import torch.nn as nn

_HERE = os.path.dirname(os.path.abspath(__file__))
_TRAIN_PY = os.path.join(_HERE, "..", "train.py")


def _load_helpers():
    """Pull RESIDUAL_DIMS / _last_linear / _prepare_residual_head out of
    train.py without importing it (its module scope launches Isaac Sim)."""
    src = open(_TRAIN_PY).read()
    tree = ast.parse(src)
    wanted = {"_last_linear", "_prepare_residual_head"}
    out = ["import torch", "import torch.nn as nn"]
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "RESIDUAL_DIMS":
            out.append(ast.get_source_segment(src, node))
        elif isinstance(node, ast.FunctionDef) and node.name in wanted:
            out.append(ast.get_source_segment(src, node))
    mod = types.ModuleType("train_helpers")
    exec(compile("\n\n".join(out), "train_helpers", "exec"), mod.__dict__)
    return mod


H = _load_helpers()
ACT_DIM = 7


def _fake_runner():
    """Minimal stand-in for rsl_rl's OnPolicyRunner: runner.alg.policy.{actor,std}."""
    actor = nn.Sequential(nn.Linear(26, 32), nn.ELU(), nn.Linear(32, ACT_DIM))
    policy = types.SimpleNamespace(actor=actor, std=nn.Parameter(torch.full((ACT_DIM,), 0.8)))
    return types.SimpleNamespace(alg=types.SimpleNamespace(policy=policy)), actor, policy


def _args(zero=False, freeze=False):
    return types.SimpleNamespace(zero_init_residual=zero, freeze_nominal=freeze)


def _backward_once(actor, policy):
    """One backward pass with a loss that touches every output dim and the std."""
    out = actor(torch.randn(4, 26))
    (out.pow(2).sum() + policy.std.pow(2).sum()).backward()


def test_last_linear_finds_the_output_layer():
    runner, actor, _ = _fake_runner()
    assert H._last_linear(actor) is actor[2]


def test_zero_init_zeroes_only_the_residual_rows():
    runner, actor, _ = _fake_runner()
    head = actor[2]
    with torch.no_grad():          # make every row non-zero first
        head.weight.fill_(0.3)
        head.bias.fill_(0.3)

    H._prepare_residual_head(runner, _args(zero=True))

    assert torch.all(head.weight[5:7] == 0.0)
    assert torch.all(head.bias[5:7] == 0.0)
    # the nominal rows must be untouched
    assert torch.all(head.weight[:5] == 0.3)
    assert torch.all(head.bias[:5] == 0.3)


def test_zero_init_makes_the_residual_output_exactly_zero():
    runner, actor, _ = _fake_runner()
    H._prepare_residual_head(runner, _args(zero=True))
    out = actor(torch.randn(8, 26))
    assert torch.allclose(out[:, 5:7], torch.zeros(8, 2), atol=0.0), \
        "a zero-initialised head must emit delta = 0 on iteration 0"


def test_freeze_stops_gradient_on_everything_but_the_residual_rows():
    runner, actor, policy = _fake_runner()
    head = actor[2]

    H._prepare_residual_head(runner, _args(freeze=True))

    # trunk is off the graph entirely
    assert actor[0].weight.requires_grad is False
    assert actor[0].bias.requires_grad is False
    # the output layer stays trainable, but masked
    assert head.weight.requires_grad is True

    _backward_once(actor, policy)

    assert torch.all(head.weight.grad[:5] == 0.0), "nominal rows must not learn"
    assert torch.all(head.bias.grad[:5] == 0.0)
    assert torch.any(head.weight.grad[5:7] != 0.0), "residual rows must learn"
    assert torch.all(policy.std.grad[:5] == 0.0), \
        "nominal exploration noise must stay fixed or the frozen policy drifts"
    assert torch.any(policy.std.grad[5:7] != 0.0)
    assert actor[0].weight.grad is None


def test_freeze_leaves_the_nominal_output_bit_identical():
    runner, actor, policy = _fake_runner()
    x = torch.randn(8, 26)
    before = actor(x)[:, :5].clone()

    H._prepare_residual_head(runner, _args(freeze=True))
    opt = torch.optim.Adam([p for p in actor.parameters() if p.requires_grad]
                           + [policy.std], lr=0.1)
    for _ in range(5):
        opt.zero_grad()
        _backward_once(actor, policy)
        opt.step()

    assert torch.allclose(actor(x)[:, :5], before, atol=1e-6), \
        "after training the residual, the nominal outputs must be unchanged"


def test_rejects_an_actor_that_is_too_narrow():
    actor = nn.Sequential(nn.Linear(26, 32), nn.ELU(), nn.Linear(32, 4))
    policy = types.SimpleNamespace(actor=actor, std=nn.Parameter(torch.ones(4)))
    runner = types.SimpleNamespace(alg=types.SimpleNamespace(policy=policy))
    with pytest.raises(RuntimeError, match="residual needs rows"):
        H._prepare_residual_head(runner, _args(zero=True))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
