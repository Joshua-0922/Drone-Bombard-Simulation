"""rsl_rl PPO runner config for Isaac-DroneBombard-Direct-v0.

Mapped from the v13/v15 SAC context (net_arch [256,256], gamma=0.995) to
PPO defaults reasonable for 2048 GPU-vectorized envs. See the migration
plan / notes/experiments/exp_012_isaac_migration_phase2.md for rationale.
"""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class DroneBombardPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 96
    """Raised from 32 on 2026-08-30. Episodes run ~97 policy steps, so a 32-step
    rollout covered only a THIRD of one -- and the single positive signal in the
    task (the terminal landing reward, ~+265) sits at the end. Every step of the
    approach nets about -1.65, so the advantage seen by the approach phase came
    almost entirely from bootstrapping V(s_32), and improving the approach
    depended on the critic having already learned to predict a reward it never
    saw inside the window. 96 covers a whole episode."""
    max_iterations = 3000
    save_interval = 25   # finer retention: keep model_{it}.pt every 25 iters so a
                         # near-peak checkpoint survives (v19 iter-375 was lost when
                         # only one hand-copied .pt was kept). NEVER wipe the log_dir.
    experiment_name = "drone_bombard_ppo"
    empirical_normalization = False

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.8,
        actor_hidden_dims=[256, 256],
        critic_hidden_dims=[256, 256],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        desired_kl=0.01,
        gamma=0.995,
        lam=0.95,
        max_grad_norm=1.0,
    )
