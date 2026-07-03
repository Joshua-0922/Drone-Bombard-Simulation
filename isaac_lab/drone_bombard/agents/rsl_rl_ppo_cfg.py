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
    num_steps_per_env = 32
    max_iterations = 3000
    save_interval = 50
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
