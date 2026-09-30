"""Training settings: sparse asymmetric PPO or privileged teacher + DAgger.

H2O/OmniH2O provides the motion-tracking/privileged-teacher/sparse-student
architecture. This implementation uses Isaac Lab and RSL-RL 2.3.3 directly;
it is an adaptation, not a reproduction of published H2O performance.
"""


def runner_config(stage="sparse", seed=42, device="cuda:0"):
    result = {
        "seed": seed, "device": device, "num_steps_per_env": 24,
        "save_interval": 100, "empirical_normalization": False,
        "logger": "tensorboard", "experiment_name": "g1_wholebody",
        "policy": {
            "class_name": "ActorCritic", "init_noise_std": 0.5,
            "actor_hidden_dims": [256, 256, 128],
            "critic_hidden_dims": [256, 256, 128], "activation": "elu",
        },
        "algorithm": {
            "class_name": "PPO", "value_loss_coef": 1.0,
            "use_clipped_value_loss": True, "clip_param": 0.2,
            "entropy_coef": 0.004, "num_learning_epochs": 5,
            "num_mini_batches": 4, "learning_rate": 0.001,
            "schedule": "adaptive", "gamma": 0.99, "lam": 0.95,
            "desired_kl": 0.01, "max_grad_norm": 1.0,
        },
    }
    if stage == "student":
        result["policy"] = {
            "class_name": "StudentTeacher", "init_noise_std": 0.05,
            "student_hidden_dims": [256, 256, 128],
            "teacher_hidden_dims": [256, 256, 128], "activation": "elu",
        }
        result["algorithm"] = {
            "class_name": "Distillation", "num_learning_epochs": 3,
            "gradient_length": 24, "learning_rate": 0.0003,
            "max_grad_norm": 1.0, "loss_type": "huber",
        }
    return result
