import json
from typing import List

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_checker import check_env

from generated_eq1_func import func_eq1
from generated_eq2_func import func_eq2


class TrainingMetricsCallback:
    """Callback to record training metrics"""

    def __init__(self):
        self.episodes: List[int] = []
        self.episode_rewards: List[float] = []
        self.episode_lengths: List[int] = []
        self.timesteps: List[int] = []
        self.mean_rewards: List[float] = []
        self.value_losses: List[float] = []
        self.policy_losses: List[float] = []

    def save_to_json(self, filepath: str):
        """Save metrics to JSON file"""
        data = {
            "episodes": self.episodes,
            "episode_rewards": self.episode_rewards,
            "episode_lengths": self.episode_lengths,
            "timesteps": self.timesteps,
            "mean_rewards": self.mean_rewards,
            "value_losses": self.value_losses,
            "policy_losses": self.policy_losses,
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

    @staticmethod
    def load_from_json(filepath: str) -> "TrainingMetricsCallback":
        """Load metrics from JSON file"""
        callback = TrainingMetricsCallback()
        with open(filepath, "r") as f:
            data = json.load(f)
        callback.episodes = data["episodes"]
        callback.episode_rewards = data["episode_rewards"]
        callback.episode_lengths = data["episode_lengths"]
        callback.timesteps = data["timesteps"]
        callback.mean_rewards = data["mean_rewards"]
        callback.value_losses = data.get("value_losses", [])
        callback.policy_losses = data.get("policy_losses", [])
        return callback


def plot_training_metrics(metrics: TrainingMetricsCallback, save_path: str = None):
    """Plot training metrics"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("PPO Training Progress", fontsize=16, fontweight="bold")

    # 1. Mean Reward Progress
    if metrics.timesteps and metrics.mean_rewards:
        axes[0, 0].plot(
            metrics.timesteps,
            metrics.mean_rewards,
            "b-",
            linewidth=2,
            label="Mean Reward",
        )
        axes[0, 0].set_xlabel("Steps")
        axes[0, 0].set_ylabel("Mean Reward")
        axes[0, 0].set_title("Mean Reward Progress")
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()

    # 2. Episode Length Progress
    if metrics.episodes and metrics.episode_lengths:
        # Display episode length with moving average for clarity
        window_size = min(50, len(metrics.episode_lengths) // 10 + 1)
        if len(metrics.episode_lengths) >= window_size:
            smoothed_lengths = np.convolve(
                metrics.episode_lengths,
                np.ones(window_size) / window_size,
                mode="valid",
            )
            smoothed_episodes = metrics.episodes[: len(smoothed_lengths)]
            axes[0, 1].plot(
                smoothed_episodes,
                smoothed_lengths,
                "g-",
                linewidth=2,
                label=f"Episode Length (MA{window_size})",
            )
        else:
            axes[0, 1].plot(
                metrics.episodes,
                metrics.episode_lengths,
                "g-",
                linewidth=2,
                label="Episode Length",
            )
        axes[0, 1].set_xlabel("Steps")
        axes[0, 1].set_ylabel("Episode Length")
        axes[0, 1].set_title("Episode Length Progress")
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()

    # 3. Value Loss (if data available)
    if metrics.value_losses and len(metrics.value_losses) > 0:
        # Filter out None values
        valid_indices = [i for i, v in enumerate(metrics.value_losses) if v is not None]
        if valid_indices:
            valid_timesteps = [
                metrics.timesteps[i]
                for i in valid_indices
                if i < len(metrics.timesteps)
            ]
            valid_losses = [metrics.value_losses[i] for i in valid_indices]
            axes[1, 0].plot(
                valid_timesteps, valid_losses, "r-", linewidth=2, label="Value Loss"
            )
            axes[1, 0].set_xlabel("Steps")
            axes[1, 0].set_ylabel("Loss")
            axes[1, 0].set_title("Value Loss Progress")
            axes[1, 0].grid(True, alpha=0.3)
            axes[1, 0].legend()
        else:
            axes[1, 0].text(
                0.5,
                0.5,
                "No Value Loss Data",
                ha="center",
                va="center",
                transform=axes[1, 0].transAxes,
            )
            axes[1, 0].set_title("Value Loss Progress")
    else:
        axes[1, 0].text(
            0.5,
            0.5,
            "No Value Loss Data",
            ha="center",
            va="center",
            transform=axes[1, 0].transAxes,
        )
        axes[1, 0].set_title("Value Loss Progress")

    # 4. Policy Loss (if data available)
    if metrics.policy_losses and len(metrics.policy_losses) > 0:
        # Filter out None values
        valid_indices = [
            i for i, v in enumerate(metrics.policy_losses) if v is not None
        ]
        if valid_indices:
            valid_timesteps = [
                metrics.timesteps[i]
                for i in valid_indices
                if i < len(metrics.timesteps)
            ]
            valid_losses = [metrics.policy_losses[i] for i in valid_indices]
            axes[1, 1].plot(
                valid_timesteps, valid_losses, "m-", linewidth=2, label="Policy Loss"
            )
            axes[1, 1].set_xlabel("Steps")
            axes[1, 1].set_ylabel("Loss")
            axes[1, 1].set_title("Policy Loss Progress")
            axes[1, 1].grid(True, alpha=0.3)
            axes[1, 1].legend()
        else:
            axes[1, 1].text(
                0.5,
                0.5,
                "No Policy Loss Data",
                ha="center",
                va="center",
                transform=axes[1, 1].transAxes,
            )
            axes[1, 1].set_title("Policy Loss Progress")
    else:
        axes[1, 1].text(
            0.5,
            0.5,
            "No Policy Loss Data",
            ha="center",
            va="center",
            transform=axes[1, 1].transAxes,
        )
        axes[1, 1].set_title("Policy Loss Progress")

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Graph saved to {save_path}.")

    plt.show()


class WheeledPendulumEnv(gym.Env):
    def __init__(self):
        super(WheeledPendulumEnv, self).__init__()

        # 物理パラメータやシミュレーション設定
        self.dt = 0.02  # 制御周期 (秒)
        self.max_torque = 12.0  # モータの最大トルク (適宜変更してください)
        self.max_theta = 45 * (np.pi / 180)  # 失敗とみなす振り子の角度 (例: ±12度)

        # アクション空間: 正規化されたトルク [-1.0, 1.0]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        # 状態空間: [theta, theta_dot, phi, phi_dot]
        # 各変数の観測可能な最大値・最小値を設定します
        obs_high = np.array(
            [
                np.pi,  # theta の最大値
                np.finfo(np.float32).max,  # theta_dot の最大値
                np.finfo(
                    np.float32
                ).max,  # phi の最大値 (走行距離の制限がなければ無限大)
                np.finfo(np.float32).max,  # phi_dot の最大値
            ],
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(
            low=-obs_high, high=obs_high, dtype=np.float32
        )

        self.state = None

    def _dynamics(self, x, tau):
        """ユーザー定義の非線形運動方程式"""
        theta, theta_dot, phi, phi_dot = x
        f_d = 0.0  # 学習時は外乱をゼロにするか、ランダムノイズを加える

        dxdt = np.zeros_like(x)
        dxdt[0] = theta_dot
        # func_eq1, func_eq2 は事前に定義されているものを使用
        dxdt[1] = func_eq1(theta, theta_dot, phi, phi_dot, tau, f_d)
        dxdt[2] = phi_dot
        dxdt[3] = func_eq2(theta, theta_dot, phi, phi_dot, tau, f_d)

        return dxdt

    def _rk4_step(self, x, tau):
        """ルンゲ=クッタ法 (RK4) による数値積分"""
        k1 = self._dynamics(x, tau)
        k2 = self._dynamics(x + 0.5 * self.dt * k1, tau)
        k3 = self._dynamics(x + 0.5 * self.dt * k2, tau)
        k4 = self._dynamics(x + self.dt * k3, tau)
        return x + (self.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def step(self, action):
        # アクションを実際のトルク値にスケール変換
        tau = action[0] * self.max_torque

        # 状態の更新
        self.state = self._rk4_step(self.state, tau)
        theta, theta_dot, phi, phi_dot = self.state

        # 終了判定: 振り子が倒れたらエピソード終了
        terminated = bool(abs(theta) > self.max_theta)
        truncated = False  # タイムステップの上限到達は学習ループ側(SB3など)で管理

        # 報酬関数の計算
        if not terminated:
            # 角度thetaを0に保ちつつ、無駄な車輪の回転(phi_dot)やトルク消費(tau)を抑える
            reward = 1.0 - (
                5.0 * theta**2
                + 1.0 * theta_dot**2
                + 0.1 * phi**2
                + 0.01 * phi_dot**2
                + 0.01 * tau**2
            )
        else:
            reward = -3.0  # 倒れた時のペナルティ

        return np.array(self.state, dtype=np.float32), reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 初期状態: 0付近で微小なランダム値を与える
        self.state = np.random.uniform(low=-0.1, high=0.1, size=(4,))
        return np.array(self.state, dtype=np.float32), {}


if __name__ == "__main__":
    import argparse

    # PPOによる学習

    parser = argparse.ArgumentParser(description="車輪型倒立振り子のPPO学習")
    parser.add_argument("--timesteps", type=int, default=100000, help="学習ステップ数")
    parser.add_argument(
        "--model",
        type=str,
        default="ppo_wheeled_pendulum",
        help="モデルの保存/読み込みパス",
    )
    args = parser.parse_args()

    # Callback class for recording metrics
    class MetricsRecorderCallback(BaseCallback):
        def __init__(self, metrics_storage: TrainingMetricsCallback, verbose=0):
            super().__init__(verbose)
            self.metrics_storage = metrics_storage
            self.episode_count = 0

        def _on_step(self) -> bool:
            return True

        def _on_rollout_end(self) -> None:
            """Called at the end of each rollout"""
            # Get statistics
            if len(self.model.ep_info_buffer) > 0:
                mean_reward = np.mean(
                    [ep_info["r"] for ep_info in self.model.ep_info_buffer]
                )
                mean_length = np.mean(
                    [ep_info["l"] for ep_info in self.model.ep_info_buffer]
                )

                # Record latest episode information
                for ep_info in self.model.ep_info_buffer:
                    if self.episode_count < len(self.model.ep_info_buffer) or True:
                        self.metrics_storage.episodes.append(self.num_timesteps)
                        self.metrics_storage.episode_rewards.append(ep_info["r"])
                        self.metrics_storage.episode_lengths.append(int(ep_info["l"]))

                # Record timesteps and rewards
                self.metrics_storage.timesteps.append(self.num_timesteps)
                self.metrics_storage.mean_rewards.append(mean_reward)

                # Record training statistics (at the same timing)
                if hasattr(self.logger, "name_to_value"):
                    if "train/value_loss" in self.logger.name_to_value:
                        self.metrics_storage.value_losses.append(
                            self.logger.name_to_value["train/value_loss"]
                        )
                    else:
                        self.metrics_storage.value_losses.append(None)

                    if "train/policy_gradient_loss" in self.logger.name_to_value:
                        self.metrics_storage.policy_losses.append(
                            self.logger.name_to_value["train/policy_gradient_loss"]
                        )
                    else:
                        self.metrics_storage.policy_losses.append(None)

                self.episode_count = len(self.model.ep_info_buffer)

    print("=== PPO Training Started ===")
    env = WheeledPendulumEnv()

    # Check environment
    print("Checking environment validity...")
    check_env(env)
    print("Environment check completed!")

    # Metrics recording object
    metrics = TrainingMetricsCallback()

    # Create PPO model
    print("\nCreating PPO model...")
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=1e-4,  # 3e-4 から下げて慎重に学習させる
        n_steps=2048,
        batch_size=128,  # 64 から増やして安定化
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,  # 0.0 から 0.01 に変更。探索を強制し Policy loss を動かす
    )

    # Set up callback
    callback = MetricsRecorderCallback(metrics)

    # Run training
    print(f"\nStarting training for {args.timesteps} steps...")
    model.learn(total_timesteps=args.timesteps, progress_bar=True, callback=callback)

    # Save model
    model.save(args.model)
    print(f"\nModel saved to {args.model}.zip.")

    # Save metrics
    metrics_file = f"{args.model}_metrics.json"
    metrics.save_to_json(metrics_file)
    print(f"Training history saved to {metrics_file}.")

    # Display and save graph
    print("\nGenerating training progress graph...")
    graph_file = f"{args.model}_training_plot.png"
    plot_training_metrics(metrics, save_path=graph_file)
