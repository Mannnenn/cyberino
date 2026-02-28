import matplotlib.pyplot as plt
import numpy as np
from stable_baselines3 import PPO

from ppo_sim import WheeledPendulumEnv


def simulate_ppo_controller(
    model_path: str,
    initial_state: np.ndarray,
    simulation_time: float = 10.0,
    dt: float = 0.02,
):
    """
    保存されたPPOモデルを使用してシミュレーションを実行

    Args:
        model_path: PPOモデルのパス (.zip拡張子なし)
        initial_state: 初期状態 [theta, theta_dot, phi, phi_dot]
        simulation_time: シミュレーション時間 [s]
        dt: 制御周期 [s]

    Returns:
        t: 時刻配列
        theta: 振り子角度の履歴
        theta_dot: 振り子角速度の履歴
        phi: 車輪角度の履歴
        phi_dot: 車輪角速度の履歴
        tau: 制御トルクの履歴
    """
    # モデルの読み込み
    print(f"Loading PPO model from {model_path}...")
    model = PPO.load(model_path)

    # 環境の作成
    env = WheeledPendulumEnv()

    # 初期状態を設定
    env.reset()
    env.state = initial_state.copy()

    # シミュレーション
    num_steps = int(simulation_time / dt)

    # データ保存用の配列
    t = np.zeros(num_steps + 1)
    theta = np.zeros(num_steps + 1)
    theta_dot = np.zeros(num_steps + 1)
    phi = np.zeros(num_steps + 1)
    phi_dot = np.zeros(num_steps + 1)
    tau = np.zeros(num_steps + 1)

    # 初期値を記録
    t[0] = 0.0
    theta[0], theta_dot[0], phi[0], phi_dot[0] = env.state
    tau[0] = 0.0

    print(f"Starting simulation for {simulation_time}s ({num_steps} steps)...")
    print(
        f"Initial state: theta={theta[0]:.4f}, theta_dot={theta_dot[0]:.4f}, phi={phi[0]:.4f}, phi_dot={phi_dot[0]:.4f}"
    )

    # シミュレーションループ
    for i in range(num_steps):
        # PPOモデルから行動を取得
        obs = np.array(env.state, dtype=np.float32)
        action, _ = model.predict(obs, deterministic=True)

        # 環境でステップを実行
        next_obs, reward, terminated, truncated, info = env.step(action)

        # データを記録
        t[i + 1] = (i + 1) * dt
        theta[i + 1] = env.state[0]
        theta_dot[i + 1] = env.state[1]
        phi[i + 1] = env.state[2]
        phi_dot[i + 1] = env.state[3]
        tau[i + 1] = action[0] * env.max_torque

        # 終了判定
        if terminated or truncated:
            print(
                f"Episode terminated at t={t[i + 1]:.2f}s (theta={theta[i + 1]:.4f} rad)"
            )
            # 残りの配列を切り詰め
            t = t[: i + 2]
            theta = theta[: i + 2]
            theta_dot = theta_dot[: i + 2]
            phi = phi[: i + 2]
            phi_dot = phi_dot[: i + 2]
            tau = tau[: i + 2]
            break

    print("Simulation completed successfully!")

    return t, theta, theta_dot, phi, phi_dot, tau


def plot_simulation_results(t, theta, theta_dot, phi, phi_dot, tau, save_path=None):
    """
    シミュレーション結果をプロット

    Args:
        t: 時刻配列
        theta: 振り子角度の履歴
        theta_dot: 振り子角速度の履歴
        phi: 車輪角度の履歴
        phi_dot: 車輪角速度の履歴
        tau: 制御トルクの履歴
        save_path: グラフの保存パス (Noneの場合は保存しない)
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle("PPO Control Result (Non-linear model)", fontsize=14)

    plot_items = [
        (axes[0, 0], "Pendulum angle [rad]", theta, "Pendulum angle θ"),
        (
            axes[0, 1],
            "Pendulum ang. vel. [rad/s]",
            theta_dot,
            "Pendulum angular velocity dθ/dt",
        ),
        (axes[1, 0], "Wheel angle [rad]", phi, "Wheel angle φ"),
        (
            axes[1, 1],
            "Wheel ang. vel. [rad/s]",
            phi_dot,
            "Wheel angular velocity dφ/dt",
        ),
        (axes[2, 0], "Control torque [Nm]", tau, "Control torque τ"),
    ]

    for ax, ylabel, data, title in plot_items:
        ax.plot(t, data, color="tab:blue")
        ax.set_title(title)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(ylabel)
        ax.grid(True)
        y_min = np.min(data)
        y_max = np.max(data)
        margin = (y_max - y_min) * 0.1 if y_max != y_min else 0.1
        ax.set_ylim(y_min - margin, y_max + margin)

    # 右下は空欄を非表示
    axes[2, 1].set_visible(False)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Graph saved to {save_path}")

    plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="PPO制御モデルのテストとプロット")
    parser.add_argument(
        "--model",
        type=str,
        default="ppo_wheeled_pendulum",
        help="モデルのパス (.zip拡張子なし)",
    )
    parser.add_argument(
        "--time",
        type=float,
        default=10.0,
        help="シミュレーション時間 [s]",
    )
    parser.add_argument(
        "--theta0",
        type=float,
        default=0.1,
        help="初期振り子角度 [rad]",
    )
    parser.add_argument(
        "--theta_dot0",
        type=float,
        default=0.0,
        help="初期振り子角速度 [rad/s]",
    )
    parser.add_argument(
        "--phi0",
        type=float,
        default=0.0,
        help="初期車輪角度 [rad]",
    )
    parser.add_argument(
        "--phi_dot0",
        type=float,
        default=0.0,
        help="初期車輪角速度 [rad/s]",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="グラフの保存パス",
    )

    args = parser.parse_args()

    # 初期状態の設定
    initial_state = np.array(
        [
            args.theta0,
            args.theta_dot0,
            args.phi0,
            args.phi_dot0,
        ]
    )

    print("=== PPO Controller Test ===")
    print(f"Model: {args.model}")
    print(f"Simulation time: {args.time}s")
    print(
        f"Initial state: [{args.theta0}, {args.theta_dot0}, {args.phi0}, {args.phi_dot0}]"
    )
    print()

    # シミュレーション実行
    t, theta, theta_dot, phi, phi_dot, tau = simulate_ppo_controller(
        model_path=args.model,
        initial_state=initial_state,
        simulation_time=args.time,
    )

    # 結果のサマリーを表示
    print()
    print("=== Simulation Summary ===")
    print(f"Final time: {t[-1]:.2f}s")
    print(f"Final theta: {theta[-1]:.4f} rad ({np.rad2deg(theta[-1]):.2f} deg)")
    print(
        f"Max |theta|: {np.max(np.abs(theta)):.4f} rad ({np.rad2deg(np.max(np.abs(theta))):.2f} deg)"
    )
    print(f"Max |tau|: {np.max(np.abs(tau)):.4f} Nm")
    print()

    # プロット
    save_path = args.save if args.save else f"{args.model}_test_plot.png"
    plot_simulation_results(t, theta, theta_dot, phi, phi_dot, tau, save_path=save_path)
