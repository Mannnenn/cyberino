import time

import casadi as ca
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import cont2discrete

from non_linear_EoM import func

_data = np.load("matrices.npz")
A = _data["A"]
B = _data["B"][:, [0]]  # tau列のみ (4x1)
C = np.eye(4)
D = np.zeros((4, 1))

# 3. 制御周期 (秒)
dt = 0.02

# 4. 離散化の実行 (method='zoh' がゼロ次ホールドを指定します)
sys_d = cont2discrete((A, B, C, D), dt, method="zoh")

# 5. 離散系の行列を取り出す
Ad = sys_d[0]
Bd = sys_d[1]


# --- 1. MPCの設定 ---
N = 50  # 予測ホライズン
nx = 4  # 状態の次元
nu = 1  # 入力の次元 (tauのみ)

# Q, R行列の設定 (重み調整用)
Q = np.diag([5.0, 1.0, 0.1, 0.01])  # thetaの偏差を強くペナルティ
R = np.diag([0.01])  # 入力トルクのペナルティ

max_tau = 12.0


# --- 2. CasADiベースのMPCセットアップ ---
def setup_mpc_casadi(Ad, Bd, N, nx, nu, Q, R, max_tau):
    """CasADiを使用したMPCの初期化"""
    # 最適化問題の定義
    opti = ca.Opti()

    # 最適化変数
    X = opti.variable(nx, N + 1)  # 状態変数
    U = opti.variable(nu, N)  # 入力変数

    # パラメータ（初期状態）
    x_init = opti.parameter(nx, 1)

    # 初期条件制約
    opti.subject_to(X[:, 0] == x_init)

    # コスト関数と制約の構築
    cost = 0
    for k in range(N):
        # ステージコスト
        state_cost = ca.mtimes([X[:, k].T, Q, X[:, k]])
        input_cost = ca.mtimes([U[:, k].T, R, U[:, k]])
        cost += state_cost + input_cost

        # 状態方程式制約
        opti.subject_to(X[:, k + 1] == Ad @ X[:, k] + Bd @ U[:, k])

        # 入力制約
        opti.subject_to(opti.bounded(-max_tau, U[:, k], max_tau))

    # 終端コスト
    terminal_cost = ca.mtimes([X[:, N].T, Q, X[:, N]])
    cost += terminal_cost

    # 目的関数の設定
    opti.minimize(cost)

    # ソルバー設定（IPOPT: Interior Point OPTimizer）
    opts = {
        "ipopt.print_level": 0,  # ログ出力を抑制
        "print_time": 0,  # 計算時間の表示を抑制
        "ipopt.warm_start_init_point": "yes",  # warm startを有効化
        "ipopt.max_iter": 100,  # 最大反復回数
        "ipopt.tol": 1e-4,  # 許容誤差（精度を少し緩める）
    }
    opti.solver("ipopt", opts)

    return opti, X, U, x_init


# --- 3. MPC求解関数 ---
def solve_mpc_casadi(opti, X, U, x_init_param, x_current, prev_U=None):
    """CasADiのMPCを解く"""
    # 初期状態を設定
    opti.set_value(x_init_param, x_current)

    # warm start（前回の解を初期推定値として使用）
    if prev_U is not None:
        try:
            opti.set_initial(U, prev_U)
        except:
            pass

    # 最適化問題を解く
    try:
        sol = opti.solve()
        u_opt = sol.value(U[:, 0])
        U_opt = sol.value(U)
        return u_opt[0] if isinstance(u_opt, np.ndarray) else u_opt, U_opt
    except Exception as e:
        # 解が見つからない場合は0を返す
        print(f"MPC solver failed: {e}")
        return 0.0, prev_U


# --- 4. MPCの初期化 ---
print("Initializing CasADi MPC...")
opti, X, U, x_init_param = setup_mpc_casadi(Ad, Bd, N, nx, nu, Q, R, max_tau)
print("CasADi MPC initialized.")


# --- 5. メインのシミュレーションループ ---
dt = 0.02
sim_steps = 500
x_history = []
tau_history = []
calc_times = []  # 各ステップの計算時間を記録
x_current = np.array([0.10, 0.0, 0.0, 0.0])  # 初期状態 (例: 少し傾いた状態)
prev_U_opt = None

print(f"Starting simulation ({sim_steps} steps)...")

for step in range(sim_steps):
    step_start = time.perf_counter()  # ステップ開始時刻

    x_history.append(x_current)

    # 1. MPCで最適な入力 tau を計算 (線形モデルを使用)
    tau, prev_U_opt = solve_mpc_casadi(opti, X, U, x_init_param, x_current, prev_U_opt)
    tau_history.append(tau)

    # 2. 非線形モデルでシミュレーションを進める
    # func(t, x, tau) は外乱などを考慮した非線形微分方程式
    sol = solve_ivp(
        fun=lambda t, x: func(t, x, tau=tau, f_d=0.0),  # 外乱は0と仮定
        t_span=[0, dt],
        y0=x_current,
        method="RK45",
    )

    # 3. 状態を更新
    x_current = sol.y[:, -1]

    step_end = time.perf_counter()  # ステップ終了時刻
    calc_times.append(step_end - step_start)

    # 進捗表示
    if (step + 1) % 100 == 0:
        avg_time = np.mean(calc_times[-100:])
        print(f"Step {step + 1}/{sim_steps}, Avg calc time: {avg_time * 1000:.2f} ms")


# --- 6. 結果の整形 ---
x_history = np.array(x_history)  # (sim_steps, 4)
tau_history = np.array(tau_history)  # (sim_steps,)
calc_times = np.array(calc_times)  # (sim_steps,)
t = np.arange(sim_steps) * dt

# 計算時間の統計情報を表示
print("\n" + "=" * 60)
print("Calculation Time Statistics (per control step)")
print("=" * 60)
print(f"Average: {np.mean(calc_times) * 1000:.3f} ms ({np.mean(calc_times):.6f} s)")
print(f"Min:     {np.min(calc_times) * 1000:.3f} ms ({np.min(calc_times):.6f} s)")
print(f"Max:     {np.max(calc_times) * 1000:.3f} ms ({np.max(calc_times):.6f} s)")
print(f"Std:     {np.std(calc_times) * 1000:.3f} ms ({np.std(calc_times):.6f} s)")
print(f"Median:  {np.median(calc_times) * 1000:.3f} ms ({np.median(calc_times):.6f} s)")
print("=" * 60)
print(f"Control period (dt): {dt * 1000:.1f} ms ({dt} s)")
if np.mean(calc_times) > dt:
    print("WARNING: Average calculation time exceeds control period!")
    print(f"Real-time factor: {np.mean(calc_times) / dt:.2f}x slower than real-time")
else:
    print("OK: Calculation is fast enough for real-time control")
    print(f"Time margin: {(dt - np.mean(calc_times)) * 1000:.3f} ms")
print("=" * 60)

_theta = x_history[:, 0]
_theta_dot = x_history[:, 1]
_phi = x_history[:, 2]
_phi_dot = x_history[:, 3]
_tau = tau_history


# --- 7. プロット ---
fig, axes = plt.subplots(3, 2, figsize=(14, 12))
fig.suptitle("MPC Control Result (CasADi, Non-linear model)", fontsize=14)

plot_items = [
    (axes[0, 0], "Pendulum angle [rad]", _theta, "Pendulum angle θ"),
    (
        axes[0, 1],
        "Pendulum ang. vel. [rad/s]",
        _theta_dot,
        "Pendulum angular velocity dθ/dt",
    ),
    (axes[1, 0], "Wheel angle [rad]", _phi, "Wheel angle φ"),
    (axes[1, 1], "Wheel ang. vel. [rad/s]", _phi_dot, "Wheel angular velocity dφ/dt"),
    (axes[2, 0], "Control torque [Nm]", _tau, "Control torque τ"),
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
plt.show()
