import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

# 1. 生成された非線形関数のインポート
try:
    from generated_eq1_func import func_eq1
    from generated_eq2_func import func_eq2

    print("Successfully imported non-linear equations.")
except ImportError:
    print("Error: generated_eq1_func.py or generated_eq2_func.py not found.")

# 2. 保存された A, B 行列の読み込み
data = np.load("matrices.npz")
A = data["A"]
B = data["B"]

_a = 0.054  # m

print(f"Loaded A matrix shape: {A.shape}")
print(f"Loaded B matrix shape: {B.shape}")

# 3. LQR制御器の設計
# 状態ベクトル x = [theta, theta_dot, phi, phi_dot]^T
# 入力ベクトル u = [tau, f_d]^T だが、制御に使うのは tau (1番目の入力) のみ
B_c = B[:, 0:1]  # 制御入力 tau に対応する列を抽出

# 重み行列の設定 (調整用)
Q = np.diag([0.2, 2, 0.5, 0.5])
R = np.array([[1]])

# リッカチ方程式を解いてゲイン F を算出
P = solve_continuous_are(A, B_c, Q, R)
F = -np.linalg.inv(R) @ B_c.T @ P
F = F.flatten()

print(f"LQR Feedback Gain F: {F}")

# 4. シミュレーション用の設定
tau_max = 6.0  # 最大トルク制限


# Non-liner
def func(t, x):
    # Extract state variables
    # State variable x = [theta, theta_dot, phi, phi_dot]^T
    theta, theta_dot, phi, phi_dot = x

    # Torque for control
    tau = torque_control(t, x)

    # Disturbance
    f_d = force_disturbance(t, x)

    # Calculate derivative of state x
    dxdt = np.zeros_like(x)
    dxdt[0] = theta_dot
    dxdt[1] = func_eq1(theta, theta_dot, phi, phi_dot, tau, f_d)
    dxdt[2] = phi_dot
    dxdt[3] = func_eq2(theta, theta_dot, phi, phi_dot, tau, f_d)

    # Return result
    return dxdt


# ------------------------------------------------
# Linearized state-space equation
# ------------------------------------------------
def func_lin(t, x):
    # Extract state variables
    # State variable x = [theta, theta_dot, phi, phi_dot]^T
    theta, theta_dot, phi, phi_dot = x

    # Torque for control
    tau = torque_control(t, x)

    # Disturbance
    f_d = force_disturbance(t, x)

    # Input vector
    u = np.array([tau, f_d])

    # Calculate derivative of state x
    dxdt = A @ x + B @ u

    # Return result
    return dxdt


# ------------------------------------------------
# Torque for control
# ------------------------------------------------
# State feedback gain, obtained by linearization and LQR!
F = F
# Maximum torque, Nm
tau_max = 3


def torque_control(t, x):
    # State variable x = [theta, theta_dot, phi, phi_dot]^T
    theta, theta_dot, phi, phi_dot = x
    # if t < 15:
    #     # State-feedback control
    #     tau = np.clip(F @ x, -tau_max, tau_max)
    # else:
    #     # Stop controlling
    #     tau = 0

    tau = np.clip(F @ x, -tau_max, tau_max)

    return tau


# ------------------------------------------------
# Torque for control PID
# ------------------------------------------------
# State feedback gain, obtained by linearization and LQR!
F = F
# Maximum torque, Nm
tau_max = 3


# ------------------------------------------------
# Disturbance force
# ------------------------------------------------
def force_disturbance(t, x):
    # # State variable x = [theta, theta_dot, phi, phi_dot]^T
    # if 5 <= t < 6:
    #     f_d = 5
    # elif 8 <= t < 11:
    #     f_d = 5 * np.sin(2 * np.pi * 1 * (t - 9))
    # else:
    #     f_d = 0

    f_d = 0

    return f_d


# ------------------------------------------------
# Solution of non-lin. and lin dynamics by SciPy
# ------------------------------------------------

# NumPy array for time
t_end = 20  # Time to end simulation, s
dt = 0.01  # Time slice, s
_t = np.arange(0, t_end, dt)  # Time array
# Initial conditions
theta_0 = np.radians(15)  # Initial pendulum angle, rad
x_w0 = 0.3  # Initial wheel positon, m
phi_0 = x_w0 / _a - theta_0  # Initial wheel angle with respect to pendulum
x0 = [theta_0, 0, phi_0, 0]  # Initial state
# Now solve it, non-linear
sol = solve_ivp(func, [0, t_end], x0, t_eval=_t)
# Now solve it, linearized
sol_lin = solve_ivp(func_lin, [0, t_end], x0, t_eval=_t)

# ------------------------------------------------
# Post-processing of the solution, non-linear
# ------------------------------------------------
# State variables
_theta = (sol.y[0, :].T + np.pi) % (2 * np.pi) - np.pi
_theta_dot = sol.y[1, :].T
_phi = (sol.y[2, :].T + np.pi) % (2 * np.pi) - np.pi
_phi_dot = sol.y[3, :].T
# Position and velocity of wheel, theta + phi
_x_w = _a * (sol.y[0, :].T + sol.y[2, :].T)
_v_w = _a * (sol.y[1, :].T + sol.y[3, :].T)
# Reproduce control torque and disturbance force
_tau_cont = np.zeros_like(_t)
_f_d = np.zeros_like(_t)
for i, __t in enumerate(_t):
    _tau_cont[i] = torque_control(__t, sol.y[:, i])
    _f_d[i] = force_disturbance(__t, sol.y[:, i])

# ------------------------------------------------
# Post-processing of the solution, linear
# ------------------------------------------------
# State variables
_theta_lin = (sol_lin.y[0, :].T + np.pi) % (2 * np.pi) - np.pi
_theta_dot_lin = sol_lin.y[1, :].T
_phi_lin = (sol_lin.y[2, :].T + np.pi) % (2 * np.pi) - np.pi
_phi_dot_lin = sol_lin.y[3, :].T
# Position and velocity of wheel, theta + phi
_x_w_lin = _a * (sol_lin.y[0, :].T + sol_lin.y[2, :].T)
_v_w_lin = _a * (sol_lin.y[1, :].T + sol_lin.y[3, :].T)
# Reproduce control torque and disturbance force
_tau_cont_lin = np.zeros_like(_t)
_f_d_lin = np.zeros_like(_t)
for i, __t in enumerate(_t):
    _tau_cont_lin[i] = torque_control(__t, sol_lin.y[:, i])
    _f_d_lin[i] = force_disturbance(__t, sol_lin.y[:, i])

# 5. 数値計算の実行
t_span = (0, 20)
t_eval = np.linspace(t_span[0], t_span[1], 2000)
x0 = [np.radians(15), 0, 0, 0]  # 初期状態: 振り子15度傾斜

# Now solve it, non-linear
sol = solve_ivp(func, [0, t_end], x0, t_eval=_t)
# Now solve it, linearized
sol_lin = solve_ivp(func_lin, [0, t_end], x0, t_eval=_t)

# 6. 結果の可視化
fig, axes = plt.subplots(4, 2, figsize=(14, 16))
fig.suptitle("Non-linear vs Linear model comparison", fontsize=14)

plot_items = [
    # (ax, ylabel, nonlin_data, lin_data, title)
    (axes[0, 0], "Pendulum angle [rad]", _theta, _theta_lin, "Pendulum angle θ"),
    (
        axes[0, 1],
        "Pendulum ang. vel. [rad/s]",
        _theta_dot,
        _theta_dot_lin,
        "Pendulum angular velocity dθ/dt",
    ),
    (axes[1, 0], "Wheel angle [rad]", _phi, _phi_lin, "Wheel angle φ"),
    (
        axes[1, 1],
        "Wheel ang. vel. [rad/s]",
        _phi_dot,
        _phi_dot_lin,
        "Wheel angular velocity dφ/dt",
    ),
    (axes[2, 0], "Wheel position [m]", _x_w, _x_w_lin, "Wheel position x_w"),
    (
        axes[2, 1],
        "Wheel trans. vel. [m/s]",
        _v_w,
        _v_w_lin,
        "Wheel translational velocity v_w",
    ),
    (axes[3, 0], "Disturbance force [N]", _f_d, _f_d_lin, "Disturbance force f_d"),
    (axes[3, 1], "Control torque [Nm]", _tau_cont, _tau_cont_lin, "Control torque τ"),
]

for ax, ylabel, data_nonlin, data_lin, title in plot_items:
    ax.plot(_t, data_nonlin, label="Non-linear", color="tab:blue")
    ax.plot(_t, data_lin, label="Linear", color="tab:orange", linestyle="--")
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.legend()
    ax.grid(True)
    # Y-axis limits based on non-linear results
    y_min = np.min(data_nonlin)
    y_max = np.max(data_nonlin)
    margin = (y_max - y_min) * 0.1 if y_max != y_min else 0.1
    ax.set_ylim(y_min - margin, y_max + margin)

plt.tight_layout()
plt.show()
