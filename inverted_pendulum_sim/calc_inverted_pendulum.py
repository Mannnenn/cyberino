import sympy as sp
from sympy.physics.mechanics import *
import numpy as np
import scipy.constants as cnst
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from sympy.printing.numpy import NumPyPrinter

init_vprinting()


# Common symbols and reference frame

# Define symbols, Common
t = sp.symbols("t")
g = sp.symbols("g")

# Define reference frame, Common
N = ReferenceFrame("N")
O = Point("O")  # Origin point
O.set_vel(N, 0)

# Pendulum

# Symbols
m, J_p, l, mu_p = sp.symbols("m J_p l mu_p")
theta = dynamicsymbols("theta")

# Reference frame and rigid body
N_p = N.orientnew("N_p", "Axis", (theta, N.y))
N_p.set_ang_vel(N, theta.diff(t, 1) * N.y)
C_p = Point("C_p")
J_p_tensor = J_p * outer(N_p.y, N_p.y)
P = RigidBody("P", C_p, N_p, m, (J_p_tensor, C_p))

# Wheel

# Symbols
M, J_w, a, mu_w = sp.symbols("M J_w a mu_w")
phi = dynamicsymbols("phi")

# Reference frame and rigid body
N_w = N_p.orientnew("N_w", "Axis", (phi, N_p.y))
N_w.set_ang_vel(N_p, phi.diff(t, 1) * N_p.y)
C_w = Point("C_w")
J_w_tensor = J_w * outer(N_w.y, N_w.y)
W = RigidBody("W", C_w, N_w, M, (J_w_tensor, C_w))

# Constraints

# Set positions
C_w.set_pos(O, a * ((theta + phi) * N.x + N.z))
C_p.set_pos(C_w, l * (sp.sin(theta) * N.x + sp.cos(theta) * N.z))

# Set velocities
C_w.set_vel(N, a * (theta.diff(t, 1) + phi.diff(t, 1)) * N.x)
C_p.set_vel(
    N,
    (a * (theta.diff(t, 1) + phi.diff(t, 1)) + l * theta.diff(t, 1) * sp.cos(theta))
    * N.x
    - l * theta.diff(t, 1) * sp.sin(theta) * N.z,
)

P_d = Point("P_d")
P_d.set_pos(C_p, l * N_p.z)
P_d.set_vel(
    N,
    (a * (theta.diff(t, 1) + phi.diff(t, 1)) + 2 * l * theta.diff(t, 1) * sp.cos(theta))
    * N.x
    - 2 * l * theta.diff(t, 1) * sp.sin(theta) * N.z,
)

# Potential energies

P.potential_energy = m * g * l * sp.cos(theta)

# Euler-Lagrange method

# Get Lagrangian
L = Lagrangian(N, P, W)

# Generalized coordinate
q = [theta, phi]

# Generalized forces and torques
# Control torque
tau = dynamicsymbols("tau")
f_control = [(N_p, -tau * N_p.y), (N_w, tau * N_w.y)]

# Friction or dissipative
# Friction between pendulum and wheel
tau_fp = sp.symbols("tau_fp")
tau_fp = mu_p * phi.diff(t, 1)

# Friction between pendulum and wheel and friction between wheel and floor
tau_fw = sp.symbols("tua_fw")
tau_fw = -mu_p * phi.diff(t, 1) - mu_w * (theta.diff(t, 1) + phi.diff(t, 1))

f_dissipation = [(N_p, tau_fp * N_p.y), (N_w, tau_fw * N_w.y)]

# Disturbance force
f_d = dynamicsymbols("f_d")
f_disturbance = [(P_d, f_d * N.x)]

# Summation of generalized forces and torques
f = f_control + f_dissipation + f_disturbance

# Define a Lagrange's method
LM = LagrangesMethod(L, q, forcelist=f, frame=N)

# Derive equation of motion!
eom = LM.form_lagranges_equations()

# Angular acceleration

# Solve equation of motion with respect to the second-order variables

sol = sp.solve(eom, [theta.diff(t, 2), phi.diff(t, 2)])

eq1 = sol[theta.diff(t, 2)].simplify()

eq2 = sol[phi.diff(t, 2)].simplify()

# Angular acceleration

# Solve equation of motion with respect to the second-order variables
sol = sp.solve(eom, [theta.diff(t, 2), phi.diff(t, 2)])

eq1 = sol[theta.diff(t, 2)].simplify()

eq2 = sol[phi.diff(t, 2)].simplify()

# Linearization

# Set operation point
op_point = {theta: 0, theta.diff(t, 1): 0, phi: 0, phi.diff(t, 1): 0}

# Linearrize!
kwargs = {
    "q_ind": [theta, phi],
    "qd_ind": [theta.diff(t, 1), phi.diff(t, 1)],
    "A_and_B": True,
    "op_point": op_point,
    "simplify": True,
}
A, B, inp_vec = LM.linearize(**kwargs)

# Numerical parameter examples

# Reset format
np.set_printoptions(formatter=None)

# Pendulum
_m = 1.50  # kg
_l = 0.30  # m
_J_p = _m * _l**2 / 3  # kgm^2
_mu_p = 0.05  # N/(rad/s)

# Wheel
_M = 0.75  # kg
_a = 0.10  # m
_J_w = _M * _a**2 / 2  # kgm^2
_mu_w = 0.05  # N/(rad/s)

# Gravity
_g = cnst.g  # m/s^2


# Lambdification of Sympy results

# Give derivatives new name
theta_dot, phi_dot = sp.symbols("theta_dot phi_dot")

# Definition of the lanbdified functions
_eq1 = eq1.subs(
    [
        (theta.diff(t, 1), theta_dot),
        (phi.diff(t, 1), phi_dot),
        (m, _m),
        (l, _l),
        (J_p, _J_p),
        (M, _M),
        (a, _a),
        (J_w, _J_w),
        (mu_p, _mu_p),
        (mu_w, _mu_w),
        (g, _g),
    ]
)

func_eq1 = sp.lambdify([theta, theta_dot, phi, phi_dot, tau, f_d], _eq1, "numpy")

_eq2 = eq2.subs(
    [
        (theta.diff(t, 1), theta_dot),
        (phi.diff(t, 1), phi_dot),
        (m, _m),
        (l, _l),
        (J_p, _J_p),
        (M, _M),
        (a, _a),
        (J_w, _J_w),
        (mu_p, _mu_p),
        (mu_w, _mu_w),
        (g, _g),
    ]
)

func_eq2 = sp.lambdify([theta, theta_dot, phi, phi_dot, tau, f_d], _eq2, "numpy")

# Linearization results to Numpy arrays,A and B

# NOTE:
# x = [theta,phi,theta_dot,phi_dot]^T
# u = [f_d,tau]^T

# The A matrix
_A_tmp = sp.matrix2numpy(
    A.subs(
        [
            (theta.diff(t, 1), theta_dot),
            (phi.diff(t, 1), phi_dot),
            (m, _m),
            (l, _l),
            (J_p, _J_p),
            (M, _M),
            (a, _a),
            (J_w, _J_w),
            (mu_p, _mu_p),
            (mu_w, _mu_w),
            (g, _g),
        ]
    ),
    dtype="float64",
)

# The B matrix
_B_tmp = sp.matrix2numpy(
    B.subs(
        [
            (theta.diff(t, 1), theta_dot),
            (phi.diff(t, 1), phi_dot),
            (m, _m),
            (l, _l),
            (J_p, _J_p),
            (M, _M),
            (a, _a),
            (J_w, _J_w),
            (mu_p, _mu_p),
            (mu_w, _mu_w),
            (g, _g),
        ]
    ),
    dtype="float64",
)

# Change raw and column orders to fit existing routines
_A_tmp = _A_tmp[[0, 2, 1, 3], :]
_A = _A_tmp[:, [0, 2, 1, 3]]
print(f"A = \n{_A}")

_B_tmp = _B_tmp[[0, 2, 1, 3], :]
_B = _B_tmp[:, [1, 0]]
print(f"B = \n{_B}")

# LQR, linear quadratic regulator

# Weights
_B_c = np.array([_B[:, 0]]).T
_Q = np.diag([35, 0, 2, 0])
_R = np.array([[20]])

# Solve Riccati equation
_P = solve_continuous_are(_A, _B_c, _Q, _R)
_F = (-np.linalg.inv(_R) @ _B_c.T @ _P)[0]
print(f"State-feedback gain _F = \n{_F}\n")

# Check eigenvalues of closed-loop system
w_lqr, v_lqr = np.linalg.eig(_A + _B_c * _F)
print(f"Eigenvalues of closed-loop system =\n{w_lqr}")

# Non-liner
def func(t, x):
    # Extract state variables
    # State variable x = [theta, theta_dot, phi, phi_dot]^T
    theta, theta_dot, phi, phi_dot = x

    # Torque for control
    # tau = torque_control(t, x)
    tau = torque_control_PID(t, x)

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
    # tau = torque_control(t, x)
    tau = torque_control_PID(t, x)

    # Disturbance
    f_d = force_disturbance(t, x)

    # Input vector
    u = np.array([tau, f_d])

    # Calculate derivative of state x
    dxdt = _A @ x + _B @ u

    # Return result
    return dxdt


# ------------------------------------------------
# Torque for control
# ------------------------------------------------
# State feedback gain, obtained by linearization and LQR!
F = _F
# Maximum torque, Nm
tau_max = 3


def torque_control(t, x):
    # State variable x = [theta, theta_dot, phi, phi_dot]^T
    theta, theta_dot, phi, phi_dot = x
    if t < 15:
        # State-feedback control
        tau = np.clip(F @ x, -tau_max, tau_max)
    else:
        # Stop controlling
        tau = 0

    return tau

# ------------------------------------------------
# Torque for control PID
# ------------------------------------------------
# State feedback gain, obtained by linearization and LQR!
F = _F
# Maximum torque, Nm
tau_max = 3


def torque_control_PID(t, x):
    # State variable x = [theta, theta_dot, phi, phi_dot]^T
    theta, theta_dot, phi, phi_dot = x
    if t < 15:
        # State-feedback control
        x_error = np.array([theta, theta_dot, phi, phi_dot]) - np.array([0, 0, 0, 0])
        tau_law = (
            x_error[0] * 8.15
            + x_error[1] * 1.78
            + x_error[2] * 0.31622777
            + x_error[3] * 0.30764017
        )
        tau = np.clip(tau_law, -tau_max, tau_max)
    else:
        # Stop controlling
        tau = 0

    return tau

    # 8.15993077 1.78704447 0.31622777 0.30764017

# ------------------------------------------------
# Disturbance force
# ------------------------------------------------
def force_disturbance(t, x):
    # State variable x = [theta, theta_dot, phi, phi_dot]^T
    if 5 <= t < 6:
        f_d = 5
    elif 8 <= t < 11:
        f_d = 5 * np.sin(2 * np.pi * 1 * (t - 9))
    else:
        f_d = 0

    # f_d = 0

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
x_w0 = 1  # Initial wheel positon, m
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

# NumPy用のPythonコードを生成



# --- 追加・修正する処理 ---
# 引数として使いたい「単なる変数」としてのシンボルを作成
sym_theta = sp.Symbol("theta")
sym_phi = sp.Symbol("phi")
sym_theta_dot = sp.Symbol("theta_dot")
sym_phi_dot = sp.Symbol("phi_dot")
sym_tau = sp.Symbol("tau")
sym_f_d = sp.Symbol("f_d")

# _eq1 に残っている関数や微分を、すべて単なる変数に置き換える
_eq1_static = _eq1.subs(
    [
        (theta, sym_theta),
        (phi, sym_phi),
        (theta_dot, sym_theta_dot),  # theta_dotも関数として定義されている場合
        (phi_dot, sym_phi_dot),  # phi_dotも関数として定義されている場合
        (tau, sym_tau),  # tauも関数として定義されている場合
        (f_d, sym_f_d),  # ここがエラーの原因でした
    ]
)

# NumPy用のPythonコードを生成
printer = NumPyPrinter({"strict": False})
code_str = printer.doprint(_eq1_static)

# エンコーディングを指定してファイルに書き出す
with open("generated_eq1_func.py", "w", encoding="utf-8") as f:
    f.write("import numpy\n\n")
    f.write("def func_eq1(theta, theta_dot, phi, phi_dot, tau, f_d):\n")
    f.write(f"    return {code_str}\n")


_eq2_static = _eq2.subs(
    [
        (theta, sym_theta),
        (phi, sym_phi),
        (theta_dot, sym_theta_dot),  # theta_dotも関数として定義されている場合
        (phi_dot, sym_phi_dot),  # phi_dotも関数として定義されている場合
        (tau, sym_tau),  # tauも関数として定義されている場合
        (f_d, sym_f_d),  # ここがエラーの原因でした
    ]
)

printer = NumPyPrinter({"strict": False})
code_str = printer.doprint(_eq2_static)

with open("generated_eq2_func.py", "w", encoding="utf-8") as f:
    f.write("import numpy\n\n")
    f.write("def func_eq2(theta, theta_dot, phi, phi_dot, tau, f_d):\n")
    f.write(f"    return {code_str}\n")



# _Aと_Bを 'matrices.npz' という1つのファイルに保存
# (A=_A とすることで、読み込み時のキー名を 'A' に指定できます)
np.savez("matrices.npz", A=_A, B=_B)
