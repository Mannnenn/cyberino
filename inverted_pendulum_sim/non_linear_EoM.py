import numpy as np

from generated_eq1_func import func_eq1
from generated_eq2_func import func_eq2


def func(t, x, tau, f_d):
    """
    状態変数 x = [theta, theta_dot, phi, phi_dot] と仮定
    """
    theta = x[0]
    theta_dot = x[1]
    phi = x[2]
    phi_dot = x[3]

    # 微分項の計算
    dxdt = np.zeros_like(x)

    dxdt[0] = theta_dot
    dxdt[1] = func_eq1(theta, theta_dot, phi, phi_dot, tau, f_d)
    dxdt[2] = phi_dot
    dxdt[3] = func_eq2(theta, theta_dot, phi, phi_dot, tau, f_d)

    return dxdt


# # --- 以下、シミュレーション実行例 ---
# from scipy.integrate import solve_ivp

# t_span = (0, 10)
# x0 = [0.1, 0, 0, 0]  # 初期値
# tau_val = 0.0  # 入力トルク
# fd_val = 0.0  # 外力

# sol = solve_ivp(func, t_span, x0, args=(tau_val, fd_val))

# print("Time points:", sol.t)
# print("State trajectories:", sol.y)
