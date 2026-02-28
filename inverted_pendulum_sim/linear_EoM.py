import numpy as np

# 1. 保存したnpzファイルから行列を読み込む
# (ファイルが同じディレクトリにある想定です)
_data = np.load("matrices.npz")
_A = _data["A"]
_B = _data["B"]


# ------------------------------------------------
# Linearized state-space equation
# ------------------------------------------------
def func_lin(t, x, tau, f_d):

    # Input vector
    u = np.array([tau, f_d])

    # Calculate derivative of state x
    dxdt = _A @ x + _B @ u

    # Return result
    return dxdt


# # --- 以下、シミュレーション実行例 ---
# from scipy.integrate import solve_ivp

# t_span = (0, 10)
# x0 = [0.1, 0, 0, 0]  # 初期値
# tau_val = 0.0  # 入力トルク
# fd_val = 0.0  # 外力

# # solve_ivpの第1引数には上で定義した func_lin を指定します
# sol = solve_ivp(func_lin, t_span, x0, args=(tau_val, fd_val))

# print("Time points:\n", sol.t)
# print("State trajectories:\n", sol.y)
