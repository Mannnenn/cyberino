import csv
import os
import socket

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 設定項目 ---
# ESP32のスケッチで指定したポート番号と一致させる
UDP_IP = "0.0.0.0"  # 全てのネットワークインターフェースで待ち受け
UDP_PORT = 3333
CSV_FILE = "sensor_data.csv"

# グラフ表示用のデータ保持
history = {
    "time": [],
    "pendulum_angle": [],
    "pendulum_angular_velocity": [],
    "wheel_speed": [],
    "wheel_angle": [],
    "target_torque": [],
}


def main():
    # CSVファイルの初期化
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "Timestamp_ms",
                    "PendulumAngle",
                    "PendulumAngularVelocity",
                    "WheelSpeed",
                    "WheelAngle",
                    "TargetTorque",
                ]
            )

    # ソケットの作成 (AF_INET: IPv4, SOCK_DGRAM: UDP)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)  # 非ブロッキングモードに設定

    try:
        # ポートをバインド
        sock.bind((UDP_IP, UDP_PORT))
        print(f"UDPサーバー起動: ポート {UDP_PORT} で待機中...")
        print(f"データを {CSV_FILE} に保存します。")

        # グラフの設定
        fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True)

        (line_p_angle,) = axes[0].plot([], [], "r-", label="Pendulum Angle (rad)")
        axes[0].set_ylabel("P Angle [rad]")
        axes[0].legend(loc="upper right")
        axes[0].grid(True)

        (line_p_vel,) = axes[1].plot([], [], "b-", label="Pendulum AngVel (rad/s)")
        axes[1].set_ylabel("P AngVel [rad/s]")
        axes[1].legend(loc="upper right")
        axes[1].grid(True)

        (line_w_speed,) = axes[2].plot([], [], "m-", label="Wheel Speed (rad/s)")
        axes[2].set_ylabel("W Speed [rad/s]")
        axes[2].legend(loc="upper right")
        axes[2].grid(True)

        (line_w_angle,) = axes[3].plot([], [], "c-", label="Wheel Angle (rad)")
        axes[3].set_ylabel("W Angle [rad]")
        axes[3].legend(loc="upper right")
        axes[3].grid(True)

        (line_torque,) = axes[4].plot([], [], "g-", label="Target Torque (Nm)")
        axes[4].set_xlabel("Time (ms)")
        axes[4].set_ylabel("Torque [Nm]")
        axes[4].legend(loc="upper right")
        axes[4].grid(True)

        def update(frame):
            try:
                while True:  # 溜まっているパケットをすべて読み出す
                    data, addr = sock.recvfrom(4096)
                    message = data.decode("utf-8")

                    # 複数行のデータが含まれる可能性があるため、改行で分割
                    lines = message.strip().split("\n")

                    with open(CSV_FILE, "a", newline="") as f:
                        writer = csv.writer(f)
                        for line_data in lines:
                            parts = line_data.split(",")
                            if len(parts) >= 6:
                                try:
                                    t = float(parts[0])
                                    p_angle = float(parts[1])
                                    p_vel = float(parts[2])
                                    w_speed = float(parts[3])
                                    w_angle = float(parts[4])
                                    t_torque = float(parts[5])

                                    history["time"].append(t)
                                    history["pendulum_angle"].append(p_angle)
                                    history["pendulum_angular_velocity"].append(p_vel)
                                    history["wheel_speed"].append(w_speed)
                                    history["wheel_angle"].append(w_angle)
                                    history["target_torque"].append(t_torque)

                                    writer.writerow(
                                        [t, p_angle, p_vel, w_speed, w_angle, t_torque]
                                    )
                                except ValueError:
                                    print(f"データ形式エラー: {line_data}")

                # 表示範囲を最新の200点に制限
                max_points = 200
                for key in history:
                    if len(history[key]) > max_points:
                        history[key] = history[key][-max_points:]

            except BlockingIOError:
                pass  # データがない場合は何もしない

            if history["time"]:
                line_p_angle.set_data(history["time"], history["pendulum_angle"])
                line_p_vel.set_data(
                    history["time"], history["pendulum_angular_velocity"]
                )
                line_w_speed.set_data(history["time"], history["wheel_speed"])
                line_w_angle.set_data(history["time"], history["wheel_angle"])
                line_torque.set_data(history["time"], history["target_torque"])

                for ax in axes:
                    ax.relim()
                    ax.autoscale_view()
            return line_p_angle, line_p_vel, line_w_speed, line_w_angle, line_torque

        ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    except KeyboardInterrupt:
        print("\nサーバーを停止します。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
