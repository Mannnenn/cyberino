#pragma once
#include <Arduino.h>

/**
 * @brief 倒立振子ロボットの状態を保持・更新するクラス
 *
 * 保持する状態:
 *   - ホイール速度          [rad/s]
 *   - ホイール角度（積分値）  [rad]
 *   - 振子角度             [rad]
 *   - 振子角速度            [rad/s]
 */
class RobotState
{
public:
    RobotState();

    /**
     * @brief ホイール速度・角度を更新する
     *
     * ホイール角度はホイール速度を台形積分で更新する。
     *
     * @param wheel_speed ホイール速度（左右平均） [rad/s]
     * @param dt          前回呼び出しからの経過時間 [s]
     */
    void updateWheel(float wheel_speed, float dt);

    /**
     * @brief 振子角速度を更新する
     *
     * @param pendulum_angular_velocity 振子角速度 [rad/s]
     */
    void updatePendulumAngularVelocity(float pendulum_angular_velocity);

    /**
     * @brief 振子角度を更新する
     *
     * @param pendulum_angle 振子角度 [rad]
     */
    void updatePendulumAngle(float pendulum_angle);

    /**
     * @brief 内部状態をすべてゼロにリセットする
     */
    void reset();

    // --- ゲッター ---
    float getWheelSpeed() const { return wheel_speed_; }
    float getWheelAngle() const { return wheel_angle_; }
    float getPendulumAngle() const { return pendulum_angle_; }
    float getPendulumAngularVelocity() const { return pendulum_angular_velocity_; }

private:
    float wheel_speed_;               ///< ホイール速度 [rad/s]
    float wheel_angle_;               ///< ホイール角度（速度の積分値） [rad]
    float pendulum_angle_;            ///< 振子角度 [rad]
    float pendulum_angular_velocity_; ///< 振子角速度 [rad/s]

    float prev_wheel_speed_; ///< 台形積分用：前回のホイール速度 [rad/s]
};
