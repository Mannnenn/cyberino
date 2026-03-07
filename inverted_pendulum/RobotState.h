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
     * @brief 左右のホイール速度から状態を更新する
     *
     * ホイール角度はホイール速度を台形積分で更新する。
     *
     * @param wheel_speed_l 左ホイール速度 [rad/s]
     * @param wheel_speed_r 右ホイール速度 [rad/s]
     * @param dt            前回呼び出しからの経過時間 [s]
     */
    void updateWheel(float wheel_speed_l, float wheel_speed_r, float dt);

    /**
     * @brief 振子角速度・ヨーレイトを更新する
     *
     * @param pendulum_angular_velocity 振子角速度 [rad/s]
     * @param yaw_rate                  ヨーレイト [rad/s]（省略時は 0）
     */
    void updatePendulumAngularVelocity(float pendulum_angular_velocity, float yaw_rate = 0.0f);

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
    float getWheelSpeedL() const { return wheel_speed_l_; }
    float getWheelSpeedR() const { return wheel_speed_r_; }
    float getWheelAngle() const { return wheel_angle_; }
    float getWheelAngleL() const { return wheel_angle_l_; }
    float getWheelAngleR() const { return wheel_angle_r_; }
    float getPendulumAngle() const { return pendulum_angle_; }
    float getPendulumAngularVelocity() const { return pendulum_angular_velocity_; }
    float getYawRate() const { return yaw_rate_; }

private:
    float wheel_speed_;                   ///< ホイール速度 [rad/s]
    float wheel_speed_l_, wheel_speed_r_; ///< 左右のホイール速度 [rad/s]
    float wheel_angle_;                   ///< ホイール角度（速度の積分値） [rad]
    float wheel_angle_l_, wheel_angle_r_; ///< 左右のホイール角度（速度の積分値） [rad]
    float pendulum_angle_;                ///< 振子角度 [rad]
    float pendulum_angular_velocity_;     ///< 振子角速度 [rad/s]

    float prev_wheel_speed_; ///< 台形積分用：前回のホイール速度 [rad/s]
    float yaw_rate_;         ///< ヨーレイト [rad/s]
};
