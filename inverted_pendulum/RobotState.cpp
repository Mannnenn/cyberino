#include "RobotState.h"
#include <math.h>

RobotState::RobotState()
    : wheel_speed_(0.0f),
      wheel_speed_l_(0.0f),
      wheel_speed_r_(0.0f),
      wheel_angle_(0.0f),
      wheel_angle_l_(0.0f),
      wheel_angle_r_(0.0f),
      pendulum_angle_(0.0f),
      pendulum_angular_velocity_(0.0f),
      prev_wheel_speed_(0.0f),
      yaw_rate_(0.0f)
{
}

void RobotState::updateWheel(float wheel_speed_l, float wheel_speed_r, float dt)
{
    // 今回の左右平均速度を計算
    float current_avg_speed = (wheel_speed_l + wheel_speed_r) / 2.0f;

    // --- 台形積分による角度の更新 ---
    // 前回の速度（wheel_speed_ 等に保持されている）と今回の速度の平均をとる
    wheel_angle_ += 0.5f * (wheel_speed_ + current_avg_speed) * dt;
    wheel_angle_l_ += 0.5f * (wheel_speed_l_ + wheel_speed_l) * dt;
    wheel_angle_r_ += 0.5f * (wheel_speed_r_ + wheel_speed_r) * dt;

    // --- 状態の更新 ---
    wheel_speed_ = current_avg_speed;
    wheel_speed_l_ = wheel_speed_l;
    wheel_speed_r_ = wheel_speed_r;
}

void RobotState::updatePendulumAngularVelocity(float pendulum_angular_velocity, float yaw_rate)
{
    pendulum_angular_velocity_ = pendulum_angular_velocity * M_PI / 180.0f; // 角速度をラジアンに変換して保存
    yaw_rate_ = yaw_rate * M_PI / 180.0f;                                   // ヨーレイトもラジアンに変換して保存
}

void RobotState::updatePendulumAngle(float pendulum_angle)
{
    pendulum_angle_ = pendulum_angle;
}

void RobotState::reset()
{
    wheel_speed_ = 0.0f;
    wheel_angle_ = 0.0f;
    pendulum_angle_ = 0.0f;
    pendulum_angular_velocity_ = 0.0f;
    prev_wheel_speed_ = 0.0f;
    yaw_rate_ = 0.0f;
}
