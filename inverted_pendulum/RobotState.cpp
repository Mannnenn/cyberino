#include "RobotState.h"

RobotState::RobotState()
    : wheel_speed_(0.0f),
      wheel_angle_(0.0f),
      pendulum_angle_(0.0f),
      pendulum_angular_velocity_(0.0f),
      prev_wheel_speed_(0.0f),
      yaw_rate_(0.0f)
{
}

void RobotState::updateWheel(float wheel_speed, float dt)
{
    // ホイール角度：台形積分（前回と今回の速度の平均 × dt）
    wheel_angle_ += 0.5f * (prev_wheel_speed_ + wheel_speed) * dt;
    prev_wheel_speed_ = wheel_speed;
    wheel_speed_ = wheel_speed;
}

void RobotState::updatePendulumAngularVelocity(float pendulum_angular_velocity, float yaw_rate)
{
    pendulum_angular_velocity_ = pendulum_angular_velocity;
    yaw_rate_ = yaw_rate;
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
