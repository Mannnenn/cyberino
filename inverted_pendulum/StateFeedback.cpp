#include "StateFeedback.h"

StateFeedback::StateFeedback(float kAngle, float kAngularVelocity, float kWheelSpeed, float kWheelAngle)
    : _kAngle(kAngle), _kAngularVelocity(kAngularVelocity), _kWheelSpeed(kWheelSpeed), _kWheelAngle(kWheelAngle) {}

StateFeedback::Torque StateFeedback::calculateTorque(const RobotState &state)
{
    float angle = state.getPendulumAngle();
    float angularVelocity = state.getPendulumAngularVelocity();

    // 左右それぞれの状態量を取得
    float wheelSpeedL = state.getWheelSpeedL();
    float wheelAngleL = state.getWheelAngleL();
    float wheelSpeedR = state.getWheelSpeedR();
    float wheelAngleR = state.getWheelAngleR();

    // 状態フィードバックの基本形: u = Kx
    // 左右それぞれのホイールに対してトルクを計算する
    // 今回は単純に1つのゲインセットを左右それぞれに適用する
    Torque torque;
    torque.left = angle * _kAngle + angularVelocity * _kAngularVelocity + wheelSpeedL * _kWheelSpeed + wheelAngleL * _kWheelAngle;
    torque.right = angle * _kAngle + angularVelocity * _kAngularVelocity + wheelSpeedR * _kWheelSpeed + wheelAngleR * _kWheelAngle;

    return torque;
}

void StateFeedback::setGains(float kAngle, float kAngularVelocity, float kWheelSpeed, float kWheelAngle)
{
    _kAngle = kAngle;
    _kAngularVelocity = kAngularVelocity;
    _kWheelSpeed = kWheelSpeed;
    _kWheelAngle = kWheelAngle;
}
