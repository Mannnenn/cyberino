#ifndef STATE_FEEDBACK_H
#define STATE_FEEDBACK_H

#include "RobotState.h"

class StateFeedback
{
public:
    StateFeedback(float kAngle, float kAngularVelocity, float kWheelSpeed, float kWheelAngle);

    /**
     * @brief 左右のホイールトルクを保持する構造体
     */
    struct Torque
    {
        float left;
        float right;
    };

    /**
     * @brief 状態フィードバックによる目標トルクを計算する
     *
     * @param state ロボットの状態
     * @return Torque 計算された左右の目標トルク [Nm]
     */
    Torque calculateTorque(const RobotState &state);

    // ゲインの更新用メソッド（必要に応じて）
    void setGains(float kAngle, float kAngularVelocity, float kWheelSpeed, float kWheelAngle);

private:
    float _kAngle;
    float _kAngularVelocity;
    float _kWheelSpeed;
    float _kWheelAngle;
};

#endif // STATE_FEEDBACK_H
