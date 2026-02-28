// madwick_1axis.cpp

#include "madwick_1axis.h"
#include <math.h>

Madgwick1Axis::Madgwick1Axis(float beta, float accel_scale, float min_norm, float max_norm)
    : beta(beta), accel_scale(accel_scale), theta(0), min_norm(min_norm), max_norm(max_norm), ax_norm(0), ay_norm(0), theta_dot(0), lastUpdateTime(0)
{
}

void Madgwick1Axis::update(float ax, float ay, float gz, unsigned long currentTime)
{
    // 初回呼び出しの場合は時間のみ記録して終了
    if (lastUpdateTime == 0)
    {
        lastUpdateTime = currentTime;
        return;
    }

    // 前回からの経過時間を計算（秒単位）
    float deltaTime = (currentTime - lastUpdateTime) / 1000000.0f;
    lastUpdateTime = currentTime;

    // 経過時間が異常値の場合はスキップ
    if (deltaTime <= 0 || deltaTime > 1.0f)
        return;

    // 重力加速度を1.0に合わせるためのスケール補正
    ax *= accel_scale;
    ay *= accel_scale;

    // ジャイロセンサーの角速度をラジアンに変換
    theta_dot = gz * (M_PI / 180.0f);

    // 加速度センサーのノルムを計算
    float norm = sqrt(ax * ax + ay * ay);

    float angle_dot;

    // 加速度のノルムが設定範囲内の時のみ、加速度による補正（傾斜計としての利用）を行う
    // 回転中心から離れている場合、遠心力が加わってノルムが1Gから外れるため
    if (norm >= min_norm && norm <= max_norm)
    {
        ax_norm = ax / norm;
        ay_norm = ay / norm;

        float grad = ay_norm * cos(theta) - ax_norm * sin(theta);
        angle_dot = theta_dot - beta * grad;
    }
    else
    {
        // ノルムが範囲外（激しい動きや自由落下など）の時はジャイロのみで積分
        angle_dot = theta_dot;
    }

    // Madgwickフィルタの更新
    this->theta = theta + angle_dot * deltaTime;
}

float Madgwick1Axis::getAngle() const
{
    return theta;
}

void Madgwick1Axis::reset()
{
    theta = 0;
    ax_norm = 0;
    ay_norm = 0;
    theta_dot = 0;
    lastUpdateTime = 0;
}