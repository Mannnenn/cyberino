// madwick_1axis.cpp

#include "madwick_1axis.h"
#include <math.h>

Madgwick1Axis::Madgwick1Axis(float beta)
    : beta(beta), theta(0), ax_norm(0), ay_norm(0), theta_dot(0), lastUpdateTime(0)
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

    // 加速度センサーの値を正規化
    float norm = sqrt(ax * ax + ay * ay);
    if (norm == 0)
        return; // ゼロ除算を防止

    ax_norm = ax / norm;
    ay_norm = ay / norm;

    // ジャイロセンサーの角速度をラジアンに変換
    theta_dot = gz * (M_PI / 180.0f);

    // Madgwickフィルタの更新（相補フィルタ）
    // 加速度から推定される角度との誤差を補正しながら積分
    float angle_dot = theta_dot + beta * (ay_norm * cos(theta) - ax_norm * sin(theta));
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