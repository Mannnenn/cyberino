// madwick_1axis.h

#ifndef MADWICK_1AXIS_H
#define MADWICK_1AXIS_H

class Madgwick1Axis
{
public:
    Madgwick1Axis(float beta = 0.1f);
    void update(float ax, float ay, float gz, unsigned long currentTime);
    float getAngle() const;
    void reset();

private:
    float beta;  // フィルタのゲイン
    float theta; // 推定される角度（ラジアン）

    // 正規化された加速度
    float ax_norm;
    float ay_norm;

    // ジャイロの角速度
    float theta_dot;

    // 最後の更新時間（ミリ秒）
    unsigned long lastUpdateTime;
};

#endif