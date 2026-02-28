// madwick_1axis.h

#ifndef MADWICK_1AXIS_H
#define MADWICK_1AXIS_H

class Madgwick1Axis
{
public:
    Madgwick1Axis(float beta = 0.1f, float accel_scale = 1.0f, float min_norm = 0.95f, float max_norm = 1.05f);
    void update(float ax, float ay, float gz, unsigned long currentTime);
    float getAngle() const;
    void reset();

private:
    float beta;        // フィルタのゲイン
    float accel_scale; // 重力加速度を1.0に合わせるためのスケール
    float theta;       // 推定される角度（ラジアン）

    float min_norm; // 加速度補正を有効にする最小ノルム
    float max_norm; // 加速度補正を有効にする最大ノルム

    // 正規化された加速度
    float ax_norm;
    float ay_norm;

    // ジャイロの角速度
    float theta_dot;

    // 最後の更新時間（ミリ秒）
    unsigned long lastUpdateTime;
};

#endif