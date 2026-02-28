#pragma once
#include <Arduino.h>

/**
 * @brief 不完全微分（一次遅れフィルタ付き微分）によるPID制御クラス
 *
 * D項の伝達関数: D(s) = Kd * s / (1 + tau_f * s)
 * 離散化（後退オイラー）:
 *   u_d[k] = tau_f/(tau_f + dt) * u_d[k-1] + Kd/(tau_f + dt) * (e[k] - e[k-1])
 *
 * アンチワインドアップ: 出力飽和時に積分項をクランプ
 */
class PID
{
public:
    /**
     * @param kp      比例ゲイン
     * @param ki      積分ゲイン
     * @param kd      微分ゲイン
     * @param tau_f   不完全微分フィルタ時定数 [s]  (tau_f = Td/N, Nは5〜20程度)
     * @param out_min 出力下限
     * @param out_max 出力上限
     */
    PID(float kp, float ki, float kd, float tau_f,
        float out_min, float out_max);

    /**
     * @brief 内部状態（積分・前回誤差・微分状態）をリセット
     */
    void reset();

    /**
     * @brief 一般PID演算（不完全微分D項付き）
     * @param setpoint    目標値
     * @param measurement 実測値
     * @param dt          制御周期 [s]
     * @return 制御出力
     */
    float compute(float setpoint, float measurement, float dt);

    /**
     * @brief 微分先行型PD制御（姿勢制御向け）
     *
     * D項に誤差微分ではなく実測値の微分（センサ直値）を使用する。
     * 目標値変化時のD項スパイクを回避できる。
     * 不完全微分フィルタは不要（センサ側で既にフィルタ済みを前提）。
     *
     * u = Kp * (setpoint - measurement) - Kd * measurement_deriv
     *
     * @param setpoint         目標値（例: 目標傾斜角）
     * @param measurement      実測値（例: 現在の傾斜角）
     * @param measurement_deriv 実測値の微分（例: IMUジャイロ角速度）[単位/s]
     * @param dt               制御周期 [s]（本実装では未使用、将来拡張用）
     * @return 制御出力
     */
    float update_pd_measurement_deriv(float setpoint, float measurement,
                                      float measurement_deriv, float dt);

    /**
     * @brief アンチワインドアップ付きPI制御（速度制御向け）
     *
     * 積分ワインドアップ対策: 出力飽和かつ誤差が飽和方向の場合は積分を停止。
     * さらに積分項自体も[-out_max, out_max]にクランプ。
     *
     * @param setpoint    目標値（例: 目標速度）
     * @param measurement 実測値（例: 現在の速度）
     * @param dt          制御周期 [s]
     * @return 制御出力
     */
    float update_pi(float setpoint, float measurement, float dt);

    // ゲイン・制限値の動的変更
    void setGains(float kp, float ki, float kd, float tau_f);
    void setOutputLimits(float out_min, float out_max);

    /** compute() / update_pi() の積分状態をリセット */
    void resetIntegral();

    // 内部状態の参照
    float getProportional() const { return _up; }
    float getIntegral() const { return _ui; }
    float getDerivative() const { return _ud; }
    float getLastError() const { return _prev_error; }

private:
    float _kp, _ki, _kd, _tau_f;
    float _out_min, _out_max;

    float _integral;   ///< 積分項の累積値
    float _prev_error; ///< 前回の誤差（微分計算用）
    float _ud;         ///< 不完全微分項の状態（前回値）

    // デバッグ用：各成分の最新値
    float _up, _ui;

    bool _initialized; ///< 初回呼び出しフラグ（微分計算のウォームアップ）
};
