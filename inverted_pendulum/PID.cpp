#include "PID.h"

PID::PID(float kp, float ki, float kd, float tau_f,
         float out_min, float out_max)
    : _kp(kp), _ki(ki), _kd(kd), _tau_f(tau_f),
      _out_min(out_min), _out_max(out_max)
{
    reset();
}

void PID::reset()
{
    _integral = 0.0f;
    _prev_error = 0.0f;
    _ud = 0.0f;
    _up = 0.0f;
    _ui = 0.0f;
    _initialized = false;
}

float PID::compute(float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    const float error = setpoint - measurement;

    // --- 比例項 ---
    _up = _kp * error;

    // --- 不完全微分項（一次遅れフィルタ付き微分） ---
    // D(s) = Kd * s / (1 + tau_f * s)
    // 後退オイラー離散化:
    //   u_d[k] = tau_f/(tau_f + dt) * u_d[k-1] + Kd/(tau_f + dt) * (e[k] - e[k-1])
    if (!_initialized)
    {
        // 初回は微分を0にしてウォームアップ
        _prev_error = error;
        _ud = 0.0f;
        _initialized = true;
    }
    else
    {
        const float alpha = _tau_f / (_tau_f + dt);
        _ud = alpha * _ud + (_kd / (_tau_f + dt)) * (error - _prev_error);
    }

    // --- 積分項（アンチワインドアップ付き） ---
    // 事前に出力飽和するか確認し、飽和方向への積分のみ抑制する
    const float output_no_integral = _up + _ud;
    const bool saturated_high = (output_no_integral >= _out_max);
    const bool saturated_low = (output_no_integral <= _out_min);

    if (!((saturated_high && error > 0.0f) || (saturated_low && error < 0.0f)))
    {
        _integral += _ki * error * dt;
    }

    // 積分項自体もクランプ（ワインドアップ対策の二重保護）
    _integral = constrain(_integral, _out_min, _out_max);
    _ui = _integral;

    // --- 出力合算・クランプ ---
    const float output = constrain(_up + _ui + _ud, _out_min, _out_max);

    _prev_error = error;
    return output;
}

float PID::update_pd_measurement_deriv(float setpoint, float measurement,
                                       float measurement_deriv, float dt)
{
    const float error = setpoint - measurement;

    _up = _kp * error;
    _ud = -_kd * measurement_deriv; // D項: 実測微分値に負号（測定値増加時に抑制）
    _ui = 0.0f;

    return constrain(_up + _ud, _out_min, _out_max);
}

float PID::update_pi(float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    const float error = setpoint - measurement;

    _up = _kp * error;
    _ud = 0.0f;

    // アンチワインドアップ: P項が既に飽和しており，かつ誤差が飽和方向の場合は積分しない
    const bool saturated_high = (_up >= _out_max);
    const bool saturated_low = (_up <= _out_min);

    if (!((saturated_high && error > 0.0f) || (saturated_low && error < 0.0f)))
    {
        _integral += _ki * error * dt;
    }

    // 積分項自体もクランプ
    _integral = constrain(_integral, _out_min, _out_max);
    _ui = _integral;

    return constrain(_up + _ui, _out_min, _out_max);
}

void PID::resetIntegral()
{
    _integral = 0.0f;
    _ui = 0.0f;
}

void PID::setGains(float kp, float ki, float kd, float tau_f)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _tau_f = tau_f;
}

void PID::setOutputLimits(float out_min, float out_max)
{
    _out_min = out_min;
    _out_max = out_max;
}
