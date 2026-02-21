#include "PIDF.h"
#include <cmath>


pid_error_t PidController::get_error() const
{
    return pid_error_t {
        .p = _error_previous*_pid.kp,
        .i = _error_integral, // _erroIntegral is already multiplied by _pid.ki
        .d = _error_derivative*_pid.kd,
        .s = _setpoint*_pid.ks,
        .k = _setpoint_derivative*_pid.kk
    };
}

pid_error_t PidController::get_error_raw() const
{
    return pid_error_t {
        .p = _error_previous,
        .i = (_pid.ki == 0.0F) ? 0.0F : _error_integral / _pid.ki,
        .d = _error_derivative,
        .s = _setpoint,
        .k = _setpoint_derivative
    };
}

void PidController::resetAll()
{
    _setpoint = 0.0F;
    _setpoint_previous = 0.0F;
    _setpoint_derivative = 0.0F;
    _error_derivative = 0.0F;
    _error_integral = 0.0F;
    _error_previous = 0.0F;
    _measurement_previous = 0.0F;
}

/*!
Calculate PID output using the provided measurementRate and ITerm error.
This allows the measurementRate to be filtered and the ITerm error to be attenuated
before the PID update is called.
*/
float PidController::update_delta_iterm(float measurement, float measurementDelta, float iterm_error, float delta_t) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurement_previous = measurement;
    const float error = _setpoint - measurement;
    _error_derivative = -measurementDelta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta
    // Partial PID sum, excludes ITerm
    // has additional S setpoint(openloop) and F feedforward(setpoint derivative) terms
    //                       P             +  D                       + S                 + K (no ITerm)
    const float partial_sum = _pid.kp*error + _pid.kd*_error_derivative + _pid.ks*_setpoint + _pid.kk*_setpoint_derivative;

    if (_integral_threshold == 0.0F || fabsf(error) >= _integral_threshold) {
        // "integrate" the error
        _error_integral += _pid.ki*iterm_error*delta_t; // Euler integration
        //_error_integral += _pid.ki*0.5F*(iterm_error + _error_previous)*delta_t; // integration using trapezoid rule
        // Anti-windup via integral clamping
        if (_integral_max > 0.0F && _error_integral > _integral_max) {
            _error_integral = _integral_max;
        } else if (_integral_min < 0.0F && _error_integral < _integral_min) {
            _error_integral = _integral_min;
        }
    }
    _error_previous = error;

    if (_output_saturation_value > 0.0F) {
        // Anti-windup by avoiding output saturation.
        // Check if partial_sum + _error_integral saturates the output
        // If so, the excess value above saturation does not help convergence to the setpoint and will result in
        // overshoot when the P value eventually comes down.
        // So limit the _error_integral to a value that avoids output saturation.
        if (_error_integral > _output_saturation_value - partial_sum) {
            _error_integral = std::fmax(_output_saturation_value - partial_sum, 0.0F);
        } else if (_error_integral < -_output_saturation_value - partial_sum) {
            _error_integral = std::fmin(-_output_saturation_value - partial_sum, 0.0F);
        }
    }


    // The PID calculation with additional S setpoint(openloop) and F feedforward(setpoint derivative) terms
    //                   P+D+S+F    +  I
    const float output = partial_sum + _error_integral;

    return output;
}

/*
Optimized update of S and P terms only (P controller).
*/
float PidController::update_sp(float measurement) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurement_previous = measurement;
    const float error = _setpoint - measurement;
    _error_previous = error;

    // The P (no I, no D) calculation with additional S setpoint(openloop) term
    //                   P             + S
    const float output = _pid.kp*error + _pid.ks*_setpoint;

    return output;
}

/*
Optimized update of S, P, and I terms only (PI controller)
*/
float PidController::update_spi(float measurement, float delta_t) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurement_previous = measurement;
    const float error = _setpoint - measurement;
    const float partial_sum = _pid.kp*error + _pid.ks*_setpoint;

    if (_integral_threshold == 0.0F || fabsf(error) >= _integral_threshold) {
        // "integrate" the error
        _error_integral += _pid.ki*error*delta_t; // Euler integration
        //_error_integral += _pid.ki*0.5F*(error + _error_previous)*delta_t; // integration using trapezoid rule
        // Anti-windup via integral clamping
        if (_integral_max > 0.0F && _error_integral > _integral_max) {
            _error_integral = _integral_max;
        } else if (_integral_min < 0.0F && _error_integral < _integral_min) {
            _error_integral = _integral_min;
        }
    }
    _error_previous = error;

    if (_output_saturation_value > 0.0F) {
        // Anti-windup by avoiding output saturation.
        // Check if partial_sum + _error_integral saturates the output
        // If so, the excess value above saturation does not help convergence to the setpoint and will result in
        // overshoot when the P value eventually comes down.
        // So limit the _error_integral to a value that avoids output saturation.
        if (_error_integral > _output_saturation_value - partial_sum) {
            _error_integral = std::fmax(_output_saturation_value - partial_sum, 0.0F);
        } else if (_error_integral < -_output_saturation_value - partial_sum) {
            _error_integral = std::fmin(-_output_saturation_value - partial_sum, 0.0F);
        }
    }

    // The PI (no D) calculation with additional S setpoint(openloop) term
    //                   P + S      +  I
    const float output = partial_sum + _error_integral;

    return output;
}

/*
Optimized update of S, P, and D terms only (PD controller).
*/
float PidController::update_spd(float measurement, float measurementDelta, float delta_t) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurement_previous = measurement;
    const float error = _setpoint - measurement;

    _error_previous = error;

    _error_derivative = -measurementDelta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta

    // The PD (no I) calculation with additional S setpoint(openloop) term
    //                   P             + D                        + S
    const float output = _pid.kp*error + _pid.kd*_error_derivative + _pid.ks*_setpoint;

    return output;
}
