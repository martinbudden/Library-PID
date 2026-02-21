# pragma once

struct pid_constants_t {
    float kp; // proportional gain
    float ki; // integral gain
    float kd; // derivative gain
    float ks; // setpoint gain
    float kk; // setpoint derivative gain ('kick')
};

struct pid_error_t {
    float p;
    float i;
    float d;
    float s;
    float k;
};

/*!
PID controller with Feedforward (open loop) control.

Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.

(In the "dependent PID" notation Kc, tauI, and tauD parameters are used, where kp = Kc, ki = Kc/tauI, kd = Kc*tauD)
*/
class PIDF {
public:
    explicit inline PIDF(const pid_constants_t& pid) : _pid {pid.kp, pid.ki, pid.kd, pid.ks, pid.kk}, _ki_saved(pid.ki)  {}
    inline PIDF() : PIDF({0.0F, 0.0F, 0.0F, 0.0F, 0.0F}) {}
public:
    inline void setP(float p) { _pid.kp = p; }
    inline void setI(float i) { _pid.ki = i; _ki_saved = _pid.ki; }
    inline void setD(float d) { _pid.kd = d; }
    inline void setS(float s) { _pid.ks = s; }
    inline void setK(float k) { _pid.kk = k; }
    inline void setPID(const pid_constants_t& pid) { _pid = pid; _ki_saved = _pid.ki; }
    inline float getP() const { return _pid.kp; }
    inline float getI() const { return _ki_saved; } // returns the set value of ki, whether integration is turned on or not
    inline float getD() const { return _pid.kd; }
    inline float getS() const { return _pid.ks; }
    inline float getK() const { return _pid.kk; }
    inline const pid_constants_t getPID() const { return pid_constants_t { _pid.kp, _ki_saved, _pid.kd, _pid.ks, _pid.kk }; }  // returns the set value of ki, whether integration is turned on or not

    inline void reset_integral() { _error_integral = 0.0F; }
    inline void switchIntegrationOff() { _ki_saved = _pid.ki; _pid.ki = 0.0F; _error_integral = 0.0F; }
    inline void switchIntegrationOn() { _pid.ki = _ki_saved; _error_integral = 0.0F; }

    inline void set_integral_max(float integral_max) { _integral_max = integral_max; }
    inline void set_integral_min(float integral_min) { _integral_min = integral_min; }
    inline void set_integral_limit(float integral_limit) { _integral_max = integral_limit; _integral_min = -integral_limit; }
    inline void set_integral_threshold(float integral_threshold) { _integral_threshold = integral_threshold; }
    inline void setoutput_saturation_value(float output_saturation_value) { _output_saturation_value = output_saturation_value; }

    inline void setSetpoint(float setpoint) { _setpoint_previous = _setpoint; _setpoint = setpoint; }
    inline void setSetpoint(float setpoint, float delta_t) {
        _setpoint_previous = _setpoint;
        _setpoint = setpoint;
        _setpoint_derivative = (_setpoint - _setpoint_previous)/delta_t;
    }
    inline void setsetpoint_derivative(float setpoint_derivative) { _setpoint_derivative = setpoint_derivative; }

    inline float getSetpoint() const { return _setpoint; }
    inline float getPreviousSetpoint() const { return _setpoint_previous; }
    inline float getSetpointDelta() const { return _setpoint - _setpoint_previous; }

    inline float getPreviousMeasurement() const { return _measurement_previous; } //!< get previous measurement, useful for DTerm filtering

    inline float update(float measurement, float delta_t) {
        return update_delta(measurement, measurement - _measurement_previous, delta_t);
    }
    inline float update_delta(float measurement, float measurementDelta, float delta_t) {
        return update_delta_iterm(measurement, measurementDelta, _setpoint - measurement, delta_t);
    }

    float update_delta_iterm(float measurement, float measurementDelta, float iterm_error, float delta_t);

    float update_sp(float measurement);

    float update_spi(float measurement, float delta_t);
    float update_skpi(float measurement, float delta_t) { return update_spi(measurement, delta_t) + _pid.kk*_setpoint_derivative; }

    float update_spd(float measurement, float measurementDelta, float delta_t);
    float update_skpd(float measurement, float measurementDelta, float delta_t) { return update_spd(measurement, measurementDelta, delta_t) + _pid.kk*_setpoint_derivative; }

    // accessor functions to obtain error values
    pid_error_t get_error() const;
    pid_error_t get_error_raw() const;
    inline float get_error_p() const { return _error_previous*_pid.kp; }
    inline float get_error_i() const { return _error_integral; } // _erroIntegral is already multiplied by _pid.ki
    inline float get_error_d() const { return _error_derivative*_pid.kd; }
    inline float get_error_s() const { return _setpoint*_pid.ks; }
    inline float get_error_k() const { return _setpoint_derivative*_pid.kk; }

    inline float get_error_raw_p() const { return _error_previous; }
    inline float get_error_raw_i() const { return (_pid.ki == 0.0F) ? 0.0F : _error_integral / _pid.ki; }
    inline float get_error_raw_d() const { return _error_derivative; }
    inline float get_error_raw_s() const { return _setpoint; }
    inline float get_error_raw_k() const { return _setpoint_derivative; }

    inline float get_previous_error() const { return _error_previous; } //!< get previous error, for test code

    void resetAll(); //!< reset all, for test code
private:
    pid_constants_t _pid;
    float _ki_saved; //!< saved value of _pid.ki, so integration can be switched on and off
    float _measurement_previous {0.0F};

    float _setpoint {0.0F};
    float _setpoint_previous {0.0F};
    float _setpoint_derivative {0.0F};

    float _error_derivative {0.0F};
    float _error_integral {0.0F};
    float _error_previous {0.0F};

    // integral anti-windup parameters
    float _integral_max {0.0F}; //!< Integral windup limit for positive integral
    float _integral_min {0.0F}; //!< Integral windup limit for negative integral
    float _integral_threshold {0.0F}; //!< Threshold for PID integration. Can be set to avoid integral wind-up due to movement in motor's backlash zone.
    float _output_saturation_value {0.0F}; //!< For integral windup control
};
