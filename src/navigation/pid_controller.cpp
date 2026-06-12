// =============================================================================
//  pid_controller.cpp  -  PID Controller Implementation
// =============================================================================

#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd,
                             float maxIntegral, float outputLimit)
    : _kp(kp), _ki(ki), _kd(kd),
      _maxIntegral(maxIntegral), _outputLimit(outputLimit) {}

// ---------------------------------------------------------------------------
// Compute PID correction
// ---------------------------------------------------------------------------

float PIDController::compute(float setpoint, float measured) {
    unsigned long now   = micros();
    float dt            = (_lastTime == 0) ? 0.001f
                          : (float)(now - _lastTime) * 1e-6f;
    _lastTime           = now;

    // Clamp dt to avoid windup on first call or after long pauses
    if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;

    float error = setpoint - measured;

    // Proportional
    float pTerm = _kp * error;

    // Integral with anti-windup clamping
    _integral += error * dt;
    _integral  = constrain(_integral, -_maxIntegral, _maxIntegral);
    float iTerm = _ki * _integral;

    // Derivative (on measurement, not error - avoids derivative kick on setpoint change)
    float dTerm = -_kd * (measured - (_prevError == 0.0f ? measured : (setpoint - _prevError))) / dt;
    // Simpler derivative on error:
    float derivative = (error - _prevError) / dt;
    dTerm            = _kd * derivative;

    _prevError = error;

    float output = pTerm + iTerm + dTerm;
    return constrain(output, -_outputLimit, _outputLimit);
}

// ---------------------------------------------------------------------------
// Reset state
// ---------------------------------------------------------------------------

void PIDController::reset() {
    _prevError = 0.0f;
    _integral  = 0.0f;
    _lastTime  = 0;
}

// ---------------------------------------------------------------------------
// Update gains at runtime
// ---------------------------------------------------------------------------

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
