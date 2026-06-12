#pragma once
// =============================================================================
//  pid_controller.h  -  Generic PID Controller
//
//  Used for line-following: maps sensor position error to a motor
//  speed correction value.
//
//  Usage:
//    PIDController pid(KP, KI, KD, MAX_INTEGRAL, OUTPUT_LIMIT);
//    float correction = pid.compute(0.0f, sensorPosition);
//    leftSpeed  = BASE_SPEED - correction;
//    rightSpeed = BASE_SPEED + correction;
// =============================================================================

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float maxIntegral, float outputLimit);

    // Compute PID output.
    // setpoint: desired value (usually 0 for line following)
    // measured: current sensor position
    // Returns: correction to apply to motor speeds
    float compute(float setpoint, float measured);

    // Reset internal state (call before a new run segment)
    void reset();

    // Update gains at runtime if needed
    void setGains(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _maxIntegral;
    float _outputLimit;

    float _prevError   = 0.0f;
    float _integral    = 0.0f;
    unsigned long _lastTime = 0;
};
