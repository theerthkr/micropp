#pragma once
// =============================================================================
//  motor_driver.h  -  TB6612FNG Dual Motor Driver Interface
//
//  Wraps the TB6612FNG H-bridge module for the two N20-6V-600 RPM motors.
//  Uses ESP32 LEDC hardware PWM for smooth, flicker-free speed control.
//
//  Speed convention: positive = forward, negative = reverse, 0 = brake.
//  Speed range: -255 to +255 (maps directly to 8-bit PWM duty cycle).
// =============================================================================

#include <Arduino.h>

class MotorDriver {
public:
    // Initialize GPIO and LEDC PWM channels. Must be called in setup().
    void begin();

    // Set individual motor speeds.
    // speed: -255 (full reverse) to +255 (full forward). 0 = active brake.
    void setLeft(int speed);
    void setRight(int speed);

    // Convenience: set both motors simultaneously.
    void setSpeed(int leftSpeed, int rightSpeed);

    // Active brake: shorts both motor terminals via the H-bridge.
    // Much faster stop than coasting.
    void brake();

    // Coast stop: tri-state the outputs (motors spin down freely).
    void coast();

    // Put TB6612FNG in standby (STBY low). Reduces power draw when idle.
    void standby();

    // Wake from standby (STBY high).
    void enable();

    // Point turns - block until turn is complete.
    void turnLeft90();
    void turnRight90();
    void turnAround();    // 180-degree U-turn

    // Move forward for a fixed distance (time-based).
    void driveForward(uint16_t durationMs, int speed = -1);

private:
    void _setMotorA(int speed);   // Left  motor
    void _setMotorB(int speed);   // Right motor
    void _writePWM(uint8_t channel, uint8_t duty);

    bool _enabled = false;
};
