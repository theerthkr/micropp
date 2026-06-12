// =============================================================================
//  motor_driver.cpp  -  TB6612FNG Dual Motor Driver Implementation
// =============================================================================

#include "motor_driver.h"
#include "../config.h"

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void MotorDriver::begin() {
    // Configure direction pins as outputs
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);

    // Configure PWM channels using ESP32 LEDC
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWMA, PWM_CHANNEL_A);
    ledcAttachPin(PIN_PWMB, PWM_CHANNEL_B);

    // Start in braked, disabled state
    brake();
    enable();
}

// ---------------------------------------------------------------------------
// Public speed setters
// ---------------------------------------------------------------------------

void MotorDriver::setLeft(int speed) {
    _setMotorA(constrain(speed, -255, 255));
}

void MotorDriver::setRight(int speed) {
    _setMotorB(constrain(speed, -255, 255));
}

void MotorDriver::setSpeed(int leftSpeed, int rightSpeed) {
    setLeft(leftSpeed);
    setRight(rightSpeed);
}

// ---------------------------------------------------------------------------
// Stop modes
// ---------------------------------------------------------------------------

void MotorDriver::brake() {
    // TB6612FNG active brake: both IN pins HIGH, PWM = 255
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(PWM_CHANNEL_A, 255);

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(PWM_CHANNEL_B, 255);
}

void MotorDriver::coast() {
    // TB6612FNG coast: both IN pins LOW
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);

    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PWM_CHANNEL_B, 0);
}

void MotorDriver::standby() {
    digitalWrite(PIN_STBY, LOW);
    _enabled = false;
}

void MotorDriver::enable() {
    digitalWrite(PIN_STBY, HIGH);
    _enabled = true;
}

// ---------------------------------------------------------------------------
// Blocking turn maneuvers (time-based)
// ---------------------------------------------------------------------------

void MotorDriver::turnLeft90() {
    // Left motor reverse, Right motor forward - pivot left
    setSpeed(-MOTOR_TURN_SPEED, MOTOR_TURN_SPEED);
    delay(TURN_90_MS);
    brake();
    delay(POST_TURN_SETTLE_MS);
}

void MotorDriver::turnRight90() {
    // Left motor forward, Right motor reverse - pivot right
    setSpeed(MOTOR_TURN_SPEED, -MOTOR_TURN_SPEED);
    delay(TURN_90_MS);
    brake();
    delay(POST_TURN_SETTLE_MS);
}

void MotorDriver::turnAround() {
    // 180-degree point turn (spin right by default)
    setSpeed(MOTOR_TURN_SPEED, -MOTOR_TURN_SPEED);
    delay(TURN_180_MS);
    brake();
    delay(POST_TURN_SETTLE_MS);
}

void MotorDriver::driveForward(uint16_t durationMs, int speed) {
    int s = (speed < 0) ? MOTOR_BASE_SPEED : constrain(speed, 0, MOTOR_MAX_SPEED);
    setSpeed(s, s);
    delay(durationMs);
    brake();
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void MotorDriver::_setMotorA(int speed) {
    if (speed > 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        _writePWM(PWM_CHANNEL_A, (uint8_t)speed);
    } else if (speed < 0) {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        _writePWM(PWM_CHANNEL_A, (uint8_t)(-speed));
    } else {
        // Active brake at speed 0
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, HIGH);
        _writePWM(PWM_CHANNEL_A, 255);
    }
}

void MotorDriver::_setMotorB(int speed) {
    if (speed > 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        _writePWM(PWM_CHANNEL_B, (uint8_t)speed);
    } else if (speed < 0) {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        _writePWM(PWM_CHANNEL_B, (uint8_t)(-speed));
    } else {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, HIGH);
        _writePWM(PWM_CHANNEL_B, 255);
    }
}

void MotorDriver::_writePWM(uint8_t channel, uint8_t duty) {
    ledcWrite(channel, duty);
}
