// =============================================================================
//  main.cpp  -  Micro-PP Autonomous Maze Solver
//  Entry point: setup() and loop()
//
//  Boot sequence:
//    1. Serial init (debug)
//    2. LED init
//    3. Motor driver init
//    4. Sensor array init
//    5. Wait for BOOT button press -> calibration -> exploration -> solve
//
//  State flow:
//    IDLE -> (button press) -> CALIBRATING -> EXPLORING -> OPTIMIZING
//         -> SPEED_RUN -> FINISHED
//
//  The BOOT button is GPIO 0 on the ESP32 DevKit V1 (active LOW).
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "motors/motor_driver.h"
#include "sensors/line_sensor.h"
#include "navigation/pid_controller.h"
#include "navigation/maze_solver.h"

// ---------------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------------

MotorDriver  motors;
LineSensor   sensors;
PIDController pid(PID_KP, PID_KI, PID_KD, PID_MAX_INTEGRAL, PID_OUTPUT_LIMIT);
MazeSolver   maze(motors, sensors, pid);

// BOOT button on ESP32 DevKit V1 is GPIO 0 (active LOW, has internal pull-up)
#define PIN_BOOT_BTN 0

// ---------------------------------------------------------------------------
// LED blink helpers
// ---------------------------------------------------------------------------

static void ledBlink(uint8_t times, uint16_t onMs = 100, uint16_t offMs = 100) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(onMs);
        digitalWrite(PIN_LED, LOW);
        delay(offMs);
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------

void setup() {
#if SERIAL_DEBUG
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println("============================================");
    Serial.println("  Micro-PP Autonomous Maze Solver");
    Serial.println("  ESP32 DevKit V1 + TB6612FNG + RLS-08");
    Serial.println("============================================");
#endif

    // Status LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // BOOT button (GPIO 0) - input with internal pull-up
    pinMode(PIN_BOOT_BTN, INPUT_PULLUP);

    // Initialize hardware
    motors.begin();
    sensors.begin();
    maze.begin();

#if SERIAL_DEBUG
    Serial.println("[MAIN] Hardware initialized.");
    Serial.println("[MAIN] Press BOOT button to begin calibration.");
#endif

    // Slow blink = waiting for button
    ledBlink(3, 200, 200);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------

void loop() {
    // --- IDLE: wait for BOOT button press to start calibration ---
    if (maze.state() == MazeState::IDLE) {
        if (digitalRead(PIN_BOOT_BTN) == LOW) {
            delay(50);   // Debounce
            if (digitalRead(PIN_BOOT_BTN) == LOW) {
                while (digitalRead(PIN_BOOT_BTN) == LOW) delay(10);   // Wait release

                ledBlink(1, 50, 0);
#if SERIAL_DEBUG
                Serial.println("[MAIN] Button pressed. Starting calibration.");
#endif
                maze.startCalibration();
                // startCalibration() triggers the calibrate state which calls
                // startExploration() internally after finishing.
            }
        }
        return;
    }

    // --- FINISHED: blink success pattern and halt ---
    if (maze.state() == MazeState::FINISHED) {
        ledBlink(5, 100, 100);
        delay(2000);
        return;
    }

    // --- All active states: drive the state machine ---
    // Blink LED at ~2 Hz to indicate activity
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        lastBlink = millis();
    }

    maze.update();
}
