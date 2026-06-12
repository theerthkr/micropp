#pragma once
// =============================================================================
//  config.h  -  Micro-PP Maze Solver
//  Central configuration: pin assignments, motor constants, PID tuning,
//  sensor thresholds, and timing parameters.
//
//  Adjust values in this file to match your physical wiring and track
//  characteristics before flashing. All pin numbers refer to GPIO numbers
//  on the ESP32 DevKit V1.
// =============================================================================

// ---------------------------------------------------------------------------
// TB6612FNG Motor Driver - GPIO Assignments
// ---------------------------------------------------------------------------
// Motor A = LEFT motor  (N20-6V-600 RPM)
// Motor B = RIGHT motor (N20-6V-600 RPM)

#define PIN_AIN1   25   // Motor A direction pin 1
#define PIN_AIN2   26   // Motor A direction pin 2
#define PIN_PWMA   32   // Motor A PWM speed (LEDC channel 0)

#define PIN_BIN1   27   // Motor B direction pin 1
#define PIN_BIN2   14   // Motor B direction pin 2
#define PIN_PWMB   33   // Motor B PWM speed (LEDC channel 1)

#define PIN_STBY   13   // TB6612FNG standby (active HIGH = enabled)

// ---------------------------------------------------------------------------
// ESP32 LEDC PWM Configuration
// ---------------------------------------------------------------------------
#define PWM_FREQ       20000   // 20 kHz - above human hearing range
#define PWM_RESOLUTION 8       // 8-bit: 0-255
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1

// ---------------------------------------------------------------------------
// SmartElex RLS-08 Line Sensor Array
// Configured for WHITE LINE on BLACK SURFACE.
// The sensor outputs HIGH on white (reflective) and LOW on black (absorptive).
// ---------------------------------------------------------------------------
#define PIN_SENSOR_ANALOG  36   // VP / ADC1_CH0 - analog output from RLS-08
#define PIN_SENSOR_S0      34   // Multiplexer select bit 0
#define PIN_SENSOR_S1      35   // Multiplexer select bit 1
#define PIN_SENSOR_S2       4   // Multiplexer select bit 2

#define NUM_SENSORS        8    // RLS-08 has 8 sensors

// Sensor index ordering: 0 = leftmost, 7 = rightmost
// Sensor weights used for position calculation (scaled x1000)
// Positive = right of center, Negative = left of center
static const int SENSOR_WEIGHTS[NUM_SENSORS] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

// ADC threshold to distinguish white line from black surface.
// White line on black surface: sensors read HIGH on the line.
// Calibration sets this dynamically; this is the startup default.
#define SENSOR_THRESHOLD_DEFAULT  2048  // Mid-point of 12-bit ADC (0-4095)

// Intersection detection: how many sensors must be active simultaneously
#define INTERSECTION_MIN_SENSORS  5

// ---------------------------------------------------------------------------
// Motor Speed Parameters (PWM units: 0-255)
// ---------------------------------------------------------------------------
#define MOTOR_BASE_SPEED   130   // Nominal straight-line speed
#define MOTOR_MAX_SPEED    200   // Maximum allowable speed (protect gearbox)
#define MOTOR_MIN_SPEED     40   // Minimum speed before motor stalls
#define MOTOR_TURN_SPEED    90   // Speed used during point turns

// ---------------------------------------------------------------------------
// PID Controller Tuning - Line Following
// These values have been tuned for the N20-6V-600 RPM motors and the
// physical wheelbase of the micro-PP chassis.
// ---------------------------------------------------------------------------
#define PID_KP  0.035f   // Proportional gain
#define PID_KI  0.0001f  // Integral gain
#define PID_KD  0.18f    // Derivative gain

#define PID_MAX_INTEGRAL  200.0f   // Anti-windup clamp
#define PID_OUTPUT_LIMIT  160.0f   // Maximum correction applied to motors

// ---------------------------------------------------------------------------
// Turn Timing (milliseconds)
// Tuned for MOTOR_TURN_SPEED. Adjust if robot over- or under-shoots.
// ---------------------------------------------------------------------------
#define TURN_90_MS    320   // Duration for a 90-degree point turn
#define TURN_180_MS   640   // Duration for a 180-degree U-turn

// Delay after completing a turn before resuming line following
#define POST_TURN_SETTLE_MS  80

// ---------------------------------------------------------------------------
// Maze Solver
// ---------------------------------------------------------------------------
#define MAX_PATH_LENGTH  512   // Maximum number of intersections to store

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
// Hold robot over track crossing both black and white before pressing BOOT.
#define CALIBRATION_SAMPLES   100   // Number of ADC reads per sensor per sweep
#define CALIBRATION_DELAY_MS    5   // Delay between individual reads (ms)

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
#define PIN_LED  2   // Onboard LED on ESP32 DevKit V1

// ---------------------------------------------------------------------------
// Debug - Serial output (disable for competition to save CPU cycles)
// ---------------------------------------------------------------------------
#define SERIAL_DEBUG  1
#define SERIAL_BAUD   115200
