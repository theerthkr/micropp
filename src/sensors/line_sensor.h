#pragma once
// =============================================================================
//  line_sensor.h  -  SmartElex RLS-08 8-Channel Line Sensor Array Interface
//
//  Configured for WHITE LINE on BLACK SURFACE.
//  The RLS-08's analog output is multiplexed through S0/S1/S2 select pins.
//  Each channel is read via a single ADC input on the ESP32.
//
//  Sensor 0 = leftmost   Sensor 7 = rightmost
//
//  Key outputs:
//    - position()       : weighted position error (-3500 to +3500)
//    - isOnLine()       : true if any sensor sees white
//    - isIntersection() : true if >= INTERSECTION_MIN_SENSORS sensors active
//    - isDeadEnd()      : true if no sensors active (completely off track)
//    - rawValue(n)      : calibrated 0-1000 reading for sensor n
// =============================================================================

#include <Arduino.h>

struct SensorReading {
    uint16_t raw[8];        // Calibrated 0-1000 for each sensor
    bool     active[8];     // true = sensor sees white line
    int      position;      // Weighted error: -3500 (far left) to +3500 (far right)
    bool     onLine;        // At least one sensor is active
    bool     isIntersection;// Multiple sensors active (crossing detected)
    bool     isDeadEnd;     // No sensors active
    uint8_t  activeCount;   // How many sensors are currently active
};

class LineSensor {
public:
    // Initialize ADC and select pins. Must be called in setup().
    void begin();

    // Run auto-calibration. Robot must be moved slowly over the track
    // so all sensors see both black and white during this call.
    // Duration: about CALIBRATION_SAMPLES * CALIBRATION_DELAY_MS * 8 ms
    void calibrate();

    // Read all 8 sensors and return structured result.
    SensorReading read();

    // Returns the last calculated position error (cached - no re-read).
    int lastPosition() const { return _lastPosition; }

    // Returns true if calibration has been run at least once.
    bool isCalibrated() const { return _calibrated; }

    // Raw ADC value for a single channel (0-4095, uncalibrated).
    uint16_t readRawADC(uint8_t channel);

private:
    void _selectChannel(uint8_t ch);
    uint16_t _calibrateValue(uint8_t ch, uint16_t raw);

    uint16_t _calMin[8];     // Minimum ADC reading per sensor (over black)
    uint16_t _calMax[8];     // Maximum ADC reading per sensor (over white)
    int      _lastPosition = 0;
    bool     _calibrated   = false;
};
