// =============================================================================
//  line_sensor.cpp  -  SmartElex RLS-08 8-Channel Line Sensor Implementation
// =============================================================================

#include "line_sensor.h"
#include "../config.h"

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void LineSensor::begin() {
    pinMode(PIN_SENSOR_S0, OUTPUT);
    pinMode(PIN_SENSOR_S1, OUTPUT);
    pinMode(PIN_SENSOR_S2, OUTPUT);

    // ADC pin - input only, no pull on analog GPIO
    // GPIO 36 (VP) is input-only on ESP32, no pinMode needed

    // Initialize calibration arrays with safe defaults
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        _calMin[i] = 0;
        _calMax[i] = 4095;
    }

    // Configure ADC resolution: 12-bit (0-4095)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);   // Full 0-3.3V range
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

void LineSensor::calibrate() {
    // Reset calibration bounds
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        _calMin[i] = 4095;
        _calMax[i] = 0;
    }

    // Collect samples across all sensors
    for (uint16_t s = 0; s < CALIBRATION_SAMPLES; s++) {
        for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
            uint16_t val = readRawADC(ch);
            if (val < _calMin[ch]) _calMin[ch] = val;
            if (val > _calMax[ch]) _calMax[ch] = val;
        }
        delay(CALIBRATION_DELAY_MS);
    }

    _calibrated = true;
}

// ---------------------------------------------------------------------------
// Main read
// ---------------------------------------------------------------------------

SensorReading LineSensor::read() {
    SensorReading result;
    result.onLine        = false;
    result.isIntersection = false;
    result.isDeadEnd     = false;
    result.activeCount   = 0;

    long  weightedSum = 0;
    long  totalActive = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint16_t raw = readRawADC(i);
        result.raw[i] = _calibrateValue(i, raw);

        // WHITE LINE on BLACK: sensor is active when calibrated value > 500
        // (i.e., reflectivity is above midpoint after normalisation to 0-1000)
        result.active[i] = (result.raw[i] > 500);

        if (result.active[i]) {
            result.activeCount++;
            weightedSum += (long)SENSOR_WEIGHTS[i] * result.raw[i];
            totalActive += result.raw[i];
        }
    }

    result.onLine         = (result.activeCount > 0);
    result.isIntersection = (result.activeCount >= INTERSECTION_MIN_SENSORS);
    result.isDeadEnd      = !result.onLine;

    if (totalActive > 0) {
        result.position = (int)(weightedSum / totalActive);
    } else {
        result.position = _lastPosition;   // Hold last known position
    }

    _lastPosition = result.position;
    return result;
}

// ---------------------------------------------------------------------------
// Raw ADC read for a specific multiplexer channel
// ---------------------------------------------------------------------------

uint16_t LineSensor::readRawADC(uint8_t channel) {
    _selectChannel(channel);
    delayMicroseconds(50);   // Allow MUX to settle
    return (uint16_t)analogRead(PIN_SENSOR_ANALOG);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void LineSensor::_selectChannel(uint8_t ch) {
    // RLS-08 uses S0/S1/S2 to select 1-of-8 sensor channels
    digitalWrite(PIN_SENSOR_S0, (ch >> 0) & 0x01);
    digitalWrite(PIN_SENSOR_S1, (ch >> 1) & 0x01);
    digitalWrite(PIN_SENSOR_S2, (ch >> 2) & 0x01);
}

uint16_t LineSensor::_calibrateValue(uint8_t ch, uint16_t raw) {
    // Normalise raw ADC value to 0-1000 using per-channel calibration
    uint16_t minVal = _calMin[ch];
    uint16_t maxVal = _calMax[ch];

    if (maxVal == minVal) return 0;   // Avoid divide-by-zero

    if (raw <= minVal) return 0;
    if (raw >= maxVal) return 1000;

    return (uint16_t)(((uint32_t)(raw - minVal) * 1000) / (maxVal - minVal));
}
