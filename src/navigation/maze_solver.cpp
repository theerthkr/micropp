// =============================================================================
//  maze_solver.cpp  -  Maze Navigation and Solving Implementation
// =============================================================================

#include "maze_solver.h"
#include "../config.h"

// ---------------------------------------------------------------------------
// Construction and initialization
// ---------------------------------------------------------------------------

MazeSolver::MazeSolver(MotorDriver& motors, LineSensor& sensors, PIDController& pid)
    : _motors(motors), _sensors(sensors), _pid(pid) {}

void MazeSolver::begin() {
    _state  = MazeState::IDLE;
    _pathLen = 0;
    _runIdx  = 0;
}

void MazeSolver::startCalibration() {
    _state = MazeState::CALIBRATING;
    _stateEnteredAt = millis();
#if SERIAL_DEBUG
    Serial.println("[MAZE] Starting calibration...");
#endif
}

void MazeSolver::startExploration() {
    _pathLen         = 0;
    _runIdx          = 0;
    _firstIntersection = true;
    _pid.reset();
    _state = MazeState::EXPLORING;
    _stateEnteredAt = millis();
#if SERIAL_DEBUG
    Serial.println("[MAZE] Starting exploration (Left-Hand Rule).");
#endif
}

// ---------------------------------------------------------------------------
// Main update - call from loop()
// ---------------------------------------------------------------------------

void MazeSolver::update() {
    switch (_state) {
        case MazeState::CALIBRATING:  _handleCalibrating(); break;
        case MazeState::EXPLORING:    _handleExploring();   break;
        case MazeState::OPTIMIZING:   _handleOptimizing();  break;
        case MazeState::SPEED_RUN:    _handleSpeedRun();    break;
        case MazeState::FINISHED:     /* nothing */          break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// CALIBRATION phase
// ---------------------------------------------------------------------------

void MazeSolver::_handleCalibrating() {
    // Slowly spin in place while calibration captures min/max per sensor
    _motors.setSpeed(MOTOR_TURN_SPEED / 2, -(MOTOR_TURN_SPEED / 2));
    _sensors.calibrate();    // Blocking call (takes ~4 seconds)
    _motors.brake();

#if SERIAL_DEBUG
    Serial.println("[MAZE] Calibration complete.");
#endif

    // Immediately start exploring after calibration
    startExploration();
}

// ---------------------------------------------------------------------------
// EXPLORATION phase  (Left-Hand Rule)
// ---------------------------------------------------------------------------

void MazeSolver::_handleExploring() {
    SensorReading sr = _sensors.read();

    // --- Goal detection: finish line is wide (all sensors active) ---
    // Finish is detected when ALL 8 sensors see white simultaneously.
    if (sr.activeCount == NUM_SENSORS) {
        _motors.brake();
        delay(200);
#if SERIAL_DEBUG
        Serial.println("[MAZE] Goal reached! Beginning path optimization.");
        Serial.print("[MAZE] Raw path ("); Serial.print(_pathLen); Serial.print("): ");
        for (uint16_t i = 0; i < _pathLen; i++) Serial.print(_path[i]);
        Serial.println();
#endif
        _state = MazeState::OPTIMIZING;
        return;
    }

    // --- Dead end: no sensors active ---
    if (sr.isDeadEnd) {
        _handleDeadEnd();
        return;
    }

    // --- Intersection: multiple sensors active ---
    if (sr.isIntersection && !_firstIntersection) {
        _handleIntersection();
        return;
    }
    _firstIntersection = false;

    // --- Normal line following via PID ---
    _followLine();
}

// ---------------------------------------------------------------------------
// Line following (PID)
// ---------------------------------------------------------------------------

void MazeSolver::_followLine() {
    SensorReading sr = _sensors.read();
    float correction = _pid.compute(0.0f, (float)sr.position);

    int leftSpeed  = constrain((int)(MOTOR_BASE_SPEED - correction),
                               MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    int rightSpeed = constrain((int)(MOTOR_BASE_SPEED + correction),
                               MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);

    _motors.setSpeed(leftSpeed, rightSpeed);
}

// ---------------------------------------------------------------------------
// Intersection handler
// ---------------------------------------------------------------------------

void MazeSolver::_handleIntersection() {
    // Stop cleanly at the centre of the intersection
    _motors.brake();
    delay(50);

    bool leftOpen, straightOpen, rightOpen;
    _probeIntersection(leftOpen, straightOpen, rightOpen);

    // Left-Hand Rule priority: L > S > R > U-turn
    char turn = 'U';   // Default: dead end (will be handled by _handleDeadEnd)

    if      (leftOpen)     turn = 'L';
    else if (straightOpen) turn = 'S';
    else if (rightOpen)    turn = 'R';
    // else turn = 'U' (already set)

    _recordTurn(turn);
    _executeTurn(turn);
    _pid.reset();

#if SERIAL_DEBUG
    Serial.print("[MAZE] Intersection -> ");
    Serial.print("L:"); Serial.print(leftOpen);
    Serial.print(" S:"); Serial.print(straightOpen);
    Serial.print(" R:"); Serial.print(rightOpen);
    Serial.print(" -> Turn: "); Serial.println(turn);
#endif
}

// ---------------------------------------------------------------------------
// Dead end handler
// ---------------------------------------------------------------------------

void MazeSolver::_handleDeadEnd() {
    _motors.brake();
    delay(100);
    _recordTurn('U');
    _motors.turnAround();
    _pid.reset();
#if SERIAL_DEBUG
    Serial.println("[MAZE] Dead end -> U-turn");
#endif
}

// ---------------------------------------------------------------------------
// Branch probe: drive slightly past centre, check each side for a white line
// ---------------------------------------------------------------------------

void MazeSolver::_probeIntersection(bool& leftOpen,
                                     bool& straightOpen,
                                     bool& rightOpen) {
    // Drive forward a small amount so all branch openings are under sensors
    _motors.setSpeed(MOTOR_BASE_SPEED - 20, MOTOR_BASE_SPEED - 20);
    delay(80);
    _motors.brake();
    delay(30);

    // Check forward (straight)
    SensorReading fwd = _sensors.read();
    straightOpen = fwd.onLine;

    // Pivot left 90 and probe
    _motors.setSpeed(-MOTOR_TURN_SPEED, MOTOR_TURN_SPEED);
    delay(TURN_90_MS);
    _motors.brake();
    delay(40);
    SensorReading lft = _sensors.read();
    leftOpen = lft.onLine;

    // Pivot right 180 (to check right branch)
    _motors.setSpeed(MOTOR_TURN_SPEED, -MOTOR_TURN_SPEED);
    delay(TURN_180_MS);
    _motors.brake();
    delay(40);
    SensorReading rgt = _sensors.read();
    rightOpen = rgt.onLine;

    // Return to original heading (left 90)
    _motors.setSpeed(-MOTOR_TURN_SPEED, MOTOR_TURN_SPEED);
    delay(TURN_90_MS);
    _motors.brake();
    delay(40);
}

// ---------------------------------------------------------------------------
// Execute a turn
// ---------------------------------------------------------------------------

void MazeSolver::_executeTurn(char turn) {
    switch (turn) {
        case 'L': _motors.turnLeft90();   break;
        case 'R': _motors.turnRight90();  break;
        case 'S':
            // Drive through intersection without turning
            _motors.setSpeed(MOTOR_BASE_SPEED, MOTOR_BASE_SPEED);
            delay(120);
            _motors.brake();
            break;
        case 'U': _motors.turnAround();   break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// Record a turn into the path array
// ---------------------------------------------------------------------------

void MazeSolver::_recordTurn(char turn) {
    if (_pathLen < MAX_PATH_LENGTH) {
        _path[_pathLen++] = turn;
    }
}

// ---------------------------------------------------------------------------
// OPTIMIZATION phase
// ---------------------------------------------------------------------------

void MazeSolver::_handleOptimizing() {
    _optimizePath();

#if SERIAL_DEBUG
    Serial.print("[MAZE] Optimized path ("); Serial.print(_pathLen); Serial.print("): ");
    for (uint16_t i = 0; i < _pathLen; i++) Serial.print(_path[i]);
    Serial.println();
#endif

    delay(500);
    _runIdx = 0;
    _state  = MazeState::SPEED_RUN;
}

void MazeSolver::_optimizePath() {
    bool changed = true;
    while (changed) {
        changed = _applyOnePass();
    }
}

bool MazeSolver::_applyOnePass() {
    for (uint16_t i = 0; i + 2 < _pathLen; i++) {
        char simplified = _simplifyThree(_path[i], _path[i+1], _path[i+2]);
        if (simplified != 0) {
            // Replace three turns with one
            _path[i] = simplified;
            // Shift the remaining path left by 2
            for (uint16_t j = i + 1; j + 2 < _pathLen; j++) {
                _path[j] = _path[j + 2];
            }
            _pathLen -= 2;
            return true;   // Restart pass after each substitution
        }
    }
    return false;
}

// Substitution table:
// Any triplet X-U-Y collapses because the robot turns around between two turns.
// The net result: turning right X degrees then 180 then Y degrees.
char MazeSolver::_simplifyThree(char a, char b, char c) {
    if (b != 'U') return 0;   // Only simplify when middle is U-turn

    // Left - U-turn - Left  -> Straight
    if (a == 'L' && c == 'L') return 'S';
    // Left - U-turn - Straight -> Right
    if (a == 'L' && c == 'S') return 'R';
    // Right - U-turn - Left  -> Straight (back to start of segment)
    if (a == 'R' && c == 'L') return 'S';
    // Straight - U-turn - Left  -> Right
    if (a == 'S' && c == 'L') return 'R';
    // Left - U-turn - Right  -> U-turn (net 180)
    if (a == 'L' && c == 'R') return 'U';
    // Straight - U-turn - Straight -> U-turn
    if (a == 'S' && c == 'S') return 'U';
    // Right - U-turn - Right -> Straight
    if (a == 'R' && c == 'R') return 'S';
    // Straight - U-turn - Right -> Left
    if (a == 'S' && c == 'R') return 'L';

    return 0;   // No simplification
}

// ---------------------------------------------------------------------------
// SPEED RUN phase
// ---------------------------------------------------------------------------

void MazeSolver::_handleSpeedRun() {
    if (_runIdx >= _pathLen) {
        _motors.brake();
        _state = MazeState::FINISHED;
#if SERIAL_DEBUG
        Serial.println("[MAZE] Speed run complete. Maze solved!");
#endif
        return;
    }

    // Follow line at high speed until next intersection
    SensorReading sr = _sensors.read();

    if (sr.isIntersection) {
        // At intersection: execute pre-computed turn
        _motors.brake();
        delay(30);
        _executeTurn(_path[_runIdx++]);
        _pid.reset();
        return;
    }

    if (sr.isDeadEnd) {
        // Should not happen in speed run; safety stop
        _motors.brake();
        delay(200);
        return;
    }

    // PID at higher speed
    float correction = _pid.compute(0.0f, (float)sr.position);
    int leftSpeed  = constrain((int)(MOTOR_MAX_SPEED - correction),
                               MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    int rightSpeed = constrain((int)(MOTOR_MAX_SPEED + correction),
                               MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    _motors.setSpeed(leftSpeed, rightSpeed);
}
