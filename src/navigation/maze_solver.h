#pragma once
// =============================================================================
//  maze_solver.h  -  Autonomous Maze Navigation and Solving
//
//  Implements a three-phase maze strategy:
//
//  PHASE 1 - EXPLORATION:
//    Follows the white line using the Left-Hand Rule to choose turns at
//    intersections. Records every turn decision as L/R/S/U into a path array.
//    Continues until the goal is detected (all sensors active = finish line).
//
//  PHASE 2 - OPTIMIZATION:
//    Compresses the recorded path by substituting patterns of three turns
//    that include a U-turn with their single-turn equivalents. Repeated
//    until no further simplification is possible.
//
//  PHASE 3 - SPEED RUN:
//    Replays the optimized path at full speed without any sensor-based
//    turn detection - just executes the pre-computed sequence.
//
//  Turn codes:
//    'L' = Left 90 degrees
//    'R' = Right 90 degrees
//    'S' = Straight through intersection
//    'U' = 180-degree U-turn (dead end)
// =============================================================================

#include <Arduino.h>
#include "../motors/motor_driver.h"
#include "../sensors/line_sensor.h"
#include "pid_controller.h"

enum class MazeState {
    IDLE,
    CALIBRATING,
    EXPLORING,
    OPTIMIZING,
    SPEED_RUN,
    FINISHED
};

class MazeSolver {
public:
    MazeSolver(MotorDriver& motors, LineSensor& sensors, PIDController& pid);

    // Initialize state. Call after motors and sensors are ready.
    void begin();

    // Call repeatedly in loop(). Drives the state machine.
    void update();

    // Trigger calibration phase from external event (e.g. button press).
    void startCalibration();

    // Start exploration phase (after calibration).
    void startExploration();

    // Returns current operational state.
    MazeState state() const { return _state; }

    // Returns number of intersections recorded so far.
    uint16_t pathLength() const { return _pathLen; }

    // Returns true once the speed run is complete.
    bool isSolved() const { return _state == MazeState::FINISHED; }

private:
    // ---- State handlers ----
    void _handleCalibrating();
    void _handleExploring();
    void _handleOptimizing();
    void _handleSpeedRun();

    // ---- Line following ----
    void _followLine();

    // ---- Intersection handling ----
    void _handleIntersection();
    bool _canGoLeft();
    bool _canGoStraight();
    bool _canGoRight();
    void _executeTurn(char turn);
    void _recordTurn(char turn);

    // ---- Dead end handling ----
    void _handleDeadEnd();

    // ---- Path optimization ----
    void _optimizePath();
    bool _applyOnePass();
    char _simplifyThree(char a, char b, char c);

    // ---- Speed run ----
    void _executeSpeedRun();

    // ---- Sensors: intersection probe ----
    // Drive forward slightly past intersection center, then probe branches
    void _probeIntersection(bool& leftOpen, bool& straightOpen, bool& rightOpen);

    // References to shared hardware objects
    MotorDriver&  _motors;
    LineSensor&   _sensors;
    PIDController& _pid;

    // State machine
    MazeState _state = MazeState::IDLE;

    // Recorded turn path
    char     _path[MAX_PATH_LENGTH];
    uint16_t _pathLen = 0;

    // Speed run replay index
    uint16_t _runIdx = 0;

    // Timing helpers
    unsigned long _stateEnteredAt = 0;
    bool _firstIntersection = true;
};
