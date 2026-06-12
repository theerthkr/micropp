# Maze Solving Algorithm - Micro-PP

> [Back to README](../README.md) | [Wiring Diagram](wiring_diagram.md) | [maze_solver.cpp](../src/navigation/maze_solver.cpp)

## Overview

The robot solves a line maze in three phases:

```
Phase 1 (Exploration)  -->  Phase 2 (Optimization)  -->  Phase 3 (Speed Run)
```

---

## Phase 1: Exploration - Left-Hand Rule

The Left-Hand Rule is a classic wall-following algorithm that guarantees
finding the goal in any simply-connected maze (a maze with no isolated walls).

At every intersection, the robot evaluates available paths in priority order:

```
LEFT  >  STRAIGHT  >  RIGHT  >  U-TURN
```

### Turn Codes

| Code | Meaning                  |
|------|--------------------------|
| `L`  | Turn left 90 degrees     |
| `R`  | Turn right 90 degrees    |
| `S`  | Go straight through      |
| `U`  | 180-degree U-turn        |

### Intersection Detection

The RLS-08 array detects intersections when 5 or more of its 8 sensors
simultaneously detect the white line. A dead end is detected when zero
sensors see white (robot has driven off the end of a branch).

### Path Recording

Every turn decision (including U-turns at dead ends) is appended to an array.

Example of a recorded exploration path:
```
L S R U R L U L S ...
```

---

## Phase 2: Path Optimization

The recorded path contains redundant U-turns caused by dead ends. Any
three-turn sequence of the form `X U Y` (where `U` is a U-turn and X and
Y are any other turn) can always be collapsed into a single turn, because
the robot doubles back and then continues - equivalent to just turning once.

### Substitution Table

| Pattern | Simplified | Reason                              |
|---------|-----------|-------------------------------------|
| `L U L` | `S`       | Left, reverse, left = straight      |
| `L U S` | `R`       | Left, reverse, straight = right     |
| `L U R` | `U`       | Left, reverse, right = 180          |
| `S U L` | `R`       | Straight, reverse, left = right     |
| `S U S` | `U`       | Straight, reverse, straight = 180   |
| `S U R` | `L`       | Straight, reverse, right = left     |
| `R U L` | `S`       | Right, reverse, left = straight     |
| `R U R` | `S`       | Right, reverse, right = straight    |
| `R U S` | `L`       | Right, reverse, straight = left     |

The algorithm applies passes repeatedly until no more substitutions are
possible.

### Example

```
Exploration:  L  S  R  U  R  L  U  L  S
              ^^ First pass finds R U L at index 2-4 -> S
After pass 1: L  S  S  L  U  L  S
              ^^ Second pass finds L U L at index 3-5 -> S
After pass 2: L  S  S  S  S
              Done - no more U patterns remain
```

---

## Phase 3: Speed Run

The robot replays the optimized turn sequence at maximum speed. Because the
path is now known exactly, no decision logic is required at intersections -
the robot simply executes the pre-computed turn when it detects an intersection.

PID line following remains active between intersections to maintain
accuracy at higher speed.

---

## State Machine

```
  [IDLE]
     |  (BOOT button press)
     v
  [CALIBRATING]  <-- slowly spins, sensors gather min/max per channel
     |  (calibration complete)
     v
  [EXPLORING]  <-- PID line follow + Left-Hand Rule at intersections
     |  (goal detected: all 8 sensors active)
     v
  [OPTIMIZING]  <-- substitution table applied repeatedly
     |  (no more simplifications)
     v
  [SPEED_RUN]  <-- PID line follow + pre-computed turn sequence
     |  (all turns executed)
     v
  [FINISHED]  <-- LED success blink
```

---

## PID Line Following

Position error from the RLS-08 is a weighted sum across 8 sensor channels:

```
position = sum(sensor_weight[i] * reading[i]) / sum(reading[i])
```

Weights: [-3500, -2500, -1500, -500, 500, 1500, 2500, 3500]

Positive position = robot drifted right -> PID outputs positive correction
-> left motor slows, right motor speeds up -> robot steers left back to center.

```
left_speed  = BASE_SPEED - correction
right_speed = BASE_SPEED + correction
```
