# Micro-PP Autonomous Maze Solver

> **ESP32-based autonomous robot that follows a white line on a black surface, explores a maze using the Left-Hand Rule, and replays the optimized shortest path at full speed.**

---

## Table of Contents

- [Hardware Components](#hardware-components)
- [Project Structure](#project-structure)
- [Pin Mapping](#pin-mapping)
- [Power Distribution](#power-distribution)
- [Getting Started](#getting-started)
- [How It Works](#how-it-works)
- [Tuning Guide](#tuning-guide)
- [Docs](#docs)

---

## Hardware Components

| # | Component | Qty | Role |
|---|-----------|:---:|------|
| 1 | N20-6V-600 RPM Micro Metal Gear Motor | 2 | Left and right drive motors |
| 2 | SmartElex RLS-08 Analog & Digital Line Sensor Array | 1 | 8-channel white-line detection |
| 3 | LM2596S DC-DC Buck Converter Power Supply | 2 | Motor rail (6 V) and logic rail (5 V) |
| 4 | Motor Driver TB6612FNG Module | 1 | Dual H-bridge for both N20 motors |
| 5 | Performance Ultra Small Volume 3 PI Matching | 1 | RF impedance matching (antenna) |
| 6 | Performance Ultra L298N | 1 | Auxiliary motor driver (reserved) |
| 7 | ESP32 DevKit V1 | 1 | Main microcontroller |
| 8 | Battery | 1 | Main power source (7.4 V 2S LiPo) |
| 9 | 8x17 Breadboard | 2 | Power bus distribution |

---

## Project Structure

```
micropp/
├── platformio.ini                        # PlatformIO build config (ESP32 DevKit V1)
├── README.md                             # This file
├── .gitignore
├── src/
│   ├── main.cpp                          # Entry point: setup + loop + state trigger
│   ├── config.h                          # ALL pin assignments and tuning constants
│   ├── motors/
│   │   ├── motor_driver.h               # TB6612FNG driver interface
│   │   └── motor_driver.cpp             # PWM, direction, brake, pivot turns
│   ├── sensors/
│   │   ├── line_sensor.h                # RLS-08 sensor array interface
│   │   └── line_sensor.cpp              # Mux read, calibration, position calc
│   └── navigation/
│       ├── pid_controller.h             # Generic PID interface
│       ├── pid_controller.cpp           # Time-scaled PID with anti-windup
│       ├── maze_solver.h                # Maze state machine interface
│       └── maze_solver.cpp              # Left-Hand Rule + optimization + speed run
└── docs/
    ├── wiring_diagram.md                # Full GPIO tables and power rail notes
    └── algorithm.md                     # Algorithm explanation and state machine
```

### Quick Links to Source Files

| File | Purpose |
|------|---------|
| [`src/config.h`](src/config.h) | Every pin, speed, PID gain, and timing constant in one place |
| [`src/main.cpp`](src/main.cpp) | Boot sequence, button trigger, LED status, main loop |
| [`src/motors/motor_driver.h`](src/motors/motor_driver.h) | Motor driver public API |
| [`src/motors/motor_driver.cpp`](src/motors/motor_driver.cpp) | LEDC PWM + TB6612FNG logic |
| [`src/sensors/line_sensor.h`](src/sensors/line_sensor.h) | Sensor array public API |
| [`src/sensors/line_sensor.cpp`](src/sensors/line_sensor.cpp) | 8-channel mux, calibration, position error |
| [`src/navigation/pid_controller.h`](src/navigation/pid_controller.h) | PID interface |
| [`src/navigation/pid_controller.cpp`](src/navigation/pid_controller.cpp) | PID with anti-windup and output clamping |
| [`src/navigation/maze_solver.h`](src/navigation/maze_solver.h) | State machine interface |
| [`src/navigation/maze_solver.cpp`](src/navigation/maze_solver.cpp) | Left-Hand Rule, path optimization, speed run |
| [`docs/wiring_diagram.md`](docs/wiring_diagram.md) | Pin tables, power distribution, LM2596S setup |
| [`docs/algorithm.md`](docs/algorithm.md) | Full algorithm walkthrough with examples |

---

## Pin Mapping

### ESP32 DevKit V1 to TB6612FNG Motor Driver

| ESP32 GPIO | TB6612FNG | Description |
|-----------|-----------|-------------|
| GPIO 25 | AIN1 | Motor A (Left) direction pin 1 |
| GPIO 26 | AIN2 | Motor A (Left) direction pin 2 |
| GPIO 32 | PWMA | Motor A speed (PWM) |
| GPIO 27 | BIN1 | Motor B (Right) direction pin 1 |
| GPIO 14 | BIN2 | Motor B (Right) direction pin 2 |
| GPIO 33 | PWMB | Motor B speed (PWM) |
| GPIO 13 | STBY | Driver standby (HIGH = enabled) |
| 3.3 V | VCC | Logic supply |
| GND | GND | Common ground |

### ESP32 DevKit V1 to SmartElex RLS-08 Sensor Array

| ESP32 GPIO | RLS-08 | Description |
|-----------|--------|-------------|
| GPIO 36 (VP) | OUT / ANA | Analog multiplexer output |
| GPIO 34 | S0 | Channel select bit 0 |
| GPIO 35 | S1 | Channel select bit 1 |
| GPIO 4 | S2 | Channel select bit 2 |
| 5 V | VCC | Sensor power |
| GND | GND | Common ground |

> All adjustable pin assignments live in [`src/config.h`](src/config.h). Change them there and the rest of the code follows automatically.

---

## Power Distribution

```
Battery (7.4 V 2S LiPo)
    |
    +--[Switch]
         |
         +-- LM2596S #1 --> 6.0 V --> TB6612FNG VM  (motor power)
         |
         +-- LM2596S #2 --> 5.0 V --> ESP32 VIN
                                  --> RLS-08 VCC
```

All ground rails are tied together (battery, both buck converters, ESP32, motor driver, sensor array).

See [`docs/wiring_diagram.md`](docs/wiring_diagram.md) for full detail and LM2596S adjustment instructions.

---

## Getting Started

### 1. Install PlatformIO

Install the [PlatformIO extension for VS Code](https://platformio.org/install/ide?install=vscode).

### 2. Clone and Open

```bash
git clone https://github.com/theerthkr/micropp.git
cd micropp
```

Open the folder in VS Code. PlatformIO detects `platformio.ini` automatically.

### 3. Build

Click the **Build** button (checkmark icon in the PlatformIO toolbar) or run:

```bash
pio run
```

### 4. Flash

Connect the ESP32 DevKit V1 via USB, then click **Upload** or run:

```bash
pio run --target upload
```

### 5. Open Serial Monitor

```bash
pio device monitor --baud 115200
```

---

## How It Works

The robot operates as a three-phase state machine. Full details are in [`docs/algorithm.md`](docs/algorithm.md).

```
[IDLE] --> (BOOT button) --> [CALIBRATING] --> [EXPLORING] --> [OPTIMIZING] --> [SPEED_RUN] --> [FINISHED]
```

### Phase 1 - Calibration

Press the **BOOT button** (GPIO 0) on the ESP32. The robot slowly spins in place while the RLS-08 sensor array records the minimum and maximum ADC reading for each of its 8 channels. This compensates for uneven sensor sensitivity and ambient lighting.

### Phase 2 - Exploration (Left-Hand Rule)

The robot follows the white line using PID control and makes decisions at every intersection using this priority order:

```
LEFT  >  STRAIGHT  >  RIGHT  >  U-TURN
```

Every turn taken is recorded as `L`, `R`, `S`, or `U`. A dead end (no line detected) triggers an automatic U-turn.

### Phase 3 - Path Optimization

The recorded turn sequence is compressed. Any triplet of the form `X U Y` (a turn, then a U-turn, then another turn) is a redundant detour and collapses into a single equivalent turn.

| Pattern | Simplifies To |
|---------|:---:|
| `L U L` | `S` |
| `L U S` | `R` |
| `L U R` | `U` |
| `S U L` | `R` |
| `S U S` | `U` |
| `S U R` | `L` |
| `R U L` | `S` |
| `R U R` | `S` |
| `R U S` | `L` |

Passes repeat until no further simplification is possible.

### Phase 4 - Speed Run

The robot replays the optimized turn sequence at `MOTOR_MAX_SPEED` without any turn-decision logic. PID line following remains active between intersections for accuracy.

---

## Tuning Guide

All tuning constants are in [`src/config.h`](src/config.h). Adjust and reflash.

| Constant | Default | Effect |
|----------|---------|--------|
| `MOTOR_BASE_SPEED` | 130 | Cruise speed during exploration |
| `MOTOR_MAX_SPEED` | 200 | Speed during the final speed run |
| `MOTOR_TURN_SPEED` | 90 | Speed used during point turns |
| `PID_KP` | 0.035 | Proportional gain - increase for faster correction |
| `PID_KI` | 0.0001 | Integral gain - reduces steady-state drift |
| `PID_KD` | 0.18 | Derivative gain - damps oscillation |
| `TURN_90_MS` | 320 | Milliseconds for a 90-degree pivot turn |
| `TURN_180_MS` | 640 | Milliseconds for a 180-degree U-turn |

> If the robot overshoots turns: decrease `TURN_90_MS` / `TURN_180_MS`.
> If the robot oscillates on the line: decrease `PID_KP` or increase `PID_KD`.

---

## Docs

- [`docs/wiring_diagram.md`](docs/wiring_diagram.md) - Complete pin tables, power rails, breadboard notes, LM2596S setup
- [`docs/algorithm.md`](docs/algorithm.md) - Left-Hand Rule, substitution table with worked example, state machine diagram

---

## License

MIT License.
