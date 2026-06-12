# Wiring Diagram Reference - Micro-PP Maze Solver

> [Back to README](../README.md) | [Algorithm](algorithm.md) | [config.h](../src/config.h)

## Power Distribution

```
[Battery 7.4V] ---[Switch]---+--- [LM2596S #1] ---> 6V ---> TB6612FNG VM pin
                             |
                             +--- [LM2596S #2] ---> 5V ---> ESP32 VIN
                                                         ---> RLS-08 VCC
```

All ground rails are common (battery GND = both buck GNDs = ESP32 GND = motor driver GND = sensor GND).

---

## ESP32 DevKit V1 to TB6612FNG Motor Driver

| ESP32 GPIO | TB6612FNG | Wire Color (diagram) | Description       |
|-----------|-----------|----------------------|-------------------|
| GPIO 25   | AIN1      | Yellow               | Motor A Dir 1     |
| GPIO 26   | AIN2      | Yellow               | Motor A Dir 2     |
| GPIO 32   | PWMA      | Green                | Motor A PWM speed |
| GPIO 27   | BIN1      | Yellow               | Motor B Dir 1     |
| GPIO 14   | BIN2      | Yellow               | Motor B Dir 2     |
| GPIO 33   | PWMB      | Green                | Motor B PWM speed |
| GPIO 13   | STBY      | Cyan                 | Standby enable    |
| 3.3V      | VCC       | Red                  | Logic supply      |
| GND       | GND       | Black                | Common ground     |

---

## TB6612FNG to N20 Motors

| TB6612FNG | Motor      | Notes                    |
|-----------|------------|--------------------------|
| AO1, AO2  | Motor A (Left)  | N20-6V-600 RPM      |
| BO1, BO2  | Motor B (Right) | N20-6V-600 RPM      |
| VM        | LM2596S #1 output (6V) | Motor supply |

---

## ESP32 DevKit V1 to SmartElex RLS-08 Sensor Array

| ESP32 GPIO     | RLS-08 Pin | Wire Color | Description              |
|---------------|------------|------------|--------------------------|
| GPIO 36 (VP)  | OUT / ANA  | Pink       | Analog mux output        |
| GPIO 34       | S0         | Yellow     | Channel select bit 0     |
| GPIO 35       | S1         | Yellow     | Channel select bit 1     |
| GPIO 4        | S2         | Yellow     | Channel select bit 2     |
| 5V            | VCC        | Red        | Sensor power (5V)        |
| GND           | GND        | Black      | Common ground            |

> Note: GPIO 34, 35, 36 on ESP32 are INPUT-ONLY pins. S0/S1/S2 are outputs
> from the ESP32 to the sensor, not inputs. Verify the RLS-08 pinout matches
> your board revision before wiring.

---

## LM2596S Adjustment

- **LM2596S #1 (Motor supply):** Turn the potentiometer until Vout = 6.0V
  with no load, then verify it holds under motor load.
- **LM2596S #2 (Logic supply):** Turn the potentiometer until Vout = 5.0V.
  This powers both the ESP32 (via VIN) and the RLS-08.

---

## Notes

- The 8x17 breadboards are used for the power distribution bus bars
  (VCC and GND rails), not for signal routing.
- Keep motor wiring away from sensor signal wires to minimize EMI.
- Add a 100uF capacitor across the motor power rails near the TB6612FNG
  to suppress voltage spikes when motors change direction.
