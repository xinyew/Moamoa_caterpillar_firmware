# Caterpillar

Motor control + IMU sensing embedded application for a custom nRF54L15 board.

Uses **nRF Connect SDK v3.3.0 / Zephyr v4.3.99** architecture.  
Debug output via Segger RTT.

## Subsystems

| # | Subsystem | IC | Interface | Purpose |
|---|-----------|-----|-----------|---------|
| 1 | Motor driver | DRV8212P | PWM + GPIO | Dual-channel motor control |
| 2 | Digital pot | MAX5419LETA | I2C | Adjust motor supply voltage via STBB1-APUR |
| 3 | IMU | ASM330LHHTR | I2C | 6-axis accel/gyro at 5–10 Hz |
| 4 | BLE | nRF54L15 radio | — | Remote control of PWM and voltage |

## Hardware Pin Map

| Signal              | Pin   | Direction | Notes                                      |
|---------------------|-------|-----------|--------------------------------------------|
| I2C SDA             | P1.02 | bidir     | Shared bus for IMU + digital pot            |
| I2C SCL             | P1.03 | out       | Shared bus                                  |
| IMU INT1            | P1.04 | in        | ASM330LHHTR data-ready / interrupt          |
| IMU INT2            | P1.05 | in        | ASM330LHHTR secondary interrupt             |
| DRV8212 ~SLEEP      | P1.06 | out       | Active-low sleep pin (LOW = sleep)          |
| PWM IN1             | P1.07 | out       | Motor driver control (PWM channel 1)        |
| PWM IN2             | P1.08 | out       | Motor driver control (PWM channel 2)        |
| DCDC EN             | P2.03 | out       | STBB1-APUR enable, active-high              |

## Custom board files
Please refer to ./boards/kamoamoa/caterpillar/* for available pins, peripherals, and configs

## I2C Bus

- **Single bus** — instance `i2c20`
- Two devices:
  - **ASM330LHHTR** (IMU): 7-bit addr `0x6A` (SA0=0)
  - **MAX5419LETA** (digipot): 7-bit addr `0x28` (A0=0) 

## PWM peripheral
pwm20

## src dir structure (beginning)
├── src/
│   ├── main.c                        
│   └── drivers/
│       ├── driver_asm330lhh.c/h           # ST register-level driver (platform-independent)
│       ├── ... other drivers




