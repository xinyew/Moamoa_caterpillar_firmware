# Caterpillar

Motor control + IMU sensing embedded application for a custom nRF54L15 board.

Uses **nRF Connect SDK v3.3.0 / Zephyr v4.3.99** architecture.  
Debug output via Segger RTT.

## Subsystems

| # | Subsystem | IC | Interface | Purpose |
|---|-----------|-----|-----------|---------|
| 1 | Motor driver | DRV8212P | PWM + GPIO | Dual-channel motor control |
| 2 | Digital pot | MAX5419LETA | I2C | Adjust motor supply voltage via STBB1-APUR |
| 3 | IMU | ASM330LHHTR | SPI | 6-axis accel/gyro, DRDY-paced at 12.5 Hz |
| 4 | BLE | nRF54L15 radio | — | Remote control of PWM and voltage |
| 5 | Status LED | Blue LED (P0.01) | GPIO | Heartbeat: board-alive indicator |

## Hardware Pin Map

| Signal              | Pin   | Direction | Notes                                      |
|---------------------|-------|-----------|--------------------------------------------|
| STATUS LED          | P0.01 | out       | Blue LED via 470R, active-high              |
| I2C SDA             | P1.02 | bidir     | Digipot only                                |
| I2C SCL             | P1.03 | out       | Digipot only                                |
| IMU INT1            | P1.04 | in        | ASM330LHHTR data-ready / interrupt          |
| IMU INT2            | P1.05 | in        | ASM330LHHTR secondary interrupt             |
| DRV8212 ~SLEEP      | P1.06 | out       | Active-low sleep pin (LOW = sleep)          |
| PWM IN1             | P1.07 | out       | Motor driver control (PWM channel 1)        |
| PWM IN2             | P1.08 | out       | Motor driver control (PWM channel 2)        |
| SPI CS              | P1.09 | out       | ASM330LHHTR chip select, active-low         |
| SPI SCLK            | P1.10 | out       | IMU SPI clock (`spi21`)                     |
| SPI MOSI            | P1.13 | out       | IMU SPI data out (SDI)                      |
| SPI MISO            | P1.14 | in        | IMU SPI data in (SDO)                       |
| DCDC EN             | P2.03 | out       | STBB1-APUR enable, active-high              |

## Custom board files
Please refer to ./boards/kamoamoa/caterpillar/* for available pins, peripherals, and configs

## Buses

- **I2C** — instance `i2c20` (P1.02 SDA / P1.03 SCL)
  - **MAX5419LETA** (digipot): 7-bit addr `0x28` (A0=0) — only device on the bus
- **SPI** — instance `spi21` (SCLK P1.10, MOSI P1.13, MISO P1.14, CS P1.09 GPIO)
  - **ASM330LHHTR** (IMU), mode 3, 8 MHz, DRDY on INT1 (P1.04)

## PWM peripheral
pwm20

## src dir structure (beginning)
├── src/
│   ├── main.c                        
│   └── drivers/
│       ├── driver_asm330lhh.c/h           # ST register-level driver (platform-independent)
│       ├── ... other drivers




