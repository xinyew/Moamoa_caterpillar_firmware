# CLAUDE.md — Agent Coding Instructions

> Detailed docs in docs/ (ble-protocol, imu-logging, architecture,
> build-flash-ota); README.md is the overview. This file is the
> compact reference for coding.

## Quick Ref

- **SDK**: nRF Connect SDK v3.3.0 / Zephyr v4.3.99
- **Board**: `caterpillar/nrf54l15/cpuapp` — custom board (Raytac AN54LQ-P15 module), defined in `boards/kamoamoa/caterpillar/`
- **Build**: `west build -b caterpillar/nrf54l15/cpuapp` (see Commands below; output in `build/`)
- **Devicetree**: edit the board files in `boards/kamoamoa/caterpillar/` (no app overlay is used)
- **Console**: RTT only (`printk()`), no UART. No `%f` — use fixed-point ints. Enable float-printing when necessary

## Commands:
Load nrf connect sdk terminal for env variables, etc, please do this before any command below other than the RTT reading:
$ nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 --terminal  
Build the app (sysbuild + MCUboot; -DBOARD_ROOT is REQUIRED for the CLI
sysbuild configure to find the custom board).  The FLPR (RISC-V IMU
coprocessor) is rebuilt automatically as part of this build and its
binary is embedded in the app image (app OTA carries it; no separate
FLPR build or flashing ever).  Bump flpr/VERSION on FLPR changes -- the
device reports it via --read ("FLPR fw vX.Y.Z") to prove the new
coprocessor code is running:
$ nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 -- west build -b caterpillar/nrf54l15/cpuapp -- -DBOARD_ROOT="C:/Users/xwang3239/Downloads/Moamoa_caterpillar_firmware"
Flash over SWD (needed once to install MCUboot; erases everything):
$ west flash --recover
OTA update over BLE (after MCUboot is on the board; needs `pip install smpclient`):
$ python scripts/ble_control.py --dfu build/Moamoa_caterpillar_firmware/zephyr/zephyr.signed.bin
Images are auto-signed with the SDK dev key (intentionally open DFU — the
"not secure" build warning is expected). Bump VERSION before OTA releases.
Control GUI (motor + IMU config + live plots + log dump; standalone,
protocol spec in scripts/protocol.py):
$ pip install -r scripts/requirements.txt
$ python scripts/caterpillar_gui.py
IMU data path (v1.2.0+): FLPR samples at configured ODR (12.5 Hz-6.66 kHz)
into a shared-SRAM ring; app pump logs full rate to 798 KB RRAM
(dual-use with OTA secondary slot - a DFU upload overwrites the log)
and streams decimated (~20 KiB/s) over 0xFFE9. Warnings/errors go to
0xFFEC (GUI console). Boot is idle: rail off, driver asleep, duty 0.
Load RTT console and connect to the board (makesure to set a timeout of 5 seconds, if there's nothing or returned meaning no output):
$ & "~\Downloads\SimplicityCommander-Windows\SimplicityCommander-Windows\Commander-cli_win32_x64_1v24p1b1980\Simplicity Commander CLI\commander-cli.exe"  rtt connect --device nrf54l15_M33

## VS Code IntelliSense
.vscode/ (committed) wires the C/C++ extension to the build's
compile_commands.json (app: build/Moamoa_caterpillar_firmware/, FLPR:
build-flpr/). On a fresh machine: install the C/C++ + CMake extensions,
run one full build, then Reload Window - no per-machine config needed
(the databases are regenerated locally with correct NCS paths).

## Pin Map

| Pin   | Function         | Notes                               |
|-------|------------------|-------------------------------------|
| P0.01 | STATUS LED (out) | Blue, 470R, active-high             |
| P1.02 | I2C SDA          | Digipot only (`i2c20`)              |
| P1.03 | I2C SCL          | Digipot only                        |
| P1.04 | IMU INT1 (in)    | ASM330 data-ready                   |
| P1.05 | IMU INT2 (in)    | ASM330 secondary                    |
| P1.06 | ~SLEEP (out)     | DRV8212P, active-low                |
| P1.07 | PWM CH1 (out)    | DRV8212P IN1                        |
| P1.08 | PWM CH2 (out)    | DRV8212P IN2                        |
| P1.09 | SPI CS (out)     | ASM330, active-low (GPIO-driven)    |
| P1.10 | SPI SCLK (out)   | ASM330 (`spi21`)                    |
| P1.13 | SPI MOSI (out)   | ASM330 SDI                          |
| P1.14 | SPI MISO (in)    | ASM330 SDO                          |
| P2.03 | DCDC_EN (out)    | STBB1-APUR, active-high             |

## I2C Addresses (7-bit)

- MAX5419LETA: `0x28` (A0=0) — only device on i2c20 (IMU is on SPI)

## SOC components to use
i2c20 (digipot), spi21 (IMU), pwm20, bunch of gpios
