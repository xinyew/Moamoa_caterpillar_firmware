# CLAUDE.md — Agent Coding Instructions

> Full project docs in README.md. This file is the compact reference for coding.

## Quick Ref

- **SDK**: nRF Connect SDK v3.3.0 / Zephyr v4.3.99
- **Board**: `caterpillar/nrf54l15/cpuapp` — custom board (Raytac AN54LQ-P15 module), defined in `boards/kamoamoa/caterpillar/`
- **Build**: `west build -b caterpillar/nrf54l15/cpuapp` (see Commands below; output in `build/`)
- **Devicetree**: edit the board files in `boards/kamoamoa/caterpillar/` (no app overlay is used)
- **Console**: RTT only (`printk()`), no UART. No `%f` — use fixed-point ints. Enable float-printing when necessary

## Commands:
Load nrf connect sdk terminal for env variables, etc, please do this before any command below other than the RTT reading:
$ nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 --terminal  
Build project (use --no-sysbuild: a fresh CLI sysbuild configure fails to
find the custom board; only VS-Code-extension-created build trees work
with sysbuild):
$ west build -b caterpillar/nrf54l15/cpuapp --no-sysbuild
One-shot non-interactive build (no terminal needed, good for agents/scripts):
$ nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 -- west build -b caterpillar/nrf54l15/cpuapp --no-sysbuild
Flash firmware after a successful build:
$ west flash --recover
Load RTT console and connect to the board (makesure to set a timeout of 5 seconds, if there's nothing or returned meaning no output):
$ & "~\Downloads\SimplicityCommander-Windows\SimplicityCommander-Windows\Commander-cli_win32_x64_1v24p1b1980\Simplicity Commander CLI\commander-cli.exe"  rtt connect --device nrf54l15_M33

## Pin Map

| Pin   | Function         | Notes                        |
|-------|------------------|------------------------------|
| P1.02 | I2C SDA          | Shared: IMU + digipot        |
| P1.03 | I2C SCL          | Shared: IMU + digipot        |
| P1.04 | IMU INT1 (in)    | ASM330 data-ready            |
| P1.05 | IMU INT2 (in)    | ASM330 secondary             |
| P1.06 | ~SLEEP (out)     | DRV8212P, active-low         |
| P1.07 | PWM CH1 (out)    | DRV8212P IN1                 |
| P1.08 | PWM CH2 (out)    | DRV8212P IN2                 |
| P2.03 | DCDC_EN (out)    | STBB1-APUR, active-high      |

## I2C Addresses (7-bit)

- ASM330LHHTR: `0x6A` (SA0=0)
- MAX5419LETA: `0x28` (A0=0)


## SOC components to use
i2c20 and pwm20, bunch of gpios
