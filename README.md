# Caterpillar

Vibration-motor control + high-rate IMU sensing firmware for a custom
nRF54L15 board (Raytac AN54LQ-P15), **nRF Connect SDK v3.3.0 /
Zephyr v4.3.99**.

Dual-core: the ARM app core runs motor control, BLE, flash logging and
OTA; the RISC-V FLPR coprocessor samples the IMU (12.5 Hz–6.66 kHz).
The FLPR binary is embedded in the app image — one build artifact, one
OTA update, both cores.

**What it does**

- Independent motor control: VDC 0.75–4.2 V (digipot) × PWM 4–1000 Hz
- IMU sessions: full-rate circular flash log (679 KB) + decimated live
  BLE preview, wall-clock-stamped session directory, per-session dump
- Full BLE control surface + open OTA DFU (MCUboot, dev-key signed)
- PySide6 GUI and CLI for everything

## Documentation

| Doc | Contents |
|---|---|
| [docs/ble-protocol.md](docs/ble-protocol.md) | GATT service, every characteristic, byte-level packet layouts, host/GUI protocol flows |
| [docs/imu-logging.md](docs/imu-logging.md) | data path, rate envelope, flash layout, session/circular semantics, dump file formats, caveats |
| [docs/architecture.md](docs/architecture.md) | cores, memory maps, boot chain, threads, motor chain |
| [docs/build-flash-ota.md](docs/build-flash-ota.md) | build command, versioning rules, SWD flash, OTA, RTT, IntelliSense |
| [CLAUDE.md](CLAUDE.md) | compact command reference |

## Quick start

```
# build everything (MCUboot + app + embedded FLPR)
nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 -- ^
  west build -b caterpillar/nrf54l15/cpuapp -- -DBOARD_ROOT="<repo path>"

# first flash (SWD, erases chip)
west flash --recover

# updates over BLE
python scripts/ble_control.py --dfu build/Moamoa_caterpillar_firmware/zephyr/zephyr.signed.bin

# GUI (motor, IMU config, live plots, session dump/viewer)
pip install -r scripts/requirements.txt
python scripts/caterpillar_gui.py
```

Bump `VERSION` per release and `flpr/VERSION` on FLPR changes —
`--read` reports both running versions.  An OTA wipes stored log
sessions; dump first.

## Hardware

| Signal | Pin | Dir | Notes |
|---|---|---|---|
| STATUS LED | P0.01 | out | blue, 470R, active-high |
| I2C SDA/SCL | P1.02/P1.03 | — | `i2c20`, app core: MAX5419 digipot @0x28 (only device) |
| IMU INT1/INT2 | P1.04/P1.05 | in | ASM330 DRDY (INT1 serviced by FLPR) |
| DRV8212 ~SLEEP | P1.06 | out | active-low (LOW = sleep) |
| PWM IN1/IN2 | P1.07/P1.08 | out | `pwm20`, motor drive (fixed 50 % duty) |
| SPI CS/SCLK/MOSI/MISO | P1.09/P1.10/P1.13/P1.14 | — | `spi21`, FLPR core: ASM330 mode 3, 8 MHz |
| DCDC EN | P2.03 | out | STBB1-APUR enable, active-high |

Board files: `boards/kamoamoa/caterpillar/` (both cores).

## Source layout

```
├── src/                     # ARM app core
│   ├── main.c               # trivial entry
│   ├── app/                 # lifecycle: boot+health (app.c),
│   │                        #   orchestration (session.c), work queue (device_cmd.c)
│   ├── imu/                 # pipeline: FLPR launch, ring pump, flash session log
│   ├── ble/                 # control plane (ble_interface), data plane (ble_transport)
│   ├── settings/            # persisted registry (generated table + store)
│   └── drivers/             # motor / digipot / ADC / LED
├── flpr/                    # RISC-V FLPR core (own VERSION, auto-built)
├── common/imu_shared.h      # cross-core shared-SRAM contract
├── boards/kamoamoa/caterpillar/
├── scripts/                 # host tools: CLI, GUI, protocol spec, codegen
└── docs/                    # detailed documentation
```
