# Caterpillar

Motor control + IMU sensing firmware for a custom nRF54L15 board
(Raytac AN54LQ-P15 module), **nRF Connect SDK v3.3.0 / Zephyr v4.3.99**.

Dual-core: the ARM app core runs motor control, BLE, flash logging and
OTA; the RISC-V FLPR coprocessor is a dedicated IMU sampler (12.5 Hz –
6.66 kHz, configurable at runtime).  The FLPR binary is embedded inside
the app image, so a single build artifact and a single OTA update carry
both cores.

## Subsystems

| # | Subsystem | IC | Interface | Purpose |
|---|-----------|-----|-----------|---------|
| 1 | Motor driver | DRV8212P | PWM + GPIO | Dual-channel motor drive (fixed 50 % duty; amplitude via VDC) |
| 2 | Digital pot | MAX5419LETA | I2C | Adjust motor supply voltage via STBB1-APUR (0.75–4.2 V) |
| 3 | IMU | ASM330LHHTR | SPI (FLPR core) | 6-axis accel/gyro, DRDY-paced, ODR/FS runtime-configurable |
| 4 | BLE | nRF54L15 radio | — | Control surface, live IMU stream, log dump, OTA DFU |
| 5 | Status LED | Blue LED (P0.01) | GPIO | Heartbeat: board-alive indicator |
| 6 | IMU flash log | on-chip RRAM | — | 679 KB full-rate sample log (dual-use with OTA secondary slot) |

## IMU data path (firmware ≥ v1.2.x)

```
ASM330 --SPI/DRDY--> FLPR core --2048-record ring @0x20036000--> app pump thread
                                                                  ├─> RRAM log (679 KB, full rate)
                                                                  └─> BLE stream 0xFFE9 (decimated ~20 KiB/s)
```

- Sampling config (ODR, accel/gyro selection, full-scale) is applied live
  over BLE; raw int16 records (16 B) carry a seq16 counter so gaps are
  detectable.
- The flash log always receives the full configured rate; the live
  stream auto-decimates.  Fill policy per session: stop-when-full or
  circular.
- The log is multi-session: a 16-entry directory in flash records each
  session's range, sampling config, and real-world start time (from the
  0xFFEE clock sync).  Sessions survive reboots; a session ends on the
  stop command, on BLE disconnect, or (stop-when-full policy) before it
  would overwrite the oldest surviving session.  Circular sessions
  overwrite oldest data and drop consumed directory entries.
- The log region is dual-use with the OTA secondary slot: a DFU upload
  overwrites the log and vice versa.  Dump before updating.  (The
  slot's first 4 KB are kept erased — MCUboot wipes any non-image bytes
  it finds at the slot header on every boot.)
- Boot is idle: rail off, driver asleep.  Every run is started over BLE.
- Device warnings/errors are pushed on a BLE text characteristic
  (0xFFEC); RTT remains available for wired debugging only.

## BLE GATT (service 0xFFE0, device name "Caterpillar")

| Char | Access | Function |
|------|--------|----------|
| 0xFFE1 | write | PWM frequency, u16 LE Hz (4–1000) |
| 0xFFE2 | write | Motor VDC, u16 LE mV (750–4200) |
| 0xFFE3 | read | Measured VDC (AIN4 divider), u16 LE mV |
| 0xFFE4 | write | Motor rail enable, u8 |
| 0xFFE5 | write | Motor driver awake, u8 |
| 0xFFE6 | read | Status packet v3, 44 B |
| 0xFFE7 | r/w | Throughput test sink (tuning aid) |
| 0xFFE8 | r/w | IMU config {odr, content, accel_fs, gyro_fs} |
| 0xFFE9 | notify | Live IMU stream (8 B header + N×16 B records) |
| 0xFFEA | r/w | Log control (stop/start/erase) + state |
| 0xFFEB | write+notify | Dump request {session, offset, len} → chunked readout |
| 0xFFEC | notify | Warning/error text lines |
| 0xFFED | r/w | Status-LED heartbeat enable |
| 0xFFEE | r/w | Wall-clock sync (unix epoch; GUI writes on connect) |
| 0xFFEF | read | Session directory (up to 16 sessions, newest first) |
| SMP | — | MCUmgr OTA DFU (MCUboot, overwrite-only, dev-key signed) |

## Host tools (`scripts/`)

- `caterpillar_gui.py` — PySide6 + pyqtgraph GUI: motor panel, IMU
  config, live plots, on-chip log control + dump to CSV/NPZ, device
  console.  `pip install -r scripts/requirements.txt`, then
  `python scripts/caterpillar_gui.py`.
- `protocol.py` — the BLE protocol spec the GUI uses (single source of
  truth, matches `src/interface/ble_interface.c`).
- `ble_control.py` — CLI: `--freq`, `--V`, `--rail`, `--drv`, `--read`,
  `--dfu <zephyr.signed.bin>`, `--tput`, plus interactive mode.

## Build / flash / OTA

See CLAUDE.md for the exact commands.  Summary: one sysbuild `west
build` produces MCUboot + app with the FLPR auto-rebuilt and embedded;
deliver via `west flash --recover` (SWD) or
`scripts/ble_control.py --dfu` (BLE, ~19 KiB/s + ~40 s install).  Bump
`VERSION` (app) and `flpr/VERSION` (FLPR) per release — `--read`
reports both, proving what actually runs.

## Hardware Pin Map

| Signal              | Pin   | Direction | Notes                                      |
|---------------------|-------|-----------|--------------------------------------------|
| STATUS LED          | P0.01 | out       | Blue LED via 470R, active-high              |
| I2C SDA             | P1.02 | bidir     | Digipot only                                |
| I2C SCL             | P1.03 | out       | Digipot only                                |
| IMU INT1            | P1.04 | in        | ASM330LHHTR data-ready (serviced by FLPR)   |
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

- **I2C** — instance `i2c20` (P1.02 SDA / P1.03 SCL), app core
  - **MAX5419LETA** (digipot): 7-bit addr `0x28` (A0=0) — only device on the bus
- **SPI** — instance `spi21` (SCLK P1.10, MOSI P1.13, MISO P1.14, CS P1.09 GPIO), FLPR core
  - **ASM330LHHTR** (IMU), mode 3, 8 MHz, DRDY on INT1 (P1.04)

## PWM peripheral
pwm20 (app core)

## Source layout

```
├── src/                     # ARM app core
│   ├── main.c               # boot, health monitor
│   ├── flpr_launch.c        # copies embedded FLPR image to SRAM, starts VPR
│   ├── imu_pump.c           # drains FLPR ring -> log + stream
│   ├── imu_log.c            # 798 KB RRAM sample log
│   ├── drivers/             # motor/digipot/ADC/LED drivers
│   └── interface/           # BLE GATT server
├── flpr/                    # RISC-V FLPR core (own VERSION, auto-built)
│   └── src/                 # IMU sampler + ASM330 SPI driver
├── common/imu_shared.h      # cross-core shared-SRAM contract
├── boards/kamoamoa/caterpillar/   # custom board (both cores)
└── scripts/                 # host tools (CLI, GUI, protocol spec)
```
