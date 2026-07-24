# Firmware Architecture

Dual-core nRF54L15 (Raytac AN54LQ-P15 module), NCS v3.3.0 / Zephyr 4.3.99.

| Core | Runs | Executes from |
|---|---|---|
| ARM Cortex-M33 ("app") | motor control, BLE + controller stack, flash logging, OTA, health monitor | RRAM (XIP) |
| RISC-V VPR ("FLPR") | dedicated IMU sampler (`flpr/`) | SRAM |

## Memory maps

RRAM (1524 KB, partition-manager layout — see `build/partitions.yml`):

| Range | Content |
|---|---|
| 0x000000–0x00D800 | MCUboot (locked by bootconf after first boot) |
| 0x00E000–0x00E800 | MCUboot pad (image header) |
| 0x00E800–0x0B9000 | Primary slot: app **with the FLPR binary embedded** |
| 0x0B9000–0x164000 | Secondary slot: OTA upload target, dual-use as IMU log (exact log offsets in imu-logging.md) |
| 0x164000–0x165000 | spare (4 KB) |
| 0x165000–0x17D000 | ex-FLPR region — **inaccessible to the app flash device**, unused |
| 0xFFD080 | bootconf (boot-region protection) |

SRAM (256 KB):

| Range | Owner |
|---|---|
| 0x20000000 + 160 KB | app core |
| 0x20028000 + 56 KB | FLPR private (code copied here at boot) |
| 0x20036000 + 40 KB | shared block: control/config + 2048-record sample ring (`common/imu_shared.h`, magic "IMU2") |

## Boot chain

1. MCUboot validates the primary slot (dev-key signature) and jumps.
2. App boots **idle**: rail off, DRV8212 asleep, PWM primed at 50 %
   duty but nothing moves until rail+driver are enabled over BLE.
   Digipot is pre-programmed to 3.1 V so the rail comes up safe.
   A board reset is therefore always a safe motor-stop.
3. `src/flpr_launch.c` memcpys the embedded FLPR image to 0x20028000,
   sets the SPU attribute, INITPC, CPURUN — the only launcher in the
   system.  The devicetree deliberately has no `execution-memory`
   property: a DT-based launcher would also appear in MCUboot's build
   and re-copy over the running coprocessor (historic fatal bug).
4. FLPR boots its own Zephyr, initializes the ASM330 over SPI
   (dummy-first-read + retries — the sensor's first transaction after
   power-up returns garbage), applies the default config (833 Hz,
   accel+gyro, ±2 g/±250 dps), publishes `magic`, then samples
   DRDY-paced, publishing raw records into the ring (never blocks;
   drops are counted).  Config changes arrive via a seq-numbered
   block in shared SRAM, polled every 32 samples.

## Source layering (biosensor-style main→app→adapter, v1.4.1)

| Layer | Files | Role |
|---|---|---|
| entry | `main.c` | trivial: `app_init()` + `app_run()` |
| lifecycle | `app/app.c` | boot sequence, health monitor |
| orchestration | `app/session.c` | wall clock, on-demand sampling arbiter, session side effects (conn-param policy) |
| execution | `app/device_cmd.c` | slow operations off the BT RX thread |
| control plane | `ble/ble_interface.c` | GATT tables + decode-and-delegate handlers, connection lifecycle |
| data plane | `ble/ble_transport.c` | credit-paced TX thread (stream/messages), dump thread, tier-2 ring |
| storage | `imu/imu_log.c`, `settings/settings_store.c` | flash session log; persisted settings |
| pipeline | `imu/imu_pump.c` | drains the FLPR ring (memcpy only) |

Rule of the house: nothing in a GATT handler or the pump may block;
nothing outside `ble_transport.c` may send notifications.

## App-core threads

| Thread | Prio | Role |
|---|---|---|
| pump (`imu_pump.c`) | 4 | drains the ring every 4 ms → `imu_log_append` + stream sink |
| dump (`ble_interface.c`) | 7 | streams session data as credit-paced notifications |
| main | — | 1 Hz health monitor → warnings to 0xFFEC (FLPR dead, IMU dead, samples stalled, overrun deltas) |
| BT stack threads | — | standard Zephyr controller/host |

Design notes:

- Logging stays on the app core on purpose: RRAM writes stall *both*
  cores' RRAM fetches (FLPR is immune only because it runs from SRAM),
  the flash driver has no cross-core arbitration, and putting flash
  latency in the FLPR would jitter the sampler.  Measured logging cost
  is single-digit CPU %.
- Bulk BLE notification streams must be credit-paced (≤2 in flight,
  gated on the sent-callback): unpaced `bt_gatt_notify` exhausts host
  TX contexts and then blocks forever.
- The status LED (P0.01): boot = 3 s of triple-flashes (reset marker),
  then 1 flash/s heartbeat; BLE-switchable via 0xFFED.

## Motor chain

STBB1-APUR buck-boost (enable P2.03) generates VDC 0.75–4.2 V, set via
MAX5419 digipot over I2C (tap-by-tap ramp; Vout = 0.5·(1+100/(12.1+R)),
R = 200·(255−tap)/255).  DRV8212P drives the LRA from PWM20 at a fixed
50 % duty — frequency (4–1000 Hz) and VDC are the independent research
variables; duty is the drive scheme, not a control.
