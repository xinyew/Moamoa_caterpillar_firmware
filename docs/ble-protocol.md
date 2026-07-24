# BLE Protocol Reference

Device name: **`Cat-NN`** (assigned fleet id) or **`Cat-XXXX`**
(unassigned — last 4 hex digits of the chip's FICR device id, same
self-naming scheme as biosensor-in-vivo) · single connection ·
no pairing/bonding (intentionally open research device).  All
multi-byte fields are **little-endian**.  Authoritative
implementations: `src/ble/ble_interface.c` (firmware) and
`scripts/protocol.py` (host spec used by the GUI);
`scripts/ble_control.py` carries its own inlined copies.
**Change the protocol → update all three.**

## Advertising data (fleet identity, fw ≥ 1.5.0)

Hosts can enumerate a fleet without connecting: `fleet.py scan`,
`ble_control.py --scan`, or the GUI's Scan + device dropdown match the
name prefix `Cat-` and decode manufacturer data (test company id
`0xFFFF`):

| Off | Content |
|---|---|
| 0–1 | company id `0xFFFF` LE |
| 2 | robot id (0 = unassigned) |
| 3–5 | app fw major/minor/patch |
| 6 | flags: bit0 = log session active |

## Services

| Service | UUID | Purpose |
|---|---|---|
| Caterpillar control | 16-bit `0xFFE0` | everything below |
| MCUmgr SMP | `8D53DC1D-1DB7-4CD3-868B-8A527460AA84` | OTA DFU (see build-flash-ota.md) |

## Characteristics (service 0xFFE0)

All writable characteristics accept both Write Request (acked) and
Write Without Response.

| UUID | Access | Content |
|---|---|---|
| 0xFFE1 | write | PWM frequency, u16 Hz, 4–1000 |
| 0xFFE2 | write | Motor VDC, u16 mV, 750–4200 (digipot ramps tap-by-tap) |
| 0xFFE3 | read | Measured VDC, u16 mV (AIN4 divider) |
| 0xFFE4 | write | Motor rail enable, u8 0/1 (STBB1-APUR) |
| 0xFFE5 | write | Motor driver awake, u8 0/1 (DRV8212 nSLEEP) |
| 0xFFE6 | read | Status packet v3, 44 B (below) |
| 0xFFE7 | write+read | Throughput sink: writes counted and discarded; read u32 byte counter |
| 0xFFE8 | write+read | IMU sampling config (below) |
| 0xFFE9 | notify | Live IMU stream (below) |
| 0xFFEA | write+read | Log control / state (below) |
| 0xFFEB | write+notify | Session dump request / chunk stream (below) |
| 0xFFEC | notify | Device warning/error text lines (UTF-8, replaces RTT) |
| 0xFFED | write+read | Heartbeat-LED enable, u8 0/1 |
| 0xFFEE | write+read | Wall-clock sync: write u32 unix epoch (UTC); read device epoch estimate (0 = never synced since boot) |
| 0xFFEF | read | Session directory (below) |
| 0xFFF0 | read | Tier-2 log: last 2 KB of warning/error lines (uptime-stamped text; includes history from before the connection) |
| 0xFFF1 | write+read | Fleet robot id, u8 0–20 (0 = unassigned; persisted; advertised name updates on the next adv restart, i.e. after disconnect) |

## Status packet (0xFFE6 read, 44 B, version 3)

| Off | Type | Field |
|---|---|---|
| 0 | u8 | packet version = 3 (reject others) |
| 1–3 | 3×u8 | app firmware major/minor/patch |
| 4 | u16 | PWM frequency [Hz] |
| 6, 7 | u8 | duty IN1, IN2 [%] |
| 8 | u16 | target VDC [mV] (0 = never set) |
| 10 | u16 | measured VDC [mV] |
| 12–15 | 4×u8 | rail on, driver awake, IMU ok, LED on |
| 16 | u32 | uptime [s] |
| 20 | u32 | boot reset cause (Zephyr hwinfo bits) |
| 24 | u32 | FLPR fw version `0x00MMmmpp` (0 = FLPR not running) |
| 28, 29 | u8 | IMU ODR code, content mask |
| 30, 31 | u8 | log active; byte 31: bit0 log policy (always 1 = circular), bit1 session is detached |
| 32 | u32 | current/last session bytes stored |
| 36 | u32 | log capacity bytes (695 648 = 43 478 records) |
| 40 | u32 | FLPR ring overruns since boot |

## IMU sampling config (0xFFE8)

Write 4 or 6 B: `{odr_code u8, content u8, accel_fs u8, gyro_fs u8
[, preview_hz u16]}`.  Logging always runs at the full ODR;
`preview_hz` caps only the live stream (0/absent = auto).  The stream
paces itself to the link: the base budget is ~1300 samples/s idle and
~500 while a log session runs (the widened connection interval only
carries ~600 on a good day), an explicit `preview_hz` above the
active budget is clamped to it, and on sustained FIFO drops the
device further halves the preview rate within a second (up to 16×,
probing back up after quiet periods — with escalating patience if
probes keep failing).  Result: the preview rate follows what the link
actually delivers instead of overflowing into bursty gaps.  Hosts
must treat the per-packet `decim` byte as dynamic.

- `odr_code`: 1=12.5, 2=26, 3=52, 4=104, 5=208, 6=416, 7=833, 8=1660,
  9=3330, 10=6660 Hz.  Defaults persist in device settings (factory
  default 7 = 833 Hz).
- `content`: bit0 accel, bit1 gyro (0 rejected in writes; the sensor
  itself uses 0 = powered down)
- `accel_fs`: 0=±2 g, 1=±4 g, 2=±8 g, 3=±16 g
- `gyro_fs`: 0=±250, 1=±500, 2=±1000, 3=±2000 dps

Read 12 B: `{odr, content, accel_fs, gyro_fs, applied u8,
status i8 (0 ok / −errno), decim u8, rsvd, overruns u32}`.
`applied=1` confirms the FLPR programmed the sensor.

**On-demand sampling**: the sensor is powered down unless a log
session runs or the stream is subscribed.  The read reports the
*applied* state, so `content` reads 0 while idle — the configured
values are remembered in settings and applied on the next enable
(~100 ms wake).

## Sample record (16 B, used by stream and log)

`{ax,ay,az,gx,gy,gz: i16 raw LSB, temp_raw: i16, seq16: u16}`

- Scale on the host: accel mg/LSB = 0.061/0.122/0.244/0.488 by FS;
  gyro mdps/LSB = 8.75/17.5/35/70 by FS; temp °C = raw/256 + 25.
- `seq16` = low 16 bits of the FLPR's total sample count *including
  dropped samples* — consecutive records normally differ by the
  stream decimation (stream) or 1 (log); larger deltas mark honest
  data loss (ring overruns), delta 0 or reversal would mean corruption.

## Live stream (0xFFE9 notify)

Packet: `{n u8, decim u8, flags u8, rsvd u8, dropped u32}` + n×16 B
records (n ≤ 14, needs MTU ≥ 243).  `dropped` counts stream-side
discards (buffer pressure), cumulative per subscription.  Time axis on
the host must come from `seq16 / ODR`, **not** packet arrival time
(notifications arrive in bursts per connection event).

## Log control (0xFFEA)

Write: `{cmd u8, arg u8}` — cmd 0 = stop, 1 = start (arg ignored;
storage is always circular), 2 = erase all sessions, 3 = start
**detached** (fleet mode: the session keeps logging after the host
disconnects; stop it on a later connection with cmd 0).
Read 20 B: `{active u8, policy u8, rsvd u16, bytes_stored u32,
capacity u32, records_total u32, overruns u32}`.

**Commands execute asynchronously** (queued off the BT RX thread): the
write ack means *accepted*; completion is visible via the status poll
and the directory.  Stop can take up to ~1 s (flash-writer drain) —
hosts should settle before refreshing 0xFFEF.  The same applies to
VDC writes (0xFFE2): the ramp runs in the background at 10 ms/tap.

A running session is auto-stopped by BLE disconnect — unless it was
started with cmd 3 (detached), in which case it survives disconnects
and reboots of the host; the advertising flag (bit0 of the mfg-data
flags byte) shows it running from the outside.

## Session directory (0xFFEF read)

`{count u8, rsvd 3×u8}` + count × 16 B entries, **newest first**
(count ≤ 16): `{seq u32, wall_start u32 (unix epoch, 0 = clock was
unsynced), rec_count u32, odr_code u8, content u8, accel_fs u8,
gyro_fs u8}`.  `rec_count` is the *readable* count — clipped when the
circular ring has overwritten part of the session.  Directory contents
survive reboot but are wiped on the first boot of new firmware.

## Session dump (0xFFEB)

Request (write 12 B): `{session_seq u32, offset u32, len u32}` —
offset/len in bytes relative to the session's readable data.
A new request cancels any dump in flight.

Chunks (notify): `{offset u32, n u16, last u8, rsvd u8}` + n data
bytes (n ≤ 232).  `last=1` marks the final chunk (also sent on a short
read).  The device paces itself (≤2 notifications in flight — unpaced
notify streams wedge the Bluetooth host, see git history) and expects
the host to **resume by re-requesting from the last received offset**
if the stream goes quiet; failures are announced on 0xFFEC.
Effective rate ≈ 18–24 KiB/s.

## Host session flow (what the GUI does)

1. Scan by the `Cat-` name prefix into a device dropdown; connect to
   the selected robot with GATT cache disabled
   (`winrt=dict(use_cached_services=False)` — Windows serves stale
   tables after OTAs otherwise), subscribe 0xFFEC + 0xFFEB.
2. Write 0xFFEE with the current unix time (timestamps for sessions).
3. Poll 0xFFE6 at 1 Hz; re-read 0xFFEF whenever the session set changes.
4. Apply 0xFFE8 config → start log (0xFFEA) + subscribe 0xFFE9
   together; stop both together, settle ~1 s (async stop), then
   refresh 0xFFEF.
5. Dump: pick a session from 0xFFEF, request `{seq, 0, rec_count*16}`,
   reassemble chunks by offset, resume on 5 s silence.

Throughput probe: read 0xFFE7 counter, flood ~64 KiB of max-MTU
writes-without-response, read counter again — device-confirmed KiB/s.
Healthy Windows link: 18–26 KiB/s; don't run while streaming.
