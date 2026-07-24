# IMU Sampling, Logging & Dump

## Data path

```
ASM330LHH --SPI 8 MHz / DRDY--> FLPR core (RISC-V)
    --2048-record SPSC ring in shared SRAM @0x20036000-->
app pump thread (4 ms poll)
    ├─> RRAM session log   (ALWAYS full configured rate)
    └─> BLE live stream    (decimated preview, 0xFFE9)
```

- **Sampling is on-demand** (v1.4.0): the sensor is powered down unless
  a log session runs or a live stream is subscribed, waking within
  ~100 ms of either.  Config written while idle is remembered
  (settings) and applied on the next enable; the 0xFFE8 *read* shows
  the *applied* state, so content reads 0 while idle.
- The FLPR samples at the configured ODR (12.5 Hz–6.66 kHz) and never
  blocks: if the shared ring is full it drops the sample and counts it
  (`overruns` in status/config reads, announced on 0xFFEC).
- The pump drains every 4 ms; the ring buffers ~0.3 s at 6.66 kHz,
  ~2.5 s at 833 Hz.
- Preview decimation is pick-every-Nth with **no anti-alias filter**:
  content above half the preview rate aliases in the live plot.  The
  full-rate log/dump is the only spectrally valid record.

## Rate envelope (measured, v1.3.7 writer-thread architecture)

FLPR-ring overruns are **eliminated at every ODR** (pump does memcpy
only; stream TX and flash writes run in their own threads).  The one
remaining bottleneck is flash bandwidth while BLE-connected: every
flash op waits for an MPSL radio timeslot, sustaining ~90–95 KiB/s
against the 104 KiB/s demand of 6.66 kHz.  Loss, when it occurs, is
counted (`log write backlog` on 0xFFEC) and marked by seq16 gaps.

| ODR | Stream | Logging while connected | Ring time in 679 KB log |
|---|---|---|---|
| ≤1660 Hz | full/×2 | clean (0 loss) | ≥26 s |
| 3330 Hz | ×3 auto | ~0–2 % (0 with a capped preview) | ~13 s |
| 6660 Hz | capped preview | ~10 % backlog loss | ~6.5 s |
| 6660 Hz | auto preview | ~20 % backlog loss | ~6.5 s |

The device widens the connection interval to ~50 ms during a session
(restored on stop) to free radio air for flash timeslots.  NOTE:
raising BT TX buffer counts makes this WORSE (longer radio events);
see prj.conf.

## Flash layout (absolute RRAM offsets)

| Range | Content |
|---|---|
| 0xB9000–0xBA000 | **Reserved, kept erased.** MCUboot reads this as the secondary-slot image header on every boot and *erases the block* if it holds non-FF bytes that aren't a valid image (verified — it ate an early directory). |
| 0xBA000–0xBA020 | Directory meta: magic "CLOG", layout version, **owning-firmware stamp** |
| 0xBA020–0xBA220 | 16 session entries × 32 B |
| 0xBA220–0x163F80 | Record ring: 695 648 B = 43 478 × 16 B records |
| 0x163F80–0x164000 | Reserved (MCUboot swap-trailer area) |

Never extend past 0x165000: the app core's flash device
(`cpuapp_rram`) ends there and returns −EINVAL — writes are silently
lost and reads wedge (this was the "dump stuck at 78–83 %" bug).
Extending the DT flash size would re-layout the partitions and break
OTA compatibility.

## Sessions

- **Circular only.** A session runs from start until the stop command
  or BLE disconnect (auto-closed).  It never stops for space: the
  write head overwrites the oldest data in the ring.
- When the head reaches the *beginning of an older session*, that
  session's entry is invalidated and `session #N overwritten by
  session #M — removed` is sent on 0xFFEC; the GUI list refreshes
  within a second.
- A session **larger than the ring** keeps only the newest
  ~43 478 records; its directory `rec_count` reflects the surviving
  window (hardware-verified: 53 k written → 43 414 listed and dumped).
- Entries persist across reboots (counter state is rebuilt from the
  directory at boot; a crash mid-session loses at most the last ~8 KB
  of *accounting*, the entry refreshes every 512 records).
- **Every firmware update wipes all sessions**: the directory carries
  the owning firmware's version stamp; a mismatch (or the DFU upload
  physically overwriting the directory) triggers a wipe on first boot.
- The log region is the OTA secondary slot: a DFU upload overwrites
  log data and vice versa.  Dump before updating.

## Dump & file formats

Per-session, offset-addressable, resumable — see
[ble-protocol.md](ble-protocol.md).  The GUI saves both:

- **CSV**: header comment (session, start time, ODR, FS ranges), then
  `t_s, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c, seq16`.
  `t_s = index/ODR` relative to the first *surviving* sample.
- **NPZ**: arrays `t_s, accel_g, gyro_dps, temp_c, seq16` plus scalars
  `odr_hz, accel_fs_g, gyro_fs_dps, session_id, wall_start_epoch`.

Integrity checking a dump: successive `seq16` deltas of 1 are
contiguous; deltas >1 mark counted overrun losses; delta 0 or a
reversal would indicate real corruption (never observed post-v1.3.6).

## Known caveats

- Sessions recorded on firmware **older than v1.3.6** that exceeded
  ~695 KB have a never-written garbage tail (the region-B bug) — treat
  those saved files as suspect beyond that point.
- Session wall-clock time stamps the start *button press*; for a
  self-wrapped session the first dumped sample is later than that (use
  seq16 to reconstruct the trim).
