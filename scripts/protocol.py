"""BLE protocol definitions for the Caterpillar device.

Mirrors src/interface/ble_interface.c (firmware >= v1.2.0).  This file
is the GUI's single source of truth — it deliberately shares no code
with scripts/ble_control.py.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

DEVICE_NAME = "Caterpillar"


def _u16(val: int) -> str:
    return f"0000{val:04x}-0000-1000-8000-00805f9b34fb"


UUID_FREQ = _u16(0xFFE1)      # write u16 LE: PWM Hz (4-1000)
UUID_VOLT = _u16(0xFFE2)      # write u16 LE: VDC mV (750-4200)
UUID_VDC_MEAS = _u16(0xFFE3)  # read u16 LE: measured VDC mV
UUID_RAIL = _u16(0xFFE4)      # write u8: rail enable
UUID_DRV = _u16(0xFFE5)       # write u8: driver awake
UUID_STATUS = _u16(0xFFE6)    # read: 44 B status packet v3
UUID_TPUT = _u16(0xFFE7)      # write: byte sink / read u32 LE counter
UUID_IMU_CFG = _u16(0xFFE8)   # write 4 B / read 12 B
UUID_STREAM = _u16(0xFFE9)    # notify: 8 B header + N x 16 B samples
UUID_LOG_CTL = _u16(0xFFEA)   # write cmd / read 20 B state
UUID_DUMP = _u16(0xFFEB)      # write 8 B request / notify chunks
UUID_MSG = _u16(0xFFEC)       # notify: text lines
UUID_LED = _u16(0xFFED)       # write u8 / read u8: heartbeat LED enable
UUID_TIME = _u16(0xFFEE)      # write u32 LE unix epoch / read device epoch
UUID_DIR = _u16(0xFFEF)       # read: session directory (newest first)

FREQ_MIN_HZ, FREQ_MAX_HZ = 4, 1000
VOLT_MIN_MV, VOLT_MAX_MV = 750, 4200

# IMU_ODR_* codes -> Hz
ODR_HZ = {1: 12.5, 2: 26, 3: 52, 4: 104, 5: 208,
          6: 416, 7: 833, 8: 1660, 9: 3330, 10: 6660}

CONTENT_ACCEL = 0x01
CONTENT_GYRO = 0x02

ACCEL_FS_G = {0: 2, 1: 4, 2: 8, 3: 16}
GYRO_FS_DPS = {0: 250, 1: 500, 2: 1000, 3: 2000}

# Sensitivity per FS index (ASM330LHH datasheet)
ACCEL_MG_PER_LSB = {0: 0.061, 1: 0.122, 2: 0.244, 3: 0.488}
GYRO_MDPS_PER_LSB = {0: 8.75, 1: 17.5, 2: 35.0, 3: 70.0}

LOG_POLICY_STOP = 0
LOG_POLICY_CIRCULAR = 1
LOG_CMD_STOP = 0
LOG_CMD_START = 1
LOG_CMD_ERASE = 2

LOG_HDR_MAGIC = 0x31474C43  # "CLG1"
LOG_HDR_SIZE = 32
RECORD_SIZE = 16

RESET_BITS = {0: "pin", 1: "soft", 2: "brownout", 3: "POR",
              4: "watchdog", 5: "debug"}


def encode_freq(hz: int) -> bytes:
    return struct.pack("<H", hz)


def encode_volt_mv(mv: int) -> bytes:
    return struct.pack("<H", mv)


def encode_onoff(on: bool) -> bytes:
    return bytes([1 if on else 0])


def encode_imu_cfg(odr_code: int, content: int,
                   accel_fs: int, gyro_fs: int) -> bytes:
    return bytes([odr_code, content, accel_fs, gyro_fs])


def encode_log_cmd(cmd: int, policy: int = LOG_POLICY_STOP) -> bytes:
    return bytes([cmd, policy])


def encode_dump_req(session: int, offset: int, length: int) -> bytes:
    return struct.pack("<III", session, offset, length)


def encode_time(epoch: int) -> bytes:
    return struct.pack("<I", epoch)


@dataclass
class Status:
    fw: tuple[int, int, int]
    flpr_fw: tuple[int, int, int] | None
    freq_hz: int
    duty1: int
    duty2: int
    vdc_target_mv: int
    vdc_meas_mv: int
    rail_on: bool
    drv_awake: bool
    imu_ok: bool
    led_on: bool
    uptime_s: int
    reset_cause: int
    odr_code: int
    content: int
    log_active: bool
    log_policy: int
    log_bytes: int
    log_capacity: int
    overruns: int


def decode_status(data: bytes) -> Status:
    if len(data) < 44 or data[0] != 3:
        raise ValueError(f"unsupported status packet v{data[0]}, "
                         f"{len(data)} B (firmware too old?)")
    (_, maj, mi, pa, freq, d1, d2, tgt, meas, rail, drv, imu, led,
     uptime, cause, flpr) = struct.unpack_from("<4BH2B2H4B3I", data, 0)
    odr, content, log_on, log_pol, log_bytes, log_cap, overruns = \
        struct.unpack_from("<4B3I", data, 28)
    flpr_fw = ((flpr >> 16) & 0xFF, (flpr >> 8) & 0xFF, flpr & 0xFF) \
        if flpr else None
    return Status((maj, mi, pa), flpr_fw, freq, d1, d2, tgt, meas,
                  bool(rail), bool(drv), bool(imu), bool(led), uptime,
                  cause, odr, content, bool(log_on), log_pol, log_bytes,
                  log_cap, overruns)


@dataclass
class ImuCfg:
    odr_code: int
    content: int
    accel_fs: int
    gyro_fs: int
    applied: bool
    status: int
    decim: int
    overruns: int


def decode_imu_cfg(data: bytes) -> ImuCfg:
    odr, content, afs, gfs, applied, st, decim, _ = \
        struct.unpack_from("<8B", data, 0)
    (ovr,) = struct.unpack_from("<I", data, 8)
    return ImuCfg(odr, content, afs, gfs, bool(applied),
                  struct.unpack("<b", bytes([st]))[0], decim, ovr)


@dataclass
class LogState:
    active: bool
    policy: int
    bytes_stored: int
    capacity: int
    records_total: int
    overruns: int


def decode_log_state(data: bytes) -> LogState:
    active, policy, _, _ = struct.unpack_from("<4B", data, 0)
    stored, cap, total, ovr = struct.unpack_from("<4I", data, 4)
    return LogState(bool(active), policy, stored, cap, total, ovr)


@dataclass
class StreamPacket:
    decim: int
    dropped: int
    samples: bytes            # n * 16 B raw records


def decode_stream_packet(data: bytes) -> StreamPacket:
    n, decim, _flags, _r = struct.unpack_from("<4B", data, 0)
    (dropped,) = struct.unpack_from("<I", data, 4)
    return StreamPacket(decim, dropped, data[8:8 + n * RECORD_SIZE])


@dataclass
class DumpChunk:
    offset: int
    last: bool
    data: bytes


def decode_dump_chunk(data: bytes) -> DumpChunk:
    offset, n, last, _ = struct.unpack_from("<IHBB", data, 0)
    return DumpChunk(offset, bool(last), data[8:8 + n])


@dataclass
class Session:
    seq: int
    wall_start: int       # unix epoch, 0 = clock was unsynced
    rec_count: int
    odr_code: int
    content: int
    accel_fs: int
    gyro_fs: int

    @property
    def bytes(self) -> int:
        return self.rec_count * RECORD_SIZE


def decode_sessions(data: bytes) -> list[Session]:
    n = data[0]
    out = []
    for i in range(n):
        seq, wall, count = struct.unpack_from("<3I", data, 4 + i * 16)
        odr, content, afs, gfs = struct.unpack_from("<4B", data, 16 + i * 16)
        out.append(Session(seq, wall, count, odr, content, afs, gfs))
    return out


@dataclass
class LogHeader:
    session_id: int
    odr_code: int
    content: int
    accel_fs: int
    gyro_fs: int
    policy: int
    record_size: int
    start_uptime_s: int


def decode_log_header(data: bytes) -> LogHeader:
    (magic, session, odr, content, afs, gfs, policy, rec_sz, _r,
     uptime) = struct.unpack_from("<2I6BHI", data, 0)
    if magic != LOG_HDR_MAGIC:
        raise ValueError("log header magic mismatch (no session recorded?)")
    return LogHeader(session, odr, content, afs, gfs, policy,
                     rec_sz, uptime)


def unpack_records(raw: bytes):
    """Yield (ax, ay, az, gx, gy, gz, temp_raw, seq16) int tuples."""
    for off in range(0, len(raw) - RECORD_SIZE + 1, RECORD_SIZE):
        yield struct.unpack_from("<7hH", raw, off)
