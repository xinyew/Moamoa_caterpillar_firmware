"""Caterpillar control GUI.

Standalone PySide6 + pyqtgraph + bleak application:
  * motor control (PWM frequency, VDC, rail, driver)
  * IMU sampling config (ODR / content / full-scale, live-applied)
  * live sample stream with scrolling plots
  * on-chip log control + dump to CSV / NPZ
  * device warning/error console (replaces RTT)

Run:  python caterpillar_gui.py
Deps: pip install -r requirements.txt
"""

from __future__ import annotations

import asyncio
import struct
import sys
import time
from pathlib import Path

import numpy as np
from bleak import BleakClient, BleakScanner
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QPlainTextEdit, QProgressBar, QPushButton, QSpinBox, QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg
import qasync

import protocol as P

PLOT_WINDOW_S = 5.0
PLOT_MAX_POINTS = 20000
SMOOTH_LAG_S = 0.15     # display latency absorbing per-conn-event bursts


class StreamBuffer:
    """Preallocated sample store for the live plots.

    Appends are O(new samples) — no reallocation per packet (the old
    np.concatenate churn moved up to ~500 KB per notification on the
    thread that also services BLE I/O).  The newest samples are always
    readable as contiguous views; compaction copies the newest `cap`
    samples once per `cap` appended (amortized O(1)/sample).
    """

    def __init__(self, cap: int = PLOT_MAX_POINTS):
        self.cap = cap
        self.t = np.zeros(2 * cap)
        self.acc = np.zeros((2 * cap, 3), np.float32)
        self.gyr = np.zeros((2 * cap, 3), np.float32)
        self.len = 0

    def clear(self):
        self.len = 0

    def append(self, t: np.ndarray, acc: np.ndarray, gyr: np.ndarray):
        n = len(t)
        if n >= self.cap:                      # absurd burst: keep newest
            t, acc, gyr = t[-self.cap:], acc[-self.cap:], gyr[-self.cap:]
            n = self.cap
        if self.len + n > 2 * self.cap:        # compact to newest cap
            keep = self.cap
            self.t[:keep] = self.t[self.len - keep:self.len]
            self.acc[:keep] = self.acc[self.len - keep:self.len]
            self.gyr[:keep] = self.gyr[self.len - keep:self.len]
            self.len = keep
        sl = slice(self.len, self.len + n)
        self.t[sl] = t
        self.acc[sl] = acc
        self.gyr[sl] = gyr
        self.len += n


class DumpPlotWindow(QWidget):
    """Full-log viewer: accel + gyro vs time, linked x-axis, auto-
    downsampled so multi-hundred-thousand-sample logs pan smoothly."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU log viewer")
        self.resize(1000, 620)
        lay = QVBoxLayout(self)
        self.lbl = QLabel("—")
        lay.addWidget(self.lbl)

        self.p_acc = pg.PlotWidget(title="Accelerometer [g]")
        self.p_gyr = pg.PlotWidget(title="Gyroscope [dps]")
        for p in (self.p_acc, self.p_gyr):
            p.showGrid(x=True, y=True, alpha=0.2)
            p.addLegend(offset=(10, 10))
            p.setDownsampling(auto=True, mode="peak")
            p.setClipToView(True)
        self.p_gyr.setXLink(self.p_acc)
        self.p_gyr.setLabel("bottom", "time", units="s")
        lay.addWidget(self.p_acc, 1)
        lay.addWidget(self.p_gyr, 1)

        colors = ("#e6194b", "#3cb44b", "#4363d8")
        self.c_acc = [self.p_acc.plot(pen=pg.mkPen(c, width=1), name=n)
                      for c, n in zip(colors, ("ax", "ay", "az"))]
        self.c_gyr = [self.p_gyr.plot(pen=pg.mkPen(c, width=1), name=n)
                      for c, n in zip(colors, ("gx", "gy", "gz"))]

    def set_data(self, t, acc, gyr, title: str):
        self.lbl.setText(title)
        for i, c in enumerate(self.c_acc):
            c.setData(t, acc[:, i])
        for i, c in enumerate(self.c_gyr):
            c.setData(t, gyr[:, i])
        self.p_acc.autoRange()
        self.p_gyr.autoRange()


class BleWorker:
    """Owns the BleakClient and all GATT traffic (runs on the qasync loop)."""

    def __init__(self, ui: "MainWindow"):
        self.ui = ui
        self.client: BleakClient | None = None
        self.last_name: str | None = None
        self._dump_buf = bytearray()
        self._dump_expected = 0
        self._dump_done: asyncio.Event | None = None

    @property
    def connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    async def scan_fleet(self, timeout: float = 5.0) -> list[dict]:
        """All visible Caterpillars: {name, address, robot_id, fw,
        session_active, rssi} — identity fields None on legacy fw."""
        found: dict[str, dict] = {}

        def on_adv(device, adv):
            name = device.name or adv.local_name
            info = P.parse_adv(name, adv.manufacturer_data)
            if info is None:
                return
            # Keep the BLEDevice handle: connecting by bare address
            # fails on WinRT unless Windows has the device cached.
            info.update(address=device.address, rssi=adv.rssi,
                        dev=device)
            found[device.address] = info

        scanner = BleakScanner(on_adv)
        await scanner.start()
        await asyncio.sleep(timeout)
        await scanner.stop()
        return sorted(found.values(),
                      key=lambda e: (e["robot_id"] is None,
                                     e["robot_id"] or 0, e["name"]))

    async def connect(self, target: dict):
        dev = target["dev"]
        self.last_name = target["name"]
        self.ui.log(f"Connecting to {target['name']} "
                    f"[{target['address']}] ...")
        # Windows caches GATT tables across firmware updates; don't.
        self.client = BleakClient(
            dev, disconnected_callback=self._on_disconnect,
            winrt=dict(use_cached_services=False))
        await self.client.connect()
        self.ui.log(f"Connected ({dev.address}, MTU={self.client.mtu_size})")

        await self.client.start_notify(P.UUID_MSG, self._on_msg)
        await self.client.start_notify(P.UUID_DUMP, self._on_dump_chunk)

        # Sync the device's wall clock so log sessions carry real
        # start timestamps (UTC epoch; rendered local on display).
        now = int(time.time())
        await self.client.write_gatt_char(P.UUID_TIME, P.encode_time(now),
                                          response=True)
        self.ui.log(f"Device clock synced: "
                    f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now))}")
        return True

    async def disconnect(self):
        if self.client is not None:
            try:
                await self.client.disconnect()
            except Exception:
                pass
            self.client = None

    def _on_disconnect(self, _client):
        self.ui.on_disconnected()

    def _on_msg(self, _char, data: bytearray):
        self.ui.log(f"[device] {data.decode(errors='replace')}")

    # ---- motor ----------------------------------------------------------

    async def set_freq(self, hz: int):
        await self.client.write_gatt_char(P.UUID_FREQ, P.encode_freq(hz),
                                          response=True)
        self.ui.log(f"PWM frequency -> {hz} Hz")

    async def set_volt(self, mv: int):
        await self.client.write_gatt_char(P.UUID_VOLT, P.encode_volt_mv(mv),
                                          response=True)
        self.ui.log(f"VDC -> {mv} mV")

    async def set_rail(self, on: bool):
        await self.client.write_gatt_char(P.UUID_RAIL, P.encode_onoff(on),
                                          response=True)
        self.ui.log(f"Motor rail -> {'ON' if on else 'OFF'}")

    async def set_drv(self, on: bool):
        await self.client.write_gatt_char(P.UUID_DRV, P.encode_onoff(on),
                                          response=True)
        self.ui.log(f"Motor driver -> {'AWAKE' if on else 'SLEEP'}")

    async def read_status(self) -> P.Status | None:
        data = await self.client.read_gatt_char(P.UUID_STATUS)
        return P.decode_status(bytes(data))

    async def set_led(self, on: bool):
        await self.client.write_gatt_char(P.UUID_LED, P.encode_onoff(on),
                                          response=True)
        self.ui.log(f"Heartbeat LED -> {'ON' if on else 'OFF'}")

    async def set_robot_id(self, new_id: int):
        await self.client.write_gatt_char(P.UUID_ROBOT_ID,
                                          bytes([new_id]), response=True)
        if new_id:
            self.ui.log(f"Robot ID -> {new_id} (name becomes "
                        f"Cat-{new_id:02d} after disconnect)")
        else:
            self.ui.log("Robot ID cleared (name reverts to Cat-XXXX "
                        "after disconnect)")

    async def read_robot_id(self) -> int:
        data = await self.client.read_gatt_char(P.UUID_ROBOT_ID)
        return data[0] if data else 0

    async def tput_test(self, kib: int = 64) -> float:
        """Send a bounded burst into the 0xFFE7 sink and measure what
        the device actually received — an end-to-end link-quality
        number (healthy on Windows: ~18-26 KiB/s)."""
        payload = bytes(max(20, self.client.mtu_size - 3))
        total = kib * 1024

        d = await self.client.read_gatt_char(P.UUID_TPUT)
        c0 = struct.unpack("<I", bytes(d))[0]
        t0 = time.monotonic()
        sent = 0
        writes = 0
        while sent < total:
            await self.client.write_gatt_char(P.UUID_TPUT, payload,
                                              response=False)
            sent += len(payload)
            if (writes := writes + 1) % 16 == 0:
                await asyncio.sleep(0)    # keep the Qt loop breathing
        d = await self.client.read_gatt_char(P.UUID_TPUT)
        el = time.monotonic() - t0
        rx = (struct.unpack("<I", bytes(d))[0] - c0) & 0xFFFFFFFF

        rate = rx / el / 1024
        loss = f", {sent - rx} B lost" if rx != sent else ", no loss"
        self.ui.log(f"Throughput: {rate:.1f} KiB/s device-confirmed "
                    f"({rx}/{sent} B in {el:.1f} s{loss})")
        return rate

    # ---- IMU ------------------------------------------------------------

    async def set_imu_cfg(self, odr: int, content: int, afs: int, gfs: int,
                          preview_hz: int = 0):
        await self.client.write_gatt_char(
            P.UUID_IMU_CFG,
            P.encode_imu_cfg(odr, content, afs, gfs, preview_hz),
            response=True)
        pv = f"{preview_hz} Hz" if preview_hz else "auto"
        self.ui.log(f"IMU config -> log {P.ODR_HZ[odr]} Hz, preview {pv}, "
                    f"content=0x{content:x}, "
                    f"±{P.ACCEL_FS_G[afs]} g / ±{P.GYRO_FS_DPS[gfs]} dps")

    async def stream_start(self):
        await self.client.start_notify(P.UUID_STREAM, self.ui.on_stream_pkt)
        self.ui.log("Live stream started")

    async def stream_stop(self):
        try:
            await self.client.stop_notify(P.UUID_STREAM)
        except Exception:
            pass
        self.ui.log("Live stream stopped")

    # ---- log ------------------------------------------------------------

    async def log_cmd(self, cmd: int, policy: int = 0):
        await self.client.write_gatt_char(P.UUID_LOG_CTL,
                                          P.encode_log_cmd(cmd, policy),
                                          response=True)

    async def log_state(self) -> P.LogState:
        data = await self.client.read_gatt_char(P.UUID_LOG_CTL)
        return P.decode_log_state(bytes(data))

    async def read_sessions(self) -> list[P.Session]:
        data = await self.client.read_gatt_char(P.UUID_DIR)
        return P.decode_sessions(bytes(data))

    async def dump(self, session: int, total: int) -> bytes | None:
        """Fetch `total` bytes of one session's records, reassembled.

        Self-healing: BLE notification streams can stall (congestion,
        Windows stack hiccups, device-side abort) — if no chunk arrives
        for a few seconds, the request is re-issued from the highest
        offset received so far.
        """
        self._dump_buf = bytearray(total)
        self._dump_expected = total
        self._dump_max_end = 0
        self._dump_last_rx = time.monotonic()
        self._dump_done = asyncio.Event()
        t0 = time.monotonic()
        resumes = 0

        await self.client.write_gatt_char(
            P.UUID_DUMP, P.encode_dump_req(session, 0, total),
            response=True)

        while not self._dump_done.is_set():
            try:
                await asyncio.wait_for(self._dump_done.wait(), timeout=1.0)
                break
            except asyncio.TimeoutError:
                pass
            if time.monotonic() - self._dump_last_rx < 5.0:
                continue
            if resumes >= 20:
                self.ui.log(f"Dump failed after {resumes} resume attempts "
                            f"({self._dump_max_end}/{total} B)")
                return None
            resumes += 1
            frm = self._dump_max_end
            self.ui.log(f"Dump stalled — resuming at {frm}/{total} B "
                        f"(attempt {resumes})")
            self._dump_last_rx = time.monotonic()
            try:
                await self.client.write_gatt_char(
                    P.UUID_DUMP,
                    P.encode_dump_req(session, frm, total - frm),
                    response=True)
            except Exception as e:
                self.ui.log(f"Dump resume write failed: {e}")
                return None

        dt = time.monotonic() - t0
        note = f", {resumes} resume(s)" if resumes else ""
        self.ui.log(f"Dump complete: {total} B in {dt:.1f} s "
                    f"({total / dt / 1024:.1f} KiB/s{note})")
        return bytes(self._dump_buf)

    def _on_dump_chunk(self, _char, data: bytearray):
        if self._dump_done is None or self._dump_done.is_set():
            return
        c = P.decode_dump_chunk(bytes(data))
        end = min(c.offset + len(c.data), self._dump_expected)
        self._dump_buf[c.offset:end] = c.data[:end - c.offset]
        self._dump_max_end = max(self._dump_max_end, end)
        self._dump_last_rx = time.monotonic()
        self.ui.on_dump_progress(self._dump_max_end, self._dump_expected)
        if (c.last and end >= self._dump_expected) or \
                self._dump_max_end >= self._dump_expected:
            self._dump_done.set()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Caterpillar Control")
        self.resize(1100, 760)
        self.ble = BleWorker(self)

        # Stream plot state
        self._sbuf = StreamBuffer()
        self._stream_n = 0
        self._stream_t0 = None
        self._stream_rate = 0.0
        self._stream_dropped = 0
        self._rate_win: list[tuple[float, int]] = []  # (wall, n) per pkt
        self._disp_edge = None       # smoothed right edge (sensor time)
        self._frame_wall = None
        self._decim = 1              # stream decimation (from packets)
        self._gap_est = 0.05         # decaying max inter-packet gap [s]
        self._last_pkt_wall = None
        self._afs = 0
        self._gfs = 0
        self._dump_data = None       # (t, acc_g, gyr_dps, title)
        self._dump_win: DumpPlotWindow | None = None
        self._odr_hz = 833.0         # updated when config is applied
        self._seq_last = None        # stream seq16 unwrap state
        self._seq_abs = 0
        self._session_seqs = ()      # for change detection in the poll

        self._build_ui()

        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(33)
        self._plot_timer.timeout.connect(self._update_plots)

        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._kick_poll)
        self._poll_task: asyncio.Task | None = None

    def _kick_poll(self):
        """Spawn a status poll only if the previous one finished —
        polls must never pile up (reads can outlast the 1 s interval
        during dumps)."""
        if self._poll_task is None or self._poll_task.done():
            self._poll_task = asyncio.ensure_future(self._poll_status())

    async def _pick_file(self, save: bool, title: str, default: str,
                         filt: str):
        """Open a modal file dialog safely under qasync.

        A modal dialog spins a nested Qt event loop inside this
        coroutine's frame; if other asyncio tasks (the status poll)
        are stepped during that, Python >=3.12 raises 'Cannot enter
        into task ... while another task is being executed' storms.
        So: stop the poll timer, drain any in-flight poll, THEN open
        the dialog.
        """
        self._status_timer.stop()
        if self._poll_task is not None and not self._poll_task.done():
            try:
                await self._poll_task
            except Exception:
                pass
        try:
            if save:
                return QFileDialog.getSaveFileName(self, title, default,
                                                   filt)
            return QFileDialog.getOpenFileName(self, title, default, filt)
        finally:
            if self.ble.connected:
                self._status_timer.start()

    # ---- UI construction ------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        root = QHBoxLayout(central)

        # Left column: connection, motor, IMU cfg, log
        left = QVBoxLayout()
        left.setSpacing(10)

        conn_box = QGroupBox("Connection")
        cl = QGridLayout(conn_box)
        self.cmb_devices = QComboBox()
        self.cmb_devices.setPlaceholderText("scan for robots…")
        self.cmb_devices.setToolTip(
            "Visible Caterpillars (Cat-*). Scan refreshes the list; "
            "Connect targets the selected robot.")
        self.btn_scan = QPushButton("Scan")
        self.btn_scan.clicked.connect(
            lambda: asyncio.ensure_future(self._do_scan()))
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(
            lambda: asyncio.ensure_future(self._do_connect()))
        self.lbl_conn = QLabel("disconnected")
        self.lbl_fw = QLabel("—")
        self.chk_led = QCheckBox("Heartbeat LED")
        self.chk_led.setChecked(True)
        self.chk_led.clicked.connect(lambda on: asyncio.ensure_future(
            self.ble.set_led(on)))
        self.btn_tput = QPushButton("Throughput test")
        self.btn_tput.clicked.connect(
            lambda: asyncio.ensure_future(self._run_tput()))
        self.btn_devlog = QPushButton("Device log")
        self.btn_devlog.setToolTip(
            "Read the device's recent warning/error history (2 KB ring "
            "— includes lines from before this connection).")
        self.btn_devlog.clicked.connect(
            lambda: asyncio.ensure_future(self._read_devlog()))
        self.spin_robot_id = QSpinBox()
        self.spin_robot_id.setRange(0, 20)
        self.spin_robot_id.setSpecialValueText("unassigned")
        self.spin_robot_id.setToolTip(
            "Fleet robot number: names the device Cat-NN so multiple "
            "robots are tellable apart when scanning (0 = unassigned, "
            "auto Cat-XXXX from chip id). Persisted on the robot; the "
            "advertised name updates after disconnect.")
        self.btn_robot_id = QPushButton("Assign ID")
        self.btn_robot_id.clicked.connect(lambda: asyncio.ensure_future(
            self.ble.set_robot_id(self.spin_robot_id.value())))
        cl.addWidget(self.cmb_devices, 0, 0)
        cl.addWidget(self.btn_scan, 0, 1)
        cl.addWidget(self.btn_connect, 1, 0)
        cl.addWidget(self.lbl_conn, 1, 1)
        cl.addWidget(QLabel("Firmware:"), 2, 0)
        cl.addWidget(self.lbl_fw, 2, 1)
        cl.addWidget(self.chk_led, 3, 0)
        cl.addWidget(self.btn_tput, 3, 1)
        cl.addWidget(self.btn_devlog, 4, 0)
        rid = QHBoxLayout()
        rid.addWidget(self.spin_robot_id, 1)
        rid.addWidget(self.btn_robot_id)
        cl.addLayout(rid, 4, 1)
        left.addWidget(conn_box)

        motor_box = QGroupBox("Motor")
        ml = QGridLayout(motor_box)
        self.spin_freq = QSpinBox()
        self.spin_freq.setRange(P.FREQ_MIN_HZ, P.FREQ_MAX_HZ)
        self.spin_freq.setValue(150)
        self.spin_freq.setSuffix(" Hz")
        self.btn_freq = btn_freq = QPushButton("Set")
        btn_freq.clicked.connect(lambda: asyncio.ensure_future(
            self.ble.set_freq(self.spin_freq.value())))
        self.spin_volt = QDoubleSpinBox()
        self.spin_volt.setRange(P.VOLT_MIN_MV / 1000, P.VOLT_MAX_MV / 1000)
        self.spin_volt.setValue(3.0)
        self.spin_volt.setSingleStep(0.1)
        self.spin_volt.setDecimals(2)
        self.spin_volt.setSuffix(" V")
        self.btn_volt = btn_volt = QPushButton("Set")
        btn_volt.clicked.connect(lambda: asyncio.ensure_future(
            self.ble.set_volt(round(self.spin_volt.value() * 1000))))
        self.chk_rail = QCheckBox("Rail on")
        self.chk_rail.clicked.connect(lambda on: asyncio.ensure_future(
            self.ble.set_rail(on)))
        self.chk_drv = QCheckBox("Driver awake")
        self.chk_drv.clicked.connect(lambda on: asyncio.ensure_future(
            self.ble.set_drv(on)))
        self.lbl_vdc = QLabel("VDC: —")
        ml.addWidget(QLabel("Frequency"), 0, 0)
        ml.addWidget(self.spin_freq, 0, 1)
        ml.addWidget(btn_freq, 0, 2)
        ml.addWidget(QLabel("Voltage"), 1, 0)
        ml.addWidget(self.spin_volt, 1, 1)
        ml.addWidget(btn_volt, 1, 2)
        ml.addWidget(self.chk_rail, 2, 0)
        ml.addWidget(self.chk_drv, 2, 1)
        ml.addWidget(self.lbl_vdc, 3, 0, 1, 3)
        left.addWidget(motor_box)

        imu_box = QGroupBox("IMU sampling")
        il = QGridLayout(imu_box)
        self.cmb_odr = QComboBox()
        for code, hz in P.ODR_HZ.items():
            self.cmb_odr.addItem(f"{hz} Hz", code)
        self.cmb_odr.setCurrentIndex(6)  # 833 Hz boot default
        self.cmb_preview = QComboBox()
        self.cmb_preview.setToolTip(
            "Rate of the live BLE preview only — flash logging always "
            "records at the full ODR. Lower rates leave link headroom.")
        self.cmb_preview.addItem("auto (max)", 0)
        for hz in (833, 416, 208, 104, 52, 26, 13):
            self.cmb_preview.addItem(f"~{hz} Hz", hz)
        self.cmb_content = QComboBox()
        self.cmb_content.addItem("accel + gyro",
                                 P.CONTENT_ACCEL | P.CONTENT_GYRO)
        self.cmb_content.addItem("accel only", P.CONTENT_ACCEL)
        self.cmb_content.addItem("gyro only", P.CONTENT_GYRO)
        self.cmb_afs = QComboBox()
        for idx, g in P.ACCEL_FS_G.items():
            self.cmb_afs.addItem(f"±{g} g", idx)
        self.cmb_gfs = QComboBox()
        for idx, dps in P.GYRO_FS_DPS.items():
            self.cmb_gfs.addItem(f"±{dps} dps", idx)
        self.btn_apply = btn_apply = QPushButton("Apply config")
        btn_apply.clicked.connect(
            lambda: asyncio.ensure_future(self._apply_imu_cfg()))
        self.lbl_stream = QLabel("stream: off")
        il.addWidget(QLabel("Log ODR"), 0, 0)
        il.addWidget(self.cmb_odr, 0, 1)
        il.addWidget(QLabel("Preview rate"), 1, 0)
        il.addWidget(self.cmb_preview, 1, 1)
        il.addWidget(QLabel("Content"), 2, 0)
        il.addWidget(self.cmb_content, 2, 1)
        il.addWidget(QLabel("Accel FS"), 3, 0)
        il.addWidget(self.cmb_afs, 3, 1)
        il.addWidget(QLabel("Gyro FS"), 4, 0)
        il.addWidget(self.cmb_gfs, 4, 1)
        il.addWidget(btn_apply, 5, 0, 1, 2)
        il.addWidget(self.lbl_stream, 6, 0, 1, 2)
        left.addWidget(imu_box)

        log_box = QGroupBox("On-chip log (circular — oldest sessions "
                            "are overwritten)")
        ll = QGridLayout(log_box)
        self.btn_log = QPushButton("Start session (log + stream)")
        self.btn_log.setCheckable(True)
        self.btn_log.clicked.connect(
            lambda: asyncio.ensure_future(self._toggle_session()))
        self.btn_erase = btn_erase = QPushButton("Erase")
        btn_erase.clicked.connect(lambda: asyncio.ensure_future(
            self._erase_log()))
        self.bar_log = QProgressBar()
        self.bar_log.setFormat("%v / %m B")
        self.btn_dump = QPushButton("Dump to file…")
        self.btn_dump.clicked.connect(
            lambda: asyncio.ensure_future(self._dump_log()))
        self.bar_dump = QProgressBar()
        self.bar_dump.setVisible(False)
        self.cmb_sessions = QComboBox()
        self.cmb_sessions.setToolTip(
            "Stored sessions on the device (survive reboots). "
            "Refreshed on connect and after each session.")
        self.btn_plot = QPushButton("Plot last dump")
        self.btn_plot.setEnabled(False)
        self.btn_plot.clicked.connect(self._plot_dump)
        btn_open = QPushButton("Open .npz…")
        btn_open.clicked.connect(self._open_npz)
        ll.addWidget(self.btn_log, 1, 0)
        ll.addWidget(btn_erase, 1, 1)
        ll.addWidget(self.bar_log, 2, 0, 1, 2)
        ll.addWidget(self.cmb_sessions, 3, 0, 1, 2)
        ll.addWidget(self.btn_dump, 4, 0, 1, 2)
        ll.addWidget(self.bar_dump, 5, 0, 1, 2)
        ll.addWidget(self.btn_plot, 6, 0)
        ll.addWidget(btn_open, 6, 1)
        left.addWidget(log_box)

        left.addStretch(1)
        root.addLayout(left, 0)

        # Right column: plots + console
        right = QVBoxLayout()
        pg.setConfigOptions(antialias=False)
        self.plot_acc = pg.PlotWidget(title="Accelerometer [g]")
        self.plot_acc.addLegend(offset=(10, 10))
        self.plot_acc.showGrid(x=True, y=True, alpha=0.2)
        self.curves_acc = [
            self.plot_acc.plot(pen=pg.mkPen(c, width=1), name=n)
            for c, n in (("#e6194b", "ax"), ("#3cb44b", "ay"),
                         ("#4363d8", "az"))]
        self.plot_gyr = pg.PlotWidget(title="Gyroscope [dps]")
        self.plot_gyr.addLegend(offset=(10, 10))
        self.plot_gyr.showGrid(x=True, y=True, alpha=0.2)
        self.curves_gyr = [
            self.plot_gyr.plot(pen=pg.mkPen(c, width=1), name=n)
            for c, n in (("#e6194b", "gx"), ("#3cb44b", "gy"),
                         ("#4363d8", "gz"))]
        # Live-view rendering: peak-mode downsampling keeps the
        # vibration envelope while capping drawn points; the x-axis is
        # driven by _update_plots (smoothed edge), gyro linked to accel.
        for p in (self.plot_acc, self.plot_gyr):
            p.setDownsampling(auto=True, mode="peak")
            p.setClipToView(True)
            p.setMouseEnabled(x=False, y=False)
        self.plot_gyr.setXLink(self.plot_acc)
        right.addWidget(self.plot_acc, 2)
        right.addWidget(self.plot_gyr, 2)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(2000)
        self.console.setFont(QFont("Consolas", 9))
        right.addWidget(self.console, 1)

        root.addLayout(right, 1)
        self.setCentralWidget(central)

        # Everything that talks to the device is disabled until a
        # connection exists.  Deliberately NOT in this list: the dump
        # viewer buttons (btn_plot / btn_open work offline on saved
        # files) and the console.
        self._conn_widgets = [
            self.chk_led, self.btn_tput, self.btn_devlog,
            self.spin_freq, self.btn_freq, self.spin_volt, self.btn_volt,
            self.chk_rail, self.chk_drv,
            self.cmb_odr, self.cmb_preview, self.cmb_content,
            self.cmb_afs, self.cmb_gfs, self.btn_apply,
            self.btn_log, self.btn_erase,
            self.cmb_sessions, self.btn_dump,
            self.spin_robot_id, self.btn_robot_id,
        ]
        self._set_connected_ui(False)

    def _set_connected_ui(self, connected: bool):
        for w in self._conn_widgets:
            w.setEnabled(connected)

    # ---- helpers --------------------------------------------------------

    def log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self.console.appendPlainText(f"[{ts}] {text}")

    @staticmethod
    def _robot_label(e: dict) -> str:
        rid = "unassigned" if not e["robot_id"] else f"#{e['robot_id']}"
        extra = f", fw {'.'.join(map(str, e['fw']))}" if e["fw"] else ""
        sess = ", LOGGING" if e["session_active"] else ""
        return f"{e['name']}  ({rid}{extra}{sess})  {e['rssi']} dBm"

    def on_disconnected(self):
        self.lbl_conn.setText("disconnected")
        self.btn_connect.setText("Connect")
        self.cmb_devices.setEnabled(True)
        self.btn_scan.setEnabled(True)
        self._status_timer.stop()
        self._plot_timer.stop()
        self.btn_log.setChecked(False)
        self.btn_log.setText("Start session (log + stream)")
        self._set_connected_ui(False)
        self.log("Disconnected.")

    # ---- async actions --------------------------------------------------

    async def _do_scan(self):
        """Refresh the device dropdown with all visible Caterpillars.
        Keeps the previously selected robot selected if still around."""
        prev = self.cmb_devices.currentData()
        self.btn_scan.setEnabled(False)
        self.btn_scan.setText("Scanning…")
        try:
            self.log(f'Scanning for Caterpillars ("{P.DEVICE_PREFIX}*") ...')
            robots = await self.ble.scan_fleet()
        finally:
            self.btn_scan.setText("Scan")
            self.btn_scan.setEnabled(not self.ble.connected)
        self.cmb_devices.clear()
        for e in robots:
            self.cmb_devices.addItem(self._robot_label(e), e)
        if prev is not None:
            idx = next((i for i in range(self.cmb_devices.count())
                        if self.cmb_devices.itemData(i)["address"]
                        == prev["address"]), -1)
            if idx >= 0:
                self.cmb_devices.setCurrentIndex(idx)
        if self.cmb_devices.currentIndex() < 0 and self.cmb_devices.count():
            self.cmb_devices.setCurrentIndex(0)   # placeholder -> first hit
        self.log(f"{len(robots)} Caterpillar(s) visible."
                 if robots else "No Caterpillar found.")
        return robots

    async def _do_connect(self):
        if self.ble.connected:
            await self.ble.disconnect()
            return
        self.btn_connect.setEnabled(False)
        try:
            if self.cmb_devices.currentData() is None:
                await self._do_scan()      # first click: scan for them
            target = self.cmb_devices.currentData()
            if target is None:
                return
            if await self.ble.connect(target):
                self.lbl_conn.setText(
                    f"connected: {self.ble.last_name or '?'}")
                self.btn_connect.setText("Disconnect")
                self._set_connected_ui(True)
                self.cmb_devices.setEnabled(False)
                self.btn_scan.setEnabled(False)
                try:
                    self.spin_robot_id.setValue(
                        await self.ble.read_robot_id())
                except Exception:
                    pass  # legacy firmware without 0xFFF1
                self._status_timer.start()
                await self._poll_status()
                await self._refresh_sessions()
        except Exception as e:
            self.log(f"Connect failed: {e}")
        finally:
            self.btn_connect.setEnabled(True)

    async def _poll_status(self):
        if not self.ble.connected:
            return
        try:
            st = await self.ble.read_status()
        except Exception as e:
            self.log(f"Status read failed: {e}")
            return
        flpr = (f"v{st.flpr_fw[0]}.{st.flpr_fw[1]}.{st.flpr_fw[2]}"
                if st.flpr_fw else "not running")
        self.lbl_fw.setText(
            f"app v{st.fw[0]}.{st.fw[1]}.{st.fw[2]}  ·  FLPR {flpr}  ·  "
            f"IMU {'ok' if st.imu_ok else 'ABSENT'}  ·  up {st.uptime_s}s")
        self.lbl_vdc.setText(
            f"VDC: target {st.vdc_target_mv} mV, "
            f"measured {st.vdc_meas_mv} mV  ·  {st.freq_hz} Hz "
            f"duty {st.duty1}%")
        self.chk_rail.setChecked(st.rail_on)
        self.chk_drv.setChecked(st.drv_awake)
        self.chk_led.setChecked(st.led_on)
        self.bar_log.setMaximum(max(st.log_capacity, 1))
        self.bar_log.setValue(min(st.log_bytes, st.log_capacity))
        if st.log_active != self.btn_log.isChecked():
            self.btn_log.setChecked(st.log_active)
            self.btn_log.setText("Stop session" if st.log_active
                                 else "Start session (log + stream)")

        # Keep the stored-session list truthful while logging runs:
        # circular overwrite can remove old sessions at any moment.
        try:
            sessions = await self.ble.read_sessions()
        except Exception:
            return
        if tuple(s.seq for s in sessions) != self._session_seqs:
            self._populate_sessions(sessions)

    async def _run_tput(self):
        if not self.ble.connected:
            return
        if self.btn_log.isChecked():
            self.log("Stop the session first — the live stream would "
                     "skew the throughput measurement.")
            return
        self.btn_tput.setEnabled(False)
        self.log("Throughput test running (64 KiB burst)...")
        try:
            await self.ble.tput_test()
        except Exception as e:
            self.log(f"Throughput test aborted — link dropped mid-burst "
                     f"({type(e).__name__}). That itself indicates a weak "
                     f"connection; reconnect and retry.")
        finally:
            self.btn_tput.setEnabled(True)

    async def _read_devlog(self):
        if not self.ble.connected:
            return
        try:
            data = await self.ble.client.read_gatt_char(P.UUID_T2LOG)
        except Exception as e:
            self.log(f"Device log read failed: {e}")
            return
        text = bytes(data).decode(errors="replace").rstrip()
        self.log("---- device log (recent history) ----")
        for line in text.splitlines():
            self.log(f"  {line}")
        self.log("---- end device log ----")

    async def _apply_imu_cfg(self):
        if not self.ble.connected:
            return
        self._afs = self.cmb_afs.currentData()
        self._gfs = self.cmb_gfs.currentData()
        self._odr_hz = float(P.ODR_HZ[self.cmb_odr.currentData()])
        await self.ble.set_imu_cfg(self.cmb_odr.currentData(),
                                   self.cmb_content.currentData(),
                                   self._afs, self._gfs,
                                   self.cmb_preview.currentData())

    async def _start_session(self):
        """One click = flash logging + live stream together."""
        await self.ble.log_cmd(P.LOG_CMD_START, P.LOG_POLICY_CIRCULAR)
        self._sbuf.clear()
        self._stream_n = 0
        self._stream_t0 = time.monotonic()
        self._stream_dropped = 0
        self._rate_win.clear()
        self._disp_edge = None
        self._frame_wall = None
        self._gap_est = 0.05
        self._last_pkt_wall = None
        self._seq_last = None
        self._seq_abs = 0
        await self.ble.stream_start()
        self._plot_timer.start()
        self.btn_log.setText("Stop session")
        self.log("Session started")

    async def _stop_session(self):
        self._plot_timer.stop()
        await self.ble.stream_stop()
        await self.ble.log_cmd(P.LOG_CMD_STOP)
        self.btn_log.setText("Start session (log + stream)")
        self.log("Session stopped")
        # stop executes async on-device (flash writer drain, up to ~1 s)
        await asyncio.sleep(1.0)
        await self._refresh_sessions()

    async def _refresh_sessions(self):
        """Sync the stored-session list from the device directory."""
        try:
            sessions = await self.ble.read_sessions()
        except Exception as e:
            self.log(f"Session list read failed: {e}")
            return
        self._populate_sessions(sessions)
        self.log(f"{len(sessions)} stored session(s) on device")

    def _populate_sessions(self, sessions: list):
        selected: P.Session | None = self.cmb_sessions.currentData()
        self._session_seqs = tuple(s.seq for s in sessions)
        self.cmb_sessions.clear()
        for s in sessions:
            when = (time.strftime("%Y-%m-%d %H:%M:%S",
                                  time.localtime(s.wall_start))
                    if s.wall_start else "no clock")
            hz = P.ODR_HZ.get(s.odr_code, "?")
            self.cmb_sessions.addItem(
                f"#{s.seq} · {when} · {s.rec_count:,} rec @ {hz} Hz", s)
        if not sessions:
            self.cmb_sessions.addItem("(no stored sessions)", None)
        elif selected is not None:
            for i in range(self.cmb_sessions.count()):
                d = self.cmb_sessions.itemData(i)
                if d is not None and d.seq == selected.seq:
                    self.cmb_sessions.setCurrentIndex(i)
                    break

    async def _toggle_session(self):
        if not self.ble.connected:
            self.btn_log.setChecked(False)
            return
        try:
            if self.btn_log.isChecked():
                await self._start_session()
            else:
                await self._stop_session()
        except Exception as e:
            self.log(f"Session command failed: {e}")

    def on_stream_pkt(self, _char, data: bytearray):
        # Hot path (runs on the qasync loop between BLE I/O): parse in
        # place — no bytes() copy — and do only O(packet) work here.
        n = data[0]
        if n == 0 or len(data) < 8 + n * P.RECORD_SIZE:
            return
        recs = np.frombuffer(data, dtype=np.int16, count=n * 8,
                             offset=8).reshape(n, 8)

        acc = recs[:, 0:3].astype(np.float32) * \
            (P.ACCEL_MG_PER_LSB[self._afs] / 1000.0)
        gyr = recs[:, 3:6].astype(np.float32) * \
            (P.GYRO_MDPS_PER_LSB[self._gfs] / 1000.0)

        # Sample times come from the FLPR-stamped sequence numbers, not
        # packet arrival: BLE delivers notifications in bursts per
        # connection event, so arrival-based timestamps bunch samples
        # into glitchy clusters.  seq16 counts every sensor sample
        # (including decimated/dropped ones), so seq/ODR is the true
        # sensor-clock time axis.
        seq = recs[:, 7].astype(np.int64) & 0xFFFF
        if self._seq_last is None:
            self._seq_last = int(seq[0])
        deltas = np.diff(seq, prepend=self._seq_last) % 65536
        abs_seq = self._seq_abs + np.cumsum(deltas)
        self._seq_last = int(seq[-1])
        self._seq_abs = int(abs_seq[-1])

        self._sbuf.append(abs_seq / self._odr_hz, acc, gyr)
        self._stream_n += n
        now = time.monotonic()
        if self._last_pkt_wall is not None:
            self._gap_est = max(now - self._last_pkt_wall,
                                self._gap_est * 0.98)
        self._last_pkt_wall = now
        self._rate_win.append((now, n))
        self._decim = data[1] or 1
        (self._stream_dropped,) = struct.unpack_from("<I", data, 4)

    def _update_plots(self):
        buf = self._sbuf
        if buf.len == 0:
            return
        t_all = buf.t[:buf.len]
        t_last = t_all[-1]

        # Smooth scrolling: notifications land in bursts per connection
        # event (~30-50 ms, wider while logging), so drawing straight
        # to the newest sample makes the trace lurch.  Instead the
        # right edge advances at wall-clock pace and is gently pulled
        # toward (newest - lag).  The lag adapts to the observed
        # inter-packet cadence, so a widened connection interval just
        # means slightly more display latency, not stalling.
        now = time.monotonic()
        dt = (now - self._frame_wall) if self._frame_wall else 0.033
        self._frame_wall = now
        # Count an in-progress silence toward the gap estimate too, so
        # the lag grows *during* a lull instead of only after it ends.
        if self._last_pkt_wall is not None:
            self._gap_est = max(self._gap_est, now - self._last_pkt_wall)
        lag = min(max(1.5 * self._gap_est, SMOOTH_LAG_S), 0.6)
        edge = self._disp_edge
        if edge is None or abs(t_last - edge) > 2.0:
            edge = t_last                      # (re)acquire after stall
        else:
            edge += dt + 2.0 * dt * ((t_last - lag) - edge)
            edge = min(edge, t_last)           # never show the future
        self._disp_edge = edge

        # t is monotonic — slice with searchsorted views, no mask copy
        hi = int(np.searchsorted(t_all, edge, "right"))
        lo = int(np.searchsorted(t_all, edge - PLOT_WINDOW_S, "left"))
        t = t_all[lo:hi]

        # Where samples were really lost (seq16 gaps), break the line
        # instead of drawing a misleading bridge across the hole.
        gaps = np.flatnonzero(np.diff(t) > 2.5 * self._decim
                              / self._odr_hz) + 1 if len(t) > 1 else \
            np.empty(0, np.int64)
        if gaps.size:
            t_p = np.insert(t, gaps, np.nan)
            for i, c in enumerate(self.curves_acc):
                c.setData(t_p, np.insert(buf.acc[lo:hi, i], gaps, np.nan),
                          connect="finite")
            for i, c in enumerate(self.curves_gyr):
                c.setData(t_p, np.insert(buf.gyr[lo:hi, i], gaps, np.nan),
                          connect="finite")
        else:
            for i, c in enumerate(self.curves_acc):
                c.setData(t, buf.acc[lo:hi, i], skipFiniteCheck=True)
            for i, c in enumerate(self.curves_gyr):
                c.setData(t, buf.gyr[lo:hi, i], skipFiniteCheck=True)
        self.plot_acc.setXRange(edge - PLOT_WINDOW_S, edge, padding=0)

        # Windowed (2 s) rate: shows what the link does *now*, not the
        # lifetime average.
        self._rate_win = [(w, k) for w, k in self._rate_win if now - w < 2.0]
        rate = sum(k for _, k in self._rate_win) / 2.0
        self.lbl_stream.setText(
            f"stream: {rate:.0f} samples/s, "
            f"{self._stream_dropped} dropped")

    async def _erase_log(self):
        if not self.ble.connected:
            return
        if self.btn_log.isChecked():
            await self._stop_session()
            self.btn_log.setChecked(False)
        await self.ble.log_cmd(P.LOG_CMD_ERASE)
        self.log("All stored sessions erased")
        await asyncio.sleep(0.5)      # erase executes async on-device
        await self._refresh_sessions()

    async def _dump_log(self):
        if not self.ble.connected:
            return
        if self.btn_log.isChecked():
            await self._stop_session()
            self.btn_log.setChecked(False)

        sess: P.Session | None = self.cmb_sessions.currentData()
        if sess is None or sess.rec_count == 0:
            self.log("No session selected / session is empty.")
            return

        stamp = (time.strftime("%Y%m%d_%H%M%S",
                               time.localtime(sess.wall_start))
                 if sess.wall_start else f"session{sess.seq}")
        path, _ = await self._pick_file(
            True, f"Save session #{sess.seq}", f"imu_{stamp}",
            "CSV (*.csv);;NumPy (*.npz)")
        if not path:
            return

        total = sess.bytes
        self.bar_dump.setVisible(True)
        self.bar_dump.setMaximum(total)
        self.bar_dump.setValue(0)
        self.btn_dump.setEnabled(False)
        try:
            raw = await self.ble.dump(sess.seq, total)
        finally:
            self.btn_dump.setEnabled(True)
            self.bar_dump.setVisible(False)
        if raw is None:
            return
        self._save_dump(Path(path), raw, sess)

    def on_dump_progress(self, received: int, total: int):
        self.bar_dump.setValue(min(received, total))

    def _save_dump(self, path: Path, raw: bytes, sess: P.Session):
        recs = np.array(list(P.unpack_records(raw)), dtype=np.int64)
        if recs.size == 0:
            self.log("No records in dump.")
            return

        odr = P.ODR_HZ.get(sess.odr_code, 0)
        a_scale = P.ACCEL_MG_PER_LSB[sess.accel_fs] / 1000.0
        g_scale = P.GYRO_MDPS_PER_LSB[sess.gyro_fs] / 1000.0
        n = recs.shape[0]
        t = np.arange(n) / odr if odr else np.arange(n, dtype=float)
        acc = recs[:, 0:3] * a_scale
        gyr = recs[:, 3:6] * g_scale
        temp = recs[:, 6] / 256.0 + 25.0
        seq = recs[:, 7]
        when = (time.strftime("%Y-%m-%d %H:%M:%S",
                              time.localtime(sess.wall_start))
                if sess.wall_start else "unsynced clock")

        base = path.with_suffix("")
        csv_path = base.with_suffix(".csv")
        npz_path = base.with_suffix(".npz")

        with open(csv_path, "w", newline="") as f:
            f.write(f"# Caterpillar IMU session {sess.seq}, started {when}, "
                    f"{odr} Hz, accel ±{P.ACCEL_FS_G[sess.accel_fs]} g, "
                    f"gyro ±{P.GYRO_FS_DPS[sess.gyro_fs]} dps\n")
            f.write("t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,seq16\n")
            for i in range(n):
                f.write(f"{t[i]:.6f},{acc[i,0]:.5f},{acc[i,1]:.5f},"
                        f"{acc[i,2]:.5f},{gyr[i,0]:.4f},{gyr[i,1]:.4f},"
                        f"{gyr[i,2]:.4f},{temp[i]:.2f},{seq[i]}\n")

        np.savez_compressed(
            npz_path, t_s=t, accel_g=acc, gyro_dps=gyr, temp_c=temp,
            seq16=seq, odr_hz=odr,
            accel_fs_g=P.ACCEL_FS_G[sess.accel_fs],
            gyro_fs_dps=P.GYRO_FS_DPS[sess.gyro_fs],
            session_id=sess.seq, wall_start_epoch=sess.wall_start)

        self.log(f"Saved {n} samples -> {csv_path.name} + {npz_path.name}")

        title = (f"session {sess.seq} · {when} · {odr} Hz · {n} samples · "
                 f"±{P.ACCEL_FS_G[sess.accel_fs]} g / "
                 f"±{P.GYRO_FS_DPS[sess.gyro_fs]} dps · {csv_path.name}")
        self._dump_data = (t, acc, gyr, title)
        self.btn_plot.setEnabled(True)
        self._plot_dump()

    def _plot_dump(self):
        if self._dump_data is None:
            return
        if self._dump_win is None:
            self._dump_win = DumpPlotWindow()
        self._dump_win.set_data(*self._dump_data)
        self._dump_win.show()
        self._dump_win.raise_()

    def _open_npz(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open IMU log", "", "NumPy (*.npz)")
        if not path:
            return
        try:
            d = np.load(path)
            t, acc, gyr = d["t_s"], d["accel_g"], d["gyro_dps"]
            odr = float(d["odr_hz"]) if "odr_hz" in d else 0
            sess = int(d["session_id"]) if "session_id" in d else 0
        except Exception as e:
            self.log(f"Could not open {Path(path).name}: {e}")
            return
        title = (f"session {sess} · {odr} Hz · {len(t)} samples · "
                 f"{Path(path).name}")
        self._dump_data = (t, acc, gyr, title)
        self.btn_plot.setEnabled(True)
        self._plot_dump()


def main():
    app = QApplication(sys.argv)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    win = MainWindow()
    win.show()
    with loop:
        loop.run_forever()


if __name__ == "__main__":
    main()
