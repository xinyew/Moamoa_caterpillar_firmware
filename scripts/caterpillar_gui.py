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
        self._dump_buf = bytearray()
        self._dump_expected = 0
        self._dump_done: asyncio.Event | None = None

    @property
    def connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    async def connect(self):
        self.ui.log(f'Scanning for "{P.DEVICE_NAME}" ...')
        dev = await BleakScanner.find_device_by_name(P.DEVICE_NAME,
                                                     timeout=10.0)
        if dev is None:
            self.ui.log("Device not found.")
            return False

        # Windows caches GATT tables across firmware updates; don't.
        self.client = BleakClient(
            dev, disconnected_callback=self._on_disconnect,
            winrt=dict(use_cached_services=False))
        await self.client.connect()
        self.ui.log(f"Connected ({dev.address}, MTU={self.client.mtu_size})")

        await self.client.start_notify(P.UUID_MSG, self._on_msg)
        await self.client.start_notify(P.UUID_DUMP, self._on_dump_chunk)
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

    # ---- IMU ------------------------------------------------------------

    async def set_imu_cfg(self, odr: int, content: int, afs: int, gfs: int):
        await self.client.write_gatt_char(
            P.UUID_IMU_CFG, P.encode_imu_cfg(odr, content, afs, gfs),
            response=True)
        self.ui.log(f"IMU config -> {P.ODR_HZ[odr]} Hz, "
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

    async def dump(self, total: int) -> bytes | None:
        """Fetch `total` logical bytes of the log, chunk-reassembled."""
        self._dump_buf = bytearray(total)
        self._dump_expected = total
        self._dump_received = 0
        self._dump_done = asyncio.Event()
        t0 = time.monotonic()

        await self.client.write_gatt_char(P.UUID_DUMP,
                                          P.encode_dump_req(0, total),
                                          response=True)
        try:
            await asyncio.wait_for(self._dump_done.wait(),
                                   timeout=60 + total / 4096)
        except asyncio.TimeoutError:
            self.ui.log(f"Dump timed out at "
                        f"{self._dump_received}/{total} B")
            return None

        dt = time.monotonic() - t0
        self.ui.log(f"Dump complete: {total} B in {dt:.1f} s "
                    f"({total / dt / 1024:.1f} KiB/s)")
        return bytes(self._dump_buf)

    def _on_dump_chunk(self, _char, data: bytearray):
        if self._dump_done is None or self._dump_done.is_set():
            return
        c = P.decode_dump_chunk(bytes(data))
        end = min(c.offset + len(c.data), self._dump_expected)
        self._dump_buf[c.offset:end] = c.data[:end - c.offset]
        self._dump_received += len(c.data)
        self.ui.on_dump_progress(self._dump_received, self._dump_expected)
        if c.last or self._dump_received >= self._dump_expected:
            self._dump_done.set()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Caterpillar Control")
        self.resize(1100, 760)
        self.ble = BleWorker(self)

        # Stream plot state
        self._t = np.zeros(0)
        self._acc = np.zeros((0, 3))
        self._gyr = np.zeros((0, 3))
        self._stream_n = 0
        self._stream_t0 = None
        self._stream_rate = 0.0
        self._stream_dropped = 0
        self._afs = 0
        self._gfs = 0
        self._dump_data = None       # (t, acc_g, gyr_dps, title)
        self._dump_win: DumpPlotWindow | None = None

        self._build_ui()

        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(33)
        self._plot_timer.timeout.connect(self._update_plots)

        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(
            lambda: asyncio.ensure_future(self._poll_status()))

    # ---- UI construction ------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        root = QHBoxLayout(central)

        # Left column: connection, motor, IMU cfg, log
        left = QVBoxLayout()
        left.setSpacing(10)

        conn_box = QGroupBox("Connection")
        cl = QGridLayout(conn_box)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(
            lambda: asyncio.ensure_future(self._do_connect()))
        self.lbl_conn = QLabel("disconnected")
        self.lbl_fw = QLabel("—")
        cl.addWidget(self.btn_connect, 0, 0)
        cl.addWidget(self.lbl_conn, 0, 1)
        cl.addWidget(QLabel("Firmware:"), 1, 0)
        cl.addWidget(self.lbl_fw, 1, 1)
        left.addWidget(conn_box)

        motor_box = QGroupBox("Motor")
        ml = QGridLayout(motor_box)
        self.spin_freq = QSpinBox()
        self.spin_freq.setRange(P.FREQ_MIN_HZ, P.FREQ_MAX_HZ)
        self.spin_freq.setValue(150)
        self.spin_freq.setSuffix(" Hz")
        btn_freq = QPushButton("Set")
        btn_freq.clicked.connect(lambda: asyncio.ensure_future(
            self.ble.set_freq(self.spin_freq.value())))
        self.spin_volt = QDoubleSpinBox()
        self.spin_volt.setRange(P.VOLT_MIN_MV / 1000, P.VOLT_MAX_MV / 1000)
        self.spin_volt.setValue(3.0)
        self.spin_volt.setSingleStep(0.1)
        self.spin_volt.setDecimals(2)
        self.spin_volt.setSuffix(" V")
        btn_volt = QPushButton("Set")
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
        self.cmb_odr.setCurrentIndex(3)  # 104 Hz boot default
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
        btn_apply = QPushButton("Apply config")
        btn_apply.clicked.connect(
            lambda: asyncio.ensure_future(self._apply_imu_cfg()))
        self.btn_stream = QPushButton("Start stream")
        self.btn_stream.setCheckable(True)
        self.btn_stream.clicked.connect(
            lambda: asyncio.ensure_future(self._toggle_stream()))
        self.lbl_stream = QLabel("stream: off")
        il.addWidget(QLabel("ODR"), 0, 0)
        il.addWidget(self.cmb_odr, 0, 1)
        il.addWidget(QLabel("Content"), 1, 0)
        il.addWidget(self.cmb_content, 1, 1)
        il.addWidget(QLabel("Accel FS"), 2, 0)
        il.addWidget(self.cmb_afs, 2, 1)
        il.addWidget(QLabel("Gyro FS"), 3, 0)
        il.addWidget(self.cmb_gfs, 3, 1)
        il.addWidget(btn_apply, 4, 0)
        il.addWidget(self.btn_stream, 4, 1)
        il.addWidget(self.lbl_stream, 5, 0, 1, 2)
        left.addWidget(imu_box)

        log_box = QGroupBox("On-chip log")
        ll = QGridLayout(log_box)
        self.cmb_policy = QComboBox()
        self.cmb_policy.addItem("stop when full", P.LOG_POLICY_STOP)
        self.cmb_policy.addItem("circular (keep newest)",
                                P.LOG_POLICY_CIRCULAR)
        self.btn_log = QPushButton("Start logging")
        self.btn_log.setCheckable(True)
        self.btn_log.clicked.connect(
            lambda: asyncio.ensure_future(self._toggle_log()))
        btn_erase = QPushButton("Erase")
        btn_erase.clicked.connect(lambda: asyncio.ensure_future(
            self._erase_log()))
        self.bar_log = QProgressBar()
        self.bar_log.setFormat("%v / %m B")
        self.btn_dump = QPushButton("Dump to file…")
        self.btn_dump.clicked.connect(
            lambda: asyncio.ensure_future(self._dump_log()))
        self.bar_dump = QProgressBar()
        self.bar_dump.setVisible(False)
        self.btn_plot = QPushButton("Plot last dump")
        self.btn_plot.setEnabled(False)
        self.btn_plot.clicked.connect(self._plot_dump)
        btn_open = QPushButton("Open .npz…")
        btn_open.clicked.connect(self._open_npz)
        ll.addWidget(QLabel("Fill policy"), 0, 0)
        ll.addWidget(self.cmb_policy, 0, 1)
        ll.addWidget(self.btn_log, 1, 0)
        ll.addWidget(btn_erase, 1, 1)
        ll.addWidget(self.bar_log, 2, 0, 1, 2)
        ll.addWidget(self.btn_dump, 3, 0, 1, 2)
        ll.addWidget(self.bar_dump, 4, 0, 1, 2)
        ll.addWidget(self.btn_plot, 5, 0)
        ll.addWidget(btn_open, 5, 1)
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
        right.addWidget(self.plot_acc, 2)
        right.addWidget(self.plot_gyr, 2)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(2000)
        self.console.setFont(QFont("Consolas", 9))
        right.addWidget(self.console, 1)

        root.addLayout(right, 1)
        self.setCentralWidget(central)

    # ---- helpers --------------------------------------------------------

    def log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self.console.appendPlainText(f"[{ts}] {text}")

    def on_disconnected(self):
        self.lbl_conn.setText("disconnected")
        self.btn_connect.setText("Connect")
        self._status_timer.stop()
        self._plot_timer.stop()
        self.btn_stream.setChecked(False)
        self.log("Disconnected.")

    # ---- async actions --------------------------------------------------

    async def _do_connect(self):
        if self.ble.connected:
            await self.ble.disconnect()
            return
        self.btn_connect.setEnabled(False)
        try:
            if await self.ble.connect():
                self.lbl_conn.setText("connected")
                self.btn_connect.setText("Disconnect")
                self._status_timer.start()
                await self._poll_status()
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
        self.bar_log.setMaximum(max(st.log_capacity, 1))
        self.bar_log.setValue(min(st.log_bytes, st.log_capacity))
        if st.log_active != self.btn_log.isChecked():
            self.btn_log.setChecked(st.log_active)
            self.btn_log.setText("Stop logging" if st.log_active
                                 else "Start logging")

    async def _apply_imu_cfg(self):
        if not self.ble.connected:
            return
        self._afs = self.cmb_afs.currentData()
        self._gfs = self.cmb_gfs.currentData()
        await self.ble.set_imu_cfg(self.cmb_odr.currentData(),
                                   self.cmb_content.currentData(),
                                   self._afs, self._gfs)

    async def _toggle_stream(self):
        if not self.ble.connected:
            self.btn_stream.setChecked(False)
            return
        if self.btn_stream.isChecked():
            self._t = np.zeros(0)
            self._acc = np.zeros((0, 3))
            self._gyr = np.zeros((0, 3))
            self._stream_n = 0
            self._stream_t0 = time.monotonic()
            self._stream_dropped = 0
            await self.ble.stream_start()
            self.btn_stream.setText("Stop stream")
            self._plot_timer.start()
        else:
            await self.ble.stream_stop()
            self.btn_stream.setText("Start stream")
            self._plot_timer.stop()

    def on_stream_pkt(self, _char, data: bytearray):
        try:
            pkt = P.decode_stream_packet(bytes(data))
        except Exception:
            return
        recs = np.frombuffer(pkt.samples, dtype=np.int16)
        n = len(recs) // 8
        if n == 0:
            return
        recs = recs.reshape(n, 8)

        now = time.monotonic() - self._stream_t0
        acc = recs[:, 0:3].astype(np.float32) * \
            (P.ACCEL_MG_PER_LSB[self._afs] / 1000.0)
        gyr = recs[:, 3:6].astype(np.float32) * \
            (P.GYRO_MDPS_PER_LSB[self._gfs] / 1000.0)
        t = np.linspace(now - 0.01, now, n)

        self._t = np.concatenate([self._t, t])[-PLOT_MAX_POINTS:]
        self._acc = np.concatenate([self._acc, acc])[-PLOT_MAX_POINTS:]
        self._gyr = np.concatenate([self._gyr, gyr])[-PLOT_MAX_POINTS:]
        self._stream_n += n
        self._stream_dropped = pkt.dropped

    def _update_plots(self):
        if len(self._t) == 0:
            return
        tmax = self._t[-1]
        mask = self._t >= tmax - PLOT_WINDOW_S
        t = self._t[mask]
        for i, c in enumerate(self.curves_acc):
            c.setData(t, self._acc[mask, i])
        for i, c in enumerate(self.curves_gyr):
            c.setData(t, self._gyr[mask, i])
        el = time.monotonic() - self._stream_t0
        rate = self._stream_n / el if el > 0 else 0
        self.lbl_stream.setText(
            f"stream: {rate:.0f} samples/s shown, "
            f"{self._stream_dropped} dropped")

    async def _toggle_log(self):
        if not self.ble.connected:
            self.btn_log.setChecked(False)
            return
        try:
            if self.btn_log.isChecked():
                await self.ble.log_cmd(P.LOG_CMD_START,
                                       self.cmb_policy.currentData())
                self.btn_log.setText("Stop logging")
                self.log("Logging started "
                         f"({self.cmb_policy.currentText()})")
            else:
                await self.ble.log_cmd(P.LOG_CMD_STOP)
                self.btn_log.setText("Start logging")
                self.log("Logging stopped")
        except Exception as e:
            self.log(f"Log command failed: {e}")

    async def _erase_log(self):
        if not self.ble.connected:
            return
        await self.ble.log_cmd(P.LOG_CMD_ERASE)
        self.btn_log.setChecked(False)
        self.btn_log.setText("Start logging")
        self.log("Log erased")

    async def _dump_log(self):
        if not self.ble.connected:
            return
        state = await self.ble.log_state()
        if state.bytes_stored == 0:
            self.log("Log is empty — nothing to dump.")
            return
        if state.active:
            await self.ble.log_cmd(P.LOG_CMD_STOP)
            self.btn_log.setChecked(False)
            self.btn_log.setText("Start logging")
            self.log("Logging stopped for dump")

        path, _ = QFileDialog.getSaveFileName(
            self, "Save IMU log", f"imu_log_{time.strftime('%Y%m%d_%H%M%S')}",
            "CSV (*.csv);;NumPy (*.npz)")
        if not path:
            return

        total = P.LOG_HDR_SIZE + state.bytes_stored
        self.bar_dump.setVisible(True)
        self.bar_dump.setMaximum(total)
        self.bar_dump.setValue(0)
        self.btn_dump.setEnabled(False)
        try:
            raw = await self.ble.dump(total)
        finally:
            self.btn_dump.setEnabled(True)
            self.bar_dump.setVisible(False)
        if raw is None:
            return
        self._save_dump(Path(path), raw)

    def on_dump_progress(self, received: int, total: int):
        self.bar_dump.setValue(min(received, total))

    def _save_dump(self, path: Path, raw: bytes):
        try:
            hdr = P.decode_log_header(raw[:P.LOG_HDR_SIZE])
        except ValueError as e:
            self.log(f"Dump parse: {e}")
            return

        recs = np.array(list(P.unpack_records(raw[P.LOG_HDR_SIZE:])),
                        dtype=np.int64)
        if recs.size == 0:
            self.log("No records in dump.")
            return

        odr = P.ODR_HZ.get(hdr.odr_code, 0)
        a_scale = P.ACCEL_MG_PER_LSB[hdr.accel_fs] / 1000.0
        g_scale = P.GYRO_MDPS_PER_LSB[hdr.gyro_fs] / 1000.0
        n = recs.shape[0]
        t = np.arange(n) / odr if odr else np.arange(n, dtype=float)
        acc = recs[:, 0:3] * a_scale
        gyr = recs[:, 3:6] * g_scale
        temp = recs[:, 6] / 256.0 + 25.0
        seq = recs[:, 7]

        base = path.with_suffix("")
        csv_path = base.with_suffix(".csv")
        npz_path = base.with_suffix(".npz")

        with open(csv_path, "w", newline="") as f:
            f.write(f"# Caterpillar IMU log session {hdr.session_id}, "
                    f"{odr} Hz, accel ±{P.ACCEL_FS_G[hdr.accel_fs]} g, "
                    f"gyro ±{P.GYRO_FS_DPS[hdr.gyro_fs]} dps\n")
            f.write("t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,seq16\n")
            for i in range(n):
                f.write(f"{t[i]:.6f},{acc[i,0]:.5f},{acc[i,1]:.5f},"
                        f"{acc[i,2]:.5f},{gyr[i,0]:.4f},{gyr[i,1]:.4f},"
                        f"{gyr[i,2]:.4f},{temp[i]:.2f},{seq[i]}\n")

        np.savez_compressed(
            npz_path, t_s=t, accel_g=acc, gyro_dps=gyr, temp_c=temp,
            seq16=seq, odr_hz=odr,
            accel_fs_g=P.ACCEL_FS_G[hdr.accel_fs],
            gyro_fs_dps=P.GYRO_FS_DPS[hdr.gyro_fs],
            session_id=hdr.session_id)

        self.log(f"Saved {n} samples -> {csv_path.name} + {npz_path.name}")

        title = (f"session {hdr.session_id} · {odr} Hz · {n} samples · "
                 f"±{P.ACCEL_FS_G[hdr.accel_fs]} g / "
                 f"±{P.GYRO_FS_DPS[hdr.gyro_fs]} dps · {csv_path.name}")
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
