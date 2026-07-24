#!/usr/bin/env python3
"""
Caterpillar BLE control — low-latency PWM frequency / motor voltage setter.

Connects to a "Caterpillar" device and writes 16-bit little-endian values
to custom GATT characteristics:
    0xFFE1 — PWM frequency in Hz
    0xFFE2 — motor rail (VDC) voltage in mV

One-shot mode uses acknowledged writes (confirmed by the device before
disconnecting); interactive mode uses Write Without Response for lowest
latency while the connection stays open.

Usage:
    python ble_control.py               # interactive mode
    python ble_control.py --freq 170    # one-shot frequency
    python ble_control.py -f 170        # short form
    python ble_control.py --V 3.5       # one-shot voltage (volts)
    python ble_control.py -f 170 -V 3.1 # both in one connection

Requires: pip install bleak
"""

import argparse
import asyncio
import struct
import sys

from bleak import BleakClient, BleakScanner

# Must match firmware — see ble_interface.c
SERVICE_UUID      = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR_UUID_FREQ    = "0000ffe1-0000-1000-8000-00805f9b34fb"
CHAR_UUID_VOLT    = "0000ffe2-0000-1000-8000-00805f9b34fb"
CHAR_UUID_VDCMEAS = "0000ffe3-0000-1000-8000-00805f9b34fb"
CHAR_UUID_RAIL    = "0000ffe4-0000-1000-8000-00805f9b34fb"
CHAR_UUID_DRV     = "0000ffe5-0000-1000-8000-00805f9b34fb"
CHAR_UUID_STATUS  = "0000ffe6-0000-1000-8000-00805f9b34fb"
CHAR_UUID_TPUT    = "0000ffe7-0000-1000-8000-00805f9b34fb"
CHAR_UUID_ROBOTID = "0000fff1-0000-1000-8000-00805f9b34fb"
DEVICE_NAME       = "Caterpillar"   # legacy name (fw < 1.5.0)
DEVICE_PREFIX     = "Cat-"          # fleet naming: Cat-NN / Cat-XXXX
MFG_COMPANY_ID    = 0xFFFF          # test company id in advertising data

# Must match firmware (driver_pwm.h): nRF54 PWM cannot go below ~4 Hz
FREQ_MIN = 4
FREQ_MAX = 1000

# Must match firmware (max5419.h): 0.75 V = digipot tap 0,
# 4.2 V = boost-runaway safety cap
VOLT_MIN = 0.75
VOLT_MAX = 4.2


def pack_u16(value: int) -> bytes:
    """Pack a 16-bit value into little-endian bytes."""
    return struct.pack("<H", value)


def _is_caterpillar(name: str | None) -> bool:
    return bool(name) and (name.startswith(DEVICE_PREFIX)
                           or name == DEVICE_NAME)


async def scan_fleet(timeout: float = 6.0) -> list[dict]:
    """Scan and return every visible Caterpillar with its identity.

    Each entry: {name, address, robot_id, fw, session_active, rssi}.
    robot_id/fw/session_active are None on legacy firmware (< 1.5.0,
    no manufacturer data in the advertisement).
    """
    found: dict[str, dict] = {}

    def on_adv(device, adv):
        if not _is_caterpillar(device.name or adv.local_name):
            return
        info = {"name": device.name or adv.local_name,
                "address": device.address,
                "robot_id": None, "fw": None, "session_active": None,
                "rssi": adv.rssi}
        mfg = adv.manufacturer_data.get(MFG_COMPANY_ID)
        if mfg and len(mfg) >= 5:
            info["robot_id"] = mfg[0]
            info["fw"] = f"{mfg[1]}.{mfg[2]}.{mfg[3]}"
            info["session_active"] = bool(mfg[4] & 0x01)
        found[device.address] = info

    scanner = BleakScanner(on_adv)
    await scanner.start()
    await asyncio.sleep(timeout)
    await scanner.stop()
    return sorted(found.values(),
                  key=lambda e: (e["robot_id"] is None,
                                 e["robot_id"] or 0, e["name"]))


def _fmt_robot(e: dict) -> str:
    rid = "unassigned" if not e["robot_id"] else f"#{e['robot_id']}"
    extra = f", fw {e['fw']}" if e["fw"] else ""
    sess = ", LOGGING" if e["session_active"] else ""
    return f"{e['name']} [{e['address']}] ({rid}{extra}{sess})"


async def discover(robot: int | None = None,
                   name: str | None = None) -> str | None:
    """Find one Caterpillar and return its BLE address.

    robot= selects by assigned fleet number, name= by exact adv name.
    With neither, a single visible Caterpillar is used; several visible
    is an error (the list is printed so you can pick with --robot).
    """
    what = (f"robot #{robot}" if robot is not None
            else name if name else "any Caterpillar (Cat-*)")
    print(f"Scanning for {what} ...", flush=True)

    robots = await scan_fleet()
    if robot is not None:
        robots = [e for e in robots if e["robot_id"] == robot
                  or e["name"] == f"Cat-{robot:02d}"]
    elif name is not None:
        robots = [e for e in robots if e["name"] == name]

    if not robots:
        print(f"No matching Caterpillar found ({what}).", file=sys.stderr)
        return None
    if len(robots) > 1:
        print("Multiple Caterpillars visible - pick one with "
              "--robot N or --name:", file=sys.stderr)
        for e in robots:
            print(f"  {_fmt_robot(e)}", file=sys.stderr)
        return None

    print(f"Found {_fmt_robot(robots[0])}")
    return robots[0]["address"]


async def set_robot_id(address: str, new_id: int):
    """Assign the fleet robot number (persisted; adv name updates on
    the next advertising start, i.e. after disconnect)."""
    async with BleakClient(address,
                           winrt=dict(use_cached_services=False)) as client:
        await client.write_gatt_char(CHAR_UUID_ROBOTID, bytes([new_id]),
                                     response=True)
        print(f"Robot ID -> {new_id} "
              f"(name becomes Cat-{new_id:02d} after disconnect)"
              if new_id else
              "Robot ID cleared (name reverts to Cat-XXXX after disconnect)")


async def send_freq(client: BleakClient, hz: int, ack: bool):
    """ack=True uses an ATT Write Request: the await returns only after
    the device confirms execution, so it survives an immediate
    disconnect and surfaces firmware rejections as exceptions.
    ack=False (Write Without Response) is fire-and-forget: lower
    latency, but writes queued right before a disconnect can be lost
    (macOS/CoreBluetooth buffers them aggressively).
    """
    data = pack_u16(hz)
    await client.write_gatt_char(CHAR_UUID_FREQ, data, response=ack)
    print(f"Sent: {hz} Hz -> {data.hex()}"
          f"{' (acked)' if ack else ''}", flush=True)


async def send_volt(client: BleakClient, volts: float, ack: bool):
    mv = round(volts * 1000)
    data = pack_u16(mv)
    await client.write_gatt_char(CHAR_UUID_VOLT, data, response=ack)
    print(f"Sent: {volts:.2f} V ({mv} mV) -> {data.hex()}"
          f"{' (acked)' if ack else ''}", flush=True)


async def send_rail(client: BleakClient, on: bool, ack: bool):
    await client.write_gatt_char(CHAR_UUID_RAIL, bytes([1 if on else 0]),
                                 response=ack)
    print(f"Motor rail -> {'ON' if on else 'OFF'}"
          f"{' (acked)' if ack else ''}", flush=True)


async def send_drv(client: BleakClient, awake: bool, ack: bool):
    await client.write_gatt_char(CHAR_UUID_DRV, bytes([1 if awake else 0]),
                                 response=ack)
    print(f"Motor driver -> {'AWAKE' if awake else 'SLEEP'}"
          f"{' (acked)' if ack else ''}", flush=True)


async def read_vdc(client: BleakClient):
    """Read the measured motor rail voltage (AIN4 sense divider)."""
    data = await client.read_gatt_char(CHAR_UUID_VDCMEAS)
    mv = struct.unpack("<H", data)[0]
    print(f"Measured VDC: {mv} mV", flush=True)
    return mv


async def tput_test(address: str, kib: int):
    """Measure raw GATT write throughput against the 0xFFE7 sink."""
    total = kib * 1024
    print(f"Connecting to {address} ...", flush=True)
    async with BleakClient(address,
                           winrt=dict(use_cached_services=False)) as client:
        mtu = client.mtu_size
        chunk = max(20, mtu - 3)
        print(f"Connected (MTU={mtu}), chunk {chunk} B, sending {kib} KiB ...")

        c0 = struct.unpack("<I",
                           await client.read_gatt_char(CHAR_UUID_TPUT))[0]
        payload = bytes(chunk)
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        sent = 0
        while sent < total:
            await client.write_gatt_char(CHAR_UUID_TPUT, payload,
                                         response=False)
            sent += len(payload)
        c1 = struct.unpack("<I",
                           await client.read_gatt_char(CHAR_UUID_TPUT))[0]
        dt = loop.time() - t0

        got = (c1 - c0) & 0xFFFFFFFF
        print(f"  sent {sent} B, device counted {got} B"
              f"{'  (LOSS!)' if got != sent else ''}")
        print(f"  elapsed {dt:.2f} s -> {sent / dt / 1024:.1f} KiB/s")


# Layout of the 0xFFE6 status packet — must match ble_interface.c
# (v1 = 24 B core; v2 appends u32 FLPR fw version at offset 24)
STATUS_FMT = "<4BH2B2H4B2I"
RESET_BITS = {0: "pin", 1: "soft", 2: "brownout", 3: "POR",
              4: "watchdog", 5: "debug"}


async def read_status(client: BleakClient):
    """Read and pretty-print the 0xFFE6 status packet."""
    try:
        data = await client.read_gatt_char(CHAR_UUID_STATUS)
    except Exception:
        # Firmware older than v1.0.4 — fall back to the VDC-only read
        await read_vdc(client)
        return

    (pkt_ver, fw_maj, fw_min, fw_pat, freq, duty1, duty2, tgt, meas,
     rail, drv, imu, _rsvd, uptime, cause) = struct.unpack(
        STATUS_FMT, data[:struct.calcsize(STATUS_FMT)])

    if pkt_ver not in (1, 2, 3):
        print(f"(unknown status packet version {pkt_ver}, raw: {data.hex()})")
        return

    flpr_ver = None
    if pkt_ver >= 2 and len(data) >= 28:
        (flpr_ver,) = struct.unpack_from("<I", data, 24)

    imu_v3 = None
    if pkt_ver >= 3 and len(data) >= 44:
        imu_v3 = struct.unpack_from("<4B3I", data, 28)  # odr, content,
        # log_active, log_policy, log_bytes, log_capacity, overruns

    causes = ", ".join(n for b, n in RESET_BITS.items() if cause & (1 << b))
    print(f"Device status:")
    print(f"  firmware   v{fw_maj}.{fw_min}.{fw_pat}")
    if flpr_ver is None:
        pass                       # v1 firmware: no FLPR version field
    elif flpr_ver == 0:
        print(f"  FLPR fw    not reported (FLPR not running?)")
    else:
        print(f"  FLPR fw    v{(flpr_ver >> 16) & 0xFF}"
              f".{(flpr_ver >> 8) & 0xFF}.{flpr_ver & 0xFF}")
    print(f"  PWM        {freq} Hz, duty IN1 {duty1}% / IN2 {duty2}%")
    print(f"  VDC        target {tgt} mV, measured {meas} mV")
    print(f"  STBB1 rail {'ON' if rail else 'OFF'}   DRV8212 "
          f"{'AWAKE' if drv else 'SLEEP'}   IMU {'ok' if imu else 'absent'}")
    print(f"  uptime     {uptime} s")
    print(f"  last reset 0x{cause:08x}" + (f" ({causes})" if causes else ""))
    if imu_v3 is not None:
        odr, content, log_on, log_policy, log_bytes, log_cap, overruns = imu_v3
        odr_hz = {1: 12.5, 2: 26, 3: 52, 4: 104, 5: 208, 6: 416,
                  7: 833, 8: 1660, 9: 3330, 10: 6660}.get(odr, "?")
        what = {1: "accel", 2: "gyro", 3: "accel+gyro"}.get(content, "?")
        pol = "circular" if log_policy else "stop-when-full"
        print(f"  IMU cfg    {odr_hz} Hz, {what}, overruns {overruns}")
        print(f"  IMU log    {'RECORDING' if log_on else 'stopped'} ({pol}), "
              f"{log_bytes}/{log_cap} B")


async def one_shot(address: str, hz: int | None, volts: float | None,
                   read: bool, rail: int | None, drv: int | None):
    """Connect, apply the requested settings (acknowledged), disconnect."""
    print(f"Connecting to {address} ...", flush=True)
    # use_cached_services=False: Windows serves stale GATT tables after
    # an OTA changes the services (winrt-only kwarg, ignored elsewhere)
    async with BleakClient(address,
                           winrt=dict(use_cached_services=False)) as client:
        print(f"Connected (MTU={client.mtu_size})", flush=True)
        if rail is not None:
            await send_rail(client, rail != 0, ack=True)
            await asyncio.sleep(0.1)   # let the rail settle before reading
        if drv is not None:
            await send_drv(client, drv != 0, ack=True)
        if volts is not None:
            await send_volt(client, volts, ack=True)
        if hz is not None:
            await send_freq(client, hz, ack=True)
        if volts is not None:
            await read_vdc(client)
        if read:
            await read_status(client)


async def interactive(address: str):
    """REPL loop for frequency / voltage changes."""
    print(f"Enter frequency in Hz ({FREQ_MIN}–{FREQ_MAX}),")
    print(f"'v <volts>' to set VDC ({VOLT_MIN}–{VOLT_MAX} V),")
    print("'r' to read measured VDC, or 'q' to quit.")
    print("Changes apply instantly via Write Without Response.\n")

    # use_cached_services=False: Windows serves stale GATT tables after
    # an OTA changes the services (winrt-only kwarg, ignored elsewhere)
    async with BleakClient(address,
                           winrt=dict(use_cached_services=False)) as client:
        print(f"Connected (MTU={client.mtu_size})\n", flush=True)
        while True:
            try:
                cmd = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if cmd.lower() in ("q", "quit", "exit"):
                break

            if not cmd:
                continue

            if cmd.lower() == "r":
                await read_status(client)
                continue

            if cmd.lower() in ("on", "off"):
                await send_rail(client, cmd.lower() == "on", ack=False)
                continue

            if cmd.lower() in ("wake", "sleep"):
                await send_drv(client, cmd.lower() == "wake", ack=False)
                continue

            if cmd.lower().startswith("v"):
                try:
                    volts = float(cmd[1:].strip())
                except ValueError:
                    print(f"  Invalid voltage: {cmd}")
                    continue
                if not (VOLT_MIN <= volts <= VOLT_MAX):
                    print(f"  Out of range ({VOLT_MIN}–{VOLT_MAX} V): {volts}")
                    continue
                await send_volt(client, volts, ack=False)
                continue

            try:
                hz = int(cmd)
            except ValueError:
                print(f"  Invalid: {cmd}")
                continue

            if hz < FREQ_MIN or hz > FREQ_MAX:
                print(f"  Out of range ({FREQ_MIN}–{FREQ_MAX}): {hz}")
                continue

            await send_freq(client, hz, ack=False)


async def dfu(address: str, path: str):
    """OTA update: upload a signed image over SMP, mark it, reboot."""
    try:
        from smpclient import SMPClient
        from smpclient.transport.ble import SMPBLETransport
        from smpclient.generics import error
        from smpclient.requests.image_management import (ImageStatesRead,
                                                         ImageStatesWrite)
        from smpclient.requests.os_management import ResetWrite
    except ImportError:
        print("DFU requires the smpclient package:  pip install smpclient",
              file=sys.stderr)
        sys.exit(1)

    with open(path, "rb") as f:
        image = f.read()
    print(f"DFU image: {path} ({len(image)} bytes)")

    print(f"Connecting (SMP) to {address} ...", flush=True)
    # Generous timeout: smpclient's Windows-MTU-bug workaround needs
    # several seconds of retries inside connect()
    async with SMPClient(SMPBLETransport(), address, timeout_s=20.0) as client:
        pre = await client.request(ImageStatesRead())
        if not error(pre):
            active = next((i for i in pre.images
                           if getattr(i, "active", False)), None)
            if active is not None:
                print(f"  device runs: version {active.version}")

        loop = asyncio.get_event_loop()
        start = loop.time()
        async for offset in client.upload(image):
            pct = 100 * offset // len(image)
            print(f"\r  upload {offset}/{len(image)} B ({pct}%)",
                  end="", flush=True)
        rate = len(image) / max(loop.time() - start, 1e-9) / 1024
        print(f"\r  upload {len(image)}/{len(image)} B (100%), {rate:.1f} KiB/s")

        states = await client.request(ImageStatesRead())
        if error(states):
            print(f"Image state read failed: {states}", file=sys.stderr)
            sys.exit(1)
        pending = next((i for i in states.images if i.slot == 1), None)
        if pending is None:
            print("Uploaded image not visible in slot 1", file=sys.stderr)
            sys.exit(1)
        print(f"  slot 1: version {pending.version}")

        marked = await client.request(
            ImageStatesWrite(hash=pending.hash, confirm=False))
        if error(marked):
            if "TEST_TO_ACTIVE_DENIED" in str(marked):
                print("Device already runs this exact image - nothing to do.")
                return
            print(f"Marking image failed: {marked}", file=sys.stderr)
            sys.exit(1)

        print("Image marked for install; rebooting device ...")
        reset = await client.request(ResetWrite())
        if error(reset):
            print(f"Reset failed: {reset}", file=sys.stderr)
            sys.exit(1)

    print("DFU sent - device installs and boots the new firmware (~10 s).")


async def main():
    parser = argparse.ArgumentParser(
        description="Caterpillar BLE frequency / voltage control")
    parser.add_argument("-f", "--freq", type=int, default=None,
                        help="Frequency in Hz (one-shot mode)")
    parser.add_argument("-V", "--V", "--volt", dest="volt", type=float,
                        default=None,
                        help="Motor rail voltage in volts (one-shot mode)")
    parser.add_argument("-r", "--read", action="store_true",
                        help="Read measured VDC (one-shot mode)")
    parser.add_argument("--rail", type=int, choices=[0, 1], default=None,
                        help="Motor rail enable: 0=off, 1=on (one-shot mode)")
    parser.add_argument("--drv", type=int, choices=[0, 1], default=None,
                        help="DRV8212 driver: 0=sleep, 1=awake (one-shot mode)")
    parser.add_argument("--dfu", metavar="SIGNED_BIN", default=None,
                        help="OTA-update firmware (zephyr.signed.bin)")
    parser.add_argument("--tput", metavar="KIB", type=int, nargs="?",
                        const=64, default=None,
                        help="BLE throughput test (default 64 KiB)")
    parser.add_argument("--robot", type=int, default=None, metavar="N",
                        help="Select fleet robot #N (adv name Cat-NN)")
    parser.add_argument("--name", default=None,
                        help="Select device by exact advertised name")
    parser.add_argument("--scan", action="store_true",
                        help="List all visible Caterpillars and exit")
    parser.add_argument("--set-id", type=int, default=None, metavar="N",
                        dest="set_id",
                        help="Assign fleet robot number 1-20 (0 = clear)")
    args = parser.parse_args()

    if args.scan:
        robots = await scan_fleet()
        if not robots:
            print("No Caterpillars visible.")
        for e in robots:
            print(f"  {_fmt_robot(e)}")
        return

    if args.set_id is not None and not (0 <= args.set_id <= 20):
        print(f"Robot ID out of range (0-20): {args.set_id}",
              file=sys.stderr)
        sys.exit(1)

    if args.freq is not None and not (FREQ_MIN <= args.freq <= FREQ_MAX):
        print(f"Frequency out of range ({FREQ_MIN}-{FREQ_MAX}): {args.freq}",
              file=sys.stderr)
        sys.exit(1)

    if args.volt is not None and not (VOLT_MIN <= args.volt <= VOLT_MAX):
        print(f"Voltage out of range ({VOLT_MIN}-{VOLT_MAX} V): {args.volt}",
              file=sys.stderr)
        sys.exit(1)

    address = await discover(robot=args.robot, name=args.name)
    if address is None:
        sys.exit(1)

    if args.set_id is not None:
        await set_robot_id(address, args.set_id)
        return

    if args.dfu is not None:
        await dfu(address, args.dfu)
        return

    if args.tput is not None:
        await tput_test(address, args.tput)
        return

    if (args.freq is not None or args.volt is not None or args.read
            or args.rail is not None or args.drv is not None):
        try:
            await one_shot(address, args.freq, args.volt, args.read,
                           args.rail, args.drv)
        except Exception as e:
            print(f"Write failed: {e}", file=sys.stderr)
            sys.exit(1)
    else:
        await interactive(address)


if __name__ == "__main__":
    asyncio.run(main())
