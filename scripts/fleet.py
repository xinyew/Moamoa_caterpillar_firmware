"""Caterpillar fleet tool — deploy / collect across many robots.

Workflow (few robots live at a time; sequential connections):

  python fleet.py scan                     # who is out there?
  python fleet.py assign 3                 # name the (single) visible robot Cat-03
  python fleet.py deploy --robots 1,2,3    # start DETACHED log sessions + disconnect
  ... robots log untethered (survives disconnect) ...
  python fleet.py collect --robots 1,2,3 -o data/   # stop, dump newest session each

`--all` targets every visible Caterpillar.  Data lands as CSV + NPZ per
robot, named cat<NN>_<start-timestamp>.  Uses protocol.py (firmware
>= v1.5.0 for identity/detached features).
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from pathlib import Path

import numpy as np
from bleak import BleakClient, BleakScanner

import protocol as P


# ---- discovery --------------------------------------------------------------

async def scan_fleet(timeout: float = 6.0) -> list[dict]:
    """Every visible Caterpillar: {name, address, robot_id, fw,
    session_active, rssi} (identity fields None on legacy firmware)."""
    found: dict[str, dict] = {}

    def on_adv(device, adv):
        name = device.name or adv.local_name
        info = P.parse_adv(name, adv.manufacturer_data)
        if info is None:
            return
        info.update(address=device.address, rssi=adv.rssi, dev=device)
        found[device.address] = info

    scanner = BleakScanner(on_adv)
    await scanner.start()
    await asyncio.sleep(timeout)
    await scanner.stop()
    return sorted(found.values(),
                  key=lambda e: (e["robot_id"] is None,
                                 e["robot_id"] or 0, e["name"]))


def fmt_robot(e: dict) -> str:
    rid = "unassigned" if not e["robot_id"] else f"#{e['robot_id']}"
    fw = f", fw {'.'.join(map(str, e['fw']))}" if e["fw"] else ""
    sess = ", LOGGING" if e["session_active"] else ""
    return (f"{e['name']:<10} [{e['address']}] ({rid}{fw}{sess}) "
            f"{e['rssi']} dBm")


def select_targets(robots: list[dict], args) -> list[dict]:
    if args.all:
        return robots
    want = {int(r) for r in args.robots.split(",")} if args.robots else None
    if want is None:
        if len(robots) == 1:
            return robots
        print("Several robots visible - pick with --robots N,M or --all:",
              file=sys.stderr)
        for e in robots:
            print(f"  {fmt_robot(e)}", file=sys.stderr)
        sys.exit(1)
    sel = [e for e in robots if e["robot_id"] in want]
    missing = want - {e["robot_id"] for e in sel}
    if missing:
        print(f"Not visible: {sorted(missing)} "
              f"(saw {[e['name'] for e in robots]})", file=sys.stderr)
        sys.exit(1)
    return sel


def connect_client(e: dict) -> BleakClient:
    # Connect via the scanner's BLEDevice handle — a bare address
    # fails on WinRT unless Windows happens to have the device cached.
    return BleakClient(e["dev"], winrt=dict(use_cached_services=False))


# ---- per-robot operations ---------------------------------------------------

async def robot_assign(e: dict, new_id: int):
    async with connect_client(e) as client:
        await client.write_gatt_char(P.UUID_ROBOT_ID, bytes([new_id]),
                                     response=True)
    if new_id:
        print(f"  ID -> {new_id} (advertises as Cat-{new_id:02d} now)")
    else:
        print("  ID cleared (reverts to Cat-XXXX)")


async def robot_deploy(e: dict) -> bool:
    """Sync clock and start a DETACHED session, then disconnect."""
    async with connect_client(e) as client:
        await client.write_gatt_char(P.UUID_TIME,
                                     P.encode_time(int(time.time())),
                                     response=True)
        state = P.decode_log_state(bytes(
            await client.read_gatt_char(P.UUID_LOG_CTL)))
        if state.active:
            print(f"  {e['name']}: session already running - left as is")
            return True
        await client.write_gatt_char(
            P.UUID_LOG_CTL,
            P.encode_log_cmd(P.LOG_CMD_START_DETACHED),
            response=True)
        await asyncio.sleep(0.3)   # start executes on the cmd queue
        state = P.decode_log_state(bytes(
            await client.read_gatt_char(P.UUID_LOG_CTL)))
        if not state.active:
            print(f"  {e['name']}: session did NOT start", file=sys.stderr)
            return False
        print(f"  {e['name']}: detached session started")
        return True


class _DumpSink:
    """Reassembles dump chunks with the same stall-resume strategy as
    the GUI (re-request from the highest contiguous offset)."""

    def __init__(self, total: int):
        self.buf = bytearray(total)
        self.total = total
        self.max_end = 0
        self.last_rx = time.monotonic()
        self.done = asyncio.Event()

    def on_chunk(self, _char, data: bytearray):
        if self.done.is_set():
            return
        c = P.decode_dump_chunk(bytes(data))
        end = min(c.offset + len(c.data), self.total)
        self.buf[c.offset:end] = c.data[:end - c.offset]
        self.max_end = max(self.max_end, end)
        self.last_rx = time.monotonic()
        if (c.last and end >= self.total) or self.max_end >= self.total:
            self.done.set()


async def robot_collect(e: dict, out_dir: Path, erase: bool) -> bool:
    """Stop the session, dump the newest one, save CSV+NPZ."""
    async with connect_client(e) as client:
        await client.write_gatt_char(P.UUID_LOG_CTL,
                                     P.encode_log_cmd(P.LOG_CMD_STOP),
                                     response=True)
        await asyncio.sleep(0.3)
        sessions = P.decode_sessions(bytes(
            await client.read_gatt_char(P.UUID_DIR)))
        if not sessions or sessions[0].rec_count == 0:
            print(f"  {e['name']}: no stored session", file=sys.stderr)
            return False
        sess = sessions[0]            # newest first
        total = sess.bytes

        sink = _DumpSink(total)
        await client.start_notify(P.UUID_DUMP, sink.on_chunk)
        await client.write_gatt_char(P.UUID_DUMP,
                                     P.encode_dump_req(sess.seq, 0, total),
                                     response=True)
        t0 = time.monotonic()
        resumes = 0
        while not sink.done.is_set():
            try:
                await asyncio.wait_for(sink.done.wait(), timeout=1.0)
                break
            except asyncio.TimeoutError:
                pass
            print(f"\r  {e['name']}: {sink.max_end * 100 // total}% "
                  f"({sink.max_end}/{total} B)", end="", flush=True)
            if time.monotonic() - sink.last_rx < 5.0:
                continue
            if resumes >= 20:
                print(f"\n  {e['name']}: dump failed after {resumes} "
                      f"resumes", file=sys.stderr)
                return False
            resumes += 1
            frm = sink.max_end
            sink.last_rx = time.monotonic()
            await client.write_gatt_char(
                P.UUID_DUMP, P.encode_dump_req(sess.seq, frm, total - frm),
                response=True)
        dt = time.monotonic() - t0
        print(f"\r  {e['name']}: dumped {total} B in {dt:.1f} s "
              f"({total / dt / 1024:.1f} KiB/s)")

        if erase:
            await client.write_gatt_char(P.UUID_LOG_CTL,
                                         P.encode_log_cmd(P.LOG_CMD_ERASE),
                                         response=True)
            await asyncio.sleep(0.5)
            print(f"  {e['name']}: storage erased")

    save_session(e, bytes(sink.buf), sess, out_dir)
    return True


def save_session(e: dict, raw: bytes, sess: P.Session, out_dir: Path):
    recs = np.array(list(P.unpack_records(raw)), dtype=np.int64)
    if recs.size == 0:
        print(f"  {e['name']}: dump contained no records", file=sys.stderr)
        return

    odr = P.ODR_HZ.get(sess.odr_code, 0)
    acc = recs[:, 0:3] * (P.ACCEL_MG_PER_LSB[sess.accel_fs] / 1000.0)
    gyr = recs[:, 3:6] * (P.GYRO_MDPS_PER_LSB[sess.gyro_fs] / 1000.0)
    temp = recs[:, 6] / 256.0 + 25.0
    seq = recs[:, 7]
    n = recs.shape[0]
    t = np.arange(n) / odr if odr else np.arange(n, dtype=float)

    rid = f"{e['robot_id']:02d}" if e["robot_id"] else e["name"][4:]
    stamp = (time.strftime("%Y%m%d_%H%M%S", time.localtime(sess.wall_start))
             if sess.wall_start else f"session{sess.seq}")
    base = out_dir / f"cat{rid}_{stamp}"
    when = (time.strftime("%Y-%m-%d %H:%M:%S",
                          time.localtime(sess.wall_start))
            if sess.wall_start else "unsynced clock")

    with open(base.with_suffix(".csv"), "w", newline="") as f:
        f.write(f"# Caterpillar {e['name']} IMU session {sess.seq}, "
                f"started {when}, {odr} Hz, "
                f"accel +/-{P.ACCEL_FS_G[sess.accel_fs]} g, "
                f"gyro +/-{P.GYRO_FS_DPS[sess.gyro_fs]} dps\n")
        f.write("t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,seq16\n")
        for i in range(n):
            f.write(f"{t[i]:.6f},{acc[i,0]:.5f},{acc[i,1]:.5f},"
                    f"{acc[i,2]:.5f},{gyr[i,0]:.4f},{gyr[i,1]:.4f},"
                    f"{gyr[i,2]:.4f},{temp[i]:.2f},{seq[i]}\n")

    np.savez_compressed(
        base.with_suffix(".npz"), t_s=t, accel_g=acc, gyro_dps=gyr,
        temp_c=temp, seq16=seq, odr_hz=odr,
        accel_fs_g=P.ACCEL_FS_G[sess.accel_fs],
        gyro_fs_dps=P.GYRO_FS_DPS[sess.gyro_fs],
        wall_start=sess.wall_start, robot=e["name"])
    print(f"  {e['name']}: saved {base}.csv / .npz ({n} records)")


# ---- commands ---------------------------------------------------------------

async def cmd_scan(_args):
    robots = await scan_fleet()
    if not robots:
        print("No Caterpillars visible.")
        return
    print(f"{len(robots)} Caterpillar(s):")
    for e in robots:
        print(f"  {fmt_robot(e)}")


async def cmd_assign(args):
    robots = await scan_fleet()
    if args.name:
        robots = [e for e in robots if e["name"] == args.name]
    if not robots:
        print("Target robot not found.", file=sys.stderr)
        sys.exit(1)
    if len(robots) > 1:
        print("Several robots visible - disambiguate with --name Cat-XXXX:",
              file=sys.stderr)
        for e in robots:
            print(f"  {fmt_robot(e)}", file=sys.stderr)
        sys.exit(1)
    print(f"Assigning ID {args.id} to {robots[0]['name']} ...")
    await robot_assign(robots[0], args.id)


async def cmd_deploy(args):
    robots = select_targets(await scan_fleet(), args)
    print(f"Deploying detached sessions to {len(robots)} robot(s) ...")
    ok = 0
    for e in robots:
        try:
            ok += await robot_deploy(e)
        except Exception as exc:
            print(f"  {e['name']}: FAILED ({exc})", file=sys.stderr)
    print(f"Deployed {ok}/{len(robots)}.")
    sys.exit(0 if ok == len(robots) else 1)


async def cmd_collect(args):
    robots = select_targets(await scan_fleet(), args)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Collecting from {len(robots)} robot(s) into {out_dir} ...")
    ok = 0
    for e in robots:
        try:
            ok += await robot_collect(e, out_dir, args.erase)
        except Exception as exc:
            print(f"  {e['name']}: FAILED ({exc})", file=sys.stderr)
    print(f"Collected {ok}/{len(robots)}.")
    sys.exit(0 if ok == len(robots) else 1)


def main():
    ap = argparse.ArgumentParser(
        description="Caterpillar fleet deploy/collect")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", help="list visible Caterpillars")

    p = sub.add_parser("assign", help="set the fleet ID of one robot")
    p.add_argument("id", type=int, help="robot number 1-20 (0 = clear)")
    p.add_argument("--name", default=None,
                   help="target by current adv name (e.g. Cat-A3F2)")

    for name, hlp in (("deploy", "start detached log sessions"),
                      ("collect", "stop sessions and dump newest data")):
        p = sub.add_parser(name, help=hlp)
        p.add_argument("--robots", default=None, metavar="N,M,...",
                       help="fleet numbers to target")
        p.add_argument("--all", action="store_true",
                       help="target every visible Caterpillar")
        if name == "collect":
            p.add_argument("-o", "--out", default="fleet_data",
                           help="output directory (default fleet_data/)")
            p.add_argument("--erase", action="store_true",
                           help="erase robot storage after a good dump")

    args = ap.parse_args()
    if args.cmd == "assign" and not (0 <= args.id <= 20):
        ap.error("id must be 0-20")

    asyncio.run({"scan": cmd_scan, "assign": cmd_assign,
                 "deploy": cmd_deploy, "collect": cmd_collect}[args.cmd](args))


if __name__ == "__main__":
    main()
