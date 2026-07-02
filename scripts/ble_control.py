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
SERVICE_UUID   = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR_UUID_FREQ = "0000ffe1-0000-1000-8000-00805f9b34fb"
CHAR_UUID_VOLT = "0000ffe2-0000-1000-8000-00805f9b34fb"
DEVICE_NAME    = "Caterpillar"

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


async def discover() -> str | None:
    """Scan for the Caterpillar device and return its BLE address."""
    print(f"Scanning for \"{DEVICE_NAME}\" ...", flush=True)

    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME,
        timeout=10.0,
    )

    if device is None:
        print(f"Device \"{DEVICE_NAME}\" not found.", file=sys.stderr)
        return None

    print(f"Found {device.name} [{device.address}]")
    return device.address


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


async def one_shot(address: str, hz: int | None, volts: float | None):
    """Connect, apply the requested settings (acknowledged), disconnect."""
    print(f"Connecting to {address} ...", flush=True)
    async with BleakClient(address) as client:
        print(f"Connected (MTU={client.mtu_size})", flush=True)
        if volts is not None:
            await send_volt(client, volts, ack=True)
        if hz is not None:
            await send_freq(client, hz, ack=True)


async def interactive(address: str):
    """REPL loop for frequency / voltage changes."""
    print(f"Enter frequency in Hz ({FREQ_MIN}–{FREQ_MAX}),")
    print(f"or 'v <volts>' to set VDC ({VOLT_MIN}–{VOLT_MAX} V), or 'q' to quit.")
    print("Changes apply instantly via Write Without Response.\n")

    async with BleakClient(address) as client:
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


async def main():
    parser = argparse.ArgumentParser(
        description="Caterpillar BLE frequency / voltage control")
    parser.add_argument("-f", "--freq", type=int, default=None,
                        help="Frequency in Hz (one-shot mode)")
    parser.add_argument("-V", "--V", "--volt", dest="volt", type=float,
                        default=None,
                        help="Motor rail voltage in volts (one-shot mode)")
    args = parser.parse_args()

    if args.freq is not None and not (FREQ_MIN <= args.freq <= FREQ_MAX):
        print(f"Frequency out of range ({FREQ_MIN}-{FREQ_MAX}): {args.freq}",
              file=sys.stderr)
        sys.exit(1)

    if args.volt is not None and not (VOLT_MIN <= args.volt <= VOLT_MAX):
        print(f"Voltage out of range ({VOLT_MIN}-{VOLT_MAX} V): {args.volt}",
              file=sys.stderr)
        sys.exit(1)

    address = await discover()
    if address is None:
        sys.exit(1)

    if args.freq is not None or args.volt is not None:
        try:
            await one_shot(address, args.freq, args.volt)
        except Exception as e:
            print(f"Write failed: {e}", file=sys.stderr)
            sys.exit(1)
    else:
        await interactive(address)


if __name__ == "__main__":
    asyncio.run(main())
