#!/usr/bin/env python3
"""
Caterpillar BLE control — low-latency PWM frequency setter.

Connects to a "Caterpillar" device and writes a 16-bit frequency value
(Hz, little-endian) via Write Without Response to a custom GATT
characteristic (UUID 0xFFE1).

Usage:
    python ble_control.py              # interactive mode
    python ble_control.py --freq 170   # one-shot
    python ble_control.py -f 170       # short form

Requires: pip install bleak
"""

import argparse
import asyncio
import struct
import sys

from bleak import BleakClient, BleakScanner

# Must match firmware — see ble_interface.c
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR_UUID    = "0000ffe1-0000-1000-8000-00805f9b34fb"
DEVICE_NAME  = "Caterpillar"

# Must match firmware (driver_pwm.h): nRF54 PWM cannot go below ~4 Hz
FREQ_MIN = 4
FREQ_MAX = 1000


def pack_freq(hz: int) -> bytes:
    """Pack a 16-bit frequency into little-endian bytes."""
    return struct.pack("<H", hz)


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


async def set_freq(address: str, hz: int):
    """Connect, set frequency, disconnect."""
    print(f"Connecting to {address} ...", flush=True)
    async with BleakClient(address) as client:
        print(f"Connected (MTU={client.mtu_size})", flush=True)
        data = pack_freq(hz)
        await client.write_gatt_char(CHAR_UUID, data, response=False)
        print(f"Sent: {hz} Hz → {data.hex()}", flush=True)


async def interactive(address: str):
    """REPL loop for frequency changes."""
    print(f"Enter frequency in Hz ({FREQ_MIN}–{FREQ_MAX}), or 'q' to quit.")
    print("Changes apply instantly via Write Without Response.\n")

    async with BleakClient(address) as client:
        print(f"Connected (MTU={client.mtu_size})\n", flush=True)
        while True:
            try:
                cmd = input("Hz> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if cmd.lower() in ("q", "quit", "exit"):
                break

            if not cmd:
                continue

            try:
                hz = int(cmd)
            except ValueError:
                print(f"  Invalid: {cmd}")
                continue

            if hz < FREQ_MIN or hz > FREQ_MAX:
                print(f"  Out of range ({FREQ_MIN}–{FREQ_MAX}): {hz}")
                continue

            data = pack_freq(hz)
            await client.write_gatt_char(CHAR_UUID, data, response=False)
            print(f"  → {hz} Hz")


async def main():
    parser = argparse.ArgumentParser(description="Caterpillar BLE frequency control")
    parser.add_argument("-f", "--freq", type=int, default=None,
                        help="Frequency in Hz (one-shot mode)")
    args = parser.parse_args()

    if args.freq is not None and not (FREQ_MIN <= args.freq <= FREQ_MAX):
        print(f"Frequency out of range ({FREQ_MIN}-{FREQ_MAX}): {args.freq}",
              file=sys.stderr)
        sys.exit(1)

    address = await discover()
    if address is None:
        sys.exit(1)

    if args.freq is not None:
        await set_freq(address, args.freq)
    else:
        await interactive(address)


if __name__ == "__main__":
    asyncio.run(main())
