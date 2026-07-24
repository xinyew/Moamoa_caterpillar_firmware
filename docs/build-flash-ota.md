# Build, Flash & OTA

## Build (one command)

```
nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 -- ^
  west build -b caterpillar/nrf54l15/cpuapp -- -DBOARD_ROOT="<repo path>"
```

- `-DBOARD_ROOT` is REQUIRED for CLI sysbuild to find the custom board.
- The FLPR is built automatically: the app's CMake runs an incremental
  FLPR build (`build-flpr/`, configured on first use) every app build
  and embeds `zephyr.bin` via `copy_if_different` staging — an
  unchanged FLPR costs ~2 s and does not re-link the app.  There is no
  separate FLPR build or flash step, ever; `build-flpr/` is a
  regenerable intermediate.
- Outputs: `build/merged.hex` (SWD: MCUboot + signed app) and
  `build/Moamoa_caterpillar_firmware/zephyr/zephyr.signed.bin` (OTA).
- Images are auto-signed with the SDK dev key — the "not secure"
  warning is expected; DFU is intentionally open.

## Versioning discipline

- Bump `VERSION` (app) every release — the DFU pre-check and `--read`
  report it, and the on-chip **log directory is wiped on the first
  boot of any new firmware** (by design: the upload destroys log data).
- Bump `flpr/VERSION` whenever FLPR code changes — the device reports
  the *running* FLPR version (status byte 24+), proving the embedded
  coprocessor update actually took.

## Flash over SWD

```
west flash --recover      # full chip: MCUboot + app; erases EVERYTHING
```

`--recover` is required because the MCUboot region is
bootconf-protected; plain `west flash` (and currently also
`--domain Moamoa_caterpillar_firmware`) fails with "protected RRAMC
region".  Kill any running `commander-cli` first (J-Link contention).

## OTA over BLE

```
python scripts/ble_control.py --dfu build/Moamoa_caterpillar_firmware/zephyr/zephyr.signed.bin
```

Needs `pip install smpclient`.  Upload ≈ 15–20 KiB/s (~3–4 min for a
~214 KB image incl. install), then MCUboot copies the slot (~40 s,
radio silent) and boots.  Same-image upload is a benign no-op.
Verify with `--read`: app version, FLPR version, IMU ok.

Rules:
- The device must be disconnected from other centrals (GUI!) — the
  scan cannot see a connected device.
- OTA wipes all stored log sessions; dump first.
- Don't power-cycle during the ~40 s install (overwrite-only mode has
  no revert; recovery is SWD `--recover`).

## RTT (wired debug)

```
commander-cli rtt connect --device nrf54l15_M33
```

Warnings/errors also go to BLE 0xFFEC; RTT carries full boot logs.

## VS Code IntelliSense

`.vscode/` (committed) points the C/C++ extension at the build's
`compile_commands.json` for both cores.  New machine: install C/C++ +
CMake extensions, run one full build, Reload Window.
