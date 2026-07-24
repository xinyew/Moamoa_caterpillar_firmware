<!-- AUTO-GENERATED from settings.yml — do not edit; run scripts/generate_settings.py -->
# Device Settings

Persisted across reboots (spare 4 KB flash partition at
0x164000); applied at boot.  Motor rail/driver enables are not
settings — the motor always boots off.  Source: `settings.yml`.

| ID | Setting | Default | Range | Description |
|---|---|---|---|---|
| 1 | `motor_freq_hz` | 150 | 4–1000 | PWM drive frequency [Hz] |
| 2 | `motor_vdc_mv` | 3100 | 750–4200 | Motor rail voltage target [mV] (digipot, tap-quantized) |
| 3 | `imu_odr_code` | 7 | 1–10 | IMU ODR code (1=12.5 Hz .. 10=6.66 kHz) |
| 4 | `imu_content` | 3 | 1–3 | IMU content mask (bit0 accel, bit1 gyro) |
| 5 | `imu_accel_fs` | 0 | 0–3 | Accel full-scale (0=±2g 1=±4g 2=±8g 3=±16g) |
| 6 | `imu_gyro_fs` | 0 | 0–3 | Gyro full-scale (0=±250 1=±500 2=±1000 3=±2000 dps) |
| 7 | `preview_hz` | 0 | 0–6660 | Live-stream rate cap [Hz] (0 = auto/link budget) |
| 8 | `led_enabled` | 1 | 0–1 | Heartbeat LED on/off |
| 9 | `robot_id` | 0 | 0–20 | Fleet robot number (0 = unassigned; advertised as Cat-NN, unassigned robots show Cat-XXXX from the chip id) |
