#ifndef MAX5419_H
#define MAX5419_H

#include <zephyr/kernel.h>

/* MAX5419LETA I2C address (A0 = GND — datasheet Table 1, suffix L) */
#define MAX5419_I2C_ADDR    0x28

/*
 * Motor rail (VDC) range achievable through the STBB1-APUR feedback
 * network (rev3: fixed 12.1 kΩ in series with the digipot leg).
 * Min = tap 0 (≈0.74 V).  Hardware bounds the max at ≈4.63 V (tap 255);
 * firmware caps requests at 4.2 V (tap ≤ 253, see max5419.c).
 */
#define MAX5419_VOUT_MIN_MV  750
#define MAX5419_VOUT_MAX_MV  4200

/*
 * NOTE: The MAX5419 is write-only over I²C.
 * The 8th address bit is NOP/W (no-op/write), not R/W (read/write).
 * Reading back the wiper position via I²C is not supported by this device.
 */

/**
 * @brief Initialize and verify the MAX5419LETA on the I2C bus.
 *
 * Probes the device at 0x28 to confirm it is present.
 *
 * @return 0 on success, negative errno on failure.
 */
int max5419_init(void);

/**
 * @brief Set the wiper tap position.
 *
 * Tap 0   = W at L end → maximum resistance H–W (200 kΩ) → minimum Vout
 * Tap 255 = W at H end → R_HW = 0 → only R9 (12.1 kΩ) remains → ≈4.63 V
 *
 * Taps that would push the STBB1-APUR output above the firmware voltage
 * limit (see max5419.c) are rejected with -EINVAL.
 *
 * @param tap  Wiper position (0–253).
 * @return 0 on success, negative errno on failure.
 */
int max5419_set_tap(uint8_t tap);

/**
 * @brief Set the STBB1-APUR output voltage via the digipot.
 *
 * Uses the rev3 feedback formula:
 *   Vout = 0.5 × (1 + 100 / (12.1 + R_HW))   [kΩ]
 *   R_HW = 200 × (255 − tap) / 255
 *
 * Requests above the firmware limit (4.2 V — see VOUT_MAX_V in
 * max5419.c) are rejected with -EINVAL; the hardware itself bounds
 * the rail at ≈4.63 V via the fixed series resistor.
 *
 * @param voltage  Desired output voltage in volts (0.5 < V ≤ 4.2).
 * @return 0 on success, negative errno on failure.
 */
int max5419_set_voltage(float voltage);

#endif /* MAX5419_H */
