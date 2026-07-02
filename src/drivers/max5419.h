#ifndef MAX5419_H
#define MAX5419_H

#include <zephyr/kernel.h>

/* MAX5419LETA I2C address (A0 = GND — datasheet Table 1, suffix L) */
#define MAX5419_I2C_ADDR    0x28

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
 * Tap 255 = W at H end → R_HW = 0 → FB shorted to GND → runaway Vout
 *
 * Taps that would push the STBB1-APUR output above the safe voltage
 * limit (see max5419.c) are rejected with -EINVAL.
 *
 * @param tap  Wiper position (0–237).
 * @return 0 on success, negative errno on failure.
 */
int max5419_set_tap(uint8_t tap);

/**
 * @brief Set the STBB1-APUR output voltage via the digipot.
 *
 * Uses the feedback formula:
 *   Vout = 0.5 × (100 / R_HW_kΩ + 1)
 *   R_HW = 200 × (255 − tap) / 255
 *
 * Requests above the safe limit (4.2 V — see VOUT_MAX_V in max5419.c)
 * are rejected with -EINVAL to prevent the boost converter from being
 * driven toward its 5.5 V maximum into the motor.
 *
 * @param voltage  Desired output voltage in volts (0.5 < V ≤ 4.2).
 * @return 0 on success, negative errno on failure.
 */
int max5419_set_voltage(float voltage);

#endif /* MAX5419_H */
