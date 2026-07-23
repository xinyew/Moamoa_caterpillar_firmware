#ifndef DRIVER_LED_H
#define DRIVER_LED_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize the status LED (P0.01, active-high) and turn it on.
 *
 * On-at-boot doubles as a power/boot indicator until the blink pattern
 * is started.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_led_init(void);

/**
 * @brief Start the alive pattern (kernel-timer driven).
 *
 * First ~3 s after boot: 3 × 3 ms flashes per second — reset marker.
 * Afterwards: a single 3 ms flash per second — running.
 */
void drv_led_blink_start(void);

/** @brief Stop the pattern and force the LED on (true) or off (false). */
void drv_led_set(bool on);

/**
 * @brief Enable/disable the heartbeat (BLE-controlled).
 *
 * Disable stops the timer and turns the LED off; enable resumes the
 * running pattern (without replaying the reset marker).
 */
void drv_led_set_enabled(bool enable);

/** @brief Whether the heartbeat is currently enabled. */
bool drv_led_enabled(void);

/** @brief Toggle the status LED (does not stop a running pattern). */
void drv_led_toggle(void);

#endif /* DRIVER_LED_H */
