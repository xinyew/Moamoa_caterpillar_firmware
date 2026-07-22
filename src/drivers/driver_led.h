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
 * First ~5 s after boot: 3 × 3 ms flashes per second — reset marker.
 * Afterwards: a single 3 ms flash per second — running.
 */
void drv_led_blink_start(void);

/** @brief Stop the pattern and force the LED on (true) or off (false). */
void drv_led_set(bool on);

/** @brief Toggle the status LED (does not stop a running pattern). */
void drv_led_toggle(void);

#endif /* DRIVER_LED_H */
