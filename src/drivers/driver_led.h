#ifndef DRIVER_LED_H
#define DRIVER_LED_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize the status LED (P0.01, active-high) and turn it on.
 *
 * On-at-boot doubles as a power/boot indicator until the main loop
 * starts heartbeat-blinking it.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_led_init(void);

/** @brief Set the status LED on (true) or off (false). */
void drv_led_set(bool on);

/** @brief Toggle the status LED. */
void drv_led_toggle(void);

#endif /* DRIVER_LED_H */
