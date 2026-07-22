#ifndef DRIVER_STBB1_APUR_H
#define DRIVER_STBB1_APUR_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize STBB1-APUR DCDC converter enable pin (P2.03).
 *
 * Configures P2.03 as output-high to enable the DCDC converter.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_stbb1_apur_init(void);

/**
 * @brief Enable or disable the motor rail (drives P2.03).
 *
 * Note: a healthy STBB1 has true output disconnect, so disable should
 * drop VDC to ~0.  VDC staying near VBATT − 0.7 V while disabled
 * indicates a passive path through the converter (damaged input FET).
 *
 * @param enable  true = converter on, false = off.
 */
void drv_stbb1_apur_set(bool enable);

/** @brief Whether the motor rail is currently enabled. */
bool drv_stbb1_apur_enabled(void);

#endif /* DRIVER_STBB1_APUR_H */
