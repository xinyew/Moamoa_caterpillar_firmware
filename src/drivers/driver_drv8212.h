#ifndef DRIVER_DRV8212_H
#define DRIVER_DRV8212_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize the DRV8212P motor driver nSLEEP pin (P1.06).
 *
 * Configures P1.06 as output-high to wake the driver.
 * nSLEEP: LOW = sleep (disabled), HIGH = awake (enabled).
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_drv8212_init(void);

/**
 * @brief Wake or sleep the DRV8212P (drives nSLEEP, P1.06).
 *
 * Sleep puts the H-bridge outputs Hi-Z (coast) at µA-level quiescent
 * draw, independent of the motor rail (see drv_stbb1_apur_set).
 *
 * @param awake  true = nSLEEP high (driver active), false = sleep.
 */
void drv_drv8212_set(bool awake);

/** @brief Whether the driver is currently awake. */
bool drv_drv8212_awake(void);

#endif /* DRIVER_DRV8212_H */
