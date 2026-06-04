#ifndef DRIVER_DRV8212_H
#define DRIVER_DRV8212_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize the DRV8212P motor driver sleep pin (P1.06).
 *
 * Configures P1.06 as output-low to enable the driver.
 * ~SLEEP: LOW = on, HIGH = off.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_drv8212_init(void);

#endif /* DRIVER_DRV8212_H */
