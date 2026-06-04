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

#endif /* DRIVER_STBB1_APUR_H */
