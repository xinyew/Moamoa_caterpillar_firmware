/*
 * STBB1-APUR DCDC converter enable driver.
 *
 * Controls P2.03 (GPIO2, pin 3) — active-high enable for the STBB1-APUR
 * DCDC converter that supplies motor voltage.
 */

#include "driver_stbb1_apur.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_stbb1_apur, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------- */
/*  Pin definition                                                            */
/* -------------------------------------------------------------------------- */

#define DCDC_EN_PORT  DEVICE_DT_GET(DT_NODELABEL(gpio2))
#define DCDC_EN_PIN   3

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

int drv_stbb1_apur_init(void)
{
    int ret;

    if (!device_is_ready(DCDC_EN_PORT)) {
        LOG_ERR("GPIO2 not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure(DCDC_EN_PORT, DCDC_EN_PIN,
                              GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure DCDC_EN (P2.03): %d", ret);
        return ret;
    }

    /*
     * Read-back verification — on nRF54 the input buffer is always
     * connected even in output mode, so gpio_pin_get() returns the
     * real physical pin level.
     */
    int level = gpio_pin_get(DCDC_EN_PORT, DCDC_EN_PIN);
    LOG_INF("STBB1-APUR DCDC_EN (P2.03): %s (read-back=%d)",
            level > 0 ? "HIGH" : "LOW", level);
    if (level <= 0) {
        LOG_WRN("DCDC_EN read-back is LOW — check for short or floating pin");
    }

    return 0;
}
