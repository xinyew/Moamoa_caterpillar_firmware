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

static bool rail_on;

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

    /* Boot with the rail OFF — sessions enable it explicitly over BLE */
    ret = gpio_pin_configure(DCDC_EN_PORT, DCDC_EN_PIN,
                              GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure DCDC_EN (P2.03): %d", ret);
        return ret;
    }

    LOG_INF("STBB1-APUR DCDC_EN (P2.03): LOW (rail off at boot)");
    rail_on = false;
    return 0;
}

void drv_stbb1_apur_set(bool enable)
{
    gpio_pin_set_raw(DCDC_EN_PORT, DCDC_EN_PIN, enable ? 1 : 0);
    rail_on = enable;
    LOG_INF("STBB1-APUR DCDC_EN -> %s", enable ? "ON" : "OFF");
}

bool drv_stbb1_apur_enabled(void)
{
    return rail_on;
}
