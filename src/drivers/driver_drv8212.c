/*
 * DRV8212P motor driver — nSLEEP pin control.
 *
 * Controls P1.06 (GPIO1, pin 6) — active-low nSLEEP pin.
 * Per TI datasheet:
 *   LOW  = sleep mode (driver disabled, outputs Hi-Z)
 *   HIGH = awake     (driver enabled)
 */

#include "driver_drv8212.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_drv8212, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------- */
/*  Pin definition                                                            */
/* -------------------------------------------------------------------------- */

#define DRV8212_SLEEP_PORT  DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define DRV8212_SLEEP_PIN   6

static bool drv_awake;

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

int drv_drv8212_init(void)
{
    int ret;

    if (!device_is_ready(DRV8212_SLEEP_PORT)) {
        LOG_ERR("GPIO1 not ready");
        return -ENODEV;
    }

    /* Boot asleep (nSLEEP LOW) — sessions wake it explicitly over BLE */
    ret = gpio_pin_configure(DRV8212_SLEEP_PORT, DRV8212_SLEEP_PIN,
                              GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Failed to configure DRV8212 nSLEEP (P1.06): %d", ret);
        return ret;
    }

    LOG_INF("DRV8212 nSLEEP (P1.06): LOW (asleep at boot)");
    drv_awake = false;
    return 0;
}

void drv_drv8212_set(bool awake)
{
    gpio_pin_set_raw(DRV8212_SLEEP_PORT, DRV8212_SLEEP_PIN, awake ? 1 : 0);
    drv_awake = awake;
    LOG_INF("DRV8212 nSLEEP -> %s", awake ? "AWAKE" : "SLEEP");
}

bool drv_drv8212_awake(void)
{
    return drv_awake;
}
