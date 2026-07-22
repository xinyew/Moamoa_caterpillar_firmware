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

    /* Drive HIGH to wake the driver (nSLEEP de-asserted) */
    ret = gpio_pin_configure(DRV8212_SLEEP_PORT, DRV8212_SLEEP_PIN,
                              GPIO_OUTPUT_HIGH);
    if (ret < 0) {
        LOG_ERR("Failed to configure DRV8212 nSLEEP (P1.06): %d", ret);
        return ret;
    }

    /* Read-back verification */
    int level = gpio_pin_get(DRV8212_SLEEP_PORT, DRV8212_SLEEP_PIN);
    LOG_INF("DRV8212 nSLEEP (P1.06): %s (read-back=%d)",
            level > 0 ? "HIGH (awake)" : "LOW (sleep)", level);
    if (level <= 0) {
        LOG_WRN("DRV8212 nSLEEP is LOW — driver is asleep, motor won't spin");
    }

    return 0;
}

void drv_drv8212_set(bool awake)
{
    gpio_pin_set_raw(DRV8212_SLEEP_PORT, DRV8212_SLEEP_PIN, awake ? 1 : 0);
    LOG_INF("DRV8212 nSLEEP -> %s", awake ? "AWAKE" : "SLEEP");
}
