/*
 * DRV8212P motor driver — sleep pin control.
 *
 * Controls P1.06 (GPIO1, pin 6) — ~SLEEP pin.
 * LOW  = on (driver enabled)
 * HIGH = off (driver sleeping)
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

    /* Start LOW = driver on */
    ret = gpio_pin_configure(DRV8212_SLEEP_PORT, DRV8212_SLEEP_PIN,
                              GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Failed to configure DRV8212 ~SLEEP (P1.06): %d", ret);
        return ret;
    }

    /* Read-back verification */
    int level = gpio_pin_get(DRV8212_SLEEP_PORT, DRV8212_SLEEP_PIN);
    LOG_INF("DRV8212 ~SLEEP (P1.06): %s (read-back=%d)",
            level > 0 ? "HIGH (off)" : "LOW (on)", level);
    if (level > 0) {
        LOG_WRN("DRV8212 ~SLEEP read-back is HIGH — driver unexpectedly off");
    }

    return 0;
}
