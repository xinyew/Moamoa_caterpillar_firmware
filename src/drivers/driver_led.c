/*
 * Status LED driver — blue LED on P0.01 (active-high, 470R series).
 *
 * The main loop heartbeat-blinks it (~1 Hz) so a running board is
 * distinguishable from a hung or reset-looping one without RTT.
 */

#include "driver_led.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_led, LOG_LEVEL_INF);

#define LED_PORT  DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define LED_PIN   1

int drv_led_init(void)
{
    if (!device_is_ready(LED_PORT)) {
        LOG_ERR("GPIO0 not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure(LED_PORT, LED_PIN, GPIO_OUTPUT_HIGH);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED (P0.01): %d", ret);
        return ret;
    }

    LOG_INF("Status LED (P0.01) on");
    return 0;
}

void drv_led_set(bool on)
{
    gpio_pin_set_raw(LED_PORT, LED_PIN, on ? 1 : 0);
}

void drv_led_toggle(void)
{
    gpio_pin_toggle(LED_PORT, LED_PIN);
}
