/*
 * Status LED driver — blue LED on P0.01 (active-high, 470R series).
 *
 * Runs a triple-flash pattern (3 × 3 ms blinks per second) on a kernel
 * timer, so the blink cadence is independent of main-loop pacing.
 */

#include "driver_led.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_led, LOG_LEVEL_INF);

#define LED_PORT  DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define LED_PIN   1

/* -------------------------------------------------------------------------- */
/*  Blink patterns                                                            */
/*                                                                            */
/*  After a reset: 3 × 3 ms flashes per second for BOOT_PATTERN_REPEATS       */
/*  seconds — a glanceable "this board just reset" marker.                    */
/*  Running: a single 3 ms flash per second.                                  */
/* -------------------------------------------------------------------------- */

#define FLASH_ON_MS    3
#define FLASH_GAP_MS   100
#define PERIOD_MS      1000

/* How many 1 s reset-marker periods to play after boot */
#define BOOT_PATTERN_REPEATS  3

struct blink_step {
    uint8_t  on;
    uint16_t ms;
};

static const struct blink_step boot_pattern[] = {
    { 1, FLASH_ON_MS },
    { 0, FLASH_GAP_MS },
    { 1, FLASH_ON_MS },
    { 0, FLASH_GAP_MS },
    { 1, FLASH_ON_MS },
    { 0, PERIOD_MS - 3 * FLASH_ON_MS - 2 * FLASH_GAP_MS },
};

static const struct blink_step run_pattern[] = {
    { 1, FLASH_ON_MS },
    { 0, PERIOD_MS - FLASH_ON_MS },
};

static const struct blink_step *pattern = boot_pattern;
static size_t pattern_len = ARRAY_SIZE(boot_pattern);
static size_t step_idx;
static uint8_t boot_periods_left;
static bool led_enabled = true;

static void blink_timer_fn(struct k_timer *timer)
{
    step_idx++;
    if (step_idx >= pattern_len) {
        step_idx = 0;
        if (boot_periods_left > 0 && --boot_periods_left == 0) {
            pattern = run_pattern;
            pattern_len = ARRAY_SIZE(run_pattern);
        }
    }
    gpio_pin_set_raw(LED_PORT, LED_PIN, pattern[step_idx].on);
    k_timer_start(timer, K_MSEC(pattern[step_idx].ms), K_NO_WAIT);
}

static K_TIMER_DEFINE(blink_timer, blink_timer_fn, NULL);

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

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

void drv_led_blink_start(void)
{
    pattern = boot_pattern;
    pattern_len = ARRAY_SIZE(boot_pattern);
    boot_periods_left = BOOT_PATTERN_REPEATS;
    step_idx = 0;
    gpio_pin_set_raw(LED_PORT, LED_PIN, pattern[0].on);
    k_timer_start(&blink_timer, K_MSEC(pattern[0].ms), K_NO_WAIT);
}

void drv_led_set(bool on)
{
    k_timer_stop(&blink_timer);
    gpio_pin_set_raw(LED_PORT, LED_PIN, on ? 1 : 0);
}

void drv_led_set_enabled(bool enable)
{
    if (enable == led_enabled) {
        return;
    }
    led_enabled = enable;

    if (enable) {
        /* Resume the running heartbeat (no reset marker) */
        pattern = run_pattern;
        pattern_len = ARRAY_SIZE(run_pattern);
        boot_periods_left = 0;
        step_idx = 0;
        gpio_pin_set_raw(LED_PORT, LED_PIN, pattern[0].on);
        k_timer_start(&blink_timer, K_MSEC(pattern[0].ms), K_NO_WAIT);
        LOG_INF("LED heartbeat ON");
    } else {
        k_timer_stop(&blink_timer);
        gpio_pin_set_raw(LED_PORT, LED_PIN, 0);
        LOG_INF("LED heartbeat OFF");
    }
}

bool drv_led_enabled(void)
{
    return led_enabled;
}

void drv_led_toggle(void)
{
    gpio_pin_toggle(LED_PORT, LED_PIN);
}
