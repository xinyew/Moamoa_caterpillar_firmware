/*
 * PWM driver — controls DRV8212P IN1/IN2 via PWM20.
 *
 * P1.07 = PWM_OUT0 (IN1), P1.08 = PWM_OUT1 (IN2).
 *
 * DRV8212P H-bridge truth table:
 *   IN1=PWM, IN2=LOW  → forward drive
 *   IN1=LOW, IN2=PWM  → reverse drive
 *   IN1=HIGH, IN2=HIGH → brake
 *   IN1=LOW, IN2=LOW  → coast
 *
 * nRF54 PWM clock: 16 MHz
 */

#include "driver_pwm.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_pwm, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/*  PWM device                                                                */
/* -------------------------------------------------------------------------- */

static const struct device *pwm_dev =
    DEVICE_DT_GET(DT_NODELABEL(pwm20));

/* Default: 113 Hz, 0 % duty — boot in coast; main starts the motor */
static uint32_t period_ns = 8849558U;
static uint8_t  duty_pct[2] = {0, 0};   /* CH0 = IN1, CH1 = IN2 */

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */

/** Apply the current period + per-channel duty to both channels. */
static int pwm_apply(void)
{
    for (uint8_t ch = 0; ch < 2; ch++) {
        uint32_t pulse =
            (uint32_t)((uint64_t)period_ns * duty_pct[ch] / 100U);
        int ret = pwm_set(pwm_dev, ch, period_ns, pulse, 0);
        if (ret < 0) {
            return ret;
        }
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

int drv_pwm_init(void)
{
    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("PWM20 not ready");
        return -ENODEV;
    }

    int ret = pwm_apply();
    if (ret < 0) {
        LOG_ERR("PWM init failed: %d", ret);
        return ret;
    }

    LOG_INF("PWM20: %u Hz, IN1(P1.07) %u %%, IN2(P1.08) %u %%",
            1000000000U / period_ns, duty_pct[0], duty_pct[1]);
    return 0;
}

int drv_pwm_set_duty(uint8_t channel, uint8_t percent)
{
    if (channel > 1 || percent > 100) {
        return -EINVAL;
    }

    duty_pct[channel] = percent;
    return pwm_apply();
}

int drv_pwm_set_frequency(uint16_t hz)
{
    if (hz < DRV_PWM_FREQ_MIN_HZ || hz > DRV_PWM_FREQ_MAX_HZ) {
        LOG_ERR("Frequency %u Hz out of range (%u–%u)",
                hz, DRV_PWM_FREQ_MIN_HZ, DRV_PWM_FREQ_MAX_HZ);
        return -EINVAL;
    }

    period_ns = 1000000000U / hz;

    int ret = pwm_apply();
    if (ret < 0) {
        LOG_ERR("PWM freq set %u Hz failed: %d", hz, ret);
        return ret;
    }

    LOG_INF("PWM20: %u Hz, IN1 %u %%, IN2 %u %%",
            hz, duty_pct[0], duty_pct[1]);
    return 0;
}
