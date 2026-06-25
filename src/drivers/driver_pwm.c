/*
 * PWM driver — controls DRV8212P IN1/IN2 via PWM20.
 *
 * P1.07 = PWM_OUT0 (IN1), P1.08 = PWM_OUT1 (IN2).
 * Base frequency: 113 Hz, 50 % duty square wave.
 *
 * nRF54 PWM clock: 16 MHz
 *   f_pwm = 16e6 / (prescaler × top) = 113 Hz
 *   prescaler = 8, top = 17699  →  f ≈ 113.01 Hz  (0.01 % error)
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

/* 113 Hz → 8,849,558 ns period */
#define PWM_PERIOD_NS       8849558U
#define PWM_PULSE_10PCT_NS  (PWM_PERIOD_NS * 10U / 100U)  /* 10 % to avoid inrush */
#define PWM_PULSE_50PCT_NS  (PWM_PERIOD_NS / 2U)           /* 50 % full power */

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

int drv_pwm_init(void)
{
    int ret;

    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("PWM20 not ready");
        return -ENODEV;
    }

    /*
     * CH0 — IN1 (P1.07): 113 Hz, 50 % PWM
     * CH1 — IN2 (P1.08): LOW (fixed 0)
     *
     * DRV8212P H-bridge truth table:
     *   IN1=PWM, IN2=LOW  → forward drive
     *   IN1=LOW, IN2=PWM  → reverse drive
     *   IN1=HIGH, IN2=HIGH → brake
     *   IN1=LOW, IN2=LOW  → coast
     */
    ret = pwm_set(pwm_dev, 0, PWM_PERIOD_NS, PWM_PULSE_50PCT_NS, 0);
    if (ret < 0) {
        LOG_ERR("PWM CH0 set failed: %d", ret);
        return ret;
    }

    ret = pwm_set(pwm_dev, 1, PWM_PERIOD_NS, 0, 0);
    if (ret < 0) {
        LOG_ERR("PWM CH1 set failed: %d", ret);
        return ret;
    }

    LOG_INF("PWM20: 113 Hz 50 %% IN1(P1.07)=PWM, IN2(P1.08)=LOW → forward");
    return 0;
}

int drv_pwm_set_duty(uint8_t channel, uint8_t percent)
{
    if (channel > 1) {
        return -EINVAL;
    }
    if (percent > 100) {
        percent = 100;
    }

    uint32_t pulse = (uint32_t)((uint64_t)PWM_PERIOD_NS * percent / 100U);
    return pwm_set(pwm_dev, channel, PWM_PERIOD_NS, pulse, 0);
}
