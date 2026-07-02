#ifndef DRIVER_PWM_H
#define DRIVER_PWM_H

#include <zephyr/kernel.h>

/*
 * Frequency limits.  The upper bound is the application's motor range;
 * the lower bound is set by the nRF54 PWM peripheral: 16 MHz clock,
 * max prescaler /128 (125 kHz) with a 15-bit counter gives a maximum
 * period of ~262 ms, so ~4 Hz is the slowest achievable frequency.
 */
#define DRV_PWM_FREQ_MIN_HZ  4
#define DRV_PWM_FREQ_MAX_HZ  1000

/**
 * @brief Initialize PWM20 at 113 Hz, 0 % duty on both channels (coast).
 *
 * P1.07 = CH0 (IN1), P1.08 = CH1 (IN2).
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_pwm_init(void);

/**
 * @brief Set duty cycle for one channel.
 *
 * DRV8212P truth table: CH0=PWM/CH1=0 → forward, CH0=0/CH1=PWM →
 * reverse, both 0 → coast.
 *
 * @param channel  0 (IN1) or 1 (IN2).
 * @param percent  Duty cycle 0–100.
 * @return 0 on success, negative errno on failure.
 */
int drv_pwm_set_duty(uint8_t channel, uint8_t percent);

/**
 * @brief Change the PWM frequency on the fly (keeps current duty ratios).
 *
 * @param hz  Frequency in Hz (DRV_PWM_FREQ_MIN_HZ–DRV_PWM_FREQ_MAX_HZ).
 * @return 0 on success, -EINVAL if out of range.
 */
int drv_pwm_set_frequency(uint16_t hz);

#endif /* DRIVER_PWM_H */
