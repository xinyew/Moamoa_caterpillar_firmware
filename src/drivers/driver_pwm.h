#ifndef DRIVER_PWM_H
#define DRIVER_PWM_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize PWM20 at 113 Hz, 50 % duty on both channels.
 *
 * P1.07 = CH0 (IN1), P1.08 = CH1 (IN2).
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_pwm_init(void);

/**
 * @brief Set duty cycle for a channel.
 *
 * @param channel  0 (IN1) or 1 (IN2).
 * @param percent  Duty cycle 0–100.
 * @return 0 on success.
 */
int drv_pwm_set_duty(uint8_t channel, uint8_t percent);

#endif /* DRIVER_PWM_H */
