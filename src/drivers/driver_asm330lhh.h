#ifndef DRIVER_ASM330LHH_H
#define DRIVER_ASM330LHH_H

#include <zephyr/kernel.h>

/** Raw sensor data from the ASM330LHHTR. */
struct asm330lhh_data {
    int16_t accel_x;   /* mg */
    int16_t accel_y;   /* mg */
    int16_t accel_z;   /* mg */
    int16_t gyro_x;    /* mdps */
    int16_t gyro_y;    /* mdps */
    int16_t gyro_z;    /* mdps */
    int32_t temp;      /* millideg C */
};

/**
 * @brief Initialize the ASM330LHHTR IMU on I2C20.
 *
 * Verifies WHO_AM_I, configures accelerometer at 12.5 Hz ±2 g and
 * gyroscope at 12.5 Hz ±250 dps with block-data-update enabled.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_asm330lhh_init(void);

/**
 * @brief Read one sample (accel + gyro + temperature).
 *
 * @param data  Pointer to struct to fill with converted values.
 * @return 0 on success, negative errno on failure.
 */
int drv_asm330lhh_read(struct asm330lhh_data *data);

/**
 * @brief Block until the IMU signals data-ready on INT1 (P1.04).
 *
 * Reading the output registers (drv_asm330lhh_read) de-asserts DRDY,
 * so call this once per read.
 *
 * @param timeout_ms  Maximum time to wait, in milliseconds.
 * @return 0 when data is ready, -EAGAIN on timeout.
 */
int drv_asm330lhh_wait_data(int32_t timeout_ms);

#endif /* DRIVER_ASM330LHH_H */
