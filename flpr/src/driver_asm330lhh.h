#ifndef DRIVER_ASM330LHH_H
#define DRIVER_ASM330LHH_H

#include <zephyr/kernel.h>

/** One raw sample from the ASM330LHHTR (sensor LSB, no conversion —
 *  scale on the consumer side from the configured full-scale ranges).
 */
struct asm330lhh_raw {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw;     /* (raw/256)+25 = deg C */
};

/**
 * @brief Initialize the ASM330LHHTR IMU on spi21 (CS = P1.09).
 *
 * Verifies WHO_AM_I (dummy-first-read + retries), enables
 * block-data-update, disables the sensor's unused I2C interface and
 * arms the DRDY interrupt (INT1 -> P1.04).  Sampling is left OFF —
 * call drv_asm330lhh_configure() to start it.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_asm330lhh_init(void);

/**
 * @brief (Re)configure sampling.  Callable at runtime.
 *
 * @param odr_code  ASM330 ODR register code (IMU_ODR_* in
 *                  common/imu_shared.h): 1=12.5 Hz .. 10=6.66 kHz.
 * @param content   IMU_CONTENT_ACCEL / IMU_CONTENT_GYRO bits; the
 *                  disabled sensor is powered down.  Must not be 0.
 * @param accel_fs  0=±2g 1=±4g 2=±8g 3=±16g
 * @param gyro_fs   0=±250 1=±500 2=±1000 3=±2000 dps
 * @return 0 on success, -EINVAL on bad arguments, other negative
 *         errno on SPI failure.
 */
int drv_asm330lhh_configure(uint8_t odr_code, uint8_t content,
                            uint8_t accel_fs, uint8_t gyro_fs);

/**
 * @brief Read one raw sample (temp + gyro + accel burst).
 *
 * @param data  Filled with raw sensor LSB values.
 * @return 0 on success, negative errno on failure.
 */
int drv_asm330lhh_read(struct asm330lhh_raw *data);

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

/** @brief Whether the IMU initialized successfully this boot. */
bool drv_asm330lhh_ok(void);

/** @brief Raw WHO_AM_I value read during init (0 if never read). */
uint8_t drv_asm330lhh_whoami(void);

#endif /* DRIVER_ASM330LHH_H */
