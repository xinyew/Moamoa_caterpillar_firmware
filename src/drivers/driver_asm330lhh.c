/*
 * ASM330LHHTR 6-axis IMU — register-level driver.
 *
 * Communicates over I2C20 at address 0x6A (SA0 = 0).
 * Accelerometer: 12.5 Hz, ±2 g
 * Gyroscope:     12.5 Hz, ±250 dps
 */

#include "driver_asm330lhh.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_asm330lhh, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------- */
/*  I2C                                                                       */
/* -------------------------------------------------------------------------- */

#define IMU_I2C         DEVICE_DT_GET(DT_NODELABEL(i2c20))
#define IMU_ADDR        0x6A
#define MAX5419_ADDR    0x28

/* -------------------------------------------------------------------------- */
/*  Register map                                                              */
/* -------------------------------------------------------------------------- */

#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_OUT_TEMP_L  0x20

#define WHO_AM_I_EXPECT 0x6B

/* CTRL1_XL: 12.5 Hz ODR, ±2 g */
#define CTRL1_XL_VAL    0x10

/* CTRL2_G: 12.5 Hz ODR, ±250 dps */
#define CTRL2_G_VAL     0x10

/* CTRL3_C: BDU (bit 6) + IF_INC (bit 2) — latch L/H, auto-increment addr */
#define CTRL3_C_VAL     0x44

/* Sensitivity */
#define ACCEL_SENS      61      /* 0.061 mg/LSB  * 1000 */
#define GYRO_SENS       875     /* 8.75 mdps/LSB * 100  */

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */

static int reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write(IMU_I2C, buf, 2, IMU_ADDR);
}

static int reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read(IMU_I2C, IMU_ADDR, &reg, 1, buf, len);
}

/** Quick probe: read 1 byte.  Returns 0 if the device ACKs its address. */
static int i2c_ping(uint8_t addr)
{
    uint8_t dummy;
    return i2c_read(IMU_I2C, &dummy, 1, addr);
}

static int16_t sign_extend_pair(uint8_t l, uint8_t h)
{
    return (int16_t)(((uint16_t)h << 8) | l);
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

int drv_asm330lhh_init(void)
{
    uint8_t whoami = 0;
    int ret;

    if (!device_is_ready(IMU_I2C)) {
        LOG_ERR("I2C20 not ready");
        return -ENODEV;
    }

    /* Probe MAX5419LETA (digipot) at 0x28 */
    ret = i2c_ping(MAX5419_ADDR);
    LOG_INF("MAX5419LETA @0x%02X: %s (%d)", MAX5419_ADDR,
            ret == 0 ? "ACK" : "NAK", ret);

    /* Probe ASM330LHHTR IMU at 0x6A */
    ret = i2c_ping(IMU_ADDR);
    if (ret != 0) {
        LOG_ERR("ASM330LHHTR @0x%02X: NAK (%d)", IMU_ADDR, ret);
        return ret;
    }
    LOG_INF("ASM330LHHTR @0x%02X: ACK", IMU_ADDR);

    /* Read WHO_AM_I to confirm it's the right chip */
    ret = reg_read(REG_WHO_AM_I, &whoami, 1);
    if (ret < 0) {
        LOG_ERR("WHO_AM_I read failed: %d", ret);
        return ret;
    }
    if (whoami != WHO_AM_I_EXPECT) {
        LOG_ERR("Bad WHO_AM_I: 0x%02X (expected 0x%02X)",
                whoami, WHO_AM_I_EXPECT);
        return -EIO;
    }
    LOG_INF("ASM330LHHTR at 0x%02X (WHO_AM_I=0x%02X)", IMU_ADDR, whoami);

    /* Configure accelerometer */
    ret = reg_write(REG_CTRL1_XL, CTRL1_XL_VAL);
    if (ret < 0) { LOG_ERR("CTRL1_XL: %d", ret); return ret; }

    /* Configure gyroscope */
    ret = reg_write(REG_CTRL2_G, CTRL2_G_VAL);
    if (ret < 0) { LOG_ERR("CTRL2_G: %d", ret); return ret; }

    /* Enable block-data-update */
    ret = reg_write(REG_CTRL3_C, CTRL3_C_VAL);
    if (ret < 0) { LOG_ERR("CTRL3_C: %d", ret); return ret; }

    LOG_INF("IMU: XL 12.5Hz ±2g, G 12.5Hz ±250dps, BDU");
    return 0;
}

int drv_asm330lhh_read(struct asm330lhh_data *data)
{
    uint8_t raw[14];  /* TEMP_L..OUTZ_H_A */
    int ret;

    if (!data) {
        return -EINVAL;
    }

    ret = reg_read(REG_OUT_TEMP_L, raw, sizeof(raw));
    if (ret < 0) {
        LOG_ERR("IMU read: %d", ret);
        return ret;
    }

    /* Temperature: (raw / 256) + 25, scaled to millideg C */
    int16_t raw_temp = sign_extend_pair(raw[0], raw[1]);
    data->temp = ((int32_t)raw_temp * 1000) / 256 + 25000;

    /* Gyroscope (raw[2]..raw[7]) */
    data->gyro_x = (int16_t)((int32_t)sign_extend_pair(raw[2], raw[3]) * GYRO_SENS / 100);
    data->gyro_y = (int16_t)((int32_t)sign_extend_pair(raw[4], raw[5]) * GYRO_SENS / 100);
    data->gyro_z = (int16_t)((int32_t)sign_extend_pair(raw[6], raw[7]) * GYRO_SENS / 100);

    /* Accelerometer (raw[8]..raw[13]) */
    data->accel_x = (int16_t)((int32_t)sign_extend_pair(raw[8],  raw[9])  * ACCEL_SENS / 1000);
    data->accel_y = (int16_t)((int32_t)sign_extend_pair(raw[10], raw[11]) * ACCEL_SENS / 1000);
    data->accel_z = (int16_t)((int32_t)sign_extend_pair(raw[12], raw[13]) * ACCEL_SENS / 1000);

    return 0;
}
