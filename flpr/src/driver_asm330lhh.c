/*
 * ASM330LHHTR 6-axis IMU — register-level driver.
 *
 * Communicates over SPI (spi21, mode 3, 8 MHz):
 *   SCLK P1.10, MOSI→SDI P1.13, MISO←SDO P1.14, CS P1.09 (GPIO, active-low)
 * Accelerometer: 12.5 Hz, ±2 g
 * Gyroscope:     12.5 Hz, ±250 dps
 */

#include "driver_asm330lhh.h"

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_asm330lhh, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------- */
/*  SPI bus                                                                   */
/* -------------------------------------------------------------------------- */

#define IMU_SPI         DEVICE_DT_GET(DT_NODELABEL(spi21))

/* CS on P1.09 — driven manually so the bus config stays trivial */
#define CS_PORT         DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define CS_PIN          9

/* Mode 3 (CPOL=1, CPHA=1); 8 MHz is SPIM21's max, sensor allows 10 MHz.
 * Non-const: init probes mode/speed combinations during bring-up.
 */
static struct spi_config spi_cfg = {
    .frequency = 8000000U,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
};

/* -------------------------------------------------------------------------- */
/*  Register map                                                              */
/* -------------------------------------------------------------------------- */

#define REG_INT1_CTRL   0x0D
#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_CTRL4_C     0x13
#define REG_OUT_TEMP_L  0x20

#define WHO_AM_I_EXPECT 0x6B

/* CTRL1_XL: 12.5 Hz ODR, ±2 g */
#define CTRL1_XL_VAL    0x10

/* CTRL2_G: 12.5 Hz ODR, ±250 dps */
#define CTRL2_G_VAL     0x10

/* CTRL3_C: BDU (bit 6) + IF_INC (bit 2) — latch L/H, auto-increment addr */
#define CTRL3_C_VAL     0x44

/* CTRL4_C: I2C_disable (bit 2) — SPI-only, ignore noise on the I2C pads */
#define CTRL4_C_VAL     0x04

/* INT1_CTRL: route gyro data-ready to the INT1 pad (bit 1) */
#define INT1_CTRL_VAL   0x02

/* INT1 pad wired to P1.04 (active-high) */
#define INT1_PORT       DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define INT1_PIN        4

/* Sensitivity */
#define ACCEL_SENS      61      /* 0.061 mg/LSB  * 1000 */
#define GYRO_SENS       875     /* 8.75 mdps/LSB * 100  */

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */

/* SPI first byte: bit 7 = read flag, bits 6..0 = register address */
#define SPI_READ_BIT    0x80

static int spi_xfer(const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
    gpio_pin_set_raw(CS_PORT, CS_PIN, 0);
    int ret = spi_transceive(IMU_SPI, &spi_cfg, tx, rx);
    gpio_pin_set_raw(CS_PORT, CS_PIN, 1);
    return ret;
}

static int reg_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    const struct spi_buf tb = { .buf = tx, .len = sizeof(tx) };
    const struct spi_buf_set txs = { .buffers = &tb, .count = 1 };

    return spi_xfer(&txs, NULL);
}

static int reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t addr = (uint8_t)(reg | SPI_READ_BIT);
    const struct spi_buf tb = { .buf = &addr, .len = 1 };
    const struct spi_buf_set txs = { .buffers = &tb, .count = 1 };
    struct spi_buf rb[2] = {
        { .buf = NULL, .len = 1 },   /* discard byte clocked with the address */
        { .buf = buf,  .len = len },
    };
    const struct spi_buf_set rxs = { .buffers = rb, .count = 2 };

    return spi_xfer(&txs, &rxs);
}

static int16_t sign_extend_pair(uint8_t l, uint8_t h)
{
    return (int16_t)(((uint16_t)h << 8) | l);
}

/* -------------------------------------------------------------------------- */
/*  Data-ready interrupt (INT1 → P1.04)                                       */
/* -------------------------------------------------------------------------- */

static K_SEM_DEFINE(drdy_sem, 0, 1);
static struct gpio_callback drdy_cb;
static bool imu_ok;
static uint8_t last_whoami;

static void drdy_isr(const struct device *port, struct gpio_callback *cb,
                     uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&drdy_sem);
}

static int drdy_int_setup(void)
{
    int ret;

    ret = gpio_pin_configure(INT1_PORT, INT1_PIN, GPIO_INPUT);
    if (ret == 0) {
        gpio_init_callback(&drdy_cb, drdy_isr, BIT(INT1_PIN));
        ret = gpio_add_callback(INT1_PORT, &drdy_cb);
    }
    if (ret == 0) {
        ret = gpio_pin_interrupt_configure(INT1_PORT, INT1_PIN,
                                           GPIO_INT_EDGE_RISING);
    }
    return ret;
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

int drv_asm330lhh_init(void)
{
    uint8_t whoami = 0;
    int ret;

    if (!device_is_ready(IMU_SPI)) {
        LOG_ERR("SPI21 not ready");
        return -ENODEV;
    }
    if (!device_is_ready(CS_PORT)) {
        LOG_ERR("GPIO1 not ready");
        return -ENODEV;
    }

    /* CS idle-high before the first clock edge */
    ret = gpio_pin_configure(CS_PORT, CS_PIN, GPIO_OUTPUT_HIGH);
    if (ret < 0) {
        LOG_ERR("CS (P1.09) config failed: %d", ret);
        return ret;
    }

    /* The first SPI transaction after power-up returns garbage: the
     * sensor latches into SPI mode on the first CS cycle, and the
     * mode-3 clock idle level isn't established until the first
     * transfer.  Issue a discarded dummy read, then verify WHO_AM_I
     * with retries.  (The pre-retry firmware read once and gave up —
     * that alone made a working IMU look dead.)
     */
    (void)reg_read(REG_WHO_AM_I, &whoami, 1);

    ret = -EIO;
    for (int attempt = 0; attempt < 3; attempt++) {
        whoami = 0;
        ret = reg_read(REG_WHO_AM_I, &whoami, 1);
        if (ret == 0 && whoami == WHO_AM_I_EXPECT) {
            break;
        }
        LOG_WRN("WHO_AM_I attempt %d: ret=%d val=0x%02X",
                attempt + 1, ret, whoami);
        ret = -EIO;
        k_msleep(2);
    }
    if (ret < 0) {
        LOG_ERR("Bad WHO_AM_I (expected 0x%02X)", WHO_AM_I_EXPECT);
        return ret;
    }
    last_whoami = whoami;
    LOG_INF("ASM330LHHTR on spi21 (WHO_AM_I=0x%02X)", whoami);

    /* Configure accelerometer */
    ret = reg_write(REG_CTRL1_XL, CTRL1_XL_VAL);
    if (ret < 0) { LOG_ERR("CTRL1_XL: %d", ret); return ret; }

    /* Configure gyroscope */
    ret = reg_write(REG_CTRL2_G, CTRL2_G_VAL);
    if (ret < 0) { LOG_ERR("CTRL2_G: %d", ret); return ret; }

    /* Enable block-data-update */
    ret = reg_write(REG_CTRL3_C, CTRL3_C_VAL);
    if (ret < 0) { LOG_ERR("CTRL3_C: %d", ret); return ret; }

    /* SPI-only from here on — shut the unused I2C interface off */
    ret = reg_write(REG_CTRL4_C, CTRL4_C_VAL);
    if (ret < 0) { LOG_ERR("CTRL4_C: %d", ret); return ret; }

    /* Route gyro data-ready to INT1 and arm the P1.04 interrupt */
    ret = reg_write(REG_INT1_CTRL, INT1_CTRL_VAL);
    if (ret < 0) { LOG_ERR("INT1_CTRL: %d", ret); return ret; }

    ret = drdy_int_setup();
    if (ret < 0) { LOG_ERR("INT1 GPIO setup: %d", ret); return ret; }

    /* Clear any already-latched DRDY so the first rising edge fires */
    struct asm330lhh_data scratch;
    (void)drv_asm330lhh_read(&scratch);

    LOG_INF("IMU: XL 12.5Hz ±2g, G 12.5Hz ±250dps, BDU, DRDY→INT1");
    imu_ok = true;
    return 0;
}

bool drv_asm330lhh_ok(void)
{
    return imu_ok;
}

uint8_t drv_asm330lhh_whoami(void)
{
    return last_whoami;
}

int drv_asm330lhh_wait_data(int32_t timeout_ms)
{
    if (k_sem_take(&drdy_sem, K_MSEC(timeout_ms)) != 0) {
        return -EAGAIN;
    }
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
    data->gyro_x = (int32_t)sign_extend_pair(raw[2], raw[3]) * GYRO_SENS / 100;
    data->gyro_y = (int32_t)sign_extend_pair(raw[4], raw[5]) * GYRO_SENS / 100;
    data->gyro_z = (int32_t)sign_extend_pair(raw[6], raw[7]) * GYRO_SENS / 100;

    /* Accelerometer (raw[8]..raw[13]) */
    data->accel_x = (int16_t)((int32_t)sign_extend_pair(raw[8],  raw[9])  * ACCEL_SENS / 1000);
    data->accel_y = (int16_t)((int32_t)sign_extend_pair(raw[10], raw[11]) * ACCEL_SENS / 1000);
    data->accel_z = (int16_t)((int32_t)sign_extend_pair(raw[12], raw[13]) * ACCEL_SENS / 1000);

    return 0;
}
