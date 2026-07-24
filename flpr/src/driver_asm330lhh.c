/*
 * ASM330LHHTR 6-axis IMU — register-level driver.
 *
 * Communicates over SPI (spi21, mode 3, 8 MHz):
 *   SCLK P1.10, MOSI→SDI P1.13, MISO←SDO P1.14, CS P1.09 (GPIO, active-low)
 * ODR / full-scale / content are runtime-configurable
 * (drv_asm330lhh_configure); output is raw sensor LSB.
 */

#include "driver_asm330lhh.h"
#include "common/imu_shared.h"

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

/* CTRL3_C: BDU (bit 6) + IF_INC (bit 2) — latch L/H, auto-increment addr */
#define CTRL3_C_VAL     0x44

/* CTRL4_C: I2C_disable (bit 2) — SPI-only, ignore noise on the I2C pads */
#define CTRL4_C_VAL     0x04

/* INT1_CTRL bits: 0 = accel DRDY, 1 = gyro DRDY */
#define INT1_DRDY_XL    0x01
#define INT1_DRDY_G     0x02

/* INT1 pad wired to P1.04 (active-high) */
#define INT1_PORT       DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define INT1_PIN        4

/* FS_XL[3:2] register bits, indexed by cfg 0..3 = ±2/±4/±8/±16 g
 * (register coding is non-monotonic: 00=2g 01=16g 10=4g 11=8g)
 */
static const uint8_t fs_xl_bits[4] = { 0x0, 0x2, 0x3, 0x1 };

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

    /* Enable block-data-update */
    ret = reg_write(REG_CTRL3_C, CTRL3_C_VAL);
    if (ret < 0) { LOG_ERR("CTRL3_C: %d", ret); return ret; }

    /* SPI-only from here on — shut the unused I2C interface off */
    ret = reg_write(REG_CTRL4_C, CTRL4_C_VAL);
    if (ret < 0) { LOG_ERR("CTRL4_C: %d", ret); return ret; }

    ret = drdy_int_setup();
    if (ret < 0) { LOG_ERR("INT1 GPIO setup: %d", ret); return ret; }

    /* Sampling stays off (ODR 0) until drv_asm330lhh_configure() */
    imu_ok = true;
    return 0;
}

int drv_asm330lhh_configure(uint8_t odr_code, uint8_t content,
                            uint8_t accel_fs, uint8_t gyro_fs)
{
    int ret;

    if (odr_code < IMU_ODR_12HZ5 || odr_code > IMU_ODR_6660HZ ||
        (content & ~(IMU_CONTENT_ACCEL | IMU_CONTENT_GYRO)) != 0 ||
        accel_fs > 3 || gyro_fs > 3) {
        return -EINVAL;
    }

    /* content == 0: power the whole sensor down (on-demand sampling) */
    bool accel_on = (content & IMU_CONTENT_ACCEL) != 0;
    bool gyro_on  = (content & IMU_CONTENT_GYRO) != 0;

    /* Disabled sensor gets ODR 0000 = power-down */
    uint8_t ctrl1 = accel_on ? (uint8_t)((odr_code << 4) |
                                         (fs_xl_bits[accel_fs] << 2)) : 0;
    uint8_t ctrl2 = gyro_on  ? (uint8_t)((odr_code << 4) |
                                         (gyro_fs << 2)) : 0;

    /* DRDY source: gyro when enabled (accel DRDY otherwise); nothing
     * routed when powered down.  With a shared ODR both assert at the
     * same rate.
     */
    uint8_t int1 = gyro_on ? INT1_DRDY_G : (accel_on ? INT1_DRDY_XL : 0);

    ret = reg_write(REG_CTRL1_XL, ctrl1);
    if (ret == 0) ret = reg_write(REG_CTRL2_G, ctrl2);
    if (ret == 0) ret = reg_write(REG_INT1_CTRL, int1);
    if (ret < 0) {
        LOG_ERR("configure: %d", ret);
        return ret;
    }

    /* Clear any already-latched DRDY so the next edge fires, and drop
     * a stale wakeup from the previous config.
     */
    struct asm330lhh_raw scratch;
    (void)drv_asm330lhh_read(&scratch);
    k_sem_reset(&drdy_sem);

    LOG_INF("IMU cfg: odr_code=%u content=0x%x fs_xl=%u fs_g=%u",
            odr_code, content, accel_fs, gyro_fs);
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

int drv_asm330lhh_read(struct asm330lhh_raw *data)
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

    data->temp_raw = sign_extend_pair(raw[0], raw[1]);
    data->gx = sign_extend_pair(raw[2],  raw[3]);
    data->gy = sign_extend_pair(raw[4],  raw[5]);
    data->gz = sign_extend_pair(raw[6],  raw[7]);
    data->ax = sign_extend_pair(raw[8],  raw[9]);
    data->ay = sign_extend_pair(raw[10], raw[11]);
    data->az = sign_extend_pair(raw[12], raw[13]);

    return 0;
}
