/*
 * MAX5419LETA digital potentiometer driver.
 *
 * Controls the STBB1-APUR DCDC feedback network via I2C20 at address 0x28.
 * The MAX5419 is a 200 kΩ, 256-tap potentiometer.
 *
 * STBB1-APUR output voltage formula:
 *   Vout = 0.5 × (100 / R_HW_kΩ + 1)
 *   R_HW = 200 × (255 − tap) / 255   (kΩ)
 *
 * Solving for tap given Vout:
 *   tap = 255 − 127.5 / (2 × Vout − 1)
 *
 * NOTE: The MAX5419 is write-only (the I²C 8th bit is NOP/W, not R/W).
 * Read-back via I²C is not supported. Verify settings by measuring Vout.
 */

#include "max5419.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(max5419, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/*  I2C                                                                       */
/* -------------------------------------------------------------------------- */

#define MAX5419_I2C  DEVICE_DT_GET(DT_NODELABEL(i2c20))

/*
 * Command byte encoding (datasheet Table 2):
 *   Bits:  TX  NV  V  R3 R2 R1 R0
 *   TX  = transfer trigger
 *   NV  = non-volatile register select
 *   V   = volatile register select
 *   R*  = reserved (set to 0001 per datasheet)
 */
#define CMD_VREG        0x11  /* Write volatile register, update wiper now */
#define CMD_NVREG       0x21  /* Write non-volatile register (no wiper update) */
#define CMD_NV2V        0x31  /* Copy NV → volatile (restore saved wiper) */
#define CMD_V2NV        0x29  /* Copy volatile → NV (save wiper permanently) */

/*
 * Output-voltage safety limit.
 *
 * The digipot is the BOTTOM leg of the STBB1-APUR feedback divider, so
 * raising the tap lowers R_HW and raises Vout — tap = 255 shorts FB to
 * GND and the converter boosts toward its 5.5 V maximum into the motor.
 * Cap all requests well below that.
 */
#define VOUT_MAX_V      4.2f

/* Highest tap that keeps Vout <= VOUT_MAX_V: tap = 255 - 127.5/(2V - 1) */
#define TAP_MAX         237

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

static const struct device *i2c_dev = MAX5419_I2C;

int max5419_init(void)
{
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C20 not ready");
        return -ENODEV;
    }

    /*
     * Probe by writing a 0-byte transaction (address-only).
     * We cannot use reads because the MAX5419 NOP/W bit means
     * "no-operation", not "read".
     */
    int ret = i2c_write(i2c_dev, NULL, 0, MAX5419_I2C_ADDR);
    if (ret < 0) {
        LOG_ERR("MAX5419 @0x%02X not responding: %d", MAX5419_I2C_ADDR, ret);
        return ret;
    }

    LOG_INF("MAX5419LETA found at 0x%02X", MAX5419_I2C_ADDR);
    return 0;
}

int max5419_set_tap(uint8_t tap)
{
    if (tap > TAP_MAX) {
        LOG_ERR("Tap %u rejected: exceeds safe limit %u (Vout > %d.%d V)",
                tap, TAP_MAX,
                (int)VOUT_MAX_V, (int)(VOUT_MAX_V * 10.0f) % 10);
        return -EINVAL;
    }

    uint8_t buf[2] = { CMD_VREG, tap };
    int ret = i2c_write(i2c_dev, buf, sizeof(buf), MAX5419_I2C_ADDR);
    if (ret < 0) {
        LOG_ERR("MAX5419 set tap %u failed: %d", tap, ret);
    }
    return ret;
}

int max5419_set_voltage(float voltage)
{
    float den = 2.0f * voltage - 1.0f;
    if (den <= 0.0f) {
        LOG_ERR("Voltage too low (requires V > 0.5)");
        return -EINVAL;
    }
    if (voltage > VOUT_MAX_V) {
        LOG_ERR("Voltage request rejected: exceeds %d.%d V limit",
                (int)VOUT_MAX_V, (int)(VOUT_MAX_V * 10.0f) % 10);
        return -EINVAL;
    }

    float tap_f = 255.0f - 127.5f / den;

    /* Clamp to valid wiper range */
    if (tap_f < 0.0f) {
        tap_f = 0.0f;
    } else if (tap_f > (float)TAP_MAX) {
        tap_f = (float)TAP_MAX;
    }

    uint8_t tap = (uint8_t)(tap_f + 0.5f);

    /* Avoid %f — Zephyr printk doesn't support float formatting */
    int v_int = (int)voltage;
    int v_dec = (int)((voltage - (float)v_int) * 100.0f + 0.5f);
    LOG_INF("Target %d.%02d V -> tap %u", v_int, v_dec, tap);

    return max5419_set_tap(tap);
}
