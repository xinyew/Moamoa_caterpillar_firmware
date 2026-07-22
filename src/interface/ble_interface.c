/*
 * BLE interface — GATT server for PWM frequency control.
 *
 * Advertises as "Caterpillar":
 *   0xFFE1 — write: PWM frequency in Hz    (4–1000, u16 LE) → drv_pwm_set_frequency()
 *   0xFFE2 — write: motor rail VDC in mV (750–4200, u16 LE) → max5419_set_voltage()
 *   0xFFE3 — read:  measured VDC in mV (u16 LE, AIN4 sense divider)
 *   0xFFE4 — write: motor rail enable (u8: 0=off, 1=on) → drv_stbb1_apur_set()
 *   0xFFE5 — write: motor driver awake (u8: 0=sleep, 1=awake) → drv_drv8212_set()
 *   0xFFE6 — read:  status packet (24 B LE struct, see on_status_read)
 * Writes accept acknowledged Write Requests as well as
 * Write-Without-Response.
 */

#include "ble_interface.h"

#include "../drivers/driver_pwm.h"
#include "../drivers/max5419.h"
#include "../drivers/driver_vdc_sense.h"
#include "../drivers/driver_stbb1_apur.h"
#include "../drivers/driver_drv8212.h"
#include "../drivers/driver_asm330lhh.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <app_version.h>

/* Boot reset cause captured in main.c */
extern uint32_t app_reset_cause;

LOG_MODULE_REGISTER(ble_if, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/*  UUIDs                                                                     */
/* -------------------------------------------------------------------------- */

/* 16-bit custom vendor UUIDs */
#define BT_UUID_CATERPILLAR_SVC_VAL     0xFFE0
#define BT_UUID_CATERPILLAR_FREQ_VAL    0xFFE1
#define BT_UUID_CATERPILLAR_VOLT_VAL    0xFFE2
#define BT_UUID_CATERPILLAR_VDCMEAS_VAL 0xFFE3
#define BT_UUID_CATERPILLAR_RAIL_VAL    0xFFE4
#define BT_UUID_CATERPILLAR_DRV_VAL     0xFFE5
#define BT_UUID_CATERPILLAR_STATUS_VAL  0xFFE6

#define BT_UUID_CATERPILLAR_SVC  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_SVC_VAL)
#define BT_UUID_CATERPILLAR_FREQ BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_FREQ_VAL)
#define BT_UUID_CATERPILLAR_VOLT BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_VOLT_VAL)
#define BT_UUID_CATERPILLAR_VDCMEAS \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_VDCMEAS_VAL)
#define BT_UUID_CATERPILLAR_RAIL BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_RAIL_VAL)
#define BT_UUID_CATERPILLAR_DRV  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_DRV_VAL)
#define BT_UUID_CATERPILLAR_STATUS \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_STATUS_VAL)

/* -------------------------------------------------------------------------- */
/*  Advertising data                                                          */
/* -------------------------------------------------------------------------- */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "Caterpillar", sizeof("Caterpillar") - 1),
};

/* -------------------------------------------------------------------------- */
/*  GATT characteristic — frequency                                           */
/* -------------------------------------------------------------------------- */

static ssize_t on_freq_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    /* Must be exactly 2 bytes, no offset */
    if (len != 2 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Little-endian 16-bit unpack (no sys_ helper needed) */
    const uint8_t *p = (const uint8_t *)buf;
    uint16_t hz = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    if (hz < DRV_PWM_FREQ_MIN_HZ || hz > DRV_PWM_FREQ_MAX_HZ) {
        LOG_WRN("BLE: freq %u Hz out of range", hz);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int ret = drv_pwm_set_frequency(hz);
    if (ret < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return len;
}

/* -------------------------------------------------------------------------- */
/*  GATT characteristic — motor rail voltage                                  */
/* -------------------------------------------------------------------------- */

static ssize_t on_volt_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    /* Must be exactly 2 bytes, no offset */
    if (len != 2 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *p = (const uint8_t *)buf;
    uint16_t mv = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    if (mv < MAX5419_VOUT_MIN_MV || mv > MAX5419_VOUT_MAX_MV) {
        LOG_WRN("BLE: VDC %u mV out of range", mv);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int ret = max5419_set_voltage((float)mv / 1000.0f);
    if (ret < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Read back the rail after the ramp settles, for the log */
    k_msleep(20);
    int32_t meas_mv = 0;
    if (drv_vdc_sense_read_mv(&meas_mv) == 0) {
        LOG_INF("BLE: VDC -> %u mV (measured %d mV)", mv, meas_mv);
    } else {
        LOG_INF("BLE: VDC -> %u mV", mv);
    }
    return len;
}

/* -------------------------------------------------------------------------- */
/*  GATT characteristic — measured VDC (read)                                 */
/* -------------------------------------------------------------------------- */

static ssize_t on_vdc_read(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    int32_t mv = 0;

    if (drv_vdc_sense_read_mv(&mv) < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    uint16_t v = (uint16_t)CLAMP(mv, 0, UINT16_MAX);
    uint8_t le[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };

    return bt_gatt_attr_read(conn, attr, buf, len, offset, le, sizeof(le));
}

/* -------------------------------------------------------------------------- */
/*  GATT characteristic — motor rail enable                                   */
/* -------------------------------------------------------------------------- */

static ssize_t on_rail_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (len != 1 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t on = *(const uint8_t *)buf;
    if (on > 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    drv_stbb1_apur_set(on != 0);
    LOG_INF("BLE: motor rail -> %s", on ? "ON" : "OFF");
    return len;
}

/* -------------------------------------------------------------------------- */
/*  GATT characteristic — motor driver sleep/wake                             */
/* -------------------------------------------------------------------------- */

static ssize_t on_drv_write(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (len != 1 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t awake = *(const uint8_t *)buf;
    if (awake > 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    drv_drv8212_set(awake != 0);
    LOG_INF("BLE: motor driver -> %s", awake ? "AWAKE" : "SLEEP");
    return len;
}

/* -------------------------------------------------------------------------- */
/*  GATT characteristic — status packet (read)                                */
/*                                                                            */
/*  Fixed 24-byte little-endian layout, parsed by scripts/ble_control.py:     */
/*    0  u8   packet format version (1)                                       */
/*    1  u8   fw major    2 u8 fw minor    3 u8 fw patch                      */
/*    4  u16  PWM frequency [Hz]                                              */
/*    6  u8   duty IN1 [%]   7 u8 duty IN2 [%]                                */
/*    8  u16  target VDC [mV] (0 = never set)                                 */
/*   10  u16  measured VDC [mV]                                               */
/*   12  u8   rail on   13 u8 driver awake   14 u8 IMU ok   15 u8 rsvd        */
/*   16  u32  uptime [s]                                                      */
/*   20  u32  boot reset cause (zephyr hwinfo bits)                           */
/* -------------------------------------------------------------------------- */

#define STATUS_PKT_VERSION  1

static ssize_t on_status_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t s[24];
    int32_t vdc_mv = 0;

    (void)drv_vdc_sense_read_mv(&vdc_mv);

    s[0] = STATUS_PKT_VERSION;
    s[1] = APP_VERSION_MAJOR;
    s[2] = APP_VERSION_MINOR;
    s[3] = APP_PATCHLEVEL;
    sys_put_le16(drv_pwm_get_frequency(), &s[4]);
    s[6] = drv_pwm_get_duty(0);
    s[7] = drv_pwm_get_duty(1);
    sys_put_le16(max5419_get_target_mv(), &s[8]);
    sys_put_le16((uint16_t)CLAMP(vdc_mv, 0, UINT16_MAX), &s[10]);
    s[12] = drv_stbb1_apur_enabled() ? 1 : 0;
    s[13] = drv_drv8212_awake() ? 1 : 0;
    s[14] = drv_asm330lhh_ok() ? 1 : 0;
    s[15] = 0;
    sys_put_le32((uint32_t)(k_uptime_get() / 1000), &s[16]);
    sys_put_le32(app_reset_cause, &s[20]);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, s, sizeof(s));
}

/* -------------------------------------------------------------------------- */
/*  GATT service definition                                                   */
/* -------------------------------------------------------------------------- */

BT_GATT_SERVICE_DEFINE(caterpillar_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CATERPILLAR_SVC),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_FREQ,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, on_freq_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_VOLT,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, on_volt_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_VDCMEAS,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        on_vdc_read, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_RAIL,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, on_rail_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_DRV,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, on_drv_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_STATUS,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        on_status_read, NULL, NULL),
);

/* -------------------------------------------------------------------------- */
/*  Connection callbacks                                                      */
/* -------------------------------------------------------------------------- */

static struct bt_le_adv_param adv_param;  /* saved for re-advertise */

static void restart_advertise(struct k_work *work)
{
    ARG_UNUSED(work);
    int ret = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (ret) {
        LOG_ERR("BLE re-advertise failed: %d", ret);
    } else {
        LOG_INF("BLE re-advertising");
    }
}

static K_WORK_DELAYABLE_DEFINE(adv_restart_work, restart_advertise);

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("BLE connection failed: %u", err);
        /* Advertising already stopped, and disconnected() will not
         * fire for a connection that never established — re-advertise
         * or the device stays unreachable until reboot.
         */
        k_work_schedule(&adv_restart_work, K_MSEC(50));
        return;
    }

    LOG_INF("BLE connected");

    /* Request low-latency connection params (7.5 ms interval) */
    struct bt_le_conn_param param = {
        .interval_min = 6,
        .interval_max = 12,
        .latency = 0,
        .timeout = 400,
    };
    bt_conn_le_param_update(conn, &param);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    LOG_INF("BLE disconnected (reason %u)", reason);

    /* Defer re-advertise — bt_le_adv_start must not be called
     * synchronously from the BLE stack's own callback context.
     */
    k_work_schedule(&adv_restart_work, K_MSEC(50));
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

int ble_interface_init(void)
{
    int ret;

    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("BLE enable failed: %d", ret);
        return ret;
    }

    LOG_INF("BLE stack initialised");

    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    /* Start connectable advertising — fast interval for quick discovery */
    adv_param = (struct bt_le_adv_param)BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONN,
        BT_GAP_ADV_FAST_INT_MIN_1,
        BT_GAP_ADV_FAST_INT_MAX_1,
        NULL);
    ret = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (ret) {
        LOG_ERR("BLE advertising start failed: %d", ret);
        return ret;
    }

    LOG_INF("BLE advertising as \"Caterpillar\"");
    return 0;
}
