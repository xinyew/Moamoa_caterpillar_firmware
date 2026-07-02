/*
 * BLE interface — GATT server for PWM frequency control.
 *
 * Advertises as "Caterpillar", exposes two writable characteristics
 * (both 16-bit little-endian, accepting acknowledged Write Requests
 * as well as Write-Without-Response):
 *   0xFFE1 — PWM frequency in Hz    (4–1000) → drv_pwm_set_frequency()
 *   0xFFE2 — motor rail VDC in mV (750–4200) → max5419_set_voltage()
 */

#include "ble_interface.h"

#include "../drivers/driver_pwm.h"
#include "../drivers/max5419.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(ble_if, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/*  UUIDs                                                                     */
/* -------------------------------------------------------------------------- */

/* 16-bit custom vendor UUIDs */
#define BT_UUID_CATERPILLAR_SVC_VAL   0xFFE0
#define BT_UUID_CATERPILLAR_FREQ_VAL  0xFFE1
#define BT_UUID_CATERPILLAR_VOLT_VAL  0xFFE2

#define BT_UUID_CATERPILLAR_SVC  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_SVC_VAL)
#define BT_UUID_CATERPILLAR_FREQ BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_FREQ_VAL)
#define BT_UUID_CATERPILLAR_VOLT BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_VOLT_VAL)

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

    LOG_INF("BLE: VDC -> %u mV", mv);
    return len;
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
