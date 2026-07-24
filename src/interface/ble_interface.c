/*
 * BLE control-plane adapter — GATT service + connection lifecycle.
 *
 * Handlers here only DECODE and DELEGATE: fast driver pokes run
 * inline, slow operations go to device_cmd, run-state policy to
 * session.c, and all sending to ble_transport.c.  Nothing in this
 * file may block the BT RX thread.
 *
 * Wire contract (details in docs/ble-protocol.md):
 *   0xFFE1 write freq · 0xFFE2 write VDC · 0xFFE3 read measured VDC
 *   0xFFE4 write rail · 0xFFE5 write driver · 0xFFE6 read status v3
 *   0xFFE7 throughput sink · 0xFFE8 IMU config · 0xFFE9 stream notify
 *   0xFFEA log control · 0xFFEB dump · 0xFFEC messages · 0xFFED LED
 *   0xFFEE time sync · 0xFFEF session dir · 0xFFF0 tier-2 log
 * Writes accept acknowledged Write Requests and Write-Without-Response.
 */

#include "ble_interface.h"
#include "ble_transport.h"
#include "ble_uuids.h"

#include "../app.h"
#include "../session.h"
#include "../device_cmd.h"
#include "../settings_store.h"
#include "../imu_pump.h"
#include "../imu_log.h"
#include "../drivers/driver_pwm.h"
#include "../drivers/max5419.h"
#include "../drivers/driver_vdc_sense.h"
#include "../drivers/driver_stbb1_apur.h"
#include "../drivers/driver_drv8212.h"
#include "../drivers/driver_led.h"
#include "common/imu_shared.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <app_version.h>

LOG_MODULE_REGISTER(ble_if, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/*  Advertising data                                                          */
/* -------------------------------------------------------------------------- */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "Caterpillar", sizeof("Caterpillar") - 1),
};

/* -------------------------------------------------------------------------- */
/*  Motor characteristics (0xFFE1–0xFFE5)                                     */
/* -------------------------------------------------------------------------- */

static ssize_t on_freq_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (len != 2 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *p = (const uint8_t *)buf;
    uint16_t hz = (uint16_t)p[0] | ((uint16_t)p[1] << 8);

    if (hz < DRV_PWM_FREQ_MIN_HZ || hz > DRV_PWM_FREQ_MAX_HZ) {
        LOG_WRN("BLE: freq %u Hz out of range", hz);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    if (drv_pwm_set_frequency(hz) < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    settings_set(SETTING_MOTOR_FREQ_HZ, hz);
    return len;
}

static ssize_t on_volt_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (len != 2 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *p = (const uint8_t *)buf;
    uint16_t mv = (uint16_t)p[0] | ((uint16_t)p[1] << 8);

    if (mv < MAX5419_VOUT_MIN_MV || mv > MAX5419_VOUT_MAX_MV) {
        LOG_WRN("BLE: VDC %u mV out of range", mv);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    /* The digipot ramps at 10 ms/tap (up to ~1.5 s for a full swing);
     * running that here would stall the BT RX thread and all inbound
     * traffic with it.  Validate, enqueue, ack.
     */
    struct device_cmd cmd = {
        .type = DEVICE_CMD_SET_VOLT_MV,
        .volt_mv = mv,
    };

    if (device_cmd_submit(&cmd) < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return len;
}

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

static ssize_t on_rail_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

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

static ssize_t on_drv_write(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

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
/*  Status packet (0xFFE6 read)                                               */
/*                                                                            */
/*  Fixed 44-byte little-endian layout, parsed by scripts/protocol.py         */
/*  and scripts/ble_control.py:                                               */
/*    0  u8   packet format version (3)                                       */
/*    1  u8   fw major    2 u8 fw minor    3 u8 fw patch                      */
/*    4  u16  PWM frequency [Hz]                                              */
/*    6  u8   duty IN1 [%]   7 u8 duty IN2 [%]                                */
/*    8  u16  target VDC [mV] (0 = never set)                                 */
/*   10  u16  measured VDC [mV]                                               */
/*   12  u8   rail on   13 u8 driver awake   14 u8 IMU ok   15 u8 LED on      */
/*   16  u32  uptime [s]                                                      */
/*   20  u32  boot reset cause (zephyr hwinfo bits)                           */
/*   24  u32  FLPR fw version 0x00MMmmpp (0 = FLPR not running/too old)       */
/*   28  u8   IMU ODR code   29 u8 content mask                               */
/*   30  u8   log active     31 u8 log fill policy                            */
/*   32  u32  log bytes stored                                                */
/*   36  u32  log capacity bytes                                              */
/*   40  u32  FLPR ring overruns                                              */
/* -------------------------------------------------------------------------- */

#define STATUS_PKT_VERSION  3

static ssize_t on_status_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t s[44];
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
    s[14] = (IMU_SHARED->magic == IMU_SHARED_MAGIC && IMU_SHARED->imu_ok)
                ? 1 : 0;
    s[15] = drv_led_enabled() ? 1 : 0;
    sys_put_le32((uint32_t)(k_uptime_get() / 1000), &s[16]);
    sys_put_le32(app_reset_cause, &s[20]);
    sys_put_le32((IMU_SHARED->magic == IMU_SHARED_MAGIC)
                     ? IMU_SHARED->flpr_version : 0, &s[24]);
    s[28] = IMU_SHARED->cfg_odr;
    s[29] = IMU_SHARED->cfg_content;
    s[30] = imu_log_active() ? 1 : 0;
    s[31] = imu_log_policy();
    sys_put_le32(imu_log_bytes_stored(), &s[32]);
    sys_put_le32(imu_log_capacity_bytes(), &s[36]);
    sys_put_le32(imu_pump_overrun(), &s[40]);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, s, sizeof(s));
}

/* -------------------------------------------------------------------------- */
/*  Throughput test sink (0xFFE7, tuning aid)                                 */
/* -------------------------------------------------------------------------- */

static uint32_t tput_rx_bytes;

static ssize_t on_tput_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(buf);
    ARG_UNUSED(offset); ARG_UNUSED(flags);

    tput_rx_bytes += len;
    return len;
}

static ssize_t on_tput_read(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    uint8_t le[4];

    sys_put_le32(tput_rx_bytes, le);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, le, sizeof(le));
}

/* -------------------------------------------------------------------------- */
/*  Wall-clock sync (0xFFEE)                                                  */
/* -------------------------------------------------------------------------- */

static ssize_t on_time_write(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (len != 4 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint32_t epoch = sys_get_le32(buf);

    if (epoch == 0) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    session_time_sync(epoch);
    return len;
}

static ssize_t on_time_read(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    uint8_t le[4];

    sys_put_le32(session_wall_now(), le);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, le, sizeof(le));
}

/* -------------------------------------------------------------------------- */
/*  Status-LED heartbeat enable (0xFFED)                                      */
/* -------------------------------------------------------------------------- */

static ssize_t on_led_write(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len,
                            uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (len != 1 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t on = *(const uint8_t *)buf;

    if (on > 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    drv_led_set_enabled(on != 0);
    settings_set(SETTING_LED_ENABLED, on);
    return len;
}

static ssize_t on_led_read(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    uint8_t v = drv_led_enabled() ? 1 : 0;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &v, sizeof(v));
}

/* -------------------------------------------------------------------------- */
/*  IMU sampling config (0xFFE8)                                              */
/* -------------------------------------------------------------------------- */

static ssize_t on_imucfg_write(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len,
                               uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (len < 4 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *p = buf;
    uint8_t odr = p[0], content = p[1], afs = p[2], gfs = p[3];

    if (odr < IMU_ODR_12HZ5 || odr > IMU_ODR_6660HZ || content == 0 ||
        (content & ~(IMU_CONTENT_ACCEL | IMU_CONTENT_GYRO)) != 0 ||
        afs > 3 || gfs > 3) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    if (IMU_SHARED->magic != IMU_SHARED_MAGIC) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Optional bytes 4-5: live-preview rate cap in Hz (u16 LE);
     * 0 or absent = auto (whatever the link budget allows).  Logging
     * always runs at the full ODR regardless.
     */
    uint16_t preview = (len >= 6) ? sys_get_le16(&p[4]) : 0;

    ble_stream_set_preview(preview);

    settings_set(SETTING_IMU_ODR_CODE, odr);
    settings_set(SETTING_IMU_CONTENT, content);
    settings_set(SETTING_IMU_ACCEL_FS, afs);
    settings_set(SETTING_IMU_GYRO_FS, gfs);
    settings_set(SETTING_PREVIEW_HZ, preview);

    /* Applied to the sensor only if something is consuming data;
     * otherwise remembered and applied on the next enable.
     */
    session_imu_run_update();

    LOG_INF("BLE: IMU cfg -> odr=%u Hz content=0x%x fs=%u/%u "
            "preview=%u Hz (decim %u)", ble_odr_hz(odr), content, afs,
            gfs, preview, ble_transport_stream_decim());
    return len;
}

static ssize_t on_imucfg_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    struct imu_shared *sh = IMU_SHARED;
    uint8_t s[12];

    s[0] = sh->cfg_odr;
    s[1] = sh->cfg_content;
    s[2] = sh->cfg_accel_fs;
    s[3] = sh->cfg_gyro_fs;
    s[4] = (sh->cfg_applied == sh->cfg_seq) ? 1 : 0;
    s[5] = (uint8_t)(int8_t)CLAMP(sh->cfg_status, -128, 0);
    s[6] = ble_transport_stream_decim();
    s[7] = 0;
    sys_put_le32(sh->overrun, &s[8]);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, s, sizeof(s));
}

/* -------------------------------------------------------------------------- */
/*  Stream CCC (0xFFE9)                                                       */
/* -------------------------------------------------------------------------- */

static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    bool on = (value == BT_GATT_CCC_NOTIFY);

    ble_transport_stream_enable(on);
    session_set_stream_active(on);
}

/* -------------------------------------------------------------------------- */
/*  Log control (0xFFEA)                                                      */
/* -------------------------------------------------------------------------- */

static ssize_t on_logctl_write(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len,
                               uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (len < 1 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *p = buf;

    if (p[0] > DEVICE_LOG_OP_ERASE) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    if (p[0] == DEVICE_LOG_OP_START && imu_log_active()) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    /* Stop waits up to ~1 s for the flash writer to drain and erase
     * does timeslot-synced flash writes — both far too slow for the
     * BT RX thread.  Validate, enqueue, ack; the status poll and the
     * session directory reflect completion.
     */
    struct device_cmd cmd = {
        .type = DEVICE_CMD_LOG,
        .log.op = p[0],
    };

    if (device_cmd_submit(&cmd) < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return len;
}

static ssize_t on_logctl_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t s[20];

    s[0] = imu_log_active() ? 1 : 0;
    s[1] = imu_log_policy();
    s[2] = 0;
    s[3] = 0;
    sys_put_le32(imu_log_bytes_stored(), &s[4]);
    sys_put_le32(imu_log_capacity_bytes(), &s[8]);
    sys_put_le32(imu_log_records_total(), &s[12]);
    sys_put_le32(imu_pump_overrun(), &s[16]);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, s, sizeof(s));
}

/* -------------------------------------------------------------------------- */
/*  Session directory (0xFFEF, read)                                          */
/* -------------------------------------------------------------------------- */

static ssize_t on_dir_read(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    struct imu_log_session list[IMU_LOG_MAX_SESSIONS];
    uint8_t pkt[4 + IMU_LOG_MAX_SESSIONS * 16];
    int n = imu_log_session_list(list, IMU_LOG_MAX_SESSIONS);

    pkt[0] = (uint8_t)n;
    pkt[1] = pkt[2] = pkt[3] = 0;
    for (int i = 0; i < n; i++) {
        uint8_t *e = &pkt[4 + i * 16];

        sys_put_le32(list[i].seq, &e[0]);
        sys_put_le32(list[i].wall_start, &e[4]);
        sys_put_le32(list[i].rec_count, &e[8]);
        e[12] = list[i].odr_code;
        e[13] = list[i].content;
        e[14] = list[i].accel_fs;
        e[15] = list[i].gyro_fs;
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, pkt,
                             4 + n * 16);
}

/* -------------------------------------------------------------------------- */
/*  Dump request (0xFFEB write) — chunks flow from ble_transport              */
/* -------------------------------------------------------------------------- */

static ssize_t on_dump_write(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (len != 12 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *p = buf;

    ble_transport_dump_request(sys_get_le32(&p[0]), sys_get_le32(&p[4]),
                               sys_get_le32(&p[8]));
    return len;
}

/* -------------------------------------------------------------------------- */
/*  Tier-2 log (0xFFF0 read)                                                  */
/* -------------------------------------------------------------------------- */

static ssize_t on_t2log_read(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    static char snap[2048];
    uint32_t used = ble_transport_t2_snapshot(snap, sizeof(snap));

    return bt_gatt_attr_read(conn, attr, buf, len, offset, snap, used);
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
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_TPUT,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        on_tput_read, on_tput_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_IMUCFG,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        on_imucfg_read, on_imucfg_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_STREAM,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(stream_ccc_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_LOGCTL,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        on_logctl_read, on_logctl_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_DUMP,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_WRITE,
        NULL, on_dump_write, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_MSG,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_LED,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        on_led_read, on_led_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_TIME,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        on_time_read, on_time_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_DIR,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        on_dir_read, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CATERPILLAR_T2LOG,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        on_t2log_read, NULL, NULL),
);

/* -------------------------------------------------------------------------- */
/*  Connection lifecycle                                                      */
/* -------------------------------------------------------------------------- */

static struct bt_conn *cur_conn;
static struct bt_le_adv_param adv_param;  /* saved for re-advertise */

void ble_conn_request_params(bool slow)
{
    if (cur_conn == NULL) {
        return;
    }

    struct bt_le_conn_param param = slow
        ? (struct bt_le_conn_param){ .interval_min = 32, .interval_max = 48,
                                     .latency = 0, .timeout = 400 }
        : (struct bt_le_conn_param){ .interval_min = 6, .interval_max = 12,
                                     .latency = 0, .timeout = 400 };
    int ret = bt_conn_le_param_update(cur_conn, &param);

    if (ret && ret != -EALREADY) {
        LOG_WRN("conn param update (%s): %d", slow ? "slow" : "fast", ret);
    }
}

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
    cur_conn = conn;

    /* Request low-latency connection params (7.5 ms interval) */
    ble_conn_request_params(false);

    /* Request 2M PHY — doubles air throughput if the central agrees */
    int perr = bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);

    if (perr) {
        LOG_WRN("2M PHY request failed: %d", perr);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    LOG_INF("BLE disconnected (reason %u)", reason);
    cur_conn = NULL;

    session_on_disconnect();

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
