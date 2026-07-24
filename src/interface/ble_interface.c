/*
 * BLE interface — GATT server for motor control + IMU data.
 *
 * Advertises as "Caterpillar":
 *   0xFFE1 — write: PWM frequency in Hz    (4–1000, u16 LE) → drv_pwm_set_frequency()
 *   0xFFE2 — write: motor rail VDC in mV (750–4200, u16 LE) → max5419_set_voltage()
 *   0xFFE3 — read:  measured VDC in mV (u16 LE, AIN4 sense divider)
 *   0xFFE4 — write: motor rail enable (u8: 0=off, 1=on) → drv_stbb1_apur_set()
 *   0xFFE5 — write: motor driver awake (u8: 0=sleep, 1=awake) → drv_drv8212_set()
 *   0xFFE6 — read:  status packet (44 B LE struct, see on_status_read)
 *   0xFFE7 — write: throughput test sink / read: u32 LE byte counter
 *            (temporary tuning aid — remove when BLE tuning is done)
 *   0xFFE8 — write: IMU sampling config {odr, content, accel_fs, gyro_fs}
 *            (+ optional u16 LE preview-rate cap in Hz, 0 = auto)
 *            read: config + applied status + overrun count (12 B)
 *   0xFFE9 — notify: live IMU sample stream (8 B header + N×16 B records,
 *            decimated to fit the link; flash always gets full rate)
 *   0xFFEA — write: log control {cmd, policy}  (0=stop 1=start 2=erase)
 *            read: log state (20 B)
 *   0xFFEB — write: dump request {session u32, offset u32, len u32} →
 *            notify: chunks {offset u32, n u16, last u8, rsvd} + data
 *   0xFFEC — notify: device warning/error text lines (replaces RTT)
 *   0xFFED — write: status-LED heartbeat enable (u8) / read: current state
 *   0xFFEE — write: wall-clock sync (u32 LE unix epoch, UTC) /
 *            read: device's current epoch estimate (0 = never synced)
 *   0xFFEF — read: session directory, newest first:
 *            {count u8, rsvd[3]} + count × 16 B
 *            {seq u32, wall_start u32, rec_count u32, odr u8,
 *             content u8, accel_fs u8, gyro_fs u8}
 * Writes accept acknowledged Write Requests as well as
 * Write-Without-Response.
 */

#include "ble_interface.h"

#include "../drivers/driver_pwm.h"
#include "../drivers/max5419.h"
#include "../drivers/driver_vdc_sense.h"
#include "../drivers/driver_stbb1_apur.h"
#include "../drivers/driver_drv8212.h"
#include "../drivers/driver_led.h"
#include "../device_cmd.h"
#include "../settings_store.h"
#include "../imu_pump.h"
#include "../imu_log.h"
#include "common/imu_shared.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <zephyr/sys/barrier.h>

/* Defined below by BT_GATT_SERVICE_DEFINE; the notify helpers need it */
extern const struct bt_gatt_service_static caterpillar_svc;

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
#define BT_UUID_CATERPILLAR_TPUT_VAL    0xFFE7
#define BT_UUID_CATERPILLAR_IMUCFG_VAL  0xFFE8
#define BT_UUID_CATERPILLAR_STREAM_VAL  0xFFE9
#define BT_UUID_CATERPILLAR_LOGCTL_VAL  0xFFEA
#define BT_UUID_CATERPILLAR_DUMP_VAL    0xFFEB
#define BT_UUID_CATERPILLAR_MSG_VAL     0xFFEC
#define BT_UUID_CATERPILLAR_LED_VAL     0xFFED
#define BT_UUID_CATERPILLAR_TIME_VAL    0xFFEE
#define BT_UUID_CATERPILLAR_DIR_VAL     0xFFEF
#define BT_UUID_CATERPILLAR_T2LOG_VAL   0xFFF0

#define BT_UUID_CATERPILLAR_SVC  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_SVC_VAL)
#define BT_UUID_CATERPILLAR_FREQ BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_FREQ_VAL)
#define BT_UUID_CATERPILLAR_VOLT BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_VOLT_VAL)
#define BT_UUID_CATERPILLAR_VDCMEAS \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_VDCMEAS_VAL)
#define BT_UUID_CATERPILLAR_RAIL BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_RAIL_VAL)
#define BT_UUID_CATERPILLAR_DRV  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_DRV_VAL)
#define BT_UUID_CATERPILLAR_STATUS \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_STATUS_VAL)
#define BT_UUID_CATERPILLAR_TPUT BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_TPUT_VAL)
#define BT_UUID_CATERPILLAR_IMUCFG \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_IMUCFG_VAL)
#define BT_UUID_CATERPILLAR_STREAM \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_STREAM_VAL)
#define BT_UUID_CATERPILLAR_LOGCTL \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_LOGCTL_VAL)
#define BT_UUID_CATERPILLAR_DUMP BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_DUMP_VAL)
#define BT_UUID_CATERPILLAR_MSG  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_MSG_VAL)
#define BT_UUID_CATERPILLAR_LED  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_LED_VAL)
#define BT_UUID_CATERPILLAR_TIME BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_TIME_VAL)
#define BT_UUID_CATERPILLAR_DIR  BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_DIR_VAL)
#define BT_UUID_CATERPILLAR_T2LOG \
    BT_UUID_DECLARE_16(BT_UUID_CATERPILLAR_T2LOG_VAL)

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

    settings_set(SETTING_MOTOR_FREQ_HZ, hz);
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
/*  Fixed 44-byte little-endian layout, parsed by scripts/ble_control.py      */
/*  and gui/protocol.py:                                                      */
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
/*  GATT characteristic — throughput test sink (TEMPORARY tuning aid)         */
/*                                                                            */
/*  Write (any length): bytes are counted and discarded.                      */
/*  Read: u32 LE running byte counter.  The script measures rate by          */
/*  reading the counter before/after a timed write flood.                     */
/* -------------------------------------------------------------------------- */

static uint32_t tput_rx_bytes;

static ssize_t on_tput_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

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
/*  Wall-clock sync (0xFFEE) — the GUI writes unix time on every connect      */
/*  so on-chip log sessions carry real-world start timestamps.                */
/* -------------------------------------------------------------------------- */

/* epoch seconds at device boot (uptime 0); 0 = never synced */
static uint32_t wall_epoch_at_boot;

uint32_t ble_wall_now(void)
{
    if (wall_epoch_at_boot == 0) {
        return 0;
    }
    return wall_epoch_at_boot + (uint32_t)(k_uptime_get() / 1000);
}

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

    wall_epoch_at_boot = epoch - (uint32_t)(k_uptime_get() / 1000);
    LOG_INF("BLE: wall clock synced (epoch %u)", epoch);
    return len;
}

static ssize_t on_time_read(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    uint8_t le[4];

    sys_put_le32(ble_wall_now(), le);
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

/* Actual rates for each IMU_ODR_* code (Hz, rounded up) */
static const uint16_t odr_hz[11] = {
    0, 13, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660
};

/* Live stream budget: ~1300 samples/s of 16 B records ≈ 20 KiB/s */
#define STREAM_BUDGET_SPS  1300

static uint8_t stream_decim = 1;
static uint16_t stream_preview_hz;   /* 0 = auto (link-budget limit) */

static void stream_update_decim(void)
{
    uint16_t hz = odr_hz[IMU_SHARED->cfg_odr <= 10 ? IMU_SHARED->cfg_odr : 0];
    uint32_t target = STREAM_BUDGET_SPS;

    if (stream_preview_hz != 0 && stream_preview_hz < target) {
        target = stream_preview_hz;
    }
    uint32_t d = (hz + target - 1) / target;

    stream_decim = (uint8_t)CLAMP(d, 1, 255);
}

void ble_stream_set_preview(uint16_t hz)
{
    stream_preview_hz = hz;
    stream_update_decim();
}

/* -------------------------------------------------------------------------- */
/*  On-demand sampling arbiter                                                */
/*                                                                            */
/*  The IMU is powered down unless someone needs data: a running log          */
/*  session or a subscribed live stream.  Config changes while idle are       */
/*  remembered (settings) and applied on the next enable.                     */
/* -------------------------------------------------------------------------- */

static bool stream_subscribed;

bool ble_imu_demand(void)
{
    return stream_subscribed || imu_log_active();
}

void ble_imu_run_update(void)
{
    struct imu_shared *sh = IMU_SHARED;

    if (sh->magic != IMU_SHARED_MAGIC) {
        return;
    }

    uint8_t content = ble_imu_demand()
        ? (uint8_t)settings_get(SETTING_IMU_CONTENT) : 0;

    sh->cfg_odr = (uint8_t)settings_get(SETTING_IMU_ODR_CODE);
    sh->cfg_content = content;
    sh->cfg_accel_fs = (uint8_t)settings_get(SETTING_IMU_ACCEL_FS);
    sh->cfg_gyro_fs = (uint8_t)settings_get(SETTING_IMU_GYRO_FS);
    barrier_dmem_fence_full();
    sh->cfg_seq = sh->cfg_seq + 1;

    stream_update_decim();
    LOG_INF("IMU %s (odr code %u)", content ? "RUNNING" : "powered down",
            sh->cfg_odr);
}

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
    ble_stream_set_preview((len >= 6) ? sys_get_le16(&p[4]) : 0);

    settings_set(SETTING_IMU_ODR_CODE, odr);
    settings_set(SETTING_IMU_CONTENT, content);
    settings_set(SETTING_IMU_ACCEL_FS, afs);
    settings_set(SETTING_IMU_GYRO_FS, gfs);
    settings_set(SETTING_PREVIEW_HZ, stream_preview_hz);

    /* Applied to the sensor only if something is consuming data;
     * otherwise remembered and applied on the next enable.
     */
    ble_imu_run_update();

    LOG_INF("BLE: IMU cfg -> odr=%u Hz content=0x%x fs=%u/%u "
            "preview=%u Hz (decim %u)", odr_hz[odr], content, afs, gfs,
            stream_preview_hz, stream_decim);
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
    s[6] = stream_decim;
    s[7] = 0;
    sys_put_le32(sh->overrun, &s[8]);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, s, sizeof(s));
}

/* -------------------------------------------------------------------------- */
/*  Notification TX thread — live stream (0xFFE9) + message lines (0xFFEC)    */
/*                                                                            */
/*  The pump thread must NEVER call into the BT stack: bt_gatt_notify can    */
/*  block indefinitely under TX-context exhaustion (proven on hardware by    */
/*  the dump wedge), and a blocked pump overruns the FLPR ring.  Instead     */
/*  the sink stages picked samples into an SPSC FIFO and this thread does    */
/*  all sending, credit-paced (≤2 notifications in flight) so the blocking  */
/*  path is never entered.  ble_msg() lines ride the same thread.            */
/*                                                                            */
/*  Stream packet: 8 B header + N × 16 B struct imu_sample:                   */
/*    0 u8 N   1 u8 decim   2 u8 flags   3 u8 rsvd   4 u32 dropped samples    */
/* -------------------------------------------------------------------------- */

#define STREAM_MAX_SAMPLES  14
#define STREAM_HDR          8

#define SFIFO_N   256               /* staged samples, power of two */
#define MSGQ_N    4
#define MSG_LEN   120

static struct imu_sample sfifo[SFIFO_N];
static volatile uint32_t sfifo_head;   /* producer: pump (sink) */
static volatile uint32_t sfifo_tail;   /* consumer: TX thread   */

static char msgq[MSGQ_N][MSG_LEN];
static volatile uint32_t msgq_head, msgq_tail;
static struct k_spinlock msgq_lock;    /* multiple msg producers */

static uint8_t stream_pkt[STREAM_HDR + STREAM_MAX_SAMPLES * 16];
static uint8_t stream_max = STREAM_MAX_SAMPLES;
static uint32_t stream_sample_ctr;
static uint32_t stream_dropped;
static int64_t sfifo_ts;               /* when the FIFO went non-empty */

static K_SEM_DEFINE(tx_wake, 0, 1);
static K_SEM_DEFINE(tx_credits, 2, 2);

static void tx_sent_cb(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn); ARG_UNUSED(user_data);
    k_sem_give(&tx_credits);
}

/* Send one notification, credit-paced.  Returns false if dropped
 * (congestion, disconnect, jam) — callers discard, never retry stale
 * preview data.
 */
static bool tx_notify(const struct bt_uuid *uuid, const void *data,
                      uint16_t len)
{
    if (k_sem_take(&tx_credits, K_MSEC(500)) != 0) {
        return false;                /* TX jammed — drop */
    }

    struct bt_gatt_notify_params np = {
        .uuid = uuid,
        .attr = caterpillar_svc.attrs,
        .data = data,
        .len = len,
        .func = tx_sent_cb,
    };
    int ret;
    int tries = 0;

    do {
        ret = bt_gatt_notify_cb(NULL, &np);
        if (ret == -ENOMEM || ret == -EAGAIN || ret == -ENOBUFS) {
            k_msleep(1);
        } else {
            break;
        }
    } while (++tries < 100);

    if (ret < 0) {
        k_sem_give(&tx_credits);     /* never handed to the stack */
        return false;
    }
    return true;
}

/* Pump-context sink: decimate and stage.  No BT calls, no blocking. */
static void stream_sink(const struct imu_sample *s, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        if ((stream_sample_ctr++ % stream_decim) != 0) {
            continue;
        }
        if (sfifo_head - sfifo_tail >= SFIFO_N) {
            stream_dropped++;        /* TX behind — drop, never block */
            continue;
        }
        if (sfifo_head == sfifo_tail) {
            sfifo_ts = k_uptime_get();
        }
        sfifo[sfifo_head & (SFIFO_N - 1)] = s[i];
        sfifo_head = sfifo_head + 1;
    }
    k_sem_give(&tx_wake);
}

static void tx_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        k_sem_take(&tx_wake, K_MSEC(50));

        /* Messages first — rare and high-value */
        while (msgq_tail != msgq_head) {
            const char *line = msgq[msgq_tail % MSGQ_N];

            (void)tx_notify(BT_UUID_CATERPILLAR_MSG, line, strlen(line));
            msgq_tail = msgq_tail + 1;
        }

        /* Stream packets: full ones eagerly, partials after 100 ms */
        while (1) {
            uint32_t avail = sfifo_head - sfifo_tail;

            if (avail == 0) {
                break;
            }
            if (avail < stream_max &&
                (k_uptime_get() - sfifo_ts) < 100) {
                break;               /* wait for a fuller packet */
            }

            uint8_t n = (uint8_t)MIN(avail, (uint32_t)stream_max);

            for (uint8_t i = 0; i < n; i++) {
                memcpy(&stream_pkt[STREAM_HDR + i * 16],
                       (const void *)&sfifo[(sfifo_tail + i) & (SFIFO_N - 1)],
                       16);
            }
            stream_pkt[0] = n;
            stream_pkt[1] = stream_decim;
            stream_pkt[2] = 0;
            stream_pkt[3] = 0;
            sys_put_le32(stream_dropped, &stream_pkt[4]);

            if (!tx_notify(BT_UUID_CATERPILLAR_STREAM, stream_pkt,
                           STREAM_HDR + n * 16)) {
                stream_dropped += n;
            }
            sfifo_tail = sfifo_tail + n;
            sfifo_ts = k_uptime_get();
        }
    }
}

K_THREAD_DEFINE(ble_tx_tid, 2048, tx_thread, NULL, NULL, NULL, 9, 0, 0);

static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    if (value == BT_GATT_CCC_NOTIFY) {
        sfifo_tail = sfifo_head;
        stream_sample_ctr = 0;
        stream_dropped = 0;
        stream_max = STREAM_MAX_SAMPLES;
        stream_update_decim();
        imu_pump_set_sink(stream_sink);
        stream_subscribed = true;
        LOG_INF("BLE: IMU stream ON (decim %u)", stream_decim);
    } else {
        imu_pump_set_sink(NULL);
        stream_subscribed = false;
        LOG_INF("BLE: IMU stream OFF");
    }
    ble_imu_run_update();
}

/* -------------------------------------------------------------------------- */
/*  Session connection parameters                                             */
/* -------------------------------------------------------------------------- */

static struct bt_conn *cur_conn;

/* slow=true: 50 ms interval while a log session runs (frees radio air
 * so flash writes get MPSL timeslots); slow=false: 7.5-15 ms for
 * responsive control and dumps.
 */
void ble_session_conn_params(bool slow)
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

/* -------------------------------------------------------------------------- */
/*  Log control (0xFFEA)                                                      */
/* -------------------------------------------------------------------------- */

#define LOGCTL_CMD_STOP   0
#define LOGCTL_CMD_START  1
#define LOGCTL_CMD_ERASE  2

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

    if (p[0] > LOGCTL_CMD_ERASE) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    if (p[0] == LOGCTL_CMD_START && imu_log_active()) {
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
/*  Log dump (0xFFEB): request {session u32, offset u32, len u32} ->          */
/*  notify chunks: 0 u32 offset   4 u16 n   6 u8 last   7 u8 rsvd   8.. data  */
/* -------------------------------------------------------------------------- */

#define DUMP_CHUNK_DATA  232
#define DUMP_CREDITS     2

static K_SEM_DEFINE(dump_sem, 0, 1);
/* Flow control: max DUMP_CREDITS notifications in flight.  Sending
 * unpaced exhausts the host TX contexts and bt_gatt_notify then
 * BLOCKS FOREVER waiting for one (observed on hardware: dump wedged
 * mid-transfer, deaf to new requests, link eventually dropped).
 */
static K_SEM_DEFINE(dump_credit_sem, 0, DUMP_CREDITS);
static uint32_t dump_req_session, dump_req_off, dump_req_len;
static volatile bool dump_abort;

static void dump_sent_cb(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn); ARG_UNUSED(user_data);
    k_sem_give(&dump_credit_sem);
}

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
    dump_abort = true;               /* cancel any dump in flight */
    dump_req_session = sys_get_le32(&p[0]);
    dump_req_off = sys_get_le32(&p[4]);
    dump_req_len = sys_get_le32(&p[8]);
    k_sem_give(&dump_sem);
    return len;
}

static void dump_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    static uint8_t chunk[8 + DUMP_CHUNK_DATA];

    while (1) {
        k_sem_take(&dump_sem, K_FOREVER);
        dump_abort = false;

        uint32_t session = dump_req_session;
        uint32_t off = dump_req_off;
        uint32_t remaining = dump_req_len;

        /* Fresh credits per request (stale sent-callbacks from an
         * aborted dump can only saturate the cap, which is harmless).
         */
        while (k_sem_take(&dump_credit_sem, K_NO_WAIT) == 0) {
        }
        for (int i = 0; i < DUMP_CREDITS; i++) {
            k_sem_give(&dump_credit_sem);
        }

        while (remaining > 0 && !dump_abort) {
            uint32_t want = MIN(remaining, (uint32_t)DUMP_CHUNK_DATA);
            int n = imu_log_read_session(session, off, &chunk[8], want);

            if (n < 0) {
                /* never silently: this was the invisible failure mode
                 * behind dumps freezing at a fixed percentage
                 */
                ble_msg("dump read error %d at offset %u", n, off);
                break;
            }
            bool last = (n < (int)want) || ((uint32_t)n == remaining);

            sys_put_le32(off, &chunk[0]);
            sys_put_le16((uint16_t)n, &chunk[4]);
            chunk[6] = last ? 1 : 0;
            chunk[7] = 0;

            /* Wait for an in-flight slot; a long wait means the TX
             * path is wedged — abort and let the host resume.
             */
            if (k_sem_take(&dump_credit_sem, K_SECONDS(3)) != 0) {
                ble_msg("dump stalled at offset %u — TX jam, resume",
                        off);
                break;
            }

            struct bt_gatt_notify_params np = {
                .uuid = BT_UUID_CATERPILLAR_DUMP,
                .attr = caterpillar_svc.attrs,
                .data = chunk,
                .len = (uint16_t)(8 + n),
                .func = dump_sent_cb,
            };
            int ret;
            int tries = 0;
            do {
                ret = bt_gatt_notify_cb(NULL, &np);
                if (ret == -ENOMEM || ret == -EAGAIN || ret == -ENOBUFS) {
                    k_msleep(2);
                } else {
                    break;
                }
            } while (++tries < 500 && !dump_abort);

            if (ret < 0) {
                k_sem_give(&dump_credit_sem);   /* never sent */
                ble_msg("dump aborted at offset %u (err %d)", off, ret);
                break;
            }
            if (last) {
                break;
            }
            off += n;
            remaining -= n;
        }
    }
}

K_THREAD_DEFINE(dump_tid, 2048, dump_thread, NULL, NULL, NULL, 7, 0, 0);

/* -------------------------------------------------------------------------- */
/*  Device message channel (0xFFEC): warning/error lines, replaces RTT        */
/* -------------------------------------------------------------------------- */

/* Tier-2 log: every ble_msg line also lands in a 2 KB RAM ring that
 * clients can read AFTER THE FACT via 0xFFF0 — warnings emitted while
 * nobody was subscribed (or even connected) stay queryable until they
 * age out.  Lesson learned: flash-write errors went to RTT only and
 * stayed invisible for weeks.
 */
#define T2_RING_SIZE 2048

static char t2_ring[T2_RING_SIZE];
static uint32_t t2_head;             /* total bytes ever written */

static void t2_store(const char *line)
{
    char stamped[MSG_LEN + 16];
    int64_t up = k_uptime_get();
    int n = snprintf(stamped, sizeof(stamped), "[%5lld.%03lld] %s\n",
                     up / 1000, up % 1000, line);

    if (n <= 0) {
        return;
    }
    n = MIN(n, (int)sizeof(stamped) - 1);
    for (int i = 0; i < n; i++) {
        t2_ring[t2_head % T2_RING_SIZE] = stamped[i];
        t2_head++;
    }
}

static ssize_t on_t2log_read(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    static char snap[T2_RING_SIZE];
    k_spinlock_key_t key = k_spin_lock(&msgq_lock);
    uint32_t used = MIN(t2_head, (uint32_t)T2_RING_SIZE);
    uint32_t start = t2_head - used;

    for (uint32_t i = 0; i < used; i++) {
        snap[i] = t2_ring[(start + i) % T2_RING_SIZE];
    }
    k_spin_unlock(&msgq_lock, key);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, snap, used);
}

void ble_msg(const char *fmt, ...)
{
    char line[MSG_LEN];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    /* Mirror to the local log for wired debugging */
    LOG_WRN("%s", line);

    /* Enqueue for the TX thread — callers include the pump thread
     * (holding the log mutex), so sending inline here is forbidden.
     */
    k_spinlock_key_t key = k_spin_lock(&msgq_lock);

    t2_store(line);
    if (msgq_head - msgq_tail < MSGQ_N) {
        strncpy(msgq[msgq_head % MSGQ_N], line, MSG_LEN - 1);
        msgq[msgq_head % MSGQ_N][MSG_LEN - 1] = '\0';
        msgq_head = msgq_head + 1;
    }
    k_spin_unlock(&msgq_lock, key);
    k_sem_give(&tx_wake);
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
    cur_conn = conn;

    /* Request low-latency connection params (7.5 ms interval) */
    struct bt_le_conn_param param = {
        .interval_min = 6,
        .interval_max = 12,
        .latency = 0,
        .timeout = 400,
    };
    bt_conn_le_param_update(conn, &param);

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

    /* Stop per-connection data flows (CCC state is not bonded) and
     * close a running log session — a session ends at the stop click
     * or at connection loss, whichever comes first.
     */
    imu_pump_set_sink(NULL);
    dump_abort = true;
    imu_log_stop();

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
