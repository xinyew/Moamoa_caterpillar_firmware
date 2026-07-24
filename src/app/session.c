/*
 * Session orchestration â€” see session.h.
 * Logic moved from ble_interface.c in the app/adapter split.
 */

#include "session.h"
#include "settings/settings_store.h"
#include "imu/imu_log.h"
#include "ble/ble_interface.h"
#include "ble/ble_transport.h"
#include "common/imu_shared.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(session, LOG_LEVEL_INF);

/* ---- wall clock ---------------------------------------------------------- */

/* epoch seconds at device boot (uptime 0); 0 = never synced */
static uint32_t wall_epoch_at_boot;

void session_time_sync(uint32_t epoch)
{
    wall_epoch_at_boot = epoch - (uint32_t)(k_uptime_get() / 1000);
    LOG_INF("wall clock synced (epoch %u)", epoch);
}

uint32_t session_wall_now(void)
{
    if (wall_epoch_at_boot == 0) {
        return 0;
    }
    return wall_epoch_at_boot + (uint32_t)(k_uptime_get() / 1000);
}

/* ---- on-demand sampling arbiter ------------------------------------------ */

/* The IMU is powered down unless someone needs data: a running log
 * session or a subscribed live stream.  Config changes while idle are
 * remembered (settings) and applied on the next enable.
 */

static bool stream_active;

bool session_imu_demand(void)
{
    return stream_active || imu_log_active();
}

void session_imu_run_update(void)
{
    struct imu_shared *sh = IMU_SHARED;

    if (sh->magic != IMU_SHARED_MAGIC) {
        return;
    }

    uint8_t content = session_imu_demand()
        ? (uint8_t)settings_get(SETTING_IMU_CONTENT) : 0;

    sh->cfg_odr = (uint8_t)settings_get(SETTING_IMU_ODR_CODE);
    sh->cfg_content = content;
    sh->cfg_accel_fs = (uint8_t)settings_get(SETTING_IMU_ACCEL_FS);
    sh->cfg_gyro_fs = (uint8_t)settings_get(SETTING_IMU_GYRO_FS);
    barrier_dmem_fence_full();
    sh->cfg_seq = sh->cfg_seq + 1;

    ble_transport_stream_update_decim();
    LOG_INF("IMU %s (odr code %u)", content ? "RUNNING" : "powered down",
            sh->cfg_odr);
}

void session_set_stream_active(bool active)
{
    stream_active = active;
    session_imu_run_update();
}

/* ---- log session lifecycle ----------------------------------------------- */

void session_on_log_started(void)
{
    /* Wide connection interval while logging: every radio event
     * blocks an MPSL flash timeslot, and at high ODR the log writer
     * needs that air time more than the (capped) preview does.
     */
    ble_conn_request_params(true);
    session_imu_run_update();
}

void session_on_log_stopped(void)
{
    ble_conn_request_params(false);
    session_imu_run_update();
}

void session_on_disconnect(void)
{
    /* Stop per-connection data flows (CCC state is not bonded) and
     * close a running log session â€” a session ends at the stop click
     * or at connection loss, whichever comes first.
     */
    stream_active = false;
    ble_transport_stream_enable(false);
    ble_transport_dump_abort();
    imu_log_stop();
    session_imu_run_update();
}
