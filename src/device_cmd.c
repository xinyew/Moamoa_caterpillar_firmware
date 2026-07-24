/*
 * Command work-queue — see device_cmd.h.
 */

#include "device_cmd.h"
#include "drivers/max5419.h"
#include "drivers/driver_vdc_sense.h"
#include "imu_log.h"
#include "settings_store.h"
#include "interface/ble_interface.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(device_cmd, LOG_LEVEL_INF);

#define CMD_QUEUE_DEPTH 8

K_MSGQ_DEFINE(cmd_msgq, sizeof(struct device_cmd), CMD_QUEUE_DEPTH, 4);

int device_cmd_submit(const struct device_cmd *cmd)
{
    return k_msgq_put(&cmd_msgq, cmd, K_NO_WAIT);
}

static void execute(const struct device_cmd *cmd)
{
    switch (cmd->type) {
    case DEVICE_CMD_SET_VOLT_MV: {
        int ret = max5419_set_voltage((float)cmd->volt_mv / 1000.0f);

        if (ret < 0) {
            ble_msg("VDC set to %u mV failed (%d)", cmd->volt_mv, ret);
            break;
        }
        settings_set(SETTING_MOTOR_VDC_MV, cmd->volt_mv);
        /* Converter settle, then verify the rail for the log */
        k_msleep(20);
        int32_t meas_mv = 0;

        if (drv_vdc_sense_read_mv(&meas_mv) == 0) {
            LOG_INF("VDC -> %u mV (measured %d mV)", cmd->volt_mv, meas_mv);
        }
        break;
    }
    case DEVICE_CMD_LOG:
        switch (cmd->log.op) {
        case 0:                          /* stop */
            imu_log_stop();
            ble_session_conn_params(false);
            ble_imu_run_update();        /* power down if stream idle */
            break;
        case 1:                          /* start (always circular) */
            if (imu_log_start(1) == 0) {
                ble_session_conn_params(true);
                ble_imu_run_update();    /* power the sensor up */
            }
            break;
        case 2:                          /* erase */
            imu_log_erase();
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void cmd_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    struct device_cmd cmd;

    while (1) {
        k_msgq_get(&cmd_msgq, &cmd, K_FOREVER);
        execute(&cmd);
    }
}

K_THREAD_DEFINE(device_cmd_tid, 2048, cmd_thread, NULL, NULL, NULL, 10, 0, 0);
