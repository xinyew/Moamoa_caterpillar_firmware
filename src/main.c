/*
 * Caterpillar — Motor Control + IMU Firmware
 * Custom nRF54L15 board (caterpillar/nrf54l15/cpuapp)
 *
 * Boot leaves the motor idle (rail off, driver asleep, duty 0): every
 * run is started explicitly over BLE by the GUI or script.  IMU
 * sampling runs on the FLPR coprocessor; the pump thread drains its
 * shared-SRAM ring into the flash log and the live BLE stream.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>

#include "drivers/driver_stbb1_apur.h"
#include "drivers/driver_drv8212.h"
#include "drivers/max5419.h"
#include "common/imu_shared.h"
#include "drivers/driver_pwm.h"
#include "drivers/driver_led.h"
#include "drivers/driver_vdc_sense.h"
#include "interface/ble_interface.h"
#include "imu_pump.h"
#include "imu_log.h"
#include "flpr_launch.h"

LOG_MODULE_REGISTER(caterpillar_main, LOG_LEVEL_DBG);

/* Motor rail (VDC) pre-programmed into the digipot at boot so the rail
 * comes up at a known-safe voltage when a session enables it (never
 * the digipot's power-on default of ~1.0 V or a stale NV value).
 */
#define MOTOR_VDC_V  3.1f

/* Boot reset cause, exposed via the BLE status characteristic */
uint32_t app_reset_cause;

int main(void)
{
    printk("\n=== Caterpillar Boot ===\n");

    /* Announce why we booted — makes brown-out reset loops visible.
     * Kept in app_reset_cause for the BLE status characteristic.
     */
    if (hwinfo_get_reset_cause(&app_reset_cause) == 0) {
        LOG_INF("Reset cause: 0x%08x%s%s%s%s%s", app_reset_cause,
                (app_reset_cause & RESET_POR)      ? " POR/brownout" : "",
                (app_reset_cause & RESET_PIN)      ? " pin"          : "",
                (app_reset_cause & RESET_SOFTWARE) ? " soft"         : "",
                (app_reset_cause & RESET_WATCHDOG) ? " watchdog"     : "",
                (app_reset_cause & RESET_DEBUG)    ? " debug"        : "");
        (void)hwinfo_clear_reset_cause();
    }

    /* Status LED on — power/boot indicator until the heartbeat starts */
    if (drv_led_init() < 0) {
        LOG_ERR("Failed to init status LED");
    }

    /* Start the IMU coprocessor from the firmware embedded in this image */
    flpr_launch();

    /* VDC sense ADC (AIN4 / P1.11) */
    if (drv_vdc_sense_init() < 0) {
        LOG_ERR("Failed to init VDC sense");
    }

    /* MAX5419LETA digipot — pre-program a safe VDC before any enable */
    if (max5419_init() != 0 || max5419_set_voltage(MOTOR_VDC_V) != 0) {
        LOG_ERR("Digipot voltage set failed");
    }

    /* Motor chain boots INACTIVE: rail off, driver asleep, duty 0 */
    if (drv_stbb1_apur_init() < 0) {
        LOG_ERR("Failed to init DCDC enable pin");
    }
    if (drv_drv8212_init() < 0) {
        LOG_ERR("Failed to init DRV8212");
    }
    if (drv_pwm_init() < 0) {
        LOG_ERR("Failed to init PWM");
    }

    /* Prime the drive waveform at 50 % duty (the fixed drive scheme —
     * amplitude is controlled via VDC, frequency via 0xFFE1).  With the
     * rail off and the driver asleep this moves nothing; enabling them
     * over BLE starts the motor.  Without this, duty stays 0 and the
     * rail/driver switches appear dead.
     */
    drv_pwm_set_duty(0, 50);

    /* IMU flash log + ring pump */
    if (imu_log_init() < 0) {
        LOG_ERR("Failed to init IMU log");
    }
    (void)imu_pump_init();

    /* BLE GATT server — full control + data surface */
    if (ble_interface_init() < 0) {
        LOG_ERR("Failed to init BLE");
    }

    /* Alive indicator: 3 × 3 ms flashes per second (timer-driven) */
    drv_led_blink_start();

    /* Health monitor: IMU pipeline problems become BLE messages (and
     * local log lines).  The per-sample console stream is gone — data
     * now flows through the flash log and the 0xFFE9 live stream.
     */
    struct imu_shared *sh = IMU_SHARED;
    bool flpr_announced = false;
    uint32_t last_drained = 0;
    uint32_t last_overrun = 0;
    int64_t last_warn = 0;

    while (1) {
        k_msleep(1000);

        bool alive = (sh->magic == IMU_SHARED_MAGIC);
        int64_t now = k_uptime_get();

        if (alive && !flpr_announced) {
            flpr_announced = true;
            uint32_t v = sh->flpr_version;
            LOG_INF("FLPR running, fw v%u.%u.%u",
                    (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
        }

        uint32_t drained = imu_pump_drained();
        uint32_t overrun = imu_pump_overrun();

        if (overrun != last_overrun) {
            ble_msg("IMU ring overrun: %u samples lost (total %u)",
                    overrun - last_overrun, overrun);
            last_overrun = overrun;
        }

        static uint32_t last_wdrop;
        uint32_t wdrop = imu_log_write_dropped();

        if (wdrop != last_wdrop && wdrop > last_wdrop) {
            ble_msg("log write backlog: %u samples lost (total %u)",
                    wdrop - last_wdrop, wdrop);
        }
        last_wdrop = wdrop;

        if (now - last_warn >= 5000) {
            if (!alive) {
                last_warn = now;
                ble_msg("FLPR not running (no shared-memory magic)");
            } else if (!sh->imu_ok) {
                last_warn = now;
                ble_msg("IMU init failed (WHO_AM_I=0x%02x)", sh->whoami);
            } else if (drained == last_drained) {
                last_warn = now;
                ble_msg("IMU samples stalled at %u", drained);
            }
        }
        last_drained = drained;
    }
}
