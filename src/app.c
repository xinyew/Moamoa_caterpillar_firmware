/*
 * Application lifecycle — boot sequence + health monitor.
 *
 * Boot leaves the motor idle (rail off, driver asleep, duty 0): every
 * run is started explicitly over BLE by the GUI or script.  IMU
 * sampling runs on the FLPR coprocessor, on demand; the pump thread
 * drains its shared-SRAM ring into the flash log and the live stream.
 */

#include "app.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>
#include <app_version.h>

#include "drivers/driver_stbb1_apur.h"
#include "drivers/driver_drv8212.h"
#include "drivers/max5419.h"
#include "drivers/driver_pwm.h"
#include "drivers/driver_led.h"
#include "drivers/driver_vdc_sense.h"
#include "interface/ble_interface.h"
#include "interface/ble_transport.h"
#include "session.h"
#include "imu_pump.h"
#include "imu_log.h"
#include "settings_store.h"
#include "flpr_launch.h"
#include "common/imu_shared.h"

LOG_MODULE_REGISTER(caterpillar_app, LOG_LEVEL_DBG);

/* Boot reset cause, exposed via the BLE status characteristic */
uint32_t app_reset_cause;

void app_init(void)
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

    /* Persisted settings (registry generated from settings.yml) */
    settings_store_init();

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

    /* MAX5419LETA digipot — pre-program the persisted VDC before any
     * enable (never the digipot's power-on default of ~1.0 V)
     */
    float vdc_v = (float)settings_get(SETTING_MOTOR_VDC_MV) / 1000.0f;

    if (max5419_init() != 0 || max5419_set_voltage(vdc_v) != 0) {
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
    (void)drv_pwm_set_frequency(settings_get(SETTING_MOTOR_FREQ_HZ));

    /* IMU flash log + ring pump */
    if (imu_log_init() < 0) {
        LOG_ERR("Failed to init IMU log");
    }
    (void)imu_pump_init();

    /* BLE GATT server — full control + data surface */
    if (ble_interface_init() < 0) {
        LOG_ERR("Failed to init BLE");
    }

    /* Boot marker into the tier-2 log ring (queryable via 0xFFF0
     * even though nobody is connected yet)
     */
    ble_msg("boot: fw v%u.%u.%u, reset cause 0x%08x",
            APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_PATCHLEVEL,
            app_reset_cause);

    /* Alive indicator: 3 × 3 ms flashes per second (timer-driven) */
    drv_led_blink_start();
    if (settings_get(SETTING_LED_ENABLED) == 0) {
        drv_led_set_enabled(false);
    }
}

void app_run(void)
{
    /* Health monitor: IMU pipeline problems become BLE messages (and
     * local log lines).
     */
    struct imu_shared *sh = IMU_SHARED;
    bool flpr_announced = false;
    uint32_t last_drained = 0;
    uint32_t last_overrun = 0;
    uint32_t last_wdrop = 0;
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

            /* Arbitrate on-demand sampling with the persisted config
             * (no consumer at boot -> sensor stays powered down)
             */
            ble_stream_set_preview(settings_get(SETTING_PREVIEW_HZ));
            session_imu_run_update();
        }

        uint32_t drained = imu_pump_drained();
        uint32_t overrun = imu_pump_overrun();

        if (overrun != last_overrun) {
            ble_msg("IMU ring overrun: %u samples lost (total %u)",
                    overrun - last_overrun, overrun);
            last_overrun = overrun;
        }

        uint32_t wdrop = imu_log_write_dropped();

        if (wdrop > last_wdrop) {
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
            } else if (drained == last_drained && session_imu_demand()) {
                /* Stalled only counts when something WANTS data —
                 * idle (powered-down) sampling is normal
                 */
                last_warn = now;
                ble_msg("IMU samples stalled at %u", drained);
            }
        }
        last_drained = drained;
    }
}
