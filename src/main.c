/*
 * Caterpillar — Motor Control + IMU Firmware
 * Custom nRF54L15 board (caterpillar/nrf54l15/cpuapp)
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>

#include "drivers/driver_stbb1_apur.h"
#include "drivers/driver_drv8212.h"
#include "drivers/max5419.h"
#include "common/imu_shared.h"
#include <zephyr/sys/barrier.h>
#include "drivers/driver_pwm.h"
#include "drivers/driver_led.h"
#include "drivers/driver_vdc_sense.h"
#include "interface/ble_interface.h"

LOG_MODULE_REGISTER(caterpillar_main, LOG_LEVEL_DBG);

/* Motor rail (VDC) target — applied via MAX5419 digipot before the
 * STBB1-APUR is enabled, so the rail never runs at the digipot's
 * power-on default (midscale ≈ 1.0 V) or a stale NV value.
 */
#define MOTOR_VDC_V  3.1f

/* Boot reset cause, exposed via the BLE status characteristic */
uint32_t app_reset_cause;

int main(void)
{
    printk("\n=== Caterpillar Boot ===\n");

    /* Announce why we booted — makes brown-out reset loops visible
     * (a silent reset shows as ~65 ms of motor dropout otherwise).
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

    /* VDC sense ADC (AIN4 / P1.11) */
    if (drv_vdc_sense_init() < 0) {
        LOG_ERR("Failed to init VDC sense");
    }

    /* MAX5419LETA digipot — program VDC feedback before DCDC enable */
    bool vdc_ok = (max5419_init() == 0) &&
                  (max5419_set_voltage(MOTOR_VDC_V) == 0);

    /* STBB1-APUR DCDC converter — only enable at a known voltage */
    if (!vdc_ok) {
        LOG_ERR("Digipot voltage set failed — leaving motor rail disabled");
    } else if (drv_stbb1_apur_init() < 0) {
        LOG_ERR("Failed to enable DCDC");
        vdc_ok = false;
    }

    /* DRV8212P motor driver — wake (nSLEEP = HIGH) */
    if (drv_drv8212_init() < 0) {
        LOG_ERR("Failed to init DRV8212");
    }

    /* PWM20 — configured at default frequency, 0 % duty (coast) */
    if (drv_pwm_init() < 0) {
        LOG_ERR("Failed to init PWM");
    }

    /* Auto-start: forward drive at 50 % once all rails are up.
     * Remove this to keep the motor idle until commanded over BLE.
     */
    if (vdc_ok) {
        /* Converter settle, then verify the rail against the target */
        k_msleep(50);
        int32_t vdc_mv = 0;
        if (drv_vdc_sense_read_mv(&vdc_mv) == 0) {
            LOG_INF("VDC measured: %d mV (target %d mV)",
                    vdc_mv, (int)(MOTOR_VDC_V * 1000.0f));
        }

        drv_pwm_set_duty(0, 50);
    }

    /* BLE GATT server — remote frequency control */
    if (ble_interface_init() < 0) {
        LOG_ERR("Failed to init BLE");
    }

    /* IMU is owned by the FLPR coprocessor (launched automatically at
     * boot); samples arrive through the shared-SRAM block.
     */

    /* Alive indicator: 3 × 3 ms flashes per second (timer-driven) */
    drv_led_blink_start();

    /* Consume IMU samples published by the FLPR into shared SRAM */
    struct imu_shared *sh = IMU_SHARED;
    uint32_t last_count = 0;
    int64_t last_warn = 0;

    while (1) {
        k_msleep(80);

        uint32_t count = sh->sample_count;
        bool alive = (sh->magic == IMU_SHARED_MAGIC);

        if (!alive || !sh->imu_ok || count == last_count) {
            int64_t now = k_uptime_get();
            if (now - last_warn >= 5000) {
                last_warn = now;
                if (!alive) {
                    LOG_WRN("FLPR not running (no shared-memory magic)");
                } else if (!sh->imu_ok) {
                    LOG_WRN("FLPR up, IMU init failed (WHO_AM_I=0x%02x)",
                            sh->whoami);
                } else {
                    LOG_WRN("FLPR up but IMU samples stalled at %u", count);
                }
            }
            continue;
        }
        last_count = count;

        /* Seqlock read of the latest sample */
        int16_t a[3];
        int32_t g[3];
        int32_t temp;
        uint32_t s1, s2;
        do {
            s1 = sh->seq;
            barrier_dmem_fence_full();
            a[0] = sh->accel_mg[0]; a[1] = sh->accel_mg[1]; a[2] = sh->accel_mg[2];
            g[0] = sh->gyro_mdps[0]; g[1] = sh->gyro_mdps[1]; g[2] = sh->gyro_mdps[2];
            temp = sh->temp_mdegc;
            barrier_dmem_fence_full();
            s2 = sh->seq;
        } while ((s1 & 1) != 0 || s1 != s2);

        int32_t deg = temp / 1000;
        int32_t mdeg = temp % 1000;
        if (mdeg < 0) mdeg = -mdeg;

        printk("[%5u] A: %6d %6d %6d mg  |  G: %6d %6d %6d mdps  |  T: %d.%03d C\n",
               count, a[0], a[1], a[2], g[0], g[1], g[2],
               (int)deg, (int)mdeg);
    }
}
