/*
 * Caterpillar — Motor Control + IMU Firmware
 * Custom nRF54L15 board (caterpillar/nrf54l15/cpuapp)
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include "drivers/driver_stbb1_apur.h"
#include "drivers/driver_drv8212.h"
#include "drivers/driver_asm330lhh.h"
#include "drivers/max5419.h"
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

/* Heartbeat half-period: LED toggles this often while the loop runs */
#define HEARTBEAT_MS  500

int main(void)
{
    printk("\n=== Caterpillar Boot ===\n");

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

    /* ASM330LHHTR IMU */
    if (drv_asm330lhh_init() < 0) {
        LOG_ERR("Failed to init IMU");
    }

    uint32_t tick = 0;
    int64_t last_beat = k_uptime_get();
    while (1) {
        /* Heartbeat — before the IMU wait/continue so a dead IMU
         * cannot starve the blink.
         */
        int64_t now = k_uptime_get();
        if (now - last_beat >= HEARTBEAT_MS) {
            drv_led_toggle();
            last_beat = now;
        }

        /* Paced by the IMU data-ready interrupt (12.5 Hz).  The timeout
         * keeps the loop alive if the IMU is absent or wedged.
         */
        if (drv_asm330lhh_wait_data(500) != 0) {
            LOG_WRN("IMU data-ready timeout");
            continue;
        }

        struct asm330lhh_data imu;
        if (drv_asm330lhh_read(&imu) == 0) {
            int32_t deg = imu.temp / 1000;
            int32_t mdeg = imu.temp % 1000;
            if (mdeg < 0) mdeg = -mdeg;

            printk("[%5u] A: %6d %6d %6d mg  |  G: %6d %6d %6d mdps  |  T: %d.%03d C\n",
                   tick,
                   imu.accel_x, imu.accel_y, imu.accel_z,
                   imu.gyro_x,  imu.gyro_y,  imu.gyro_z,
                   (int)deg, (int)mdeg);
        }

        tick++;
    }
}
