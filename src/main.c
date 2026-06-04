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

LOG_MODULE_REGISTER(caterpillar_main, LOG_LEVEL_DBG);

int main(void)
{
    printk("\n=== Caterpillar Boot ===\n");

    /* STBB1-APUR DCDC converter enable */
    if (drv_stbb1_apur_init() < 0) {
        LOG_ERR("Failed to enable DCDC");
    }

    /* DRV8212P motor driver — enable (LOW = on) */
    if (drv_drv8212_init() < 0) {
        LOG_ERR("Failed to init DRV8212");
    }

    /* ASM330LHHTR IMU */
    if (drv_asm330lhh_init() < 0) {
        LOG_ERR("Failed to init IMU");
    }

    uint32_t tick = 0;
    while (1) {
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
        k_msleep(200);
    }
}
