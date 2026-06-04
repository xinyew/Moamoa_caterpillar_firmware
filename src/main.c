/*
 * Caterpillar — Motor Control + IMU Firmware
 * Custom nRF54L15 board (caterpillar/nrf54l15/cpuapp)
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include "drivers/driver_stbb1_apur.h"
#include "drivers/driver_drv8212.h"

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

    while (1) {
        printk("Heartbeat\n");
        k_msleep(5000);
    }
}
