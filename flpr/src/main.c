/*
 * Caterpillar FLPR — dedicated IMU sampler.
 *
 * Owns the ASM330LHHTR on spi21 (DRDY-paced) and publishes each
 * sample into the shared-SRAM block for the app core (see
 * common/imu_shared.h).  Runs independently of the ARM core's BLE
 * workload.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/barrier.h>
#include <app_version.h>

#include "driver_asm330lhh.h"
#include "common/imu_shared.h"

/* Boot breadcrumbs (diagnostic): stage markers in the shared block so
 * the app-side debugger can see how far FLPR boot progressed.
 */
static int flpr_crumb_earliest(void)
{
    *(volatile uint32_t *)IMU_SHARED_ADDR = 0xAA000000;
    return 0;
}
SYS_INIT(flpr_crumb_earliest, EARLY, 0);

static int flpr_crumb_early(void)
{
    *(volatile uint32_t *)IMU_SHARED_ADDR = 0xAA000001;
    return 0;
}
SYS_INIT(flpr_crumb_early, PRE_KERNEL_1, 0);

static int flpr_crumb_post(void)
{
    *(volatile uint32_t *)IMU_SHARED_ADDR = 0xAA000002;
    return 0;
}
SYS_INIT(flpr_crumb_post, POST_KERNEL, 99);

int main(void)
{
    struct imu_shared *sh = IMU_SHARED;

    *(volatile uint32_t *)IMU_SHARED_ADDR = 0xAA000003;

    sh->magic = 0;
    sh->seq = 0;
    sh->sample_count = 0;
    sh->imu_ok = 0;

    int ret = drv_asm330lhh_init();
    sh->imu_ok = (ret == 0) ? 1 : 0;
    sh->whoami = drv_asm330lhh_whoami();
    sh->flpr_version = ((uint32_t)APP_VERSION_MAJOR << 16) |
                       ((uint32_t)APP_VERSION_MINOR << 8) |
                       (uint32_t)APP_PATCHLEVEL;   /* from flpr/VERSION */
    barrier_dmem_fence_full();
    sh->magic = IMU_SHARED_MAGIC;   /* app may trust the block now */

    if (ret < 0) {
        /* Stay parked; magic+imu_ok=0 tells the app "FLPR alive, IMU dead" */
        while (1) {
            k_msleep(1000);
        }
    }

    struct asm330lhh_data d;
    while (1) {
        if (drv_asm330lhh_wait_data(500) != 0) {
            continue;
        }
        if (drv_asm330lhh_read(&d) != 0) {
            continue;
        }

        sh->seq++;                       /* odd: write in progress */
        barrier_dmem_fence_full();
        sh->accel_mg[0] = d.accel_x;
        sh->accel_mg[1] = d.accel_y;
        sh->accel_mg[2] = d.accel_z;
        sh->gyro_mdps[0] = d.gyro_x;
        sh->gyro_mdps[1] = d.gyro_y;
        sh->gyro_mdps[2] = d.gyro_z;
        sh->temp_mdegc = d.temp;
        barrier_dmem_fence_full();
        sh->seq++;                       /* even: stable */
        sh->sample_count++;
    }
}
