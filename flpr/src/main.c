/*
 * Caterpillar FLPR — dedicated IMU sampler.
 *
 * Owns the ASM330LHHTR on spi21 (DRDY-paced) and publishes raw samples
 * into the shared-SRAM ring buffer for the app core (see
 * common/imu_shared.h).  Sampling config (ODR / content / full-scale)
 * is applied from the shared control block, so the app can reconfigure
 * it live over BLE.  Runs independently of the ARM core's BLE workload.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <app_version.h>

#include "driver_asm330lhh.h"
#include "common/imu_shared.h"

/* Poll for a new config every N samples at high rate (and on every
 * DRDY timeout), so a reconfig lands within a few ms without putting
 * a shared-SRAM read in every 150 us sample period.
 */
#define CFG_POLL_INTERVAL  32

static void apply_config(struct imu_shared *sh)
{
    uint32_t seq = sh->cfg_seq;

    if (seq == sh->cfg_applied) {
        return;
    }

    int ret = drv_asm330lhh_configure(sh->cfg_odr, sh->cfg_content,
                                      sh->cfg_accel_fs, sh->cfg_gyro_fs);
    sh->cfg_status = ret;
    barrier_dmem_fence_full();
    sh->cfg_applied = seq;
}

int main(void)
{
    struct imu_shared *sh = IMU_SHARED;

    /* Zero the whole control block (not the 32 KB ring — every slot is
     * fully written before head advances past it).
     */
    sh->magic = 0;
    barrier_dmem_fence_full();
    sh->head = 0;
    sh->tail = 0;
    sh->overrun = 0;
    sh->cfg_applied = 0;
    sh->cfg_status = 0;

    /* Boot with the sensor POWERED DOWN (content 0): sampling is
     * on-demand — the app enables it only while a log session or live
     * stream needs data.
     */
    sh->cfg_odr      = IMU_CFG_DEFAULT_ODR;
    sh->cfg_content  = 0;
    sh->cfg_accel_fs = IMU_CFG_DEFAULT_ACCEL_FS;
    sh->cfg_gyro_fs  = IMU_CFG_DEFAULT_GYRO_FS;
    sh->cfg_seq      = 1;

    int ret = drv_asm330lhh_init();
    if (ret == 0) {
        ret = drv_asm330lhh_configure(sh->cfg_odr, sh->cfg_content,
                                      sh->cfg_accel_fs, sh->cfg_gyro_fs);
    }
    sh->cfg_applied = sh->cfg_seq;
    sh->cfg_status = ret;
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

    struct asm330lhh_raw d;
    uint32_t sampled = 0;      /* total DRDY reads, including drops */
    uint32_t since_cfg_poll = 0;

    while (1) {
        if (drv_asm330lhh_wait_data(100) != 0) {
            /* Timeout: idle/powered-down sensor — check for a pending
             * reconfig so an enable takes effect within ~100 ms.
             */
            apply_config(sh);
            continue;
        }
        if (++since_cfg_poll >= CFG_POLL_INTERVAL) {
            since_cfg_poll = 0;
            apply_config(sh);
        }
        if (drv_asm330lhh_read(&d) != 0) {
            continue;
        }

        sampled++;

        uint32_t head = sh->head;
        if (head - sh->tail >= IMU_RING_N) {
            sh->overrun++;          /* ring full: consumer too slow */
            continue;
        }

        volatile struct imu_sample *s = &sh->ring[head & (IMU_RING_N - 1)];
        s->ax = d.ax; s->ay = d.ay; s->az = d.az;
        s->gx = d.gx; s->gy = d.gy; s->gz = d.gz;
        s->temp_raw = d.temp_raw;
        s->seq16 = (uint16_t)sampled;
        barrier_dmem_fence_full();  /* record visible before head moves */
        sh->head = head + 1;
    }
}
