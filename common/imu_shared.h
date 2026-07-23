#ifndef IMU_SHARED_H
#define IMU_SHARED_H

#include <stdint.h>

/*
 * Shared-SRAM contract between the FLPR (writer) and the app core
 * (reader).  Lives in the top 1 KB of the FLPR's SRAM, which both
 * linkers exclude:
 *   FLPR SRAM: 0x20028000 + 95 KB  (see caterpillar_nrf54l15_cpuflpr.dts)
 *   shared:    0x2003FC00 + 1 KB
 *
 * Concurrency: seqlock.  The FLPR increments `seq` to odd before
 * writing the sample and to even after; the reader retries while
 * seq is odd or changed mid-copy.  `sample_count` increments once
 * per published sample.
 */

#define IMU_SHARED_ADDR   0x2003FC00UL
#define IMU_SHARED_MAGIC  0x494D5531UL   /* "IMU1" */

struct imu_shared {
    volatile uint32_t magic;         /* IMU_SHARED_MAGIC once FLPR booted */
    volatile uint32_t seq;           /* seqlock; odd = write in progress */
    volatile uint32_t sample_count;  /* total samples published */
    volatile uint8_t  imu_ok;        /* IMU init succeeded */
    volatile uint8_t  whoami;        /* raw WHO_AM_I readback */
    volatile uint8_t  rsvd[2];

    /* Latest converted sample */
    volatile int16_t accel_mg[3];    /* ±2 g FS */
    volatile int16_t rsvd2;
    volatile int32_t gyro_mdps[3];   /* ±250 dps FS */
    volatile int32_t temp_mdegc;
};

#define IMU_SHARED ((struct imu_shared *)IMU_SHARED_ADDR)

#endif /* IMU_SHARED_H */
