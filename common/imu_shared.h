#ifndef IMU_SHARED_H
#define IMU_SHARED_H

#include <stdint.h>

/*
 * Shared-SRAM contract between the FLPR (producer) and the app core
 * (consumer), v2: a control block plus an SPSC ring buffer of raw
 * samples.  Lives in the top 40 KB of SRAM, which both linkers exclude:
 *   app  SRAM: 0x20000000 + 160 KB               (cpuapp dtsi)
 *   FLPR SRAM: 0x20028000 +  56 KB               (cpuflpr dts)
 *   shared:    0x20036000 +  40 KB  = this block
 *
 * Concurrency (single producer / single consumer, absolute counters):
 *   - FLPR writes ring[head % N], barrier, head++.  Never blocks: if
 *     the ring is full (head - tail == N) the sample is dropped and
 *     `overrun` incremented.
 *   - App reads ring[tail % N] while tail != head, barrier, tail++.
 *   - u32 counters wrap naturally (2's-complement subtraction).
 *
 * Reconfiguration: app fills the cfg_* fields, then increments cfg_seq
 * (last).  FLPR polls cfg_seq between samples, re-programs the sensor,
 * then echoes the seq in cfg_applied with the result in cfg_status.
 */

#define IMU_SHARED_ADDR   0x20036000UL
#define IMU_SHARED_MAGIC  0x494D5532UL   /* "IMU2" */

#define IMU_RING_ORDER    11
#define IMU_RING_N        (1UL << IMU_RING_ORDER)   /* 2048 records, 32 KB */

/* cfg_odr: ASM330LHH ODR register code (shared by CTRL1_XL/CTRL2_G) */
#define IMU_ODR_12HZ5     1
#define IMU_ODR_26HZ      2
#define IMU_ODR_52HZ      3
#define IMU_ODR_104HZ     4
#define IMU_ODR_208HZ     5
#define IMU_ODR_416HZ     6
#define IMU_ODR_833HZ     7
#define IMU_ODR_1660HZ    8
#define IMU_ODR_3330HZ    9
#define IMU_ODR_6660HZ    10

/* cfg_content bits */
#define IMU_CONTENT_ACCEL 0x01
#define IMU_CONTENT_GYRO  0x02

/* cfg_accel_fs: 0=±2g 1=±4g 2=±8g 3=±16g
 * cfg_gyro_fs:  0=±250 1=±500 2=±1000 3=±2000 dps
 */

/* Raw sample record, 16 B.  Values are sensor LSB (scale from the FS
 * config on the consumer side).  Disabled-sensor fields read 0.
 * seq16 = low 16 bits of the FLPR's total *sampled* count (including
 * ring-full drops), so gaps in the published stream are detectable.
 */
struct imu_sample {
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int16_t  temp_raw;
    uint16_t seq16;
};

struct imu_shared {
    /* FLPR -> app identity (written before magic) */
    volatile uint32_t magic;         /* IMU_SHARED_MAGIC once FLPR booted */
    volatile uint32_t flpr_version;  /* 0x00MMmmpp from flpr/VERSION */
    volatile uint8_t  imu_ok;        /* IMU init succeeded */
    volatile uint8_t  whoami;        /* raw WHO_AM_I readback */
    volatile uint8_t  rsvd0[2];

    /* app -> FLPR sampling config; bump cfg_seq LAST */
    volatile uint32_t cfg_seq;
    volatile uint8_t  cfg_odr;       /* IMU_ODR_* code */
    volatile uint8_t  cfg_content;   /* IMU_CONTENT_* bits */
    volatile uint8_t  cfg_accel_fs;
    volatile uint8_t  cfg_gyro_fs;

    /* FLPR -> app config acknowledgement */
    volatile uint32_t cfg_applied;   /* last cfg_seq actually applied */
    volatile int32_t  cfg_status;    /* 0 = ok, <0 = -errno from apply */

    /* SPSC ring counters (absolute record counts) */
    volatile uint32_t head;          /* records published (FLPR) */
    volatile uint32_t tail;          /* records consumed  (app)  */
    volatile uint32_t overrun;       /* records dropped, ring full */

    volatile uint32_t rsvd1[6];      /* pad control block to 64 B */

    volatile struct imu_sample ring[IMU_RING_N];
};

#define IMU_SHARED ((struct imu_shared *)IMU_SHARED_ADDR)

/* Boot default until the GUI configures a session */
#define IMU_CFG_DEFAULT_ODR       IMU_ODR_833HZ
#define IMU_CFG_DEFAULT_CONTENT   (IMU_CONTENT_ACCEL | IMU_CONTENT_GYRO)
#define IMU_CFG_DEFAULT_ACCEL_FS  0   /* ±2 g */
#define IMU_CFG_DEFAULT_GYRO_FS   0   /* ±250 dps */

#endif /* IMU_SHARED_H */
