#ifndef IMU_LOG_H
#define IMU_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "common/imu_shared.h"

/*
 * On-chip IMU sample log in RRAM.
 *
 * Storage is dual-use: the MCUboot secondary slot plus the former FLPR
 * code region (~779 KB total).  Logging and a *staged* OTA image are
 * mutually exclusive — starting a log session invalidates any uploaded
 * (not yet installed) image, and a DFU upload overwrites the log.
 * Installed firmware and the OTA mechanism itself are unaffected.
 *
 * A 32 B session header (magic "CLG1") holds the sampling config so a
 * dump is self-describing.  Records are struct imu_sample (16 B).
 * Fill policy: stop-when-full keeps the oldest data and halts;
 * circular overwrites the oldest.  Counters live in RAM: a log is
 * dumpable until the device reboots or a new session starts.
 */

#define IMU_LOG_POLICY_STOP     0
#define IMU_LOG_POLICY_CIRCULAR 1

#define IMU_LOG_HDR_SIZE   32
#define IMU_LOG_HDR_MAGIC  0x31474C43UL   /* "CLG1" */

struct imu_log_header {
    uint32_t magic;
    uint32_t session_id;     /* increments per start */
    uint8_t  odr_code;
    uint8_t  content;
    uint8_t  accel_fs;
    uint8_t  gyro_fs;
    uint8_t  policy;
    uint8_t  record_size;    /* 16 */
    uint16_t rsvd0;
    uint32_t start_uptime_s;
    uint32_t rsvd1[3];
};

int  imu_log_init(void);

/* Start a session, stamping the current sampling config into the
 * header.  Returns 0 or negative errno.
 */
int  imu_log_start(uint8_t policy);
void imu_log_stop(void);
void imu_log_erase(void);            /* invalidate header, reset counters */

bool     imu_log_active(void);
uint8_t  imu_log_policy(void);
uint32_t imu_log_capacity_bytes(void);   /* record space, excl. header */
uint32_t imu_log_bytes_stored(void);     /* record bytes currently held */
uint32_t imu_log_records_total(void);    /* written incl. overwritten */

/* Append records (pump thread only). */
void imu_log_append(const struct imu_sample *s, uint32_t n);

/* Read the log as a flat logical space for dumping:
 * offset 0..31 = session header, then records oldest-first.
 * Returns bytes read (0 at end), negative errno on error.
 */
int imu_log_read(uint32_t offset, void *buf, uint32_t len);

#endif /* IMU_LOG_H */
