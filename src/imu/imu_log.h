#ifndef IMU_LOG_H
#define IMU_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "common/imu_shared.h"

/*
 * On-chip IMU sample log in RRAM, multi-session.
 *
 * Storage is dual-use with OTA: the MCUboot secondary slot plus the
 * former FLPR code region (~780 KB).  A DFU upload overwrites the log
 * and vice versa (installed firmware and the OTA mechanism itself are
 * unaffected) — dump before updating.
 *
 * Layout: a 16-entry session DIRECTORY (512 B) at the region start,
 * then one shared ring of 16 B records.  Sessions append after each
 * other; the directory persists across reboots, so old sessions stay
 * listable/dumpable until erased or overwritten.  Each entry carries
 * the sampling config and the wall-clock start time (from the 0xFFEE
 * sync), so any dump is self-describing.
 *
 * Storage is ALWAYS circular: a session runs from start until stop
 * (BLE) or BLE disconnect (the app auto-closes it), never stopping for
 * space.  When the write head reaches the beginning of an older
 * session's data, that session's directory entry is removed and a BLE
 * message (0xFFEC) announces it, so the GUI list stays truthful.
 * Power loss mid-session loses at most the last ~8 KB of that
 * session's accounting (the entry is refreshed every 512 records).
 */

#define IMU_LOG_POLICY_STOP     0   /* deprecated: no longer honored */
#define IMU_LOG_POLICY_CIRCULAR 1   /* the only behavior since v1.3.3 */

#define IMU_LOG_MAX_SESSIONS    16

/* Session summary as reported to the host */
struct imu_log_session {
    uint32_t seq;         /* monotonic session id */
    uint32_t wall_start;  /* unix epoch at start (0 = clock unsynced) */
    uint32_t rec_count;   /* readable records (clipped if overwritten) */
    uint8_t  odr_code, content, accel_fs, gyro_fs;
};

int  imu_log_init(void);

int  imu_log_start(uint8_t policy);
void imu_log_stop(void);
void imu_log_erase(void);            /* wipe the whole directory */

bool     imu_log_active(void);
uint8_t  imu_log_policy(void);

/* Detached sessions (fleet mode): survive BLE disconnect and keep
 * logging untethered until an explicit stop.  Mark right after a
 * successful start; cleared by stop.
 */
void imu_log_mark_detached(void);
bool imu_log_detached(void);

uint32_t imu_log_capacity_bytes(void);
uint32_t imu_log_bytes_stored(void);     /* current/last session bytes */
uint32_t imu_log_records_total(void);    /* current/last session records */
uint32_t imu_log_write_dropped(void);    /* staged samples lost: flash
                                          * couldn't keep up (counted) */

/* Append records (pump thread only). */
void imu_log_append(const struct imu_sample *s, uint32_t n);

/* List sessions, newest first.  Returns the number written to out. */
int imu_log_session_list(struct imu_log_session *out, int max);

/* Read a session's records (oldest-first) as a flat byte space.
 * offset/len in bytes relative to the session's readable data.
 * Returns bytes read (0 at end / unknown session), negative errno.
 */
int imu_log_read_session(uint32_t seq, uint32_t offset,
                         void *buf, uint32_t len);

#endif /* IMU_LOG_H */
