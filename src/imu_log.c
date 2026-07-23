/*
 * Multi-session IMU sample log in RRAM — see imu_log.h.
 *
 * Physical layout (absolute flash offsets):
 *   0xB9000..0xBA000   RESERVED, never written: MCUboot reads the
 *                      secondary-slot image header here on every boot
 *                      and ERASES this block if it holds non-FF bytes
 *                      that aren't a valid image (verified: it wiped
 *                      an earlier directory placed at 0xB9000).  Kept
 *                      erased, MCUboot skips the slot untouched.
 *   0xBA000..0xBA200   session directory: 16 x 32 B entries
 *   0xBA200..0x163F80  record ring, part A (last 128 B reserved so log
 *                      data can never look like an MCUboot swap trailer)
 *   0x165000..0x17D000 record ring, part B (former FLPR code region)
 *
 * Records live in ONE shared ring addressed by an absolute record
 * counter (total_abs); a session is just a [start, start+count) range
 * of that counter, remembered in its directory entry.  RRAM needs no
 * erase, so entries are freely overwritten in place.
 */

#include "imu_log.h"
#include "interface/ble_interface.h"   /* ble_wall_now() */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(imu_log, LOG_LEVEL_INF);

#define FLASH_DEV  DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller))

#define REGION_A_BASE   0xB9000UL
#define REGION_A_SIZE   0xAB000UL
#define REGION_B_BASE   0x165000UL
#define REGION_B_SIZE   0x18000UL
#define TRAILER_RESERVE 128UL
#define HDR_RESERVE     0x1000UL       /* MCUboot's header erase block */

#define ENTRY_MAGIC     0x53455353UL   /* "SESS" */
#define ENTRY_SIZE      32UL
#define DIR_BYTES       (IMU_LOG_MAX_SESSIONS * ENTRY_SIZE)   /* 512 */
#define DIR_BASE        (REGION_A_BASE + HDR_RESERVE)

#define REC_SIZE        ((uint32_t)sizeof(struct imu_sample))   /* 16 */
#define A_DATA_BYTES    (REGION_A_SIZE - HDR_RESERVE - DIR_BYTES - \
                         TRAILER_RESERVE)
#define DATA_CAPACITY   (A_DATA_BYTES + REGION_B_SIZE)
#define REC_CAPACITY    (DATA_CAPACITY / REC_SIZE)

#define ACCUM_BYTES     512   /* RRAMC write-buffer sized batches */
#define DIR_REFRESH_RECS 512  /* persist rec_count every N records */

struct dir_entry {
    uint32_t magic;
    uint32_t seq;
    uint32_t start_rec;    /* absolute record index */
    uint32_t rec_count;
    uint32_t wall_start;
    uint8_t  odr_code, content, accel_fs, gyro_fs;
    uint8_t  policy;
    uint8_t  open;
    uint8_t  rsvd[2];
    uint32_t rsvd2;
};
BUILD_ASSERT(sizeof(struct dir_entry) == ENTRY_SIZE);

static const struct device *flash_dev;
static K_MUTEX_DEFINE(log_lock);

static uint32_t total_abs;       /* records ever written (ring position) */
static uint32_t seq_next = 1;

static bool     active;
static uint8_t  cur_policy;
static uint32_t cur_seq;
static uint32_t cur_start;       /* absolute */
static uint32_t cur_count;
static uint32_t cur_slot;
static uint32_t cur_limit;       /* stop-when-full: absolute stop point */
static uint32_t last_dir_refresh;

static uint8_t  accum[ACCUM_BYTES];
static uint32_t accum_fill;
static uint32_t accum_rec_base;  /* absolute record index of accum[0] */

/* ---- directory helpers --------------------------------------------------- */

static uint32_t entry_addr(uint32_t slot)
{
    return DIR_BASE + slot * ENTRY_SIZE;
}

static int entry_read(uint32_t slot, struct dir_entry *e)
{
    return flash_read(flash_dev, entry_addr(slot), e, sizeof(*e));
}

static int entry_write(uint32_t slot, const struct dir_entry *e)
{
    return flash_write(flash_dev, entry_addr(slot), e, sizeof(*e));
}

static void entry_invalidate(uint32_t slot)
{
    struct dir_entry e;

    memset(&e, 0xFF, sizeof(e));
    e.magic = 0;
    (void)entry_write(slot, &e);
}

/* Oldest absolute record still physically present in the ring */
static uint32_t ring_floor(void)
{
    return (total_abs > REC_CAPACITY) ? total_abs - REC_CAPACITY : 0;
}

/* Drop directory entries whose data has been (partly) overwritten */
static void invalidate_consumed(void)
{
    uint32_t floor = ring_floor();

    for (uint32_t i = 0; i < IMU_LOG_MAX_SESSIONS; i++) {
        struct dir_entry e;

        if (active && i == cur_slot) {
            continue;
        }
        if (entry_read(i, &e) != 0 || e.magic != ENTRY_MAGIC) {
            continue;
        }
        if (e.start_rec < floor) {
            LOG_INF("session %u overwritten — dropped from directory",
                    e.seq);
            entry_invalidate(i);
        }
    }
}

static void dir_refresh_current(bool closing)
{
    struct dir_entry e;

    if (entry_read(cur_slot, &e) != 0 || e.magic != ENTRY_MAGIC) {
        return;
    }
    e.rec_count = cur_count;
    e.open = closing ? 0 : 1;
    (void)entry_write(cur_slot, &e);
    last_dir_refresh = cur_count;
}

/* ---- ring data helpers (byte offsets inside the record ring) ------------- */

static uint32_t phys_addr(uint32_t off)
{
    return (off < A_DATA_BYTES)
        ? DIR_BASE + DIR_BYTES + off
        : REGION_B_BASE + (off - A_DATA_BYTES);
}

static int phys_rw(uint32_t off, uint8_t *buf, uint32_t len, bool write)
{
    while (len > 0) {
        uint32_t span = len;
        if (off < A_DATA_BYTES && off + span > A_DATA_BYTES) {
            span = A_DATA_BYTES - off;
        }
        int ret = write
            ? flash_write(flash_dev, phys_addr(off), buf, span)
            : flash_read(flash_dev, phys_addr(off), buf, span);
        if (ret < 0) {
            return ret;
        }
        off += span; buf += span; len -= span;
    }
    return 0;
}

static void accum_flush(void)
{
    if (accum_fill == 0) {
        return;
    }
    uint32_t off = (accum_rec_base % REC_CAPACITY) * REC_SIZE;
    int ret = phys_rw(off, accum, accum_fill, true);
    if (ret < 0) {
        LOG_ERR("log flash write @%u: %d", off, ret);
    }
    accum_fill = 0;
}

/* ---- public API ---------------------------------------------------------- */

int imu_log_init(void)
{
    flash_dev = FLASH_DEV;
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("flash controller not ready");
        return -ENODEV;
    }

    /* Rebuild RAM state from the directory (sessions survive reboot) */
    int found = 0;

    for (uint32_t i = 0; i < IMU_LOG_MAX_SESSIONS; i++) {
        struct dir_entry e;

        if (entry_read(i, &e) != 0 || e.magic != ENTRY_MAGIC) {
            continue;
        }
        if (e.rec_count == 0xFFFFFFFFUL) {
            e.rec_count = 0;   /* died before the first refresh */
        }
        if (e.open) {
            e.open = 0;        /* close sessions cut off by reset */
            (void)entry_write(i, &e);
        }
        uint32_t end = e.start_rec + e.rec_count;
        if (end > total_abs) {
            total_abs = end;
        }
        if (e.seq >= seq_next) {
            seq_next = e.seq + 1;
        }
        found++;
    }

    LOG_INF("IMU log: %u KB ring (%u records), %d stored session(s), "
            "next seq %u", (unsigned)(DATA_CAPACITY / 1024),
            (unsigned)REC_CAPACITY, found, seq_next);
    return 0;
}

int imu_log_start(uint8_t policy)
{
    if (policy > IMU_LOG_POLICY_CIRCULAR) {
        return -EINVAL;
    }

    k_mutex_lock(&log_lock, K_FOREVER);
    if (active) {
        k_mutex_unlock(&log_lock);
        return -EBUSY;
    }

    struct imu_shared *sh = IMU_SHARED;

    cur_seq = seq_next++;
    cur_slot = (cur_seq - 1) % IMU_LOG_MAX_SESSIONS;
    cur_start = total_abs;
    cur_count = 0;
    cur_policy = policy;
    last_dir_refresh = 0;
    accum_fill = 0;

    /* Stop-when-full: never destroy the oldest surviving session */
    uint32_t oldest = cur_start;
    for (uint32_t i = 0; i < IMU_LOG_MAX_SESSIONS; i++) {
        struct dir_entry e;
        if (i != cur_slot && entry_read(i, &e) == 0 &&
            e.magic == ENTRY_MAGIC && e.start_rec < oldest) {
            oldest = e.start_rec;
        }
    }
    cur_limit = oldest + REC_CAPACITY;

    struct dir_entry e = {
        .magic = ENTRY_MAGIC,
        .seq = cur_seq,
        .start_rec = cur_start,
        .rec_count = 0,
        .wall_start = ble_wall_now(),
        .odr_code = sh->cfg_odr,
        .content = sh->cfg_content,
        .accel_fs = sh->cfg_accel_fs,
        .gyro_fs = sh->cfg_gyro_fs,
        .policy = policy,
        .open = 1,
    };
    int ret = entry_write(cur_slot, &e);
    if (ret < 0) {
        LOG_ERR("session entry write: %d", ret);
        k_mutex_unlock(&log_lock);
        return ret;
    }

    active = true;
    k_mutex_unlock(&log_lock);
    LOG_INF("IMU log session %u start (policy %u, wall %u)",
            cur_seq, policy, e.wall_start);
    return 0;
}

void imu_log_stop(void)
{
    k_mutex_lock(&log_lock, K_FOREVER);
    if (active) {
        accum_flush();
        dir_refresh_current(true);
        active = false;
        LOG_INF("IMU log session %u stop: %u records", cur_seq, cur_count);
    }
    k_mutex_unlock(&log_lock);
}

void imu_log_erase(void)
{
    imu_log_stop();

    k_mutex_lock(&log_lock, K_FOREVER);
    uint8_t ff[DIR_BYTES];
    memset(ff, 0xFF, sizeof(ff));
    (void)flash_write(flash_dev, DIR_BASE, ff, sizeof(ff));
    total_abs = 0;
    seq_next = 1;
    cur_count = 0;
    k_mutex_unlock(&log_lock);
    LOG_INF("IMU log directory erased");
}

bool imu_log_active(void)             { return active; }
uint8_t imu_log_policy(void)          { return cur_policy; }
uint32_t imu_log_capacity_bytes(void) { return DATA_CAPACITY; }

uint32_t imu_log_bytes_stored(void)
{
    uint32_t recs = (cur_count > REC_CAPACITY) ? REC_CAPACITY : cur_count;
    return recs * REC_SIZE;
}

uint32_t imu_log_records_total(void) { return cur_count; }

void imu_log_append(const struct imu_sample *s, uint32_t n)
{
    if (!active) {
        return;
    }

    k_mutex_lock(&log_lock, K_FOREVER);
    for (uint32_t i = 0; i < n; i++) {
        if (cur_policy == IMU_LOG_POLICY_STOP && total_abs >= cur_limit) {
            accum_flush();
            dir_refresh_current(true);
            active = false;
            LOG_WRN("IMU log session %u full (%u records) — stopped",
                    cur_seq, cur_count);
            k_mutex_unlock(&log_lock);
            return;
        }

        if (accum_fill == 0) {
            accum_rec_base = total_abs;
        }
        memcpy(&accum[accum_fill], &s[i], REC_SIZE);
        accum_fill += REC_SIZE;
        total_abs++;
        cur_count++;

        /* Flush on a full batch or at the physical wrap point */
        if (accum_fill == ACCUM_BYTES ||
            (total_abs % REC_CAPACITY) == 0) {
            accum_flush();
        }

        if (cur_count - last_dir_refresh >= DIR_REFRESH_RECS) {
            dir_refresh_current(false);
            invalidate_consumed();   /* circular may eat old sessions */
        }
    }
    k_mutex_unlock(&log_lock);
}

int imu_log_session_list(struct imu_log_session *out, int max)
{
    struct { struct imu_log_session s; uint32_t sort; } tmp[IMU_LOG_MAX_SESSIONS];
    int n = 0;

    k_mutex_lock(&log_lock, K_FOREVER);
    accum_flush();
    uint32_t floor = ring_floor();

    for (uint32_t i = 0; i < IMU_LOG_MAX_SESSIONS; i++) {
        struct dir_entry e;

        if (entry_read(i, &e) != 0 || e.magic != ENTRY_MAGIC) {
            continue;
        }
        uint32_t count = (active && i == cur_slot) ? cur_count : e.rec_count;
        uint32_t start = e.start_rec;
        uint32_t end = start + count;
        if (end <= floor) {
            continue;               /* fully overwritten */
        }
        if (start < floor) {
            start = floor;          /* clip to what survives */
        }
        tmp[n].s = (struct imu_log_session){
            .seq = e.seq, .wall_start = e.wall_start,
            .rec_count = end - start,
            .odr_code = e.odr_code, .content = e.content,
            .accel_fs = e.accel_fs, .gyro_fs = e.gyro_fs,
        };
        tmp[n].sort = e.seq;
        n++;
    }
    k_mutex_unlock(&log_lock);

    /* Newest first (insertion sort, n <= 16) */
    for (int i = 1; i < n; i++) {
        for (int j = i; j > 0 && tmp[j].sort > tmp[j - 1].sort; j--) {
            __typeof__(tmp[0]) t = tmp[j];
            tmp[j] = tmp[j - 1];
            tmp[j - 1] = t;
        }
    }

    int cnt = MIN(n, max);
    for (int i = 0; i < cnt; i++) {
        out[i] = tmp[i].s;
    }
    return cnt;
}

int imu_log_read_session(uint32_t seq, uint32_t offset,
                         void *buf, uint32_t len)
{
    uint8_t *out = buf;
    uint32_t done = 0;

    k_mutex_lock(&log_lock, K_FOREVER);
    accum_flush();

    struct dir_entry e;
    bool found = false;

    for (uint32_t i = 0; i < IMU_LOG_MAX_SESSIONS; i++) {
        if (entry_read(i, &e) == 0 && e.magic == ENTRY_MAGIC &&
            e.seq == seq) {
            if (active && i == cur_slot) {
                e.rec_count = cur_count;
            }
            found = true;
            break;
        }
    }
    if (!found) {
        k_mutex_unlock(&log_lock);
        return 0;
    }

    uint32_t floor = ring_floor();
    uint32_t start = e.start_rec;
    uint32_t end = e.start_rec + e.rec_count;
    if (start < floor) {
        start = floor;
    }
    if (end <= start) {
        k_mutex_unlock(&log_lock);
        return 0;
    }

    uint32_t avail = (end - start) * REC_SIZE;
    if (offset >= avail) {
        k_mutex_unlock(&log_lock);
        return 0;
    }
    len = MIN(len, avail - offset);

    uint32_t abs_byte = start * REC_SIZE + offset;
    while (len > 0) {
        uint32_t ring_off = abs_byte % DATA_CAPACITY;
        uint32_t span = MIN(len, DATA_CAPACITY - ring_off);
        int ret = phys_rw(ring_off, out, span, false);
        if (ret < 0) {
            k_mutex_unlock(&log_lock);
            return ret;
        }
        abs_byte += span; out += span; len -= span; done += span;
    }

    k_mutex_unlock(&log_lock);
    return (int)done;
}
