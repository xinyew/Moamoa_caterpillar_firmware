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
 *   0xBA000..0xBA220   directory meta (32 B: layout + firmware stamp)
 *                      + 16 x 32 B session entries.  A DFU upload
 *                      overwrites this with image bytes; on the first
 *                      boot of ANY new/reflashed firmware the stamp
 *                      mismatches and every session slot is wiped.
 *   0xBA220..0x163F80  record ring (last 128 B of the slot reserved so
 *                      log data can never look like an MCUboot trailer)
 *
 * The ex-FLPR region at 0x165000 is deliberately NOT used: the app
 * core's flash device (cpuapp_rram) ends at 0x165000, so every access
 * beyond it fails with -EINVAL — writes there were silently lost and
 * dump reads wedged at the boundary (verified on hardware, "stuck at
 * 78-83%").  Extending the device range would change the partition
 * map and break OTA compatibility; 679 KB in-range is the honest size.
 *
 * Records live in ONE ring addressed by an absolute record counter
 * (total_abs); a session is just a [start, start+count) range of that
 * counter, remembered in its directory entry.  RRAM needs no erase,
 * so entries are freely overwritten in place.
 */

#include "imu_log.h"
#include "settings/settings_store.h"
#include "app/session.h"                    /* session_wall_now() */
#include "ble/ble_transport.h"    /* ble_msg() */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <app_version.h>
#include <string.h>

LOG_MODULE_REGISTER(imu_log, LOG_LEVEL_INF);

#define FLASH_DEV  DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller))

#define REGION_A_BASE   0xB9000UL
#define REGION_A_SIZE   0xAB000UL
#define TRAILER_RESERVE 128UL
#define HDR_RESERVE     0x1000UL       /* MCUboot's header erase block */

#define ENTRY_MAGIC     0x53455353UL   /* "SESS" */
#define ENTRY_SIZE      32UL
#define DIR_BASE        (REGION_A_BASE + HDR_RESERVE)
/* meta block + session entries */
#define DIR_BYTES       ((IMU_LOG_MAX_SESSIONS + 1) * ENTRY_SIZE)

#define META_MAGIC      0x474F4C43UL   /* "CLOG" */
#define META_LAYOUT     2              /* v2: single-region ring */

struct dir_meta {
    uint32_t magic;
    uint32_t layout;
    uint32_t fw_version;   /* 0x00MMmmpp of the firmware that owns it */
    uint32_t rsvd[5];
};
BUILD_ASSERT(sizeof(struct dir_meta) == ENTRY_SIZE);

#define REC_SIZE        ((uint32_t)sizeof(struct imu_sample))   /* 16 */
#define DATA_CAPACITY   (REGION_A_SIZE - HDR_RESERVE - DIR_BYTES - \
                         TRAILER_RESERVE)
#define REC_CAPACITY    (DATA_CAPACITY / REC_SIZE)
#define DATA_BASE       (DIR_BASE + DIR_BYTES)

/* Every flash op waits for an MPSL radio timeslot
 * (SOC_FLASH_NRF_RADIO_SYNC_MPSL) — measured ~30 ms per call while
 * BLE streams.  Writes therefore run in a dedicated writer thread fed
 * from a big SRAM staging ring, in 4 KB batches so one timeslot wait
 * amortizes over 256 records.  The pump-side append is a pure memcpy.
 */
#define STAGE_N          1536  /* staged records (24 KB, ~230 ms @6.66 kHz) */
#define BATCH_RECS       256   /* records per flash_write (4 KB) */
#define DIR_REFRESH_RECS 512   /* persist rec_count every N records */

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
static bool     session_detached;
static uint32_t cur_seq;
static uint32_t cur_start;       /* absolute */
static uint32_t cur_count;
static uint32_t cur_slot;
static uint32_t last_dir_refresh;

/* SPSC staging ring: producer = pump (imu_log_append), consumer =
 * writer thread.  Same-core, so volatile indices suffice.
 */
static struct imu_sample stage[STAGE_N];
static volatile uint32_t stage_head, stage_tail;
static uint32_t log_dropped;         /* staged ring full — samples lost */
static struct imu_sample batch[BATCH_RECS];
static K_SEM_DEFINE(writer_wake, 0, 1);

/* ---- directory helpers --------------------------------------------------- */

static uint32_t entry_addr(uint32_t slot)
{
    return DIR_BASE + (slot + 1) * ENTRY_SIZE;   /* slot 0 after meta */
}

#define FW_VERSION_WORD  (((uint32_t)APP_VERSION_MAJOR << 16) | \
                          ((uint32_t)APP_VERSION_MINOR << 8) | \
                          (uint32_t)APP_PATCHLEVEL)

/* Wipe every session slot and stamp the directory as owned by the
 * running firmware.
 */
static void dir_wipe_and_stamp(void)
{
    uint8_t ff[DIR_BYTES];
    struct dir_meta meta = {
        .magic = META_MAGIC,
        .layout = META_LAYOUT,
        .fw_version = FW_VERSION_WORD,
    };
    memset(&meta.rsvd, 0xFF, sizeof(meta.rsvd));

    memset(ff, 0xFF, sizeof(ff));
    (void)flash_write(flash_dev, DIR_BASE, ff, sizeof(ff));
    (void)flash_write(flash_dev, DIR_BASE, &meta, sizeof(meta));
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

/* Drop directory entries whose data the write head has reached; the
 * BLE message makes the removal visible in the GUI console.
 */
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
            entry_invalidate(i);
            ble_msg("session #%u overwritten by session #%u — removed",
                    e.seq, cur_seq);
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

static int phys_rw(uint32_t off, uint8_t *buf, uint32_t len, bool write)
{
    return write
        ? flash_write(flash_dev, DATA_BASE + off, buf, len)
        : flash_read(flash_dev, DATA_BASE + off, buf, len);
}

/* ---- writer thread ------------------------------------------------------- */

/* Drain up to BATCH_RECS staged records into flash as ONE write call
 * (one radio-timeslot wait).  Runs with log_lock held.
 */
static void writer_drain_batch(void)
{
    uint32_t avail = stage_head - stage_tail;
    uint32_t n = MIN(avail, (uint32_t)BATCH_RECS);

    if (n == 0) {
        return;
    }

    /* Clamp to the flash-ring wrap so the write stays contiguous */
    uint32_t ring_pos = total_abs % REC_CAPACITY;

    n = MIN(n, REC_CAPACITY - ring_pos);

    for (uint32_t i = 0; i < n; i++) {
        batch[i] = stage[(stage_tail + i) % STAGE_N];
    }

    int ret = phys_rw(ring_pos * REC_SIZE, (uint8_t *)batch,
                      n * REC_SIZE, true);
    if (ret < 0) {
        LOG_ERR("log flash write @%u: %d", ring_pos * REC_SIZE, ret);
    }

    stage_tail = stage_tail + n;
    total_abs += n;
    cur_count += n;

    if (cur_count - last_dir_refresh >= DIR_REFRESH_RECS) {
        dir_refresh_current(false);
        invalidate_consumed();   /* circular may eat old sessions */
    }
}

static void writer_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        k_sem_take(&writer_wake, K_MSEC(20));

        while (active && stage_head != stage_tail) {
            k_mutex_lock(&log_lock, K_FOREVER);
            writer_drain_batch();
            k_mutex_unlock(&log_lock);
        }
    }
}

K_THREAD_DEFINE(imu_log_writer_tid, 2048, writer_thread,
                NULL, NULL, NULL, 6, 0, 0);

/* Block until the writer has persisted everything staged so far
 * (bounded); used by stop so directory counts are final.
 */
static void writer_flush(void)
{
    for (int i = 0; i < 200 && stage_head != stage_tail; i++) {
        k_sem_give(&writer_wake);
        k_msleep(5);
    }
}

/* ---- public API ---------------------------------------------------------- */

int imu_log_init(void)
{
    flash_dev = FLASH_DEV;
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("flash controller not ready");
        return -ENODEV;
    }

    /* A directory is only trusted if it was written by THIS firmware:
     * a DFU upload overwrites it with image bytes, and even a same-
     * layout directory from older firmware describes data the upload
     * may have destroyed.  Anything else -> wipe all session slots.
     */
    struct dir_meta meta;

    if (flash_read(flash_dev, DIR_BASE, &meta, sizeof(meta)) != 0 ||
        meta.magic != META_MAGIC || meta.layout != META_LAYOUT ||
        meta.fw_version != FW_VERSION_WORD) {
        dir_wipe_and_stamp();
        LOG_INF("IMU log: directory wiped (new firmware v%u.%u.%u)",
                APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_PATCHLEVEL);
        LOG_INF("IMU log: %u KB ring (%u records), 0 stored sessions",
                (unsigned)(DATA_CAPACITY / 1024), (unsigned)REC_CAPACITY);
        return 0;
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
    ARG_UNUSED(policy);   /* storage is always circular */

    k_mutex_lock(&log_lock, K_FOREVER);
    if (active) {
        k_mutex_unlock(&log_lock);
        return -EBUSY;
    }

    cur_seq = seq_next++;
    cur_slot = (cur_seq - 1) % IMU_LOG_MAX_SESSIONS;
    cur_start = total_abs;
    cur_count = 0;
    last_dir_refresh = 0;
    stage_tail = stage_head;     /* discard any stale staged records */
    log_dropped = 0;

    /* Stamp the header from the persisted settings — the shared block
     * may still read content=0 at this instant (on-demand sampling is
     * enabled right after start).
     */
    struct dir_entry e = {
        .magic = ENTRY_MAGIC,
        .seq = cur_seq,
        .start_rec = cur_start,
        .rec_count = 0,
        .wall_start = session_wall_now(),
        .odr_code = (uint8_t)settings_get(SETTING_IMU_ODR_CODE),
        .content = (uint8_t)settings_get(SETTING_IMU_CONTENT),
        .accel_fs = (uint8_t)settings_get(SETTING_IMU_ACCEL_FS),
        .gyro_fs = (uint8_t)settings_get(SETTING_IMU_GYRO_FS),
        .policy = IMU_LOG_POLICY_CIRCULAR,
        .open = 1,
    };
    int ret = entry_write(cur_slot, &e);
    if (ret < 0) {
        LOG_ERR("session entry write: %d", ret);
        k_mutex_unlock(&log_lock);
        return ret;
    }

    /* Reusing this directory slot means its 16-sessions-ago occupant
     * disappears from the list too — surface that like an overwrite.
     */
    active = true;
    k_mutex_unlock(&log_lock);
    LOG_INF("IMU log session %u start (wall %u)", cur_seq, e.wall_start);
    return 0;
}

void imu_log_stop(void)
{
    if (!active) {
        return;
    }

    /* Let the writer persist everything staged, then finalize */
    writer_flush();

    k_mutex_lock(&log_lock, K_FOREVER);
    if (active) {
        active = false;
        session_detached = false;
        dir_refresh_current(true);
        LOG_INF("IMU log session %u stop: %u records (%u dropped)",
                cur_seq, cur_count, log_dropped);
    }
    k_mutex_unlock(&log_lock);
}

void imu_log_erase(void)
{
    imu_log_stop();

    k_mutex_lock(&log_lock, K_FOREVER);
    dir_wipe_and_stamp();
    total_abs = 0;
    seq_next = 1;
    cur_count = 0;
    k_mutex_unlock(&log_lock);
    LOG_INF("IMU log directory erased");
}

bool imu_log_active(void)             { return active; }
uint8_t imu_log_policy(void)          { return IMU_LOG_POLICY_CIRCULAR; }

void imu_log_mark_detached(void)      { session_detached = true; }
bool imu_log_detached(void)           { return active && session_detached; }
uint32_t imu_log_capacity_bytes(void) { return DATA_CAPACITY; }

uint32_t imu_log_bytes_stored(void)
{
    uint32_t recs = (cur_count > REC_CAPACITY) ? REC_CAPACITY : cur_count;
    return recs * REC_SIZE;
}

uint32_t imu_log_records_total(void) { return cur_count; }
uint32_t imu_log_write_dropped(void) { return log_dropped; }

void imu_log_append(const struct imu_sample *s, uint32_t n)
{
    if (!active) {
        return;
    }

    /* Pump context: stage only — no mutex, no flash, no blocking.
     * The writer thread persists in radio-timeslot-friendly batches.
     */
    for (uint32_t i = 0; i < n; i++) {
        if (stage_head - stage_tail >= STAGE_N) {
            log_dropped++;           /* flash can't keep up — counted */
            continue;
        }
        stage[stage_head % STAGE_N] = s[i];
        stage_head = stage_head + 1;
    }
    k_sem_give(&writer_wake);
}

int imu_log_session_list(struct imu_log_session *out, int max)
{
    struct { struct imu_log_session s; uint32_t sort; } tmp[IMU_LOG_MAX_SESSIONS];
    int n = 0;

    k_mutex_lock(&log_lock, K_FOREVER);
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
