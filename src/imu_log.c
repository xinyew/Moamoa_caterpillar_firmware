/*
 * IMU sample log in RRAM — see imu_log.h for the storage contract.
 *
 * Physical layout (absolute flash offsets):
 *   region A: 0xB9000..0x164000  = MCUboot secondary slot (684 KB);
 *             first 32 B hold the session header, the last 128 B are
 *             reserved so log data can never masquerade as an MCUboot
 *             swap trailer
 *   region B: 0x165000..0x17D000 = former FLPR code region (96 KB)
 * Records fill A (after the header) then continue into B.
 *
 * RRAM has no erase requirement, so "erase" just invalidates the
 * header; writes go through a 512 B accumulator sized to the RRAMC
 * write buffer.
 */

#include "imu_log.h"

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

#define REC_SIZE        ((uint32_t)sizeof(struct imu_sample))   /* 16 */
#define A_DATA_BYTES    (REGION_A_SIZE - IMU_LOG_HDR_SIZE - TRAILER_RESERVE)
#define DATA_CAPACITY   (A_DATA_BYTES + REGION_B_SIZE)
#define REC_CAPACITY    (DATA_CAPACITY / REC_SIZE)

#define ACCUM_BYTES     512   /* RRAMC write-buffer sized batches */

static const struct device *flash_dev;
static K_MUTEX_DEFINE(log_lock);   /* append (pump) vs read/ctl (BLE) */
static bool     active;
static uint8_t  cur_policy;
static uint32_t session_id;
static uint32_t total_recs;      /* records written, incl. overwritten */
static uint8_t  accum[ACCUM_BYTES];
static uint32_t accum_fill;
static uint32_t accum_rec_base;  /* record index of accum[0] */

/* Map a physical DATA-space byte offset (0..DATA_CAPACITY) to an
 * absolute flash offset.
 */
static uint32_t phys_addr(uint32_t off)
{
    return (off < A_DATA_BYTES)
        ? REGION_A_BASE + IMU_LOG_HDR_SIZE + off
        : REGION_B_BASE + (off - A_DATA_BYTES);
}

/* Write to DATA space, splitting across the A/B region boundary. */
static int phys_write(uint32_t off, const uint8_t *buf, uint32_t len)
{
    while (len > 0) {
        uint32_t span = len;
        if (off < A_DATA_BYTES && off + span > A_DATA_BYTES) {
            span = A_DATA_BYTES - off;
        }
        int ret = flash_write(flash_dev, phys_addr(off), buf, span);
        if (ret < 0) {
            return ret;
        }
        off += span; buf += span; len -= span;
    }
    return 0;
}

static int phys_read(uint32_t off, uint8_t *buf, uint32_t len)
{
    while (len > 0) {
        uint32_t span = len;
        if (off < A_DATA_BYTES && off + span > A_DATA_BYTES) {
            span = A_DATA_BYTES - off;
        }
        int ret = flash_read(flash_dev, phys_addr(off), buf, span);
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
    int ret = phys_write(off, accum, accum_fill);
    if (ret < 0) {
        LOG_ERR("log flash write @%u: %d", off, ret);
    }
    accum_fill = 0;
}

int imu_log_init(void)
{
    flash_dev = FLASH_DEV;
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("flash controller not ready");
        return -ENODEV;
    }
    LOG_INF("IMU log: %u KB (%u records)",
            (unsigned)(DATA_CAPACITY / 1024), (unsigned)REC_CAPACITY);
    return 0;
}

int imu_log_start(uint8_t policy)
{
    if (policy > IMU_LOG_POLICY_CIRCULAR) {
        return -EINVAL;
    }

    struct imu_shared *sh = IMU_SHARED;
    struct imu_log_header hdr = {
        .magic = IMU_LOG_HDR_MAGIC,
        .session_id = ++session_id,
        .odr_code = sh->cfg_odr,
        .content = sh->cfg_content,
        .accel_fs = sh->cfg_accel_fs,
        .gyro_fs = sh->cfg_gyro_fs,
        .policy = policy,
        .record_size = REC_SIZE,
        .start_uptime_s = (uint32_t)(k_uptime_get() / 1000),
    };

    int ret = flash_write(flash_dev, REGION_A_BASE, &hdr, sizeof(hdr));
    if (ret < 0) {
        LOG_ERR("log header write: %d", ret);
        return ret;
    }

    total_recs = 0;
    accum_fill = 0;
    accum_rec_base = 0;
    cur_policy = policy;
    active = true;
    LOG_INF("IMU log start: session %u policy %u", session_id, policy);
    return 0;
}

void imu_log_stop(void)
{
    if (!active) {
        return;
    }
    accum_flush();
    active = false;
    LOG_INF("IMU log stop: %u records", total_recs);
}

void imu_log_erase(void)
{
    imu_log_stop();

    /* Invalidate the header magic; record space needs no wiping */
    struct imu_log_header hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    (void)flash_write(flash_dev, REGION_A_BASE, &hdr, sizeof(hdr));
    total_recs = 0;
}

bool imu_log_active(void)          { return active; }
uint8_t imu_log_policy(void)       { return cur_policy; }
uint32_t imu_log_capacity_bytes(void) { return DATA_CAPACITY; }

uint32_t imu_log_bytes_stored(void)
{
    uint32_t recs = (total_recs > REC_CAPACITY) ? REC_CAPACITY : total_recs;
    return recs * REC_SIZE;
}

uint32_t imu_log_records_total(void) { return total_recs; }

void imu_log_append(const struct imu_sample *s, uint32_t n)
{
    if (!active) {
        return;
    }

    k_mutex_lock(&log_lock, K_FOREVER);
    for (uint32_t i = 0; i < n; i++) {
        if (cur_policy == IMU_LOG_POLICY_STOP &&
            total_recs >= REC_CAPACITY) {
            accum_flush();
            active = false;
            LOG_WRN("IMU log full (%u records) — stopped", total_recs);
            k_mutex_unlock(&log_lock);
            return;
        }

        if (accum_fill == 0) {
            accum_rec_base = total_recs;
        }
        memcpy(&accum[accum_fill], &s[i], REC_SIZE);
        accum_fill += REC_SIZE;
        total_recs++;

        /* Flush when the batch is full or would wrap the record space
         * (keeps each flash_write contiguous in DATA space).
         */
        if (accum_fill == ACCUM_BYTES ||
            (total_recs % REC_CAPACITY) == 0) {
            accum_flush();
        }
    }
    k_mutex_unlock(&log_lock);
}

int imu_log_read(uint32_t offset, void *buf, uint32_t len)
{
    uint8_t *out = buf;
    uint32_t done = 0;

    k_mutex_lock(&log_lock, K_FOREVER);

    /* Make sure everything appended so far is readable */
    accum_flush();

    uint32_t stored = imu_log_bytes_stored();
    uint32_t logical_end = IMU_LOG_HDR_SIZE + stored;

    if (offset >= logical_end) {
        k_mutex_unlock(&log_lock);
        return 0;
    }
    if (offset + len > logical_end) {
        len = logical_end - offset;
    }

    /* Header part */
    if (offset < IMU_LOG_HDR_SIZE) {
        uint32_t span = MIN(len, IMU_LOG_HDR_SIZE - offset);
        int ret = flash_read(flash_dev, REGION_A_BASE + offset, out, span);
        if (ret < 0) {
            return ret;
        }
        offset += span; out += span; len -= span; done += span;
    }

    /* Record part: logical order = oldest first.  When wrapped, the
     * oldest record sits at total_recs % REC_CAPACITY.
     */
    while (len > 0) {
        uint32_t rec_logical = (offset - IMU_LOG_HDR_SIZE) / REC_SIZE;
        uint32_t intra = (offset - IMU_LOG_HDR_SIZE) % REC_SIZE;
        uint32_t oldest = (total_recs > REC_CAPACITY)
                              ? (total_recs % REC_CAPACITY) : 0;
        uint32_t rec_phys = (oldest + rec_logical) % REC_CAPACITY;
        uint32_t phys_off = rec_phys * REC_SIZE + intra;

        /* Read contiguously until the physical wrap point */
        uint32_t contig = DATA_CAPACITY - phys_off;
        uint32_t span = MIN(len, contig);
        int ret = phys_read(phys_off, out, span);
        if (ret < 0) {
            k_mutex_unlock(&log_lock);
            return ret;
        }
        offset += span; out += span; len -= span; done += span;
    }

    k_mutex_unlock(&log_lock);
    return (int)done;
}
