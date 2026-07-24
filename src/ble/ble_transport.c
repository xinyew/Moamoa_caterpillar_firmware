/*
 * BLE data-plane transport — see ble_transport.h.
 * Code moved verbatim from ble_interface.c in the app/adapter split.
 */

#include "ble_transport.h"
#include "ble_uuids.h"

#include "imu/imu_pump.h"
#include "imu/imu_log.h"
#include "common/imu_shared.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_tx, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/*  ODR table + stream decimation                                             */
/* -------------------------------------------------------------------------- */

/* Actual rates for each IMU_ODR_* code (Hz, rounded up) */
static const uint16_t odr_hz_tab[11] = {
    0, 13, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660
};

uint16_t ble_odr_hz(uint8_t odr_code)
{
    return (odr_code <= 10) ? odr_hz_tab[odr_code] : 0;
}

/* Live stream budget: ~1300 samples/s of 16 B records = ~20 KiB/s on
 * an idle link.  While a log session runs the connection interval is
 * widened to ~50 ms for flash timeslots and the link really carries
 * only ~600 samples/s — the budget drops so the stream FIFO paces
 * itself instead of overflowing (measured at auto/833 Hz while
 * logging: ~270 dropped/s in bursts, i.e. a visibly choppy preview).
 * The budget is a physical link property, so it also caps an explicit
 * preview rate.
 */
#define STREAM_BUDGET_SPS      1300
#define STREAM_BUDGET_LOG_SPS  500

static uint8_t stream_decim = 1;
static uint16_t stream_preview_hz;   /* 0 = auto (link-budget limit) */

/* The budgets above are what the link carries on a GOOD day; real
 * capacity varies run to run (RF, retransmissions, what the central
 * negotiated).  On sustained FIFO drops the sink halves the preview
 * rate within a second (shift up to 16x) and probes back up after 5
 * quiet seconds — the preview stays gap-free at whatever rate the
 * link actually delivers.
 */
static uint8_t stream_adapt;         /* extra decim shift, 0..4 */
static uint8_t adapt_calm;           /* consecutive drop-free seconds */
static uint8_t adapt_calm_need = 5;  /* seconds of calm before probing */
static uint32_t adapt_last_dropped;
static int64_t adapt_win_start;
static int64_t adapt_last_probe = INT64_MIN / 2;

static uint8_t stream_decim_eff(void)
{
    uint32_t d = (uint32_t)stream_decim << stream_adapt;

    return (uint8_t)MIN(d, 255);
}

void ble_transport_stream_update_decim(void)
{
    uint16_t hz = ble_odr_hz(IMU_SHARED->cfg_odr);
    uint32_t target = imu_log_active() ? STREAM_BUDGET_LOG_SPS
                                       : STREAM_BUDGET_SPS;

    if (stream_preview_hz != 0 && stream_preview_hz < target) {
        target = stream_preview_hz;
    }
    uint32_t d = (hz + target - 1) / target;

    stream_decim = (uint8_t)CLAMP(d, 1, 255);
    stream_adapt = 0;                /* config changed: re-learn */
    adapt_calm = 0;
    adapt_calm_need = 5;
    adapt_last_probe = INT64_MIN / 2;
    adapt_win_start = k_uptime_get();
}

void ble_stream_set_preview(uint16_t hz)
{
    stream_preview_hz = hz;
    ble_transport_stream_update_decim();
}

uint8_t ble_transport_stream_decim(void)
{
    return stream_decim;
}

/* -------------------------------------------------------------------------- */
/*  Notification TX thread — live stream (0xFFE9) + message lines (0xFFEC)    */
/*                                                                            */
/*  The pump thread must NEVER call into the BT stack: bt_gatt_notify can    */
/*  block indefinitely under TX-context exhaustion (proven on hardware by    */
/*  the dump wedge), and a blocked pump overruns the FLPR ring.  Instead     */
/*  the sink stages picked samples into an SPSC FIFO and this thread does    */
/*  all sending, credit-paced (≤2 notifications in flight) so the blocking  */
/*  path is never entered.  ble_msg() lines ride the same thread.            */
/*                                                                            */
/*  Stream packet: 8 B header + N × 16 B struct imu_sample:                   */
/*    0 u8 N   1 u8 decim   2 u8 flags   3 u8 rsvd   4 u32 dropped samples    */
/* -------------------------------------------------------------------------- */

#define STREAM_MAX_SAMPLES  14
#define STREAM_HDR          8

#define SFIFO_N   256               /* staged samples, power of two */
#define MSGQ_N    4
#define MSG_LEN   120

static struct imu_sample sfifo[SFIFO_N];
static volatile uint32_t sfifo_head;   /* producer: pump (sink) */
static volatile uint32_t sfifo_tail;   /* consumer: TX thread   */

static char msgq[MSGQ_N][MSG_LEN];
static volatile uint32_t msgq_head, msgq_tail;
static struct k_spinlock msgq_lock;    /* multiple msg producers */

static uint8_t stream_pkt[STREAM_HDR + STREAM_MAX_SAMPLES * 16];
static uint8_t stream_max = STREAM_MAX_SAMPLES;
static uint32_t stream_sample_ctr;
static uint32_t stream_dropped;
static int64_t sfifo_ts;               /* when the FIFO went non-empty */

static K_SEM_DEFINE(tx_wake, 0, 1);
static K_SEM_DEFINE(tx_credits, 2, 2);

static void tx_sent_cb(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn); ARG_UNUSED(user_data);
    k_sem_give(&tx_credits);
}

/* Send one notification, credit-paced.  Returns false if dropped
 * (congestion, disconnect, jam) — callers discard, never retry stale
 * preview data.
 */
static bool tx_notify(const struct bt_uuid *uuid, const void *data,
                      uint16_t len)
{
    if (k_sem_take(&tx_credits, K_MSEC(500)) != 0) {
        return false;                /* TX jammed — drop */
    }

    struct bt_gatt_notify_params np = {
        .uuid = uuid,
        .attr = caterpillar_svc.attrs,
        .data = data,
        .len = len,
        .func = tx_sent_cb,
    };
    int ret;
    int tries = 0;

    do {
        ret = bt_gatt_notify_cb(NULL, &np);
        if (ret == -ENOMEM || ret == -EAGAIN || ret == -ENOBUFS) {
            k_msleep(1);
        } else {
            break;
        }
    } while (++tries < 100);

    if (ret < 0) {
        k_sem_give(&tx_credits);     /* never handed to the stack */
        return false;
    }
    return true;
}

/* 1 Hz adaptation window, run in pump context (sink is called every
 * 4 ms while streaming): back off fast on drops, recover slowly.
 */
static void stream_adapt_tick(void)
{
    int64_t now = k_uptime_get();

    if (now - adapt_win_start < 1000) {
        return;
    }
    adapt_win_start = now;

    uint32_t drops = stream_dropped - adapt_last_dropped;

    adapt_last_dropped = stream_dropped;
    if (drops > 10) {
        if (stream_adapt < 4) {
            stream_adapt++;
        }
        /* A probe up that immediately re-dropped means the link
         * really can't take more: probe less and less often instead
         * of sawtoothing between clean and choppy.
         */
        if (now - adapt_last_probe < 10000 && adapt_calm_need < 30) {
            adapt_calm_need += 5;
        }
        adapt_calm = 0;
    } else if (drops == 0) {
        if (++adapt_calm >= adapt_calm_need && stream_adapt > 0) {
            stream_adapt--;
            adapt_last_probe = now;
            adapt_calm = 0;
        }
    } else {
        adapt_calm = 0;
    }
}

/* Pump-context sink: decimate and stage.  No BT calls, no blocking. */
static void stream_sink(const struct imu_sample *s, uint32_t n)
{
    uint8_t decim = stream_decim_eff();

    for (uint32_t i = 0; i < n; i++) {
        if ((stream_sample_ctr++ % decim) != 0) {
            continue;
        }
        if (sfifo_head - sfifo_tail >= SFIFO_N) {
            stream_dropped++;        /* TX behind — drop, never block */
            continue;
        }
        if (sfifo_head == sfifo_tail) {
            sfifo_ts = k_uptime_get();
        }
        sfifo[sfifo_head & (SFIFO_N - 1)] = s[i];
        sfifo_head = sfifo_head + 1;
    }
    stream_adapt_tick();
    k_sem_give(&tx_wake);
}

static void tx_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        k_sem_take(&tx_wake, K_MSEC(50));

        /* Messages first — rare and high-value */
        while (msgq_tail != msgq_head) {
            const char *line = msgq[msgq_tail % MSGQ_N];

            (void)tx_notify(BT_UUID_CATERPILLAR_MSG, line, strlen(line));
            msgq_tail = msgq_tail + 1;
        }

        /* Stream packets: full ones eagerly, partials after 100 ms */
        while (1) {
            uint32_t avail = sfifo_head - sfifo_tail;

            if (avail == 0) {
                break;
            }
            if (avail < stream_max &&
                (k_uptime_get() - sfifo_ts) < 100) {
                break;               /* wait for a fuller packet */
            }

            uint8_t n = (uint8_t)MIN(avail, (uint32_t)stream_max);

            for (uint8_t i = 0; i < n; i++) {
                memcpy(&stream_pkt[STREAM_HDR + i * 16],
                       (const void *)&sfifo[(sfifo_tail + i) & (SFIFO_N - 1)],
                       16);
            }
            stream_pkt[0] = n;
            stream_pkt[1] = stream_decim_eff();
            stream_pkt[2] = 0;
            stream_pkt[3] = 0;
            sys_put_le32(stream_dropped, &stream_pkt[4]);

            if (!tx_notify(BT_UUID_CATERPILLAR_STREAM, stream_pkt,
                           STREAM_HDR + n * 16)) {
                stream_dropped += n;
            }
            sfifo_tail = sfifo_tail + n;
            sfifo_ts = k_uptime_get();
        }
    }
}

K_THREAD_DEFINE(ble_tx_tid, 2048, tx_thread, NULL, NULL, NULL, 9, 0, 0);

void ble_transport_stream_enable(bool on)
{
    if (on) {
        sfifo_tail = sfifo_head;
        stream_sample_ctr = 0;
        stream_dropped = 0;
        stream_max = STREAM_MAX_SAMPLES;
        ble_transport_stream_update_decim();
        imu_pump_set_sink(stream_sink);
        LOG_INF("IMU stream ON (decim %u)", stream_decim);
    } else {
        imu_pump_set_sink(NULL);
        LOG_INF("IMU stream OFF");
    }
}

/* -------------------------------------------------------------------------- */
/*  Log dump (0xFFEB): request {session u32, offset u32, len u32} ->          */
/*  notify chunks: 0 u32 offset   4 u16 n   6 u8 last   7 u8 rsvd   8.. data  */
/* -------------------------------------------------------------------------- */

#define DUMP_CHUNK_DATA  232
#define DUMP_CREDITS     2

static K_SEM_DEFINE(dump_sem, 0, 1);
/* Flow control: max DUMP_CREDITS notifications in flight.  Sending
 * unpaced exhausts the host TX contexts and bt_gatt_notify then
 * BLOCKS FOREVER waiting for one (observed on hardware: dump wedged
 * mid-transfer, deaf to new requests, link eventually dropped).
 */
static K_SEM_DEFINE(dump_credit_sem, 0, DUMP_CREDITS);
static uint32_t dump_req_session, dump_req_off, dump_req_len;
static volatile bool dump_abort;

static void dump_sent_cb(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn); ARG_UNUSED(user_data);
    k_sem_give(&dump_credit_sem);
}

void ble_transport_dump_request(uint32_t session, uint32_t offset,
                                uint32_t len)
{
    dump_abort = true;               /* cancel any dump in flight */
    dump_req_session = session;
    dump_req_off = offset;
    dump_req_len = len;
    k_sem_give(&dump_sem);
}

void ble_transport_dump_abort(void)
{
    dump_abort = true;
}

static void dump_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    static uint8_t chunk[8 + DUMP_CHUNK_DATA];

    while (1) {
        k_sem_take(&dump_sem, K_FOREVER);
        dump_abort = false;

        uint32_t session = dump_req_session;
        uint32_t off = dump_req_off;
        uint32_t remaining = dump_req_len;

        /* Fresh credits per request (stale sent-callbacks from an
         * aborted dump can only saturate the cap, which is harmless).
         */
        while (k_sem_take(&dump_credit_sem, K_NO_WAIT) == 0) {
        }
        for (int i = 0; i < DUMP_CREDITS; i++) {
            k_sem_give(&dump_credit_sem);
        }

        while (remaining > 0 && !dump_abort) {
            uint32_t want = MIN(remaining, (uint32_t)DUMP_CHUNK_DATA);
            int n = imu_log_read_session(session, off, &chunk[8], want);

            if (n < 0) {
                /* never silently: this was the invisible failure mode
                 * behind dumps freezing at a fixed percentage
                 */
                ble_msg("dump read error %d at offset %u", n, off);
                break;
            }
            bool last = (n < (int)want) || ((uint32_t)n == remaining);

            sys_put_le32(off, &chunk[0]);
            sys_put_le16((uint16_t)n, &chunk[4]);
            chunk[6] = last ? 1 : 0;
            chunk[7] = 0;

            /* Wait for an in-flight slot; a long wait means the TX
             * path is wedged — abort and let the host resume.
             */
            if (k_sem_take(&dump_credit_sem, K_SECONDS(3)) != 0) {
                ble_msg("dump stalled at offset %u — TX jam, resume",
                        off);
                break;
            }

            struct bt_gatt_notify_params np = {
                .uuid = BT_UUID_CATERPILLAR_DUMP,
                .attr = caterpillar_svc.attrs,
                .data = chunk,
                .len = (uint16_t)(8 + n),
                .func = dump_sent_cb,
            };
            int ret;
            int tries = 0;
            do {
                ret = bt_gatt_notify_cb(NULL, &np);
                if (ret == -ENOMEM || ret == -EAGAIN || ret == -ENOBUFS) {
                    k_msleep(2);
                } else {
                    break;
                }
            } while (++tries < 500 && !dump_abort);

            if (ret < 0) {
                k_sem_give(&dump_credit_sem);   /* never sent */
                ble_msg("dump aborted at offset %u (err %d)", off, ret);
                break;
            }
            if (last) {
                break;
            }
            off += n;
            remaining -= n;
        }
    }
}

K_THREAD_DEFINE(dump_tid, 2048, dump_thread, NULL, NULL, NULL, 7, 0, 0);

/* -------------------------------------------------------------------------- */
/*  Message channel (0xFFEC live) + tier-2 history ring (0xFFF0)              */
/* -------------------------------------------------------------------------- */

/* Tier-2 log: every ble_msg line also lands in a 2 KB RAM ring that
 * clients can read AFTER THE FACT via 0xFFF0 — warnings emitted while
 * nobody was subscribed (or even connected) stay queryable until they
 * age out.  Lesson learned: flash-write errors went to RTT only and
 * stayed invisible for weeks.
 */
#define T2_RING_SIZE 2048

static char t2_ring[T2_RING_SIZE];
static uint32_t t2_head;             /* total bytes ever written */

static void t2_store(const char *line)
{
    char stamped[MSG_LEN + 16];
    int64_t up = k_uptime_get();
    int n = snprintf(stamped, sizeof(stamped), "[%5lld.%03lld] %s\n",
                     up / 1000, up % 1000, line);

    if (n <= 0) {
        return;
    }
    n = MIN(n, (int)sizeof(stamped) - 1);
    for (int i = 0; i < n; i++) {
        t2_ring[t2_head % T2_RING_SIZE] = stamped[i];
        t2_head++;
    }
}

uint32_t ble_transport_t2_snapshot(char *buf, uint32_t max)
{
    k_spinlock_key_t key = k_spin_lock(&msgq_lock);
    uint32_t used = MIN(t2_head, (uint32_t)T2_RING_SIZE);

    used = MIN(used, max);
    uint32_t start = t2_head - used;

    for (uint32_t i = 0; i < used; i++) {
        buf[i] = t2_ring[(start + i) % T2_RING_SIZE];
    }
    k_spin_unlock(&msgq_lock, key);
    return used;
}

void ble_msg(const char *fmt, ...)
{
    char line[MSG_LEN];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    /* Mirror to the local log for wired debugging */
    LOG_WRN("%s", line);

    /* Enqueue for the TX thread — callers include the pump thread
     * (holding the log mutex), so sending inline here is forbidden.
     */
    k_spinlock_key_t key = k_spin_lock(&msgq_lock);

    t2_store(line);
    if (msgq_head - msgq_tail < MSGQ_N) {
        strncpy(msgq[msgq_head % MSGQ_N], line, MSG_LEN - 1);
        msgq[msgq_head % MSGQ_N][MSG_LEN - 1] = '\0';
        msgq_head = msgq_head + 1;
    }
    k_spin_unlock(&msgq_lock, key);
    k_sem_give(&tx_wake);
}
