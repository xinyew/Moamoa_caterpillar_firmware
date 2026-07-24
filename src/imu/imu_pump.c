/*
 * IMU ring-buffer pump — see imu_pump.h.
 *
 * Polls every 4 ms: at 6.66 kHz that is ~27 new records per pass, and
 * the 2048-record ring gives ~300 ms of slack before the FLPR starts
 * counting overruns, so BLE/flash hiccups are absorbed silently.
 */

#include "imu_pump.h"
#include "imu_log.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_pump, LOG_LEVEL_INF);

#define PUMP_PERIOD_MS   4
#define PUMP_CHUNK       64          /* records copied per inner pass */
#define PUMP_STACK_SIZE  2048
#define PUMP_PRIO        4           /* above the BLE RX/TX threads */

static imu_stream_sink_t volatile stream_sink;
static uint32_t drained_total;

void imu_pump_set_sink(imu_stream_sink_t sink)
{
    stream_sink = sink;
}

uint32_t imu_pump_overrun(void)
{
    struct imu_shared *sh = IMU_SHARED;
    return (sh->magic == IMU_SHARED_MAGIC) ? sh->overrun : 0;
}

uint32_t imu_pump_drained(void)
{
    return drained_total;
}

static void pump_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    struct imu_shared *sh = IMU_SHARED;
    static struct imu_sample chunk[PUMP_CHUNK];

    while (1) {
        int64_t t_sleep = k_uptime_get();

        k_msleep(PUMP_PERIOD_MS);

        int64_t t_wake = k_uptime_get();

        if (t_wake - t_sleep > 50) {
            LOG_WRN("pump not scheduled for %lld ms",
                    t_wake - t_sleep - PUMP_PERIOD_MS);
        }

        if (sh->magic != IMU_SHARED_MAGIC || !sh->imu_ok) {
            continue;
        }

        uint32_t head = sh->head;
        uint32_t tail = sh->tail;
        barrier_dmem_fence_full();   /* records published before head */

        while (tail != head) {
            uint32_t n = MIN(head - tail, (uint32_t)PUMP_CHUNK);

            for (uint32_t i = 0; i < n; i++) {
                volatile struct imu_sample *s =
                    &sh->ring[(tail + i) & (IMU_RING_N - 1)];
                chunk[i].ax = s->ax; chunk[i].ay = s->ay;
                chunk[i].az = s->az;
                chunk[i].gx = s->gx; chunk[i].gy = s->gy;
                chunk[i].gz = s->gz;
                chunk[i].temp_raw = s->temp_raw;
                chunk[i].seq16 = s->seq16;
            }
            barrier_dmem_fence_full();   /* copies done before slots free */
            tail += n;
            sh->tail = tail;
            drained_total += n;

            int64_t t0 = k_uptime_get();

            imu_log_append(chunk, n);

            int64_t t1 = k_uptime_get();

            imu_stream_sink_t sink = stream_sink;
            if (sink != NULL) {
                sink(chunk, n);
            }

            int64_t t2 = k_uptime_get();

            if (t1 - t0 > 50) {
                LOG_WRN("log_append(%u) took %lld ms", n, t1 - t0);
            }
            if (t2 - t1 > 50) {
                LOG_WRN("stream sink(%u) took %lld ms", n, t2 - t1);
            }
        }
    }
}

K_THREAD_DEFINE(imu_pump_tid, PUMP_STACK_SIZE, pump_thread,
                NULL, NULL, NULL, PUMP_PRIO, 0, 0);

int imu_pump_init(void)
{
    /* Thread starts automatically (K_THREAD_DEFINE); nothing else */
    return 0;
}
