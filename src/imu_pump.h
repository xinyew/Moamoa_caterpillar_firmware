#ifndef IMU_PUMP_H
#define IMU_PUMP_H

#include <stdint.h>
#include "common/imu_shared.h"

/*
 * Ring-buffer consumer thread: drains the FLPR's shared-SRAM sample
 * ring every few ms and fans each batch out to
 *   1. the flash log (imu_log, when a session is active)
 *   2. an optional live-stream sink (registered by the BLE layer)
 * Flash always receives the full rate; the sink may decimate/drop.
 */

typedef void (*imu_stream_sink_t)(const struct imu_sample *s, uint32_t n);

int  imu_pump_init(void);

/* Register/unregister the live-stream sink (NULL = off).  The sink
 * runs in the pump thread — keep it fast and non-blocking.
 */
void imu_pump_set_sink(imu_stream_sink_t sink);

/* FLPR-side ring overruns observed (mirror of shared block). */
uint32_t imu_pump_overrun(void);

/* Total records drained since boot. */
uint32_t imu_pump_drained(void);

#endif /* IMU_PUMP_H */
