#ifndef SESSION_H
#define SESSION_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Session orchestration — single source of truth for run-state policy
 * (pattern from the biosensor project's device_state):
 *   - wall-clock reference (synced by the host on connect)
 *   - on-demand IMU sampling arbiter (sensor powered only while a log
 *     session runs or a live stream is subscribed)
 *   - session lifecycle side effects (connection-parameter policy)
 *
 * Transports report intent here (stream subscribed, time synced);
 * device_cmd drives lifecycle transitions; drivers/imu_log stay
 * policy-free.
 */

/** Host synced the wall clock (unix epoch seconds, UTC, nonzero). */
void session_time_sync(uint32_t epoch);

/** Current wall-clock estimate, 0 if never synced since boot. */
uint32_t session_wall_now(void);

/** Live stream subscription changed (from the CCC handler). */
void session_set_stream_active(bool active);

/** Whether anything currently needs IMU data. */
bool session_imu_demand(void);

/** Re-arbitrate on-demand sampling: sensor up with the persisted
 *  config when there is demand, powered down otherwise.
 */
void session_imu_run_update(void);

/** Log session lifecycle side effects (called by device_cmd after
 *  imu_log_start/stop): conn-param policy + sampling arbitration.
 */
void session_on_log_started(void);
void session_on_log_stopped(void);

/** Connection dropped: stop data flows and close a running session. */
void session_on_disconnect(void);

#endif /* SESSION_H */
