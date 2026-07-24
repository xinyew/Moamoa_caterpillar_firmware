#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include <zephyr/kernel.h>

/**
 * @brief Initialise Bluetooth — GATT server, advertising, frequency control.
 *
 * After this call the board advertises as "Caterpillar" and accepts
 * Write-Without-Response to a custom characteristic that sets the PWM
 * frequency (uint16_t, little-endian, Hz).
 *
 * @return 0 on success, negative errno on failure.
 */
int ble_interface_init(void);

/**
 * @brief Send a printf-style warning/error line to the message
 *        characteristic (0xFFEC) and mirror it to the local log.
 *
 * Silently dropped when no client is subscribed.  Callable from any
 * thread after ble_interface_init().
 */
void ble_msg(const char *fmt, ...);

/**
 * @brief Current wall-clock time as unix epoch seconds (UTC).
 *
 * Derived from the last 0xFFEE sync; returns 0 if no client has
 * synced the clock since boot.
 */
uint32_t ble_wall_now(void);

/**
 * @brief Switch connection parameters for a log session.
 *
 * slow=true widens the interval to ~50 ms (frees radio air so flash
 * writes get MPSL timeslots); slow=false restores 7.5–15 ms.
 */
void ble_session_conn_params(bool slow);

/**
 * @brief Set the live-stream preview rate cap (0 = auto/link budget)
 *        and recompute the decimation factor.
 */
void ble_stream_set_preview(uint16_t hz);

#endif /* BLE_INTERFACE_H */
