#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>

/*
 * BLE data-plane transport: everything that SENDS over the link.
 * All senders are credit-paced (bounded notifications in flight) —
 * unpaced bt_gatt_notify blocks forever under TX-context exhaustion
 * (proven on hardware), so no other module may call into the BT TX
 * path directly.
 *
 *   - live IMU stream (0xFFE9): staged by the pump sink, sent by the
 *     TX thread with ODR-derived decimation
 *   - message lines (0xFFEC) + tier-2 history ring (0xFFF0)
 *   - session dump (0xFFEB): chunked, host-resumable
 */

/** printf-style warning/error line: notified live on 0xFFEC (when
 *  subscribed), stored in the tier-2 ring, mirrored to the local log.
 *  Callable from any thread.
 */
void ble_msg(const char *fmt, ...);

/** Enable/disable the live stream path (CCC subscribe state):
 *  resets staging, recomputes decimation, attaches/detaches the pump
 *  sink.  Does NOT arbitrate sensor power — that's the session layer.
 */
void ble_transport_stream_enable(bool on);

/** Set the preview rate cap (0 = auto/link budget) and re-derive
 *  decimation.
 */
void ble_stream_set_preview(uint16_t hz);

/** Re-derive decimation from the current shared-block ODR. */
void ble_transport_stream_update_decim(void);

/** Current stream decimation factor (for the 0xFFE8 read). */
uint8_t ble_transport_stream_decim(void);

/** Queue a session dump {session, offset, len}; cancels any dump in
 *  flight.  Chunks flow on 0xFFEB from the dump thread.
 */
void ble_transport_dump_request(uint32_t session, uint32_t offset,
                                uint32_t len);

/** Abort a dump in flight (disconnect, new request). */
void ble_transport_dump_abort(void);

/** Copy the tier-2 history ring (oldest first) into buf; returns the
 *  number of bytes written (≤ max).
 */
uint32_t ble_transport_t2_snapshot(char *buf, uint32_t max);

/** Actual sample rate in Hz for an IMU_ODR_* code (0 for invalid). */
uint16_t ble_odr_hz(uint8_t odr_code);

#endif /* BLE_TRANSPORT_H */
