#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include <zephyr/kernel.h>

/*
 * BLE control-plane adapter: GATT service table, request decoding, and
 * connection lifecycle.  Handlers validate + delegate (drivers,
 * device_cmd, session, ble_transport) and never block the BT RX
 * thread.  The data plane (stream/dump/messages) lives in
 * ble_transport.c; run-state policy in session.c.
 */

/**
 * @brief Initialise Bluetooth — GATT server + advertising.
 *
 * After this call the board advertises as "Caterpillar".
 *
 * @return 0 on success, negative errno on failure.
 */
int ble_interface_init(void);

/**
 * @brief Request connection parameters.
 *
 * slow=true widens the interval to ~50 ms (frees radio air so flash
 * writes get MPSL timeslots during log sessions); slow=false restores
 * the responsive 7.5–15 ms.  No-op when disconnected.
 */
void ble_conn_request_params(bool slow);

#endif /* BLE_INTERFACE_H */
