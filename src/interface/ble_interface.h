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

#endif /* BLE_INTERFACE_H */
