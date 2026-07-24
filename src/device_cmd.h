#ifndef DEVICE_CMD_H
#define DEVICE_CMD_H

#include <stdint.h>

/*
 * Command work-queue: slow device operations execute on a dedicated
 * thread instead of the Bluetooth RX thread.
 *
 * Rationale (pattern borrowed from the biosensor project's
 * device_interface): a GATT write handler runs in the BT RX thread,
 * so anything slow there stalls ALL inbound BLE traffic.  The worst
 * offender was the VDC ramp (10 ms/tap, up to ~1.5 s for a full
 * swing); log stop can also wait up to ~1 s for the flash writer to
 * drain.  Handlers now validate + enqueue and return immediately; an
 * acknowledged write means "accepted", with effects visible through
 * the status poll.
 */

enum device_cmd_type {
    DEVICE_CMD_SET_VOLT_MV,   /* MAX5419 ramp + verify (slow)   */
    DEVICE_CMD_LOG,           /* session start/stop/erase (slow) */
};

struct device_cmd {
    uint8_t type;
    union {
        uint16_t volt_mv;
        struct {
            uint8_t op;       /* LOGCTL_CMD_* */
        } log;
    };
};

/* Enqueue a command.  Returns 0 or -ENOSPC when the queue is full. */
int device_cmd_submit(const struct device_cmd *cmd);

#endif /* DEVICE_CMD_H */
