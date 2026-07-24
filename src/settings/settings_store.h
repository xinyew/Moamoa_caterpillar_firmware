#ifndef SETTINGS_STORE_H
#define SETTINGS_STORE_H

#include <stdint.h>
#include "settings_gen.h"   /* SETTING_* ids, table, ranges */

/*
 * Persistent device settings (pattern from the biosensor project's
 * settings registry, scaled down).  IDs/ranges/defaults come from
 * settings.yml via scripts/generate_settings.py; values persist in
 * the spare 4 KB partition at 0x164000 and survive reboots and OTA.
 *
 * Usage: call settings_store_init() early in main(), settings_get()
 * to read, settings_set() on every user change (clamps to range,
 * debounced flash persist — safe from any thread incl. BT RX).
 */

void settings_store_init(void);
uint16_t settings_get(uint8_t id);
void settings_set(uint8_t id, uint16_t value);
uint16_t settings_max(uint8_t id);   /* registry max (0 if unknown id) */

#endif /* SETTINGS_STORE_H */
