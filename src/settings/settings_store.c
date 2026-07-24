/*
 * Persistent device settings — see settings_store.h.
 *
 * Storage: the spare 4 KB flash partition at 0x164000 (between the
 * OTA secondary slot and the ex-FLPR region; nothing else touches it,
 * and it survives OTA updates).  Two 256 B slots are written
 * alternately with a generation counter and CRC; load picks the valid
 * slot with the highest generation, so a power cut mid-write can only
 * lose the newest change, never corrupt the store.  RRAM overwrites
 * freely — no erase handling needed.
 *
 * Persisting is debounced (500 ms) onto the system workqueue: flash
 * writes wait for MPSL radio timeslots and must never run in the
 * caller's context (GATT handlers!).
 */

#include "settings_store.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(settings_store, LOG_LEVEL_INF);

#define FLASH_DEV   DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller))
#define STORE_BASE  0x164000UL
#define SLOT_SIZE   256UL
#define SLOT_ADDR(i) (STORE_BASE + (i) * SLOT_SIZE)

#define STORE_MAGIC 0x54455343UL   /* "CSET" */

struct store_rec {
    uint32_t magic;
    uint32_t generation;
    uint16_t values[SETTINGS_MAX_ID + 1];   /* indexed by setting id */
    uint32_t crc;                            /* over all prior bytes */
};
BUILD_ASSERT(sizeof(struct store_rec) <= SLOT_SIZE);

static const struct setting_meta table[SETTINGS_COUNT] = SETTINGS_TABLE;

static const struct device *flash_dev;
static struct store_rec rec;
static bool loaded;

static const struct setting_meta *meta_for(uint8_t id)
{
    for (int i = 0; i < SETTINGS_COUNT; i++) {
        if (table[i].id == id) {
            return &table[i];
        }
    }
    return NULL;
}

static uint32_t rec_crc(const struct store_rec *r)
{
    return crc32_ieee((const uint8_t *)r,
                      offsetof(struct store_rec, crc));
}

static void persist_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Write the slot the current generation does NOT live in */
    uint32_t slot = rec.generation % 2;

    rec.generation++;
    slot = rec.generation % 2;
    rec.crc = rec_crc(&rec);

    int ret = flash_write(flash_dev, SLOT_ADDR(slot), &rec, sizeof(rec));

    if (ret < 0) {
        LOG_ERR("settings persist failed: %d", ret);
    } else {
        LOG_INF("settings persisted (gen %u)", rec.generation);
    }
}

static K_WORK_DELAYABLE_DEFINE(persist_work, persist_work_fn);

static void load_defaults(void)
{
    memset(rec.values, 0, sizeof(rec.values));
    for (int i = 0; i < SETTINGS_COUNT; i++) {
        rec.values[table[i].id] = table[i].def;
    }
    rec.magic = STORE_MAGIC;
    rec.generation = 0;
}

void settings_store_init(void)
{
    flash_dev = FLASH_DEV;
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("flash not ready — settings are defaults, not persisted");
        load_defaults();
        loaded = true;
        return;
    }

    struct store_rec a, b;
    bool a_ok = false, b_ok = false;

    if (flash_read(flash_dev, SLOT_ADDR(0), &a, sizeof(a)) == 0) {
        a_ok = (a.magic == STORE_MAGIC && a.crc == rec_crc(&a));
    }
    if (flash_read(flash_dev, SLOT_ADDR(1), &b, sizeof(b)) == 0) {
        b_ok = (b.magic == STORE_MAGIC && b.crc == rec_crc(&b));
    }

    if (a_ok && (!b_ok || a.generation >= b.generation)) {
        rec = a;
    } else if (b_ok) {
        rec = b;
    } else {
        load_defaults();
        LOG_INF("settings: no valid store — using defaults");
        loaded = true;
        return;
    }

    /* Clamp against the current registry (ranges may have changed
     * between firmware versions; unknown ids fall back to defaults).
     */
    for (int i = 0; i < SETTINGS_COUNT; i++) {
        uint16_t v = rec.values[table[i].id];

        if (v < table[i].min || v > table[i].max) {
            rec.values[table[i].id] = table[i].def;
        }
    }

    LOG_INF("settings loaded (gen %u)", rec.generation);
    loaded = true;
}

uint16_t settings_max(uint8_t id)
{
    const struct setting_meta *m = meta_for(id);

    return m ? m->max : 0;
}

uint16_t settings_get(uint8_t id)
{
    const struct setting_meta *m = meta_for(id);

    if (!loaded || m == NULL) {
        return m ? m->def : 0;
    }
    return rec.values[id];
}

void settings_set(uint8_t id, uint16_t value)
{
    const struct setting_meta *m = meta_for(id);

    if (!loaded || m == NULL) {
        return;
    }
    value = CLAMP(value, m->min, m->max);
    if (rec.values[id] == value) {
        return;
    }
    rec.values[id] = value;
    k_work_schedule(&persist_work, K_MSEC(500));
}
