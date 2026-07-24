#ifndef APP_H
#define APP_H

#include <stdint.h>

/*
 * Application lifecycle (pattern from the biosensor project's app.c):
 * main() is a trivial entry; app_init() runs the boot sequence,
 * app_run() the health monitor loop (never returns).
 */

/** Boot reset cause (Zephyr hwinfo bits), for the status packet. */
extern uint32_t app_reset_cause;

void app_init(void);
void app_run(void);

#endif /* APP_H */
