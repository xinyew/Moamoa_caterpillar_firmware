/*
 * Caterpillar â€” Motor Control + IMU Firmware
 * Custom nRF54L15 board (caterpillar/nrf54l15/cpuapp)
 *
 * Trivial entry point; see src/app.c for the boot sequence and
 * health-monitor loop.
 */

#include "app/app.h"

int main(void)
{
    app_init();
    app_run();      /* never returns */
    return 0;
}
