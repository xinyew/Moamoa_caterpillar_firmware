/*
 * FLPR launch — copies the FLPR firmware EMBEDDED IN THIS APP IMAGE
 * into the coprocessor's SRAM and starts it.
 *
 * Because the FLPR binary travels inside the app image, every app OTA
 * update carries the matching FLPR firmware: the two are version-atomic
 * and no separate SWD flash of the coprocessor is ever needed.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <hal/nrf_vpr.h>
#include <hal/nrf_spu.h>

#include "flpr_launch.h"

LOG_MODULE_REGISTER(flpr_launch, LOG_LEVEL_INF);

/* FLPR image built from flpr/ (see CMakeLists: generate_inc_file_for_target) */
static const uint8_t flpr_image[] = {
#include "flpr_image.inc"
};

/* Execution SRAM base — matches cpuflpr_sram in the FLPR devicetree */
#define FLPR_EXEC_ADDR  0x20028000UL

#define FLPR_VPR ((NRF_VPR_Type *)DT_REG_ADDR(DT_NODELABEL(cpuflpr_vpr)))

void flpr_launch(void)
{
    memcpy((void *)FLPR_EXEC_ADDR, flpr_image, sizeof(flpr_image));

    /* Mark the VPR peripheral secure (as the DT launcher did for the
     * node's enable-secure property)
     */
    nrf_spu_periph_perm_secattr_set(NRF_SPU00,
                                    nrf_address_slave_get((uint32_t)FLPR_VPR),
                                    true);

    nrf_vpr_initpc_set(FLPR_VPR, FLPR_EXEC_ADDR);
    nrf_vpr_cpurun_set(FLPR_VPR, true);

    LOG_INF("FLPR launched (embedded image, %u bytes)",
            (unsigned)sizeof(flpr_image));
}
