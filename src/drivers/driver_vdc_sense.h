#ifndef DRIVER_VDC_SENSE_H
#define DRIVER_VDC_SENSE_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize the VDC sense ADC channel (AIN4 / P1.11).
 *
 * The motor rail reaches the pin through a 2:1 divider (100k/100k),
 * so full scale covers the whole 0–4.63 V rail range.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv_vdc_sense_init(void);

/**
 * @brief Sample the actual motor rail voltage.
 *
 * @param mv  Out: measured VDC in millivolts (divider-corrected).
 * @return 0 on success, negative errno on failure.
 */
int drv_vdc_sense_read_mv(int32_t *mv);

#endif /* DRIVER_VDC_SENSE_H */
