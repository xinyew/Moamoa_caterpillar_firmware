/*
 * VDC sense — reads the actual motor rail voltage via SAADC.
 *
 * Rail → R6 100k → AIN4 (P1.11) → R5 100k → GND, 100 nF filter at the
 * pin, so V_rail = 2 × V_pin.  Channel config lives in the board
 * devicetree (gain 1/4, internal 0.9 V reference, 12-bit).
 */

#include "driver_vdc_sense.h"

#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drv_vdc_sense, LOG_LEVEL_INF);

static const struct adc_dt_spec vdc_adc =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

/* 100k / 100k divider */
#define DIVIDER_RATIO   2

int drv_vdc_sense_init(void)
{
    if (!adc_is_ready_dt(&vdc_adc)) {
        LOG_ERR("ADC not ready");
        return -ENODEV;
    }

    int ret = adc_channel_setup_dt(&vdc_adc);
    if (ret < 0) {
        LOG_ERR("ADC channel setup failed: %d", ret);
        return ret;
    }

    LOG_INF("VDC sense on AIN4 (P1.11), 2:1 divider");
    return 0;
}

int drv_vdc_sense_read_mv(int32_t *mv)
{
    int16_t sample;
    struct adc_sequence seq = {
        .buffer = &sample,
        .buffer_size = sizeof(sample),
    };
    int ret;

    if (!mv) {
        return -EINVAL;
    }

    adc_sequence_init_dt(&vdc_adc, &seq);

    ret = adc_read_dt(&vdc_adc, &seq);
    if (ret < 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return ret;
    }

    int32_t val = sample;
    ret = adc_raw_to_millivolts_dt(&vdc_adc, &val);
    if (ret < 0) {
        LOG_ERR("mV conversion failed: %d", ret);
        return ret;
    }

    if (val < 0) {
        val = 0;   /* single-ended: negative noise floor reads as 0 */
    }
    *mv = val * DIVIDER_RATIO;
    return 0;
}
