

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#include "ldr.h"

/*
 * ADC configuration from Device Tree overlay.
 * The overlay must define zephyr,user and io-channels.
 */
static const struct adc_dt_spec adc_ch =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t sample_buf;

static struct adc_sequence sequence = {
    .buffer = &sample_buf,
    .buffer_size = sizeof(sample_buf),
};

int ldr_init(void)
{
    int ret;

    if (!adc_is_ready_dt(&adc_ch)) {
        return -ENODEV;
    }

    ret = adc_channel_setup_dt(&adc_ch);
    if (ret < 0) {
        return ret;
    }

    ret = adc_sequence_init_dt(&adc_ch, &sequence);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int ldr_read_raw(void)
{
    int ret;

    ret = adc_read_dt(&adc_ch, &sequence);

    if (ret < 0) {
        return ret;
    }

    return sample_buf;
}

int ldr_raw_to_percent(int raw)
{
    /* Initial calibration values: to be adjusted after testing */
    const int raw_dark = 2700;
    const int raw_bright = 4000;

    if (raw <= raw_dark) {
        return 0;
    }

    if (raw >= raw_bright) {
        return 100;
    }

    return ((raw - raw_dark) * 100) /
           (raw_bright - raw_dark);
}
