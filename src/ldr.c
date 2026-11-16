/*
 * Capteur de lumière (LDR) : lecture ADC, calibration et validité.
 *
 * Le module LDR est un pont diviseur : sa tension de sortie varie avec la
 * lumière. Selon le module, elle MONTE ou DESCEND quand la lumière augmente
 * (sur notre montage, elle descend). La formule de calibration ci-dessous
 * fonctionne dans les deux sens : seules les deux constantes mesurées changent.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#include "ldr.h"

/*
 * ---------- Calibration (SR5) ----------
 * Procédure (à refaire si le montage change) :
 *   1. TEST MODE, lire la valeur "raw" affichée dans la ligne LDR ;
 *   2. LDR complètement couvert (doigt ou capuchon opaque) -> LDR_RAW_DARK ;
 *   3. lampe du téléphone à 5 cm du LDR                    -> LDR_RAW_BRIGHT ;
 *   4. recompiler. 0 % = obscurité, 100 % = lampe à 5 cm.
 * Valeurs mesurées le : (date à compléter)
 */
#define LDR_RAW_DARK            3200    
#define LDR_RAW_BRIGHT          570    

/* Moyenne de 8 conversions : réduit le bruit de l'ADC (environ ±10 points) */
#define LDR_AVERAGE_SAMPLES     8

/*
 * Validité (flag 0x0004) :
 *   - valeur collée à 0 ou à 4095 : saturation, ou fil débranché ;
 *   - valeur nettement hors de la plage calibrée (plus de 10 % de l'écart
 *     au-delà de l'obscurité ou de la lumière forte) : mesure non fiable.
 */
#define LDR_SATURATION_MARGIN   20
#define LDR_OUT_OF_RANGE_PCT    10

#define ADC_MAX_12BIT           4095

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
    /*
     * (raw - dark) * 100 / (bright - dark) : si la tension baisse avec la
     * lumière, numérateur et dénominateur sont négatifs, le résultat reste
     * positif. On borne ensuite à 0..100.
     */
    const int span = LDR_RAW_BRIGHT - LDR_RAW_DARK;

    if (span == 0) {
        return 0;
    }

    int percent = ((raw - LDR_RAW_DARK) * 100) / span;

    if (percent < 0) {
        return 0;
    }
    if (percent > 100) {
        return 100;
    }
    return percent;
}

int ldr_read(struct ldr_sample *out)
{
    long sum = 0;

    for (int i = 0; i < LDR_AVERAGE_SAMPLES; i++) {
        int raw = ldr_read_raw();

        if (raw < 0) {
            out->valid = false;
            return raw;
        }
        sum += raw;
    }

    const int raw = (int)(sum / LDR_AVERAGE_SAMPLES);
    const int span = LDR_RAW_BRIGHT - LDR_RAW_DARK;
    const int margin = (span < 0 ? -span : span) * LDR_OUT_OF_RANGE_PCT / 100;
    const int lo = (LDR_RAW_DARK < LDR_RAW_BRIGHT ? LDR_RAW_DARK : LDR_RAW_BRIGHT) - margin;
    const int hi = (LDR_RAW_DARK > LDR_RAW_BRIGHT ? LDR_RAW_DARK : LDR_RAW_BRIGHT) + margin;

    out->raw = (uint16_t)raw;

    /* Pourcentage avec une décimale (même formule que ldr_raw_to_percent) */
    float percent = (span != 0) ? (float)(raw - LDR_RAW_DARK) * 100.0f / (float)span : 0.0f;

    out->percent = (percent <= 0.0f) ? 0.0f : (percent > 100.0f) ? 100.0f : percent;

    out->valid = raw > LDR_SATURATION_MARGIN &&
                 raw < ADC_MAX_12BIT - LDR_SATURATION_MARGIN &&
                 raw >= lo && raw <= hi;

    return 0;
}
