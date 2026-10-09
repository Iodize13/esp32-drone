/**
 * adc_batt.c - battery voltage sense on ADC1 via continuous DMA.
 *
 * No polling: the ADC digital controller samples GPIO1 into DMA
 * buffers on its own, and the conversion-done ISR averages each frame
 * into s_avg_raw. adc_batt_pin_mv() only reads that stored value.
 */
#include <stddef.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"
#include "adc_batt.h"

#define BATT_ADC_UNIT       (ADC_UNIT_1)
#define BATT_ADC_CHANNEL    (ADC_CHANNEL_0)     /* GPIO1 */
#define BATT_ADC_ATTEN      (ADC_ATTEN_DB_12)   /* about 0-3.1 V   */
#define BATT_ADC_BITS       (ADC_BITWIDTH_12)
#define BATT_SAMPLE_HZ      (20000U)            /* 1 kHz gave no frames */
#define BATT_FRAME_SAMPLES  (256U)              /* ~13 ms per ISR  */
#define BATT_FRAME_BYTES    (BATT_FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES)
#define BATT_POOL_BYTES     (BATT_FRAME_BYTES * 4U)
#define BATT_RAW_FULL       (4095U)
#define BATT_FULL_MV        (3100U)             /* uncalibrated fallback */

static adc_continuous_handle_t s_adc  = NULL;
static adc_cali_handle_t       s_cali = NULL;
static volatile uint32_t       s_avg_raw = 0U;
static volatile bool           s_have    = false;

/* ISR: average the valid GPIO1 samples of one finished DMA frame. */
static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t handle,
                                   const adc_continuous_evt_data_t *edata,
                                   void *user_data)
{
    const adc_digi_output_data_t *d;
    uint32_t sum = 0U;
    uint32_t n   = 0U;
    uint32_t i;

    (void)handle;
    (void)user_data;

    for (i = 0U; (i + SOC_ADC_DIGI_RESULT_BYTES) <= edata->size;
         i += SOC_ADC_DIGI_RESULT_BYTES)
    {
        d = (const adc_digi_output_data_t *)&edata->conv_frame_buffer[i];
        if ((d->type2.unit == 0U) && (d->type2.channel == (uint32_t)BATT_ADC_CHANNEL))
        {
            sum += d->type2.data;
            n++;
        }
        else
        {
            /* other channel or invalid sample */
        }
    }

    if (n > 0U)
    {
        s_avg_raw = sum / n;
        s_have    = true;
    }
    else
    {
        /* keep the previous average */
    }

    return false;                        /* no task woken */
}

bool adc_batt_init(void)
{
    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = BATT_POOL_BYTES,
        .conv_frame_size    = BATT_FRAME_BYTES,
        .flags              = { .flush_pool = 1U },
    };
    adc_digi_pattern_config_t pat = {
        .atten     = (uint8_t)BATT_ADC_ATTEN,
        .channel   = (uint8_t)BATT_ADC_CHANNEL,
        .unit      = (uint8_t)BATT_ADC_UNIT,
        .bit_width = (uint8_t)BATT_ADC_BITS,
    };
    adc_continuous_config_t ccfg = {
        .pattern_num    = 1U,
        .adc_pattern    = &pat,
        .sample_freq_hz = BATT_SAMPLE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
    };
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = on_conv_done,
        .on_pool_ovf  = NULL,
    };
    adc_cali_curve_fitting_config_t cal = {
        .unit_id  = BATT_ADC_UNIT,
        .chan     = BATT_ADC_CHANNEL,
        .atten    = BATT_ADC_ATTEN,
        .bitwidth = BATT_ADC_BITS,
    };
    esp_err_t err;

    err = adc_continuous_new_handle(&hcfg, &s_adc);
    if (err == ESP_OK)
    {
        err = adc_continuous_config(s_adc, &ccfg);
    }
    else
    {
        /* handle failed */
    }
    if (err == ESP_OK)
    {
        err = adc_continuous_register_event_callbacks(s_adc, &cbs, NULL);
    }
    else
    {
        /* config failed */
    }
    if (err == ESP_OK)
    {
        err = adc_continuous_start(s_adc);
    }
    else
    {
        /* callback registration failed */
    }

    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK)
    {
        s_cali = NULL;                   /* fall back to linear scaling */
    }
    else
    {
        /* calibrated */
    }

    return (err == ESP_OK);
}

bool adc_batt_pin_mv(uint32_t *mv)
{
    bool    ok  = false;
    int32_t out = 0;
    int     cal_mv;

    if ((mv != NULL) && (s_have == true))
    {
        if ((s_cali != NULL) &&
            (adc_cali_raw_to_voltage(s_cali, (int)s_avg_raw, &cal_mv) == ESP_OK))
        {
            out = (int32_t)cal_mv;
        }
        else
        {
            out = (int32_t)((s_avg_raw * BATT_FULL_MV) / BATT_RAW_FULL);
        }
        *mv = (out > 0) ? (uint32_t)out : 0U;
        ok  = true;
    }
    else
    {
        /* not started or no frame yet */
    }

    return ok;
}
