/*******************************************************************************
 * File Name    : adc_batt.c
 * Description  : วัดแรงดันแบตที่ GPIO1 (ADC1_CH0) ผ่าน resistor divider
 *                ADC ทำงานแบบ continuous + DMA: ADC อ่านเอง 20 kHz แล้ว DMA
 *                เขียนลง buffer เอง เมื่อครบ 1 frame (256 ค่า) จะเกิด interrupt
 *                on_conv_done ซึ่งเฉลี่ยค่าเก็บไว้ ไม่มีการวนรอ ADC (no polling)
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "adc_batt.h"
#include <stddef.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BATT_ADC_UNIT       (ADC_UNIT_1)
#define BATT_ADC_CHANNEL    (ADC_CHANNEL_0)     /* GPIO1 */
#define BATT_ADC_ATTEN      (ADC_ATTEN_DB_12)   /* about 0-3.1 V */
#define BATT_ADC_BITS       (ADC_BITWIDTH_12)
#define BATT_SAMPLE_HZ      (20000U)            /* 1 kHz gave no frames */
#define BATT_FRAME_SAMPLES  (256U)              /* ~13 ms per ISR */
#define BATT_RESULT_BYTES   (4U)                /* SOC_ADC_DIGI_RESULT_BYTES on S3 */
#define BATT_FRAME_BYTES    (BATT_FRAME_SAMPLES * BATT_RESULT_BYTES)
#define BATT_POOL_FRAMES    (4U)
#define BATT_POOL_BYTES     (BATT_FRAME_BYTES * BATT_POOL_FRAMES)
#define BATT_PATTERN_NUM    (1U)
#define BATT_RAW_FULL       (4095U)
#define BATT_FULL_MV        (3100U)             /* uncalibrated fallback */

/* S3 DMA result word (type2): data bits 0-11, channel 13-16, unit 17 */
#define RES_DATA_MASK       (0x0FFFU)
#define RES_CH_SHIFT        (13U)
#define RES_CH_MASK         (0x0FU)
#define RES_UNIT_SHIFT      (17U)
#define RES_UNIT_MASK       (0x01U)
#define RES_UNIT_ADC1       (0U)
#define BYTE1_SHIFT         (8U)
#define BYTE2_SHIFT         (16U)
#define BYTE3_SHIFT         (24U)
#define IDX_BYTE1           (1U)
#define IDX_BYTE2           (2U)
#define IDX_BYTE3           (3U)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/
static adc_continuous_handle_t s_adc  = NULL;
static adc_cali_handle_t       s_cali = NULL;
static volatile uint32_t       s_avg_raw = 0U;
static volatile bool           s_have    = false;

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static bool on_conv_done(adc_continuous_handle_t handle,
                         const adc_continuous_evt_data_t *edata,
                         void *user_data);
static uint32_t result_word(const uint8_t *p);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * ตั้ง ADC continuous 1 channel, ผูก ISR on_conv_done แล้วเริ่ม DMA
 * calibration (curve fitting จาก eFuse) สร้างหลังสุด ถ้าชิปไม่มีค่า
 * eFuse ก็ยังทำงานได้ แค่แปลงหน่วยแบบเส้นตรงแทน
 */
bool adc_batt_init(void)
{
    adc_continuous_handle_cfg_t hcfg =
    {
        .max_store_buf_size = BATT_POOL_BYTES,
        .conv_frame_size    = BATT_FRAME_BYTES,
        .flags              =
        {
            .flush_pool = 1U,
        },
    };
    adc_digi_pattern_config_t pat =
    {
        .atten     = (uint8_t)BATT_ADC_ATTEN,
        .channel   = (uint8_t)BATT_ADC_CHANNEL,
        .unit      = (uint8_t)BATT_ADC_UNIT,
        .bit_width = (uint8_t)BATT_ADC_BITS,
    };
    adc_continuous_config_t ccfg =
    {
        .pattern_num    = BATT_PATTERN_NUM,
        .adc_pattern    = &pat,
        .sample_freq_hz = BATT_SAMPLE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
    };
    adc_continuous_evt_cbs_t cbs =
    {
        .on_conv_done = on_conv_done,
        .on_pool_ovf  = NULL,
    };
    adc_cali_curve_fitting_config_t cal =
    {
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
        /* No action: handle failed */
    }
    if (err == ESP_OK)
    {
        err = adc_continuous_register_event_callbacks(s_adc, &cbs, NULL);
    }
    else
    {
        /* No action: config failed */
    }
    if (err == ESP_OK)
    {
        err = adc_continuous_start(s_adc);
    }
    else
    {
        /* No action: callback registration failed */
    }

    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK)
    {
        s_cali = NULL;                   /* fall back to linear scaling */
    }
    else
    {
        /* No action: calibrated */
    }

    return (err == ESP_OK);
}

/*
 * แรงดันที่ขา GPIO1 (mV) จากค่าเฉลี่ยล่าสุดที่ ISR เก็บไว้
 * ไม่ได้สั่ง ADC อ่าน แค่อ่านตัวแปร คืน false ถ้ายังไม่มี frame เข้ามา
 */
bool adc_batt_pin_mv(uint32_t *mv)
{
    bool    ok     = false;
    int32_t out    = 0;
    int     cal_mv = 0;

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

        if (out > 0)
        {
            *mv = (uint32_t)out;
        }
        else
        {
            *mv = 0U;
        }
        ok = true;
    }
    else
    {
        /* No action: not started or no frame yet */
    }

    return ok;
}

/* Callback functions ------------------------------------------------------------*/

/*
 * ISR: DMA เติม buffer ครบ 1 frame แล้ว - เฉลี่ยเฉพาะ sample ของ ADC1_CH0
 * คืน false เพราะไม่ได้ปลุก task ไหน
 */
static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t handle,
                                   const adc_continuous_evt_data_t *edata,
                                   void *user_data)
{
    uint32_t sum = 0U;
    uint32_t n   = 0U;
    uint32_t i;
    uint32_t w;
    uint32_t ch;
    uint32_t unit;

    (void)handle;
    (void)user_data;

    for (i = 0U; (i + BATT_RESULT_BYTES) <= edata->size; i += BATT_RESULT_BYTES)
    {
        w    = result_word(&edata->conv_frame_buffer[i]);
        ch   = (w >> RES_CH_SHIFT) & RES_CH_MASK;
        unit = (w >> RES_UNIT_SHIFT) & RES_UNIT_MASK;
        if ((unit == RES_UNIT_ADC1) && (ch == (uint32_t)BATT_ADC_CHANNEL))
        {
            sum += (w & RES_DATA_MASK);
            n++;
        }
        else
        {
            /* No action: other channel or invalid sample */
        }
    }

    if (n > 0U)
    {
        s_avg_raw = sum / n;
        s_have    = true;
    }
    else
    {
        /* No action: keep the previous average */
    }

    return false;
}

/* Private functions ------------------------------------------------------------*/

/*
 * ประกอบ result 4 byte (little-endian) เป็น uint32 แทนการ cast pointer
 * ไปเป็น struct (MISRA ไม่ให้ cast pointer ข้าม type)
 */
static uint32_t IRAM_ATTR result_word(const uint8_t *p)
{
    uint32_t w;

    w = (uint32_t)p[0];
    w = w | ((uint32_t)p[IDX_BYTE1] << BYTE1_SHIFT);
    w = w | ((uint32_t)p[IDX_BYTE2] << BYTE2_SHIFT);
    w = w | ((uint32_t)p[IDX_BYTE3] << BYTE3_SHIFT);

    return w;
}
