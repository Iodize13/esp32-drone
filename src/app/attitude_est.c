/*******************************************************************************
 * File Name    : attitude_est.c
 * Description  : attitude estimator (app layer) - เป็นเจ้าของ state ของ
 *                complementary filter ใช้ร่วมกันทั้ง mapping test และ PID test
 *                calibrate: เฉลี่ย gyro 1 วินาทีตอนเครื่องนิ่ง ถ้า bias เกิน
 *                10 dps แปลว่าเครื่องยังแกว่งอยู่ จะลองใหม่ (สูงสุด 5 ครั้ง)
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "attitude_est.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "attitude_filter.h"
#include "button.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ATT_TAU_S         (0.5f)
#define ATT_ACC_MIN_G     (0.85f)
#define ATT_ACC_MAX_G     (1.15f)
#define BIAS_SAMPLES      (1000U)        /* 1 s still at 1 kHz */
#define BIAS_MIN_SAMPLES  (BIAS_SAMPLES / 2U)
#define CAL_BIAS_MAX_DPS  (10.0f)        /* more = frame was swinging */
#define CAL_TRIES         (5U)
#define CAL_RETRY_MS      (500U)
#define ONE_TICK          (1U)
#define AXIS_X            (0U)
#define AXIS_Y            (1U)
#define AXIS_Z            (2U)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/
static attitude_t s_att;

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static bool calibrate_once(void);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * calibrate gyro bias ลองซ้ำจนเครื่องนิ่ง เพื่อไม่ให้การแกว่งหลังกด BOOT
 * ทำให้มุม drift ไปทั้งรอบ คืน false ถ้าไม่สำเร็จเลยทั้ง 5 ครั้ง
 */
bool att_est_calibrate(void)
{
    bool     ok = false;
    uint32_t n;

    for (n = 0U; (n < CAL_TRIES) && (ok == false); n++)
    {
        ok = calibrate_once();
        if (ok == false)
        {
            vTaskDelay(pdMS_TO_TICKS(CAL_RETRY_MS));
        }
        else
        {
            /* No action: calibrated */
        }
    }

    return ok;
}

/*
 * อ่าน IMU 1 ครั้ง ลบ bias แล้วอัปเดต filter ด้วย dt จริง
 * rate_dps ได้ gyro ที่ลบ bias แล้ว (ใช้เป็น D-term) - ไม่แตะถ้าอ่านพลาด
 */
bool att_est_update(float dt_s, float rate_dps[MPU_AXES])
{
    float acc[MPU_AXES];
    float raw[MPU_AXES];
    bool  ok;

    ok = mpu6500_read_all(acc, raw);
    if (ok == true)
    {
        att_unbias(&s_att, raw, rate_dps);
        att_update(&s_att, acc, rate_dps, dt_s);
    }
    else
    {
        /* No action: keep the last angle and rate */
    }

    return ok;
}

/*
 * วน filter ที่ 1 kHz นาน ms คืน true ถ้ากด BOOT ยกเลิก
 */
bool att_est_run(uint32_t ms)
{
    TickType_t last;
    uint32_t   i;
    bool       aborted = false;
    float      rate[MPU_AXES];

    last = xTaskGetTickCount();
    for (i = 0U; (i < ms) && (aborted == false); i++)
    {
        (void)att_est_update(ATT_EST_DT_S, rate);
        aborted = button_pressed();
        vTaskDelayUntil(&last, ONE_TICK);
    }

    return aborted;
}

/*
 * มุม roll ล่าสุด (องศา) ด้านซ้ายลง = บวก
 */
float att_est_roll(void)
{
    return s_att.roll_deg;
}

/*
 * มุม pitch ล่าสุด (องศา) หัวลง = ลบ
 */
float att_est_pitch(void)
{
    return s_att.pitch_deg;
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * เก็บ gyro 1000 sample ตอนนิ่ง เฉลี่ยเป็น bias แล้วเช็คว่าสมเหตุผล
 */
static bool calibrate_once(void)
{
    att_config_t acfg =
    {
        ATT_TAU_S, ATT_ACC_MIN_G, ATT_ACC_MAX_G
    };
    float    acc[MPU_AXES];
    float    raw[MPU_AXES];
    uint32_t i;
    bool     ok;

    att_init(&s_att, &acfg);
    for (i = 0U; i < BIAS_SAMPLES; i++)
    {
        if (mpu6500_read_all(acc, raw) == true)
        {
            att_bias_add(&s_att, raw);
        }
        else
        {
            /* No action: skip a bad read */
        }
        vTaskDelay(ONE_TICK);
    }

    ok = att_bias_finish(&s_att, BIAS_MIN_SAMPLES);
    for (i = 0U; i < MPU_AXES; i++)
    {
        if (fabsf(s_att.bias_dps[i]) > CAL_BIAS_MAX_DPS)
        {
            ok = false;                  /* frame was moving */
        }
        else
        {
            /* No action: plausible bias */
        }
    }

    if (ok == true)
    {
        (void)printf("gyro bias ok: ");
    }
    else
    {
        (void)printf("gyro bias REJECTED (frame moving?): ");
    }
    (void)printf("%.2f %.2f %.2f dps\n", (double)s_att.bias_dps[AXIS_X],
                 (double)s_att.bias_dps[AXIS_Y], (double)s_att.bias_dps[AXIS_Z]);

    return ok;
}
