/*******************************************************************************
 * File Name    : attitude_filter.c
 * Description  : complementary filter หามุม roll/pitch จาก gyro + accel (app layer)
 *                alpha = tau / (tau + dt): tau 0.5 s, dt 1 ms -> alpha ~0.998 มุมตาม gyro
 *                และถูกดึงเข้าหามุมจาก accel ช้าๆ เพื่อลบ drift
 *                sample ที่ขนาด accel ห่างจาก 1 g มาก (แรงสั่น/เร่ง) จะใช้ gyro อย่างเดียว
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <math.h>
#include "attitude_filter.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RAD_TO_DEG   (57.29578f)
#define ZERO_F       (0.0f)
#define AX           (0U)
#define AY           (1U)
#define AZ           (2U)
#define ONE_F        (1.0f)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * ตั้งค่า config และล้างมุม/bias ทั้งหมด
 */
void att_init(attitude_t *att, const att_config_t *cfg)
{
    uint32_t a;

    if ((att != NULL) && (cfg != NULL))
    {
        att->cfg         = *cfg;
        att->roll_deg    = ZERO_F;
        att->pitch_deg   = ZERO_F;
        att->bias_n      = 0U;
        att->initialised = false;
        att->acc_used    = false;

        for (a = 0U; a < ATT_AXES; a++)
        {
            att->bias_dps[a] = ZERO_F;
            att->bias_sum[a] = ZERO_F;
        }
    }
    else
    {
        /* invalid arguments */
    }
}

/*
 * สะสม gyro ตอนเครื่องนิ่ง เพื่อหา bias
 */
void att_bias_add(attitude_t *att, const float gyro_dps[ATT_AXES])
{
    uint32_t a;

    if ((att != NULL) && (gyro_dps != NULL))
    {
        for (a = 0U; a < ATT_AXES; a++)
        {
            att->bias_sum[a] += gyro_dps[a];
        }
        att->bias_n++;
    }
    else
    {
        /* invalid arguments */
    }
}

/*
 * เฉลี่ย bias จากที่สะสมไว้ คืน false ถ้า sample น้อยเกิน (IMU อ่านพลาดบ่อย)
 */
bool att_bias_finish(attitude_t *att, uint32_t min_samples)
{
    bool     ok = false;
    uint32_t a;

    if ((att != NULL) && (att->bias_n >= min_samples) && (att->bias_n > 0U))
    {
        for (a = 0U; a < ATT_AXES; a++)
        {
            att->bias_dps[a] = att->bias_sum[a] / (float)att->bias_n;
            att->bias_sum[a] = ZERO_F;
        }
        att->bias_n = 0U;
        ok = true;
    }
    else
    {
        /* not enough samples - keep the old bias */
    }

    return ok;
}

/*
 * ลบ bias ออกจาก gyro ดิบ
 */
void att_unbias(const attitude_t *att, const float raw_dps[ATT_AXES],
                float out_dps[ATT_AXES])
{
    uint32_t a;

    if ((att != NULL) && (raw_dps != NULL) && (out_dps != NULL))
    {
        for (a = 0U; a < ATT_AXES; a++)
        {
            out_dps[a] = raw_dps[a] - att->bias_dps[a];
        }
    }
    else
    {
        /* invalid arguments */
    }
}

/*
 * filter 1 รอบ: มุมจาก gyro (มุมเดิม + rate*dt) ผสมกับมุมจาก accel
 * รอบแรกที่ accel ใช้ได้ ตั้งมุมจาก accel ตรงๆ (เริ่มจากท่าที่วางอยู่จริง)
 */
void att_update(attitude_t *att, const float acc_g[ATT_AXES],
                const float gyro_dps[ATT_AXES], float dt_s)
{
    float mag;
    float acc_roll;
    float acc_pitch;
    float alpha;
    float gyro_roll;
    float gyro_pitch;

    if ((att != NULL) && (acc_g != NULL) && (gyro_dps != NULL) && (dt_s > ZERO_F))
    {
        mag = sqrtf((acc_g[AX] * acc_g[AX]) + (acc_g[AY] * acc_g[AY]) +
                    (acc_g[AZ] * acc_g[AZ]));

        acc_roll  = atan2f(acc_g[AY], acc_g[AZ]) * RAD_TO_DEG;
        acc_pitch = atan2f(-acc_g[AX],
                           sqrtf((acc_g[AY] * acc_g[AY]) + (acc_g[AZ] * acc_g[AZ])))
                    * RAD_TO_DEG;

        att->acc_used = (mag >= att->cfg.acc_min_g) && (mag <= att->cfg.acc_max_g);

        if (att->initialised == false)
        {
            if (att->acc_used == true)
            {
                att->roll_deg    = acc_roll;     /* start from where we are */
                att->pitch_deg   = acc_pitch;
                att->initialised = true;
            }
            else
            {
                /* wait for a clean accel sample */
            }
        }
        else
        {
            gyro_roll  = att->roll_deg  + (gyro_dps[AX] * dt_s);
            gyro_pitch = att->pitch_deg + (gyro_dps[AY] * dt_s);

            if (att->acc_used == true)
            {
                alpha          = att->cfg.tau_s / (att->cfg.tau_s + dt_s);
                att->roll_deg  = (alpha * gyro_roll)  + ((ONE_F - alpha) * acc_roll);
                att->pitch_deg = (alpha * gyro_pitch) + ((ONE_F - alpha) * acc_pitch);
            }
            else
            {
                att->roll_deg  = gyro_roll;      /* gyro only this step */
                att->pitch_deg = gyro_pitch;
            }
        }
    }
    else
    {
        /* invalid arguments */
    }
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/
