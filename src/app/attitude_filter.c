/**
 * attitude_filter.c - complementary filter for roll and pitch.
 *
 * alpha = tau / (tau + dt). With tau = 0.5 s and dt = 1 ms, alpha is
 * ~0.998: the gyro dominates, the accel slowly removes drift.
 *
 * Accel samples whose magnitude is far from 1 g are skipped. Prop
 * vibration and thrust changes make the accel lie for short moments;
 * trusting the gyro alone for those samples keeps the angle clean.
 */

#include <stddef.h>
#include <math.h>
#include "attitude_filter.h"

#define RAD_TO_DEG   (57.29578f)
#define ZERO_F       (0.0f)
#define AX           (0U)
#define AY           (1U)
#define AZ           (2U)

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
                att->roll_deg  = (alpha * gyro_roll)  + ((1.0f - alpha) * acc_roll);
                att->pitch_deg = (alpha * gyro_pitch) + ((1.0f - alpha) * acc_pitch);
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
