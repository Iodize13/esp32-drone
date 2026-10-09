/*******************************************************************************
 * File Name    : attitude_filter.h
 * Description  : Header ของ complementary filter หามุม roll/pitch (app layer)
 *                gyro แม่นระยะสั้นแต่ drift, accel ไม่ drift แต่โดนแรงสั่นกวน จึงผสมกัน
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef ATTITUDE_FILTER_H
#define ATTITUDE_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported define/macro/constants --------------------------------------------*/
#define ATT_AXES  (3U)

/* Exported typedef/enum/struct/union -----------------------------------------*/
typedef struct
{
    float tau_s;        /* time constant: how slowly accel corrects drift */
    float acc_min_g;    /* accel magnitude outside this band is ignored   */
    float acc_max_g;    /* (vibration spike or real acceleration)         */
} att_config_t;

typedef struct
{
    att_config_t cfg;
    float        roll_deg;
    float        pitch_deg;
    float        bias_dps[ATT_AXES];
    float        bias_sum[ATT_AXES];
    uint32_t     bias_n;
    bool         initialised;   /* first accel sample seeds the angles */
    bool         acc_used;      /* last update used the accel          */
} attitude_t;

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
void att_init(attitude_t *att, const att_config_t *cfg);

/* Gyro bias: feed samples while the frame is still, then finish. */
void att_bias_add(attitude_t *att, const float gyro_dps[ATT_AXES]);
bool att_bias_finish(attitude_t *att, uint32_t min_samples);

/* Removes the stored bias from a raw gyro reading. */
void att_unbias(const attitude_t *att, const float raw_dps[ATT_AXES],
                float out_dps[ATT_AXES]);

/**
 * One filter step.
 *   acc_g    : accel x, y, z in g
 *   gyro_dps : gyro x, y, z in dps, bias already removed (att_unbias)
 *   dt_s     : time since the previous call
 * Axis convention follows the IMU: roll about X, pitch about Y.
 */
void att_update(attitude_t *att, const float acc_g[ATT_AXES],
                const float gyro_dps[ATT_AXES], float dt_s);

#endif /* ATTITUDE_FILTER_H */
