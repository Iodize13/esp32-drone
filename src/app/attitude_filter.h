/**
 * attitude_filter.h - complementary filter for roll and pitch.
 *
 * Application layer: takes scaled sensor values (g, dps), no I2C here.
 * Gyro is accurate short-term but drifts; accel has no drift but picks
 * up vibration and any acceleration. Blend the two: angle follows the
 * gyro, and is pulled slowly toward the accel angle.
 */

#ifndef ATTITUDE_FILTER_H
#define ATTITUDE_FILTER_H

#include <stdint.h>
#include <stdbool.h>

#define ATT_AXES  (3U)

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
