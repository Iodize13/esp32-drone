/**
 * pid.h - angle PID with filtered-gyro D-term and anti-windup.
 *
 * Application layer: no register or driver access. Inputs are plain
 * numbers (degrees, dps, seconds); output is a correction in per-mille
 * duty that the mixer adds to / subtracts from the motors.
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

/* first-order low-pass filter state */
typedef struct
{
    float alpha;          /* dt / (RC + dt)          */
    float y;              /* last output             */
    bool  primed;         /* first sample seeds y    */
} lpf_t;

typedef struct
{
    float kp;             /* per-mille per degree        */
    float ki;             /* per-mille per degree-second */
    float kd;             /* per-mille per dps           */
    float i_limit;        /* |ki * integral| clamp, per-mille */
    float out_limit;      /* |output| clamp, per-mille        */
    float d_cutoff_hz;    /* D-term low-pass cutoff           */
} pid_config_t;

typedef struct
{
    pid_config_t cfg;
    float        integral;   /* degree-seconds */
    lpf_t        d_lpf;
    float        last_p;     /* last terms, for telemetry/tuning */
    float        last_i;
    float        last_d;
} pid_ctrl_t;

void  lpf_init(lpf_t *f, float cutoff_hz, float dt_s);
float lpf_update(lpf_t *f, float x);

/* dt_s is the fixed control-loop period used to set up the D filter */
void  pid_init(pid_ctrl_t *p, const pid_config_t *cfg, float dt_s);

/* clear integral and filter state - call on arm / disarm */
void  pid_reset(pid_ctrl_t *p);

/**
 * One control step.
 *   setpoint_deg : wanted angle (0 for level hover)
 *   angle_deg    : measured angle from the attitude filter
 *   rate_dps     : gyro rate on the same axis (bias already removed)
 *   dt_s         : time since the previous call
 * Returns the correction in per-mille, clamped to +/- out_limit.
 */
float pid_update(pid_ctrl_t *p, float setpoint_deg, float angle_deg,
                 float rate_dps, float dt_s);

#endif /* PID_H */
