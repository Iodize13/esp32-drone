/*******************************************************************************
 * File Name    : pid.h
 * Description  : Header ของ angle PID (app layer, ไม่แตะ hardware)
 *                input เป็นองศา, dps, วินาที - output เป็นแรงแก้หน่วย per-mille ให้ mixer
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef PID_H
#define PID_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/
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

/* Exported define/macro/constants --------------------------------------------*/

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
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
