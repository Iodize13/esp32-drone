/**
 * pid.c - angle PID with filtered-gyro D-term and anti-windup.
 *
 * D-term: the gyro already measures d(angle)/dt, so the derivative is
 * taken from the gyro rate instead of differencing the angle (which
 * divides noise by dt = 1 ms and blows it up). The rate is low-pass
 * filtered first because prop vibration lands mostly in the gyro.
 * Using the measurement instead of the error also avoids a "derivative
 * kick" when the setpoint jumps.
 *
 * Anti-windup: the integral is clamped, and it stops growing while the
 * output is saturated in the same direction (conditional integration).
 */

#include <stddef.h>
#include "pid.h"

#define PI_F          (3.14159265f)
#define TWO           (2.0f)
#define ZERO_F        (0.0f)
#define ONE_F         (1.0f)

static float clampf(float x, float lim)
{
    float out;

    if (x > lim)
    {
        out = lim;
    }
    else if (x < -lim)
    {
        out = -lim;
    }
    else
    {
        out = x;
    }

    return out;
}

void lpf_init(lpf_t *f, float cutoff_hz, float dt_s)
{
    float rc;

    if (f != NULL)
    {
        if ((cutoff_hz > ZERO_F) && (dt_s > ZERO_F))
        {
            rc       = ONE_F / (TWO * PI_F * cutoff_hz);
            f->alpha = dt_s / (rc + dt_s);
        }
        else
        {
            f->alpha = ONE_F;             /* no filtering */
        }
        f->y      = ZERO_F;
        f->primed = false;
    }
    else
    {
        /* nothing to init */
    }
}

float lpf_update(lpf_t *f, float x)
{
    float out = x;

    if (f != NULL)
    {
        if (f->primed == false)
        {
            f->y      = x;                /* no start-up ramp from 0 */
            f->primed = true;
        }
        else
        {
            f->y += f->alpha * (x - f->y);
        }
        out = f->y;
    }
    else
    {
        /* pass through */
    }

    return out;
}

void pid_init(pid_ctrl_t *p, const pid_config_t *cfg, float dt_s)
{
    if ((p != NULL) && (cfg != NULL))
    {
        p->cfg = *cfg;
        lpf_init(&p->d_lpf, cfg->d_cutoff_hz, dt_s);
        pid_reset(p);
    }
    else
    {
        /* invalid arguments */
    }
}

void pid_reset(pid_ctrl_t *p)
{
    if (p != NULL)
    {
        p->integral     = ZERO_F;
        p->d_lpf.y      = ZERO_F;
        p->d_lpf.primed = false;
        p->last_p       = ZERO_F;
        p->last_i       = ZERO_F;
        p->last_d       = ZERO_F;
    }
    else
    {
        /* nothing to reset */
    }
}

float pid_update(pid_ctrl_t *p, float setpoint_deg, float angle_deg,
                 float rate_dps, float dt_s)
{
    float out = ZERO_F;
    float error;
    float rate_f;
    float i_term;
    float unsat;
    float new_integral;

    if ((p != NULL) && (dt_s > ZERO_F))
    {
        error  = setpoint_deg - angle_deg;
        rate_f = lpf_update(&p->d_lpf, rate_dps);

        p->last_p = p->cfg.kp * error;
        p->last_d = -(p->cfg.kd * rate_f);     /* oppose the rotation */

        /* candidate integral, kept inside the I clamp */
        new_integral = p->integral + (error * dt_s);
        if (p->cfg.ki > ZERO_F)
        {
            i_term = clampf(p->cfg.ki * new_integral, p->cfg.i_limit);
            new_integral = i_term / p->cfg.ki;
        }
        else
        {
            i_term       = ZERO_F;
            new_integral = ZERO_F;
        }

        unsat = p->last_p + i_term + p->last_d;

        /* conditional integration: do not wind further into saturation */
        if (((unsat > p->cfg.out_limit) && (error > ZERO_F)) ||
            ((unsat < -p->cfg.out_limit) && (error < ZERO_F)))
        {
            i_term = p->cfg.ki * p->integral;  /* keep the old integral */
            unsat  = p->last_p + i_term + p->last_d;
        }
        else
        {
            p->integral = new_integral;
        }

        p->last_i = i_term;
        out       = clampf(unsat, p->cfg.out_limit);
    }
    else
    {
        /* invalid arguments - no correction */
    }

    return out;
}
