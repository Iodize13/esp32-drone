/*******************************************************************************
 * File Name    : pid.c
 * Description  : angle PID ของ app layer (ไม่แตะ hardware) ใช้ทั้งแกน roll และ pitch
 *                D-term ใช้ gyro rate (ผ่าน low-pass) แทนการ diff มุม เพราะ diff มุม
 *                หารด้วย dt 1 ms จะขยาย noise และไม่เกิด derivative kick ตอน setpoint เปลี่ยน
 *                anti-windup: clamp I และหยุดสะสม I ตอน output อิ่มตัวทิศเดียวกัน
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "pid.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PI_F          (3.14159265f)
#define TWO           (2.0f)
#define ZERO_F        (0.0f)
#define ONE_F         (1.0f)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static float clampf(float x, float lim);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * low-pass อันดับ 1: alpha = dt / (RC + dt) โดย RC = 1 / (2 pi fc)
 * cutoff <= 0 แปลว่าไม่กรอง (alpha = 1)
 */
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

/*
 * y += alpha * (x - y) ค่าแรกใช้ x ตรงๆ จะได้ไม่ไต่ขึ้นจาก 0
 */
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

/*
 * คัดลอก gain, ตั้ง filter ของ D ตามคาบ loop แล้วล้าง state
 */
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

/*
 * ล้าง I และ filter ของ D - เรียกตอนเริ่มบิน หรือเปิด PID กลับ
 * กันไม่ให้ค่าเก่าค้างแล้วกระชากมอเตอร์
 */
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

/*
 * คำนวณ 1 รอบ: P = kp * error, D = -kd * gyro rate (กรองแล้ว),
 * I = ki * integral (clamp + conditional integration) คืนค่ารวมที่ clamp แล้ว
 */
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

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * จำกัด x ให้อยู่ในช่วง -lim .. +lim
 */
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
