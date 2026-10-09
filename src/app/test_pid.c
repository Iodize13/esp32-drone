/*******************************************************************************
 * File Name    : test_pid.c
 * Description  : angle PID test (ใส่ใบพัด) บน rig แกนเดียว
 *                loop เดินตาม data-ready interrupt ของ IMU (1 kHz):
 *                IMU -> filter -> PID -> mixer -> PWM
 *                ปรับ gain สดผ่าน serial: p/P kp, i/I ki, d/D kd, t/T throttle
 *                l/r/c เอียงซ้าย/ขวา/ตรง, 0 ปิด/เปิด PID, x หรือ BOOT ดับมอเตอร์
 *                failsafe: เอียงเกิน 45 องศา, I2C พลาด 20 ครั้งติด, แบตต่ำ
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "test_pid.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "esp_timer.h"
#include "app_util.h"
#include "attitude_est.h"
#include "battery.h"
#include "button.h"
#include "imu_int.h"
#include "mixer.h"
#include "motor_pwm.h"
#include "pid.h"
#include "serial_keys.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/
typedef struct
{
    pid_ctrl_t     pid[CTRL_AXES];       /* [AX_ROLL], [AX_PITCH] */
    float          rate[MPU_AXES];       /* gyro, bias removed    */
    float          ang[CTRL_AXES];
    float          sp;                   /* ramped, fed to the PID */
    float          sp_cmd;               /* target from l / r / c  */
    uint32_t       thr_target;
    uint16_t       duty[MOTOR_COUNT];
    uint32_t       err_run;              /* consecutive I2C errors */
    uint32_t       loops;
    uint32_t       int_miss;             /* loops without data-ready */
    batt_monitor_t bat;
} pid_state_t;

/* Private define ------------------------------------------------------------*/
#define PID_AXES_ROLL     (0U)
#define PID_AXES_PITCH    (1U)
#define PID_AXES_BOTH     (2U)
#define PID_AXES          (PID_AXES_ROLL)
#define M1                (0U)
#define M2                (1U)
#define M3                (2U)
#define M4                (3U)

#define PID_KP_START      (12.0f)        /* per-mille per degree        */
#define PID_KI_START      (7.0f)         /* per-mille per degree-second */
#define PID_KD_START      (2.0f)         /* per-mille per dps           */
#define PID_KP_STEP       (0.5f)
#define PID_KI_STEP       (0.5f)
#define PID_KD_STEP       (0.05f)
#define PID_I_LIMIT       (50.0f)        /* per-mille */
#define PID_OUT_LIMIT     (250.0f)       /* per-mille */
#define PID_D_CUTOFF_HZ   (40.0f)

#define PID_THROTTLE      (400U)         /* per-mille base, all motors */
#define PID_THR_STEP      (25U)
#define PID_THR_MAX       (700U)
#define PID_MAX_DUTY      (850U)
#define PID_IDLE_DUTY     (60U)          /* keep motors turning */
#define PID_SPOOL_MS      (1500U)        /* throttle ramp, no PID yet */
#define PID_STEP_DEG      (20.0f)
#define PID_SP_RATE_DPS   (20.0f)        /* setpoint ramp, deg per s */
#define PID_CUTOFF_DEG    (45.0f)        /* past this: motors off */
#define PID_MAX_READ_ERR  (20U)
#define PID_PRINT_MS      (50U)
#define PID_INT_WAIT_MS   (2U)           /* > 1 ms sample period */
#define PID_DT_MIN_S      (0.0002f)
#define PID_DT_MAX_S      (0.005f)
#define US_TO_S           (1.0e-6f)
#define ZERO_F            (0.0f)
#define INT_MISS_DIV      (2U)           /* > half the loops missed = broken */

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/
/* from the mapping test 2026-10-05: M1 FR, M2 BR, M3 BL, M4 FL */
static const mixer_config_t s_mix_cfg =
{
    .roll      = { 1, 1, -1, -1 },
    .pitch     = { 1, -1, -1, 1 },
    .yaw       = { 1, -1, 1, -1 },       /* direction unverified, unused */
    .max_duty  = (uint16_t)PID_MAX_DUTY,
    .idle_duty = (uint16_t)PID_IDLE_DUTY,
};

/* Private variables ------------------------------------------------------------*/
static bool s_pid_on = true;

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static const char *start_checks(void);
static void state_init(pid_state_t *st);
static float clamp_dt(float dt);
static void ramp_setpoint(pid_state_t *st, float dt);
static void control_step(pid_state_t *st, uint32_t elapsed_ms, float dt);
static const char *failsafe_check(pid_state_t *st, float dt);
static bool handle_key(pid_state_t *st);
static void apply_gain_key(pid_config_t *k, uint8_t c);
static float step_down(float v, float step);
static void print_gains(const pid_config_t *c, uint32_t thr, float sp);
static void print_log(const pid_state_t *st);
static void print_end(const pid_state_t *st, const char *why);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * เช็คแบต -> calibrate -> นับถอยหลัง แล้ววน control loop จนกว่าจะมี
 * failsafe หรือผู้ใช้สั่งหยุด ท้ายสุดดับมอเตอร์ทุกครั้ง
 */
void test_pid(void)
{
    pid_state_t st;
    const char *why;
    int64_t     now;
    int64_t     prev;
    int64_t     t_start;
    uint32_t    elapsed_ms;
    float       dt;

    (void)printf("ANGLE PID (roll rig) - PROPS ON, frame on the pivot rod, hands clear\n");
    (void)printf("keys: p/P kp  i/I ki  d/D kd  t/T throttle  l/r/c lean  0 PID on/off\n"
                 "      x stop  BOOT stop\n");
    (void)printf("Hold the frame still for calibration...\n");
    serial_keys_flush();                 /* drop keys typed while idle */

    why = start_checks();
    state_init(&st);
    print_gains(&st.pid[AX_ROLL].cfg, st.thr_target, st.sp);

    prev    = esp_timer_get_time();
    t_start = prev;

    while (why == NULL)
    {
        now  = esp_timer_get_time();
        dt   = clamp_dt((float)(now - prev) * US_TO_S);
        prev = now;

        if (att_est_update(dt, st.rate) == true)
        {
            st.err_run = 0U;
        }
        else
        {
            st.err_run++;                /* keep the last angle and rate */
        }
        st.ang[AX_ROLL]  = att_est_roll() - PID_TRIM_ROLL_DEG;
        st.ang[AX_PITCH] = att_est_pitch() - PID_TRIM_PITCH_DEG;

        ramp_setpoint(&st, dt);
        elapsed_ms = (uint32_t)((now - t_start) / US_PER_MS);
        control_step(&st, elapsed_ms, dt);
        why = failsafe_check(&st, dt);

        if ((st.loops % PID_PRINT_MS) == 0U)
        {
            print_log(&st);
        }
        else
        {
            /* No action: not a print step */
        }
        st.loops++;

        /* pace the loop on the IMU data-ready interrupt (EXTI); a timeout
         * means INT is not wired and the loop runs every PID_INT_WAIT_MS */
        if (imu_int_wait(PID_INT_WAIT_MS) == false)
        {
            st.int_miss++;
        }
        else
        {
            /* No action: new sample ready */
        }
    }

    motor_pwm_stop_all();
    print_end(&st, why);
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * ก่อนหมุนมอเตอร์: แบตพอ, calibrate ผ่าน, นับถอยหลังไม่ถูกยกเลิก
 * คืน NULL ถ้าพร้อม ไม่งั้นคืนเหตุผลที่ไม่เริ่ม
 */
static const char *start_checks(void)
{
    const char *why = NULL;

    if (battery_ok_to_start() == false)
    {
        why = "battery low - charge it";
    }
    else if (att_est_calibrate() == false)
    {
        why = "calibration failed";
    }
    else
    {
        (void)att_est_run(ATT_EST_SETTLE_MS);   /* settle the angle */
        if (app_countdown() == true)
        {
            why = "BOOT";
        }
        else
        {
            /* No action: ready */
        }
    }

    return why;
}

/*
 * ค่าเริ่มต้นของ state ทั้งหมด และ PID ทั้ง 2 แกน
 */
static void state_init(pid_state_t *st)
{
    pid_config_t pcfg =
    {
        PID_KP_START, PID_KI_START, PID_KD_START,
        PID_I_LIMIT, PID_OUT_LIMIT, PID_D_CUTOFF_HZ
    };
    uint32_t a;

    for (a = 0U; a < CTRL_AXES; a++)
    {
        pid_init(&st->pid[a], &pcfg, ATT_EST_DT_S);
        st->ang[a] = ZERO_F;
    }
    for (a = 0U; a < MPU_AXES; a++)
    {
        st->rate[a] = ZERO_F;
    }
    for (a = 0U; a < MOTOR_COUNT; a++)
    {
        st->duty[a] = 0U;
    }
    st->sp         = ZERO_F;
    st->sp_cmd     = ZERO_F;
    st->thr_target = PID_THROTTLE;
    st->err_run    = 0U;
    st->loops      = 0U;
    st->int_miss   = 0U;
    battery_monitor_reset(&st->bat);
    s_pid_on = true;
}

/*
 * dt จริงจาก esp_timer จำกัดไว้ในช่วงที่สมเหตุผล กัน dt เพี้ยนตอน
 * task ถูกขัดจังหวะนานๆ แล้ว I/filter กระโดด
 */
static float clamp_dt(float dt)
{
    float out = dt;

    if (dt < PID_DT_MIN_S)
    {
        out = PID_DT_MIN_S;
    }
    else if (dt > PID_DT_MAX_S)
    {
        out = PID_DT_MAX_S;
    }
    else
    {
        /* No action */
    }

    return out;
}

/*
 * เลื่อน setpoint เข้าหาคำสั่งที่ 20 องศา/วินาที แทนการกระโดดทีเดียว
 * (กระโดด 10 องศา P จะกระชากแรงจนเกินเป้า ~5 องศา)
 */
static void ramp_setpoint(pid_state_t *st, float dt)
{
    float sp_step;

    sp_step = PID_SP_RATE_DPS * dt;
    if ((st->sp_cmd - st->sp) > sp_step)
    {
        st->sp += sp_step;
    }
    else if ((st->sp - st->sp_cmd) > sp_step)
    {
        st->sp -= sp_step;
    }
    else
    {
        st->sp = st->sp_cmd;
    }
}

/*
 * คำนวณ throttle + แรงแก้ แล้วส่งให้ mixer และ PWM
 * ช่วง spool 1.5 s แรก เร่ง throttle อย่างเดียว ไม่ใช้ PID (มอเตอร์ยัง
 * หมุนช้าเกินจะตอบสนอง ถ้าเปิด PID ตอนนี้ I จะสะสมจนล้น)
 */
static void control_step(pid_state_t *st, uint32_t elapsed_ms, float dt)
{
    float    corr[CTRL_AXES] = { 0.0f, 0.0f };
    float    pitch_sp = ZERO_F;
    uint32_t thr;
    uint32_t a;

    if (elapsed_ms < PID_SPOOL_MS)
    {
        thr = (st->thr_target * elapsed_ms) / PID_SPOOL_MS;
        if (thr == 0U)
        {
            thr = 1U;                    /* 0 would mean "disarmed" in the mixer */
        }
        else
        {
            /* No action */
        }
    }
    else if (s_pid_on == false)
    {
        thr = st->thr_target;            /* demo: throttle only */
    }
    else
    {
        thr = st->thr_target;
        if (PID_AXES != PID_AXES_PITCH)
        {
            corr[AX_ROLL] = pid_update(&st->pid[AX_ROLL], st->sp, st->ang[AX_ROLL],
                                       st->rate[AX_ROLL], dt);
        }
        else
        {
            pitch_sp = st->sp;           /* pitch rig: l / r tilt the nose */
        }
        if (PID_AXES != PID_AXES_ROLL)
        {
            corr[AX_PITCH] = pid_update(&st->pid[AX_PITCH], pitch_sp, st->ang[AX_PITCH],
                                        st->rate[AX_PITCH], dt);
        }
        else
        {
            /* No action: pitch not controlled */
        }
    }

    mixer_mix(&s_mix_cfg, (uint16_t)thr, corr[AX_ROLL], corr[AX_PITCH], ZERO_F, st->duty);
    for (a = 0U; a < MOTOR_COUNT; a++)
    {
        motor_pwm_set_one(a, st->duty[a]);
    }
}

/*
 * เช็ค failsafe ทุกรอบ คืน NULL ถ้าบินต่อได้ ไม่งั้นคืนเหตุผล
 */
static const char *failsafe_check(pid_state_t *st, float dt)
{
    const char *why = NULL;
    bool        tilt = false;
    bool        bat_cut;

    if ((PID_AXES != PID_AXES_PITCH) && (app_absf(st->ang[AX_ROLL]) > PID_CUTOFF_DEG))
    {
        tilt = true;
    }
    else if ((PID_AXES != PID_AXES_ROLL) && (app_absf(st->ang[AX_PITCH]) > PID_CUTOFF_DEG))
    {
        tilt = true;
    }
    else
    {
        /* No action */
    }
    bat_cut = battery_monitor_update(&st->bat, dt);

    if (tilt == true)
    {
        why = "angle limit";
    }
    else if (st->err_run >= PID_MAX_READ_ERR)
    {
        why = "I2C errors";
    }
    else if (bat_cut == true)
    {
        why = "battery low";
    }
    else if (button_pressed() == true)
    {
        why = "BOOT";
    }
    else if (handle_key(st) == true)
    {
        why = "x key";
    }
    else
    {
        /* No action: keep flying */
    }

    return why;
}

/*
 * อ่านปุ่ม 1 ตัวจาก serial แล้วทำตาม คืน true เมื่อกด 'x' (หยุด)
 * ทุกครั้งที่ค่าเปลี่ยนจะพิมพ์บรรทัด gain ให้ live_plot รู้
 */
static bool handle_key(pid_state_t *st)
{
    uint8_t  c       = 0U;
    bool     stop    = false;
    bool     changed = true;
    uint32_t a;

    if (serial_keys_get(&c) == true)
    {
        switch (c)
        {
            case 'P':
            case 'p':
            case 'I':
            case 'i':
            case 'D':
            case 'd':
                /* intended fall-through: all gain keys share one handler */
                for (a = 0U; a < CTRL_AXES; a++)
                {
                    apply_gain_key(&st->pid[a].cfg, c);
                }
                break;
            case 'T':
                st->thr_target += PID_THR_STEP;
                if (st->thr_target > PID_THR_MAX)
                {
                    st->thr_target = PID_THR_MAX;
                }
                else
                {
                    /* No action */
                }
                break;
            case 't':
                if (st->thr_target > PID_THR_STEP)
                {
                    st->thr_target -= PID_THR_STEP;
                }
                else
                {
                    st->thr_target = 0U;
                }
                break;
            case 'l':
                st->sp_cmd = PID_STEP_DEG;        /* left side down = +roll */
                break;
            case 'r':
                st->sp_cmd = -PID_STEP_DEG;
                break;
            case 'c':
                st->sp_cmd = ZERO_F;
                break;
            case 'x':
                stop = true;
                break;
            case '0':
                s_pid_on = (s_pid_on == false);
                for (a = 0U; a < CTRL_AXES; a++)
                {
                    pid_reset(&st->pid[a]);       /* no stale I or D on resume */
                }
                break;
            default:
                changed = false;
                (void)printf("key 0x%02x ignored\n", (unsigned int)c);
                break;
        }
    }
    else
    {
        changed = false;
    }

    if (changed == true)
    {
        print_gains(&st->pid[AX_ROLL].cfg, st->thr_target, st->sp_cmd);
    }
    else
    {
        /* No action: nothing to report */
    }

    return stop;
}

/*
 * เพิ่ม/ลด kp ki kd ตามปุ่ม ตัวพิมพ์ใหญ่ = เพิ่ม, ตัวพิมพ์เล็ก = ลด (ไม่ต่ำกว่า 0)
 */
static void apply_gain_key(pid_config_t *k, uint8_t c)
{
    switch (c)
    {
        case 'P':
            k->kp += PID_KP_STEP;
            break;
        case 'p':
            k->kp = step_down(k->kp, PID_KP_STEP);
            break;
        case 'I':
            k->ki += PID_KI_STEP;
            break;
        case 'i':
            k->ki = step_down(k->ki, PID_KI_STEP);
            break;
        case 'D':
            k->kd += PID_KD_STEP;
            break;
        case 'd':
            k->kd = step_down(k->kd, PID_KD_STEP);
            break;
        default:
            /* No action: not a gain key */
            break;
    }
}

/*
 * ลดค่าทีละ step แต่ไม่ให้ติดลบ
 */
static float step_down(float v, float step)
{
    float out = ZERO_F;

    if (v > step)
    {
        out = v - step;
    }
    else
    {
        /* No action */
    }

    return out;
}

/*
 * บรรทัด gain (live_plot ใช้ regex อ่านบรรทัดนี้ ห้ามเปลี่ยนรูปแบบ)
 */
static void print_gains(const pid_config_t *c, uint32_t thr, float sp)
{
    const char *on = "OFF";

    if (s_pid_on == true)
    {
        on = "ON";
    }
    else
    {
        /* No action */
    }
    (void)printf("\nkp %.2f  ki %.2f  kd %.3f  thr %lu  sp %+.1f  PID %s\n",
                 (double)c->kp, (double)c->ki, (double)c->kd,
                 (unsigned long)thr, (double)sp, on);
}

/*
 * log ทุก 50 ms (live_plot ใช้ regex อ่านบรรทัดนี้ ห้ามเปลี่ยนรูปแบบ)
 */
static void print_log(const pid_state_t *st)
{
    uint32_t    a    = AX_ROLL;
    const char *name = "R";

    if (PID_AXES == PID_AXES_PITCH)
    {
        a    = AX_PITCH;
        name = "P";
    }
    else
    {
        /* No action */
    }

    (void)printf("%s %+6.1f sp %+5.1f rate %+7.1f P %+5.0f I %+5.0f D %+5.0f",
                 name, (double)st->ang[a], (double)st->sp, (double)st->rate[a],
                 (double)st->pid[a].last_p, (double)st->pid[a].last_i,
                 (double)st->pid[a].last_d);
    (void)printf(" M %3u %3u %3u %3u B %4lu\n",
                 (unsigned int)st->duty[M1], (unsigned int)st->duty[M2],
                 (unsigned int)st->duty[M3], (unsigned int)st->duty[M4],
                 (unsigned long)battery_mv());
}

/*
 * สรุปตอนดับมอเตอร์: เหตุผล และว่า EXTI ทำงานหรือไม่
 */
static void print_end(const pid_state_t *st, const char *why)
{
    (void)printf("\nMOTORS OFF (%s)\n", why);
    (void)printf("EXTI: %lu loops, %lu without data-ready ",
                 (unsigned long)st->loops, (unsigned long)st->int_miss);
    if (st->int_miss > (st->loops / INT_MISS_DIV))
    {
        (void)printf("(INT NOT WORKING - check GPIO10 wire)\n");
    }
    else
    {
        (void)printf("(ok)\n");
    }
    print_gains(&st->pid[AX_ROLL].cfg, st->thr_target, st->sp);
    (void)printf("\n");
}
