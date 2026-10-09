/*******************************************************************************
 * File Name    : test_bench.c
 * Description  : การทดสอบบนโต๊ะก่อนทำ PID
 *                vibration: วัด noise ของ gyro ตอนมอเตอร์ดับ เทียบกับตอนหมุน 40 %
 *                  (ทั้ง 4 ตัว แล้วทีละตัว) เป้าหมาย +/-3 dps ถ้าเกิน D-term
 *                  จะขยาย noise จน tune ไม่ได้
 *                thrust: เร่ง 0 -> 100 % ช้าๆ ดู az ว่าแรงยกเกินน้ำหนักหรือยัง
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "test_bench.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_util.h"
#include "button.h"
#include "motor_pwm.h"
#include "mpu6500.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/
typedef struct
{
    uint32_t n;
    float    mean;
    float    m2;                         /* Welford running sum of squares */
    float    vmin;
    float    vmax;
} stat_t;

/* Private define ------------------------------------------------------------*/
#define TEST_DUTY         (400U)         /* 40 % */
#define RAMP_STEP_DUTY    (10U)
#define RAMP_STEP_MS      (20U)
#define PHASE_SETTLE_MS   (2000U)
#define PHASE_MEASURE_MS  (5000U)
#define STAGGER_MS        (100U)
#define THRUST_STEP_DUTY  (10U)          /* 1 % per step       */
#define THRUST_STEP_MS    (100U)         /* 0 -> 100 % in 10 s */
#define THRUST_HOLD_MS    (2000U)
#define THRUST_PRINT_EVERY (5U)
#define DUTY_PER_PERCENT  (10U)

/* ~99 % of gaussian noise sits inside +/-3 sigma, so the verdict uses
 * 3 x std instead of the raw peak (one spike should not fail the test) */
#define SIGMA_BAND        (3.0f)
#define LIMIT_PASS_DPS    (3.0f)
#define LIMIT_FAIL_DPS    (10.0f)
#define SATURATION_DPS    (245.0f)       /* +/-250 dps full scale */
#define MAX_DEV_DPS       (60.0f)
#define MIN_SAMPLES_MEAN  (10U)
/* motors always add noise; std this close to idle = they never spun */
#define SPIN_STD_RATIO    (2.0f)
#define STAT_INIT_MIN     (1.0e9f)
#define STAT_INIT_MAX     (-1.0e9f)
#define ZERO_F            (0.0f)
#define ONE_TICK          (1U)
#define AXIS_X            (0U)
#define AXIS_Y            (1U)
#define AXIS_Z            (2U)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/
static const char *const motor_name[MOTOR_COUNT] =
{
    "M1 G4", "M2 G5", "M3 G6", "M4 G7"
};

/* Private variables ------------------------------------------------------------*/
static uint32_t s_read_err = 0U;         /* I2C transfers that failed      */
static uint32_t s_rejected = 0U;         /* reads that came back corrupted */

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static void stat_reset(stat_t *s);
static void stat_add(stat_t *s, float x);
static float stat_std(const stat_t *s);
static bool sample_is_sane(const stat_t st[MPU_AXES], const float g[MPU_AXES]);
static bool capture(stat_t st[MPU_AXES], uint32_t duration_ms);
static void print_stats(const char *name, const stat_t st[MPU_AXES]);
static bool ramp(uint32_t idx, uint32_t target);
static bool ramp_staggered(uint32_t target);
static void verdict(const stat_t idle[MPU_AXES], const stat_t run[MPU_AXES]);
static void print_end(bool aborted);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * 1) มอเตอร์ดับ วัด baseline 2) ทั้ง 4 ตัว 40 % (เริ่มทีละตัว)
 * 3) ทีละตัว 40 % - กด BOOT เมื่อไหร่ก็ดับมอเตอร์ทันที
 */
void test_vibration(void)
{
    stat_t   idle[MPU_AXES];
    stat_t   run[MPU_AXES];
    uint32_t m;
    bool     aborted;

    aborted = app_countdown();

    if (aborted == false)
    {
        (void)printf("[1/3] motors OFF - baseline\n");
        motor_pwm_stop_all();
        aborted = app_wait_abortable(PHASE_SETTLE_MS);
    }
    else
    {
        /* No action: skip */
    }

    if (aborted == false)
    {
        (void)capture(idle, PHASE_MEASURE_MS);   /* motors off - nothing to abort */
        print_stats("idle", idle);
        (void)printf("[2/3] all motors 40%% (staggered start)\n");
        aborted = ramp_staggered(TEST_DUTY);
        aborted = aborted || app_wait_abortable(PHASE_SETTLE_MS);
    }
    else
    {
        /* No action: skip */
    }

    if (aborted == false)
    {
        aborted = capture(run, PHASE_MEASURE_MS);
        motor_pwm_stop_all();
    }
    else
    {
        /* No action: skip */
    }

    if (aborted == false)
    {
        print_stats("all", run);
        verdict(idle, run);
        (void)printf("[3/3] each motor alone at 40%%\n");
    }
    else
    {
        /* No action: skip */
    }

    for (m = 0U; (m < MOTOR_COUNT) && (aborted == false); m++)
    {
        motor_pwm_stop_all();
        aborted = app_wait_abortable(PHASE_SETTLE_MS);   /* spin down */
        aborted = aborted || ramp(m, TEST_DUTY);
        aborted = aborted || app_wait_abortable(PHASE_SETTLE_MS);

        if (aborted == false)
        {
            aborted = capture(run, PHASE_MEASURE_MS);
            motor_pwm_stop_all();
            print_stats(motor_name[m], run);
        }
        else
        {
            /* No action: stop below */
        }
    }

    motor_pwm_stop_all();
    print_end(aborted);
}

/*
 * เร่งทุกตัวจาก 0 ถึง 100 % ใน 10 วินาที พิมพ์ az ทุก 5 step
 * az เกิน ~1 g ขณะเร่ง = แรงยกมากกว่าน้ำหนัก (ต้องผูกเครื่องไว้)
 */
void test_thrust(void)
{
    uint32_t duty = 0U;
    uint32_t step = 0U;
    bool     aborted;
    float    g[MPU_AXES];
    float    az;

    (void)printf("THRUST RAMP - props on, TETHERED, hands clear. BOOT = stop\n");
    aborted = app_countdown();

    while ((duty <= MOTOR_DUTY_FULL) && (aborted == false))
    {
        motor_pwm_set_all(duty);

        if ((step % THRUST_PRINT_EVERY) == 0U)
        {
            if (mpu6500_read_gyro(g, &az) == true)
            {
                (void)printf("duty %3lu%%  az %5.2f g\n",
                             (unsigned long)(duty / DUTY_PER_PERCENT), (double)az);
            }
            else
            {
                (void)printf("duty %3lu%%  read error\n",
                             (unsigned long)(duty / DUTY_PER_PERCENT));
            }
        }
        else
        {
            /* No action: not a print step */
        }

        aborted = app_wait_abortable(THRUST_STEP_MS);
        duty   += THRUST_STEP_DUTY;
        step++;
    }

    if (aborted == false)
    {
        aborted = app_wait_abortable(THRUST_HOLD_MS);
    }
    else
    {
        /* No action: already stopping */
    }

    motor_pwm_stop_all();
    print_end(aborted);
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * ล้างสถิติของ 1 แกน
 */
static void stat_reset(stat_t *s)
{
    s->n    = 0U;
    s->mean = ZERO_F;
    s->m2   = ZERO_F;
    s->vmin = STAT_INIT_MIN;
    s->vmax = STAT_INIT_MAX;
}

/*
 * เพิ่ม sample แบบ Welford: หา mean/variance ได้ในรอบเดียวโดยไม่เก็บทุกค่า
 */
static void stat_add(stat_t *s, float x)
{
    float delta;

    s->n++;
    delta    = x - s->mean;
    s->mean += delta / (float)s->n;
    s->m2   += delta * (x - s->mean);

    if (x < s->vmin)
    {
        s->vmin = x;
    }
    else
    {
        /* No action */
    }
    if (x > s->vmax)
    {
        s->vmax = x;
    }
    else
    {
        /* No action */
    }
}

/*
 * standard deviation (0 ถ้ามีไม่ถึง 2 sample)
 */
static float stat_std(const stat_t *s)
{
    float out = ZERO_F;

    if (s->n > 1U)
    {
        out = sqrtf(s->m2 / (float)(s->n - 1U));
    }
    else
    {
        /* No action */
    }

    return out;
}

/*
 * ค่าที่อ่านพลาดเพราะ EMI จากมอเตอร์จะเป็นค่าเต็ม scale หรือกระโดดเกิน
 * กว่าที่เครื่องที่ผูกไว้จะหมุนได้ใน 1 ms - ทิ้ง sample แบบนั้น
 */
static bool sample_is_sane(const stat_t st[MPU_AXES], const float g[MPU_AXES])
{
    bool     ok = true;
    uint32_t a;
    float    v;
    float    dev;

    for (a = 0U; a < MPU_AXES; a++)
    {
        v   = g[a];
        dev = v - st[a].mean;

        if ((v >= SATURATION_DPS) || (v <= -SATURATION_DPS))
        {
            ok = false;
        }
        else if ((st[a].n >= MIN_SAMPLES_MEAN) &&
                 ((dev > MAX_DEV_DPS) || (dev < -MAX_DEV_DPS)))
        {
            ok = false;
        }
        else
        {
            /* No action: plausible */
        }
    }

    return ok;
}

/*
 * อ่าน gyro ที่ 1 kHz นาน duration_ms เก็บสถิติ คืน true ถ้ากด BOOT
 */
static bool capture(stat_t st[MPU_AXES], uint32_t duration_ms)
{
    TickType_t last;
    uint32_t   i;
    uint32_t   a;
    float      g[MPU_AXES];
    float      az;
    bool       aborted = false;

    for (a = 0U; a < MPU_AXES; a++)
    {
        stat_reset(&st[a]);
    }
    s_read_err = 0U;
    s_rejected = 0U;

    last = xTaskGetTickCount();
    for (i = 0U; (i < duration_ms) && (aborted == false); i++)
    {
        if (mpu6500_read_gyro(g, &az) == false)
        {
            s_read_err++;
        }
        else if (sample_is_sane(st, g) == false)
        {
            s_rejected++;
        }
        else
        {
            for (a = 0U; a < MPU_AXES; a++)
            {
                stat_add(&st[a], g[a]);
            }
        }
        aborted = button_pressed();
        vTaskDelayUntil(&last, ONE_TICK);
    }

    return aborted;
}

/*
 * พิมพ์ mean / std / peak-to-peak ของทั้ง 3 แกน
 */
static void print_stats(const char *name, const stat_t st[MPU_AXES])
{
    (void)printf("%-6s n=%4lu err=%lu bad=%lu | mean %6.2f %6.2f %6.2f | ",
                 name, (unsigned long)st[AXIS_X].n, (unsigned long)s_read_err,
                 (unsigned long)s_rejected, (double)st[AXIS_X].mean,
                 (double)st[AXIS_Y].mean, (double)st[AXIS_Z].mean);
    (void)printf("std %5.2f %5.2f %5.2f | p-p %5.2f %5.2f %5.2f\n",
                 (double)stat_std(&st[AXIS_X]), (double)stat_std(&st[AXIS_Y]),
                 (double)stat_std(&st[AXIS_Z]),
                 (double)(st[AXIS_X].vmax - st[AXIS_X].vmin),
                 (double)(st[AXIS_Y].vmax - st[AXIS_Y].vmin),
                 (double)(st[AXIS_Z].vmax - st[AXIS_Z].vmin));
}

/*
 * เร่งมอเตอร์ 1 ตัว (idx < MOTOR_COUNT) หรือทุกตัว (idx == MOTOR_COUNT)
 * ทีละ 1 % ทุก 20 ms
 */
static bool ramp(uint32_t idx, uint32_t target)
{
    uint32_t d;
    bool     aborted = false;

    for (d = 0U; (d <= target) && (aborted == false); d += RAMP_STEP_DUTY)
    {
        if (idx < MOTOR_COUNT)
        {
            motor_pwm_set_one(idx, d);
        }
        else
        {
            motor_pwm_set_all(d);
        }
        aborted = app_wait_abortable(RAMP_STEP_MS);
    }

    return aborted;
}

/*
 * เริ่มมอเตอร์ทีละตัว กระแสกระชากตอนสตาร์ทจะได้ไม่รวมกันจนวงจร
 * ป้องกันของแบตตัด เริ่มจาก M4 ก่อน: ถ้ายังตัดตอนตัวที่ 4 แปลว่าเป็นกระแส
 * รวม ไม่ใช่ channel ใด channel หนึ่งเสีย
 */
static bool ramp_staggered(uint32_t target)
{
    uint32_t m;
    bool     aborted = false;

    for (m = MOTOR_COUNT; (m > 0U) && (aborted == false); m--)
    {
        (void)printf("  start %s\n", motor_name[m - 1U]);
        aborted = ramp(m - 1U, target);
        aborted = aborted || app_wait_abortable(STAGGER_MS);
    }

    return aborted;
}

/*
 * ตัดสินผล: ใช้แกนที่แย่สุด x 3 sigma เทียบกับเกณฑ์ 3 dps
 */
static void verdict(const stat_t idle[MPU_AXES], const stat_t run[MPU_AXES])
{
    float    worst = ZERO_F;
    float    s;
    float    band;
    uint32_t a;

    for (a = 0U; a < MPU_AXES; a++)
    {
        s = stat_std(&run[a]);
        if (s > worst)
        {
            worst = s;
        }
        else
        {
            /* No action */
        }
    }
    band = worst * SIGMA_BAND;

    (void)printf("worst axis: +/-%.2f dps (3 std) | idle std %.2f %.2f %.2f\n",
                 (double)band, (double)stat_std(&idle[AXIS_X]),
                 (double)stat_std(&idle[AXIS_Y]), (double)stat_std(&idle[AXIS_Z]));

    if (worst < (SPIN_STD_RATIO * stat_std(&idle[AXIS_X])))
    {
        (void)printf("RESULT: INVALID - std same as idle, motors did not spin\n");
    }
    else if (band <= LIMIT_PASS_DPS)
    {
        (void)printf("RESULT: PASS - OK to start PID\n");
    }
    else if (band < LIMIT_FAIL_DPS)
    {
        (void)printf("RESULT: MARGINAL - more foam / lower DLPF / check props\n");
    }
    else
    {
        (void)printf("RESULT: FAIL - fix vibration BEFORE PID\n");
    }
}

/*
 * ข้อความตอนจบการทดสอบ
 */
static void print_end(bool aborted)
{
    if (aborted == true)
    {
        (void)printf("ABORTED - motors off\n\n");
    }
    else
    {
        (void)printf("done - press BOOT to repeat\n\n");
    }
}
