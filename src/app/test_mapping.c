/*******************************************************************************
 * File Name    : test_mapping.c
 * Description  : motor mapping test (ถอดใบพัด) หาเครื่องหมายของ mixer
 *                มอเตอร์แต่ละตัวหมุนสั้นๆ ให้ดูว่าเป็นตัวไหน แล้วผู้ใช้กดมุมของ
 *                มอเตอร์ตัวนั้นลง ~15-20 องศา เครื่องหมายของ roll/pitch ตอนมุมนั้น
 *                ต่ำบอกเครื่องหมายใน mixer: มอเตอร์ฝั่งต่ำของมุมบวกต้องเป็น -1
 *                มุมวัดเทียบกับท่าที่วางนิ่งตอนเริ่ม (IMU อาจติดเอียง)
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "test_mapping.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_util.h"
#include "attitude_est.h"
#include "button.h"
#include "motor_pwm.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MAP_DUTY          (250U)         /* 25 %, props off           */
#define MAP_SPIN_MS       (1500U)
#define MAP_TILT_DEG      (8.0f)         /* corner counts as "down"   */
#define MAP_SIGN_DEG      (MAP_TILT_DEG * 0.5f)
#define MAP_LEVEL_DEG     (3.0f)         /* back to level             */
#define MAP_HOLD_MS       (500U)         /* must hold tilt this long  */
#define MAP_LEVEL_HOLD_MS (1500U)        /* must stay level this long */
#define MAP_NEXT_PAUSE_MS (1000U)        /* gap before next motor     */
#define MAP_TIMEOUT_MS    (30000U)
#define MAP_PRINT_MS      (250U)
#define POSE_OK           (0U)
#define POSE_ABORTED      (1U)
#define POSE_TIMEOUT      (2U)
#define SIGN_POS          (1)
#define SIGN_NEG          (-1)
#define SIGN_NONE         (0)
#define ONE_TICK          (1U)
#define M1                (0U)
#define M2                (1U)
#define M3                (2U)
#define M4                (3U)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/
static const char *const motor_name[MOTOR_COUNT] =
{
    "M1 G4", "M2 G5", "M3 G6", "M4 G7"
};

/* Private variables ------------------------------------------------------------*/
static float s_ref_roll  = 0.0f;         /* resting pose; tilt is measured */
static float s_ref_pitch = 0.0f;         /* from here                      */

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static uint32_t run_result(uint32_t ms);
static uint32_t wait_pose(bool want_tilt);
static int8_t sign_when_low(float angle_deg);
static void print_row(const char *name, const int8_t v[MOTOR_COUNT]);
static void print_result(const int8_t roll_s[MOTOR_COUNT],
                         const int8_t pitch_s[MOTOR_COUNT]);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * วนทีละมอเตอร์: หมุน -> รอกดมุมลง -> อ่านเครื่องหมาย -> รอกลับระดับ
 * จบแล้วพิมพ์ตาราง mixer_config_t ให้ copy ไปใส่ test_pid.c
 */
void test_mapping(void)
{
    int8_t   roll_s[MOTOR_COUNT]  = { 0, 0, 0, 0 };
    int8_t   pitch_s[MOTOR_COUNT] = { 0, 0, 0, 0 };
    uint32_t m;
    uint32_t res = POSE_OK;
    bool     ok;

    (void)printf("MOTOR MAPPING - PROPS OFF. Keep the frame level and still...\n");
    ok = att_est_calibrate();
    (void)att_est_run(MAP_HOLD_MS);      /* let the angle settle from accel */
    s_ref_roll  = att_est_roll();
    s_ref_pitch = att_est_pitch();
    (void)printf("resting pose: roll %+.1f pitch %+.1f (tilt measured from here)\n",
                 (double)s_ref_roll, (double)s_ref_pitch);

    for (m = 0U; (m < MOTOR_COUNT) && (res == POSE_OK) && (ok == true); m++)
    {
        (void)printf("\n[%s] spinning - see WHICH motor it is (write its position down)\n",
                     motor_name[m]);
        motor_pwm_set_one(m, MAP_DUTY);
        res = run_result(MAP_SPIN_MS);
        motor_pwm_stop_all();

        if (res == POSE_OK)
        {
            (void)printf("[%s] now press THAT motor's corner DOWN ~15-20 deg and hold\n",
                         motor_name[m]);
            res = wait_pose(true);
        }
        else
        {
            /* No action: aborted */
        }

        if (res == POSE_OK)
        {
            roll_s[m]  = sign_when_low(att_est_roll() - s_ref_roll);
            pitch_s[m] = sign_when_low(att_est_pitch() - s_ref_pitch);
            (void)printf("[%s] corner down: d-roll %+.1f d-pitch %+.1f -> roll %+d pitch %+d\n",
                         motor_name[m], (double)(att_est_roll() - s_ref_roll),
                         (double)(att_est_pitch() - s_ref_pitch),
                         (int)roll_s[m], (int)pitch_s[m]);
            (void)printf("    release - back to level\n");
            res = wait_pose(false);
        }
        else
        {
            /* No action: aborted or timed out */
        }

        if ((res == POSE_OK) && ((m + 1U) < MOTOR_COUNT))
        {
            (void)printf("    level - next motor in %lu ms\n",
                         (unsigned long)MAP_NEXT_PAUSE_MS);
            res = run_result(MAP_NEXT_PAUSE_MS);
        }
        else
        {
            /* No action: aborted or timed out */
        }
    }

    motor_pwm_stop_all();

    if (res == POSE_ABORTED)
    {
        (void)printf("ABORTED - mapping not finished\n\n");
    }
    else if (res == POSE_TIMEOUT)
    {
        (void)printf("TIMEOUT - mapping not finished\n\n");
    }
    else
    {
        print_result(roll_s, pitch_s);
    }
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * att_est_run แล้วแปลงผลเป็นรหัส POSE_*
 */
static uint32_t run_result(uint32_t ms)
{
    uint32_t res = POSE_OK;

    if (att_est_run(ms) == true)
    {
        res = POSE_ABORTED;
    }
    else
    {
        /* No action */
    }

    return res;
}

/*
 * รอจนเอียงค้าง (want_tilt, 500 ms) หรือกลับระดับค้าง (1.5 s)
 * พิมพ์มุมระหว่างรอ คืน POSE_OK / POSE_ABORTED / POSE_TIMEOUT
 */
static uint32_t wait_pose(bool want_tilt)
{
    TickType_t last;
    uint32_t   i;
    uint32_t   held   = 0U;
    uint32_t   result = POSE_TIMEOUT;
    uint32_t   hold_ms;
    float      rate[MPU_AXES];
    float      dr;
    float      dp;
    float      r;
    float      p;
    bool       match;

    if (want_tilt == true)
    {
        hold_ms = MAP_HOLD_MS;
    }
    else
    {
        hold_ms = MAP_LEVEL_HOLD_MS;
    }

    last = xTaskGetTickCount();
    for (i = 0U; (i < MAP_TIMEOUT_MS) && (result == POSE_TIMEOUT); i++)
    {
        (void)att_est_update(ATT_EST_DT_S, rate);
        dr = att_est_roll() - s_ref_roll;
        dp = att_est_pitch() - s_ref_pitch;
        r  = app_absf(dr);
        p  = app_absf(dp);

        if (want_tilt == true)
        {
            match = (r >= MAP_TILT_DEG) || (p >= MAP_TILT_DEG);
        }
        else
        {
            match = (r < MAP_LEVEL_DEG) && (p < MAP_LEVEL_DEG);
        }

        if (match == true)
        {
            held = held + 1U;
        }
        else
        {
            held = 0U;
        }

        if (held >= hold_ms)
        {
            result = POSE_OK;
        }
        else if (button_pressed() == true)
        {
            result = POSE_ABORTED;
        }
        else
        {
            /* No action: keep waiting */
        }

        if ((i % MAP_PRINT_MS) == 0U)
        {
            (void)printf("    d-roll %+6.1f  d-pitch %+6.1f\r", (double)dr, (double)dp);
            (void)fflush(stdout);
        }
        else
        {
            /* No action: not a print step */
        }
        vTaskDelayUntil(&last, ONE_TICK);
    }
    (void)printf("\n");

    return result;
}

/*
 * +1 / -1 / 0 จากมุมตอนที่มุมของมอเตอร์ตัวนี้ถูกกดลง
 * มอเตอร์อยู่ฝั่งต่ำของมุมบวก -> -1, ไม่เอียงชัดเจน -> 0 (อยู่บนแกน)
 */
static int8_t sign_when_low(float angle_deg)
{
    int8_t s;

    if (angle_deg >= MAP_SIGN_DEG)
    {
        s = SIGN_NEG;
    }
    else if (angle_deg <= -MAP_SIGN_DEG)
    {
        s = SIGN_POS;
    }
    else
    {
        s = SIGN_NONE;
    }

    return s;
}

/*
 * พิมพ์ 1 แถวของ mixer_config_t ในรูปแบบที่ copy ไปใส่โค้ดได้เลย
 */
static void print_row(const char *name, const int8_t v[MOTOR_COUNT])
{
    (void)printf("    .%-5s = { %+d, %+d, %+d, %+d },\n", name,
                 (int)v[M1], (int)v[M2], (int)v[M3], (int)v[M4]);
}

/*
 * พิมพ์ตาราง roll/pitch/yaw และเช็คว่าเครื่องหมายสมดุล (X layout ต้องมี
 * +1 สองตัว -1 สองตัว) yaw: มอเตอร์แนวทแยงหมุนทางเดียวกัน คือคู่ที่
 * roll * pitch เท่ากัน
 */
static void print_result(const int8_t roll_s[MOTOR_COUNT],
                         const int8_t pitch_s[MOTOR_COUNT])
{
    int8_t   yaw_s[MOTOR_COUNT];
    int32_t  sum_r = 0;
    int32_t  sum_p = 0;
    uint32_t m;

    for (m = 0U; m < MOTOR_COUNT; m++)
    {
        yaw_s[m] = (int8_t)(roll_s[m] * pitch_s[m]);
        sum_r   += roll_s[m];
        sum_p   += pitch_s[m];
    }

    (void)printf("\nmixer_config_t signs (paste into the flight code):\n");
    print_row("roll", roll_s);
    print_row("pitch", pitch_s);
    print_row("yaw", yaw_s);

    if ((sum_r != 0) || (sum_p != 0))
    {
        (void)printf("WARNING: roll or pitch signs do not balance (sum %ld / %ld).\n"
                     "Two motors probably got the same corner - redo the test.\n",
                     (long)sum_r, (long)sum_p);
    }
    else
    {
        (void)printf("roll/pitch signs balance - looks like a valid X layout.\n");
    }
    (void)printf("yaw: only the diagonal pairing is known. If the drone spins up\n"
                 "faster when yaw control is on, negate the whole yaw row.\n\n");
}
