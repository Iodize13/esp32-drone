/*******************************************************************************
 * File Name    : motor_pwm.c
 * Description  : สร้าง PWM 20 kHz ให้ gate ของ MOSFET มอเตอร์ 4 ตัว ด้วย LEDC
 *                M1 GPIO4, M2 GPIO5, M3 GPIO6, M4 GPIO7
 *                ใช้ 20 kHz เพราะเกินช่วงที่หูได้ยิน และ MOSFET ยังไม่ร้อน
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "motor_pwm.h"
#include "driver/ledc.h"
#include "esp_err.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PWM_FREQ_HZ       (20000U)
#define PWM_RES           (LEDC_TIMER_11_BIT)
#define PWM_FULL_COUNTS   (2048U)        /* 2^11 = 100 % */
#define PWM_HPOINT        (0)
#define PIN_M1            (4)            /* front right */
#define PIN_M2            (5)            /* back right  */
#define PIN_M3            (6)            /* back left   */
#define PIN_M4            (7)            /* front left  */

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/
static const int32_t motor_gpio[MOTOR_COUNT] =
{
    PIN_M1, PIN_M2, PIN_M3, PIN_M4
};

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * ตั้ง timer LEDC 1 ตัว แล้วผูก channel 0-3 เข้ากับขามอเตอร์
 * ทุก channel ใช้ timer เดียวกัน จึงได้ความถี่ 20 kHz เท่ากันทุกตัว
 */
void motor_pwm_init(void)
{
    ledc_timer_config_t tcfg =
    {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    uint32_t m;

    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    for (m = 0U; m < MOTOR_COUNT; m++)
    {
        ledc_channel_config_t ccfg =
        {
            .gpio_num   = motor_gpio[m],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)m,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0U,
            .hpoint     = PWM_HPOINT,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    }
}

/*
 * duty (per-mille) ของมอเตอร์ 1 ตัว แปลงเป็นจำนวน count ของ timer 11 bit
 * ค่าที่เกิน 1000 ถูกตัดไว้ที่ 1000 กันไม่ให้ register ล้น
 */
void motor_pwm_set_one(uint32_t idx, uint32_t duty)
{
    uint32_t d = duty;
    uint32_t counts;

    if (d > MOTOR_DUTY_FULL)
    {
        d = MOTOR_DUTY_FULL;
    }
    else
    {
        /* No action */
    }
    counts = (d * PWM_FULL_COUNTS) / MOTOR_DUTY_FULL;

    if (idx < MOTOR_COUNT)
    {
        (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, counts);
        (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
    }
    else
    {
        /* No action: invalid motor */
    }
}

/*
 * duty เท่ากันทุกตัว
 */
void motor_pwm_set_all(uint32_t duty)
{
    uint32_t m;

    for (m = 0U; m < MOTOR_COUNT; m++)
    {
        motor_pwm_set_one(m, duty);
    }
}

/*
 * ดับมอเตอร์ทุกตัว - ทางออกของทุก failsafe ต้องไม่ block
 */
void motor_pwm_stop_all(void)
{
    motor_pwm_set_all(0U);
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/
