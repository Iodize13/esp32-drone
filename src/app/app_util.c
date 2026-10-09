/*******************************************************************************
 * File Name    : app_util.c
 * Description  : ฟังก์ชันช่วยที่ทุกการทดสอบใช้ร่วมกัน ทุกการรอต้องเช็คปุ่ม
 *                BOOT ไปด้วย เพื่อให้ผู้ใช้ดับมอเตอร์ได้ตลอดเวลา
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "app_util.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "button.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COUNTDOWN_S       (3U)
#define ONE_TICK          (1U)
#define ZERO_F            (0.0f)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * รอ ms มิลลิวินาที พร้อมดูปุ่ม BOOT ทุก 1 ms
 * คืน true ถ้าผู้ใช้กดปุ่ม (caller ต้องดับมอเตอร์)
 */
bool app_wait_abortable(uint32_t ms)
{
    int64_t end;
    bool    aborted = false;

    end = esp_timer_get_time() + ((int64_t)ms * US_PER_MS);
    while ((esp_timer_get_time() < end) && (aborted == false))
    {
        aborted = button_pressed();
        vTaskDelay(ONE_TICK);
    }

    return aborted;
}

/*
 * นับถอยหลัง 3-2-1 ก่อนมอเตอร์หมุน ให้ผู้ใช้เอามือออก
 * คืน true ถ้ากด BOOT ยกเลิกระหว่างนับ
 */
bool app_countdown(void)
{
    uint32_t s;
    bool     aborted = false;

    for (s = COUNTDOWN_S; (s > 0U) && (aborted == false); s--)
    {
        (void)printf("starting in %lu...\n", (unsigned long)s);
        aborted = app_wait_abortable(MS_PER_S);
    }

    return aborted;
}

/*
 * |x| ของ float (เขียนเองแทน fabsf เพื่อเลี่ยง ternary)
 */
float app_absf(float x)
{
    float out = x;

    if (x < ZERO_F)
    {
        out = -x;
    }
    else
    {
        /* No action */
    }

    return out;
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/
