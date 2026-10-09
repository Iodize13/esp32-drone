/*******************************************************************************
 * File Name    : console.c
 * Description  : คำสั่ง serial ตอนรอกด BOOT ใช้เช็คฮาร์ดแวร์โดยมอเตอร์ยังดับอยู่
 *                a = มุม roll/pitch จาก accel (หา trim ตอนจับเครื่องได้ระดับ)
 *                b = แรงดันแบต (เทียบกับมิเตอร์)
 *                i = นับ data-ready interrupt ใน 100 ms (ควรได้ ~100)
 *                ปุ่มอื่นจะตอบกลับว่าได้รับ เพื่อเช็คว่าสาย serial ใช้ได้
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "console.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "battery.h"
#include "imu_int.h"
#include "mpu6500.h"
#include "serial_keys.h"
#include "test_pid.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LEVEL_SAMPLES     (200U)
#define RAD_TO_DEG        (57.29578f)
#define INT_CHECK_MS      (100U)
#define ONE_TICK          (1U)
#define AXIS_X            (0U)
#define AXIS_Y            (1U)
#define AXIS_Z            (2U)
#define CHAR_FIRST        ((uint8_t)' ')  /* printable ASCII range */
#define CHAR_LAST         ((uint8_t)'~')

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static void level_print(void);
static void int_print(void);
static void echo_key(uint8_t c);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * เรียกทุกรอบของ loop รอ BOOT: อ่านปุ่ม 1 ตัว (ถ้ามี) แล้วทำตาม
 */
void console_idle_poll(void)
{
    uint8_t c = 0U;

    if (serial_keys_get(&c) == true)
    {
        switch (c)
        {
            case 'a':
                level_print();
                break;
            case 'b':
                battery_print();
                break;
            case 'i':
                int_print();
                break;
            default:
                echo_key(c);
                break;
        }
    }
    else
    {
        /* No action: no key */
    }
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * เฉลี่ย accel 200 ค่าแล้วคำนวณ roll/pitch (ไม่ใช้ gyro) พร้อมค่าหลัง trim
 * จับเครื่องให้ได้ระดับจริงแล้วกด 'a' จะได้ค่า trim ที่ต้องใส่
 */
static void level_print(void)
{
    float    acc[MPU_AXES];
    float    raw[MPU_AXES];
    float    sum[MPU_AXES] = { 0.0f, 0.0f, 0.0f };
    float    roll;
    float    pitch;
    uint32_t n = 0U;
    uint32_t i;
    uint32_t k;

    for (i = 0U; i < LEVEL_SAMPLES; i++)
    {
        if (mpu6500_read_all(acc, raw) == true)
        {
            for (k = 0U; k < MPU_AXES; k++)
            {
                sum[k] += acc[k];
            }
            n++;
        }
        else
        {
            /* No action: skip a bad read */
        }
        vTaskDelay(ONE_TICK);
    }

    if (n > 0U)
    {
        roll  = atan2f(sum[AXIS_Y], sum[AXIS_Z]) * RAD_TO_DEG;
        pitch = atan2f(-sum[AXIS_X], sqrtf((sum[AXIS_Y] * sum[AXIS_Y]) +
                                           (sum[AXIS_Z] * sum[AXIS_Z]))) * RAD_TO_DEG;
        (void)printf("level: roll %+.1f pitch %+.1f (raw)  ->  roll %+.1f pitch %+.1f"
                     " (after trim)\n", (double)roll, (double)pitch,
                     (double)(roll - PID_TRIM_ROLL_DEG),
                     (double)(pitch - PID_TRIM_PITCH_DEG));
    }
    else
    {
        (void)printf("level: IMU read failed\n");
    }
}

/*
 * นับ data-ready interrupt ใน 100 ms ระหว่างนั้นอ่าน INT_STATUS ทุก 1 ms
 * เพราะขา INT ตั้งเป็น latch: ไม่อ่านขาจะค้าง high และไม่เกิด edge ใหม่
 */
static void int_print(void)
{
    uint32_t n0;
    uint32_t k;

    n0 = imu_int_count();
    for (k = 0U; k < INT_CHECK_MS; k++)
    {
        (void)mpu6500_clear_int();
        vTaskDelay(ONE_TICK);
    }
    (void)printf("IMU INT: %lu edges in %u ms (expect ~%u)\n",
                 (unsigned long)(imu_int_count() - n0), (unsigned int)INT_CHECK_MS,
                 (unsigned int)INT_CHECK_MS);
}

/*
 * ตอบกลับทุกปุ่มที่ไม่ใช่คำสั่ง ตัวที่พิมพ์ไม่ได้แสดงเป็น '?'
 */
static void echo_key(uint8_t c)
{
    char shown = '?';

    if ((c >= CHAR_FIRST) && (c <= CHAR_LAST))
    {
        shown = (char)c;
    }
    else
    {
        /* No action */
    }
    (void)printf("key '%c' received - motors off, press BOOT to start\n", shown);
}
