/*******************************************************************************
 * File Name    : main.c
 * Description  : โปรแกรมหลักของ flight controller โดรน (ESP32-S3 + MPU6500)
 *                init driver ทุกตัว แล้วรอกดปุ่ม BOOT เพื่อเริ่มการทดสอบที่เลือกไว้
 *                ใน TEST_MODE ระหว่างรอรับคำสั่ง serial (a / b / i) ได้
 *                WIRING: MPU6500 SDA GPIO8, SCL GPIO9, INT GPIO10
 *                        มอเตอร์ M1 GPIO4 (หน้าขวา), M2 GPIO5 (หลังขวา),
 *                        M3 GPIO6 (หลังซ้าย), M4 GPIO7 (หน้าซ้าย)
 *                        แบต: divider 100k/100k เข้า GPIO1
 *                Serial: 115200
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adc_batt.h"
#include "button.h"
#include "imu_int.h"
#include "motor_pwm.h"
#include "mpu6500.h"
#include "serial_keys.h"
#include "console.h"
#include "test_bench.h"
#include "test_mapping.h"
#include "test_pid.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TEST_VIBRATION    (0U)
#define TEST_THRUST       (1U)
#define TEST_MAPPING      (2U)
#define TEST_PID          (3U)
#define TEST_MODE         (TEST_PID)

#define BOOT_SETTLE_MS    (2000U)        /* time to open the serial monitor */
#define HALT_DELAY_MS     (1000U)
#define DEBOUNCE_MS       (30U)
#define POLL_MS           (10U)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static void drivers_init(void);
static void report(bool ok, const char *good, const char *bad);
static void halt(void);
static bool boot_pressed(void);
static void run_test(void);

/* Private user code ------------------------------------------------------------*/

/* main() ------------------------------------------------------------*/
void app_main(void)
{
    drivers_init();

    for (;;)
    {
        console_idle_poll();
        if (boot_pressed() == true)
        {
            run_test();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        }
    }
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * ดับมอเตอร์ก่อนอย่างอื่น (gate ลอยตอนบูต MOSFET อาจเปิด) แล้ว init
 * peripheral ที่เหลือ IMU ใช้ไม่ได้ = ทดสอบอะไรไม่ได้ จึงหยุดตรงนั้น
 */
static void drivers_init(void)
{
    motor_pwm_init();
    motor_pwm_stop_all();
    button_init();

    vTaskDelay(pdMS_TO_TICKS(BOOT_SETTLE_MS));
    if (TEST_MODE == TEST_THRUST)
    {
        (void)printf("\n\nTHRUST test");
    }
    else if (TEST_MODE == TEST_MAPPING)
    {
        (void)printf("\n\nMAPPING test");
    }
    else if (TEST_MODE == TEST_PID)
    {
        (void)printf("\n\nPID test");
    }
    else
    {
        (void)printf("\n\nVIBRATION test");
    }
    (void)printf(" - PROPS OFF first, drone tied down\n");

    if (mpu6500_init() == false)
    {
        (void)printf("MPU init failed - halting\n");
        halt();
    }
    else
    {
        (void)printf("ready - press BOOT to start\n");
    }

    report(serial_keys_init(), "", "UART0 RX driver failed - keys will not work\n");
    report(imu_int_init(), "IMU INT (EXTI) on GPIO10\n", "IMU INT (EXTI) init failed\n");
    report(adc_batt_init(), "battery ADC on GPIO1 - press 'b' to read\n",
           "battery ADC init failed\n");
}

/*
 * พิมพ์ผลการ init ของ peripheral แต่ละตัว
 */
static void report(bool ok, const char *good, const char *bad)
{
    if (ok == true)
    {
        (void)printf("%s", good);
    }
    else
    {
        (void)printf("%s", bad);
    }
}

/*
 * หยุดถาวร (task ยังต้องหลับเป็นช่วงๆ ไม่งั้น watchdog จะ reset)
 */
static void halt(void)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(HALT_DELAY_MS));
    }
}

/*
 * true เมื่อกด BOOT ค้างเกิน 30 ms (กันสัญญาณเด้งของหน้าสัมผัส)
 */
static bool boot_pressed(void)
{
    bool pressed = false;

    if (button_pressed() == true)
    {
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
        pressed = button_pressed();
    }
    else
    {
        /* No action */
    }

    return pressed;
}

/*
 * รอปล่อยปุ่ม แล้วเรียกการทดสอบตาม TEST_MODE
 * รอปล่อยอีกรอบตอนจบ ไม่งั้นการกดเพื่อหยุดจะเริ่มรอบใหม่ทันที
 */
static void run_test(void)
{
    button_wait_release();
    if (TEST_MODE == TEST_THRUST)
    {
        test_thrust();
    }
    else if (TEST_MODE == TEST_MAPPING)
    {
        test_mapping();
    }
    else if (TEST_MODE == TEST_PID)
    {
        test_pid();
    }
    else
    {
        test_vibration();
    }
    button_wait_release();
}
