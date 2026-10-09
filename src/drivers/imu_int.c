/*******************************************************************************
 * File Name    : imu_int.c
 * Description  : EXTI ของ data-ready จาก MPU6500 ที่ GPIO10 (rising edge)
 *                IMU ยกขา INT เป็น high ทุกครั้งที่มี sample ใหม่ (1 kHz)
 *                ISR give semaphore แล้ว PID loop รอ semaphore นี้ แทนการ
 *                หน่วงเวลาด้วย timer loop จึงเดินตรงจังหวะ sample ของ IMU
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "imu_int.h"
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_attr.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define IMU_INT_GPIO      (GPIO_NUM_10)
#define ISR_FLAGS_DEFAULT (0)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/
static SemaphoreHandle_t s_ready = NULL;
static StaticSemaphore_t s_ready_buf;
static volatile uint32_t s_count = 0U;

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static void imu_int_isr(void *arg);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * ตั้ง GPIO10 เป็น input + interrupt ขอบขาขึ้น แล้วผูก ISR
 * เปิด pull-down ไว้ ถ้าสาย INT หลุด ขาจะไม่ลอยจนเกิด interrupt มั่ว
 */
bool imu_int_init(void)
{
    gpio_config_t io =
    {
        .pin_bit_mask = (1ULL << (uint32_t)IMU_INT_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    esp_err_t err;

    s_ready = xSemaphoreCreateBinaryStatic(&s_ready_buf);
    err = gpio_config(&io);
    if (err == ESP_OK)
    {
        err = gpio_install_isr_service(ISR_FLAGS_DEFAULT);
        if (err == ESP_ERR_INVALID_STATE)
        {
            err = ESP_OK;                /* already installed by another driver */
        }
        else
        {
            /* No action */
        }
    }
    else
    {
        /* No action: gpio_config failed */
    }
    if (err == ESP_OK)
    {
        err = gpio_isr_handler_add(IMU_INT_GPIO, imu_int_isr, NULL);
    }
    else
    {
        /* No action: ISR service failed */
    }

    return ((err == ESP_OK) && (s_ready != NULL));
}

/*
 * block จนกว่าจะมี data-ready หรือหมดเวลา
 * true = มี sample ใหม่, false = timeout (สาย INT หลุดหรือ IMU หยุด)
 */
bool imu_int_wait(uint32_t timeout_ms)
{
    bool got = false;

    if (s_ready != NULL)
    {
        got = (xSemaphoreTake(s_ready, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
    }
    else
    {
        /* No action: not initialised */
    }

    return got;
}

/*
 * จำนวน interrupt ทั้งหมดตั้งแต่ boot ใช้เช็คว่าสาย INT ทำงาน (~1000/s)
 */
uint32_t imu_int_count(void)
{
    return s_count;
}

/* Callback functions ------------------------------------------------------------*/

/*
 * ISR ของ GPIO10: นับจำนวน แล้ว give semaphore ปลุก PID loop
 * ถ้าปลุก task ที่ priority สูงกว่า ให้สลับ task ทันทีตอนออกจาก ISR
 */
static void IRAM_ATTR imu_int_isr(void *arg)
{
    BaseType_t woken = pdFALSE;

    (void)arg;
    s_count++;
    (void)xSemaphoreGiveFromISR(s_ready, &woken);
    if (woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
    else
    {
        /* No action: no higher-priority task woken */
    }
}

/* Private functions ------------------------------------------------------------*/
