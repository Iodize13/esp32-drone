/*******************************************************************************
 * File Name    : button.c
 * Description  : อ่านปุ่ม BOOT (GPIO0) ใช้เริ่มการทดสอบ และกดอีกครั้งเพื่อ
 *                ดับมอเตอร์ ปุ่มเป็น active-low จึงเปิด pull-up ภายใน
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "button.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PIN_BUTTON        (GPIO_NUM_0)   /* BOOT */
#define BUTTON_LEVEL_DOWN (0)            /* active-low: กด = 0 */
#define POLL_MS           (10U)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * ตั้ง GPIO0 เป็น input + pull-up เพราะบนบอร์ดปุ่มต่อลง GND ไม่มี
 * resistor ภายนอก ถ้าไม่เปิด pull-up ขาจะลอยตอนไม่กด
 */
void button_init(void)
{
    gpio_config_t io =
    {
        .pin_bit_mask = (1ULL << (uint32_t)PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    (void)gpio_config(&io);
}

/*
 * true ตราบที่ปุ่มยังถูกกดอยู่ (เช็คสถานะปัจจุบัน ไม่จำสถานะเก่า)
 */
bool button_pressed(void)
{
    return (gpio_get_level(PIN_BUTTON) == BUTTON_LEVEL_DOWN);
}

/*
 * รอจนปล่อยปุ่ม กันไม่ให้การกดครั้งเดียวทั้งเริ่มและหยุดการทดสอบ
 */
void button_wait_release(void)
{
    while (button_pressed() == true)
    {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/
