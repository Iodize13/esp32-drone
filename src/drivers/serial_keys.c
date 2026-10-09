/*******************************************************************************
 * File Name    : serial_keys.c
 * Description  : รับปุ่มจากคอมพิวเตอร์ทาง UART0 (CH343, 115200)
 *                ใช้ UART driver ของ ESP-IDF ซึ่งรับข้อมูลด้วย RX interrupt
 *                ลง ring buffer ไม่มีการวนเช็ค register ของ UART
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "serial_keys.h"
#include <stddef.h>
#include "driver/uart.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define KEYS_UART         (UART_NUM_0)
#define UART_RX_BUF       (256)          /* > 128 byte hardware FIFO */
#define UART_TX_BUF       (0)            /* TX stays on the console path */
#define UART_QUEUE_LEN    (0)
#define UART_INTR_FLAGS   (0)
#define KEY_LEN           (1U)
#define NO_WAIT           (0U)
#define ONE_BYTE          (1)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * ติดตั้ง UART driver ที่ boot เลย จะได้เช็คปุ่มได้ตั้งแต่ก่อนมอเตอร์หมุน
 */
bool serial_keys_init(void)
{
    esp_err_t err = ESP_OK;

    if (uart_is_driver_installed(KEYS_UART) == false)
    {
        err = uart_driver_install(KEYS_UART, UART_RX_BUF, UART_TX_BUF,
                                  UART_QUEUE_LEN, NULL, UART_INTR_FLAGS);
    }
    else
    {
        /* No action: already installed */
    }

    return (err == ESP_OK);
}

/*
 * หยิบปุ่ม 1 ตัวจาก ring buffer โดยไม่รอ (timeout 0)
 * control loop 1 kHz จึงไม่ถูกหน่วงตอนไม่มีใครกดปุ่ม
 */
bool serial_keys_get(uint8_t *key)
{
    return (uart_read_bytes(KEYS_UART, key, KEY_LEN, NO_WAIT) == ONE_BYTE);
}

/*
 * ทิ้งปุ่มที่ค้างใน buffer (เช่นกดไว้ก่อนเริ่มการทดสอบ)
 */
void serial_keys_flush(void)
{
    (void)uart_flush_input(KEYS_UART);
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/
