/**
 * imu_int.c - MPU6500 data-ready interrupt (EXTI) on GPIO10.
 *
 * The MPU6500 drives INT high for ~50 us each time a new sample is in
 * its registers. A rising-edge GPIO interrupt gives a binary semaphore;
 * imu_int_wait() blocks on it, so no CPU time is spent polling.
 */
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "imu_int.h"

#define IMU_INT_GPIO      (GPIO_NUM_10)

static SemaphoreHandle_t s_ready = NULL;
static StaticSemaphore_t s_ready_buf;
static volatile uint32_t s_count = 0U;

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
        /* no higher-priority task woken */
    }
}

bool imu_int_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << (uint32_t)IMU_INT_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* idle low if unwired */
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    esp_err_t err;

    s_ready = xSemaphoreCreateBinaryStatic(&s_ready_buf);
    err = gpio_config(&io);
    if (err == ESP_OK)
    {
        err = gpio_install_isr_service(0);
        /* already installed by another driver is fine */
        err = (err == ESP_ERR_INVALID_STATE) ? ESP_OK : err;
    }
    else
    {
        /* gpio_config failed */
    }
    if (err == ESP_OK)
    {
        err = gpio_isr_handler_add(IMU_INT_GPIO, imu_int_isr, NULL);
    }
    else
    {
        /* ISR service failed */
    }

    return ((err == ESP_OK) && (s_ready != NULL));
}

bool imu_int_wait(uint32_t timeout_ms)
{
    bool got = false;

    if (s_ready != NULL)
    {
        got = (xSemaphoreTake(s_ready, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
    }
    else
    {
        /* not initialised */
    }
    return got;
}

uint32_t imu_int_count(void)
{
    return s_count;
}
