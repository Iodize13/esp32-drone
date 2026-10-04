/**
 * Bring-up test: ESP32-S3 DevKit + MPU6500 over I2C.
 * COM port (UART0) -> I2C scan -> WHO_AM_I -> gyro stream at 10 Hz.
 *
 * WIRING
 *   VCC -> 3V3        GND -> GND
 *   SDA -> GPIO8       SCL -> GPIO9
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define PIN_SDA           (8)
#define PIN_SCL           (9)
#define I2C_HZ            (400000U)
#define I2C_TIMEOUT_MS    (100)

#define MPU_ADDR          (0x68U)
#define REG_WHO_AM_I      (0x75U)
#define REG_PWR_MGMT_1    (0x6BU)
#define REG_CONFIG        (0x1AU)
#define REG_GYRO_XOUT_H   (0x43U)
#define WHO_MPU6500       (0x70U)
#define WHO_MPU6050       (0x68U)

#define GYRO_LSB_PER_DPS  (131.0f)
#define ADDR_FIRST        (0x08U)
#define ADDR_LAST         (0x77U)
#define STREAM_PERIOD_MS  (100U)
#define RETRY_MS          (2000U)

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_mpu = NULL;

static uint32_t i2c_scan(void)
{
    uint32_t found = 0U;
    uint16_t a;

    printf("I2C scan:");
    for (a = ADDR_FIRST; a <= ADDR_LAST; a++)
    {
        if (i2c_master_probe(s_bus, a, I2C_TIMEOUT_MS) == ESP_OK)
        {
            printf(" 0x%02X", (unsigned int)a);
            found++;
        }
        else
        {
            /* nobody home */
        }
    }
    printf((found == 0U) ? " none - check SDA/SCL, GND, 3V3\n" : "\n");

    return found;
}

/* Idle I2C lines must read HIGH. LOW = stuck slave, unpowered module
 * (its ESD diodes clamp the line), short to GND, or wrong pin. */
static void i2c_line_check(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_SDA) | (1ULL << PIN_SCL),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    (void)gpio_config(&io);
    vTaskDelay(pdMS_TO_TICKS(10U));
    printf("idle lines: SDA(GPIO%d)=%d  SCL(GPIO%d)=%d  (both must be 1)\n",
           PIN_SDA, gpio_get_level(PIN_SDA), PIN_SCL, gpio_get_level(PIN_SCL));
}

static bool mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    return (i2c_master_transmit(s_mpu, buf, sizeof(buf), I2C_TIMEOUT_MS) == ESP_OK);
}

static bool mpu_read(uint8_t reg, uint8_t *dst, size_t len)
{
    return (i2c_master_transmit_receive(s_mpu, &reg, 1U, dst, len, I2C_TIMEOUT_MS) == ESP_OK);
}

void app_main(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_SDA,
        .scl_io_num        = PIN_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU_ADDR,
        .scl_speed_hz    = I2C_HZ,
    };
    uint8_t who = 0U;
    uint8_t b[6];

    vTaskDelay(pdMS_TO_TICKS(RETRY_MS));   /* time to open the monitor */
    printf("\n\nESP32-S3 DevKit bring-up v2\n");

    i2c_line_check();
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_mpu));

    while (i2c_scan() == 0U)
    {
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }

    if (mpu_read(REG_WHO_AM_I, &who, 1U) == true)
    {
        printf("WHO_AM_I = 0x%02X (0x70=6500, 0x68=6050)\n", (unsigned int)who);
    }
    else
    {
        printf("WHO_AM_I read failed\n");
    }

    if ((who == WHO_MPU6500) || (who == WHO_MPU6050))
    {
        (void)mpu_write(REG_PWR_MGMT_1, 0x01U);   /* wake, PLL clock */
        (void)mpu_write(REG_CONFIG, 0x03U);       /* DLPF ~41 Hz     */
        vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));

        for (;;)
        {
            if (mpu_read(REG_GYRO_XOUT_H, b, sizeof(b)) == true)
            {
                int16_t gx = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
                int16_t gy = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
                int16_t gz = (int16_t)(((uint16_t)b[4] << 8) | b[5]);

                printf("gyro dps %7.2f %7.2f %7.2f\n",
                       (double)((float)gx / GYRO_LSB_PER_DPS),
                       (double)((float)gy / GYRO_LSB_PER_DPS),
                       (double)((float)gz / GYRO_LSB_PER_DPS));
            }
            else
            {
                printf("read error\n");
            }
            vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));
        }
    }
    else
    {
        printf("unexpected ID - halting\n");
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
        }
    }
}
