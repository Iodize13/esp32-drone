/*******************************************************************************
 * File Name    : mpu6500.c
 * Description  : driver IMU MPU6500 ผ่าน I2C 400 kHz (address 0x68)
 *                - ตั้ง sample rate 1 kHz, gyro +/-250 dps, accel +/-8 g
 *                - เปิด data-ready interrupt ที่ขา INT (ไปเข้า EXTI GPIO10)
 *                - อ่าน 14 byte รวด (accel 6, temp 2, gyro 6) แล้วแปลงหน่วย
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mpu6500.h"
#include <stddef.h>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PIN_SDA           (8)
#define PIN_SCL           (9)
#define I2C_HZ            (400000U)
#define I2C_TIMEOUT_MS    (100)
#define I2C_GLITCH_CNT    (7U)
#define I2C_INIT_RETRIES  (3U)
#define I2C_RETRY_MS      (50U)
#define I2C_ADDR_FIRST    (0x08U)        /* bus scan range */
#define I2C_ADDR_END      (0x78U)
#define CONFIG_SETTLE_MS  (100U)

#define MPU_ADDR          (0x68U)
#define REG_SMPLRT_DIV    (0x19U)
#define REG_CONFIG        (0x1AU)
#define REG_GYRO_CONFIG   (0x1BU)
#define REG_ACCEL_CONFIG  (0x1CU)
#define REG_ACCEL_CONFIG2 (0x1DU)        /* MPU6500 only: accel DLPF */
#define REG_INT_PIN_CFG   (0x37U)
#define REG_INT_ENABLE    (0x38U)
#define REG_INT_STATUS    (0x3AU)
#define REG_ACCEL_XOUT_H  (0x3BU)
#define REG_PWR_MGMT_1    (0x6BU)
#define REG_WHO_AM_I      (0x75U)
#define WHO_MPU6500       (0x70U)
#define WHO_MPU6050       (0x68U)

#define PWR_WAKE_PLL      (0x01U)
#define DLPF_CFG          (0x04U)        /* gyro DLPF 20 Hz */
#define SMPLRT_DIV_1KHZ   (0x00U)
#define GYRO_FS_250DPS    (0x00U)
#define ACCEL_FS_8G       (0x10U)
#define ACCEL_DLPF_10HZ   (0x05U)
#define INT_PIN_LATCH_HI  (0x30U)        /* active high, latched until any read */
#define INT_RAW_RDY_EN    (0x01U)        /* INT on every new sample */

#define ACCEL_LSB_PER_G   (4096.0f)      /* +/-8 g */
#define GYRO_LSB_PER_DPS  (131.0f)       /* +/-250 dps */

#define BURST_LEN         (14U)          /* accel 6 + temp 2 + gyro 6 */
#define OFS_AX            (0U)           /* byte offsets in the burst */
#define OFS_AY            (2U)
#define OFS_AZ            (4U)
#define OFS_GX            (8U)
#define OFS_GY            (10U)
#define OFS_GZ            (12U)
#define BYTE_SHIFT        (8U)
#define AXIS_X            (0U)
#define AXIS_Y            (1U)
#define AXIS_Z            (2U)
#define WRITE_LEN         (2U)           /* register + value */
#define REG_LEN           (1U)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_mpu = NULL;
static esp_err_t s_last_err = ESP_OK;

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/
static bool mpu_write(uint8_t reg, uint8_t val);
static bool mpu_read(uint8_t reg, uint8_t *dst, size_t len);
static bool mpu_configure(uint8_t who);
static void bus_scan_print(void);
static int16_t be16(const uint8_t *p);

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * สร้าง I2C bus แล้วอ่าน WHO_AM_I (ลองซ้ำ 3 ครั้ง เพราะบางครั้ง IMU ยัง
 * ไม่พร้อมตอนบอร์ดเพิ่งเปิด) ถ้าไม่มีใครตอบจะ scan bus แล้วพิมพ์ address
 * ที่เจอ ช่วยแยกว่าสายหลุดหรือ address ผิด
 */
bool mpu6500_init(void)
{
    i2c_master_bus_config_t bus_cfg =
    {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_SDA,
        .scl_io_num        = PIN_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_GLITCH_CNT,
        .flags.enable_internal_pullup = true,
    };
    i2c_device_config_t dev_cfg =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU_ADDR,
        .scl_speed_hz    = I2C_HZ,
    };
    uint8_t  who  = 0U;
    bool     ok   = false;
    bool     read = false;
    uint32_t tries;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_mpu));

    for (tries = 0U; (tries < I2C_INIT_RETRIES) && (read == false); tries++)
    {
        read = mpu_read(REG_WHO_AM_I, &who, REG_LEN);
        if (read == false)
        {
            (void)i2c_master_bus_reset(s_bus);
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_MS));
        }
        else
        {
            /* No action: got it */
        }
    }

    if (read == false)
    {
        (void)printf("I2C read failed (%s) - scanning bus:", esp_err_to_name(s_last_err));
        bus_scan_print();
    }
    else if ((who != WHO_MPU6500) && (who != WHO_MPU6050))
    {
        (void)printf("WHO_AM_I = 0x%02X - unexpected\n", (unsigned int)who);
    }
    else
    {
        (void)printf("WHO_AM_I = 0x%02X\n", (unsigned int)who);
        ok = mpu_configure(who);
    }

    return ok;
}

/*
 * อ่าน accel (g) และ gyro (dps) ทั้ง 3 แกนในครั้งเดียว (burst 14 byte)
 * อ่านรวดเดียวเพื่อให้ทุกค่ามาจาก sample เดียวกัน
 * อ่านไม่ได้จะไม่แตะค่าใน output (caller ใช้ค่าเดิมต่อ)
 */
bool mpu6500_read_all(float acc_g[MPU_AXES], float gyro_dps[MPU_AXES])
{
    uint8_t b[BURST_LEN];
    bool    ok;

    ok = mpu_read(REG_ACCEL_XOUT_H, b, sizeof(b));
    if (ok == true)
    {
        acc_g[AXIS_X]    = (float)be16(&b[OFS_AX]) / ACCEL_LSB_PER_G;
        acc_g[AXIS_Y]    = (float)be16(&b[OFS_AY]) / ACCEL_LSB_PER_G;
        acc_g[AXIS_Z]    = (float)be16(&b[OFS_AZ]) / ACCEL_LSB_PER_G;
        gyro_dps[AXIS_X] = (float)be16(&b[OFS_GX]) / GYRO_LSB_PER_DPS;
        gyro_dps[AXIS_Y] = (float)be16(&b[OFS_GY]) / GYRO_LSB_PER_DPS;
        gyro_dps[AXIS_Z] = (float)be16(&b[OFS_GZ]) / GYRO_LSB_PER_DPS;
    }
    else
    {
        /* No action: leave outputs untouched */
    }

    return ok;
}

/*
 * เหมือน mpu6500_read_all แต่คืน gyro 3 แกน + accel แกน z เท่านั้น
 * ใช้กับ vibration/thrust test ที่ไม่ต้องคำนวณมุม
 */
bool mpu6500_read_gyro(float gyro_dps[MPU_AXES], float *az_g)
{
    float acc[MPU_AXES];
    bool  ok;

    ok = mpu6500_read_all(acc, gyro_dps);
    if (ok == true)
    {
        *az_g = acc[AXIS_Z];
    }
    else
    {
        /* No action */
    }

    return ok;
}

/*
 * อ่าน INT_STATUS เพื่อปล่อยขา INT (ตั้งเป็น latch) กลับเป็น low
 * PID loop ไม่ต้องเรียก เพราะการอ่านข้อมูลทุกรอบปล่อยขาอยู่แล้ว
 */
bool mpu6500_clear_int(void)
{
    uint8_t st = 0U;

    return mpu_read(REG_INT_STATUS, &st, REG_LEN);
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/

/*
 * เขียน register 1 ตัว: ส่ง [reg, val] ใน transaction เดียว
 */
static bool mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[WRITE_LEN];

    buf[0] = reg;
    buf[1] = val;

    return (i2c_master_transmit(s_mpu, buf, sizeof(buf), I2C_TIMEOUT_MS) == ESP_OK);
}

/*
 * อ่าน len byte เริ่มที่ reg เก็บ error ล่าสุดไว้พิมพ์ตอน init ล้มเหลว
 */
static bool mpu_read(uint8_t reg, uint8_t *dst, size_t len)
{
    s_last_err = i2c_master_transmit_receive(s_mpu, &reg, REG_LEN, dst, len, I2C_TIMEOUT_MS);
    return (s_last_err == ESP_OK);
}

/*
 * ตั้งค่า register ทั้งหมดหลังรู้ว่าเป็น MPU6500/6050
 * accel 8 g กันไม่ให้แรงสั่นของใบพัดอิ่มตัว และต้องเปิด accel DLPF 10 Hz
 * (0x1D) เอง เพราะบน MPU6500 register CONFIG (0x1A) กรองแค่ gyro ถ้าไม่ตั้ง
 * accel จะวิ่งที่ 460 Hz แรงสั่นจะ alias กลายเป็นมุมเอียงหลอก ~15 องศา
 */
static bool mpu_configure(uint8_t who)
{
    bool ok;

    ok = mpu_write(REG_PWR_MGMT_1, PWR_WAKE_PLL);
    ok = ok && mpu_write(REG_CONFIG, DLPF_CFG);
    ok = ok && mpu_write(REG_SMPLRT_DIV, SMPLRT_DIV_1KHZ);
    ok = ok && mpu_write(REG_GYRO_CONFIG, GYRO_FS_250DPS);
    ok = ok && mpu_write(REG_ACCEL_CONFIG, ACCEL_FS_8G);
    if (who == WHO_MPU6500)
    {
        ok = ok && mpu_write(REG_ACCEL_CONFIG2, ACCEL_DLPF_10HZ);
    }
    else
    {
        /* No action: MPU6050 CONFIG already filters accel and gyro */
    }
    /* data-ready ที่ขา INT -> EXTI บน GPIO10 */
    ok = ok && mpu_write(REG_INT_PIN_CFG, INT_PIN_LATCH_HI);
    ok = ok && mpu_write(REG_INT_ENABLE, INT_RAW_RDY_EN);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_SETTLE_MS));

    return ok;
}

/*
 * ลอง probe ทุก address บน bus แล้วพิมพ์ตัวที่ตอบ
 * ไม่เจอเลย = ไฟ/GND/สาย SDA-SCL มีปัญหา, เจอแต่ไม่ใช่ 0x68 = address ผิด
 */
static void bus_scan_print(void)
{
    uint16_t a;
    uint32_t found = 0U;

    for (a = I2C_ADDR_FIRST; a < I2C_ADDR_END; a++)
    {
        if (i2c_master_probe(s_bus, a, I2C_TIMEOUT_MS) == ESP_OK)
        {
            (void)printf(" 0x%02X", (unsigned int)a);
            found++;
        }
        else
        {
            /* No action: nobody home */
        }
    }

    if (found == 0U)
    {
        (void)printf(" none - no device answers (power / GND / SDA-SCL)\n");
    }
    else
    {
        (void)printf("\n");
    }
}

/*
 * รวม 2 byte แบบ big-endian (MPU ส่ง high byte ก่อน) เป็น int16
 */
static int16_t be16(const uint8_t *p)
{
    uint16_t hi = (uint16_t)p[0];
    uint16_t lo = (uint16_t)p[1];

    return (int16_t)((uint16_t)(hi << BYTE_SHIFT) | lo);
}
