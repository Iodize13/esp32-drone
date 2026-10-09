/**
 * Vibration test - ESP32-S3 DevKit + MPU6500 + 4 brushed motors.
 *
 * WIRING
 *   MPU6500: VCC -> 3V3, GND -> GND, SDA -> GPIO8, SCL -> GPIO9
 *   Motor gates (via 100R): M1 GPIO4, M2 GPIO5, M3 GPIO6, M4 GPIO7
 *   GND common between board, driver board and motor battery minus.
 *
 * Press BOOT (GPIO0) to start. Press BOOT again at any time = motors off.
 *   TEST_MODE TEST_VIBRATION: idle baseline -> all motors 40 % -> each alone
 *   TEST_MODE TEST_THRUST   : slow ramp 0 -> 100 % to see if it lifts
 *   TEST_MODE TEST_MAPPING  : find where each motor sits, print the
 *                             mixer sign table (PROPS OFF)
 *   TEST_MODE TEST_PID      : angle PID on the rig (one axis) or
 *                             tethered (roll + pitch), live tuning
 *
 * Serial: COM port, 115200.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/uart.h"
#include "attitude_filter.h"
#include "pid.h"
#include "mixer.h"
#include "adc_batt.h"
#include "imu_int.h"

/* ------------------------------------------------------------------ */
#define TEST_VIBRATION    (0U)
#define TEST_THRUST       (1U)
#define TEST_MAPPING      (2U)
#define TEST_PID          (3U)
#define TEST_MODE         (TEST_PID)

#define PIN_SDA           (8)
#define PIN_SCL           (9)
#define PIN_BUTTON        (0)            /* BOOT, active low */
#define I2C_HZ            (400000U)
#define I2C_TIMEOUT_MS    (100)

#define MPU_ADDR          (0x68U)
#define REG_SMPLRT_DIV    (0x19U)
#define REG_CONFIG        (0x1AU)
#define REG_GYRO_CONFIG   (0x1BU)
#define REG_ACCEL_CONFIG  (0x1CU)
#define REG_ACCEL_CONFIG2 (0x1DU)        /* MPU6500 only: accel DLPF */
#define ACCEL_FS_8G       (0x10U)
#define ACCEL_DLPF_10HZ   (0x05U)
#define REG_INT_PIN_CFG   (0x37U)
#define REG_INT_ENABLE    (0x38U)
#define REG_INT_STATUS    (0x3AU)
#define REG_ACCEL_XOUT_H  (0x3BU)
#define REG_PWR_MGMT_1    (0x6BU)
#define REG_WHO_AM_I      (0x75U)
#define WHO_MPU6500       (0x70U)
#define WHO_MPU6050       (0x68U)
#define BURST_LEN         (14U)

#define DLPF_CFG          (0x04U)        /* 0x03=41 Hz, 0x04=20 Hz, 0x05=10 Hz */
#define SMPLRT_DIV_1KHZ   (0x00U)
#define INT_PIN_PULSE_HI  (0x30U)        /* active high, push-pull, latched until any read */
#define INT_RAW_RDY_EN    (0x01U)        /* INT on every new sample            */
#define ACCEL_LSB_PER_G   (4096.0f)       /* +/-8 g */
#define GYRO_LSB_PER_DPS  (131.0f)

#define MOTOR_COUNT       (4U)
#define PWM_FREQ_HZ       (20000U)
#define PWM_RES           (LEDC_TIMER_11_BIT)
#define PWM_FULL_COUNTS   (2048U)        /* 2^11 = 100 % */
#define DUTY_FULL         (1000U)        /* per-mille    */
#define TEST_DUTY         (400U)         /* 40 %         */
#define RAMP_STEP_DUTY    (10U)
#define RAMP_STEP_MS      (20U)

#define PHASE_SETTLE_MS   (2000U)
#define PHASE_MEASURE_MS  (5000U)
#define COUNTDOWN_S       (3U)
#define MS_PER_S          (1000U)
#define US_PER_MS         (1000)
#define DEBOUNCE_MS       (30U)
#define POLL_MS           (10U)

#define THRUST_STEP_DUTY  (10U)          /* 1 % per step       */
#define THRUST_STEP_MS    (100U)         /* 0 -> 100 % in 10 s */
#define THRUST_HOLD_MS    (2000U)
#define THRUST_PRINT_EVERY (5U)

/* Target band is +/-3 dps. With gaussian-ish noise ~99 % of samples sit
 * inside +/-3 sigma, so the verdict uses 3 x std instead of the raw peak
 * (one spike should not fail the test). */
#define SIGMA_BAND        (3.0f)
#define LIMIT_PASS_DPS    (3.0f)
#define STAGGER_MS        (100U)
#define SATURATION_DPS    (245.0f)       /* +/-250 dps full scale */
#define MAX_DEV_DPS       (60.0f)
#define MIN_SAMPLES_FOR_MEAN (10U)
/* motors running always add noise; std this close to idle means they
 * never spun (battery cut-out, wiring), so the result is meaningless */
#define SPIN_STD_RATIO    (2.0f)
#define LIMIT_FAIL_DPS    (10.0f)
#define AXES              (3U)

typedef struct
{
    uint32_t n;
    float    mean;
    float    m2;         /* Welford running sum of squares */
    float    vmin;
    float    vmax;
} stat_t;

static const gpio_num_t motor_pin[MOTOR_COUNT] =
    { GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7 };
static const char *const motor_name[MOTOR_COUNT] =
    { "M1 G4", "M2 G5", "M3 G6", "M4 G7" };

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_mpu = NULL;

/* ================================================================== */
/* PWM (LEDC)                                                          */
/* ================================================================== */
static void pwm_init(void)
{
    ledc_timer_config_t tcfg = {
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
        ledc_channel_config_t ccfg = {
            .gpio_num   = (int)motor_pin[m],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)m,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0U,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    }
}

/* duty in per-mille on one motor */
static void pwm_set_one(uint32_t idx, uint32_t duty)
{
    uint32_t d      = (duty > DUTY_FULL) ? DUTY_FULL : duty;
    uint32_t counts = (d * PWM_FULL_COUNTS) / DUTY_FULL;

    if (idx < MOTOR_COUNT)
    {
        (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, counts);
        (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
    }
    else
    {
        /* invalid motor - ignore */
    }
}

static void pwm_set_all(uint32_t duty)
{
    uint32_t m;

    for (m = 0U; m < MOTOR_COUNT; m++)
    {
        pwm_set_one(m, duty);
    }
}

static void pwm_stop_all(void)
{
    pwm_set_all(0U);
}

/* ================================================================== */
/* Button                                                              */
/* ================================================================== */
static void button_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    (void)gpio_config(&io);
}

static bool button_pressed(void)
{
    return (gpio_get_level(PIN_BUTTON) == 0);
}

static void wait_release(void)
{
    while (button_pressed() == true)
    {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

/* Wait ms while watching the button. Returns true if the user aborted. */
static bool wait_abortable(uint32_t ms)
{
    int64_t end     = esp_timer_get_time() + ((int64_t)ms * US_PER_MS);
    bool    aborted = false;

    while ((esp_timer_get_time() < end) && (aborted == false))
    {
        aborted = button_pressed();
        vTaskDelay(1);
    }

    return aborted;
}

/* ================================================================== */
/* MPU6500                                                             */
/* ================================================================== */
static bool mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    return (i2c_master_transmit(s_mpu, buf, sizeof(buf), I2C_TIMEOUT_MS) == ESP_OK);
}

static esp_err_t s_last_err = ESP_OK;

static bool mpu_read(uint8_t reg, uint8_t *dst, size_t len)
{
    s_last_err = i2c_master_transmit_receive(s_mpu, &reg, 1U, dst, len, I2C_TIMEOUT_MS);
    return (s_last_err == ESP_OK);
}

#define I2C_RECOVERY_CLOCKS (9U)
#define I2C_INIT_RETRIES    (3U)
#define I2C_RETRY_MS        (50U)

static void pin_delay(void)
{
    esp_rom_delay_us(5U);                    /* ~100 kHz bit-bang */
}

/* A slave interrupted mid-byte (e.g. by an MCU reset) keeps SDA low
 * forever. Clock SCL until it lets go, then send a STOP. Also prints
 * the idle line levels, which must both be 1. */
static void i2c_bus_recover(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_SDA) | (1ULL << PIN_SCL),
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    uint32_t i;

    (void)gpio_config(&io);
    (void)gpio_set_level(PIN_SDA, 1U);
    (void)gpio_set_level(PIN_SCL, 1U);
    pin_delay();
    printf("idle lines: SDA=%d SCL=%d (both must be 1)\n",
           gpio_get_level(PIN_SDA), gpio_get_level(PIN_SCL));

    for (i = 0U; (i < I2C_RECOVERY_CLOCKS) && (gpio_get_level(PIN_SDA) == 0); i++)
    {
        (void)gpio_set_level(PIN_SCL, 0U);
        pin_delay();
        (void)gpio_set_level(PIN_SCL, 1U);
        pin_delay();
    }

    /* STOP: SDA low -> high while SCL high */
    (void)gpio_set_level(PIN_SDA, 0U);
    pin_delay();
    (void)gpio_set_level(PIN_SDA, 1U);
    pin_delay();

    if (i > 0U)
    {
        printf("bus recovery: %lu clocks, SDA now %d\n",
               (unsigned long)i, gpio_get_level(PIN_SDA));
    }
    else
    {
        /* bus was free */
    }

    /* hand the pins back clean so the I2C driver can claim them */
    (void)gpio_reset_pin(PIN_SDA);
    (void)gpio_reset_pin(PIN_SCL);
}

static bool mpu_init(void)
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
    uint8_t  who   = 0U;
    bool     ok    = false;
    bool     read  = false;
    uint32_t tries;

    /* i2c_bus_recover(); -- disabled: MPU NACKed register reads after it */
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_mpu));

    for (tries = 0U; (tries < I2C_INIT_RETRIES) && (read == false); tries++)
    {
        read = mpu_read(REG_WHO_AM_I, &who, 1U);
        if (read == false)
        {
            (void)i2c_master_bus_reset(s_bus);
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_MS));
        }
        else
        {
            /* got it */
        }
    }

    if (read == false)
    {
        printf("I2C read failed (%s) - scanning bus:", esp_err_to_name(s_last_err));
        {
            uint16_t a;
            uint32_t found = 0U;

            for (a = 0x08U; a < 0x78U; a++)
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
            printf((found == 0U) ? " none - no device answers (power / GND / SDA-SCL)\n" : "\n");
        }
    }
    else if ((who != WHO_MPU6500) && (who != WHO_MPU6050))
    {
        printf("WHO_AM_I = 0x%02X - unexpected\n", (unsigned int)who);
    }
    else
    {
        printf("WHO_AM_I = 0x%02X\n", (unsigned int)who);
        ok = mpu_write(REG_PWR_MGMT_1, 0x01U);                   /* wake, PLL */
        ok = ok && mpu_write(REG_CONFIG, (uint8_t)DLPF_CFG);
        ok = ok && mpu_write(REG_SMPLRT_DIV, (uint8_t)SMPLRT_DIV_1KHZ);
        ok = ok && mpu_write(REG_GYRO_CONFIG, 0x00U);            /* 250 dps */
        /* 8 g so prop vibration does not clip, and a 10 Hz accel DLPF:
         * CONFIG (0x1A) only filters the gyro on the MPU6500, so without
         * this the accel runs at 460 Hz and vibration aliases into a
         * false tilt of ~15 deg with the motors on */
        ok = ok && mpu_write(REG_ACCEL_CONFIG, (uint8_t)ACCEL_FS_8G);
        if (who == WHO_MPU6500)
        {
            ok = ok && mpu_write(REG_ACCEL_CONFIG2, (uint8_t)ACCEL_DLPF_10HZ);
        }
        else
        {
            /* MPU6050: CONFIG already filters accel and gyro */
        }
        /* data-ready on the INT pin -> EXTI on GPIO10 */
        ok = ok && mpu_write(REG_INT_PIN_CFG, (uint8_t)INT_PIN_PULSE_HI);
        ok = ok && mpu_write(REG_INT_ENABLE, (uint8_t)INT_RAW_RDY_EN);
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    return ok;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* g[] in dps, *az in g */
static bool mpu_read_gyro(float g[AXES], float *az)
{
    uint8_t b[BURST_LEN];
    bool    ok = mpu_read(REG_ACCEL_XOUT_H, b, sizeof(b));

    if (ok == true)
    {
        *az  = (float)be16(&b[4])  / ACCEL_LSB_PER_G;
        g[0] = (float)be16(&b[8])  / GYRO_LSB_PER_DPS;
        g[1] = (float)be16(&b[10]) / GYRO_LSB_PER_DPS;
        g[2] = (float)be16(&b[12]) / GYRO_LSB_PER_DPS;
    }
    else
    {
        /* leave outputs untouched */
    }

    return ok;
}

/* acc[] in g, g[] in dps */
static bool mpu_read_all(float acc[AXES], float g[AXES])
{
    uint8_t b[BURST_LEN];
    bool    ok = mpu_read(REG_ACCEL_XOUT_H, b, sizeof(b));

    if (ok == true)
    {
        acc[0] = (float)be16(&b[0])  / ACCEL_LSB_PER_G;
        acc[1] = (float)be16(&b[2])  / ACCEL_LSB_PER_G;
        acc[2] = (float)be16(&b[4])  / ACCEL_LSB_PER_G;
        g[0]   = (float)be16(&b[8])  / GYRO_LSB_PER_DPS;
        g[1]   = (float)be16(&b[10]) / GYRO_LSB_PER_DPS;
        g[2]   = (float)be16(&b[12]) / GYRO_LSB_PER_DPS;
    }
    else
    {
        /* leave outputs untouched */
    }

    return ok;
}

/* ================================================================== */
/* Statistics                                                          */
/* ================================================================== */
static void stat_reset(stat_t *s)
{
    s->n    = 0U;
    s->mean = 0.0f;
    s->m2   = 0.0f;
    s->vmin = 1.0e9f;
    s->vmax = -1.0e9f;
}

static void stat_add(stat_t *s, float x)
{
    float delta;

    s->n++;
    delta    = x - s->mean;
    s->mean += delta / (float)s->n;
    s->m2   += delta * (x - s->mean);

    if (x < s->vmin) { s->vmin = x; } else { /* no new min */ }
    if (x > s->vmax) { s->vmax = x; } else { /* no new max */ }
}

static float stat_std(const stat_t *s)
{
    return (s->n > 1U) ? sqrtf(s->m2 / (float)(s->n - 1U)) : 0.0f;
}

/* Sample gyro at 1 kHz (FreeRTOS tick = 1 ms) for duration_ms.
 * Returns true if BOOT was pressed (caller stops the motors). */
static uint32_t s_read_err = 0U;   /* I2C transfers that failed       */
static uint32_t s_rejected = 0U;   /* reads that came back corrupted  */

/* A corrupted I2C read (motor EMI) shows up as a full-scale value or a
 * jump no tied-down frame can make in 1 ms. Real vibration seen so far
 * stays well inside these limits. */
static bool sample_is_sane(const stat_t st[AXES], const float g[AXES])
{
    bool     ok = true;
    uint32_t a;

    for (a = 0U; a < AXES; a++)
    {
        float v   = g[a];
        float dev = v - st[a].mean;

        if ((v >= SATURATION_DPS) || (v <= -SATURATION_DPS))
        {
            ok = false;
        }
        else if ((st[a].n >= MIN_SAMPLES_FOR_MEAN) &&
                 ((dev > MAX_DEV_DPS) || (dev < -MAX_DEV_DPS)))
        {
            ok = false;
        }
        else
        {
            /* plausible */
        }
    }

    return ok;
}

/* Sample gyro at 1 kHz (FreeRTOS tick = 1 ms) for duration_ms.
 * Returns true if BOOT was pressed (caller stops the motors). */
static bool capture(stat_t st[AXES], uint32_t duration_ms)
{
    TickType_t last    = xTaskGetTickCount();
    uint32_t   i;
    uint32_t   a;
    float      g[AXES];
    float      az;
    bool       aborted = false;

    for (a = 0U; a < AXES; a++)
    {
        stat_reset(&st[a]);
    }
    s_read_err = 0U;
    s_rejected = 0U;

    for (i = 0U; (i < duration_ms) && (aborted == false); i++)
    {
        if (mpu_read_gyro(g, &az) == false)
        {
            s_read_err++;
        }
        else if (sample_is_sane(st, g) == false)
        {
            s_rejected++;
        }
        else
        {
            for (a = 0U; a < AXES; a++)
            {
                stat_add(&st[a], g[a]);
            }
        }
        aborted = button_pressed();
        vTaskDelayUntil(&last, 1);
    }

    return aborted;
}

static void print_stats(const char *name, const stat_t st[AXES])
{
    printf("%-6s n=%4lu err=%lu bad=%lu | mean %6.2f %6.2f %6.2f | std %5.2f %5.2f %5.2f | p-p %5.2f %5.2f %5.2f\n",
           name, (unsigned long)st[0].n, (unsigned long)s_read_err, (unsigned long)s_rejected,
           (double)st[0].mean, (double)st[1].mean, (double)st[2].mean,
           (double)stat_std(&st[0]), (double)stat_std(&st[1]), (double)stat_std(&st[2]),
           (double)(st[0].vmax - st[0].vmin), (double)(st[1].vmax - st[1].vmin),
           (double)(st[2].vmax - st[2].vmin));
}

/* ================================================================== */
/* Tests                                                               */
/* ================================================================== */
static bool countdown(void)
{
    uint32_t s;
    bool     aborted = false;

    for (s = COUNTDOWN_S; (s > 0U) && (aborted == false); s--)
    {
        printf("starting in %lu...\n", (unsigned long)s);
        aborted = wait_abortable(MS_PER_S);
    }

    return aborted;
}

/* ramp one motor (idx < MOTOR_COUNT) or all (idx == MOTOR_COUNT) */
static bool ramp(uint32_t idx, uint32_t target)
{
    uint32_t d;
    bool     aborted = false;

    for (d = 0U; (d <= target) && (aborted == false); d += RAMP_STEP_DUTY)
    {
        if (idx < MOTOR_COUNT)
        {
            pwm_set_one(idx, d);
        }
        else
        {
            pwm_set_all(d);
        }
        aborted = wait_abortable(RAMP_STEP_MS);
    }

    return aborted;
}

/* Start motors one at a time so their inrush currents do not add up
 * (a toy LiPo's protection circuit can trip on the combined surge). */
static bool ramp_staggered(uint32_t target)
{
    uint32_t m;
    bool     aborted = false;

    /* reverse order (M4 first): if everything still cuts out when the
     * 4th motor starts, it is total current, not one bad channel */
    for (m = MOTOR_COUNT; (m > 0U) && (aborted == false); m--)
    {
        printf("  start %s\n", motor_name[m - 1U]);
        aborted = ramp(m - 1U, target);
        aborted = aborted || wait_abortable(STAGGER_MS);
    }

    return aborted;
}

static void verdict(const stat_t idle[AXES], const stat_t run[AXES])
{
    float    worst = 0.0f;
    float    s;
    float    band;
    uint32_t a;

    for (a = 0U; a < AXES; a++)
    {
        s = stat_std(&run[a]);
        if (s > worst) { worst = s; } else { /* keep */ }
    }
    band = worst * SIGMA_BAND;

    printf("worst axis: +/-%.2f dps (3 std) | idle std %.2f %.2f %.2f | DLPF=0x%02X\n",
           (double)band, (double)stat_std(&idle[0]), (double)stat_std(&idle[1]),
           (double)stat_std(&idle[2]), (unsigned int)DLPF_CFG);

    if (worst < (SPIN_STD_RATIO * stat_std(&idle[0])))
    {
        printf("RESULT: INVALID - std same as idle, motors did not spin\n");
    }
    else if (band <= LIMIT_PASS_DPS)
    {
        printf("RESULT: PASS - OK to start PID\n");
    }
    else if (band < LIMIT_FAIL_DPS)
    {
        printf("RESULT: MARGINAL - more foam / lower DLPF / check props\n");
    }
    else
    {
        printf("RESULT: FAIL - fix vibration BEFORE PID\n");
    }
}

static void vibration_test(void)
{
    stat_t   idle[AXES];
    stat_t   run[AXES];
    uint32_t m;
    bool     aborted;

    aborted = countdown();

    if (aborted == false)
    {
        printf("[1/3] motors OFF - baseline\n");
        pwm_stop_all();
        aborted = wait_abortable(PHASE_SETTLE_MS);
    }
    else
    {
        /* skip */
    }

    if (aborted == false)
    {
        (void)capture(idle, PHASE_MEASURE_MS);   /* motors off - nothing to abort */
        print_stats("idle", idle);
        printf("[2/3] all motors 40%% (staggered start)\n");
        aborted = ramp_staggered(TEST_DUTY);
        aborted = aborted || wait_abortable(PHASE_SETTLE_MS);
    }
    else
    {
        /* skip */
    }

    if (aborted == false)
    {
        aborted = capture(run, PHASE_MEASURE_MS);
        pwm_stop_all();
    }
    else
    {
        /* skip */
    }

    if (aborted == false)
    {
        print_stats("all", run);
        verdict(idle, run);
        printf("[3/3] each motor alone at 40%%\n");
    }
    else
    {
        /* skip */
    }

    for (m = 0U; (m < MOTOR_COUNT) && (aborted == false); m++)
    {
        pwm_stop_all();
        aborted = wait_abortable(PHASE_SETTLE_MS);   /* spin down */
        aborted = aborted || ramp(m, TEST_DUTY);
        aborted = aborted || wait_abortable(PHASE_SETTLE_MS);

        if (aborted == false)
        {
            aborted = capture(run, PHASE_MEASURE_MS);
            pwm_stop_all();
            print_stats(motor_name[m], run);
        }
        else
        {
            /* stop below */
        }
    }

    pwm_stop_all();
    printf((aborted == true) ? "ABORTED - motors off\n\n"
                             : "done - press BOOT to repeat\n\n");
}

/* az above ~1 g while climbing means thrust > weight */
static void thrust_test(void)
{
    uint32_t duty = 0U;
    uint32_t step = 0U;
    bool     aborted;
    float    g[AXES];
    float    az;

    printf("THRUST RAMP - props on, TETHERED, hands clear. BOOT = stop\n");
    aborted = countdown();

    while ((duty <= DUTY_FULL) && (aborted == false))
    {
        pwm_set_all(duty);

        if ((step % THRUST_PRINT_EVERY) == 0U)
        {
            if (mpu_read_gyro(g, &az) == true)
            {
                printf("duty %3lu%%  az %5.2f g\n", (unsigned long)(duty / 10U), (double)az);
            }
            else
            {
                printf("duty %3lu%%  read error\n", (unsigned long)(duty / 10U));
            }
        }
        else
        {
            /* not a print step */
        }

        aborted = wait_abortable(THRUST_STEP_MS);
        duty   += THRUST_STEP_DUTY;
        step++;
    }

    if (aborted == false)
    {
        aborted = wait_abortable(THRUST_HOLD_MS);
    }
    else
    {
        /* already stopping */
    }

    pwm_stop_all();
    printf((aborted == true) ? "ABORTED - motors off\n\n" : "done - motors off\n\n");
}


/* ================================================================== */
/* Motor mapping test (PROPS OFF)                                      */
/*                                                                     */
/* For each motor: it spins briefly so you can see which one it is,    */
/* then you press THAT motor's corner down ~15-20 deg and hold. The    */
/* sign of roll/pitch while that corner is low gives the mixer sign:   */
/* a motor on the LOW side of a positive angle must be -1 (mixer.h).   */
/* ================================================================== */
#define MAP_DUTY          (250U)         /* 25 %, props off           */
#define MAP_SPIN_MS       (1500U)
#define MAP_BIAS_SAMPLES  (1000U)        /* 1 s still at 1 kHz        */
#define MAP_TILT_DEG      (8.0f)         /* corner counts as "down"   */
#define MAP_LEVEL_DEG     (3.0f)         /* back to level             */
#define MAP_HOLD_MS       (500U)         /* must hold tilt this long  */
#define MAP_LEVEL_HOLD_MS (1500U)        /* must stay level this long */
#define MAP_NEXT_PAUSE_MS (1000U)        /* gap before next motor     */
#define MAP_TIMEOUT_MS    (30000U)
#define MAP_PRINT_MS      (250U)
#define ATT_TAU_S         (0.5f)
#define ATT_ACC_MIN_G     (0.85f)
#define ATT_ACC_MAX_G     (1.15f)
#define LOOP_DT_S         (0.001f)

static attitude_t s_att;
static float      s_ref_roll;        /* resting pose; tilt is      */
static float      s_ref_pitch;       /* measured from here         */

/* Gyro bias with the frame still, then let the angle settle from the
 * accel. Returns false if too few samples were read. */
/* A still MPU6500 shows a few dps of bias. More than this means the
 * frame was swinging during calibration, which would make the angle
 * drift at that rate for the whole run. */
#define CAL_BIAS_MAX_DPS  (10.0f)
#define CAL_TRIES         (5U)
#define CAL_RETRY_MS      (500U)

static bool att_calibrate_once(void)
{
    att_config_t acfg = { ATT_TAU_S, ATT_ACC_MIN_G, ATT_ACC_MAX_G };
    float        acc[AXES];
    float        raw[AXES];
    uint32_t     i;
    bool         ok;

    att_init(&s_att, &acfg);
    for (i = 0U; i < MAP_BIAS_SAMPLES; i++)
    {
        if (mpu_read_all(acc, raw) == true)
        {
            att_bias_add(&s_att, raw);
        }
        else
        {
            /* skip */
        }
        vTaskDelay(1);
    }
    ok = att_bias_finish(&s_att, MAP_BIAS_SAMPLES / 2U);
    for (i = 0U; i < AXES; i++)
    {
        if (fabsf(s_att.bias_dps[i]) > CAL_BIAS_MAX_DPS)
        {
            ok = false;                  /* frame was moving */
        }
        else
        {
            /* plausible bias */
        }
    }
    printf("gyro bias %s: %.2f %.2f %.2f dps\n",
           (ok == true) ? "ok" : "REJECTED (frame moving?)",
           (double)s_att.bias_dps[0], (double)s_att.bias_dps[1],
           (double)s_att.bias_dps[2]);

    return ok;
}

/* Retry until the frame holds still, so a swing after BOOT does not
 * spoil the whole run. */
static bool att_calibrate(void)
{
    bool     ok = false;
    uint32_t n;

    for (n = 0U; (n < CAL_TRIES) && (ok == false); n++)
    {
        ok = att_calibrate_once();
        if (ok == false)
        {
            vTaskDelay(pdMS_TO_TICKS(CAL_RETRY_MS));
        }
        else
        {
            /* calibrated */
        }
    }

    return ok;
}

static bool att_run(uint32_t ms);

/* one 1 ms filter step; returns false on read error */
static bool att_step(void)
{
    float acc[AXES];
    float raw[AXES];
    float g[AXES];
    bool  ok = mpu_read_all(acc, raw);

    if (ok == true)
    {
        att_unbias(&s_att, raw, g);
        att_update(&s_att, acc, g, LOOP_DT_S);
    }
    else
    {
        /* skip this step */
    }

    return ok;
}

static float absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* Run the filter for ms. Returns true if BOOT aborted. */
static bool att_run(uint32_t ms)
{
    TickType_t last    = xTaskGetTickCount();
    uint32_t   i;
    bool       aborted = false;

    for (i = 0U; (i < ms) && (aborted == false); i++)
    {
        (void)att_step();
        aborted = button_pressed();
        vTaskDelayUntil(&last, 1);
    }

    return aborted;
}

/* Wait until want_tilt (corner held down, MAP_HOLD_MS) or level
 * (MAP_LEVEL_HOLD_MS).
 * Returns 0 ok, 1 aborted, 2 timeout. Prints angles while waiting. */
static uint32_t wait_pose(bool want_tilt)
{
    TickType_t last   = xTaskGetTickCount();
    uint32_t   i;
    uint32_t   held   = 0U;
    uint32_t   result = 2U;
    float      dr;
    float      dp;
    float      r;
    float      p;
    bool       match;
    uint32_t   hold_ms = (want_tilt == true) ? MAP_HOLD_MS : MAP_LEVEL_HOLD_MS;

    for (i = 0U; (i < MAP_TIMEOUT_MS) && (result == 2U); i++)
    {
        (void)att_step();
        dr = s_att.roll_deg  - s_ref_roll;
        dp = s_att.pitch_deg - s_ref_pitch;
        r  = absf(dr);
        p  = absf(dp);

        if (want_tilt == true)
        {
            match = (r >= MAP_TILT_DEG) || (p >= MAP_TILT_DEG);
        }
        else
        {
            match = (r < MAP_LEVEL_DEG) && (p < MAP_LEVEL_DEG);
        }

        held = (match == true) ? (held + 1U) : 0U;

        if (held >= hold_ms)
        {
            result = 0U;
        }
        else if (button_pressed() == true)
        {
            result = 1U;
        }
        else
        {
            /* keep waiting */
        }

        if ((i % MAP_PRINT_MS) == 0U)
        {
            printf("    d-roll %+6.1f  d-pitch %+6.1f\r", (double)dr, (double)dp);
            (void)fflush(stdout);
        }
        else
        {
            /* not a print step */
        }
        vTaskDelayUntil(&last, 1);
    }
    printf("\n");

    return result;
}

/* +1 / -1 / 0 for an angle held with this motor's corner DOWN */
static int8_t sign_when_low(float angle_deg)
{
    int8_t s;

    if (angle_deg >= (MAP_TILT_DEG * 0.5f))
    {
        s = -1;          /* low side of a positive angle -> -1 */
    }
    else if (angle_deg <= -(MAP_TILT_DEG * 0.5f))
    {
        s = 1;
    }
    else
    {
        s = 0;           /* motor sits on this axis (plus layout) */
    }

    return s;
}

static void print_row(const char *name, const int8_t v[MOTOR_COUNT])
{
    printf("    .%-5s = { %+d, %+d, %+d, %+d },\n", name,
           (int)v[0], (int)v[1], (int)v[2], (int)v[3]);
}

static void mapping_test(void)
{
    int8_t       roll_s[MOTOR_COUNT]  = { 0, 0, 0, 0 };
    int8_t       pitch_s[MOTOR_COUNT] = { 0, 0, 0, 0 };
    int8_t       yaw_s[MOTOR_COUNT];
    int32_t      sum_r = 0;
    int32_t      sum_p = 0;
    uint32_t     m;
    uint32_t     res  = 0U;
    bool         ok;

    printf("MOTOR MAPPING - PROPS OFF. Keep the frame level and still...\n");
    ok = att_calibrate();
    (void)att_run(MAP_HOLD_MS);   /* let the angle settle from accel */
    s_ref_roll  = s_att.roll_deg;
    s_ref_pitch = s_att.pitch_deg;
    printf("resting pose: roll %+.1f pitch %+.1f (tilt measured from here)\n",
           (double)s_ref_roll, (double)s_ref_pitch);

    for (m = 0U; (m < MOTOR_COUNT) && (res == 0U) && (ok == true); m++)
    {
        printf("\n[%s] spinning - see WHICH motor it is (write its position down)\n",
               motor_name[m]);
        pwm_set_one(m, MAP_DUTY);
        res = (att_run(MAP_SPIN_MS) == true) ? 1U : 0U;
        pwm_stop_all();

        if (res == 0U)
        {
            printf("[%s] now press THAT motor's corner DOWN ~15-20 deg and hold\n",
                   motor_name[m]);
            res = wait_pose(true);
        }
        else
        {
            /* aborted */
        }

        if (res == 0U)
        {
            roll_s[m]  = sign_when_low(s_att.roll_deg - s_ref_roll);
            pitch_s[m] = sign_when_low(s_att.pitch_deg - s_ref_pitch);
            printf("[%s] corner down: d-roll %+.1f d-pitch %+.1f -> roll %+d pitch %+d\n",
                   motor_name[m], (double)(s_att.roll_deg - s_ref_roll),
                   (double)(s_att.pitch_deg - s_ref_pitch),
                   (int)roll_s[m], (int)pitch_s[m]);
            printf("    release - back to level\n");
            res = wait_pose(false);
        }
        else
        {
            /* aborted or timed out */
        }

        if ((res == 0U) && ((m + 1U) < MOTOR_COUNT))
        {
            printf("    level - next motor in %lu ms\n", (unsigned long)MAP_NEXT_PAUSE_MS);
            res = (att_run(MAP_NEXT_PAUSE_MS) == true) ? 1U : 0U;
        }
        else
        {
            /* aborted or timed out */
        }
    }

    pwm_stop_all();

    if (res != 0U)
    {
        printf("%s - mapping not finished\n\n", (res == 1U) ? "ABORTED" : "TIMEOUT");
    }
    else
    {
        /* X quad: diagonal motors share a prop direction, and the
         * diagonals are the motors with the same roll*pitch product. */
        for (m = 0U; m < MOTOR_COUNT; m++)
        {
            yaw_s[m] = (int8_t)(roll_s[m] * pitch_s[m]);
            sum_r   += roll_s[m];
            sum_p   += pitch_s[m];
        }

        printf("\nmixer_config_t signs (paste into the flight code):\n");
        print_row("roll", roll_s);
        print_row("pitch", pitch_s);
        print_row("yaw", yaw_s);

        if ((sum_r != 0) || (sum_p != 0))
        {
            printf("WARNING: roll or pitch signs do not balance (sum %ld / %ld).\n"
                   "Two motors probably got the same corner - redo the test.\n",
                   (long)sum_r, (long)sum_p);
        }
        else
        {
            printf("roll/pitch signs balance - looks like a valid X layout.\n");
        }
        printf("yaw: only the diagonal pairing is known. If the drone spins up\n"
               "faster when yaw control is on, negate the whole yaw row.\n\n");
    }
}

/* ================================================================== */
/* Angle PID (PROPS ON)                                                */
/*                                                                     */
/* PID_AXES picks what is controlled:                                  */
/*   PID_AXES_ROLL / PID_AXES_PITCH : single-axis rig - frame on a rod */
/*       through the CG, free on that axis only. Tune here first.      */
/*   PID_AXES_BOTH : roll + pitch, drone tethered. Same gains on both  */
/*       axes (X frame is near symmetric). Yaw is not controlled yet.  */
/*                                                                     */
/* Calibrate with the frame still, BOOT starts after a countdown, BOOT */
/* again = motors off. Live tuning over serial (applies to all axes):  */
/*   p/P kp -/+   i/I ki -/+   d/D kd -/+   t/T throttle -/+           */
/*   l / r / c  lean left / lean right / centre (PID_STEP_DEG)         */
/*              roll; on the pitch rig l = nose up, r = nose down      */
/*   x   motors off                                                    */
/*   0   PID off <-> on, gains kept (demo: falls over, then recovers)  */
/* ================================================================== */
#define PID_AXES_ROLL     (0U)
#define PID_AXES_PITCH    (1U)
#define PID_AXES_BOTH     (2U)
#define PID_AXES          (PID_AXES_ROLL)

/* angle the IMU reads with the frame level (mapping-test "resting
 * pose" on a flat table); PID holds this as zero */
#define TRIM_ROLL_DEG     (0.0f)
#define TRIM_PITCH_DEG    (-2.5f)

#define PID_KP_START      (12.0f)        /* per-mille per degree        */
#define PID_KI_START      (7.0f)         /* per-mille per degree-second */
#define PID_KD_START      (2.0f)         /* per-mille per dps           */
#define PID_KP_STEP       (0.5f)
#define PID_KI_STEP       (0.5f)
#define PID_KD_STEP       (0.05f)
#define PID_I_LIMIT       (50.0f)        /* per-mille                   */
#define PID_OUT_LIMIT     (250.0f)       /* per-mille                   */
#define PID_D_CUTOFF_HZ   (40.0f)

#define PID_THROTTLE      (400U)         /* per-mille base, all motors  */
#define PID_THR_STEP      (25U)
#define PID_THR_MAX       (700U)
#define PID_MAX_DUTY      (850U)
#define PID_IDLE_DUTY     (60U)          /* keep motors turning         */
#define PID_SPOOL_MS      (1500U)        /* throttle ramp, no PID yet   */
#define PID_STEP_DEG      (20.0f)
#define PID_SP_RATE_DPS   (20.0f)        /* setpoint ramp, deg per s    */
#define PID_CUTOFF_DEG    (45.0f)        /* past this: motors off       */
#define PID_MAX_READ_ERR  (20U)          /* consecutive I2C errors      */
#define PID_PRINT_MS      (50U)
#define PID_INT_WAIT_MS   (2U)           /* > 1 ms sample period        */
#define PID_DT_MIN_S      (0.0002f)
#define PID_DT_MAX_S      (0.005f)
#define US_TO_S           (1.0e-6f)
#define UART_RX_BUF       (256)

/* from the mapping test 2026-10-05: M1 FR, M2 BR, M3 BL, M4 FL */
static const mixer_config_t s_mix_cfg = {
    .roll      = { 1, 1, -1, -1 },
    .pitch     = { 1, -1, -1, 1 },
    .yaw       = { 1, -1, 1, -1 },       /* direction unverified, unused */
    .max_duty  = (uint16_t)PID_MAX_DUTY,
    .idle_duty = (uint16_t)PID_IDLE_DUTY,
};

static float step_down(float v, float step)
{
    return (v > step) ? (v - step) : 0.0f;
}

static bool s_pid_on = true;

static void pid_print_gains(const pid_config_t *c, uint32_t thr, float sp)
{
    printf("\nkp %.2f  ki %.2f  kd %.3f  thr %lu  sp %+.1f  PID %s\n", (double)c->kp,
           (double)c->ki, (double)c->kd, (unsigned long)thr, (double)sp,
           (s_pid_on == true) ? "ON" : "OFF");
}

/* Battery divider: 100k / 100k, so battery = pin * 2. Check with the
 * 'b' key against a meter on BATT+ (an analog meter loads a 100k divider
 * and reads low at the midpoint - measure BATT+ only). */
#define BATT_DIV_RATIO    (2.0f)

/* 1S LiPo limits. Under load the voltage sags ~0.3-0.5 V, so the
 * in-flight cut is lower than the start check and must last a while. */
#define BATT_START_MIN_MV (3600U)        /* refuse to start below this  */
#define BATT_WARN_MV      (3500U)        /* print a warning once        */
#define BATT_CUT_MV       (3200U)        /* motors off below this ...   */
#define BATT_CUT_S        (1.0f)         /* ... for this long           */
#define BATT_GLITCH_MV    (5000U)        /* impossible for 1S: ignore   */

/* Battery voltage in mV, 0 if the ADC has no data yet. */
static uint32_t batt_mv(void)
{
    uint32_t pin = 0U;
    uint32_t out = 0U;

    if (adc_batt_pin_mv(&pin) == true)
    {
        out = (uint32_t)(((float)pin * BATT_DIV_RATIO) + 0.5f);
    }
    else
    {
        /* no data */
    }
    return out;
}

/* One serial key. Gains are changed in both controllers. Returns true
 * on 'x' (stop). */
static bool pid_key(pid_ctrl_t pid[2], uint32_t *thr, float *sp)
{
    uint8_t c       = 0U;
    bool    stop    = false;
    bool    changed = true;
    uint32_t a;

    if (uart_read_bytes(UART_NUM_0, &c, 1U, 0) == 1)
    {
        for (a = 0U; a < 2U; a++)
        {
            pid_config_t *k = &pid[a].cfg;

            switch (c)
            {
                case 'P': k->kp += PID_KP_STEP;               break;
                case 'p': k->kp  = step_down(k->kp, PID_KP_STEP); break;
                case 'I': k->ki += PID_KI_STEP;               break;
                case 'i': k->ki  = step_down(k->ki, PID_KI_STEP); break;
                case 'D': k->kd += PID_KD_STEP;               break;
                case 'd': k->kd  = step_down(k->kd, PID_KD_STEP); break;
                default:  /* not a gain key */                break;
            }
        }

        switch (c)
        {
            case 'P': case 'p': case 'I': case 'i': case 'D': case 'd':
                break;
            case 'T':
                *thr = ((*thr + PID_THR_STEP) > PID_THR_MAX) ? PID_THR_MAX
                                                             : (*thr + PID_THR_STEP);
                break;
            case 't':
                *thr = (*thr > PID_THR_STEP) ? (*thr - PID_THR_STEP) : 0U;
                break;
            case 'l':
                *sp = PID_STEP_DEG;          /* left side down = +roll */
                break;
            case 'r':
                *sp = -PID_STEP_DEG;
                break;
            case 'c':
                *sp = 0.0f;
                break;
            case 'x':
                stop = true;
                break;
            case '0':
                s_pid_on = (s_pid_on == false);
                for (a = 0U; a < 2U; a++)
                {
                    pid_reset(&pid[a]);     /* no stale I or D on resume */
                }
                break;
            default:
                changed = false;
                printf("key 0x%02x ignored\n", (unsigned int)c);
                break;
        }
    }
    else
    {
        changed = false;
    }

    if (changed == true)
    {
        pid_print_gains(&pid[0].cfg, *thr, *sp);
    }
    else
    {
        /* nothing to report */
    }

    return stop;
}

static void pid_test(void)
{
    pid_config_t pcfg = { PID_KP_START, PID_KI_START, PID_KD_START,
                          PID_I_LIMIT, PID_OUT_LIMIT, PID_D_CUTOFF_HZ };
    pid_ctrl_t   pid[2];                 /* [0] roll, [1] pitch */
    uint16_t     duty[MOTOR_COUNT] = { 0U, 0U, 0U, 0U };
    float        acc[AXES];
    float        raw[AXES];
    float        g[AXES]   = { 0.0f, 0.0f, 0.0f };
    float        ang[2]    = { 0.0f, 0.0f };
    float        corr[2]   = { 0.0f, 0.0f };
    float        sp        = 0.0f;       /* ramped, fed to the PID      */
    float        sp_cmd    = 0.0f;       /* target set by l / r / c     */
    float        sp_step;
    float        dt;
    uint32_t     thr_target = PID_THROTTLE;
    uint32_t     thr;
    uint32_t     elapsed_ms;
    uint32_t     err_run   = 0U;
    uint32_t     loops     = 0U;
    uint32_t     a;
    int64_t      now;
    int64_t      prev;
    int64_t      t_start;
    uint32_t     int_miss  = 0U;
    uint32_t     bat;
    float        bat_low_s = 0.0f;
    bool         bat_warned = false;
    bool         stop      = false;
    const char  *why       = "BOOT";
    const bool   use_roll  = (PID_AXES != PID_AXES_PITCH);
    const bool   use_pitch = (PID_AXES != PID_AXES_ROLL);

    printf("ANGLE PID (%s) - PROPS ON, %s, hands clear\n",
           (PID_AXES == PID_AXES_ROLL) ? "roll rig" :
           ((PID_AXES == PID_AXES_PITCH) ? "pitch rig" : "roll + pitch"),
           (PID_AXES == PID_AXES_BOTH) ? "TETHERED" : "frame on the pivot rod");
    printf("keys: p/P kp  i/I ki  d/D kd  t/T throttle  l/r/c lean  0 PID on/off\n"
           "      x stop  BOOT stop\n");
    printf("Hold the frame still for calibration...\n");

    (void)uart_flush_input(UART_NUM_0);  /* drop keys typed while idle */

    bat = batt_mv();
    if ((bat > 0U) && (bat < BATT_START_MIN_MV))
    {
        stop = true;
        why  = "battery low - charge it";
        printf("battery %lu mV < %u mV\n", (unsigned long)bat,
               (unsigned int)BATT_START_MIN_MV);
    }
    else if (att_calibrate() == false)
    {
        stop = true;
        why  = "calibration failed";
    }
    else
    {
        (void)att_run(MAP_HOLD_MS);      /* settle the angle */
        stop = countdown();
    }

    for (a = 0U; a < 2U; a++)
    {
        pid_init(&pid[a], &pcfg, LOOP_DT_S);
    }
    s_pid_on = true;
    pid_print_gains(&pcfg, thr_target, sp);

    prev    = esp_timer_get_time();
    t_start = prev;

    while (stop == false)
    {
        now  = esp_timer_get_time();
        dt   = (float)(now - prev) * US_TO_S;
        prev = now;
        dt   = (dt < PID_DT_MIN_S) ? PID_DT_MIN_S :
               ((dt > PID_DT_MAX_S) ? PID_DT_MAX_S : dt);

        if (mpu_read_all(acc, raw) == true)
        {
            err_run = 0U;
            att_unbias(&s_att, raw, g);
            att_update(&s_att, acc, g, dt);
        }
        else
        {
            err_run++;                   /* keep the last angle and rate */
        }

        ang[0] = s_att.roll_deg  - TRIM_ROLL_DEG;
        ang[1] = s_att.pitch_deg - TRIM_PITCH_DEG;

        /* move the setpoint toward the command at a fixed rate: a 10 deg
         * jump made P kick hard and overshoot by ~5 deg */
        sp_step = PID_SP_RATE_DPS * dt;
        if ((sp_cmd - sp) > sp_step)
        {
            sp += sp_step;
        }
        else if ((sp - sp_cmd) > sp_step)
        {
            sp -= sp_step;
        }
        else
        {
            sp = sp_cmd;
        }

        /* battery: warn once, cut only if it stays low (a short sag
         * while the motors speed up is normal) */
        bat = batt_mv();
        if ((bat > 0U) && (bat < BATT_CUT_MV))
        {
            bat_low_s += dt;
        }
        else if ((bat > 0U) && (bat < BATT_GLITCH_MV))
        {
            bat_low_s = 0.0f;
        }
        else
        {
            /* no data or ADC glitch - keep the timer as is */
        }
        if ((bat > 0U) && (bat < BATT_WARN_MV) && (bat_warned == false))
        {
            bat_warned = true;
            printf("WARNING battery %lu mV - land soon\n", (unsigned long)bat);
        }
        else
        {
            /* fine, or already warned */
        }

        elapsed_ms = (uint32_t)((now - t_start) / US_PER_MS);
        if ((elapsed_ms >= PID_SPOOL_MS) && (s_pid_on == false))
        {
            thr     = thr_target;            /* demo: throttle only */
            corr[0] = 0.0f;
            corr[1] = 0.0f;
        }
        else if (elapsed_ms < PID_SPOOL_MS)
        {
            /* spool up with no correction, so I does not wind up while
             * the motors are still too slow to respond */
            thr = (thr_target * elapsed_ms) / PID_SPOOL_MS;
            thr = (thr == 0U) ? 1U : thr;
            corr[0] = 0.0f;
            corr[1] = 0.0f;
        }
        else
        {
            thr     = thr_target;
            corr[0] = (use_roll == true)
                    ? pid_update(&pid[0], sp, ang[0], g[0], dt) : 0.0f;
            corr[1] = (use_pitch == true)
                    ? pid_update(&pid[1], (use_roll == true) ? 0.0f : sp,
                                 ang[1], g[1], dt) : 0.0f;
        }

        mixer_mix(&s_mix_cfg, (uint16_t)thr, corr[0], corr[1], 0.0f, duty);
        for (a = 0U; a < MOTOR_COUNT; a++)
        {
            pwm_set_one(a, duty[a]);
        }

        if (((use_roll == true) && (absf(ang[0]) > PID_CUTOFF_DEG)) ||
            ((use_pitch == true) && (absf(ang[1]) > PID_CUTOFF_DEG)))
        {
            stop = true;
            why  = "angle limit";
        }
        else if (err_run >= PID_MAX_READ_ERR)
        {
            stop = true;
            why  = "I2C errors";
        }
        else if (bat_low_s >= BATT_CUT_S)
        {
            stop = true;
            why  = "battery low";
        }
        else if (button_pressed() == true)
        {
            stop = true;
            why  = "BOOT";
        }
        else if (pid_key(pid, &thr_target, &sp_cmd) == true)
        {
            stop = true;
            why  = "x key";
        }
        else
        {
            /* keep flying */
        }

        if ((loops % PID_PRINT_MS) == 0U)
        {
            a = (use_roll == true) ? 0U : 1U;    /* axis shown in the log */
            printf("%s %+6.1f sp %+5.1f rate %+7.1f P %+5.0f I %+5.0f D %+5.0f"
                   " M %3u %3u %3u %3u B %4lu\n", (a == 0U) ? "R" : "P",
                   (double)ang[a], (double)sp, (double)g[a],
                   (double)pid[a].last_p, (double)pid[a].last_i,
                   (double)pid[a].last_d, (unsigned int)duty[0],
                   (unsigned int)duty[1], (unsigned int)duty[2],
                   (unsigned int)duty[3], (unsigned long)batt_mv());
            if (PID_AXES == PID_AXES_BOTH)
            {
                printf("P %+6.1f rate %+7.1f P %+5.0f I %+5.0f D %+5.0f\n",
                       (double)ang[1], (double)g[1], (double)pid[1].last_p,
                       (double)pid[1].last_i, (double)pid[1].last_d);
            }
            else
            {
                /* single axis */
            }
        }
        else
        {
            /* not a print step */
        }
        loops++;

        /* pace the loop on the IMU data-ready interrupt; a timeout means
         * INT is not wired, and the loop then runs every PID_INT_WAIT_MS */
        if (imu_int_wait(PID_INT_WAIT_MS) == false)
        {
            int_miss++;
        }
        else
        {
            /* new sample ready */
        }
    }

    pwm_stop_all();
    printf("\nMOTORS OFF (%s)\n", why);
    printf("EXTI: %lu loops, %lu without data-ready (%s)\n",
           (unsigned long)loops, (unsigned long)int_miss,
           (int_miss > (loops / 2U)) ? "INT NOT WORKING - check GPIO10 wire" : "ok");
    pid_print_gains(&pid[0].cfg, thr_target, sp);
    printf("\n");
}

/* UART0 RX driver for the live-tuning keys. Installed at boot so a
 * typed key can be checked before the motors run. */
static bool keys_init(void)
{
    esp_err_t err = ESP_OK;

    if (uart_is_driver_installed(UART_NUM_0) == false)
    {
        err = uart_driver_install(UART_NUM_0, UART_RX_BUF, 0, 0, NULL, 0);
    }
    else
    {
        /* already installed */
    }
    return (err == ESP_OK);
}

/* 'a' while idle: accel-only roll/pitch, averaged, with the PID trim
 * applied. Hold the frame physically level to read the trim needed. */
#define LEVEL_SAMPLES     (200U)
#define LEVEL_RAD_TO_DEG  (57.29578f)

static void level_print(void)
{
    float    acc[AXES];
    float    raw[AXES];
    float    sum[AXES] = { 0.0f, 0.0f, 0.0f };
    uint32_t n = 0U;
    uint32_t i;
    uint32_t k;

    for (i = 0U; i < LEVEL_SAMPLES; i++)
    {
        if (mpu_read_all(acc, raw) == true)
        {
            for (k = 0U; k < AXES; k++)
            {
                sum[k] += acc[k];
            }
            n++;
        }
        else
        {
            /* skip a bad read */
        }
        vTaskDelay(1);
    }

    if (n > 0U)
    {
        float roll  = atan2f(sum[1], sum[2]) * LEVEL_RAD_TO_DEG;
        float pitch = atan2f(-sum[0], sqrtf((sum[1] * sum[1]) +
                                            (sum[2] * sum[2]))) * LEVEL_RAD_TO_DEG;
        printf("level: roll %+.1f pitch %+.1f (raw)  ->  roll %+.1f pitch %+.1f"
               " (after trim)\n", (double)roll, (double)pitch,
               (double)(roll - TRIM_ROLL_DEG), (double)(pitch - TRIM_PITCH_DEG));
    }
    else
    {
        printf("level: IMU read failed\n");
    }
}

/* 'b' while idle: pin and battery voltage, to check BATT_DIV_RATIO
 * against a multimeter on BATT+. */
static void batt_print(void)
{
    uint32_t pin = 0U;

    if (adc_batt_pin_mv(&pin) == true)
    {
        printf("battery: pin %lu mV -> battery %lu mV (ratio %.3f)\n",
               (unsigned long)pin, (unsigned long)batt_mv(),
               (double)BATT_DIV_RATIO);
    }
    else
    {
        printf("battery: no ADC data\n");
    }
}

/* 'i' while idle: count data-ready edges over 100 ms; ~100 means the
 * INT wire to GPIO10 works. */
#define INT_CHECK_MS      (100U)

static void int_print(void)
{
    uint8_t  st = 0U;
    uint32_t n0 = imu_int_count();
    uint32_t k;

    /* the INT line is latched until a read, so read the status every
     * 1 ms, as the PID loop does with the sample */
    for (k = 0U; k < INT_CHECK_MS; k++)
    {
        (void)mpu_read(REG_INT_STATUS, &st, 1U);
        vTaskDelay(1);
    }
    printf("IMU INT: %lu edges in %u ms (expect ~%u)\n",
           (unsigned long)(imu_int_count() - n0), (unsigned int)INT_CHECK_MS,
           (unsigned int)INT_CHECK_MS);
}

/* While idle, answer every key so the serial link can be checked with
 * the motors off. */
static void keys_echo_idle(void)
{
    uint8_t c = 0U;

    if (uart_read_bytes(UART_NUM_0, &c, 1U, 0) == 1)
    {
        switch (c)
        {
            case 'a':
                level_print();
                break;
            case 'b':
                batt_print();
                break;
            case 'i':
                int_print();
                break;
            default:
                printf("key '%c' received - motors off, press BOOT to start\n",
                       ((c >= (uint8_t)' ') && (c <= (uint8_t)'~')) ? (char)c : '?');
                break;
        }
    }
    else
    {
        /* no key */
    }
}

/* ================================================================== */
void app_main(void)
{
    pwm_init();
    pwm_stop_all();
    button_init();

    vTaskDelay(pdMS_TO_TICKS(PHASE_SETTLE_MS));   /* time to open the monitor */
    printf("\n\n%s - PROPS OFF first, drone tied down\n",
           (TEST_MODE == TEST_THRUST) ? "THRUST test" :
           ((TEST_MODE == TEST_MAPPING) ? "MAPPING test" :
           ((TEST_MODE == TEST_PID) ? "PID test" : "VIBRATION test")));

    if (mpu_init() == false)
    {
        printf("MPU init failed - halting\n");
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(MS_PER_S));
        }
    }
    else
    {
        printf("ready - press BOOT to start\n");
    }

    if (keys_init() == false)
    {
        printf("UART0 RX driver failed - keys will not work\n");
    }
    else
    {
        /* keys ready */
    }

    if (imu_int_init() == false)
    {
        printf("IMU INT (EXTI) init failed\n");
    }
    else
    {
        printf("IMU INT (EXTI) on GPIO10\n");
    }

    if (adc_batt_init() == false)
    {
        printf("battery ADC init failed\n");
    }
    else
    {
        printf("battery ADC on GPIO1 - press 'b' to read\n");
    }

    for (;;)
    {
        keys_echo_idle();
        if (button_pressed() == true)
        {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            if (button_pressed() == true)
            {
                wait_release();
                if (TEST_MODE == TEST_THRUST)
                {
                    thrust_test();
                }
                else if (TEST_MODE == TEST_MAPPING)
                {
                    mapping_test();
                }
                else if (TEST_MODE == TEST_PID)
                {
                    pid_test();
                }
                else
                {
                    vibration_test();
                }
                wait_release();
            }
            else
            {
                /* bounce */
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        }
    }
}
