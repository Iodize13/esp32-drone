/**
 * Vibration test - ESP32-S3 DevKit + MPU6500 + 4 brushed motors.
 *
 * WIRING
 *   MPU6500: VCC -> 3V3, GND -> GND, SDA -> GPIO8, SCL -> GPIO9
 *   Motor gates (via 100R): M1 GPIO4, M2 GPIO5, M3 GPIO6, M4 GPIO7
 *   GND common between board, driver board and motor battery minus.
 *
 * Press BOOT (GPIO0) to start. Press BOOT again at any time = motors off.
 *   RUN_THRUST_TEST 0: idle baseline -> all motors 40 % -> each motor alone
 *   RUN_THRUST_TEST 1: slow ramp 0 -> 100 % to see if it lifts
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

/* ------------------------------------------------------------------ */
#define RUN_THRUST_TEST   (0U)

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
#define REG_ACCEL_XOUT_H  (0x3BU)
#define REG_PWR_MGMT_1    (0x6BU)
#define REG_WHO_AM_I      (0x75U)
#define WHO_MPU6500       (0x70U)
#define WHO_MPU6050       (0x68U)
#define BURST_LEN         (14U)

#define DLPF_CFG          (0x05U)        /* 0x03=41 Hz, 0x04=20 Hz, 0x05=10 Hz */
#define SMPLRT_DIV_1KHZ   (0x00U)
#define ACCEL_LSB_PER_G   (16384.0f)
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
        ok = ok && mpu_write(REG_ACCEL_CONFIG, 0x00U);           /* 2 g     */
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
void app_main(void)
{
    pwm_init();
    pwm_stop_all();
    button_init();

    vTaskDelay(pdMS_TO_TICKS(PHASE_SETTLE_MS));   /* time to open the monitor */
    printf("\n\n%s - PROPS OFF first, drone tied down\n",
           (RUN_THRUST_TEST == 1U) ? "THRUST test" : "VIBRATION test");

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

    for (;;)
    {
        if (button_pressed() == true)
        {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            if (button_pressed() == true)
            {
                wait_release();
                if (RUN_THRUST_TEST == 1U)
                {
                    thrust_test();
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
