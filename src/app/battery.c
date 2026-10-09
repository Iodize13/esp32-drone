/*******************************************************************************
 * File Name    : battery.c
 * Description  : battery monitor (app layer) ใช้ค่าจาก driver adc_batt
 *                - divider 100k/100k: แรงดันแบต = แรงดันขา x 2
 *                - ไม่ให้เริ่มบินถ้าแบต < 3.6 V, เตือนเมื่อ < 3.5 V
 *                - ดับมอเตอร์เมื่อ < 3.2 V ต่อเนื่อง 1 วินาที (แรงดันตกชั่วคราว
 *                  ตอนมอเตอร์เร่งเป็นเรื่องปกติ จึงไม่ตัดทันที)
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "battery.h"
#include <stddef.h>
#include <stdio.h>
#include "adc_batt.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Analog meters load a 100k divider and read low at the midpoint, so check
 * this ratio with the 'b' key against a meter on BATT+ only. */
#define BATT_DIV_RATIO    (2.0f)
#define BATT_START_MIN_MV (3600U)        /* refuse to start below this  */
#define BATT_WARN_MV      (3500U)        /* print a warning once        */
#define BATT_CUT_MV       (3200U)        /* motors off below this ...   */
#define BATT_CUT_S        (1.0f)         /* ... for this long           */
#define BATT_GLITCH_MV    (5000U)        /* impossible for 1S: ignore   */
#define ROUND_HALF        (0.5f)
#define ZERO_F            (0.0f)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * แรงดันแบต (mV) คืน 0 ถ้า ADC ยังไม่มีข้อมูล
 */
uint32_t battery_mv(void)
{
    uint32_t pin = 0U;
    uint32_t out = 0U;

    if (adc_batt_pin_mv(&pin) == true)
    {
        out = (uint32_t)(((float)pin * BATT_DIV_RATIO) + ROUND_HALF);
    }
    else
    {
        /* No action: no data */
    }

    return out;
}

/*
 * พิมพ์แรงดันที่ขาและแรงดันแบต (ปุ่ม 'b' ตอนรอ BOOT) ใช้เทียบกับมิเตอร์
 */
void battery_print(void)
{
    uint32_t pin = 0U;

    if (adc_batt_pin_mv(&pin) == true)
    {
        (void)printf("battery: pin %lu mV -> battery %lu mV (ratio %.3f)\n",
                     (unsigned long)pin, (unsigned long)battery_mv(),
                     (double)BATT_DIV_RATIO);
    }
    else
    {
        (void)printf("battery: no ADC data\n");
    }
}

/*
 * เช็คก่อนเริ่มบิน: false ถ้าแบตต่ำกว่า 3.6 V
 * ถ้า ADC ยังไม่มีข้อมูลจะยอมให้เริ่ม (ADC เสียไม่ควรทำให้บินไม่ได้เลย)
 */
bool battery_ok_to_start(void)
{
    uint32_t bat;
    bool     ok = true;

    bat = battery_mv();
    if ((bat > 0U) && (bat < BATT_START_MIN_MV))
    {
        ok = false;
        (void)printf("battery %lu mV < %u mV\n", (unsigned long)bat,
                     (unsigned int)BATT_START_MIN_MV);
    }
    else
    {
        /* No action */
    }

    return ok;
}

/*
 * ล้างตัวจับเวลาและสถานะเตือน ก่อนเริ่มบินแต่ละรอบ
 */
void battery_monitor_reset(batt_monitor_t *m)
{
    if (m != NULL)
    {
        m->low_s  = ZERO_F;
        m->warned = false;
    }
    else
    {
        /* No action */
    }
}

/*
 * เรียกทุกรอบของ control loop คืน true เมื่อต้องดับมอเตอร์
 * ค่า > 5 V เป็น glitch ของ ADC ไม่นับ และไม่ล้างตัวจับเวลา
 */
bool battery_monitor_update(batt_monitor_t *m, float dt_s)
{
    uint32_t bat;
    bool     cut = false;

    if (m != NULL)
    {
        bat = battery_mv();
        if ((bat > 0U) && (bat < BATT_CUT_MV))
        {
            m->low_s += dt_s;
        }
        else if ((bat > 0U) && (bat < BATT_GLITCH_MV))
        {
            m->low_s = ZERO_F;
        }
        else
        {
            /* No action: no data or ADC glitch - keep the timer */
        }

        if ((bat > 0U) && (bat < BATT_WARN_MV) && (m->warned == false))
        {
            m->warned = true;
            (void)printf("WARNING battery %lu mV - land soon\n", (unsigned long)bat);
        }
        else
        {
            /* No action: fine, or already warned */
        }

        cut = (m->low_s >= BATT_CUT_S);
    }
    else
    {
        /* No action */
    }

    return cut;
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/
