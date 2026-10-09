/*******************************************************************************
 * File Name    : mixer.c
 * Description  : รวม throttle กับแรงแก้ roll/pitch/yaw เป็น duty ของมอเตอร์ 4 ตัว
 *                ถ้ามอเตอร์ตัวไหนเกินช่วง จะเลื่อนทุกตัวเท่ากัน (desaturation) เพื่อรักษา
 *                ส่วนต่างระหว่างมอเตอร์ซึ่งเป็นตัวแก้มุมไว้ ยอมเสีย throttle นิดหน่อยแทน
 * Date         : 2026-10-09
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "mixer.h"

/* Private includes ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ZERO_F  (0.0f)
#define MIX_INIT_HI   (-1.0e9f)
#define MIX_INIT_LO   (1.0e9f)
#define ROUND_HALF    (0.5f)

/* Private macro ------------------------------------------------------------*/

/* Private constants ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

/* External variables ------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------*/

/* Private user code ------------------------------------------------------------*/

/* Public functions ------------------------------------------------------------*/

/*
 * duty[i] = throttle + roll[i]*roll + pitch[i]*pitch + yaw[i]*yaw
 * throttle 0 = disarm: มอเตอร์ดับหมด ไม่สนแรงแก้
 */
void mixer_mix(const mixer_config_t *cfg, uint16_t throttle,
               float roll, float pitch, float yaw,
               uint16_t duty_out[MIXER_MOTORS])
{
    float    mix[MIXER_MOTORS];
    float    hi;
    float    lo;
    float    shift = ZERO_F;
    float    top;
    float    floor_d;
    uint32_t m;

    if ((cfg != NULL) && (duty_out != NULL))
    {
        top     = (float)cfg->max_duty;
        floor_d = (float)cfg->idle_duty;

        if (throttle == 0U)
        {
            /* throttle 0 = disarmed: motors off, corrections ignored */
            for (m = 0U; m < MIXER_MOTORS; m++)
            {
                duty_out[m] = 0U;
            }
        }
        else
        {
            hi = MIX_INIT_HI;
            lo = MIX_INIT_LO;

            for (m = 0U; m < MIXER_MOTORS; m++)
            {
                mix[m] = (float)throttle
                       + ((float)cfg->roll[m]  * roll)
                       + ((float)cfg->pitch[m] * pitch)
                       + ((float)cfg->yaw[m]   * yaw);

                if (mix[m] > hi)
                {
                    hi = mix[m];
                }
                else
                {
                    /* No action */
                }
                if (mix[m] < lo)
                {
                    lo = mix[m];
                }
                else
                {
                    /* No action */
                }
            }

            if (hi > top)
            {
                shift = top - hi;            /* pull everyone down */
            }
            else if (lo < floor_d)
            {
                shift = floor_d - lo;        /* push everyone up   */
            }
            else
            {
                /* already inside the range */
            }

            for (m = 0U; m < MIXER_MOTORS; m++)
            {
                mix[m] += shift;

                if (mix[m] > top)
                {
                    mix[m] = top;
                }
                else if (mix[m] < floor_d)
                {
                    mix[m] = floor_d;
                }
                else
                {
                    /* in range */
                }

                duty_out[m] = (uint16_t)(mix[m] + ROUND_HALF);   /* round */
            }
        }
    }
    else
    {
        /* invalid arguments - nothing written */
    }
}

/* Callback functions ------------------------------------------------------------*/

/* Private functions ------------------------------------------------------------*/
