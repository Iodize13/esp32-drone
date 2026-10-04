/**
 * mixer.c - throttle + roll/pitch/yaw corrections -> 4 motor duties.
 *
 * Desaturation: when the mix pushes a motor above max_duty (or below
 * idle_duty), every motor is shifted by the same amount. That keeps the
 * difference between motors - the part that rights the frame - and
 * gives up a little throttle instead. Only if the corrections alone need
 * more than the whole idle..max range are individual motors clipped.
 */

#include <stddef.h>
#include "mixer.h"

#define ZERO_F  (0.0f)

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
            hi = -1.0e9f;
            lo = 1.0e9f;

            for (m = 0U; m < MIXER_MOTORS; m++)
            {
                mix[m] = (float)throttle
                       + ((float)cfg->roll[m]  * roll)
                       + ((float)cfg->pitch[m] * pitch)
                       + ((float)cfg->yaw[m]   * yaw);

                if (mix[m] > hi) { hi = mix[m]; } else { /* keep */ }
                if (mix[m] < lo) { lo = mix[m]; } else { /* keep */ }
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

                duty_out[m] = (uint16_t)(mix[m] + 0.5f);   /* round */
            }
        }
    }
    else
    {
        /* invalid arguments - nothing written */
    }
}
