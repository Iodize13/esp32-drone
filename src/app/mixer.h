/**
 * mixer.h - throttle + roll/pitch/yaw corrections -> 4 motor duties.
 *
 * Application layer: output is per-mille duty (0..1000) for the PWM
 * driver; no hardware access here.
 *
 * Each motor has a sign per axis in mixer_config_t. The signs depend on
 * where each motor sits and how the IMU is mounted, so they must be set
 * from the bench check below - not guessed. Wrong signs flip the drone
 * on the first arm.
 *
 * Mix: duty[i] = throttle + roll[i]*roll + pitch[i]*pitch + yaw[i]*yaw
 * A positive correction must turn the frame toward a positive angle
 * (PID gives a negative correction when the angle is positive).
 *
 * Sign check (props OFF):
 *   roll : tilt the frame by hand until roll_deg reads positive. Motors
 *          on the side that is now HIGHER get roll = +1, the LOWER side
 *          gets -1 (a positive correction speeds up the high side and
 *          tilts further; a negative one lifts the low side back).
 *   pitch: same with pitch_deg.
 *   yaw  : spin the frame by hand so gyro Z reads positive. Motors whose
 *          prop spins the SAME way as that rotation get yaw = -1, the
 *          others +1 (a prop's reaction torque turns the frame the
 *          opposite way to the prop).
 * Verify each sign with the motor test before flying.
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

#define MIXER_MOTORS  (4U)

typedef struct
{
    int8_t   roll[MIXER_MOTORS];    /* +1 / -1 / 0 */
    int8_t   pitch[MIXER_MOTORS];
    int8_t   yaw[MIXER_MOTORS];
    uint16_t max_duty;              /* per-mille ceiling, e.g. 1000      */
    uint16_t idle_duty;             /* per-mille floor while armed       */
} mixer_config_t;

/**
 * throttle      : per-mille 0..max_duty. 0 = motors off (disarmed path).
 * roll/pitch/yaw: PID corrections in per-mille.
 * duty_out      : per-mille per motor.
 *
 * If a correction pushes a motor past the limits, all motors are shifted
 * together so the attitude correction is kept and throttle gives way.
 */
void mixer_mix(const mixer_config_t *cfg, uint16_t throttle,
               float roll, float pitch, float yaw,
               uint16_t duty_out[MIXER_MOTORS]);

#endif /* MIXER_H */
