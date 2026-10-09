/*******************************************************************************
 * File Name    : attitude_est.h
 * Description  : Header ของ attitude estimator: calibrate gyro bias แล้ว
 *                อ่าน IMU -> complementary filter -> มุม roll/pitch
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef ATTITUDE_EST_H
#define ATTITUDE_EST_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "mpu6500.h"

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/
#define ATT_EST_SETTLE_MS (500U)         /* let the angle settle from accel */
#define ATT_EST_DT_S      (0.001f)       /* nominal 1 kHz step */

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
bool att_est_calibrate(void);
bool att_est_update(float dt_s, float rate_dps[MPU_AXES]);
bool att_est_run(uint32_t ms);
float att_est_roll(void);
float att_est_pitch(void);

#endif /* ATTITUDE_EST_H */
