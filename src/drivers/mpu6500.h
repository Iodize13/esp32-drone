/*******************************************************************************
 * File Name    : mpu6500.h
 * Description  : Header ของ driver IMU MPU6500 ผ่าน I2C (SDA GPIO8, SCL GPIO9)
 *                ค่าที่ส่งออกเป็นหน่วยจริงแล้ว: accel เป็น g, gyro เป็น dps
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef MPU6500_H
#define MPU6500_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/
#define MPU_AXES          (3U)           /* x, y, z */

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
bool mpu6500_init(void);
bool mpu6500_read_all(float acc_g[MPU_AXES], float gyro_dps[MPU_AXES]);
bool mpu6500_read_gyro(float gyro_dps[MPU_AXES], float *az_g);
bool mpu6500_clear_int(void);

#endif /* MPU6500_H */
