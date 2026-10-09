/*******************************************************************************
 * File Name    : motor_pwm.h
 * Description  : Header ของ driver PWM มอเตอร์ 4 ตัว (LEDC, 20 kHz)
 *                duty เป็น per-mille 0-1000 ทุกฟังก์ชัน
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/
#define MOTOR_COUNT       (4U)
#define MOTOR_DUTY_FULL   (1000U)        /* per-mille = 100 % */

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
void motor_pwm_init(void);
void motor_pwm_set_one(uint32_t idx, uint32_t duty);
void motor_pwm_set_all(uint32_t duty);
void motor_pwm_stop_all(void);

#endif /* MOTOR_PWM_H */
