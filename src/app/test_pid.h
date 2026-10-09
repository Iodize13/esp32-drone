/*******************************************************************************
 * File Name    : test_pid.h
 * Description  : Header ของ angle PID test บน rig แกนเดียว (หรือผูกเชือก 2 แกน)
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef TEST_PID_H
#define TEST_PID_H

/* Includes ------------------------------------------------------------------*/

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/
/* angle the IMU reads with the frame level; PID holds this as zero */
#define PID_TRIM_ROLL_DEG  (0.0f)
#define PID_TRIM_PITCH_DEG (-2.5f)
#define AX_ROLL            (0U)            /* controlled axes */
#define AX_PITCH           (1U)
#define CTRL_AXES          (2U)

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
void test_pid(void);

#endif /* TEST_PID_H */
