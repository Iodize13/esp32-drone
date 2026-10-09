/*******************************************************************************
 * File Name    : button.h
 * Description  : Header ของ driver ปุ่ม BOOT (GPIO0, active-low)
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef BUTTON_H
#define BUTTON_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
void button_init(void);
bool button_pressed(void);
void button_wait_release(void);

#endif /* BUTTON_H */
