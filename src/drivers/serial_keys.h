/*******************************************************************************
 * File Name    : serial_keys.h
 * Description  : Header ของ driver รับปุ่มจาก serial (UART0 RX, interrupt)
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef SERIAL_KEYS_H
#define SERIAL_KEYS_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
bool serial_keys_init(void);
bool serial_keys_get(uint8_t *key);
void serial_keys_flush(void);

#endif /* SERIAL_KEYS_H */
