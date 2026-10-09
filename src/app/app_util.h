/*******************************************************************************
 * File Name    : app_util.h
 * Description  : Header ของฟังก์ชันช่วยที่ทุกการทดสอบใช้ร่วมกัน
 *                (รอแบบกด BOOT ยกเลิกได้, นับถอยหลัง, ค่าสัมบูรณ์ float)
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef APP_UTIL_H
#define APP_UTIL_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/
#define MS_PER_S          (1000U)
#define US_PER_MS         (1000)

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
bool app_wait_abortable(uint32_t ms);
bool app_countdown(void);
float app_absf(float x);

#endif /* APP_UTIL_H */
