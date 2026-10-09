/*******************************************************************************
 * File Name    : adc_batt.h
 * Description  : Header ของ driver วัดแรงดันแบต (ADC1 + DMA ที่ GPIO1)
 *                driver คืนแรงดันที่ขา ADC เท่านั้น อัตราส่วน divider เป็น
 *                หน้าที่ของ app layer
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef ADC_BATT_H
#define ADC_BATT_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/

/* Exported define/macro/constants --------------------------------------------*/

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
bool adc_batt_init(void);
bool adc_batt_pin_mv(uint32_t *mv);

#endif /* ADC_BATT_H */
