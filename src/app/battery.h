/*******************************************************************************
 * File Name    : battery.h
 * Description  : Header ของ battery monitor (app layer): แปลงแรงดันขา ADC
 *                เป็นแรงดันแบต และ failsafe แบตต่ำ (1S LiPo)
 * Date         : 2026-10-09
 ******************************************************************************/
#ifndef BATTERY_H
#define BATTERY_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported typedef/enum/struct/union -----------------------------------------*/
typedef struct
{
    float low_s;          /* how long the battery has been below the cut */
    bool  warned;         /* warning printed once already                */
} batt_monitor_t;

/* Exported define/macro/constants --------------------------------------------*/

/* Exported variables -----------------------------------------------------------*/

/* Exported function prototypes ------------------------------------------------*/
uint32_t battery_mv(void);
void battery_print(void);
bool battery_ok_to_start(void);
void battery_monitor_reset(batt_monitor_t *m);
bool battery_monitor_update(batt_monitor_t *m, float dt_s);

#endif /* BATTERY_H */
