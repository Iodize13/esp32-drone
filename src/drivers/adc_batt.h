/**
 * adc_batt.h - battery voltage sense on ADC1 via continuous DMA.
 *
 * Driver layer: hardware only. Returns the voltage at the ADC pin; the
 * divider ratio that turns it into battery voltage belongs to the app.
 */
#ifndef ADC_BATT_H
#define ADC_BATT_H

#include <stdbool.h>
#include <stdint.h>

/* Start the ADC: DMA fills frames in the background and an ISR
 * averages each finished frame. Returns false if any step failed. */
bool adc_batt_init(void);

/* Latest frame average at the pin, in millivolts (calibrated if the
 * chip has eFuse calibration). Returns false if no frame has arrived
 * yet. Task context only. */
bool adc_batt_pin_mv(uint32_t *mv);

#endif /* ADC_BATT_H */
