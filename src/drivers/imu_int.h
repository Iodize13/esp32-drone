/**
 * imu_int.h - MPU6500 data-ready interrupt (EXTI) on a GPIO.
 *
 * Driver layer: the IMU pulses INT once per sample (1 kHz); the GPIO
 * ISR gives a semaphore that the control loop waits on, so the loop
 * runs in step with the sensor instead of on a software timer.
 */
#ifndef IMU_INT_H
#define IMU_INT_H

#include <stdbool.h>
#include <stdint.h>

/* Configure the GPIO as a rising-edge interrupt. False on any error. */
bool imu_int_init(void);

/* Block until the next data-ready edge or the timeout. True on an edge,
 * false on timeout (INT not wired, or the IMU stopped). */
bool imu_int_wait(uint32_t timeout_ms);

/* Edges seen by the ISR since boot. */
uint32_t imu_int_count(void);

#endif /* IMU_INT_H */
