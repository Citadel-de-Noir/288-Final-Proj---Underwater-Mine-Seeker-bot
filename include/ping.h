/**
 * Driver for ping sensor
 * @file ping.c
 * @author sayonsn
 */
#ifndef PING_H_
#define PING_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

extern volatile uint32_t g_start_time;   // timer count captured on rising edge
extern volatile uint32_t g_end_time;     // timer count captured on falling edge
extern volatile uint32_t g_overflow_count; // running count of timer overflows
extern volatile uint32_t pulse_cycles;
extern volatile bool overflow;
extern volatile float distance_cm;

typedef enum {LOW, HIGH, DONE} state_t;
extern volatile state_t g_state;


/**
 * Initialize ping sensor. Uses PB3 and Timer 3B
 */
void ping_init (void);

/**
 * @brief Trigger the ping sensor
 */
void ping_trigger (void);

/**
 * @brief Timer3B ping ISR
 */
void TIMER3B_Handler(void);

/**
 * @brief Calculate the distance in cm
 *
 * @return Distance in cm
 */
float ping_getDistance (void);

#endif /* PING_H_ */
