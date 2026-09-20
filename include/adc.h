#ifndef ADC_H_
#define ADC_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inc/tm4c123gh6pm.h>
#include "./include/Timer.h"

uint16_t adc_read();
uint16_t adc_read_single();
uint16_t rawToDistance(uint16_t raw);
void adc_init();

#endif /* ADC_H_ */