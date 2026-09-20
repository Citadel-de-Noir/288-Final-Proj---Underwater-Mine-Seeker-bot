/*
 * adc.c
 *  Created on: Mar 24, 2026
 *  Author: @amotwofour / aryanm@iastate.edu
*/

#include "./include/adc.h"
#include <math.h>

#define SAMPLE_SIZE 16

/*
 * calibration instructions
 *
 * step 1: collect data
 *   - place bot at each distance from a flat wall
 *   - record the raw value from the LCD into a spreadsheet (ignore d: for now)
 *   - distances (cm): 9, 11, 13, 15, 17, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 50
 *
 *   spreadsheet:
 *     col A: distance (cm) -- fill in ahead of time
 *     col B: raw ADC value -- record from LCD at each distance
 *
 * step 2: get equation from excel
 *   - select both columns -> insert -> scatter chart
 *   - right click data point -> add trendline
 *   - try polynomial order 2 and power, check "display equation" and "display R²"
 *   - use whichever has R² closest to 1.0
 *     polynomial: y = Ax^2 + Bx + C  ->  update A, B, C below
 *     power:      y = A / x^B         ->  update A, B below, change rawToDistance()
 *
 * step 3: verify
 *   - check that d: on LCD is within 2 cm of actual distance at a few points
 */

// update these after Excel curve fit

#define B -1.881
#define C 0.0

void adc_init() {
    SYSCTL_RCGCADC_R |= 0x01; // enable ADC0 clock
    SYSCTL_RCGCGPIO_R |= 0x02; // enable port B clock
    while ((SYSCTL_PRGPIO_R & 0x02) != 0x02); // wait for port B ready

    GPIO_PORTB_DIR_R &= ~0x10; // set PB4 as input
    GPIO_PORTB_AFSEL_R |= 0x10; // enable alternate function on PB4
    GPIO_PORTB_DEN_R &= ~0x10; // disable digital I/O on PB4
    GPIO_PORTB_AMSEL_R |= 0x10; // enable analog on PB4

    while ((SYSCTL_PRADC_R & 0x01) != 0x01);
    ADC0_PC_R = 0x01;
    ADC0_SSPRI_R &= 0x3210;
    ADC0_SSPRI_R |= 0x3210;

    ADC0_ACTSS_R &= ~0x0001; // disable SS0 before configuring
    ADC0_EMUX_R &= ~0x000F; // set SS0 to software trigger

    ADC0_SSMUX0_R &= ~0x000F; // clear SS0 channel
    ADC0_SSMUX0_R += 10; // set channel 10 (PB4)

    ADC0_SSCTL0_R &= 0x0006; // clear SS0 control

    ADC0_SSCTL0_R |= 0x0006; // set IE0 and END0

    ADC0_ACTSS_R |= 0x0001; // enable SS0
}

uint16_t adc_read_single() {
    ADC0_PSSI_R |= 0x01;
    while ((ADC0_RIS_R & 0x01) == 0);
    ADC0_ISC_R = 0x01;
    return ADC0_SSFIFO0_R;
}

uint16_t adc_read() {
    uint32_t sum = 0;
    int i;
    for (i = 0; i < SAMPLE_SIZE; i++) {
        sum += adc_read_single();
    }
    return sum / SAMPLE_SIZE;
}

uint16_t rawToDistance(uint16_t raw) {
    return (uint16_t)(((2 * pow(10, 7)) * pow(raw, B)));
}
