#include "./include/servo.h"
#include "./include/Timer.h"
#include <stdint.h>
#include "./include/button.h"
#include "./include/lcd.h"

#define COUNTS_0        7907
#define COUNTS_180      35107
#define COUNTS_PER_DEG  ((float)(COUNTS_180 - COUNTS_0) / 180.0f)

/* CALIBRATION INSTRUCTIONS
 * 1. call servo_calibrate() in main, comment out servo_move() calls
 * 2. put protractor under servo, align 90 deg with center
 * 3. nudge servo to true 0 deg with buttons:
 *      SW1: +10 counts
 *      SW2: -10 counts
 *      SW3: +100 counts
 *      SW4: -100 counts
 * 4. write down match value on LCD -> set COUNTS_0 to this
 * 5. nudge to true 180 deg, write down match value -> set COUNTS_180 to this
 * 6. update defines at top of file, COUNTS_PER_DEG updates automatically
 * 7. comment out servo_calibrate(), uncomment servo_move() calls, recompile
*/

// initialize servo
void servo_init (void){
    SYSCTL_RCGCTIMER_R |= 0x2; // enable port B timer
    while ((SYSCTL_PRTIMER_R & 0x2) == 0) {} // wait for port B timer to be ready

    // timer init
    TIMER1_CTL_R &= ~0x100; // clear timer ctrl register
    TIMER1_CTL_R |= 0x4000; // invert pwm out signal so that match is low and rest of period is high
    TIMER1_CFG_R |= 0x4; // set config so timer mode reg controls it
    TIMER1_TBMR_R |= 0b1010; // turn on pwm mode
    TIMER1_TBPR_R |= 0x4; // extends TBILR timer with extra prescaler register 3 bits
    TIMER1_TBILR_R |= 0xE200; // set interval load with least sig 16 bits (timer interval load)
    TIMER1_TBPMR_R &= 0x00; // set prescale match register to 0x00 so no default to 0xFF;
    
    // initial position 90 deg
    uint32_t start = COUNTS_0 + (uint32_t)(90 * COUNTS_PER_DEG);
    TIMER1_TBMATCHR_R = start & 0xFFFF;
    TIMER1_TBPMR_R    = (start >> 16) & 0xFF;

    // init PB5
    SYSCTL_RCGCGPIO_R |= 0x2; 
    while((SYSCTL_PRGPIO_R & 0x2) == 0) {}
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R |= 0x700000;
    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;

    TIMER1_CTL_R |= 0x100; // turn on timer
}

void servo_move(uint16_t degrees) {
    if (degrees > 180) degrees = 180;

    uint32_t match = COUNTS_0 + (uint32_t)(degrees * COUNTS_PER_DEG);

    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R    = (match >> 16) & 0xFF;

    timer_waitMillis(90);
}

void servo_calibrate(void) {
    char msgBuffer[90];
    button_init();

    while(1) {
        uint32_t match = ((TIMER1_TBPMR_R & 0xFF) << 16) 
                        | (TIMER1_TBMATCHR_R & 0xFFFF);

        switch(button_getButton()) {
            case 1:
                match += 100;
                timer_waitMillis(200);
                break;
            case 2:
                match -= 100;
                timer_waitMillis(200);
                break;
            case 3:
                match += 400;
                timer_waitMillis(200);
                break;
            case 4:
                match -= 400;
                timer_waitMillis(200);
                break;
        }

        TIMER1_TBMATCHR_R = match & 0xFFFF;
        TIMER1_TBPMR_R    = (match >> 16) & 0xFF;

        sprintf(msgBuffer, "%u", match);  // display current match value
        lcd_printf(msgBuffer);
    }
}
