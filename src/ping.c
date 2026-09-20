/**
 * Driver for ping sensor
 * @file ping.c
 * @author sayonsn@iastate.edu
 */

#include "./include/ping.h"
#include "./include/Timer.h"

// Global shared variables
// Use extern declarations in the header file

volatile uint32_t g_start_time;
volatile uint32_t g_end_time;
volatile uint32_t g_overflow_count;
volatile uint32_t pulse_cycles;
volatile bool overflow = false;
volatile float distance_cm;
//volatile enum{LOW, HIGH, DONE} g_state = LOW; // State of ping echo pulse
volatile state_t g_state = LOW;

#define soundSpd 34300.0 // cm/s

void ping_init(void)
{

    SYSCTL_RCGCTIMER_R |= 0x08;
    SYSCTL_RCGCGPIO_R |= 0x02;

    while ((SYSCTL_PRGPIO_R & 0x02) != 0x02)
    {
    };
    while ((SYSCTL_PRTIMER_R & 0x08) != 0x08)
    {
    };

    GPIO_PORTB_AFSEL_R &= ~0x08;  // disable alternate function on PB3
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF0FFF) | 0x00007000;
    GPIO_PORTB_DIR_R |= 0x08;     // PB3 = output
    GPIO_PORTB_DEN_R |= 0x08;     // keep digital enable
    GPIO_PORTB_DATA_R &= ~0x08;   // PB3 starts LOW

    NVIC_EN1_R |= 0x10;
    NVIC_PRI9_R = (NVIC_PRI9_R & 0xFFFFFF0F) | 0x00000020;

    IntRegister(INT_TIMER3B, TIMER3B_Handler);
    IntMasterEnable();

    // Configure and enable the timer
    TIMER3_CTL_R &= ~0x100;            // TBEN = 0 (disable Timer 3B for setup)
    TIMER3_CFG_R = 0x4;         // 16-bit timer configuration...half-width mode
    TIMER3_TBMR_R |= 0x07;
    TIMER3_TBMR_R &= ~0x10;
    // Setting up timer b mode register...bits 1 & 0 input edge time mode, bit 2 = 0 for edgetime, bit 3 = 0 for count down
    TIMER3_CTL_R |= 0xC00; // Set up to capture on both edges
    TIMER3_TAILR_R = 0xFFFF; // loading interval load register with 0xFFFF for 16 bit mode
    TIMER3_TBPR_R = 0xFF; // loading prescaler to 0xFF for 8 bit prescaler extension, gives us a 24 bit effective counter
    TIMER3_IMR_R |= 0x400; // CBEIM = 1 (enable input capture interrupt for timer b
    TIMER3_ICR_R |= 0x400;
}

void ping_trigger(void)
{
    g_state = LOW;
    // Disable timer and disable timer interrupt
    TIMER3_CTL_R &= ~0x0100;    // TBEN = 0 (disable Timer 3B)
    TIMER3_IMR_R &= ~0x0400; // CBEIM = 0 (disable input capture interrupt for timer b)
    // Disable alternate function (disconnect timer from port pin)
    GPIO_PORTB_AFSEL_R &= ~0x08;
    GPIO_PORTB_DIR_R |= 0x08;
    GPIO_PORTB_DEN_R |= 0x08;

    // YOUR CODE HERE FOR PING TRIGGER/START PULSE

    GPIO_PORTB_DATA_R |= 0x08;     // PB3 = HIGH (rising edge of trigger)
    timer_waitMicros(5);     // hold high >=5 us (datasheet: 2 us min, 5 us typ)
    GPIO_PORTB_DATA_R &= ~0x08;     // PB3 = LOW  (falling edge; PING starts burst)

    // Clear an interrupt that may have been erroneously triggered
    TIMER3_ICR_R |= 0X0400;         // write 1 to CBECINT to clear capture interrupt
    // Re-enable alternate function, timer interrupt, and timer
    GPIO_PORTB_AFSEL_R |= 0X08;     // enable alternate function on PB3
    TIMER3_IMR_R |= 0x0400;         // unmask CBEIM (enable input capture interrupt for timer b)
    TIMER3_CTL_R |= 0x0100;         // TBEN = 1 (enable Timer 3B)
}

void TIMER3B_Handler(void)
{

    // YOUR CODE HERE
    // As needed, go back to review your interrupt handler code for the UART lab.
    // What are the first lines of code in the ISR? Regardless of the device, interrupt handling
    // includes checking the source of the interrupt and clearing the interrupt status bit.
    // Checking the source: test the MIS bit in the MIS register (is the ISR executing
    // because the input capture event happened and interrupts were enabled for that event?
    // Clearing the interrupt: set the ICR bit (so that same event doesn't trigger another interrupt)
    // The rest of the code in the ISR depends on actions needed when the event happens.

    if (TIMER3_MIS_R & 0x0400)
    {
        TIMER3_ICR_R |= 0x0400;    // write 1 to CBECINT to clear interrupt

        uint32_t captured = TIMER3_TBR_R & 0x00FFFFFF; // read the 24 bit captured counter value from GPTMTBR

        if (g_state == LOW)
        {
            //this is where start edge is detected and time is saved
            g_start_time = captured;
            g_state = HIGH;
        }
        else if (g_state == HIGH)
        {
            //this is where end edge is detected and time is saved
            g_end_time = captured;
            g_state = DONE;
        }
    }

}

float ping_getDistance(void)
{

    // YOUR CODE HERE

    ping_trigger(); // trigger the sensor

    uint32_t timeout_ms = timer_getMillis() + 20; // timeout after 20 ms (PING max echo = 18.5 ms)
    while (g_state != DONE)
    {
        if (timer_getMillis() > timeout_ms)
        {
            return -1.0; // sensor timed out (nothing in range or malfunction)
        }
    }

    //uint32_t pulse_cycles;
    bool overflow = false;

    if (g_start_time >= g_end_time)
    {
        pulse_cycles = g_start_time - g_end_time;
    }
    else
    {
        //pulse_cycles = -1 * (g_start_time - g_end_time);
        pulse_cycles = g_start_time + (0x100000 - g_end_time);
        overflow = true;
        g_overflow_count++;
    }

    float pulse_ms = ((float)pulse_cycles / 16000000.0 )* 1000.0; // Convert cycles to ms
    float distance_cm = pulse_ms * 34.3f / 2.0f;
    //(void) overflow;
    return distance_cm;

}
