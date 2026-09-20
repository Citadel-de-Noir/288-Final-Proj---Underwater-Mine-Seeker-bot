/*
 * uart-interrupt.c
 *
 *  Created on: Mar 11, 2026
 *      Author: aryanm
 */

/*
 * @date 04/27/2026
 * Updated for Naval Seeker Final Project
 * @author sayonsn
 */

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "./include/uart_interrupt.h"
#include "./include/timer.h"

#define COMMAND_DEBOUNCE_MS 250

// These variables are declared as examples for your use in the interrupt handler.
volatile char command_byte = 'g';    // byte value for special character used as a command
volatile int command_flag_start = 0; // flag to tell the main program a special command was received
volatile int command_flag_stop = 0;
volatile int command_flag_reset = 0; // flag to tell the main program a special command was received
volatile char char_received = '\0';

volatile int manual_flag = 0;
volatile int auto_flag = 0;
volatile int stop_flag = 0;
volatile int start_flag = 0;
volatile int initiate_scan = 0;
volatile int scan_active = 0;

volatile unsigned int move_timer = 0;

volatile char command_status_byte = '\0';
volatile char move_byte = '\0';

static volatile unsigned int command_timer = 0;
static volatile char last_command_byte = '\0';

static char normalize_command_byte(char byte)
{
    if (byte >= 'A' && byte <= 'Z')
    {
        return byte + ('a' - 'A');
    }

    return byte;
}

static int is_debounced_command(char byte)
{
    return byte == 'm' || byte == 'n' || byte == 'z' || byte == 'x' || byte == 'i';
}

void uart_interrupt_init(void)
{
    // TODO
    // enable clock to GPIO port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

    // enable clock to UART1
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;

    // wait for GPIOB and UART1 peripherals to be ready
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0)
    {
    };
    while ((SYSCTL_PRUART_R & SYSCTL_PRUART_R1) == 0)
    {
    };

    // enable digital functionality on port B pins
    GPIO_PORTB_DEN_R |= 0x03;

    // enable alternate functions on port B pins
    GPIO_PORTB_AFSEL_R |= 0x03;

    // enable UART1 Rx and Tx on port B pins
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) | 0x00000011;

    // calculate baud rate
    uint16_t iBRD = 8;  // use equations
    uint16_t fBRD = 44; // use equations

    // turn off UART1 while setting it up
    UART1_CTL_R &= ~0x01;

    // set baud rate
    // note: to take effect, there must be a write to LCRH after these assignments
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    // set frame, 8 data bits, 1 stop bit, no parity, no FIFO
    // note: this write to LCRH must be after the BRD assignments
    UART1_LCRH_R = 0x60;

    // use system clock as source
    // note from the datasheet UARTCCC register description:
    // field is 0 (system clock) by default on reset
    // Good to be explicit in your code
    UART1_CC_R = 0x0;

    //////Enable interrupts

    // first clear RX interrupt flag (clear by writing 1 to ICR)
    UART1_ICR_R |= 0b00010000;

    // enable RX raw interrupts in interrupt mask register
    UART1_IM_R |= 0b00010000;

    // NVIC setup: set priority of UART1 interrupt to 1 in bits 21-23
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

    // NVIC setup: enable interrupt for UART1, IRQ #6, set bit 6
    NVIC_EN0_R |= 0x40;

    // tell CPU to use ISR handler for UART1 (see interrupt.h file)
    // from system header file: #define INT_UART1 22
    IntRegister(INT_UART1, UART1_Handler);

    // globally allow CPU to service interrupts (see interrupt.h file)
    IntMasterEnable();

    // re-enable UART1 and also enable RX, TX (three bits)
    // note from the datasheet UARTCTL register descriptio n:
    // RX and TX are enabled by default on reset
    // Good to be explicit in your code
    // Be careful to not clear RX and TX enable bits
    //(either preserve if already set or set them)
    UART1_CTL_R = 0x301;
}

void uart_sendChar(char data)
{
    // TODO
    while ((UART1_FR_R & 0x20) != 0)
        ; // wait until there is space in the FIFO (TXFF flag is 0)
    UART1_DR_R = data;
}

char uart_receive(void)
{
    // DO NOT USE this busy-wait function if using RX interrupt
}

void uart_sendStr(const char *data)
{
    // TODO for reference see lcd_puts from lcd.c file
    while (*data != '\0') // loop until null character
    {
        uart_sendChar(*data); // send the current character
        data++;               // move to the next character in the string
    }
}

// Interrupt handler for receive interrupts
void UART1_Handler(void)
{
    char byte_received;
    // check if handler called due to RX event
    if ((UART1_MIS_R & 0x10) == 0x10) // check RX interrupt status (RXMIS bit in MIS register)
    {
        // byte was received in the UART data register
        // clear the RX trigger flag (clear by writing 1 to ICR)
        UART1_ICR_R |= 0b00010000;

        // read the byte received from UART1_DR_R and echo it back to PuTTY
        // ignore the error bits in UART1_DR_R
        byte_received = (char)(UART1_DR_R & 0xFF);
        uart_sendChar(byte_received);

        // if byte received is a carriage return
        if (byte_received == '\r')
        {
            // send a newline character back to PuTTY
            uart_sendChar('\n');
        }
        else
        {
            char command = normalize_command_byte(byte_received);

            if (is_debounced_command(command))
            {
                unsigned int now = timer_getMillis();

                if (command == last_command_byte &&
                    (now - command_timer) < COMMAND_DEBOUNCE_MS)
                {
                    return;
                }

                last_command_byte = command;
                command_timer = now;
            }

            // AS NEEDED
            // code to handle any other special characters
            // code to update global shared variables
            // DO NOT PUT TIME-CONSUMING CODE IN AN ISR
            /*
            if (byte_received == 'g') {
                command_flag_start = 1;
            }
            if (byte_received == 's') {
                command_flag_stop = 1;
            }
            if (byte_received == 'r') {
                command_flag_reset = 1;
            }*/
            if (command == 'm' && !manual_flag)
            {
                manual_flag = 1;
                auto_flag = 0;
                start_flag = 0;
                stop_flag = 0;
                initiate_scan = 0;
                move_byte = '\0';
                command_status_byte = 'm';
            }
            else if (command == 'n' && !auto_flag)
            {
                auto_flag = 1;
                manual_flag = 0;
                start_flag = 0;
                stop_flag = 0;
                initiate_scan = 0;
                move_byte = '\0';
                command_status_byte = 'n';
            }
            else if (command == 'z')
            {
                if (auto_flag && !start_flag)
                {
                    start_flag = 1;
                    stop_flag = 0;
                    command_status_byte = 'z';
                }
            }
            else if (command == 'x')
            {
                if (manual_flag || auto_flag || start_flag || initiate_scan || scan_active || move_byte != '\0')
                {
                    stop_flag = 1;
                    start_flag = 0;
                    initiate_scan = 0;
                    move_byte = '\0';
                    command_status_byte = 'x';
                }
            }
            else if (command == 'i')
            {
                if (manual_flag && !scan_active && !initiate_scan)
                {
                    initiate_scan = 1;
                    move_byte = '\0';
                    command_status_byte = 'i';
                }
            }
            else if (manual_flag)
            {
                switch (byte_received)
                {
                case 'w':
                case 'W':
                    move_byte = 'w';
                    move_timer = timer_getMillis(); // example: set move_timer to 500 ms when 'w' is received
                    break;
                case 'a':
                case 'A':
                    move_byte = 'a';
                    move_timer = timer_getMillis();
                    break;
                case 's':
                case 'S':
                    move_byte = 's';
                    move_timer = timer_getMillis();
                    break;
                case 'd':
                case 'D':
                    move_byte = 'd';
                    move_timer = timer_getMillis();
                    break;
                case 'q':
                case 'Q':
                    move_byte = '\0';
                    break;
                default:
                    break;
                }
            }
        }
    }
}
