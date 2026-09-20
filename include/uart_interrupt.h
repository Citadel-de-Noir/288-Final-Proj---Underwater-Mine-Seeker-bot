/*
 *
 *   uart-interrupt.h
 *
 *   Used to set up the RS232 connector and WIFI module
 *   Uses RX interrupt
 *   Functions for communicating between CyBot and PC via UART1
 *   Serial parameters: Baud = 115200, 8 data bits, 1 stop bit,
 *   no parity, no flow control on COM1, FIFOs disabled on UART1
 *
 *   @author Dane Larson
 *   @date 07/18/2016
 *   Phillip Jones updated 9/2019, removed WiFi.h, Timer.h
 *   Diane Rover updated 2/2020, added interrupt code
 */

/*
 * @date 04/27/2026
 * Updated for Naval Seeker Final Project
 * @author sayonsn
 */

#ifndef UART_H_
#define UART_H_

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"

// Notice that interrupt.h provides library function prototypes for IntMasterEnable() and IntRegister()

// The following externals are global variables defined in uart-interrupt.c for use with the interrupt handler.
// Using extern here, the global variables become visible to other c files that include uart-interrupt.h
// Extern does not allocate storage for a variable. It tells the compiler that the variable is defined in another file.
// extern volatile char receive_buffer[]; // buffer for characters received from PuTTY
// extern volatile int receive_index; // index to keep track of characters in buffer
extern volatile char command_byte;      // byte value for special character used as a command
extern volatile int command_flag_start; // flag to tell the main program a special command was received
extern volatile int command_flag_stop;
extern volatile char char_received;
extern volatile int command_flag_reset;

extern volatile int manual_flag;
extern volatile int auto_flag;
extern volatile int stop_flag;
extern volatile int start_flag;
extern volatile int initiate_scan;
extern volatile int scan_active;
extern volatile char command_status_byte;
extern volatile char move_byte;

extern volatile unsigned int move_timer;

// UART1 device initialization for CyBot to PuTTY
void uart_interrupt_init(void);

// Send a byte over UART1 from CyBot to PuTTY
void uart_sendChar(char data);

// CyBot waits (i.e. blocks) to receive a byte from PuTTY
// returns byte that was received by UART1
// Not used with interrupts; see UART1_Handler
char uart_receive(void);

// Send a string over UART1
// Sends each char in the string one at a time
void uart_sendStr(const char *data);

// Interrupt handler for receive interrupts
void UART1_Handler(void);

#endif /* UART_H_ */
