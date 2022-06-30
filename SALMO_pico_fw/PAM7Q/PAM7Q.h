#ifndef _PAM7Q_H_
#define _PAM7Q_H_

/* Includes */
#include <stdbool.h>
#include <stdint.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"

// UART CONFIG
#define GPS_UART_ID uart1
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

void PAM7Q_init(uint8_t txpin, uint8_t rxpin);

#endif