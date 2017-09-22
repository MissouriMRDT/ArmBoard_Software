// RoveBoard.h for Energia
// Author: Gbenga Osibodu
//

#ifndef ROVEBOARD_TM4C1294NCPDT_ROVEBOARD_GENERICS_ROVEUART_H_
#define ROVEBOARD_TM4C1294NCPDT_ROVEBOARD_GENERICS_ROVEUART_H_

#include <stddef.h>
#include "RoveUartTypenames.h"

//settings arguments:
//constants to set word length in transmission in bits
#define WordLength8 0
#define WordLength7 1
#define WordLength6 2
#define WordLength5 3

//constants to set stop bits in transmission
#define OneStopBit 0
#define TwoStopBit 1

//constants to set up parity bits in transmission
#define NoParity 0
#define EvenParity 1
#define OddParity 2
#define AlwaysZero 3
#define AlwaysOne 4

//sets up the specified uart to run at the specified baud rate
//inputs: index of the uart module to run (0 for uart 0, 1 for uart 1...), baud rate in bits/second (max/min ranges are board specific)
//returns: reference to the now setup uart, for using in the other functions
extern roveUART_Handle roveUartOpen(unsigned int uart_index, unsigned int baud_rate);

//writes bytes out on a uart port
//inputs: reference of a setup uart module from roveUartOpen, a pointer to the information to write
//(can be address of a single piece of data, an array, etc), and how many bytes are to be sent.
//returns: Information on how the process went based on roveBoard_ERROR enum
extern roveBoard_ERROR roveUartWrite(roveUART_Handle uart, void* write_buffer, size_t bytes_to_write);

//reads bytes from a uart port
//inputs: reference of a setup uart module from roveUartOpen, a pointer to the buffer to read into
//(can be address of a single piece of data, an array, etc), and how many bytes are to be read.
//returns: Information on how the process went based on roveBoard_ERROR enum
//warning: Blocking, won't return until the uart has that many bytes in its incoming buffer.
extern roveBoard_ERROR roveUartRead(roveUART_Handle uart, void* read_buffer, size_t bytes_to_read);

//reads bytes from a uart port
//inputs: reference of a setup uart module from roveUartOpen, a pointer to the buffer to read into
//(can be address of a single piece of data, an array, etc), and how many bytes are to be read.
//returns: Information on how the process went based on roveBoard_ERROR enum.
//Nonblocking, so if there wasn't that amount of bytes currently in the uart's incoming data buffer it returns error
extern roveBoard_ERROR roveUartReadNonBlocking(roveUART_Handle uart, void* read_buffer, size_t bytes_to_read);

//changes the uart's output settings such as settings about its parity bit, stop bits, and word length
//inputs: reference of a setup uart module from roveUartOpen, and arguments based on the constants defined above
//returns: Information on how the process went based on roveBoard_ERROR enum.
extern roveBoard_ERROR roveUartSettings(roveUART_Handle uart, unsigned int parityBits, unsigned int stopBits, unsigned int wordLength);

//checks how many bytes the uart currently has in its read buffer
//inputs: reference of a setup uart module from roveUartOpen
//returns: How many bytes the uart currently has in its read buffer
extern int roveUartAvailable(roveUART_Handle uart);

//checks what number (a byte) is at the top of the uart's read buffer, without actually taking it out of the buffer
//inputs: reference of a setup uart module from roveUartOpen
//returns: the byte at the top of the uart's read buffer
extern int roveUartPeek(roveUART_Handle uart);

//for deprecated libraries
#define roveBoard_UART_open(x, y)		roveUartOpen(x, y)
#define roveBoard_UART_write(x, y, z)	roveUartWrite(x, y, z)
#define roveBoard_UART_available(x)		roveUartAvailable(x)
#define roveBoard_UART_read(x, y, z)	roveUartRead(x, y, z)

#endif
