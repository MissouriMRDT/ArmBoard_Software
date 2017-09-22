/*
 * RoveUartEnergia_TivaTM4C1294NCPDT.h
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEUARTENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEUARTENERGIA_TIVATM4C1294NCPDT_H_

#include <stddef.h>
#include "RoveUartTypenames.h"
#include "Energia.h"

roveUART_Handle roveUartOpen(unsigned int uart_index, unsigned int baud_rate);
roveBoard_ERROR roveUartWrite(roveUART_Handle uart, void* write_buffer, size_t bytes_to_write);
roveBoard_ERROR roveUartRead(roveUART_Handle uart, void* read_buffer, size_t bytes_to_read);
roveBoard_ERROR roveUartReadNonBlocking(roveUART_Handle uart, void* read_buffer, size_t bytes_to_read);
roveBoard_ERROR roveUartSettings(roveUART_Handle uart, unsigned int parityBits, unsigned int stopBits, unsigned int wordLength);
int roveUartAvailable(roveUART_Handle uart);
int roveUartPeek(roveUART_Handle uart);

#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_ROVEUARTENERGIA_TIVATM4C1294NCPDT_H_ */
