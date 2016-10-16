// RoveBoard.h for Energia
// Author: Gbenga Osibodu

#ifndef ROVEBOARD_H_
#define ROVEBOARD_H_

#include <Energia.h>

typedef enum {
    ROVE_BOARD_ERROR_SUCCESS = 0,
    ROVE_BOARD_ERROR_UNKNOWN = -1
} roveBoard_ERROR;
typedef HardwareSerial* roveUART_Handle;

roveUART_Handle roveBoard_UART_open(unsigned int uart_index, unsigned int baud_rate);
roveBoard_ERROR roveBoard_UART_write(roveUART_Handle uart, void* write_buffer, size_t bytes_to_write);
roveBoard_ERROR roveBoard_UART_read(roveUART_Handle uart, void* read_buffer, size_t bytes_to_read);
bool roveBoard_UART_available(roveUART_Handle uart);

void wait(int micros);

#endif
