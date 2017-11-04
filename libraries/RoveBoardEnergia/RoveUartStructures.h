#ifndef ROVEUARTSTRUCTURES_H_
#define ROVEUARTSTRUCTURES_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ROVE_UART_ERROR_SUCCESS = 0,
    ROVE_UART_ERROR_UNKNOWN = -1
} roveUart_ERROR;

typedef struct roveUART_Handle
{
	unsigned int uart_index;
	bool initialized;

	roveUART_Handle()
	{
	  initialized = false;
	}
} roveUART_Handle;

#endif
