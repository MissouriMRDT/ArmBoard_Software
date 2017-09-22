#ifndef ROVEUARTTYPENAMES_H_
#define ROVEUARTTYPENAMES_H_

typedef enum {
    ROVE_BOARD_ERROR_SUCCESS = 0,
    ROVE_BOARD_ERROR_UNKNOWN = -1
} roveBoard_ERROR;

typedef struct roveUART_Handle
{
	unsigned int uart_index;
} roveUART_Handle;

#endif
