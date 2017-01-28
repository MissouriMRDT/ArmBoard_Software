// RoveBoard.cpp for Energia
// Author: Gbenga Osibodu

#include "RoveBoard.h"

roveUART_Handle roveBoard_UART_open(unsigned int uart_index, unsigned int baud_rate) {
  HardwareSerial* uartArray[8] = {&Serial , &Serial1, &Serial2, &Serial3,
                                  &Serial4, &Serial5, &Serial6, &Serial7};
  
  uartArray[uart_index] -> begin(baud_rate);
  return uartArray[uart_index];
}

roveBoard_ERROR roveBoard_UART_write(roveUART_Handle uart, void* write_buffer, size_t bytes_to_write) {
  uart -> write((uint8_t*)write_buffer, bytes_to_write);
  uart -> flush();
    
  return ROVE_BOARD_ERROR_SUCCESS;
}

roveBoard_ERROR roveBoard_UART_read(roveUART_Handle uart, void* read_buffer, size_t bytes_to_read) {
  char* temp; //Serial.print("read#: "); Serial.println(bytes_to_read);

  if (bytes_to_read == 0) {
    return ROVE_BOARD_ERROR_SUCCESS;
  }
  
  if (read_buffer == NULL) {
    char trash[bytes_to_read];
    temp = trash;
  } else {
    temp = (char*)read_buffer;
  }

  for (int i =0; i<bytes_to_read; i++) {
    while(uart -> available() == 0);
    temp[i] = uart -> read();//Serial.println(temp[i],DEC);
  }
  
  return ROVE_BOARD_ERROR_SUCCESS;
} 

bool roveBoard_UART_available(roveUART_Handle uart) {
  if (uart -> available() > 0) {
    return true;
  } else {
    return false;
  }
}

void wait(int micros) {
  delayMicroseconds(micros);
}
