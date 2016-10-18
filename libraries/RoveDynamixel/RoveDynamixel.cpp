// RoveDynamixel.cpp
// Author: Gbenga Osibodu

#include "RoveDynamixel.h"

void DynamixelInit(Dynamixel* dyna, DynamixelType type, uint8_t id, uint8_t uartIndex, int baud) {
  dyna -> type = type;
  dyna -> id = id;
  dyna -> uart = roveBoard_UART_open(uartIndex, baud);
  wait(5000);
}

void DynamixelSendPacket(Dynamixel dyna, uint8_t length, uint8_t* instruction) {
  int i;
  uint8_t checksum;
  
  checksum = dyna.id + (length + 1);
  for(i=0; i < length; i++) {
    checksum += instruction[i];
  }
  checksum = ~checksum;
  
  uint8_t packet[length + 5];
  
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = dyna.id;
  packet[3] = length + 1;
  memcpy(&(packet[4]), instruction, length);
  packet[length + 4] = checksum;
  
  roveBoard_UART_write(dyna.uart, packet, length + 5);
  wait(600);
  roveBoard_UART_read(dyna.uart, NULL, length + 5);
}

uint8_t DynamixelGetReturnPacket(Dynamixel dyna, uint8_t* data, size_t dataSize) {
  // To be fixed
  uint8_t id, length, error = 0;
  uint8_t temp1, temp2;
  while(roveBoard_UART_available(dyna.uart) == true)
    roveBoard_UART_read(dyna.uart, NULL, 1);
  return 0;
  
  if(roveBoard_UART_available(dyna.uart) == true){
    roveBoard_UART_read(dyna.uart, &temp2, 1);
    
    while(roveBoard_UART_available(dyna.uart) == true) { 
      temp1 = temp2;Serial.println("Check");
      roveBoard_UART_read(dyna.uart, &temp2, 1);
      if (temp1 == 255 && temp2 == 255) {
        if(roveBoard_UART_available(dyna.uart) == true)
          roveBoard_UART_read(dyna.uart, &id, 1); 
        else return DYNAMIXEL_ERROR_UNKNOWN;Serial.println("ID");
        if(roveBoard_UART_available(dyna.uart) == true)
          roveBoard_UART_read(dyna.uart, &length, 1); 
        else return DYNAMIXEL_ERROR_UNKNOWN;Serial.println("Length");
        if (length > 0 && roveBoard_UART_available(dyna.uart) == true)
          roveBoard_UART_read(dyna.uart, &error, 1); 
        else return DYNAMIXEL_ERROR_UNKNOWN;Serial.println("Error");
        if (dataSize + 2 != length) {Serial.println("Bad");
          //roveBoard_UART_read(dyna.uart, NULL, length-2);
          return (error & DYNAMIXEL_ERROR_UNKNOWN);
        } else {Serial.println("Good");
          roveBoard_UART_read(dyna.uart, data, length-2);
          roveBoard_UART_read(dyna.uart, NULL, 1);Serial.println("End");
          return error;
        }
      }
    }
  }
  return DYNAMIXEL_ERROR_UNKNOWN;
}

uint8_t DynamixelGetError(Dynamixel dyna) {
  return DynamixelGetReturnPacket(dyna, NULL, 0);
}

uint8_t DynamixelPing(Dynamixel dyna) {
  uint8_t msgLength = 1;
  uint8_t data = DYNAMIXEL_PING;
  
  DynamixelSendPacket(dyna, msgLength, &data);
  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

void DynamixelSendWriteCommand(Dynamixel dyna, uint8_t dynamixelRegister, uint8_t dataLength, uint8_t* data) {
  uint8_t buffer[dataLength + 2];

  buffer[0] = DYNAMIXEL_WRITE_DATA;
  buffer[1] = dynamixelRegister;
  memcpy(&(buffer[2]), data, dataLength);

  DynamixelSendPacket(dyna, dataLength + 2, buffer);
}

void DynamixelSendReadCommand(Dynamixel dyna, uint8_t dynamixelRegister, uint8_t readLength) {
  uint8_t buffer[3];

  buffer[0] = DYNAMIXEL_READ_DATA;
  buffer[1] = dynamixelRegister;
  buffer[2] = readLength;

  DynamixelSendPacket(dyna, 3, buffer);
}

uint8_t DynamixelRotateJoint(Dynamixel dyna, uint16_t position) {
  uint8_t msgLength = 2;
  uint8_t data[msgLength];
  
  data[0] = position & 0x00FF;
  data[1] = position >> 8;
  
  DynamixelSendWriteCommand(dyna, DYNAMIXEL_GOAL_POSITION_L, msgLength, data);
  
  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

uint8_t DynamixelSpinWheel(Dynamixel dyna, uint16_t speed) {
  uint8_t msgLength = 2;
  uint8_t data[msgLength];
  
  data[0] = speed & 0x00FF;
  data[1] = speed >> 8;
  
  DynamixelSendWriteCommand(dyna, DYNAMIXEL_MOVING_SPEED_L, msgLength, data);
  
  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

uint8_t DynamixelSetId(Dynamixel* dyna, uint8_t id) {
  uint8_t msgLength = 1;
  
  DynamixelSendWriteCommand(*dyna, DYNAMIXEL_ID, msgLength, &id);
  
  dyna -> id = id;
  
  wait(TXDELAY);
  return DynamixelGetError(*dyna);
}

uint8_t DynamixelSetBaudRate(Dynamixel dyna, uint8_t baudByte) {
  uint8_t msgLength = 1;
  
  DynamixelSendWriteCommand(dyna, DYNAMIXEL_BAUD_RATE, msgLength, &baudByte);

  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

uint8_t DynamixelSetReturnDelayTime(Dynamixel dyna, uint8_t returnDelayByte) {
  uint8_t msgLength = 1;
  
  DynamixelSendWriteCommand(dyna, DYNAMIXEL_RETURN_DELAY_TIME, msgLength, &returnDelayByte);

  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

uint8_t DynamixelSetMaxTorque(Dynamixel dyna, uint16_t maxTorque) {
  uint8_t msgLength = 2;
  uint8_t data[msgLength];
  
  data[0] = maxTorque & 0x00FF;
  data[1] = maxTorque >> 8;
  
  DynamixelSendWriteCommand(dyna, DYNAMIXEL_MAX_TORQUE_L, msgLength, data);

  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

uint8_t DynamixelSetStatusReturnLevel(Dynamixel dyna, uint8_t level) {
  uint8_t msgLength = 1;
  
  DynamixelSendWriteCommand(dyna, DYNAMIXEL_RETURN_LEVEL, msgLength, &level);

  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

uint8_t DynamixelSetMode(Dynamixel dyna, DynamixelMode mode) {
  uint8_t msgLength = 4;
  uint8_t data[msgLength];
  uint8_t ccwHighByte;
  
  switch (mode) {
    case Wheel:
      data[0] = 0x00;
      data[1] = 0x00;
      data[2] = 0x00;
      data[3] = 0x00;
      break;
    case Joint:
      if (dyna.type == MX) {
        ccwHighByte = 0xFF & MX_HIGH_BYTE_MASK;
      }
      
      if (dyna.type == AX) {
        ccwHighByte = 0xFF & AX_HIGH_BYTE_MASK;
      }
      
      data[0] = 0x00;
      data[1] = 0x00;
      data[2] = 0xFF;
      data[3] = ccwHighByte;
      break;
    case MultiTurn:
      if (dyna.type == AX)
        return DYNAMIXEL_ERROR_UNKNOWN;
      
      if (dyna.type == MX) {
        ccwHighByte = 0xFF & MX_HIGH_BYTE_MASK;
      }
      
      data[0] = 0xFF;
      data[1] = ccwHighByte;
      data[2] = 0xFF;
      data[3] = ccwHighByte;
      break;
    default:
      return DYNAMIXEL_ERROR_UNKNOWN;
  }
  
  DynamixelSendWriteCommand(dyna, DYNAMIXEL_CW_ANGLE_LIMIT_L, msgLength, data);

  wait(TXDELAY);
  return DynamixelGetError(dyna);
}

uint8_t DynamixelGetMode(Dynamixel dyna, DynamixelMode* mode) {
  uint8_t msgLength = 3, dataSize = 4, error;
  uint8_t data[msgLength], buffer[dataSize];
  uint16_t cwAngleLimit, ccwAngleLimit;
  
  DynamixelSendReadCommand(dyna, DYNAMIXEL_CW_ANGLE_LIMIT_L, dataSize);
  
  wait(TXDELAY);
  error = DynamixelGetReturnPacket(dyna, buffer, dataSize);
  
  cwAngleLimit = buffer[1];
  cwAngleLimit = (cwAngleLimit << 8) | buffer[0];
  ccwAngleLimit = buffer[3];
  ccwAngleLimit = (ccwAngleLimit << 8) | buffer[2];
  
  if (cwAngleLimit == 0 && ccwAngleLimit == 0) {
    *mode = Wheel;
  }
  
  switch (dyna.type) {
    case AX:
      if (cwAngleLimit == 0 && ccwAngleLimit == 0x03FF) {
        *mode = Joint;
      }
      break;
    case MX:
      if (cwAngleLimit == 0 && ccwAngleLimit == 0x0FFF) {
        *mode = Joint;
      }
      if (cwAngleLimit == 0x0FFF && ccwAngleLimit == 0x0FFF) {
        *mode = MultiTurn;
      }
  }
  return error;
}

uint8_t DynamixelGetPresentPosition(Dynamixel dyna, uint16_t* pos) {
  uint8_t dataSize = 2, error;
  uint8_t buffer[dataSize];
  
  DynamixelSendReadCommand(dyna, DYNAMIXEL_PRESENT_POSITION_L, dataSize);
  
  wait(TXDELAY);
  error = DynamixelGetReturnPacket(dyna, buffer, dataSize);
  
  *pos = buffer[1];
  *pos = (*pos << 8) | buffer[0];
  
  return  error;
}

uint8_t DynamixelGetPresentSpeed(Dynamixel dyna, uint16_t* speed) {
  uint8_t dataSize = 2, error;
  uint8_t buffer[dataSize];
  
  DynamixelSendReadCommand(dyna, DYNAMIXEL_PRESENT_SPEED_L, dataSize);
  
  wait(TXDELAY);
  error = DynamixelGetReturnPacket(dyna, buffer, dataSize);
  
  *speed = buffer[1];
  *speed = (*speed << 8) | buffer[0];
  
  return error;
}

uint8_t DynamixelGetLoad(Dynamixel dyna, uint16_t* load) {
  uint8_t dataSize = 2, error;
  uint8_t buffer[dataSize];
  
  DynamixelSendReadCommand(dyna, DYNAMIXEL_PRESENT_LOAD_L, dataSize);
  
  wait(TXDELAY);
  error = DynamixelGetReturnPacket(dyna, buffer, dataSize);
  
  *load = buffer[1];
  *load = (*load << 8) | buffer[0];
  
  return error;
}

uint8_t DynamixelGetVoltage(Dynamixel dyna, uint8_t* voltage) {
  uint8_t dataSize = 1;
  
  DynamixelSendReadCommand(dyna, DYNAMIXEL_PRESENT_VOLTAGE, dataSize);
  
  wait(TXDELAY);
  return DynamixelGetReturnPacket(dyna, voltage, dataSize);
}

uint8_t DynamixelGetTemperature(Dynamixel dyna, uint8_t* temp) {
  uint8_t dataSize = 1;
  
  DynamixelSendReadCommand(dyna, DYNAMIXEL_PRESENT_TEMPERATURE, dataSize);

  wait(TXDELAY);
  return DynamixelGetReturnPacket(dyna, temp, dataSize);
}
