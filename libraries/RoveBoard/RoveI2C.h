#ifndef ROVEI2C_H_
#define ROVEI2C_H_

#include "RoveI2cTypenames.h"
#include <stdint.h>
#include <stddef.h>

//Initializes the i2c module for usage
//input: i2cIndex: the index of the module, based on board specific constatns
//       speed: The speed of communication. Acceptable inputs are board specific
//       clock and data pin: The SCL and SDA pins you want to use, that should match the module.
//returns: a handle for the now initialized i2c module
extern RoveI2C_Handle i2cInit(uint8_t i2cIndex, RoveI2C_Speed speed, uint8_t clockPin, uint8_t dataPin);

//sends a single message to a slave device, without specifying a destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The message byte to transmit
//returns: result of the transmission
extern RoveI2C_Error roveI2cSend(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t msg);

//sends a single message to a slave device, specifying a destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The address of the destination register within the slave device, from 0 to 255
//       The message byte to transmit
//returns: result of the transmission
extern RoveI2C_Error roveI2cSend(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t msg);

//sends a series of messages to a slave device, without specifying a destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The message array to transmit
//       The amount of messages in the array
//returns: result of the transmission
extern RoveI2C_Error roveI2cSendBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t msg[], size_t msgSize);

//sends a series of messages to a slave device, specifying the starting destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The address of the starting destination register within the slave from 0 to 255
//       The message array to transmit
//       The amount of messages in the array
//returns: result of the transmission
extern RoveI2C_Error roveI2cSendBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t msg[], size_t msgSize);

//requests a byte of information from a slave, without specifying a register within the slave to read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The variable, passed by pointer, to store the received byte into
//returns: result of the transmission
extern RoveI2C_Error roveI2cReceive(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t* buffer);

//requests a byte of information from a slave, specifying a register within the slave to read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The address of the slave register to read from
//       The variable, passed by pointer, to store the received byte into
//returns: result of the transmission
extern RoveI2C_Error roveI2cReceive(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t* buffer);

//requests a series of bytes of information from a slave, without specifying a register within the slave to read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The buffer to fill up with the received information
//       The amount of bytes to receive
//returns: result of the transmission
extern RoveI2C_Error roveI2cReceiveBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t* buffer, size_t sizeOfReceive);

//requests a series of bytes of information from a slave, specifying the starting register within the slave to first read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device
//       The address of the register within the slave to read from.
//       The buffer to fill up with the received information
//       The amount of bytes to receive
//returns: result of the transmission
extern RoveI2C_Error roveI2cReceiveBurst(RoveI2C_Handle handle, uint16_t SlaveAddr,  uint8_t reg, uint8_t* buffer, size_t sizeOfReceive);

#endif
