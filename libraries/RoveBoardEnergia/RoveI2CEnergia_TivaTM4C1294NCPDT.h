#ifndef ROVEI2C_TIVATM4C1294NCPDT_H_
#define ROVEI2C_TIVATM4C1294NCPDT_H_

/* Programmer: Drue Satterfield, Jimmy Haviland
 * Date of creation: Original program: sometime 2016. Revised for roveboard: September 2017
 * Microcontroller used: Tiva TM4C1294NCPDT
 * Hardware components used by this file: Potentially all I2C modules, 0 - 9; hardware has to be explicitely initialized for usage by the user
 *
 * Description: This library is used to read and write data to and from devices with the I2C protocol, based off of the chip's
 * internal hardware I2C modules. This library uses the roveboard pin mapping standard, for passing pins to the functions.
 * Refer to the roveboard github's wiki for more info.
 *
 * The functions are generally built to be non blocking; they'll return immediately if they think that the line is already in usage
 * by another master or if they detect an error in transmission. The one holdup will be if the slave(s) tries to hold the clock line
 * low; after a period of time, the function will time out in this condition. The timeout value is 1/(baud rate / 4080) seconds.
 *
 * Module     clock/SCL pin(s)    data/SDA pin(s)
 *   0              PB_2              PB_3
 *   1              PG_0              PG_1
 *   2        PL_1, PP_5, PN_5     PL_0, PN_4
 *   3              PK_4              PK_5
 *   4              PK_6              PK_7
 *   5           PB_0, PB_4        PB_1, PB_5
 *   6              PA_6              PA_7
 *   7           PD_0, PA_4        PD_1, PA_5
 *   8           PD_2, PA_2        PD_3, PA_3
 *   9              PA_0              PA_1
 *
 *
 * Warnings:
 * The hardware only supports 7 bit addressing for the slaves, for a maximum of 127 slave devices on one module.
 *
 * If the function returns a timeout error, you'll need to re-initialize the i2c module.
 */

#include "RoveI2cStructures.h"
#include <stdint.h>
#include <stddef.h>
#include "Energia.h"

#define I2C_Module0 0
#define I2C_Module1 1
#define I2C_Module2 2
#define I2C_Module3 3
#define I2C_Module4 4
#define I2C_Module5 5
#define I2C_Module6 6
#define I2C_Module7 7
#define I2C_Module8 8
#define I2C_Module9 9

//Initializes the i2c module for usage
//input: i2cIndex: the index of the module, based on the defines above
//       speed: The speed of communication. Acceptable inputs are I2CSPEED_STANDARD, I2CSPEED_FULL, and I2CSPEED_FAST
//       clock and data pin: The SCL and SDA pins you want to use. Make sure they match up with the module
//returns: a handle for the now initialized i2c module
//warning: if the arguments are invalid, the function enters an infinite loop fault routine for checking in a debugger
RoveI2C_Handle i2cInit(uint8_t i2cIndex, RoveI2C_Speed speed, uint8_t clockPin, uint8_t dataPin);

//sends a single message to a slave device, without specifying a destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The message byte to transmit
//returns: result of the transmission
RoveI2C_Error roveI2cSend(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t msg);

//sends a single message to a slave device, specifying a destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The address of the destination register within the slave device, from 0 to 255
//       The message byte to transmit
//returns: result of the transmission
RoveI2C_Error roveI2cSend(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t msg);

//sends a series of messages to a slave device, without specifying a destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The message array to transmit
//       The amount of messages in the array
//returns: result of the transmission
RoveI2C_Error roveI2cSendBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t msg[], size_t msgSize);

//sends a series of messages to a slave device, specifying the starting destination register within the slave
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The address of the starting destination register within the slave from 0 to 255
//       The message array to transmit
//       The amount of messages in the array
//returns: result of the transmission
RoveI2C_Error roveI2cSendBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t msg[], size_t msgSize);

//requests a byte of information from a slave, without specifying a register within the slave to read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The variable, passed by pointer, to store the received byte into
//returns: result of the transmission
RoveI2C_Error roveI2cReceive(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t* buffer);

//requests a byte of information from a slave, specifying a register within the slave to read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The address of the slave register to read from
//       The variable, passed by pointer, to store the received byte into
//returns: result of the transmission
RoveI2C_Error roveI2cReceive(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t* buffer);

//requests a series of bytes of information from a slave, without specifying a register within the slave to read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The buffer to fill up with the received information
//       The amount of bytes to receive
//returns: result of the transmission
RoveI2C_Error roveI2cReceiveBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t* buffer, size_t sizeOfReceive);

//requests a series of bytes of information from a slave, specifying the starting register within the slave to first read from.
//input: The handle of the i2c module to use, gained from the init function
//       The address of the slave device, from 0 to 127
//       The address of the register within the slave to read from.
//       The buffer to fill up with the received information
//       The amount of bytes to receive
//returns: result of the transmission
RoveI2C_Error roveI2cReceiveBurst(RoveI2C_Handle handle, uint16_t SlaveAddr,  uint8_t reg, uint8_t* buffer, size_t sizeOfReceive);

#endif
