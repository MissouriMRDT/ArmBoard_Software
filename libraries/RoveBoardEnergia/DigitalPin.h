/*
 * Programmer: Drue Satterfield
 * Date o creation: ~9/4/17
 *
 * General, non-hardware-specific list of GPIO pin digital read/write functions that each board in the RoveBoard network supports.
 */

#ifndef DIGITALPIN_H_
#define DIGITALPIN_H_

#include <stdbool.h>
#include <stdint.h>

#include "DigitalPinStructures.h"

//returns if the voltage on the passed pin is at the digital HIGH voltage level or the digital LOW voltage level.
//High voltage = 1 in code, low voltage = 0 in code, what those are in volts is hardware dependant.
//Inputs: number of the pin to read from, as defined in the board's PinMap file.
//returns: 1 if high voltage, 0 if low voltage
extern bool digitalPinRead(uint8_t pinNumber);

//places pin into a certain input mode (or keeps it at that mode if called with same argument before),
//returns if the voltage on the passed pin is at the digital HIGH voltage level or the digital LOW voltage level.
//High voltage = 1 in code, low voltage = 0 in code, what those are in volts is hardware dependant.
//Inputs: number of the pin to read from, as defined in the board's PinMap file.
//returns: 1 if high voltage, 0 if low voltage
extern bool digitalPinRead(uint8_t pinNumber, uint8_t inputMode);

//writes a High voltage (equal to data bit 1) or a Low voltage (equal to data bit 0) on the passed pin.
//Inputs: number of the pin to read from, as defined in the board's PinMap file. 1 for high output, 0 for low output.
//returns true if successful, false if incorrect arguments are passed.
extern bool digitalPinWrite(uint8_t pinNumber, bool outputLevel);

//places pin into a certain output mode (or keeps it at that mode if called with same argument before),
//writes a High voltage (equal to data bit 1) or a Low voltage (equal to data bit 0) on the passed pin.
//Inputs: number of the pin to read from, as defined in the board's PinMap file. 1 for high output, 0 for low output.
//returns true if successful, false if incorrect arguments are passed.
extern bool digitalPinWrite(uint8_t pinNumber, bool outputLevel, uint8_t outputMode);

#endif
