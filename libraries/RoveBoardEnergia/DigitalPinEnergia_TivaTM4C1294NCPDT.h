#ifndef DIGITALPIN_TIVATM4C1294NCPDT_H_
#define DIGITALPIN_TIVATM4C1294NCPDT_H_

/* Programmer: Drue Satterfield
 * Date of creation: September 2017
 * Microcontroller used: Tiva TM4C1294NCPDT
 * Hardware components used by this file: Potentially all GPIO pins. Pins are explicitely called into usage by the user
 * and when not initialized can be used by other hardware components.
 *
 * Description: This library is used to input and output digital signals on GPIO pins (digital signals = just a high voltage
 * representing bit 1, or a low voltage for bit 0). Voltage is in CMOS for the tivac, IE 3.3V for 1 and 0V for 0.
 * As well, the user can choose to modify the pin's setting when reading or writing signals; the available settings are
 * writing in normal or open drain mode, and reading in normal mode, or with internal pull up or down resistors. More information below on those.
 *
 * This library uses the roveboard pin mapping standard, for passing pins to the functions. Refer to the roveboard github's wiki for more info.
 *
 *
 * Warnings: The functions will override each other's settings if called seperately; if a special setting is used, it must be
 * specified on each call.
 * Pin PD_7 is locked by the hardware and shouldn't be used
 */

#include <stdint.h>
#include <stdbool.h>
#include "Energia.h"
#include "DigitalPinStructures.h"

//pin mode constants
const uint8_t Output = 0; //generic digital out signal (pulls signal to high or low voltage). Default output mode
const uint8_t OpenDrainOutput = 1; //open drain style output (pulls signal low, or lets it float)
const uint8_t PullUpInput = 2; //input mode with an internal pull up resistor (pulls line up if read signal is floating)
const uint8_t PullDownInput = 3; //input mode with an internal pull down resistor (pulls line down if read signal is floating)
const uint8_t Input = 4; //generic digital signal read (reads a high or low voltage). Default input mode

//pin output power constants
const uint8_t MA_2 = 0; //output 2 milliamps (default)
const uint8_t MA_4 = 1; //and so on going down
const uint8_t MA_6 = 2;
const uint8_t MA_8 = 3;
const uint8_t MA_10 = 4;
const uint8_t MA_12 = 5;

/*************************standard******************/

//returns if the voltage on the passed pin is at the digital HIGH voltage level or the digital LOW voltage level.
//Inputs: number of the pin to read from, as defined in the PinMap file.
//Note: This function puts the read pin into the default input mode
//returns: 1 or 0
bool digitalPinRead(uint8_t pinNumber);

//places pin into a certain input mode (or keeps it at that mode if called with same argument before),
//returns if the voltage on the passed pin is at the digital HIGH voltage level or the digital LOW voltage level.
//Inputs: number of the pin to read from, as defined in the PinMap file., and input mode, based on the above constants
//returns: 1 if high voltage, 0 if low voltage
bool digitalPinRead(uint8_t pinNumber, uint8_t inputMode);

//writes a High voltage (equal to data bit 1) or a Low voltage (equal to data bit 0) on the passed pin.
//Inputs: number of the pin to read from, as defined in the PinMap file. 1 for high output, 0 for low output.
//Note: This function puts the write pint into the default output mode
//returns true if successful, false if incorrect arguments are passed.
bool digitalPinWrite(uint8_t pinNumber, bool outputLevel);

//places pin into a certain output mode (or keeps it at that mode if called with same argument before),
//writes a High voltage (equal to data bit 1) or a Low voltage (equal to data bit 0) on the passed pin.
//Inputs: number of the pin to read from, as defined in the board's PinMap file. 1 for high output, 0 for low output.
//		  Output mode, based on the constants above
//returns true if successful, false if incorrect arguments are passed.
bool digitalPinWrite(uint8_t pinNumber, bool outputLevel, uint8_t outputMode);


/**************************nonstandard*******************/

//places pin into a certain output mode (or keeps it at that mode if called with same argument before),
//writes a High voltage (equal to data bit 1) or a Low voltage (equal to data bit 0) on the passed pin, and sets out how
//much current the pin is allowed to pour out.
//Inputs: number of the pin to read from, as defined in the board's PinMap file. 1 for high output, 0 for low output.
//		  Output mode, based on the constants above. OutputPowerLevel, also based on the constants above.
//returns true if successful, false if incorrect arguments are passed.
bool digitalPinWrite(uint8_t pinNumber, bool outputLevel, uint8_t outputMode, uint8_t outputPowerLevel);

#endif
