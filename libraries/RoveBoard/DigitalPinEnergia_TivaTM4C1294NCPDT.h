/*
 * DigitalPinEnergia_TivaTM4C1294NCPDT.h
 *
 *  Created on: Sep 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_DIGITALPINENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_DIGITALPINENERGIA_TIVATM4C1294NCPDT_H_

#include <stdint.h>
#include <stdbool.h>

//pin mode constants
const uint8_t Output = 0; //generic digital out signal (pulls signal to high or low voltage). Default output mode
const uint8_t OpenDrainOutput = 1; //open drain style output (pulls signal low, or lets it float)
const uint8_t PullUpInput = 2; //input mode with an internal pull up resistor (pulls line up if read signal is floating)
const uint8_t PullDownInput = 3; //input mode with an internal pull down resistor (pulls line down if read signal is floating)
const uint8_t Input = 4; //generic digital signal read (reads a high or low voltage). Default input mode

//pin output power constants
const uint8_t MA_2 = 0; //output 2 milliamps
const uint8_t MA_4 = 1; //and so on going down
const uint8_t MA_6 = 2;
const uint8_t MA_8 = 3;
const uint8_t MA_10 = 4;
const uint8_t MA_12 = 5;

const uint8_t InvalidPinCount = 1;
const uint8_t InvalidPins[InvalidPinCount] = {27}; //locked or used by JTAG. 27 = PD_7 in energia

bool digitalPinRead(uint8_t pinNumber);
bool digitalPinRead(uint8_t pinNumber, uint8_t inputMode);
bool digitalPinWrite(uint8_t pinNumber, bool outputLevel);
bool digitalPinWrite(uint8_t pinNumber, bool outputLevel, uint8_t outputMode);
bool digitalPinWrite(uint8_t pinNumber, bool outputLevel, uint8_t outputMode, uint8_t outputPowerLevel);

#define HIGH true
#define LOW false //for pins


#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_DIGITALPINENERGIA_TIVATM4C1294NCPDT_H_ */
