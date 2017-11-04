/*
 * Programmer: Drue Satterfield
 * Date o creation: ~9/4/17
 *
 * General, non-hardware-specific list of pwm reading functions that each board in the RoveBoard network supports.
 */

#ifndef PWMREADER_H_
#define PWMREADER_H_

#include <stdint.h>
#include <stdbool.h>
#include "RovePwmReadStructures.h"

//Begins reading pwm pulses on the specified pin using the specified timer.
//Input: The pwmRead module to use, and which of its associated GPIO pins are to be used
//Returns a rovePwmRead handle with internal settings initialized
//warning: if the arguments are invalid, the function enters an infinite loop fault routine for checking in a debugger
extern rovePwmRead_Handle initPwmRead(uint8_t readingModule, uint8_t mappedPin);

//Stops reading pwm.
//Input: The handle for the pwm reading instance to stop reading with
//Note: initPwmRead must be called before hand
extern void stopPwmRead(rovePwmRead_Handle handle);

//gets the duty cycle being read on the specified pin.
//Input: The handle for the pwm reading instance to stop reading with
//Note: initPwmRead must be called before hand
//Output: 0-100 duty cycle
extern uint8_t getDuty(rovePwmRead_Handle handle);

//gets the total period of the PWM signal last transmitted for
//the specified pin
//Input: The handle for the pwm reading instance to stop reading with
//Note: initPwmRead must be called before hand
//Output: period of last transmission in microseconds
extern uint32_t getTotalPeriod_us(rovePwmRead_Handle handle);

//Gets the on period of the last tramsittted PWM signal for
//the specified pin
//Input: The handle for the pwm reading instance to stop reading with
//Note: initPwmRead must be called before hand
//Output: On-period of pulse in microseconds
extern uint32_t getOnPeriod_us(rovePwmRead_Handle handle);

#endif
