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

//Begins reading pwm pulses on the specified pin (if it's a pin that supports pwm).
//Input: The pin reading the pwm signal, as defined in the board's PinMap file.
//Output: True if initialized successfully, false if error
//occured (most likely you input a pin that can't read pwm pulses)
bool initPwmRead(uint8_t mappedPin);

//Stops reading pwm.
//Input: The pin to cease reading pwm on, as defined in the board's PinMap file.
extern void stopPwmRead(uint8_t mappedPin);

//gets the duty cycle being read on the specified pin (if it's a pin that supports pwm and been set up).
//Input: The pin reading the pwm signal, as defined in the board's PinMap file.
//Output: 0-100 duty cycle
extern uint8_t getDuty(uint8_t mappedPin);

//gets the total period of the PWM signal last transmitted for
//the specified pin (if it's a pin that supports pwm and been set up)
//Input: The pin reading the pwm signal, as defined in the board's PinMap file.
//Output: period of last transmission in microseconds
extern uint32_t getTotalPeriod_us(uint8_t mappedPin);

//Gets the on period of the last tramsittted PWM signal for
//the specified pin (if it's a pin that supports pwm and been set up)
//Input: The pin reading the pwm signal, as defined in the board's PinMap file.
//Output: On-period of pulse in microseconds
extern uint32_t getOnPeriod_us(uint8_t mappedPin);

#endif
