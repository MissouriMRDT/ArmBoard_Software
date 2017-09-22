/* Programmer: Drue Satterfield
 * Date of creation: 10/14/2016
 * Microcontroller used: Tiva TM4C1294NCPDT
 * Hardware components used by this file: Internal timers 1-5
 *
 * Update 10/17/16: under nominal PWM conditions, return values work on timer 1. Edge conditions do not work, timeout interrupt is buggy as hell, currently disabled in timer setup
 * Update 10/20/16: works for 0% and 100%. All pins tested.
 * Update 10/21/16: Works when all 5 are on at once
 * Update 11/3/16: added pin map layer so the user now only has to pass in one argument
 *
 * Description: This library is used to read a pwm signal on
 * pins utilized by timers 1-5. The program is started by calling the
 * pwmStart function, and afterwords the program shall use
 * the timer attached to that pin to monitor for incoming
 * pulses. When the voltage on that pin changes, the timer
 * triggers an interrupt, and the time of the voltage change
 * is recorded. By doing this, it finds the time in microseconds
 * of how long the pulse was high, and how long it was low.
 * Thus it calculates the on period, off period, total period,
 * and duty cycle. The duty cycle, total period, and on period
 * of the most recent pulse can be read by calling the related
 * get functions.
 *
 * This library uses the energia pin mapping standard, for passing pins to the functions. Refer to this link to see what value
 * you should use to represent a pin:  https://github.com/energia/Energia/blob/master/hardware/lm4f/variants/launchpad_129/pins_energia.h
 *
 * Acceptable pins (and which timer they use, don't try to use multiple pins that use the same timer)
 * A2 (t1), A4(t2), A6(t3), B0(t4), B2(t5), D2(t1), D4(t3), L6(t1), M0(t2), M2(t3), M4(t4), M6(t5)
 *
 * Warnings: The file triggers interrupts quite frequently,
 * once ever time the voltage on the pin changes and a second
 * timed interrupt that occurs roughly once every half a second
 * (it's there to basically monitor the line and detect
 * if the line has 0% or 100% duty). It can be processor-intensive.
 * Also the first pulse of a transmission can sometimes be
 * disregarded as garbage data depending on conditions,
 * though this doesn't tend to matter in pwm transmissions.
 *
 * Minimum frequency that can be read is 1 hz. Accuracy: can read difference of 62.5 nanoseconds between pulses
 */

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_PWMREADERENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_PWMREADERENERGIA_TIVATM4C1294NCPDT_H_

#include <stdint.h>
#include <stdbool.h>
#include "Energia.h"

//Begins reading pwm pulses on the specified pin.
//Input: The pin to cease reading pwm on, using the energia
//pin map standard
//Output: True if initialized successfully, false if error
//occured (most likely you input a pin that can't read pwm pulses
bool initPwmRead(uint8_t mappedPin);

//Stops reading pwm.
//Input: The pin to cease reading pwm on, using the energia
//pin map standard
void stopPwmRead(uint8_t mappedPin);

//gets the duty cycle being read on the specified pin.
//Input: The pin to cease reading pwm on, using the energia
//pin map standard
//Output: 0-100 duty cycle
uint8_t getDuty(uint8_t mappedPin);

//gets the total period of the PWM signal last transmitted for
//the specified pin
//Input: The pin to cease reading pwm on, using the energia
//pin map standard
//Output: period of last transmission in microseconds
uint32_t getTotalPeriod_us(uint8_t mappedPin);

//Gets the on period of the last tramsittted PWM signal for
//the specified pin
//Input: The pin to cease reading pwm on, using the energia
//pin map standard
//Output: On-period of pulse in microseconds
uint32_t getOnPeriod_us(uint8_t mappedPin);




#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_PWMREADERENERGIA_TIVATM4C1294NCPDT_H_ */
