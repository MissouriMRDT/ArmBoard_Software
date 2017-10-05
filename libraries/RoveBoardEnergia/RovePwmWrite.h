/*
 * Programmer: Drue Satterfield
 * Date o creation: ~9/4/17
 *
 * General, non-hardware-specific list of pwm writing functions that each board in the RoveBoard network supports.
 */

#ifndef PWMWRITER_H_
#define PWMWRITER_H_

#include <stdint.h>
#include <stdbool.h>
#include "RovePwmWriteStructures.h"

//sets up the pwm generator to be able to write a pwm wave on the specified pin
//inputs:
//  writeModule: the generator to use to write a pwm wave
//  pin: the pin to output the pwm wave on, must is used by the module.
//returns: a handle to the initialized pwm instance
extern rovePwmWrite_Handle setupPwmWrite(uint8_t writeModule, uint8_t pin);

//outputs a pwm wave at the specified duty cycle
//inputs:
//  handle: the handle of the pwm instance to write with
//  duty: duty cycle, between 0 and 255 with 0 being 0% and 255 being 100% duty cycle
extern void pwmWriteDuty(rovePwmWrite_Handle handle, uint8_t duty);

//outputs a pwm wave at the specified pulse width
//inputs:
//  handle: the handle of the pwm instance to write with
//  pulseW_us: width of the pwm's On period, in microseconds. 0 for 0% duty cycle, and >= to the pulse total period for 100% duty cycle.
extern void pwmWriteWidth(rovePwmWrite_Handle handle, uint32_t pulseW_us);

//sets the pwm total period
//inputs:
//  handle: the handle of the pwm instance to write with
//  pulsePeriod_us: width of the entire pwm wave, in microseconds. Should never exceed 32 bits(3 min or so, quite the long pwm wave)
extern void setPwmTotalPeriod(rovePwmWrite_Handle handle, uint32_t pulsePeriod_us);


#endif /* PWMWRITER_H_ */

