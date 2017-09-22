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

//writes a pwm wave out on a pin (if it's a pin that supports pwm) at the specified duty cycle
//inputs: pin to write out of, as specified by the board's pinmap, and what duty cycle is from 0 to 255 (represents 0 to 100%)
extern void pwmWrite(uint8_t pin, uint8_t duty);

//writes a pwm wave out on a pin (if it's a pin that supports pwm) at the specified pulse width and pulse period
//inputs: pin to write out of, as specified by the board's pinmap, pulse width in microseconds, pulse total period in microseconds
extern void pwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us);


#endif /* PWMWRITER_H_ */

