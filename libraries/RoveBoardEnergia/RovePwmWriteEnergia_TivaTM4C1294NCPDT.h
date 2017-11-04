/*Main Programmmers: David Strickland, Chris Dutcher
 *Other Programmers: Drue Satterfield
**Last Modified: November 6, 2016
**
**This library is used to generate a PWM wave on the TI tiva tm4c1294ncpdt board, using its internal hardware PWM module (the board has just one, it makes 8 outputs).
**There are a total of eight GPIO pins which are capable of using the PWM function,
**and eight possible PWM waves can be generated at a single time.
*
*
*The library is split into 3 functions. The first one is the simplest, emulating the arduino standard of just putting in the duty cycle and the pin to output it on, using the arduino
*default pwm frequency of 490 hz. The next two let you specify more advanced functionalities.
*
*Warning: While 8 outputs can be made at once, and they can all have unique pulse widths, two of each are paired and will have the same pulse frequency and
*other pulse settings. This is because the pwm module on the tiva has four internal generators: These generators are what make the pwm output. Each generator
*has two pwm outputs, and is capable of making them have different pulse widths, but it forces them to share the same settings otherwise.
*Keep this in mind when selecting pins, choose the pins wisely so as to not cause conflicxs for the PWM outputs.
*If you do try to give all 8 different settings, they'll just be overwritten
**
*
*Pins that can output pwm with this library and thus be passed to the functions in this library using the energia pinmap standard:
*(note again that pins that share a generator will have the same settings except for pulse width)
* PF_0 (generator 0)
* PF_1 (generator 0)
* PF_2 (generator 1)
* PF_3 (generator 1)
* PG_0 (generator 2)
* PG_1 (generator 2)
* PK_4 (generator 3)
* PK_5 (generator 3)
*
* Energia pinmapping standard can be found here:
* https://github.com/energia/Energia/blob/master/hardware/lm4f/variants/launchpad_129/pins_energia.h
*
* Warning: Timing notices:
* The minimum waveform frequency that can be output is 29 hz, with an accuracy of .53 microseconds (though only 1 microsecond of difference can be input to the lib anyway).
* These are the best conditions, and the best conditions are when the system clock is 120Mhz/2^n (120, 60, 30...), up to 1.875 Mhz.
* When the system clock isn't 120Mhz/2^n, then the minimum frequency increases, potentially up to 57 hz at worst.
* To calculate your new mininum frequency yourself, use this formula if you're using a clock freq that isn't 120Mhz/n^n:
*
* minimum freq = (clock F / divisor) / 65535, where divisor = {
*   64, F = 120Mhz
*   32, 60Mhz <= F < 120Mhz
*   16, 30Mhz <= F < 60Mhz
*   8,  15Mhz <= F < 30Mhz
*   4,  7.5Mhz <= F < 15Mhz
*   2,  3.75Mhz <= F < 7.5Mhz
*   1,  1.875Mhz <= F < 3.75Mhz
*   }
*
*   Minimum freq for the system clock to use this library is 1.875Mhz. Below that, the pwmwrite functions shall not do anything.
*   In all these cases, the functions shall have at least an accuracy of 1 microsecond of change
*/

#ifndef ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_PWMWRITERENERGIA_TIVATM4C1294NCPDT_H_
#define ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_PWMWRITERENERGIA_TIVATM4C1294NCPDT_H_


#include <stdint.h>
#include <stdbool.h>
#include "Energia.h"
#include "RovePwmWriteStructures.h"
//Decides the alignment of the PWM wave. That is, where the pulse is generated in the overall waveform.
//For example, leftAligned generates a pulse like this:   --____
//Center aligned would look like this: __--__
typedef enum pwmAlignment {LeftAligned = 0, CenterAligned = 1} pwmAlignment;

const int PwmGenerator0 = 0;
const int PwmGenerator1 = 1;
const int PwmGenerator2 = 2;
const int PwmGenerator3 = 3;

//sets up the pwm generator to be able to write a pwm wave on the specified pin
//inputs:
//  writeModule: the generator to use to write a pwm wave, 0 to 3
//  pin: the pin to output the pwm wave on, must is used by the module. See header comments above for which pins are used by which generator.
//returns: a handle to the initialized pwm instance
//warning: Function will enter an infinite fault loop if arguments are incorrect
//note: You CAN call this multiple times to set up a module with different pins without issue. To use all 8 pwm write pins,
//  you'd call this twice for each pwm generator, once for each pin.
//note: Default settings are: Left aligned wave, 2040 microsecond total period, and output isn't inverted.
rovePwmWrite_Handle setupPwmWrite(uint8_t writeModule, uint8_t pin);

//outputs a pwm wave at the specified duty cycle
//inputs:
//  handle: the handle of the pwm instance to write with
//  duty: duty cycle, between 0 and 255 with 0 being 0% and 255 being 100% duty cycle
void pwmWriteDuty(rovePwmWrite_Handle handle, uint8_t duty);

//outputs a pwm wave at the specified pulse width
//inputs:
//  handle: the handle of the pwm instance to write with
//  pulseW_us: width of the pwm's On period, in microseconds. 0 for 0% duty cycle, and >= to the pulse total period for 100% duty cycle.
void pwmWriteWidth(rovePwmWrite_Handle handle, uint32_t pulseW_us);

//sets the pwm total period for this generator.
//inputs:
//  handle: the handle of the pwm instance to write with
//  pulsePeriod_us: width of the entire pwm wave, in microseconds. Should never exceed 32 bits(3 min or so, quite the long pwm wave)
//warning: This will affect all the handles that are using the generator this handle uses.
void setPwmTotalPeriod(rovePwmWrite_Handle handle, uint32_t pulsePeriod_us);

//sets the pwm wave alignment for this generator.
//inputs:
//  handle: the handle of the pwm instance to write with
//  alignment: pwm wave alignment
//warning: This will affect all the handles that are using the generator this handle uses.
void setPwmAlignment(rovePwmWrite_Handle handle, pwmAlignment alignment);

//sets whether or not the output of this generator is inverted (whether the On period is a high voltage or a low, inverted means latter).
//inputs:
//  handle: the handle of the pwm instance to write with
//  invertOutput: whether or not the invert the pwm wave
//warning: This will affect all the handles that are using the generator this handle uses.
//note: if the wave form is left aligned normally, inverting it will actually make it right aligned.
void setPwmInvert(rovePwmWrite_Handle handle, bool invertOutput);


#endif /* ROVEBOARD_ENERGIA_VERSION_ROVEBOARD_PWMWRITERENERGIA_TIVATM4C1294NCPDT_H_ */
