/*Main Programmmers: David Strickland, Chris Dutcher
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
*/

#define PART_TM4C1294NCPDT

#include "Energia.h"
#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

//Decides the alignment of the PWM wave. That is, where the pulse is generated in the overall waveform. 
//For example, leftAligned generates a pulse like this:   --____
//Center aligned would look like this: __--__
enum pwmAlignment {LeftAligned = 0, CenterAligned = 1};

//outputs a pwm wave at the specified duty cycle on the specified pin
//pin: PF_0  PF_1  PF_2  PF_3  PG_0  PG_1  PK_4  PK_5 
//Duty: between 0 and 255 with 0 being 0% and 255 being 100% duty cycle
//defaults are left alignment and non-inverted, with frequency of 490 hz
void PwmWrite(uint8_t pin, uint8_t duty);

//outputs a pwm wave on the specified pin, where the wave will have the specified on period/pulse width, and the specified total period (on period + off period)
//pin: PF_0  PF_1  PF_2  PF_3  PG_0  PG_1  PK_4  PK_5 
//PulseW_us is the time in microseconds which you want the PWM to be high. 0 for 0% duty cycle and = to the PulsePeriod_us for 100% duty cycle.
//PulsePeriod_us is the pulse overall period in microseconds. Should never exceed 32 bits(3 min or so, quite the long pwm wave)
//defaults are left alignment and non-inverted
void PwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us);

//outputs a pwm wave ont eh specified pin, where the wave will have the specified on period/pulse width, the specified total period (on period + off period), the on-period pulses
//will have the specified alignment in the total waveform, and specified inversion.
//pin: PF_0  PF_1  PF_2  PF_3  PG_0  PG_1  PK_4  PK_5
//PulseW_us is the time in microseconds which you want the PWM to be high. 0 for 0% duty cycle and = to the PulsePeriod_us for 100% duty cycle.
//PulsePeriod_us is the period in microseconds which the PWM will be read. Should never exceed 32 bits(3 min or so)
//pwmAlignemt is an enum to select the desired alignment of the PWM pulse. LeftAligned gives left alignement and CenterAligned makes center aligned.
//invertOutput inverts the output. Makes wave either act as active low for certain components as well as right align the wave (when left aligned).
//************************************************************************************************************************************************
//         MAKE SURE NOT TO USE THE SAME GENERATOR IF PULSE PERIOD ALIGNMENT OR INVERT ARE DIFFERENT DOUBLE CHECK 
//************************************************************************************************************************************************
void PwmWrite(uint8_t pin, uint32_t PulseW_us, uint32_t PulsePeriod_us, pwmAlignment alignment, bool invertOutput);    //Most complex PWM write, has all implemented options required

