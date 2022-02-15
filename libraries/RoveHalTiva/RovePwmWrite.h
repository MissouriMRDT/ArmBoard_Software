///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Generator (PWM Gen)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_PWM_WRITE_H
#define ROVE_PWM_WRITE_H

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo => Clock Divide by 64 on a 16b generator at 120Mhz system clock => 8 nS ~ 17.476 mS period => 120Mhz ~ 29Hz frequency
// Todo => PWM_GEN_USE_CLOCK_DIV_64 for Todo Clock Divide
// Todo => value=0~255 at 490hz for period_micros=2040uS
// Todo forward declare => instead of comments and .h file ?
// Todo => all this goes IN roveware:: and then one more level of RoveHal include wrapper?
// Pwm Pins => ti.com/lit/ds/symlink/tm4c1294ncpdt.pdf => Table 23-1. PWM Signals (128TQFP) => Tiva1294XL Launchpad variant

// Energia.pin_map.h::     PF_0
// Energia.pin_map.h::     PF_1
// Energia.pin_map.h::     PF_2
// Energia.pin_map.h::     PF_3
// Energia.pin_map.h::     PG_0
// Energia.pin_map.h::     PG_1
// Energia.pin_map.h::     PK_4
// Energia.pin_map.h::     PK_5

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////////////
void rovePwmWrite(       uint8_t pin, int pulse_width_micros, int pulse_period_micros );
void rovePwmAnalogWrite( uint8_t pin, int value );

#endif // ROVE_PWM_WRITE_H