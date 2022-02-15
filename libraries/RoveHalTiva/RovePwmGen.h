///////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Generator (PWM Gen)
///////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_PWM_GEN_H
#define ROVE_PWM_GEN_H

#include <stdint.h>
#include "driverlib/pwm.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo => 120Mhz / ( micros / sec ) 
// Todo => PWM_GEN_DIV_1_TICKS_PER_MICRO ?
// Todo => all this goes IN roveware:: and then one more level of RoveHal include wrapper?
/*  enum  pwm_gen_clock_div_t { 
    PWM_GEN_USE_CLOCK_DIV_1  = PWM_SYSCLK_DIV_1,    //  8 nanos ~    546 micros => 120 MHz ~ 1,831 Hz
    PWM_GEN_USE_CLOCK_DIV_2  = PWM_SYSCLK_DIV_2,    // 16 nanos ~  1.092 millis =>  60 MHz ~   916 Hz
    PWM_GEN_USE_CLOCK_DIV_4  = PWM_SYSCLK_DIV_4,    // 33 nanos ~  2.185 millis =>  30 MHz ~   458 Hz
    PWM_GEN_USE_CLOCK_DIV_8  = PWM_SYSCLK_DIV_8,    // 66 nanos ~  4.369 millis =>  15 MHz ~   229 Hz
    PWM_GEN_USE_CLOCK_DIV_16 = PWM_SYSCLK_DIV_16,   //133 nanos ~  8.738 millis =>   8 MHz ~   114 Hz
    PWM_GEN_USE_CLOCK_DIV_32 = PWM_SYSCLK_DIV_32,   //266 nanos ~ 17.476 millis =>   4 MHz ~    57 Hz
    PWM_GEN_USE_CLOCK_DIV_64 = PWM_SYSCLK_DIV_64 }; //533 nanos ~ 34.952 millis =>   2 MHZ ~    29 Hz*/
/*void                  setPwmGen(     uint32_t PWM_PIN_MUX,
                                       uint32_t PWM_GEN,
                                       uint32_t CLOCK_DIV
                                       uint32_t PORT_BASE_ADDRESS,
                                       uint32_t PIN_BIT_MASK,
                                       uint16_t PERIOD_TICKS_16 ); */
/*void                  pwmGenWrite(   uint32_t PWM_GEN,
                                       uint32_t PWM_OUT, 
                                       uint32_t PIN_BIT_MASK, 
                                       uint16_t WIDTH_TICKS_16 ); */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace roveware
{
  enum pwm_gen_div { PWM_USE_FULL_SPEED_CLOCK   = PWM_SYSCLK_DIV_1, 
                     PWM_USE_DIVIDE_CLOCK_BY_2  = PWM_SYSCLK_DIV_2, 
                     PWM_USE_DIVIDE_CLOCK_BY_4  = PWM_SYSCLK_DIV_4, 
                     PWM_USE_DIVIDE_CLOCK_BY_8  = PWM_SYSCLK_DIV_8, 
                     PWM_USE_DIVIDE_CLOCK_BY_16 = PWM_SYSCLK_DIV_16, 
                     PWM_USE_DIVIDE_CLOCK_BY_32 = PWM_SYSCLK_DIV_32,
                     PWM_USE_DIVIDE_CLOCK_BY_64 = PWM_SYSCLK_DIV_64 };

  //const int      SYSCLOCK_TICKS_PER_MICRO   = ( 120000000 / 1000000 );

  /////////////////////////////////////
  struct PwmGenHardware
  {
    volatile uint32_t PWM_GEN_PIN_MUX;
    volatile uint32_t PWM_GEN;
    volatile uint32_t PWM_OUT;
    volatile uint32_t PWM_BIT_MASK;
    volatile uint32_t PORT_BASE_ADDRESS;
    volatile uint32_t PIN_BIT_MASK;
  };

  ////////////////////////////////////////////////////
  struct PwmGenHardware pwmGenHardware( uint8_t pin );

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void setPwmGen(     uint32_t PWM_PIN_MUX, uint32_t PWM_GEN, uint32_t CLOCK_DIV, uint32_t PORT_BASE_ADDRESS, uint32_t PIN_BIT_MASK );

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void pwmGenWrite(   uint32_t PWM_GEN, uint32_t PWM_OUT, uint32_t PWM_BIT_MASK, uint16_t WIDTH_TICKS_16, uint16_t PERIOD_TICKS_16 );

  bool isPwmGenValid( uint8_t pin );

} // end namespace roveware ///////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // ROVE_PWM_GEN_HARDWARE_H