/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Generator (PWM Gen)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// Todo clock_div? enum hmmm...
// Todo forward declare => instead of comments and .h file ?
// Todo => PWM_SYSCLK_DIV_64 this goes IN roveware:: 
// and then one more level of RoveHal include wrapper?
// void rovePwmDutyWrite( int pin, duty_milli_percent );
// {    rovePwmMicrosWrite(     pin, ( (2040 * value) / 255 ), 2040 ); }
// Todo Overload these
// void rovePwmDutyWrite( int pin, duty_milli_percent, period_micros ) ...
// void rovePwmHzWrite(    int pin, int pulse_width_hz) ////////////////////
// {    rovePwmMicrosWrite(    pin, ( ? ); }
// void rovePwmHzWrite(    int pin, int pulse_width_hz, int pulse_period_hz )
// {    rovePwmMicrosWrite(    pin, ( ? ); }
// etc...

///////////////////////////////////////////////////////////////////////////////////////
#include "RovePwmWrite.h"

#include "RovePwmGen.h"

#include "RoveTimer.h" // Todo => move roveware::SYSCLOCK_TICKS_PER_MICRO to map file?

#include <stdint.h> // wtf? => must be included BEFORE driverlib/timer.h ?

#include "Energia.h"       // Todo?
#include "driverlib/pwm.h" // Todo?

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rovePwmMicrosWrite( uint8_t pin, int pulse_width_micros, int pulse_period_micros )
{
  if( roveware::isPwmGenValid( pin ) )
  { uint32_t  pulse_width_ticks_16  = ( pulse_width_micros  * roveware::SYSCLOCK_TICKS_PER_MICRO ) / 64;
    uint32_t  pulse_period_ticks_16 = ( pulse_period_micros * roveware::SYSCLOCK_TICKS_PER_MICRO ) / 64;

    if (     pulse_width_ticks_16 == 0 )
    { digitalWrite( pin, LOW  ); }

    else if ( pulse_width_ticks_16 >= pulse_period_ticks_16 )
    { digitalWrite( pin, HIGH ); }

    else 
    { struct roveware::PwmGenHardware Hw = roveware::pwmGenHardware( pin );
      roveware::setPwmGen   ( Hw.PWM_GEN_PIN_MUX, Hw.PWM_GEN, roveware::PWM_USE_DIVIDE_CLOCK_BY_64,  Hw.PORT_BASE_ADDRESS, Hw.PIN_BIT_MASK );
      roveware::pwmGenWrite ( Hw.PWM_GEN,         Hw.PWM_OUT, Hw.PWM_BIT_MASK, pulse_width_ticks_16, pulse_period_ticks_16 ); }
} }

///////////////////////////////////////////////////////////////////////////
void rovePwmAnalogWrite( uint8_t pin,        int value )
{    rovePwmMicrosWrite(         pin, ( ( 2040 * value ) / 255 ), 2040 ); }