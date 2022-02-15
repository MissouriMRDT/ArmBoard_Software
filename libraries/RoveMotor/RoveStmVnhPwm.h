/////////////////////////////////////////////////////////////////////////////////////////////////////
// Last update MRDT Jan 2019                                                                       //
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_STM_VNH_PWM_H
#define ROVE_STM_VNH_PWM_H

#include "RoveBoardMap.h"

#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// USE_ENERGIA_ANALOG_WRITE  => Energia analogWrite acquires a general timer (0 ~ 255 units) @ 490 hz => 2040 uS period, 8uS minimum width
// USE_ROVE_PWM_ANALOG_WRITE => roveGenAnalogWrite  acquires a pwm generator (0 ~ 255 units) @ 490 hz => 2040 uS period, 8uS minimum width

/////////////////////////////////////////////////////////////////////////////
class RoveStmVnhPwm
{
public:
  enum PWM_MODE { USE_ENERGIA_ANALOG_WRITE  = 1,
                  USE_ROVE_PWM_ANALOG_WRITE = 2, };

  uint8_t pwm_mode              = INVALID;
  uint8_t ina_pin;
  uint8_t inb_pin;
  uint8_t pwm_pin;
  uint8_t adc_pin;
  bool    invert_motor          = false;
  int     scale_pwm_decipercent = 1000;
  int     scale_adc_milliamps   = 20000;

  bool isEnergiaAnalogWritePin( uint8_t pin );
  bool isPwmAnalogWritePin(     uint8_t pin );
  bool isPinValid(              uint8_t pin );

  void attach( uint8_t ina_pin,
               uint8_t inb_pin,
               uint8_t pwm_pin,
               bool    invert_motor        = false,
               int     bus_millivolts      = 12000,    // 12V bus
               int     scale_to_millivolts = 12000,    // 12V max = scale_to_millivolts / bus_millivolts => 100.0% scale
               uint8_t adc_pin             = INVALID , 
               int     scale_to_milliamps  = 20000 );  // 20A max
        
  void drive(  int     decipercent );
  void brake(  int     decipercent );

  void coast(); 
  int  readMilliamps();

  ////////////////////////////////////////////////////////////////////
  void writeCommand( bool ina_value, bool inb_value, int pwm_value );
};

#endif // ROVE_STM_VNH_PWM_H