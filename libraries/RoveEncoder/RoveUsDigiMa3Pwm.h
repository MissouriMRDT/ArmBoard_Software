///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Energia Texas Instruments Tiva C, US Digital Encoder, Pwm 12b Ma3
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_US_DIGI_MA3_PWM_H
#define ROVE_US_DIGI_MA3_PWM_H

#include "RovePwmRead.h"

#include <stdint.h>

class RoveUsDigiMa3Pwm /////////////////////////////////////////////////////
{
public:
  void  attach( uint8_t pin, int  priority                        =     7, 
                             bool auto_recalibrate                = false,
                             int  offset_millidegrees             =     0, 
                             bool invert                          = false,
                             int  read_decipercent_at_0_dgrees    =     0, 
                             int  read_decipercent_at_360_degrees =  1000 );
  void  start();
  void  stop();

  int   readMillidegrees();
  float readDegrees();
  float readRadians();
  bool  isWireBroken();

// private:
  RovePwmRead    PwmRead;
  uint8_t        pin;

  int  READ_DECIPECENT_AT_0_DEGREES;
  int  READ_DECIPECENT_AT_360_DEGREES;
  int  OFFSET_MILLIDEGREES;
  bool AUTO_RECALIBRATE;
  bool INVERT_READING;

};

class RoveUsDigiMa3PwmWireBreaks ///////////////////////////////////////
{
  public: 
  void attach(       uint8_t timer, int priority=7 );
  void attachMillis( uint8_t timer, int period_millis, int priority=7 );
  void attachMicros( uint8_t timer, int period_micros, int priority=7 );

  void start();
  void stop();

  RovePwmReadWireBreaks    WireBreaks;
};

#endif // ROVE_US_DIGI_MA3_PWM_H