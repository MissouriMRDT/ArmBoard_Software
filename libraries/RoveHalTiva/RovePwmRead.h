/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Capture (PWM CCP)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ROVE_PWM_READ_H
#define ROVE_PWM_READ_H

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo forward declare => instead of comments and .h file ?
// Todo validate pin and piority?
// Todo => wrap the ring buffer size template one level higher => roveware::CcpTicks < uint32_t, 16 > Ccp;
// Todo => all this goes IN roveware:: and then one more level of RoveHal include wrapper?
// Todo => Template the buffer size?
// Todo => int read(); // Duty in millipercent
// Timer                  => ti.com/lit/ds/symlink/tm4c1294ncpdt.pdf => 13-2.    General-Purpose Timers Signals
// Edge Time Capture Pins => ti.com/lit/ds/symlink/tm4c1294ncpdt.pdf => 13.3.3.4 Input Edge-Time Mode
// PD_0 or PA_0 or PL_4             // PIN_CONFLICTS    T0_A    Energia::analogWrite or roveware::PwmRead
// PD_1 or PA_1 or PL_5             // PIN_CONFLICTS    T0_B    Energia::analogWrite or RoveWare::PwmRead
// PD_2 or PA_2 or PL_6             // PIN_CONFLICTS    T1_A    Energia::analogWrite or RoveWare::PwmRead 
// PD_3 or PA_3 or PL_7             // PIN_CONFLICTS    T1_B    Energia::analogWrite or RoveWare::PwmRead
// PA_4 or PM_0 or                  // PIN_CONFLICTS    T2_A    Energia::analogWrite or RoveWare::PwmRead or Energia::Servo
// PA_5 or PM_1 or                  // PIN_CONFLICTS    T2_B    Energia::analogWrite or RoveWare::PwmRead or Energia::Servo
// PA_6 or PM_2 or PD_4             // PIN_CONFLICTS    T3_A    Energia::analogWrite or RoveWare::PwmRead
// PA_7 or PM_3 or PD_5             // PIN_CONFLICTS    T3_B    Energia::analogWrite or RoveWare::PwmRead
// PM_4 or PB_0 or PD_6             // PIN_CONFLICTS    T4_A    Energia::analogWrite or RoveWare::PwmRead
// PM_5 or PB_1 or PD_7             // PIN_CONFLICTS    T4_B    Energia::analogWrite or RoveWare::PwmRead
// PM_6 or PB_2 or                  // PIN_CONFLICTS    T5_A    Energia::analogWrite or RoveWare::PwmRead
// PM_7 or PB_3 or                  // PIN_CONFLICTS    T5_B    Energia::analogWrite or RoveWare::PwmRead
//                                  // NO_CONFLICTS   ( T6_A )  Pin not wired on TI TivaC1294XL Launchpad
//                                  // NO_CONFLICTS   ( T6_B )  Pin not wired on TI TivaC1294XL Launchpad
//                                  // NO_CONFLICTS   ( T7_A )  Pin not wired on TI TivaC1294XL Launchpad
//                                  // NO_CONFLICTS   ( T7_B )  Pin not wired on TI TivaC1294XL Launchpad

///////////////////////////////////////////////////////////////////////////////////////////////
#include "RoveTimerInterrupt.h"
#include "RoveBoardMap.h"
#include "RoveCcp.h"

#include <stdint.h>

//////////////////////////////////////////////
class RovePwmRead
{
public:

  void  attach( uint8_t pin, int priority=7 );
  void  start();
  void  stop();

  bool  isWireBroken();

  int   readHighWidthTicks();
  int   readLowWidthTicks();
  int   readPeriodTicks();

  int   readHighWidthMicros();
  int   readLowWidthMicros();
  int   readPeriodMicros();

  int   readHighWidthMillis();
  int   readLowWidthMillis();
  int   readPeriodMillis();

  int   readDutyDecipercent();
  float readDutyPercent();

  void attachUserIsr( roveware::userCcpTicksArgsIsrPtr userIsr );

// private:
  const   uint32_t  MAX_POSSIBLE_PERIOD_MICROS=1000000;
  struct  roveware::CcpTicks CcpTicks;
  uint8_t pin;
};

///////////////////////////////////////////////////////////////////////////////
class  RovePwmReadWireBreaks
{
public:
  void attach(       uint8_t timer, int priority=7 );
  void attachMillis( uint8_t timer, int period_millis=100,    int priority=7 );
  void attachMicros( uint8_t timer, int period_micros=100000, int priority=7 );

  void start();
  void stop();

// private:
  struct RoveTimerInterrupt    AllWireBreaksTimer;
};

#endif // ROVE_PWM_READ_H