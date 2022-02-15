//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Launchpad Timer Interrupt Module (Timer ISR)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_TIMER_INTERRUPT_H
#define ROVE_TIMER_INTERRUPT_H

/////////////////////////////////////////////////////////////
// Todo => timer_clock_source_t clock_source=USE_PIOSC );
// Todo => forward declare => instead of comments and .h file ?

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 'Attach' the userFunction to a 24 bit by timer_ab number (1~16)
// (16b) periodic halfwidth (+8b) prescale timer interrupt 

// T0_A             // PIN_CONFLICTS     PD_0 or PA_0 or PL_4     Energia::analogWrite or roveware::PwmRead
// T0_B             // PIN_CONFLICTS     PD_1 or PA_1 or PL_5     Energia::analogWrite or RoveWare::PwmRead
// T1_A             // PIN_CONFLICTS     PD_2 or PA_2 or PL_6     Energia::analogWrite or RoveWare::PwmRead 
// T1_B             // PIN_CONFLICTS     PD_3 or PA_3 or PL_7     Energia::analogWrite or RoveWare::PwmRead
// T2_A             // PIN_CONFLICTS     PA_4 or PM_0 or          Energia::analogWrite or RoveWare::PwmRead or Energia::Servo
// T2_B             // PIN_CONFLICTS     PA_5 or PM_1 or          Energia::analogWrite or RoveWare::PwmRead or Energia::Servo
// T3_A             // PIN_CONFLICTS     PA_6 or PM_2 or PD_4     Energia::analogWrite or RoveWare::PwmRead
// T3_B             // PIN_CONFLICTS     PA_7 or PM_3 or PD_5     Energia::analogWrite or RoveWare::PwmRead
// T4_A             // PIN_CONFLICTS     PM_4 or PB_0 or PD_6     Energia::analogWrite or RoveWare::PwmRead
// T4_B             // PIN_CONFLICTS     PM_5 or PB_1 or PD_7     Energia::analogWrite or RoveWare::PwmRead
// T5_A             // PIN_CONFLICTS     PM_6 or PB_2 or          Energia::analogWrite or RoveWare::PwmRead
// T5_B             // PIN_CONFLICTS     PM_7 or PB_3 or          Energia::analogWrite or RoveWare::PwmRead
// T6_A             // NO_CONFLICTS                               Pin not wired on TI TivaC1294XL Launchpad
// T6_B             // NO_CONFLICTS                               Pin not wired on TI TivaC1294XL Launchpad
// T7_A             // NO_CONFLICTS                               Pin not wired on TI TivaC1294XL Launchpad
// T7_B             // NO_CONFLICTS                               Pin not wired on TI TivaC1294XL Launchpad

#include "RoveBoardMap.h"
#include "RoveTimer.h"

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
class RoveTimerInterrupt
{
public:
  void attachMillis( void(*userFunction)(void), uint8_t timer,  int period_millis, int priority=7 );
  void attachMicros( void(*userFunction)(void), uint8_t timer , int period_micros, int priority=7 );
  void start();
  void stop();
  
// private:
  struct  roveware::Timer     Timer;
  uint8_t                     timer;
  void    setupTimer( uint8_t timer );
};

#endif // ROVE_TIMER_INTERRUPT_H