///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C 1294/129E Launchpad Timer Interrupt Module (Timer ISR)
// Todo      => add support for the low frequency oscillator as a clock source (presently only supports the system clock source or the precision oscillator clock source)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RoveTimerInterrupt.h"
#include "RoveTimer.h"

#include <stdint.h> // wtf? => must be included BEFORE driverlib/timer.h ?

#include "inc/hw_memmap.h" /* Todo? */
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "tm4c1294ncpdt.h"

void RoveTimerInterrupt::setupTimer( uint8_t timer ) ///////////////////////////////
{         this->timer =                      timer;
  if(   ( this->timer %  2) == 0 ){
          this->Timer.interrupt_source = TIMER_TIMB_TIMEOUT; }
  else {  this->Timer.interrupt_source = TIMER_TIMA_TIMEOUT; }
          this->Timer.Hw       = roveware::lookupTimerHardware(    this->timer );
          this->Timer.timerIsr = roveware::lookupTimerIsrPeriodic( this->timer );
          roveware::attachTimerHardware(                           this->timer, 
                                                                &( this->Timer ) );
}

void RoveTimerInterrupt::attachMicros( void( *userFunction )( void ), ////////////////
                                      uint8_t timer, int period_micros, int priority )
{ if ( roveware::isTimerValid(                timer ) )
  { this->setupTimer(                         timer );

    this->Timer.period_ticks = period_micros * roveware::PIOSC_TICKS_PER_MICRO;
    this->Timer.userFunction = userFunction;

    roveware::setupTimerHardware( roveware::TIMER_USE_PIOSC,
                                  roveware::TIMER_USE_PERIODIC_UP_AB,
                                  this->Timer.Hw.TIMER_PERIPHERAL,
                                  this->Timer.Hw.TIMER_BASE_ADDRESS,
                                  this->Timer.Hw.TIMER_CHANNEL_AB );

    roveware::attachTimerIsr(     this->Timer.Hw.TIMER_BASE_ADDRESS,
                                  this->Timer.Hw.TIMER_CHANNEL_AB,
                                  this->Timer.interrupt_source,
                                  this->Timer.Hw.TIMER_INTERRUPT,
                                  this->Timer.timerIsr, 
                                  priority ); }
    this->stop();
}

void RoveTimerInterrupt::attachMillis( void( *userFunction )( void ), uint8_t timer,  int period_millis, int priority ) //
{  this->attachMicros(                        userFunction,                   timer, 1000*period_millis,     priority ); }

void RoveTimerInterrupt::start() /////////////////////////////////
{  if ( roveware::isTimerValid( this->timer ) )
   {    roveware::startTimer(   this->Timer.Hw.TIMER_BASE_ADDRESS,
                                this->Timer.Hw.TIMER_CHANNEL_AB,
                                this->Timer.interrupt_source,
                                this->Timer.period_ticks ); } 
}

void RoveTimerInterrupt::stop() //////////////////////////////////
{  if ( roveware::isTimerValid( this->timer ) )
  {     roveware::stopTimer(    this->Timer.Hw.TIMER_BASE_ADDRESS,
                                this->Timer.interrupt_source ); } 
}