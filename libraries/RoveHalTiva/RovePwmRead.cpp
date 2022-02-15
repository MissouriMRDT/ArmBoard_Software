/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Capture (PWM CCP)                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo move to a Gpio struct?
// Todo 2^24 - 1
// Todo pinMode unwrapped as drivelib calls?

#include "RovePwmRead.h"
#include "RoveTimerInterrupt.h"
#include "RoveCcp.h"
#include "RoveTimer.h"

#include <stdint.h> // wtf? => must be included BEFORE driverlib/timer.h ?

#include "Energia.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

void RovePwmRead::attach( uint8_t pin, int priority ) ///////////////////////////////////////////////////////////
{
  this->pin                          = pin;
  this->CcpTicks.priority            = priority;
  this->CcpTicks.timer               = roveware::pinToTimer( this->pin );

  if ( ( this->CcpTicks.timer %  2) == 0 ){ this->CcpTicks.Timer.interrupt_source = TIMER_CFG_B_CAP_TIME; }
  else {                                    this->CcpTicks.Timer.interrupt_source = TIMER_CFG_A_CAP_TIME; }

  this->CcpTicks.Hw.PORT_BASE_ADDRESS = (uint32_t)portBASERegister( digitalPinToPort( this->pin ) );
  this->CcpTicks.Hw.PIN_BIT_MASK      =                          digitalPinToBitMask( this->pin   );
  this->CcpTicks.Hw.CCP_PIN_MUX       = roveware::ccpPinMux(                          this->pin   );
  this->CcpTicks.Timer.Hw             = roveware::lookupTimerHardware(                this->CcpTicks.timer );
  this->CcpTicks.Timer.timerIsr       = roveware::lookupCcpTicksIsr(                  this->CcpTicks.timer );
  this->CcpTicks.Timer.period_ticks   = RovePwmRead::MAX_POSSIBLE_PERIOD_MICROS * roveware::PIOSC_TICKS_PER_MICRO;

  roveware::attachCcpTicks( this->CcpTicks.timer, &( this->CcpTicks ) );
}

void RovePwmRead::attachUserIsr( roveware::userCcpTicksArgsIsrPtr userIsr )
{       roveware::attachUserCcpTicksIsr( &( this->CcpTicks ),     userIsr );  }

void RovePwmRead::start() //////////////////////////////////////////////////////
{ 
  if( ( roveware::isTimerValid(          this->CcpTicks.timer ) )
  &&  ( roveware::isPeriodTicks24Valid ( this->CcpTicks.Timer.period_ticks ) ) )
  {
    pinMode(                  this->pin, INPUT);
    GPIOPinTypeTimer(         this->CcpTicks.Hw.PORT_BASE_ADDRESS,
                              this->CcpTicks.Hw.PIN_BIT_MASK);

    GPIOPinConfigure(         this->CcpTicks.Hw.CCP_PIN_MUX );
    
    uint32_t TIMER_CHANNEL_AB;
    if ( ( this->CcpTicks.timer %  2) == 0 ){ TIMER_CHANNEL_AB = TIMER_B; }
    else                                    { TIMER_CHANNEL_AB = TIMER_A; }
                   
    roveware::setupTimerHardware( roveware::TIMER_USE_PIOSC,
                                  roveware::TIMER_USE_CAPTURE_TICKS_AB,
                                  this->CcpTicks.Timer.Hw.TIMER_PERIPHERAL,
                                  this->CcpTicks.Timer.Hw.TIMER_BASE_ADDRESS,
                                  TIMER_CHANNEL_AB );

    roveware::captureBothEdges(   this->CcpTicks.Timer.Hw.TIMER_BASE_ADDRESS,
                                  this->CcpTicks.Timer.Hw.TIMER_CHANNEL_AB );

    roveware::attachTimerIsr(     this->CcpTicks.Timer.Hw.TIMER_BASE_ADDRESS,
                                  this->CcpTicks.Timer.Hw.TIMER_CHANNEL_AB,
                                  this->CcpTicks.Timer.interrupt_source,
                                  this->CcpTicks.Timer.Hw.TIMER_INTERRUPT,
                                  this->CcpTicks.Timer.timerIsr,
                                  this->CcpTicks.priority );

    roveware::startTimer(         this->CcpTicks.Timer.Hw.TIMER_BASE_ADDRESS,
                                  this->CcpTicks.Timer.Hw.TIMER_CHANNEL_AB,
                                  this->CcpTicks.Timer.interrupt_source,
                                  this->CcpTicks.Timer.period_ticks ); }
}

void RovePwmRead::stop() ///////////////////////////////////////////////////////////////////////////////////////////
{ if ( roveware::isTimerValid( this->CcpTicks.timer ) )
  {    roveware::stopTimer(    this->CcpTicks.Timer.Hw.TIMER_BASE_ADDRESS, this->CcpTicks.Timer.interrupt_source ); }
}

bool  RovePwmRead::isWireBroken() ////////////////////////////////////////
{ return roveware::isCcpWireBroken( roveware::pinToTimer( this->pin ) ); }

int   RovePwmRead::readHighWidthTicks()  { return this->CcpTicks.PulseWidthsHigh.average(); } ///////////////////
int   RovePwmRead::readLowWidthTicks()   { return this->CcpTicks.PulseWidthsLow.average();  }
int   RovePwmRead::readPeriodTicks()     { return this->readHighWidthTicks() + this->readLowWidthTicks(); }

int   RovePwmRead::readHighWidthMicros() { return this->readHighWidthTicks() / roveware::PIOSC_TICKS_PER_MICRO; }
int   RovePwmRead::readLowWidthMicros()  { return this->readLowWidthTicks()  / roveware::PIOSC_TICKS_PER_MICRO; }
int   RovePwmRead::readPeriodMicros()    { return this->readPeriodTicks()    / roveware::PIOSC_TICKS_PER_MICRO; }

int   RovePwmRead::readHighWidthMillis() { return this->readHighWidthMicros() / 1000; }
int   RovePwmRead::readLowWidthMillis()  { return this->readLowWidthMicros()  / 1000; }
int   RovePwmRead::readPeriodMillis()    { return this->readPeriodMicros()    / 1000; }

int   RovePwmRead::readDutyDecipercent() { return ( 1000   * this->readHighWidthTicks() ) / this->readPeriodTicks(); }
float RovePwmRead::readDutyPercent()     { return            this->readDutyDecipercent()  / 10.0; }

void RovePwmReadWireBreaks::attach(                           uint8_t timer, int  priority ) //
{  this->AllWireBreaksTimer.attachMillis( roveware::ccpWireBreaksIsr, timer, 100, priority ); }

void RovePwmReadWireBreaks::attachMillis(                      uint8_t timer, int period_millis, int priority ) //
{   this->AllWireBreaksTimer.attachMillis( roveware::ccpWireBreaksIsr, timer,     period_millis,     priority ); }

void RovePwmReadWireBreaks::attachMicros(                      uint8_t timer, int period_micros, int priority )
{   this->AllWireBreaksTimer.attachMicros( roveware::ccpWireBreaksIsr, timer,     period_micros,     priority ); }

void RovePwmReadWireBreaks::start() { this->AllWireBreaksTimer.start(); }
void RovePwmReadWireBreaks::stop()  { this->AllWireBreaksTimer.stop();  }
