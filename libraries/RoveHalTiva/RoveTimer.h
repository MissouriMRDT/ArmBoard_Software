///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Launchpad Timer Interrupt Module (Timer ISR)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_TIMER_H
#define ROVE_TIMER_H

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo...Note: These two should be TivaWare functions... 
// but were never implemented by the TivaWare DriverLib team => TimerValueGet returned a 16 bit timer, 
// not the full 24 bit timer, TimerConfigure stomps on sthe other half AB timer
// Come on TI wtf...
// Todo timer_clock_source = 120Mhz SYSCLOCK or 16Mhz PIOSC
// Todo 120Mhz / ( micros / sec )
// Todo  16Mhz / ( micros / sec )

#include <stdint.h> // wtf? => must be included BEFORE driverlib/timer.h ?

#include "driverlib/timer.h"

uint32_t TimerValueGet24(    uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB ); /////////////////////////
void     TimerConfigure24AB( uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB, uint32_t TIMER_CONFIGURE );

namespace roveware ///////////////////////////////////////////////////////////////////////////////////////////////////////
{
  typedef void (*isrPtr)( void );

  enum  timer_clock_source { TIMER_USE_SYSCLOCK = 0x00000000, 
                             TIMER_USE_PIOSC    = 0x00000001 };

  const int      SYSCLOCK_TICKS_PER_MICRO   = ( 120000000 / 1000000 );
  const int      PIOSC_TICKS_PER_MICRO      = (  16000000 / 1000000 );

  const uint32_t TIMER_USE_PERIODIC_UP_AB   = ( TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP | TIMER_CFG_B_PERIODIC_UP );
  const uint32_t TIMER_USE_CAPTURE_TICKS_AB = ( TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME    | TIMER_CFG_B_CAP_TIME);


  struct TimerHardware //////////////////
  {
    volatile uint32_t TIMER_PERIPHERAL;
    volatile uint32_t TIMER_BASE_ADDRESS;
    volatile uint32_t TIMER_CHANNEL_AB;
    volatile uint32_t TIMER_INTERRUPT;
  };

  struct Timer /////////////////////////////
  {
    volatile uint32_t      interrupt_source;
    volatile uint32_t      period_ticks;
    struct   TimerHardware Hw;

    void ( *timerIsr )(     void );
    void ( *userFunction )( void );
  };

  void setupTimerHardware(  uint32_t TIMER_CLOCK_SOURCE, ////
                            uint32_t TIMER_CONFIGURE,
                            uint32_t TIMER_PERIPHERAL,
                            uint32_t TIMER_BASE_ADDRESS,
                            uint32_t TIMER_CHANNEL_AB );

  void attachTimerIsr(      uint32_t TIMER_BASE_ADDRESS, ///
                            uint32_t TIMER_CHANNEL_AB,
                            uint32_t TIMER_INTERRUPT_SOURCE,
                            uint32_t TIMER_INTERRUPT,
                            void  ( *timerIsr )( void ),
                            uint8_t  priority );

  void loadTimer(           uint32_t TIMER_BASE_ADDRESS, ////
                            uint32_t TIMER_CHANNEL_AB,
                            uint32_t TIMER_INTERRUPT_SOURCE,
                            uint32_t TIMER_PERIOD_TICKS_24 );

  void enableTimer(         uint32_t TIMER_BASE_ADDRESS,  ////
                            uint32_t TIMER_INTERRUPT_SOURCE );

  void startTimer(          uint32_t TIMER_BASE_ADDRESS, /////
                            uint32_t TIMER_CHANNEL_AB,
                            uint32_t TIMER_INTERRUPT_SOURCE,
                            uint32_t TIMER_PERIOD_TICKS_24 );

  void stopTimer(           uint32_t TIMER_BASE_ADDRESS,
                            uint32_t TIMER_INTERRUPT_SOURCE );

  void captureBothEdges(    uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB );
  void captureRisingEdges(  uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB );
  void captureFallingEdges( uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB );

  
  uint8_t              pinToTimer(             uint8_t  pin ); ////////////
  bool                 isTimerValid(           uint8_t  timer );
  bool                 isPeriodTicks24Valid(   uint32_t period_ticks_24 ) ;



  struct TimerHardware lookupTimerHardware (   uint8_t  timer ); /////
  isrPtr               lookupTimerIsrPeriodic( uint8_t  timer );
  void                 attachTimerHardware(    uint8_t  timer,   
                                               struct   Timer* Timer );
  void timerIsrPeriodic(                       struct   Timer* Timer );

  void dispatchTimerIsrPeriodic_0A ( void );
  void dispatchTimerIsrPeriodic_0B ( void );
  void dispatchTimerIsrPeriodic_1A ( void );
  void dispatchTimerIsrPeriodic_1B ( void );
  void dispatchTimerIsrPeriodic_2A ( void );
  void dispatchTimerIsrPeriodic_2B ( void );
  void dispatchTimerIsrPeriodic_3A ( void );
  void dispatchTimerIsrPeriodic_3B ( void );
  void dispatchTimerIsrPeriodic_4A ( void );
  void dispatchTimerIsrPeriodic_4B ( void );
  void dispatchTimerIsrPeriodic_5A ( void );
  void dispatchTimerIsrPeriodic_5B ( void );
  void dispatchTimerIsrPeriodic_6A ( void );
  void dispatchTimerIsrPeriodic_6B ( void );
  void dispatchTimerIsrPeriodic_7A ( void );
  void dispatchTimerIsrPeriodic_7B ( void );

}// end namespace roveware ////////////////////////////////////////////////////

#endif // ROVE_TIMER_H