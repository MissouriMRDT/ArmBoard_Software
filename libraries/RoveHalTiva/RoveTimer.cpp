/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Launchpad Timer Interrupt Module (Timer ISR)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo...Note: These two should be TivaWare functions... 
// but were never implemented by the TivaWare DriverLib team => TimerValueGet returned a 16 bit timer, 
// not the full 24 bit timer, TimerConfigure stomps on sthe other half AB timer
// Come on TI wtf...
// Todo configure a and b at same time
// Todo configure one side only a or b
// 'TxCCPx'                              => Tiva C Series TM4C1294NCPDT Microcontroller Data Sheet (Rev. B) defines the module top down register hierarchy
//                                       => ti.com/lit/ds/symlink/tm4c1294ncpdt.pdf
// SYSCTL_PERIPH_TIMER0, TIMER_A, TIMER_TIMA_TIMEOUT => TivaWare Peripheral Driver Library provides todo
//                                                   => ti.com/lit/ug/spmu298d/spmu298d.pdf
// TIMER0_BASE, INT_TIMER0A,             => TivaWare Include todo provides todo
//                                       => tivaware/inc/

//////////////////////////
#include "RoveTimer.h"
#include "RoveBoardMap.h"

#include <stdint.h> // wtf? => must be included BEFORE driverlib/timer.h ?

#include "Energia.h" /* Todo => reeally ? ...allllthese include ? */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "tm4c1294ncpdt.h"

uint32_t TimerValueGet24( uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB ) ///////////////////////
{ 
  if (      TIMER_CHANNEL_AB == TIMER_A ) 
  { return (uint32_t)( ( ( HWREG( TIMER_BASE_ADDRESS + TIMER_O_TAPS ) << 16 ) & 0x00FF0000 )
                   + (   ( HWREG( TIMER_BASE_ADDRESS + TIMER_O_TAR  )       ) & 0x0000FFFF ) ); }

  else if ( TIMER_CHANNEL_AB == TIMER_B ) 
  { return (uint32_t)( ( ( HWREG( TIMER_BASE_ADDRESS + TIMER_O_TBPS ) << 16 ) & 0x00FF0000 )
                   + (   ( HWREG( TIMER_BASE_ADDRESS + TIMER_O_TBR  )       ) & 0x0000FFFF ) ); }
}

void TimerConfigure24AB( uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB, uint32_t TIMER_CONFIGURE )
{
  if(      TIMER_CHANNEL_AB  == TIMER_A ) 
  { HWREG( TIMER_BASE_ADDRESS + TIMER_O_CTL  ) &=  ~( TIMER_CTL_TAEN );
    HWREG( TIMER_BASE_ADDRESS + TIMER_O_CFG  )  =     TIMER_CFG_SPLIT_PAIR           >> 24;
    HWREG( TIMER_BASE_ADDRESS + TIMER_O_TAMR )  = ((( TIMER_CONFIGURE & 0x000F0000 ) >> 4 )
                                                  | ( TIMER_CONFIGURE & 0xFF ) // Todo => 0x000000FF
                                                  |   TIMER_TAMR_TAPWMIE ); }
  else if( TIMER_CHANNEL_AB == TIMER_B )  
  { HWREG( TIMER_BASE_ADDRESS + TIMER_O_CTL  ) &=   ~( TIMER_CTL_TBEN );
    HWREG( TIMER_BASE_ADDRESS + TIMER_O_CFG  )  =      TIMER_CFG_SPLIT_PAIR           >> 24;
    HWREG( TIMER_BASE_ADDRESS + TIMER_O_TBMR )  =  ((( TIMER_CONFIGURE & 0x00F00000 ) >> 8 ) 
                                                  | (( TIMER_CONFIGURE                >> 8 ) & 0xFF ) // Todo => 0x000000FF
                                                  |    TIMER_TBMR_TBPWMIE ); }
}

namespace roveware /////////////////////////////////////////////////////////////////////////////////////////
{
  struct Timer* Timers[16] = {};

  bool isTimerValid( uint8_t timer ) ////////////////////////////////////////
  {          return (        timer >  INVALID ) and ( timer <= MAX_TIMER); }

  bool isPeriodTicks24Valid( uint32_t period_ticks_24 ) /////////////
  {                            return period_ticks_24 <= 0x00FFFFFF; }

  void startTimer(    uint32_t TIMER_BASE_ADDRESS, ////////////////////////////////////////////////////////
                      uint32_t TIMER_CHANNEL_AB,
                      uint32_t TIMER_INTERRUPT_SOURCE,
                      uint32_t TIMER_PERIOD_TICKS_24 )
  { if( isPeriodTicks24Valid ( TIMER_PERIOD_TICKS_24 ) )
    { stopTimer(   TIMER_BASE_ADDRESS, TIMER_INTERRUPT_SOURCE );
      loadTimer(   TIMER_BASE_ADDRESS, TIMER_CHANNEL_AB, TIMER_INTERRUPT_SOURCE, TIMER_PERIOD_TICKS_24 );
      enableTimer( TIMER_BASE_ADDRESS, TIMER_INTERRUPT_SOURCE ); }
  }

  void loadTimer(     uint32_t TIMER_BASE_ADDRESS, ////////////////////////////////////////////////////////
                      uint32_t TIMER_CHANNEL_AB,
                      uint32_t TIMER_INTERRUPT_SOURCE,
                      uint32_t TIMER_PERIOD_TICKS_24 )
  { if( isPeriodTicks24Valid ( TIMER_PERIOD_TICKS_24 ) )
    { TimerPrescaleSet( TIMER_BASE_ADDRESS, TIMER_CHANNEL_AB, ( TIMER_PERIOD_TICKS_24 >> 16 ) & 0x000000FF);
      TimerLoadSet(     TIMER_BASE_ADDRESS, TIMER_CHANNEL_AB, ( TIMER_PERIOD_TICKS_24 )       & 0x0000FFFF); }
  }

  void enableTimer(         uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_INTERRUPT_SOURCE ) //
  { TimerIntEnable(                  TIMER_BASE_ADDRESS,          TIMER_INTERRUPT_SOURCE ); }

  void captureBothEdges(    uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB ) /////////////////////////
  { TimerControlEvent(               TIMER_BASE_ADDRESS,          TIMER_CHANNEL_AB, TIMER_EVENT_BOTH_EDGES ); }

  void captureRisingEdges(  uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB ) ////////////////////////
  { TimerControlEvent(               TIMER_BASE_ADDRESS,          TIMER_CHANNEL_AB, TIMER_EVENT_POS_EDGE ); }

  void captureFallingEdges( uint32_t TIMER_BASE_ADDRESS, uint32_t TIMER_CHANNEL_AB ) ////////////////////////
  { TimerControlEvent(               TIMER_BASE_ADDRESS,          TIMER_CHANNEL_AB, TIMER_EVENT_NEG_EDGE ); }

  void stopTimer( uint32_t TIMER_BASE_ADDRESS, //////////////////
                  uint32_t TIMER_INTERRUPT_SOURCE )
  {
    TimerIntClear(   TIMER_BASE_ADDRESS, TIMER_INTERRUPT_SOURCE );
    TimerIntDisable( TIMER_BASE_ADDRESS, TIMER_INTERRUPT_SOURCE );
  }

  void attachTimerHardware( uint8_t timer, struct Timer* Timer )
  {                        Timers [ timer - 1 ] =        Timer; }

  void setupTimerHardware( uint32_t TIMER_CLOCK_SOURCE, ///////////////////////////////////////////////
                           uint32_t TIMER_CONFIGURE,
                           uint32_t TIMER_PERIPHERAL,
                           uint32_t TIMER_BASE_ADDRESS,
                           uint32_t TIMER_CHANNEL_AB )
  {
            SysCtlPeripheralEnable( TIMER_PERIPHERAL );
    while( !SysCtlPeripheralReady(  TIMER_PERIPHERAL ) )
    {   }; 

    TimerClockSourceSet( TIMER_BASE_ADDRESS, TIMER_CLOCK_SOURCE );

    if ( TIMER_CHANNEL_AB == TIMER_BOTH ){ TimerConfigure(     TIMER_BASE_ADDRESS, TIMER_CONFIGURE ); }
    else {                                 TimerConfigure24AB( TIMER_BASE_ADDRESS, TIMER_CHANNEL_AB, 
                                                                                   TIMER_CONFIGURE ); }
  }

  void attachTimerIsr( uint32_t TIMER_BASE_ADDRESS, ////////////////////////
                       uint32_t TIMER_CHANNEL_AB,
                       uint32_t TIMER_INTERRUPT_SOURCE,
                       uint32_t TIMER_INTERRUPT,
                       void(*timerIsr)(void),
                       uint8_t priority )
  {
    TimerIntRegister( TIMER_BASE_ADDRESS,  TIMER_CHANNEL_AB,    timerIsr );
    TimerEnable(      TIMER_BASE_ADDRESS,  TIMER_CHANNEL_AB );
    TimerIntEnable(   TIMER_BASE_ADDRESS,  TIMER_INTERRUPT_SOURCE);
    TimerIntClear(    TIMER_BASE_ADDRESS,  TIMER_INTERRUPT_SOURCE); 
    IntPrioritySet(   TIMER_INTERRUPT,   ( priority << 5 ));
    IntEnable(        TIMER_INTERRUPT );
    IntMasterEnable(); 
  }

  void timerIsrPeriodic( struct Timer* Timer ) //////////////////////////////
  {
    TimerIntClear(   Timer->Hw.TIMER_BASE_ADDRESS, Timer->interrupt_source );
    TimerIntDisable( Timer->Hw.TIMER_BASE_ADDRESS, Timer->interrupt_source );
    
    Timer->userFunction();
    
    TimerIntClear(  Timer->Hw.TIMER_BASE_ADDRESS, Timer->interrupt_source );
    TimerIntEnable( Timer->Hw.TIMER_BASE_ADDRESS, Timer->interrupt_source );
  }

  uint8_t pinToTimer( uint8_t pin ) ////////////////////////////////////////////////
  {
          if( ( pin == PD_0 ) ||( pin == PA_0 ) ||( pin == PL_4 ) ) { return T0_A; }
    else  if( ( pin == PD_1 ) ||( pin == PA_1 ) ||( pin == PL_5 ) ) { return T0_B; }
    else  if( ( pin == PD_2 ) ||( pin == PA_2 ) ||( pin == PL_6 ) ) { return T1_A; }
    else  if( ( pin == PD_3 ) ||( pin == PA_3 ) ||( pin == PL_7 ) ) { return T1_B; }
    else  if( ( pin == PA_4 ) ||( pin == PM_0 )                   ) { return T2_A; }
    else  if( ( pin == PA_5 ) ||( pin == PM_1 )                   ) { return T2_B; }
    else  if( ( pin == PA_6 ) ||( pin == PM_2 ) ||( pin == PD_4 ) ) { return T3_A; }
    else  if( ( pin == PA_7 ) ||( pin == PM_3 ) ||( pin == PD_5 ) ) { return T3_B; }
    else  if( ( pin == PM_4 ) ||( pin == PB_0 ) ||( pin == PD_6 ) ) { return T4_A; }
    else  if( ( pin == PM_5 ) ||( pin == PB_1 ) ||( pin == PD_7 ) ) { return T4_B; }
    else  if( ( pin == PM_6 ) ||( pin == PB_2 )                   ) { return T5_A; }
    else  if( ( pin == PM_7 ) ||( pin == PB_3 )                   ) { return T5_B; }
    else                                                            { return    0; }
  }

  struct TimerHardware lookupTimerHardware ( uint8_t timer ) ////////////////////////////////////////
  {
         if ( timer == T0_A ) { return { SYSCTL_PERIPH_TIMER0, TIMER0_BASE, TIMER_A, INT_TIMER0A }; }
    else if ( timer == T0_B ) { return { SYSCTL_PERIPH_TIMER0, TIMER0_BASE, TIMER_B, INT_TIMER0B }; }
    else if ( timer == T1_A ) { return { SYSCTL_PERIPH_TIMER1, TIMER1_BASE, TIMER_A, INT_TIMER1A }; }
    else if ( timer == T1_B ) { return { SYSCTL_PERIPH_TIMER1, TIMER1_BASE, TIMER_B, INT_TIMER1B }; }
    else if ( timer == T2_A ) { return { SYSCTL_PERIPH_TIMER2, TIMER2_BASE, TIMER_A, INT_TIMER2A }; }
    else if ( timer == T2_B ) { return { SYSCTL_PERIPH_TIMER2, TIMER2_BASE, TIMER_B, INT_TIMER2B }; }
    else if ( timer == T3_A ) { return { SYSCTL_PERIPH_TIMER3, TIMER3_BASE, TIMER_A, INT_TIMER3A }; }
    else if ( timer == T3_B ) { return { SYSCTL_PERIPH_TIMER3, TIMER3_BASE, TIMER_B, INT_TIMER3B }; }
    else if ( timer == T4_A ) { return { SYSCTL_PERIPH_TIMER4, TIMER4_BASE, TIMER_A, INT_TIMER4A }; }
    else if ( timer == T4_B ) { return { SYSCTL_PERIPH_TIMER4, TIMER4_BASE, TIMER_B, INT_TIMER4B }; }
    else if ( timer == T5_A ) { return { SYSCTL_PERIPH_TIMER5, TIMER5_BASE, TIMER_A, INT_TIMER5A }; }
    else if ( timer == T5_B ) { return { SYSCTL_PERIPH_TIMER5, TIMER5_BASE, TIMER_B, INT_TIMER5B }; }
    else if ( timer == T6_A ) { return { SYSCTL_PERIPH_TIMER6, TIMER6_BASE, TIMER_A, INT_TIMER6A }; }
    else if ( timer == T6_B ) { return { SYSCTL_PERIPH_TIMER6, TIMER6_BASE, TIMER_B, INT_TIMER6B }; }
    else if ( timer == T7_A ) { return { SYSCTL_PERIPH_TIMER7, TIMER7_BASE, TIMER_A, INT_TIMER7A }; }
    else if ( timer == T7_B ) { return { SYSCTL_PERIPH_TIMER7, TIMER7_BASE, TIMER_B, INT_TIMER7B }; }
    else                      { return { 0,                    0,           0,       0           }; }
  }

  isrPtr lookupTimerIsrPeriodic( uint8_t timer ) ////////////////////
  {
         if ( timer == T0_A ) { return dispatchTimerIsrPeriodic_0A; }
    else if ( timer == T0_B ) { return dispatchTimerIsrPeriodic_0B; }
    else if ( timer == T1_A ) { return dispatchTimerIsrPeriodic_1A; }
    else if ( timer == T1_B ) { return dispatchTimerIsrPeriodic_1B; }
    else if ( timer == T2_A ) { return dispatchTimerIsrPeriodic_2A; }
    else if ( timer == T2_B ) { return dispatchTimerIsrPeriodic_2B; }
    else if ( timer == T3_A ) { return dispatchTimerIsrPeriodic_3A; }
    else if ( timer == T3_B ) { return dispatchTimerIsrPeriodic_3B; }
    else if ( timer == T4_A ) { return dispatchTimerIsrPeriodic_4A; }
    else if ( timer == T4_B ) { return dispatchTimerIsrPeriodic_4B; }
    else if ( timer == T5_A ) { return dispatchTimerIsrPeriodic_5A; }
    else if ( timer == T5_B ) { return dispatchTimerIsrPeriodic_5B; }
    else if ( timer == T6_A ) { return dispatchTimerIsrPeriodic_6A; }
    else if ( timer == T6_B ) { return dispatchTimerIsrPeriodic_6B; }
    else if ( timer == T7_A ) { return dispatchTimerIsrPeriodic_7A; }
    else if ( timer == T7_B ) { return dispatchTimerIsrPeriodic_7B; }
    else                      { return                           0; }
  }

  void dispatchTimerIsrPeriodic_0A ( void ) { timerIsrPeriodic( Timers[ T0_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_0B ( void ) { timerIsrPeriodic( Timers[ T0_B - 1 ] ); }
  void dispatchTimerIsrPeriodic_1A ( void ) { timerIsrPeriodic( Timers[ T1_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_1B ( void ) { timerIsrPeriodic( Timers[ T1_B - 1 ] ); }
  void dispatchTimerIsrPeriodic_2A ( void ) { timerIsrPeriodic( Timers[ T2_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_2B ( void ) { timerIsrPeriodic( Timers[ T2_B - 1 ] ); }
  void dispatchTimerIsrPeriodic_3A ( void ) { timerIsrPeriodic( Timers[ T3_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_3B ( void ) { timerIsrPeriodic( Timers[ T3_B - 1 ] ); }
  void dispatchTimerIsrPeriodic_4A ( void ) { timerIsrPeriodic( Timers[ T4_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_4B ( void ) { timerIsrPeriodic( Timers[ T4_B - 1 ] ); }
  void dispatchTimerIsrPeriodic_5A ( void ) { timerIsrPeriodic( Timers[ T5_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_5B ( void ) { timerIsrPeriodic( Timers[ T5_B - 1 ] ); }
  void dispatchTimerIsrPeriodic_6A ( void ) { timerIsrPeriodic( Timers[ T6_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_6B ( void ) { timerIsrPeriodic( Timers[ T6_B - 1 ] ); }
  void dispatchTimerIsrPeriodic_7A ( void ) { timerIsrPeriodic( Timers[ T7_A - 1 ] ); }
  void dispatchTimerIsrPeriodic_7B ( void ) { timerIsrPeriodic( Timers[ T7_B - 1 ] ); }
}// end namespace
