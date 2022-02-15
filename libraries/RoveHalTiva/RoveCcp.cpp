//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C Pin Module Capture and Compare Pin Timer ( CCP )
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo HWREG( Ccp->Hardware.PORT_BASE_ADDRESS + (GPIO_O_DATA + ( Ccp->Hardware.PIN_BIT_MASK << 2 )) ); // todo wut 2?
// Todo switch the prescaler to first everywhere?
// Todo prescale takes the last 8 bits
// Todoload register only holds first 16 bits
// Todo wire break clear on valid handling
// Todo Table 13-2. General-Purpose Timers Signals (128TQFP)
// Todo edge_captured = false
// Todo wire_broken   = true
// Todo CcpTicks = { 0 } ?
// Todo digitalRead, digitalWrite
// TxCCPx                                => Tiva C Series TM4C1294NCPDT Microcontroller Data Sheet (Rev. B) defines the module top down register hierarchy
//                                       => ti.com/lit/ds/symlink/tm4c1294ncpdt.pdf
// GPIO_PORTx_BASE, GPIO_PIN_x, PX_x     => Energia provides Wiring api lookup of TivaWare gpio port values by Energia pin_number number
//                                       => github.com/energia/Energia/blob/master/hardware/lm4f/variants/launchpad_129/pins_energia.h
// SYSCTL_PERIPH_TIMERX,GPIO_Pxx_TxCCPx.. => TivaWare Peripheral Driver Library provides pin_number peripheral mux, timer base, capture periph, and interrupt masks
// TIMERX_BASE, INT_TIMERXA, INT_TIMERXB => ti.com/lit/ug/spmu298d/spmu298d.pdf   
// TivaC_1294 support => Wiring 'style' Api,  progmem lookup table, conforming to the Arduino / Energia pin_map pattern

/////////////////////////////////
#include "RoveCcp.h"
#include "RoveTimer.h"
#include "RoveBoardMap.h"

#include <stdint.h>

#include "Energia.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"
#include "tm4c1294ncpdt.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
namespace roveware
{
  struct CcpTicks*    CcpTicks[12]           = {     0 }; // todo initializer?
  struct CcpWireBreak CcpWireBreak           = { false,  false,  false,  false,  false,  false,
                                                 false,  false,  false,  false,  false,  false,
                                                 false,  false,  false,  false,  false,  false,
                                                 false,  false,  false,  false,  false,  false };

  /////////////////////////////////////////
  bool isCcpValid( uint8_t pin )
  { return ccpPinMux(      pin ) && true; }

  ///////////////////////////////////////////////////////////////
  void attachCcpTicks( uint8_t timer, struct CcpTicks* CcpTick )
  {                 CcpTicks [ timer - 1 ] =           CcpTick; }

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  void attachUserCcpTicksIsr( struct CcpTicks* CcpTick, userCcpTicksArgsIsrPtr userCcpTicksIsr ) 
  {                                            CcpTick->userCcpTicksIsr      = userCcpTicksIsr; }

  void noUserCcpTicksIsr( void* DummyArg1, bool dummy_arg_2, uint32_t dummy_arg_3 ){ /* Dummy No Op */ }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void ccpTicksIsr( struct CcpTicks* CcpTicks )
  {
    stopTimer(         CcpTicks->Timer.Hw.TIMER_BASE_ADDRESS, CcpTicks->Timer.interrupt_source );
    loadTimer(         CcpTicks->Timer.Hw.TIMER_BASE_ADDRESS, CcpTicks->Timer.Hw.TIMER_CHANNEL_AB,  
                       CcpTicks->Timer.interrupt_source,      CcpTicks->Timer.period_ticks );
    ccpEdgeIsCaptured( CcpTicks->timer, true );

    bool     digital_read  = GPIOPinRead( CcpTicks->Hw.PORT_BASE_ADDRESS,    CcpTicks->Hw.PIN_BIT_MASK ) && true;
    uint32_t capture_ticks = CcpTicks->Timer.period_ticks - TimerValueGet24( CcpTicks->Timer.Hw.TIMER_BASE_ADDRESS,
                                                                             CcpTicks->Timer.Hw.TIMER_CHANNEL_AB );
    if( digital_read == HIGH ) // Rising edge interrupt, save low width ticks
    { CcpTicks->PulseWidthsHigh.pushToBack( capture_ticks ); } 

    else                       // Falling edge interrupt, save high width ticks
    { CcpTicks->PulseWidthsLow.pushToBack( capture_ticks ); }
 
    CcpTicks->userCcpTicksIsr( CcpTicks, digital_read, capture_ticks );
    enableTimer(               CcpTicks->Timer.Hw.TIMER_BASE_ADDRESS, 
                               CcpTicks->Timer.interrupt_source );
  }

  ////////////////////////////////////////////////////////
  void ccpWireBreaksIsr( void )
  {
    ccpWireIsBroken(   T0_A, !isCcpEdgeCaptured( T0_A ) );
    ccpWireIsBroken(   T0_B, !isCcpEdgeCaptured( T0_B ) );
    ccpWireIsBroken(   T1_A, !isCcpEdgeCaptured( T1_A ) );
    ccpWireIsBroken(   T1_B, !isCcpEdgeCaptured( T1_B ) );
    ccpWireIsBroken(   T2_A, !isCcpEdgeCaptured( T2_A ) );
    ccpWireIsBroken(   T2_B, !isCcpEdgeCaptured( T2_B ) );
    ccpWireIsBroken(   T3_A, !isCcpEdgeCaptured( T3_A ) );
    ccpWireIsBroken(   T3_B, !isCcpEdgeCaptured( T3_B ) );
    ccpWireIsBroken(   T4_A, !isCcpEdgeCaptured( T4_A ) );
    ccpWireIsBroken(   T4_B, !isCcpEdgeCaptured( T4_B ) );
    ccpWireIsBroken(   T5_A, !isCcpEdgeCaptured( T5_A ) );
    ccpWireIsBroken(   T5_B, !isCcpEdgeCaptured( T5_B ) );
  
    ccpEdgeIsCaptured( T0_A,  false );
    ccpEdgeIsCaptured( T0_B,  false );
    ccpEdgeIsCaptured( T1_A,  false );
    ccpEdgeIsCaptured( T1_B,  false );
    ccpEdgeIsCaptured( T2_A,  false );
    ccpEdgeIsCaptured( T2_B,  false );
    ccpEdgeIsCaptured( T3_A,  false );
    ccpEdgeIsCaptured( T3_B,  false );
    ccpEdgeIsCaptured( T4_A,  false );
    ccpEdgeIsCaptured( T4_B,  false );
    ccpEdgeIsCaptured( T5_A,  false );
    ccpEdgeIsCaptured( T5_B,  false );
  }

  ///////////////////////////////////////////////////////////////////////////////
  void ccpEdgeIsCaptured( uint8_t timer, bool edge_captured )
  {      if ( timer == T0_A ){ CcpWireBreak.edge_captured_T0_A = edge_captured; }
    else if ( timer == T0_B ){ CcpWireBreak.edge_captured_T0_B = edge_captured; }
    else if ( timer == T1_A ){ CcpWireBreak.edge_captured_T1_A = edge_captured; }
    else if ( timer == T1_B ){ CcpWireBreak.edge_captured_T1_B = edge_captured; }
    else if ( timer == T2_A ){ CcpWireBreak.edge_captured_T2_A = edge_captured; }
    else if ( timer == T2_B ){ CcpWireBreak.edge_captured_T2_B = edge_captured; }
    else if ( timer == T3_A ){ CcpWireBreak.edge_captured_T3_A = edge_captured; }
    else if ( timer == T3_B ){ CcpWireBreak.edge_captured_T3_B = edge_captured; }
    else if ( timer == T4_A ){ CcpWireBreak.edge_captured_T4_A = edge_captured; }
    else if ( timer == T4_B ){ CcpWireBreak.edge_captured_T4_B = edge_captured; }
    else if ( timer == T5_A ){ CcpWireBreak.edge_captured_T5_A = edge_captured; }
    else if ( timer == T5_B ){ CcpWireBreak.edge_captured_T5_B = edge_captured; }
  }

  ///////////////////////////////////////////////////////////////////////
  bool isCcpEdgeCaptured( uint8_t timer )
  {      if ( timer == T0_A ) { return CcpWireBreak.edge_captured_T0_A; }
    else if ( timer == T0_B ) { return CcpWireBreak.edge_captured_T0_B; }
    else if ( timer == T1_A ) { return CcpWireBreak.edge_captured_T1_A; }
    else if ( timer == T1_B ) { return CcpWireBreak.edge_captured_T1_B; }
    else if ( timer == T2_A ) { return CcpWireBreak.edge_captured_T2_A; }
    else if ( timer == T2_B ) { return CcpWireBreak.edge_captured_T2_B; }
    else if ( timer == T3_A ) { return CcpWireBreak.edge_captured_T3_A; }
    else if ( timer == T3_B ) { return CcpWireBreak.edge_captured_T3_B; }
    else if ( timer == T4_A ) { return CcpWireBreak.edge_captured_T4_A; }
    else if ( timer == T4_B ) { return CcpWireBreak.edge_captured_T4_B; }
    else if ( timer == T5_A ) { return CcpWireBreak.edge_captured_T5_A; }
    else if ( timer == T5_B ) { return CcpWireBreak.edge_captured_T5_B; }
    else                      { return                           false; }
  }

  /////////////////////////////////////////////////////////////////////////
  void ccpWireIsBroken( uint8_t timer, bool wirebreak )
  {      if ( timer == T0_A ){ CcpWireBreak.wire_broken_T0_A = wirebreak; }
    else if ( timer == T0_B ){ CcpWireBreak.wire_broken_T0_B = wirebreak; }
    else if ( timer == T1_A ){ CcpWireBreak.wire_broken_T1_A = wirebreak; }
    else if ( timer == T1_B ){ CcpWireBreak.wire_broken_T1_B = wirebreak; }
    else if ( timer == T2_A ){ CcpWireBreak.wire_broken_T2_A = wirebreak; }
    else if ( timer == T2_B ){ CcpWireBreak.wire_broken_T2_B = wirebreak; }
    else if ( timer == T3_A ){ CcpWireBreak.wire_broken_T3_A = wirebreak; }
    else if ( timer == T3_B ){ CcpWireBreak.wire_broken_T3_B = wirebreak; }
    else if ( timer == T4_A ){ CcpWireBreak.wire_broken_T4_A = wirebreak; }
    else if ( timer == T4_B ){ CcpWireBreak.wire_broken_T4_B = wirebreak; }
    else if ( timer == T5_A ){ CcpWireBreak.wire_broken_T5_A = wirebreak; }
    else if ( timer == T5_B ){ CcpWireBreak.wire_broken_T5_B = wirebreak; }
  }

  /////////////////////////////////////////////////////////////////////
  bool isCcpWireBroken( uint8_t timer )
  {      if ( timer == T0_A ) { return CcpWireBreak.wire_broken_T0_A; }
    else if ( timer == T0_B ) { return CcpWireBreak.wire_broken_T0_B; }
    else if ( timer == T1_A ) { return CcpWireBreak.wire_broken_T1_A; }
    else if ( timer == T1_B ) { return CcpWireBreak.wire_broken_T1_B; }
    else if ( timer == T2_A ) { return CcpWireBreak.wire_broken_T2_A; }
    else if ( timer == T2_B ) { return CcpWireBreak.wire_broken_T2_B; }
    else if ( timer == T3_A ) { return CcpWireBreak.wire_broken_T3_A; }
    else if ( timer == T3_B ) { return CcpWireBreak.wire_broken_T3_B; }
    else if ( timer == T4_A ) { return CcpWireBreak.wire_broken_T4_A; }
    else if ( timer == T4_B ) { return CcpWireBreak.wire_broken_T4_B; }
    else if ( timer == T5_A ) { return CcpWireBreak.wire_broken_T5_A; }
    else if ( timer == T5_B ) { return CcpWireBreak.wire_broken_T5_B; }
    else                      { return                          true; }
  }

  ////////////////////////////////////////////////////
  uint32_t ccpPinMux( uint8_t pin )
  {
         if( pin == PD_3 ) { return GPIO_PD3_T1CCP1; }
    else if( pin == PB_2 ) { return GPIO_PB2_T5CCP0; }
    else if( pin == PB_3 ) { return GPIO_PB3_T5CCP1; }
    else if( pin == PD_0 ) { return GPIO_PD0_T0CCP0; }
    else if( pin == PD_1 ) { return GPIO_PD1_T0CCP1; }
    else if( pin == PM_3 ) { return GPIO_PM3_T3CCP1; }
    else if( pin == PD_7 ) { return GPIO_PD7_T4CCP1; }
    else if( pin == PA_6 ) { return GPIO_PA6_T3CCP0; }
    else if( pin == PM_4 ) { return GPIO_PM4_T4CCP0; }
    else if( pin == PM_5 ) { return GPIO_PM5_T4CCP1; }
    else if( pin == PL_5 ) { return GPIO_PL5_T0CCP1; }
    else if( pin == PL_4 ) { return GPIO_PL4_T0CCP0; }
    else if( pin == PD_2 ) { return GPIO_PD2_T1CCP0; }
    else if( pin == PD_4 ) { return GPIO_PD4_T3CCP0; }
    else if( pin == PD_5 ) { return GPIO_PD5_T3CCP1; }
    else if( pin == PM_6 ) { return GPIO_PM6_T5CCP0; }
    else if( pin == PA_7 ) { return GPIO_PA7_T3CCP1; }
    else if( pin == PM_7 ) { return GPIO_PM7_T5CCP1; }
    else if( pin == PA_4 ) { return GPIO_PA4_T2CCP0; }
    else if( pin == PA_5 ) { return GPIO_PA5_T2CCP1; }
    else if( pin == PM_2 ) { return GPIO_PA5_T2CCP1; }
    else if( pin == PM_1 ) { return GPIO_PM1_T2CCP1; }
    else if( pin == PM_0 ) { return GPIO_PM0_T2CCP0; }
    else if( pin == PD_6 ) { return GPIO_PD6_T4CCP0; }
    else if( pin == PA_0 ) { return GPIO_PA0_T0CCP0; }
    else if( pin == PA_1 ) { return GPIO_PA1_T0CCP1; }
    else if( pin == PA_2 ) { return GPIO_PA2_T1CCP0; }
    else if( pin == PA_3 ) { return GPIO_PA3_T1CCP1; }
    else if( pin == PL_6 ) { return GPIO_PL6_T1CCP0; }
    else if( pin == PL_7 ) { return GPIO_PL7_T1CCP1; }
    else if( pin == PB_0 ) { return GPIO_PB0_T4CCP0; }
    else if( pin == PB_1 ) { return GPIO_PB1_T4CCP1; }
    else                   { return               0; }
  }

  //////////////////////////////////////////////////////////////
  isrPtr lookupCcpTicksIsr( uint8_t timer )
  {
         if ( timer == T0_A ){ return dispatchCcpTicksIsr_0A; }
    else if ( timer == T0_B ){ return dispatchCcpTicksIsr_0B; }
    else if ( timer == T1_A ){ return dispatchCcpTicksIsr_1A; }
    else if ( timer == T1_B ){ return dispatchCcpTicksIsr_1B; }
    else if ( timer == T2_A ){ return dispatchCcpTicksIsr_2A; }
    else if ( timer == T2_B ){ return dispatchCcpTicksIsr_2B; }
    else if ( timer == T3_A ){ return dispatchCcpTicksIsr_3A; }
    else if ( timer == T3_B ){ return dispatchCcpTicksIsr_3B; }
    else if ( timer == T4_A ){ return dispatchCcpTicksIsr_4A; }
    else if ( timer == T4_B ){ return dispatchCcpTicksIsr_4B; }
    else if ( timer == T5_A ){ return dispatchCcpTicksIsr_5A; }
    else if ( timer == T5_B ){ return dispatchCcpTicksIsr_5B; }
    else                     { return                      0; }
  }

  ////////////////////////////////////////////////////////////////////////////
  void dispatchCcpTicksIsr_0A ( void ) { ccpTicksIsr( CcpTicks[ T0_A - 1] ); }
  void dispatchCcpTicksIsr_0B ( void ) { ccpTicksIsr( CcpTicks[ T0_B - 1] ); }
  void dispatchCcpTicksIsr_1A ( void ) { ccpTicksIsr( CcpTicks[ T1_A - 1] ); }
  void dispatchCcpTicksIsr_1B ( void ) { ccpTicksIsr( CcpTicks[ T1_B - 1] ); }
  void dispatchCcpTicksIsr_2A ( void ) { ccpTicksIsr( CcpTicks[ T2_A - 1] ); }
  void dispatchCcpTicksIsr_2B ( void ) { ccpTicksIsr( CcpTicks[ T2_B - 1] ); }
  void dispatchCcpTicksIsr_3A ( void ) { ccpTicksIsr( CcpTicks[ T3_A - 1] ); }
  void dispatchCcpTicksIsr_3B ( void ) { ccpTicksIsr( CcpTicks[ T3_B - 1] ); }
  void dispatchCcpTicksIsr_4A ( void ) { ccpTicksIsr( CcpTicks[ T4_A - 1] ); }
  void dispatchCcpTicksIsr_4B ( void ) { ccpTicksIsr( CcpTicks[ T4_B - 1] ); }
  void dispatchCcpTicksIsr_5A ( void ) { ccpTicksIsr( CcpTicks[ T5_A - 1] ); }
  void dispatchCcpTicksIsr_5B ( void ) { ccpTicksIsr( CcpTicks[ T5_B - 1] ); }

}// end namespace roveware ///////////////////////////////////////////////////////////