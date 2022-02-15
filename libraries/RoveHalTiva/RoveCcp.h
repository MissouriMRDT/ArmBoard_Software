//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Tiva C Pin Module Capture and Compare Pin Timer ( CCP )
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_CCP_H
#define ROVE_CCP_H

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TxCCPx                                => Tiva C Series TM4C1294NCPDT Microcontroller Data Sheet (Rev. B) defines the module top down register hierarchy
//                                       => ti.com/lit/ds/symlink/tm4c1294ncpdt.pdf

// GPIO_PORTx_BASE, GPIO_PIN_x, PX_x     => Energia provides Wiring api lookup of TivaWare gpio port values by Energia pin_number number
//                                       => github.com/energia/Energia/blob/master/hardware/lm4f/variants/launchpad_129/pins_energia.h

// SYSCTL_PERIPH_TIMERX,GPIO_Pxx_TxCCPx.. => TivaWare Peripheral Driver Library provides pin_number peripheral mux, timer base, capture periph, and interrupt masks
// TIMERX_BASE, INT_TIMERXA, INT_TIMERXB => ti.com/lit/ug/spmu298d/spmu298d.pdf   

// TivaC_1294 support => Wiring 'style' Api,  progmem lookup table, conforming to the Arduino / Energia pin_map pattern

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo => template<class BUFF_TYPE_T, size_t BUFF_SIZE_T>
// PIOSC                  => 13.3.2 Timer Clock Source
// CCP                    => 13.4 Initialization and Configuration
// Edge Time              => 13.4.4 Input Edge Time Mode
// SYSCLK_DIV             => 13-5. 16-Bit Timer With Prescaler Configurations
// Clock Divide by 1 ~ 64 on a 16b generator with 8bit prescale (16+8=24) at 120Mhz system clock => ? nS ~ ? mS period => 120Mhz ~ ?Hz frequency
//      => 1 micros < ?      < ? micros
//   && => 0 micros < ?      < ? micros
// else => we default to 
// defualts to TODO on rovePwmRead and rovePwmReadTicks24
// Todo wrap the struct in a template wrapper on BUFF_TYPE_T, BUFF_SIZE_T

//////////////////////////
#include "RoveRingBuff.h"
#include "RoveTimer.h"

#include <stdint.h>

namespace roveware /////////////////////////////////////////////////////////////////////////////////////
{
  typedef void (*userCcpTicksArgsIsrPtr)( void* CcpTicksPtr, bool digital_read, uint32_t capture_ticks );

  bool     isCcpEdgeCaptured(     uint8_t  timer ); //////
  void     ccpEdgeIsCaptured(     uint8_t  timer, 
                                  bool     edge_captured );

  bool     isCcpWireBroken(       uint8_t  timer ); ///
  void     ccpWireIsBroken(       uint8_t  timer,
                                  bool     wirebreak );

  bool     isCcpValid(            uint8_t  pin ); /////////////////
  uint32_t ccpPinMux(             uint8_t  pin );
  void     attachCcpTicks(        uint8_t  timer,
                                  struct   CcpTicks* Ccp );
  isrPtr   lookupCcpTicksIsr(     uint8_t  timer );
  void           ccpTicksIsr(     struct   CcpTicks* Ccp );
  void attachUserCcpTicksIsr(     struct   CcpTicks* CcpTick, userCcpTicksArgsIsrPtr userCcpTicksIsr );
  void noUserCcpTicksIsr(         void*    CcpTicksArg,       bool     digital_read_arg, 
                                                              uint32_t capture_ticks_arg );

  void dispatchCcpTicksIsr_0A ( void );
  void dispatchCcpTicksIsr_0B ( void );
  void dispatchCcpTicksIsr_1A ( void );
  void dispatchCcpTicksIsr_1B ( void );
  void dispatchCcpTicksIsr_2A ( void );
  void dispatchCcpTicksIsr_2B ( void );
  void dispatchCcpTicksIsr_3A ( void );
  void dispatchCcpTicksIsr_3B ( void );
  void dispatchCcpTicksIsr_4A ( void );
  void dispatchCcpTicksIsr_4B ( void );
  void dispatchCcpTicksIsr_5A ( void );
  void dispatchCcpTicksIsr_5B ( void );

  void ccpWireBreaksIsr(        void );

  struct CcpHardware ////////////////////
  {
    volatile uint32_t CCP_PIN_MUX;
    volatile uint32_t PORT_BASE_ADDRESS;
    volatile uint32_t PIN_BIT_MASK; };

  struct CcpTicks ///////////////////////////////////////////
  {  
    struct  CcpHardware Hw;
    struct  Timer       Timer;
    uint8_t             timer;

    volatile int        priority;

    RoveRingBuff < uint32_t, 64 > PulseWidthsHigh;
    RoveRingBuff < uint32_t, 64 > PulseWidthsLow; 
    
    userCcpTicksArgsIsrPtr userCcpTicksIsr = noUserCcpTicksIsr; };

  struct CcpWireBreak ///////////////////////////////////////
  {
    volatile bool wire_broken_T0_A:1,   wire_broken_T0_B:1,
                  wire_broken_T1_A:1,   wire_broken_T1_B:1,
                  wire_broken_T2_A:1,   wire_broken_T2_B:1,
                  wire_broken_T3_A:1,   wire_broken_T3_B:1,
                  wire_broken_T4_A:1,   wire_broken_T4_B:1,
                  wire_broken_T5_A:1,   wire_broken_T5_B:1,
                  
                  edge_captured_T0_A:1, edge_captured_T0_B:1,
                  edge_captured_T1_A:1, edge_captured_T1_B:1,
                  edge_captured_T2_A:1, edge_captured_T2_B:1,
                  edge_captured_T3_A:1, edge_captured_T3_B:1,
                  edge_captured_T4_A:1, edge_captured_T4_B:1,
                  edge_captured_T5_A:1, edge_captured_T5_B:1; };

}// end namespace roveware ///////////////////////////////

#endif // ROVE_CCP_H

