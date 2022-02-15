/////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RoveWatchdog.h"
#include "RoveTimer.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"
#include "driverlib/rom_map.h"

static roveware::isrPtr userIsr;
static void watchdogIsr();

static int  isr_without_user_clear_count  = 0;
static int  isr_before_board_reset_count = 0;
static ResetConfig board_reset_config;

//////////////////////////////////////////////////////////
void RoveWatchdog::attach( void(*userFunction)(void) )
{         SysCtlPeripheralEnable( SYSCTL_PERIPH_WDOG0 );
  while( !SysCtlPeripheralReady(  SYSCTL_PERIPH_WDOG0 ) )
  {   }; 
  userIsr = userFunction;
  WatchdogUnlock( WATCHDOG0_BASE );
}

void RoveWatchdog::start( int timeout_millis, ResetConfig reset, int estops_before_board_reset ) /////////////////////
{ isr_before_board_reset_count = estops_before_board_reset;
  board_reset_config = reset;
  WatchdogIntRegister(  WATCHDOG0_BASE, watchdogIsr);
  WatchdogReloadSet(    WATCHDOG0_BASE, 1000 * roveware::SYSCLOCK_TICKS_PER_MICRO  * timeout_millis );
  WatchdogResetEnable(  WATCHDOG0_BASE );
  WatchdogIntEnable(    WATCHDOG0_BASE );
  WatchdogEnable(       WATCHDOG0_BASE );
  WatchdogIntClear(     WATCHDOG0_BASE ); 
}

void RoveWatchdog::stop() /////////////////
{ IntDisable( INT_WATCHDOG_TM4C129 );
  WatchdogResetDisable( WATCHDOG0_BASE ); }

void RoveWatchdog::clear() //////////
{ WatchdogIntClear( WATCHDOG0_BASE ); 
  isr_without_user_clear_count = 0; }

void watchdogIsr() ///////////////////////////////////////////////////
{ if( (board_reset_config == DISABLE_BOARD_RESET)  || isr_without_user_clear_count++ <= isr_before_board_reset_count )
  { WatchdogIntClear( WATCHDOG0_BASE ); 
    userIsr(); }
}
