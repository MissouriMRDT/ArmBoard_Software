/////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Todo

#ifndef ROVE_WATCHDOG_H
#define ROVE_WATCHDOG_H

#include <stdint.h>

enum ResetConfig
{
  ENABLE_BOARD_RESET,
  DISABLE_BOARD_RESET
};

class RoveWatchdog
{
public:
  void attach( void(*userFunction)(void) ); 
  void start(  int   timeout_millis, ResetConfig reset=DISABLE_BOARD_RESET, int estops_before_board_reset=5 );
  void stop();
  void clear();
};

#endif // ROVE_WATCHDOG_H