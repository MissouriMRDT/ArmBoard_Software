#ifndef TIMERINTERFACE_H_
#define TIMERINTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include "RoveTimerStructures.h"

//interrupt id's
#define TimerPeriodicInterrupt 0

//sets up the specified timer to generate the specified interrupt at a specified rate
//Input: Timer Id (hardware dependant), the interruptId based on above constants, and how frequently
//the timer runs the interrupt in microseconds.
roveTimer_Handle setupTimer(uint32_t timerId, uint32_t interruptId, uint32_t timerTimeout_us);

//begins timer operation
//inputs: handle of the timer to start
void startTimer(roveTimer_Handle handle);

//stops timer operation
//inputs: handle of the timer to stop
void stopTimer(roveTimer_Handle handle);

//attaches a functions for the timer to run everytime it interrupts
//inputs: handle of the timer to modify, an interrupt function to run
void attachTimerInterrupt(roveTimer_Handle handle, void (*interruptFunc)(void) );

#endif
