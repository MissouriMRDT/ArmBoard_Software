#ifndef TIMERINTERFACE_H_
#define TIMERINTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

//interrupt id's
#define TimerPeriodicInterrupt 0

//sets up the specified timer to generate the specified interrupt at a specified rate
//Input: Timer Id (hardware dependent), the interruptId based on above constants, and how frequently
//the timer runs the interrupt in microseconds (min/max are hardware specific)
extern void setupTimer(uint32_t timerId, uint32_t interruptId, uint32_t timerTimeout_us);

//begins timer operation
//inputs: timer Id
extern void startTimer(uint32_t timerId);

//stops timer operation
//inputs: timer Id
extern void stopTimer(uint32_t timerId);

//attaches a functions for the timer to run everytime it interrupts
//inputs: timer Id, an interrupt function to run
extern void attachTimerInterrupt(uint32_t timerId, void (*interruptFunc)(void) );

#endif
