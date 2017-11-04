/*
 * RoveTimerStructures.h
 *
 *  Created on: Oct 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_STRUCTURES_ROVETIMERSTRUCTURES_H_
#define ROVEBOARD_STRUCTURES_ROVETIMERSTRUCTURES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct roveTimer_Handle
{
    bool initialized;
    uint16_t index;

    roveTimer_Handle()
    {
      initialized = false;
    }
} roveTimer_Handle;



#endif /* ROVEBOARD_STRUCTURES_ROVETIMERSTRUCTURES_H_ */
