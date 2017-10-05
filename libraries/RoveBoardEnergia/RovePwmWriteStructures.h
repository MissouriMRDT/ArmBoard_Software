/*
 * RovePwmWriteStructures.h
 *
 *  Created on: Oct 4, 2017
 *      Author: drue
 */

#ifndef ROVEBOARD_STRUCTURES_ROVEPWMWRITESTRUCTURES_H_
#define ROVEBOARD_STRUCTURES_ROVEPWMWRITESTRUCTURES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct rovePwmWrite_Handle
{
  bool initialized;
  uint16_t index;
  uint16_t pin;

  rovePwmWrite_Handle()
  {
    initialized = false;
  }
}rovePwmWrite_Handle;


#endif /* ROVEBOARD_STRUCTURES_ROVEPWMWRITESTRUCTURES_H_ */
