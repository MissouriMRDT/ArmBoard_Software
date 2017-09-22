#ifndef ROVEI2CTYPENAMES_H_
#define ROVEI2CTYPENAMES_H_

#include <stdint.h>

typedef struct RoveI2C_Handle
{
  uint8_t index;
} RoveI2C_Handle;

typedef enum RoveI2C_Speed
{
  //standard i2c speed, max of 100 kbit/s
  I2CSPEED_STANDARD,

  //full i2c speed, max of 400 kbit/s
  I2CSPEED_FULL,

  //fast-mode i2c speed, max of 1000 kbit/s
  I2CSPEED_FAST,

  //high speed-mode i2c speed, max of 3 Mbits/s
  I2CSPEED_HIGH

} RoveI2C_Speed;

typedef enum RoveI2C_Error
{
  //no errors encountered in exchange
  I2CERROR_NONE,

  //an acknowledgement wasn't received from the other device
  I2CERROR_ACK,

  //the other device held the clock line low past timeout
  I2CERROR_TIMEOUT,

  //the line was busy; in master mode, this means another master was talking
  I2CERROR_BUSY,

  //some other kind of error occurred
  I2CERROR_OTHER

} RoveI2C_Error;

#endif
