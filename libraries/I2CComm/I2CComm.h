#define PART_TM4C1294NCPDT //the part to use, replace if you're using a different part

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/pin_map.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

class I2CComm
{
  private:
    uint32_t i2cBase;
    
  public:
    bool init(uint8_t PinSCL, uint8_t PinSDA);
    void send(uint8_t SlaveAddr, uint8_t msg);
    void send(uint8_t SlaveAddr, uint8_t msg, uint8_t reg);
    void send(uint8_t SlaveAddr, uint8_t msg[]);
    void send(uint8_t SlaveAddr, uint8_t msg[], uint8_t reg);
    uint32_t receive(uint8_t SlaveAddr);
    uint32_t receive(uint8_t SlaveAddr, uint8_t reg);
    uint32_t* receive(uint8_t SlaveAddr, uint32_t sizeOfReceive);
    uint32_t* receive(uint8_t SlaveAddr, uint32_t sizeOfReceive, uint8_t reg);
    I2CComm();
    ~I2CComm();
};












