#include <Energia.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

const uint8_t PortARef = 0;
const uint8_t PortBRef = 0;
const uint8_t PortDRef = 0;
const uint8_t PortLRef = 0;
const uint8_t PortMRef = 0;
const uint32_t SysClockFreq = 120000000; //frequency of the system clock, which the timers use
const uint8_t MinInputFreq = 8; //max value the timer can store is 2^24, which works out to being able to pulse at max 1/8 times a second. So lowest input freq is 8


bool initPwmRead(char gpioPort, uint8_t pinNumber, uint32_t expectedFrequecy, uint8_t timerNumber);

void stopPwmRead(uint8_t timerNum);

uint8_t getDuty(uint8_t timerNum);

uint16_t getTotalPeriod_us(uint8_t timerNum);

uint16_t getOnPeriod_us(uint8_t timerNum);
