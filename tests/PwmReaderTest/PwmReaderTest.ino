#include <PwmReader.h>

void setup()
{
  initPwmRead('A', 2, 100, 1); //pin A2 -- associated with timer 1 -- shall read a pwm signal with an expected frequency of 100 hz
}

void loop()
{
  getDuty(1);
}

