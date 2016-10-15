#include <PwmReader.h>

void setup()
{
  initPwmRead('A', 2, 100, 1);
}

void loop()
{
  getDuty(1);
}

