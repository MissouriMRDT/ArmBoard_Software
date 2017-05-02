#include "Ma3Encoder12b.h"
#include <PwmReader.h>

long Ma3Encoder12b::getFeedback()
{
  uint32_t readOnPeriod = getOnPeriod_us(pwmMappedPin); 
  
  //values will be between PWM_READ_MIN and PWM_READ_MAX, that is 1 and 4097. 
  //Or at least they should be; if it's above there was slight comm error and it can be scaled down to the max val.
  if(readOnPeriod > PWM_READ_MAX) readOnPeriod = PWM_READ_MAX;

  //Alternatively, if the value read is 0 then it means the duty cycle is either at 0 or 100%, 
  //so if we get a 0 then we need to check the duty cycle to see which it is.
  //If it's 0%, then use min value. If it's 100%, use max value.
  if(readOnPeriod == 0)
  {
    if(getDuty(pwmMappedPin) == 0)
      readOnPeriod = PWM_READ_MIN;
    else
      readOnPeriod = PWM_READ_MAX;
  }
  //scale the values from the pwm values to the common position values, IE 1-4097 to POS_MIN-POS_MAX
  return map(readOnPeriod, PWM_READ_MIN, PWM_READ_MAX, POS_MIN, POS_MAX);
}


Ma3Encoder12b::Ma3Encoder12b(uint8_t mappedPinNumber): FeedbackDevice()
{
  pwmMappedPin = mappedPinNumber;
  fType = pos;
  
  initPwmRead(pwmMappedPin); 
}