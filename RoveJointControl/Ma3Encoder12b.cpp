#include "Ma3Encoder12b.h"
#include "RoveJointUtilities.h"

static const int PWM_READ_MAX = 4095;
static const int PWM_READ_MIN = 1;

long Ma3Encoder12b::getFeedback()
{
  int32_t readOnPeriod = getOnPeriod(PwmHandle, PWM_MICRO); //signed for calculation below

  if(abs(lastReading - readOnPeriod) < deadband || readOnPeriod == 0 || readOnPeriod > pwmMax)
  {
    readOnPeriod = lastReading;
  }
  else
  {
    uint16_t approx45Deg = pwmMax / 8;
    if(lastReading > (pwmMax - approx45Deg) && readOnPeriod < approx45Deg) //likely just rolled over in these boundaries
    {
      readOnPeriod = (uint32_t)(filterConstant * (float)lastReading + (1.0 - filterConstant) * ((float)readOnPeriod + pwmMax)) % pwmMax;
    }
    else if(readOnPeriod > (pwmMax - approx45Deg) && lastReading < approx45Deg)
    {
      readOnPeriod = (uint32_t)(filterConstant * ((float)lastReading + pwmMax) + (1.0 - filterConstant) * (float)readOnPeriod) % pwmMax;
    }
    else
    {
      readOnPeriod = filterConstant * (float)lastReading + (1.0 - filterConstant) * (float)readOnPeriod;
    }

    lastReading = readOnPeriod;
  }
  
  if(reversed)
  {
    readOnPeriod = pwmMax - readOnPeriod + PWM_READ_MIN;
  }

  //scale the values from the pwm values to the common position values, IE 1-4095 to POS_MIN-POS_MAX, giving the absolute read angle
  long absoluteAngle = map(readOnPeriod, PWM_READ_MIN, pwmMax, POS_MIN, POS_MAX);
  
  //set the relative angle, and account for any overflow due to the calculations
  long relativeAngle = absoluteAngle+offsetAngle;
  if(relativeAngle > (long)POS_MAX) //cast to long as math screws up if it doesn't
  {
    relativeAngle -= (POS_MAX-POS_MIN); 
  }
  else if(relativeAngle < (long)POS_MIN)
  {
    relativeAngle += (POS_MAX-POS_MIN);
  }
  
  return(relativeAngle);
}

Ma3Encoder12b::Ma3Encoder12b(uint16_t pwmReadModule, uint16_t mappedPinNumber)
  : FeedbackDevice(InputPosition), offsetAngle(0), PwmHandle(initPwmRead(pwmReadModule, mappedPinNumber)), deadband(5), lastReading(PWM_READ_MIN),
    pwmMax(PWM_READ_MAX), reversed(false), filterConstant(0)
{}

float Ma3Encoder12b::getFeedbackDegrees()
{
  float position = getFeedback();
  return(position * 360.0 / ((float)(POS_MAX-POS_MIN)));
}

void Ma3Encoder12b::setOffsetAngle(float offset)
{
  int sign = sign(offset);
  offset = constrain(offset, -360.0, 360.0); //constrain offset to 0 - 360 degrees
  offsetAngle = sign*((abs(offset) * ((float)(POS_MAX - POS_MIN)) / 360.0) + POS_MIN); //offset in 0-360, so convert to framework positional values
}

void Ma3Encoder12b::setDeadband(uint16_t deadband_us)
{
  deadband = deadband_us;
}

void Ma3Encoder12b::setMaxPwm(uint32_t maxPwm_us)
{
  pwmMax = maxPwm_us;
}

void Ma3Encoder12b::reverseDirection(bool reverse)
{
  reversed = reverse;
}

void Ma3Encoder12b::setFilterConstant(float filter)
{
  filterConstant = filter;
}
