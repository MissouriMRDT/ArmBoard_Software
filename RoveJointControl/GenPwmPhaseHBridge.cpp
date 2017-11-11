#include "GenPwmPhaseHBridge.h"
#include "RoveJointUtilities.h"

static const int PWM_MIN = 0, PWM_MAX = 255;

GenPwmPhaseHBridge::GenPwmPhaseHBridge(const int PwmGen, const int PwmPin, const int PhPin, bool upsideDown)
  : OutputDevice(InputPowerPercent, upsideDown), PHASE_PIN(PhPin), PwmHandle(setupPwmWrite(PwmGen, PwmPin)),
    currentPower(0), rampUsed(false), ENABLE_PIN(-1)
{
  magChangeLimUp = (POWERPERCENT_MAX - POWERPERCENT_MIN);
  magChangeLimDown = (POWERPERCENT_MAX - POWERPERCENT_MIN);
}

GenPwmPhaseHBridge::GenPwmPhaseHBridge(const int PwmGen, const int PwmPin, const int PhPin, const int EnPin, bool enLogicHigh, bool upsideDown)
  : OutputDevice(InputPowerPercent, upsideDown), PHASE_PIN(PhPin), PwmHandle(setupPwmWrite(PwmGen, PwmPin)),
    currentPower(0), rampUsed(false), ENABLE_PIN(EnPin), enableLogicHigh(enLogicHigh)
{
  magChangeLimUp = (POWERPERCENT_MAX - POWERPERCENT_MIN);
  magChangeLimDown = (POWERPERCENT_MAX - POWERPERCENT_MIN);

  //set power will actually toggle turning on or off the device with the enable pin, so we want to make sure it's physically turned on
  //at the start instead of just simply setting the enabled flag
  setPower(true);
}


void GenPwmPhaseHBridge::move(const long movement)
{
  if(!enabled) return;

  int mov = invert ? -movement : movement;
  int pwm; 
  
  if(rampUsed)
  {
    mov = scaleRamp(mov);
  }
  
  //if supposed to move backwards
  if(mov < 0)
  {
    pwm = map(-mov, 0, POWERPERCENT_MAX, PWM_MIN, PWM_MAX);

    //set phase to 1 for "reverse" rotation
    digitalPinWrite(PHASE_PIN, HIGH);
    
    //pulsate enable pin to control motor
    pwmWriteDuty(PwmHandle, pwm);
  }
  
  //if forwards
  else if(mov > 0)
  {
    pwm = map(mov, 0, POWERPERCENT_MAX, PWM_MIN, PWM_MAX);
      
    //set phase to 0 for "forward" rotation
    digitalPinWrite(PHASE_PIN, LOW);
    
    //pulsate enable pin to control motor
    pwmWriteDuty(PwmHandle, pwm);
  }
  
  //stop
  else if(mov == 0)
  {   
    stop();
    //phase don't matter
  }
  
  currentPower = mov;
}

void GenPwmPhaseHBridge::setPower(bool powerOn)
{

  if(ENABLE_PIN != -1)
  {
    int pinNewState = ((powerOn && enableLogicHigh) || (!powerOn && !enableLogicHigh)) ? HIGH : LOW;
    
    digitalPinWrite(ENABLE_PIN, pinNewState);
  }
  
  if(!powerOn)
  {
    stop();
    currentPower = 0;
  }
  
  enabled = powerOn;
}

void GenPwmPhaseHBridge::setRampUp(unsigned int magnitudeChangeLimit)
{
  magChangeLimUp = magnitudeChangeLimit;
  rampUsed = true;
}

void GenPwmPhaseHBridge::setRampDown(unsigned int magnitudeChangeLimit)
{
  magChangeLimDown = magnitudeChangeLimit;
  rampUsed = true;
}

int GenPwmPhaseHBridge::scaleRamp(int desiredPower)
{
  bool accelerate;
  int magnitudeChangeLimit;
  
  //scale the output so that it's not allowed to change once per call more than the amount specified by magChangeLimUp or Down, for accel and decel
  if(sign(desiredPower) == sign(currentPower) || currentPower == 0) 
  {
    if(abs(desiredPower) - abs(currentPower ) >= 0)
    {
      accelerate = true;
    }
    else
    {
      accelerate = false;
    }
  }
  
  //if the destination speed and current speed are in different directions, we're decelerating until we get back to 0 speed
  else
  {
    accelerate = false;
  }
  
  if(accelerate)
  {
    magnitudeChangeLimit = magChangeLimUp;
  }
  else
  {
    magnitudeChangeLimit = magChangeLimDown;
  }
  
  //if the desired change is greater than the maximum allowed step, scale it down
  if(abs(desiredPower - currentPower) > magnitudeChangeLimit)
  {
    desiredPower = currentPower + sign(desiredPower - currentPower) * magnitudeChangeLimit;
    if(desiredPower > POWERPERCENT_MAX)
    {
      desiredPower = POWERPERCENT_MAX;
    }
    else if(desiredPower < POWERPERCENT_MIN)
    {
      desiredPower = POWERPERCENT_MIN;
    }
  }
    
  return(desiredPower);
}

long GenPwmPhaseHBridge::getCurrentMove()
{
  if(invert)
  {
    return(currentPower * -1); 
  }
  else
  {
    return(currentPower);
  }
}

void GenPwmPhaseHBridge::stop()
{
  pwmWriteDuty(PwmHandle, 0);
  currentPower = 0;
}
