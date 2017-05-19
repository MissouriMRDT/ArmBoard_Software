#include "GenPwmPhaseHBridge.h"
#include "Energia.h"
#include "PwmWriter.h"

GenPwmPhaseHBridge::GenPwmPhaseHBridge(const int PwmPin, const int PhPin, bool upsideDown) : OutputDevice()
{
  PWM_PIN = PwmPin;
  PHASE_PIN = PhPin;
  invert = upsideDown;
  inType = spd;
  pinMode(PHASE_PIN, OUTPUT);
}

GenPwmPhaseHBridge::GenPwmPhaseHBridge(const int PwmPin, const int PhPin, const int EnPin, bool enLogicHigh, bool upsideDown) : OutputDevice()
{
  PWM_PIN = PwmPin;
  PHASE_PIN = PhPin;
  ENABLE_PIN = EnPin;
  enableLogicHigh = enLogicHigh;
  inType = spd;
  invert = upsideDown;
  pinMode(PHASE_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  setPower(false); //set power to off for initial setup
}


void GenPwmPhaseHBridge::move(const long movement)
{
  if(!enabled) return;

  int mov = invert ? -movement : movement;
  
  if(rampUsed)
  {
    mov = scaleRamp(mov);
  }
  
  currentSpeed = mov;
  
  //if supposed to move backwards
  if(mov < 0)
  {
    int pwm = map(-mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

    //set phase to 1 for "reverse" rotation
    digitalWrite(PHASE_PIN, HIGH);
    
    //pulsate enable pin to control motor
    PwmWrite(PWM_PIN, pwm);
  }
  
  //if forwards
  else if(mov > 0)
  {
    int pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);
      
    //set phase to 0 for "forward" rotation
    digitalWrite(PHASE_PIN, LOW);
    
    //pulsate enable pin to control motor
    PwmWrite(PWM_PIN, pwm);
  }
  
  //stop
  else if(mov == 0)
  {
    PwmWrite(PWM_PIN, 0);//set enable to 0 to brake motor
    //phase don't matter
  }
}

void GenPwmPhaseHBridge::setPower(bool powerOn)
{
  //when enable pin does not have its default value, assume device was constructed with an enable pin
  if(ENABLE_PIN != -1)
  {
    int pinNewState = ((powerOn && enableLogicHigh) || (!powerOn && !enableLogicHigh)) ? HIGH : LOW;
    
    digitalWrite(ENABLE_PIN, pinNewState);
  }
  
  if(!powerOn)
  {
    PwmWrite(PWM_PIN, 0);//set enable to 0 to brake motor
    currentSpeed = 0;
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

int GenPwmPhaseHBridge::scaleRamp(int desiredSpeed)
{
  bool accelerate;
  int magnitudeChangeLimit;
  
  //scale the output so that it's not allowed to change once per call more than the amount specified by magChangeLimUp or Down, for accel and decel
  if(sign(desiredSpeed) == sign(currentSpeed) || currentSpeed == 0) 
  {
    if(abs(desiredSpeed) - abs(currentSpeed ) >= 0)
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
  if(abs(desiredSpeed - currentSpeed) > magnitudeChangeLimit)
  {
    desiredSpeed = currentSpeed + sign(desiredSpeed - currentSpeed) * magnitudeChangeLimit;
    if(desiredSpeed > SPEED_MAX)
    {
      desiredSpeed = SPEED_MAX;
    }
    else if(desiredSpeed < SPEED_MIN)
    {
      desiredSpeed = SPEED_MIN;
    }
  }
    
  return(desiredSpeed);
}

long GenPwmPhaseHBridge::getCurrentMove()
{
  if(invert) //if we're inverted, then we technically move negatively even if we're moving in the 'positive' direction. The direction is the important part
  {
    return(currentSpeed * -1); 
  }
  else
  {
    return(currentSpeed);
  }
}