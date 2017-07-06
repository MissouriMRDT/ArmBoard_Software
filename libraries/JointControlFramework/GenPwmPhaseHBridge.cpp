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
  
  //scale the output so that it's not allowed to change once per call more than the amount specified by magChangeLimit
  //exception is if it's 0, since the joint should halt all movement whenever commanded
  if((abs(mov) > abs(currentSpeed) + magChangeLimit) && mov != 0)
  {
    mov = currentSpeed + sign(mov) * magChangeLimit;
  }
  
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
    move(0);
  }
  
  enabled = powerOn;
}

void GenPwmPhaseHBridge::setRamping(unsigned int magnitudeChangeLimit)
{
  magChangeLimit = magnitudeChangeLimit;
}
