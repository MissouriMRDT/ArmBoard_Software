#include "GenPwmPhaseHBridge.h"
#include "Energia.h"
#include "PwmWriter.h"

GenPwmPhaseHBridge::GenPwmPhaseHBridge(const int PwmPin, const int PhPin, bool upsideDown) : OutputDevice()
{
  PWM_PIN = PwmPin;
  PHASE_PIN = PhPin;
  inType = spd;
  invert = upsideDown;

  pinMode(PHASE_PIN, OUTPUT);
}

GenPwmPhaseHBridge::GenPwmPhaseHBridge(const int PwmPin, const int PhPin, const int EnPin, bool enLogicHigh, bool upsideDown) : OutputDevice()
{
  PWM_PIN = PwmPin;
  PHASE_PIN = PhPin;
  ENABLE_PIN = EnPin;
  inType = spd;
  invert = upsideDown;
  enableLogicHigh = enLogicHigh;

  pinMode(PHASE_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  togglePower(false); //set power to off for initial setup
}


//move function which passes in speed ( which is converted to phase and PWM) to move device
void GenPwmPhaseHBridge::move(const long movement)
{
  int mov = movement;
  int pwm = 0;
  
  if(enabled) //only perform output if the device has been enabled
  {
    
    //if mounted upside down then invert the signal passed to it and move accordingly
    if (invert)
    {
      mov = -mov;
    }
    
    //if supposed to move backwards
    if(mov < 0)
    {
      mov = abs(mov);
      pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

      //set phase to 1 for "reverse" rotation
      digitalWrite(PHASE_PIN, HIGH);
      
      //pulsate enable pin to control motor
      PwmWrite(ENABLE_PIN, pwm);
    }
    
    //if forwards
    else if(mov > 0)
    {
      pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);
        
      //set phase to 0 for "forward" rotation
      digitalWrite(PHASE_PIN, LOW);
      
      //pulsate enable pin to control motor
      PwmWrite(ENABLE_PIN, pwm);
    }
    
    //stop
    else if(mov == 0)
    {
      PwmWrite(ENABLE_PIN, 0);//set enable to 0 to brake motor
      //phase don't matter
    }
  }
  
  return;
}

//tells the device to power on or off. Note that on setup, device assumes power is off
//If the device class was constructed with an enable pin, the function will physically turn the device on or off
//If it wasn't, it will virtually turn the device on or off, IE if it's off it will refuse to send an output
void GenPwmPhaseHBridge::togglePower(bool powerOn)
{
  //when enable pin does not have its default value, assume device was constructed with an enable pin
  if(ENABLE_PIN != -1)
  {
    int pinNewState;
    
    if((powerOn && enableLogicHigh) || (!powerOn && !enableLogicHigh))
    {
      pinNewState = HIGH;
    }
    else
    {
      pinNewState = LOW;
    }
    
    digitalWrite(ENABLE_PIN, pinNewState);
  }
  
  if(powerOn == false)
  {
    move(0);
  }
  
  enabled = powerOn;
}