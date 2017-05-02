#include "VNH5019.h"
#include "Energia.h"
#include "PwmWriter.h"

VNH5019::VNH5019(const int PwmPin, const int InaPin, const int InbPin, bool upsideDown) : OutputDevice()
{
  PWM_PIN = PwmPin;
  INA_PIN = InaPin;
  INB_PIN = InbPin;
  inType = spd;
  invert = upsideDown;
  pinMode(INA_PIN, OUTPUT);
  pinMode(INB_PIN, OUTPUT);
  
  //brake motor by default
  digitalWrite(INA_PIN, LOW);
  digitalWrite(INB_PIN, LOW);
}

void VNH5019::move(const long movement)
{
  if(!enabled) return;

  int mov = invert ? -movement : movement;
  
  //if supposed to move backwards
  if(mov < 0)
  {
    int pwm = map(-mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

    //set InB to 0 and InA to 1 for "reverse" rotation
    digitalWrite(INA_PIN, HIGH);
    digitalWrite(INB_PIN, LOW);
    
    //pulsate enable pin to control motor
    PwmWrite(PWM_PIN, pwm);
  }
  
  //if forwards
  else if(mov > 0)
  {
    int pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);
      
    //set InB to 1 and InA to 0 for forward rotation
    digitalWrite(INA_PIN, LOW);
    digitalWrite(INB_PIN, HIGH);
    
    //pulsate enable pin to control motor
    PwmWrite(PWM_PIN, pwm);
  }
  
  //stop
  else if(mov == 0)
  {
    PwmWrite(PWM_PIN, 0);//set all pins to 0 to brake motor
    digitalWrite(INA_PIN, LOW);
    digitalWrite(INB_PIN, LOW);
  }
}

void VNH5019::setPower(bool powerOn)
{
  if(!powerOn) //stop movement before storing into enabled variable, as motor will refuse to accept move commands after being disabled
  {
    move(0);
  }
  
  enabled = powerOn;
}