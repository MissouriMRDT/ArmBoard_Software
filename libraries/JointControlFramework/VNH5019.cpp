#include "VNH5019.h"
#include "Energia.h"
#include "PwmWriter.h"

//constructor for VNH5019 motor driver. Inputs are pin asignments for hardware pins, also a bool to determine the orientation of da motor
VNH5019::VNH5019(const int PwmPin, const int InaPin, const int InbPin, bool upsideDown)
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

//move function which passes in speed (which is converted to phase and PWM) to move device
void VNH5019::move(const long movement)
{
  int mov = movement;
  int pwm = 0;
  
  if(enabled)//only move if device has been enabled by the user
  {
    //if mounted upside down then invert the signal passed to it and move accordingly
    if (invert)
    {
      //inverts the input easily
      mov = -mov;
    }
    
    currentSpeed = mov;
    
    //if supposed to move backwards
    if(mov < 0)
    {
      mov = abs(mov);
      pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

      //set InB to 0 and InA to 1 for "reverse" rotation
      digitalWrite(INA_PIN, HIGH);
      digitalWrite(INB_PIN, LOW);
      
      //pulsate enable pin to control motor
      PwmWrite(PWM_PIN, pwm);
    }
    
    //if forwards
    else if(mov > 0)
    {
      pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);
        
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
  
  return;
}

void VNH5019::setPower(bool powerOn)
{
  if(powerOn == false)
  {
    move(0);
  }
  
  enabled = powerOn;
}

long VNH5019::getCurrentMove()
{
  return(currentSpeed);
}