#include "ControlFramework.h";

//deletes pointers used in framework to 'prevent memory leaks'. Most likely not neccessary but good practice.
ControlFrameworkInterface::~ControlFrameworkInterface()
{
  delete controller;
  delete manip;
  delete feedback;
}

//moves the device based on what was recieved from base station
//may return something later but now just assume it does its thing
void ControlFrameworkInterface::runOutputControl(const int movement)
{
  //var used as interrum value since algorithm can change the value
  int mov;
  //calls algorithm
  mov = manip->runAlgorithm(movement);
  //moves device
  controller->move(mov);
  return;
}

//Creates the device. Assigns the pins correctly.
//Make sure to put pins in correctly
DirectDiscreteHBridge::DirectDiscreteHBridge(const int FPIN, const int RPIN) : OutputDevice()
{
  FPWM_PIN = FPIN;
  RPWM_PIN = RPIN;
  outType = spd;
}

//moves based on what is passed from algorithm
void DirectDiscreteHBridge::move(const int movement)
{
  int mov = movement;
  int pwm = 0;

  //if supposed to move backwards
  if(movement < 0)
  {
    
    mov = abs(movement);
    pwm = map(mov, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);
    //stop the other motor.
    analogWrite(FPWM_PIN, 0);    
    analogWrite(RPWM_PIN, pwm);
  }

  //if forwards
  else if(movement > 0)
  {
    pwm = mov;
    pwm = map(mov, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);
    //stop the other motor.
    analogWrite(RPWM_PIN, 0);    
    analogWrite(FPWM_PIN, pwm);
  }

  //stop
  else if(movement == 0)
  {
    analogWrite(RPWM_PIN, 0);
    analogWrite(FPWM_PIN, 0);
  }
  
  return;
}


//speed to speed algorithm with no feedback just returns the input
int SpdToSpdNoFeedAlgorithm::runAlgorithm(const int input)
{
  return input;
}

