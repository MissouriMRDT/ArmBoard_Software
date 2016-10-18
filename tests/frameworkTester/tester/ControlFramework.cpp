#include "ControlFramework.h";

interface::~interface()
{
  delete controller;
  delete manip;
}

void interface::mov(const int movement)
{
  int mov;
  mov = manip->modify(movement);
  controller->moveMotor(mov);
  return;
}

directDiscreteHBridge::directDiscreteHBridge(const int fwd, const int rev) : controllerType()
{
  FPWM_PIN = fwd;
  RPWM_PIN = rev;
}

void directDiscreteHBridge::moveMotor(const int movement)
{
  int mov = movement;
  int pwm = 0;

  Serial.print("Moving at movement: ");
  Serial.println(mov);
  
  /*
  if(movement < 0)
  {
    
    mov = abs(movement);
    pwm = map(mov, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);
    analogWrite(RPWM_PIN, pwm);
  }

  else if(movement > 0)
  {
    pwm = mov;
    analogWrite(FPWM_PIN, pwm);
  }
  
  else if(movement == 0)
  {
    analogWrite(RPWM_PIN, pwm);
    analogWrite(FPWM_PIN, pwm);
  }*/
  
  return;
}



int spdToSpdNoFeedAlgorithm::modify(const int movement)
{
  return movement;
}

