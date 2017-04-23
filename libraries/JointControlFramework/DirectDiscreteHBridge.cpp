#include "DirectDiscreteHBridge.h"
#include <pwmWriter.h>
#include "Energia.h"

DirectDiscreteHBridge::DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown) : OutputDevice(),
  FPWM_PIN(FPIN),
  RPWM_PIN(RPIN),
  inType(spd),
  invert(upsideDown)
{ }

void DirectDiscreteHBridge::move(const long movement)
{
  if(!enabled) return;

  int mov = invert ? -movement : movement;

  //if supposed to move backwards
  if(mov < 0)
  {
    int pwm = map(-mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

    //stop the transistor for the other direction -- if both were on, the h bridge would short out
    PwmWrite(FPWM_PIN, 0);
    PwmWrite(RPWM_PIN, pwm);
  }

  //if forwards
  else if(mov > 0)
  {
    int pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

    //stop the transistor for the other direction -- if both were on, the h bridge would short out
    PwmWrite(FPWM_PIN, pwm);
    PwmWrite(RPWM_PIN, 0);
  }

  //stop
  else if(mov == 0)
  {
    PwmWrite(RPWM_PIN, 0);
    PwmWrite(FPWM_PIN, 0);
  }
}

void DirectDiscreteHBridge::setPower(bool powerOn)
{
  enabled = powerOn;
  
  if(!enabled)
    move(0);
}