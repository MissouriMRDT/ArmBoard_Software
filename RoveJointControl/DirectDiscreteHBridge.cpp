#include "DirectDiscreteHBridge.h"
#include "RoveJointUtilities.h"

//Value ranges for conversting input to expected output when sending
static const int PWM_MIN = 0, PWM_MAX = 255;

DirectDiscreteHBridge::DirectDiscreteHBridge(const int FPIN_GEN, const int RPIN_GEN, const int FPIN, const int RPIN, bool upsideDown)
  : OutputDevice(InputPowerPercent, upsideDown), currentPower(0), fpwm_handle(setupPwmWrite(FPIN_GEN, FPIN)),
    rpwm_handle(setupPwmWrite(RPIN_GEN, RPIN))
{}

void DirectDiscreteHBridge::move(const long movement)
{
	int mov = movement;
	int pwm = 0;
  if(enabled) //if the manager turned off the device but we somehow got a movement command, perform no output
  {

    //if mounted upside down then invert the signal passed to it and move accordingly
    if (invert)
    {
      //inverts the input easily
      mov = -mov;
    }
    
    currentPower = mov;
    
    //if supposed to move backwards
    if(mov < 0)
    {
      mov = abs(mov);
      pwm = map(mov, 0, POWERPERCENT_MAX, PWM_MIN, PWM_MAX);

      //stop the transistor for the other direction -- if both were on, the h bridge would short out
      pwmWriteDuty(fpwm_handle, 0);
      pwmWriteDuty(rpwm_handle, pwm);
    }

    //if forwards
    else if(mov > 0)
    {
      pwm = map(mov, 0, POWERPERCENT_MAX, PWM_MIN, PWM_MAX);

      //stop the transistor for the other direction -- if both were on, the h bridge would short out
      pwmWriteDuty(fpwm_handle, pwm);
      pwmWriteDuty(rpwm_handle, 0);
    }

    //stop
    else if(mov == 0)
    {
      stop();
    }
  }

	return;
}

void DirectDiscreteHBridge::setPower(bool powerOn)
{
  if(powerOn == false)
  {
    stop();
  }
  enabled = powerOn;
}


long DirectDiscreteHBridge::getCurrentMove()
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

void DirectDiscreteHBridge::stop()
{
  pwmWriteDuty(fpwm_handle, 0);
  pwmWriteDuty(rpwm_handle, 0);
  
  currentPower = 0;
}
