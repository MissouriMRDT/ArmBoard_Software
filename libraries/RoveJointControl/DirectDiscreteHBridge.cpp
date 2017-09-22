#include "DirectDiscreteHBridge.h"
#include "RoveBoard.h"

//Value ranges for conversting input to expected output when sending
static const int PWM_MIN = 0, PWM_MAX = 255;

DirectDiscreteHBridge::DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown) : OutputDevice()
{
	FPWM_PIN = FPIN;
	RPWM_PIN = RPIN;
	inType = InputPowerPercent;
	invert = upsideDown;
	currentPower = 0;
}

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
      pwmWrite(FPWM_PIN, 0);
      pwmWrite(RPWM_PIN, pwm);
    }

    //if forwards
    else if(mov > 0)
    {
      pwm = map(mov, 0, POWERPERCENT_MAX, PWM_MIN, PWM_MAX);

      //stop the transistor for the other direction -- if both were on, the h bridge would short out
      pwmWrite(FPWM_PIN, pwm);
      pwmWrite(RPWM_PIN, 0);
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
  pwmWrite(RPWM_PIN, 0);
  pwmWrite(FPWM_PIN, 0);
  
  currentPower = 0;
}
