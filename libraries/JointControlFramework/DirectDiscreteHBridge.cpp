#include "DirectDiscreteHBridge.h"
#include <pwmWriter.h>
#include "Energia.h"

/* Creates the device. Assigns the pins correctly.
 * int FPIN: the GPIO pin connected to the H bridge's forward transistor, pin number is defined by energia's pinmapping
 * int RPIN: the GPIO pin connected to the H bridge's forward transistor, pin number is defined by energia's pinmapping
 * bool upside down: Whether or not the motor is mounted in reverse and as such the inputs also need to be reversed
 */
DirectDiscreteHBridge::DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown) : OutputDevice()
{
	FPWM_PIN = FPIN;
	RPWM_PIN = RPIN;
	inType = spd;
	invert = upsideDown;
}

//moves by passing a pwm signal to the H bridge.
//Input: expects int values constrained between the SPEED_MIN and SPEED_MAX constants
void DirectDiscreteHBridge::move(const long movement)
{
	int mov = movement;
	int pwm = 0;
  if(enabled) //if the user has not turned this device on, perform no output
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

      //stop the transistor for the other direction -- if both were on, the h bridge would short out
      PwmWrite(FPWM_PIN, 0);
      PwmWrite(RPWM_PIN, pwm);
    }

    //if forwards
    else if(mov > 0)
    {
      pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

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

	return;
}

//Instructs DirectDiscreteHBridge to behave as if it is on or off; that is, when commanded to be off, it will refuse to send any output
void DirectDiscreteHBridge::setPower(bool powerOn)
{
  if(powerOn == false)
  {
    move(0);
  }
  enabled = powerOn;
}


long DirectDiscreteHBridge::getCurrentMove()
{
  return(currentSpeed);
}