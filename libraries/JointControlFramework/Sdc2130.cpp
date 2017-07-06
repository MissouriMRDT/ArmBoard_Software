#include "Sdc2130.h"
#include "PwmWriter.h"

/*constructor for the Sdc2130 when controlled via pwm.
  Note that inType of speed is the only one currently implemented.
  inputs:
  pwmPin: GPIO pin that's connected to the pwm input of the SDC2130 device. GPIO pin number id's are defined by energia's pinmapping
  inType: instance of the ValueType enum that defines what kind of input the device should take, currently pos or spd. The motor controller can move based on either, so pick one
  upside down: whether or not the motor is mounted in reverse so the input values would also need to be inverted
 */
Sdc2130::Sdc2130(const int pwmPin, ValueType inType, bool upsideDown): OutputDevice()
{
	PWM_PIN = pwmPin;
	inType = inType;
	invert = upsideDown;
	controlType = Pwm;
	pwmVal = 0;
}

//sdc2130 general move function. Selects the specific move function
//based on the specified movement type, such as speed or position
//Input: Can be either position or speed values constrained between SPEED_MIN and SPEED_MAX or POS_MIN and POS_MAX
void Sdc2130::move(const long movement)
{
	if(inType == spd)
	{
		moveSpeed(movement);
	}
	//position-input movement not implemented
	else if(inType == pos)
	{

	}
}

//sdc2130 movement command for moving based on a speed input, IE a value constrained between the SPEED_MIN and SPEED_MAX constants.
//Simply causes the joint to move a couple of positional ticks,
//as true speed based movement is not implemented due to strange delays in sdc2130 response
void Sdc2130::moveSpeed(const int movement)
{
	int speed = movement;

  if(enabled) //only move if device has been enabled by the user
  {
    if(invert)
    {
      speed = -speed;
    }

    currentSpeed = speed;
    
    if(controlType == Pwm)
    {
      if(speed > 0)
      {
        pwmVal+=POS_INC;
      }
      else if(speed < 0)
      {
        pwmVal-=POS_INC;
      }

      if(pwmVal < PWM_MIN)
      {
        pwmVal = PWM_MIN;
      }
      else if(pwmVal > PWM_MAX)
      {
        pwmVal = PWM_MAX;
      }

      PwmWrite(PWM_PIN, pwmVal);

    }

    //serial control not implemented
    else
    {

    }
  }
}

//Tells device to behave as if it's on or off; that is, if it's off, stop and refuse to perform output until user re-enables
void Sdc2130::setPower(bool powerOn)
{
  if(powerOn == false)
  {
    moveSpeed(0);
  }
  
  enabled = powerOn;
}

long Sdc2130::getCurrentMove()
{
  if(inType == spd)
  {
    if(invert) //if we're inverted, then we technically move negatively even if we're moving in the 'positive' direction. The direction is the important part
    {
      return(currentSpeed * -1); 
    }
    else
    {
      return(currentSpeed);
    }
  }
  else if(inType == pos)
  {
    return(currentPosition);
  }
}