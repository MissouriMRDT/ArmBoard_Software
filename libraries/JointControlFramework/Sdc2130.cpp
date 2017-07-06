#include "Sdc2130.h"
#include "PwmWriter.h"

Sdc2130::Sdc2130(const int pwmPin, ValueType inType, bool upsideDown): OutputDevice()
{
	PWM_PIN = pwmPin;
	inType = inType;
	invert = upsideDown;
	controlType = Pwm;
	pwmVal = 0;
}

void Sdc2130::move(const long movement)
{
	switch(inType)
	{
		case spd:
			moveSpeed(movement);
	}
}

void Sdc2130::moveSpeed(const int movement)
{
  if(!enabled) return;

  int speed = invert ? -movement : movement;
  
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

    pwmVal = constrain(pwmVal, PWM_MIN, PWM_MAX);

    PwmWrite(PWM_PIN, pwmVal);
  }
}

void Sdc2130::setPower(bool powerOn)
{
  if(!powerOn) //stop movement before storing into enabled variable, as motor will refuse to accept move commands after being disabled
  {
    move(0);
  }
  
  enabled = powerOn;
}