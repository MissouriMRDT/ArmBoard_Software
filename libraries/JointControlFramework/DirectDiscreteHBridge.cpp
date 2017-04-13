#include "DirectDiscreteHBridge.h"

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

	//if mounted upside down then invert the signal passed to it and move accordingly
	if (invert)
	{
		//inverts the input easily
		mov = -mov;
	}

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

	return;
}