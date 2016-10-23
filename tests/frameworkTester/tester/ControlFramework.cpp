#include "ControlFramework.h";

//constructor for single motor joints with feedback device
SingleMotorJoint::SingleMotorJoint(InputType inputType, OutputDevice* cont, FeedbackDevice* feed) : JointInterface()
{
	//assignments
	inType = inputType;
	controller = cont;
	feedback = feed;
		
	//selects algorithm 
	selector(feedback, controller->outType, manip);
}

//constructor for single motor joints without feedback
SingleMotorJoint::SingleMotorJoint(InputType inputType, OutputDevice* cont) : JointInterface()
{
	//assignments
	inType = inputType;
	controller = cont;

	//selects algorithm (only spd to spd is implemented)
	selector(controller->outType, manip);
}

//deletes pointers used in single motor joint to 'prevent memory leaks'. Most likely not neccessary but good practice.
SingleMotorJoint::~SingleMotorJoint()
{
	delete controller;
	delete manip;
	delete feedback;
}

//run the output algorithm for this tilt joint correctly (I mean, hopefully).
//calls the algorithm to manipulate the input and sends it to the motor device.
void SingleMotorJoint::runOutputControl(const int movement) 
{
	//var used as interrum value since algorithm can change the value
	int mov;

	//calls algorithm
	mov = manip->runAlgorithm(movement);

	//moves device with output decided on by the algorithm
	controller->move(mov);

	return;
}

//creates the joint interface for a tilt joint with a feedback device.
//Note both output devices are assumed to have the same input type
TiltJoint::TiltJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;
	feedback = feed;	

	//select algorithm
	selector(feedback, controller1->outType, manip);    
}

//creates joint interface for a tilt joint with no feedback.
//Note both output devices are assumed to have the same input type
TiltJoint::TiltJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;
	
	//select algorithm
	selector(controller1->outType, manip);
}

//Destructor for the tilt joint since it has pointers
TiltJoint::~TiltJoint()
{
	delete controller1;
	delete controller2;
	delete manip;
	delete feedback;
}

//run the output algorithm for this tilt joint correctly (I mean, hopefully).
//calls the algorithm to manipulate the input and sends it to each controller.
//Both devices get the same command since they're supposed to move together.
void TiltJoint::runOutputControl(const int movement)
{
	//largely a temp value to store any modifications made to the input
	int mov;

	//runs the algorithm on the input
	mov = manip->runAlgorithm(movement);

	//send to the motor move command
	controller1->move(mov);

	//both the controllers should move the arm in the same direction. send command to motor 2
	controller2->move(mov);

	return;
}

//constructor for the rotate joint class without feedback.
//Assumes both passed devices have the same input type
RotateJoint::RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;

	//since both of the controllers use the same inputType, then just use one (the first one)
	selector(controller1->outType, manip);
}

//constructor for the rotate joint class with feedback.
//Assumes both passed devices have the same input type.
RotateJoint::RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;
	feedback = feed;

	//just use the first controller
	selector(feedback, controller1->outType, manip);
}

//rotate joint deconstructor. Deletes pointers
RotateJoint::~RotateJoint() 
{
	delete controller1;
	delete controller2;
	delete manip;
	delete feedback;
}

//run the output algorithm for this tilt joint correctly (I mean, hopefully).
//calls the algorithm to manipulate the input and sends it to each controller.
//One device gets an inverted direction from the other, since they move in opposite tandem on rotate joints.
void RotateJoint::runOutputControl(const int movement)
{
	//largely a temp value to store any modifications made to the input
	int mov;

	//runs the algorithm on the input
	mov = manip->runAlgorithm(movement);

	//send to the motor move command
	controller1->move(mov);

	//since the motors are supposed to work against eachother to rotate send the negative to controller 2
	mov = -mov;

	//send command to motor 2
	controller2->move(mov);

	return;
}

//constructor for the Sdc2130 when controlled via pwm.
//Note that inType of speed is the only one currently implemented
Sdc2130::Sdc2130(const int pwmPin, InputType inType, bool upsideDown): OutputDevice()
{
	PWM_PIN = pwmPin;
	outType = inType;
	invert = upsideDown;
	controlType = Pwm;
	pwmVal = 0;
}

//sdc2130 general move function. Selects the specific move function
//based on the specified movement type, such as speed or position
void Sdc2130::move(const int movement)
{
	if(outType == spd)
	{
		moveSpeed(movement);
	}
	//position-input movement not implemented
	else if(outType == pos)
	{
		
	}
}

//sdc2130 movement command for moving based on a speed input.
//Simply causes the joint to move a couple of positional ticks,
//as true speed based movement is not implemented due to strange delays in sdc2130 response
void Sdc2130::moveSpeed(const int movement)
{
	int speed = movement;
	
	if(invert)
	{
		speed = -speed;
	}
	
	if(controlType == Pwm)
	{
		if(speed > 0)
		{
			pwmVal+=POS_INC;
		}
		else
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
		
		analogWrite(PWM_PIN, pwmVal);
		
	}
	
	//serial control not implemented
	else
	{
		
	}
}


//Creates the device. Assigns the pins correctly.
//Make sure to put pins in correctly
DirectDiscreteHBridge::DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown) : OutputDevice()
{
	FPWM_PIN = FPIN;
	RPWM_PIN = RPIN;
	outType = spd;
	invert = upsideDown;
}

//moves by passing a pwm signal to the H bridge
void DirectDiscreteHBridge::move(const int movement)
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
		pwm = map(mov, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);

		//stop the transistor for the other direction -- if both were on, the h bridge would short out
		analogWrite(FPWM_PIN, 0);    
		analogWrite(RPWM_PIN, pwm);
	}

	//if forwards
	else if(mov > 0)
	{
		pwm = map(mov, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);

		//stop the transistor for the other direction -- if both were on, the h bridge would short out
		analogWrite(FPWM_PIN, pwm);
		analogWrite(RPWM_PIN, 0);    
	}

	//stop
	else if(mov == 0)
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

