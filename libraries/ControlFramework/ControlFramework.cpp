#include "ControlFramework.h";

SingleMotorJoint::SingleMotorJoint(InputType inputType, OutputDevice* cont, FeedbackDevice* feed) : JointInterface()
{
  //assignments
  inType = inputType;
  controller = cont;
  feedback = feed;
		
  //selects algorithm (not implemented)
  selector(feedback, controller->outType, manip);
}

SingleMotorJoint::SingleMotorJoint(InputType inputType, OutputDevice* cont) : JointInterface()
{
  //assignments
  inType = inputType;
  controller = cont;
  
  //selects algorithm (only spd to spd is implemented)
  selector(controller->outType, manip);
}

//deletes pointers used in framework to 'prevent memory leaks'. Most likely not neccessary but good practice.
SingleMotorJoint::~SingleMotorJoint()
{
  delete controller;
  delete manip;
  delete feedback;
}

//moves the device based on what was recieved from base station
//may return something later but now just assume it does its thing
void SingleMotorJoint::runOutputControl(const int movement) 
{
  //var used as interrum value since algorithm can change the value
  int mov;
  //calls algorithm
  mov = manip->runAlgorithm(movement);
  //moves device
  controller->move(mov);
  return;
}

//creates the joint interface for a tilt joint(generally also combined with a rotate joint).
TiltJoint::TiltJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface()
{
  //assignments
  inType = inputType;
  controller1 = cont1;
  controller2 = cont2;
  feedback = feed;	

  selector(feedback, controller1->outType, manip);    
}

//creates joint interface with no feedback
TiltJoint::TiltJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface()
{
  inType = inputType;
  controller1 = cont1;
  controller2 = cont2;
  
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

//move the joint correctly (hopefully)
//calls the algorithm to manipulate the input and sends it to each controller
void TiltJoint::runOutputControl(const int movement)
{
  //largely a temp value to store any modifications made to the input
  int mov;
  
  //runs the algorithm on the input
  mov = manip->runAlgorithm(movement);
  
  //contoller 1 and 2 need to be fixed but currently have no defined rule on how each motor is defined.
  //send to the motor move command
  controller1->move(mov);

  //both the controllers should move the arm in the same direction
  //send command to motor 2
  controller2->move(mov);
  
  return;
}

RotateJoint::RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface()
{
  inType = inputType;
  controller1 = cont1;
  controller2 = cont2;
  
  //since both of the controllers use the same inputType, then just use one (the first one)
  selector(controller1->outType, manip);
}

RotateJoint::RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface()
{
  inType = inputType;
  controller1 = cont1;
  controller2 = cont2;
  feedback = feed;

  //just use the first controller
  selector(feedback, controller1->outType, manip);
}

//delete 
RotateJoint::~RotateJoint() 
{
  delete controller1;
  delete controller2;
  delete manip;
  delete feedback;
}

//move the joint correctly (hopefully)
//calls the algorithm to manipulate the input and sends it to each controller
void RotateJoint::runOutputControl(const int movement)
{
  //largely a temp value to store any modifications made to the input
  int mov;
  
  //runs the algorithm on the input
  mov = manip->runAlgorithm(movement);
  
  //contoller 1 and 2 need to be fixed but currently have no defined rule on how each motor is defined.
  //send to the motor move command
  controller1->move(mov);
  
  //since the motors are supposed to work against eachother to rotate send the negative to controller 2
  mov = -mov;
  
  //send command to motor 2
  controller2->move(mov);
  
  return;
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

//moves based on what is passed from algorithm
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
    
    mov = abs(movement);
    pwm = map(mov, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);
    //stop the other motor.
    analogWrite(FPWM_PIN, 0);    
    analogWrite(RPWM_PIN, pwm);
  }

  //if forwards
  else if(mov > 0)
  {
    pwm = mov;
    pwm = map(mov, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);
    //stop the other motor.
    analogWrite(RPWM_PIN, 0);    
    analogWrite(FPWM_PIN, pwm);
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

