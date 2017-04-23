#include "RotateJoint.h"

//constructor for the rotate joint class without feedback.
//Constructor automatically chooses an open loop algorithm that inputs the specified inputType and outputs the values the output device accepts
//Assumes both passed devices have the same input type
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//cont1: The first output device controlling the first motor on this joint
//cont2: The second output device controlling the second motor on this joint
RotateJoint::RotateJoint(ValueType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface(),
  inType(inputType),
  controller1(cont1),
  controller2(cont2),
  algorithmUsed(false)
{
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, both output devices line up properly, etc
  validConstruction = (inputType == cont1->inType) && (cont1->inType == cont2->inType);
}

//constructor for the rotate joint class with feedback.
//Assumes both passed devices have the same input type.
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//alg: the IOAlgorithm this joint should use
//cont1: The first output device controlling the first motor on this joint
//cont2: The second output device controlling the second motor on this joint
//feed: A pointer to the feedback device used on this joint.
RotateJoint::RotateJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface(),
  inType(inputType),
  controller1(cont1),
  controller2(cont2),
  feedback(feed),
  manip(alg),
  algorithmUsed(true)
{
  manip -> setFeedDevice(feed);
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc
  validConstruction = (inputType == alg->inType) && (cont1->inType == alg->outType) && (alg->feedbackInType == feed->fType) && (cont1->inType == cont2->inType);
}

RotateJoint::~RotateJoint()
{
  //deliberately left blank; we don't want to destroy the pointers to other classes as the main program could still be handling them
}

//run the output algorithm for this tilt joint correctly (I mean, hopefully).
//calls the algorithm to manipulate the input and sends it to each controller.
//One device gets an inverted direction from the other, since they move in opposite tandem on rotate joints.
//input: a long that represents the desired movement. What values this int is constrained to depend on what this joint was set up to take as an input.
//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar
//ranges defined in the framework's .h file
//returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
JointControlStatus RotateJoint::runOutputControl(const long movement)
{
  if(!controller1->enabled || !controller2->enabled) return DeviceDisabled;
  if(!verifyInput(movement)) return InvalidInput;
  if(!validConstruction) return InvalidConstruction;
  
  long mov; //var used as interrum value since algorithm can change the value
  bool motionComplete;
  //runs the algorithm on the input. If there is none, output is just passed directly to output device
  if(algorithmUsed)
  {
    mov = manip->runAlgorithm(movement, &motionComplete);
  }
  else
  {
    mov = movement;
    motionComplete = true;
  }

  motorOneSpeed = mov;
  motorTwoSpeed = -mov;

  //this only happens if this joint has been coupled with another joint
  //the coupled logic will modify the speed calculated by the algorithm
  //to allow smooth tilting and rotating at the same time.
  //It is automatically assumed that the other joint is a tilt joint
  if(coupled)
  {
    motorOneSpeed += coupledJoint->motorOneSpeed;
    motorTwoSpeed += coupledJoint->motorTwoSpeed;
    
    motorOneSpeed = constrain(motorOneSpeed, -1000, 1000);
    motorTwoSpeed = constrain(motorTwoSpeed, -1000, 1000);
  }

  //send to the motor move command
  controller1->move(motorOneSpeed);

  //send command to motor 2. Since this is a rotate joint, the motors need to go in opposite directions so send the second one a negafied value
  controller2->move(motorTwoSpeed);
  
  //if motionComplete returned false but movement is 0, that's an indication that an error state occured
  if (motionComplete)
	return OutputComplete;
  else if(mov != 0)
	return OutputRunning;
  else
    return InvalidInput;
}