#include "RotateJoint.h"

RotateJoint::RotateJoint(ValueType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface()
{
  inType = inputType;
  controller1 = cont1;
  controller2 = cont2;
  algorithmUsed = false;
  
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, both output devices line up properly, etc
  validConstruction = (inputType == cont1->inType) && (cont1->inType == cont2->inType);
}

RotateJoint::RotateJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface()
{
  inType = inputType;
  controller1 = cont1;
  controller2 = cont2;
  feedback = feed;
  manip = alg;
  algorithmUsed = true;
  
  manip -> setFeedDevice(feed);
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc
  validConstruction = (inputType == alg->inType) && (cont1->inType == alg->outType) && (alg->feedbackInType == feed->fType) && (cont1->inType == cont2->inType);
}

RotateJoint::~RotateJoint()
{
  //deliberately left blank; we don't want to destroy the pointers to other classes as the main program could still be handling them
}

JointControlStatus RotateJoint::runOutputControl(const long movement)
{
  if(!controller1->enabled || !controller2->enabled) return DeviceDisabled;
  if(!verifyInput(movement)) return InvalidInput;
  if(!validConstruction) return InvalidConstruction;
  
  long mov;
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

  controller1->move(motorOneSpeed);
  controller2->move(motorTwoSpeed);
  
  //if motionComplete returned false but movement is 0, that's an indication that an error state occured
  if (motionComplete)
	return OutputComplete;
  else if(mov != 0)
	return OutputRunning;
  else
    return InvalidInput;
}