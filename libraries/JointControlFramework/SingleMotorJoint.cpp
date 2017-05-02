#include "SingleMotorJoint.h"

SingleMotorJoint::SingleMotorJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont, FeedbackDevice* feed) : JointInterface()
{
  inType = inputType;
  controller1 = cont;
  feedback = feed;
  manip = alg;
  algorithmUsed = true;
  manip -> setFeedDevice(feed);
  
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc
  validConstruction =(inputType == alg->inType) && (cont->inType == alg->outType) && (alg->feedbackInType == feed->fType);
}

SingleMotorJoint::SingleMotorJoint(ValueType inputType, OutputDevice* cont) : JointInterface()
{
  inType = inputType;
  controller1 = cont;
  algorithmUsed = false;
  
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc.
  //Technically this should never go wrong as long as the algorithmSelector is working properly, but it never hurts to double check. If indeed the construction
  //winds up being invalid, for debugging try checking the algorithm selector method for bugs
  validConstruction = inputType == cont->inType;
}

SingleMotorJoint::~SingleMotorJoint()
{
  //deliberately left blank; we don't want to destroy the pointers to other classes as the main program could still be handling them
}

JointControlStatus SingleMotorJoint::runOutputControl(const long movement)
{
  if(!controller1->enabled) return DeviceDisabled;
  if(!verifyInput(movement)) return InvalidInput;
  if(!validConstruction) return InvalidConstruction;

  long mov;
  bool motionComplete;
  
  //calls algorithm if there's one used. If not, output passed directly to output device
  if(algorithmUsed)
  {
    mov = manip->runAlgorithm(movement, &motionComplete);
  }
  else
  {
    mov = movement;
    motionComplete = true;
  }
  
  controller1->move(mov);
  
  //if motionComplete returned false but movement is 0, that's an indication that an error state occured
  if (motionComplete)
	  return OutputComplete;
  else if(mov != 0)
	  return OutputRunning;
  else
    return InvalidInput;
}