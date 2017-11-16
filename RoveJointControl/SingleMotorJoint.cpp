#include "SingleMotorJoint.h"
#include "RoveJointUtilities.h"

SingleMotorJoint::SingleMotorJoint(ValueType inputType, IOConverter *alg, OutputDevice* cont) : JointInterface(inputType, alg, cont)
{
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc
  if((inputType == alg->inType) && (cont->inType == alg->outType))
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

SingleMotorJoint::SingleMotorJoint(ValueType inputType, OutputDevice* cont) : JointInterface(inputType, cont)
{
  //checks to make sure the passed arguments all work with each other, that is that device's input type is the same as what the user is putting in
  if(inputType == cont->inType)
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

SingleMotorJoint::~SingleMotorJoint()
{
  //deliberately left blank; we don't want to destroy the pointers to other classes as the main program could still be handling them
}

JointControlStatus SingleMotorJoint::runOutputControl(const long movement)
{
	long mov; //var used as interrum value since algorithm can change the value
  bool motionComplete;
  JointControlStatus returnStatus;

  if(!enabled)
  {
    returnStatus = DeviceDisabled;
  }
  else if(verifyInput(movement) == false)
  {
    returnStatus = InvalidInput;
  }

  else if(!validConstruction)
  {
    returnStatus = InvalidConstruction;
  }
  else
  {
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
    
    //if motionComplete returned false but movement is 0, that's an indication that an error state occured
    if(motionComplete == false && mov == 0)
    {
      returnStatus = AlgorithmError;
    }
    else
    {
      if(motionComplete == true)
      {
        returnStatus = OutputComplete;
      }
      else
      {
        returnStatus = OutputRunning;
      }

      //moves device with output decided on by the algorithm
      controller1->move(mov);
    }
  }

	return(returnStatus);
}

void SingleMotorJoint::stop()
{
  controller1->stop();
}

void SingleMotorJoint::disableJoint()
{
  controller1->setPower(false);
  enabled = false;
}

void SingleMotorJoint::enableJoint()
{
  controller1->setPower(true);
  enabled = true;
}
