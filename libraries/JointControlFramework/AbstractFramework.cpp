#include "AbstractFramework.h"

//function that checks to see if the user put in a proper input value when calling the runOutputControl function
//returns true if the input is in a valid range, false if it's not
bool JointInterface::verifyInput(long inputToVerify)
{
  switch(inType)
  {
	  case spd:
		return SPEED_MIN <= inputToVerify && inputToVerify <= SPEED_MAX;
	  case pos:
		return POS_MIN <= inputToVerify && inputToVerify <= POS_MIN;
	  default:
	    return inputToVerify == 0;
  }
}

void JointInterface::coupleJoint(JointInterface* otherJoint)
{
  this->coupledJoint = otherJoint;
  this->coupled = true;
  otherJoint->coupledJoint = this;
  otherJoint->coupled = true;
}

                                            
//if this IOAlgorithm uses feedback device, this function is used by the joint interface to set it, and sets the feedbackInitialized flag to true
void IOAlgorithm::setFeedDevice(FeedbackDevice* fdDev)
{
  feedbackDev = fdDev;
  feedbackInitialized = true;
}