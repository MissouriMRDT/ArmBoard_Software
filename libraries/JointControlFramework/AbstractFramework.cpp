#include "AbstractFramework.h"

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

void IOAlgorithm::setFeedDevice(FeedbackDevice* fdDev)
{
  feedbackDev = fdDev;
  feedbackInitialized = true;
}