#include "AbstractFramework.h"

//function that checks to see if the user put in a proper input value when calling the runOutputControl function
//returns true if the input is in a valid range, false if it's not
bool JointInterface::verifyInput(long inputToVerify)
{
  long valueMin;
  long valueMax;

  if(inType == spd)
  {
    valueMin = SPEED_MIN;
    valueMax = SPEED_MAX;
  }
  else if(inType == pos)
  {
    valueMin = POS_MIN;
    valueMax = POS_MAX;
  }
  //if any other intypes are made, put them here
  else
  {
    valueMin = 0;
    valueMax = 0;
  }

  if(valueMin <= inputToVerify && inputToVerify <= valueMax)
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

void JointInterface::coupleJoint(JointInterface* otherJoint)
{
  //only couple the joint if it isn't already coupled to avoid infinite recursion
  if(!coupled)
  {
    coupledJoint = otherJoint;
    coupled = true;
    otherJoint -> coupleJoint(this);
  }
}

                                            
//if this IOAlgorithm uses feedback device, this function is used by the joint interface to set it, and sets the feedbackInitialized flag to true
void IOAlgorithm::setFeedDevice(FeedbackDevice* fdDev)
{
  feedbackDev = fdDev;
  feedbackInitialized = true;
}