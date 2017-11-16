#include "AbstractFramework.h"
#include "RoveJointUtilities.h"

bool JointInterface::verifyInput(long inputToVerify)
{
  switch(inType)
  {
	  case InputSpeed:
		return SPEED_MIN <= inputToVerify && inputToVerify <= SPEED_MAX;
	  case InputPosition:
		return POS_MIN <= inputToVerify && inputToVerify <= POS_MAX;
	  case InputPowerPercent:
		return POWERPERCENT_MIN <= inputToVerify && inputToVerify <= POWERPERCENT_MAX;
	  case InputTorque:
	  return TORQUE_MIN <= inputToVerify && inputToVerify <= TORQUE_MAX;
	  case InputVoltage:
	  return VOLT_MIN <= inputToVerify && inputToVerify <= VOLT_MAX;
	  default:
	    return inputToVerify == 0;
  }
}

bool JointInterface::switchModules(ValueType newInputType, IOConverter* newAlgorithm)
{
  if((newInputType == newAlgorithm->inType) && (controller1->inType == newAlgorithm->outType))
  {
    manip = newAlgorithm;
    inType = newInputType;
    
    algorithmUsed = true;
	
    return(true);
  }
  else
  {
    return(false);
  }
}

bool JointInterface::switchModules(ValueType newInputType, IOConverter* newAlgorithm, OutputDevice* newDevice)
{
  if((newInputType == newAlgorithm->inType) && (newDevice->inType == newAlgorithm->outType))
   {
     manip = newAlgorithm;
     inType = newInputType;
     controller1 = newDevice;

     algorithmUsed = true;

     return(true);
   }
   else
   {
     return(false);
   }
}

bool JointInterface::switchModules(ValueType newInputType, OutputDevice* newDevice)
{
  bool valid = false;

  if(algorithmUsed)
  {
    if(((newInputType == manip->inType) && newDevice->inType == manip->outType))
    {
      valid = true;
    }
  }
  else if((newInputType == newDevice->inType))
  {
    valid = true;
  }

  if(!valid)
  {
    return false;
  }
  else
  {
    inType = newInputType;
    controller1 = newDevice;
    return(true);
  }
}

bool JointInterface::removeIOConverter(ValueType newInputType)
{
  if(!algorithmUsed)
  {
    return(true);
  }
  else if(newInputType == controller1->inType)
  {
    inType = newInputType;
    manip = 0;
    algorithmUsed = false;
	
    return(true);
  }
  else
  {
    return(false);
  }
}

bool JointInterface::removeIOConverter(ValueType newInputType, OutputDevice* newDevice)
{
  if(newInputType == newDevice->inType)
  {
    inType = newInputType;
    manip = 0;
    algorithmUsed = false;
    controller1 = newDevice;

    return(true);
  }
  else
  {
    return(false);
  }
}

bool IOConverter::addSupportingAlgorithm(IOConverter* support)
{
  if(outType == support->outType && inType == support->inType)
  {
    supportingAlgorithm = support;
    supportUsed = true;
    return true;
  }
  else
  {
    return false;
  }
}

void IOConverter::persistantSupport(bool persistant)
{
  supportIsPersistant = persistant;
}
