/*
 * GravityCompensator.cpp
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#include "GravityCompensator.h"
#include "RoveBoard.h"

long GravityCompensator::addToOutput(const long inputValue, const long calculatedOutput)
{
  double gravTorque;
  long gravPwm;
  bool dummy;

  gravTorque = systemStatus->getGravity(jointId);
  gravPwm = torqueConverter->runAlgorithm(gravTorque, &dummy);

  if(supportUsed)
  {
    gravPwm += supportingAlgorithm->addToOutput(inputValue, calculatedOutput + gravPwm);
  }

  return gravPwm;
}

GravityCompensator::GravityCompensator(GravityInertiaSystemStatus* sysStatus, DrivingAlgorithm* torquePowerPercentConverter, uint8_t joint_Id)
  : SupportingAlgorithm(InputPosition, InputPowerPercent), systemStatus(sysStatus), jointId(joint_Id), torqueConverter(torquePowerPercentConverter)
{
  if(!(torquePowerPercentConverter->getInType() == InputTorque && torquePowerPercentConverter->getOutType() == InputPowerPercent))
  {
    debugFault("GravityCompensator constructor: passed torque converter class doesn't have the right input type or output type");
  }
}
