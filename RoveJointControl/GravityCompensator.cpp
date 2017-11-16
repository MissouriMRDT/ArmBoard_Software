/*
 * GravityCompensator.cpp
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#include "GravityCompensator.h"
#include "RoveBoard.h"
#include "RoveJointUtilities.h"

long GravityCompensator::addToOutput(const long inputValue, const long calculatedOutput)
{
  double gravTorque;
  long gravPwm;
  bool dummy;

  gravTorque = -1 * systemStatus->getGravity(jointId);

  if(gravTorque != 0)
  {
    gravPwm = ((DrivingAlgorithm*)(&torqueConverter))->runAlgorithm(gravTorque, &dummy);
  }
  else
  {
    gravPwm = 0;
  }

  if(supportUsed)
  {
    gravPwm += supportingAlgorithm->addToOutput(inputValue, calculatedOutput + gravPwm);
  }

  return gravPwm * scalar;
}

GravityCompensator::GravityCompensator(GravityInertiaSystemStatus* sysStatus, TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, int staticMillivolts, uint8_t joint_Id)
  : SupportingAlgorithm(InputPosition, InputPowerPercent), systemStatus(sysStatus), jointId(joint_Id), torqueConverter(motor_type, Kt, motResistance_milliOhms, staticMillivolts), scalar(1)
{
}

GravityCompensator::GravityCompensator(GravityInertiaSystemStatus* sysStatus, TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, FeedbackDevice* fdev, uint8_t joint_Id)
  : SupportingAlgorithm(InputPosition, InputPowerPercent), systemStatus(sysStatus), jointId(joint_Id), torqueConverter(motor_type, Kt, motResistance_milliOhms, fdev), scalar(1)
{
}

void GravityCompensator::setScalar(float scale)
{
  scalar = scale;
}

