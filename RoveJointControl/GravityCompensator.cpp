/*
 * GravityCompensator.cpp
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#include "GravityCompensator.h"
#include "RoveBoard.h"
#include "RoveJointUtilities.h"

const uint32_t DefaultLoopsTillErrorComp = 10;
const uint32_t DefaultAcceptableError = 1000; //milli newton meters
const uint32_t DefaultErrorReadingDeadband = 100; //milli newton meters

long GravityCompensator::runAlgorithm(const long input, bool * ret_OutputFinished)
{
  //if runAlgorithm is called this might be the only alg called, so use the input as also being the output so this class doesn't just
  //kill all movement
  *ret_OutputFinished = true;
  return addToOutput(input, input) + input;
}

long GravityCompensator::addToOutput(const long inputValue, const long calculatedOutput)
{
  double gravTorque;
  long gravPwm;
  bool dummy;

  gravTorque = -1 * systemStatus->getGravity(jointId);

  if(useErrorComp && abs(calculatedOutput) != POWERPERCENT_MAX && calculatedOutput != 0) //don't change compensation if motor's already saturated
  {
    int64_t signedTorqueCalculationError = (gravTorque - errorTorqueSensor->getFeedback()) + compensationValue;
    int64_t signedDiffFromLastError = abs(signedTorqueCalculationError) - lastError;

    if(abs(signedTorqueCalculationError) > acceptableError && abs(signedDiffFromLastError) <= errorReadingDeadband)
    {
      //error hasn't changed much from last check, and it's more error than we want so we want to compensate for it. So,
      //change loop counter on error compensation. If the counter times out, then start compensating for the error in
      //our torque calculations.
      loopsLeftTillErrorComp--;
      if(loopsLeftTillErrorComp <= 0)
      {
        loopsLeftTillErrorComp = loopsTillErrorComp;
        compensationValue = signedTorqueCalculationError;
      }
    }
    else
    {
      //error wavered or went into acceptable range, reset loop counter on error compensation
      loopsLeftTillErrorComp = loopsTillErrorComp;
    }
  }

  if(gravTorque != 0)
  {
    gravPwm = ((IOConverter*)(&torqueConverter))->runAlgorithm(gravTorque, &dummy) + compensationValue;
  }
  else
  {
    gravPwm = 0;
  }

  if(supportUsed && (calculatedOutput != 0 || supportIsPersistant))
  {
    gravPwm += supportingAlgorithm->addToOutput(inputValue, calculatedOutput + gravPwm);
  }

  return gravPwm * scalar;
}

GravityCompensator::GravityCompensator(GravityInertiaSystemStatus* sysStatus, TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, int staticMillivolts, uint8_t joint_Id)
  : IOConverter(InputPosition, InputPowerPercent), systemStatus(sysStatus), jointId(joint_Id), torqueConverter(motor_type, Kt, motResistance_milliOhms, staticMillivolts), scalar(1),
    useErrorComp(false), loopsTillErrorComp(DefaultLoopsTillErrorComp), acceptableError(DefaultAcceptableError), lastError(0), errorReadingDeadband(DefaultErrorReadingDeadband), compensationValue(0)
{
}

GravityCompensator::GravityCompensator(GravityInertiaSystemStatus* sysStatus, TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, FeedbackDevice* fdev, uint8_t joint_Id)
  : IOConverter(InputPosition, InputPowerPercent), systemStatus(sysStatus), jointId(joint_Id), torqueConverter(motor_type, Kt, motResistance_milliOhms, fdev), scalar(1),
    useErrorComp(false), loopsTillErrorComp(DefaultLoopsTillErrorComp), acceptableError(DefaultAcceptableError), lastError(0), errorReadingDeadband(DefaultErrorReadingDeadband), compensationValue(0)
{
}

void GravityCompensator::setScalar(float scale)
{
  scalar = scale;
}

void GravityCompensator::useErrorCompensation(FeedbackDevice *torqueDev)
{
  if(torqueDev->getFeedbackType() == InputTorque)
  {
    errorTorqueSensor = torqueDev;
    useErrorComp = true;
  }
  else
  {
    debugFault("Gravity Compensator.useDynamicLoadCompensation: passed feedback device didn't have correct input type");
  }
}

void GravityCompensator::setLoopsTillErrorCompensation(uint32_t loopsTillErrorCompensation)
{
  loopsTillErrorComp = loopsTillErrorCompensation;
}

void GravityCompensator::setAcceptableErrorRange(uint32_t errorRange_milliNewtonMeters)
{
  acceptableError = errorRange_milliNewtonMeters;
}

void GravityCompensator::setErrorReadingDeadband(uint32_t deadband)
{
  errorReadingDeadband = deadband;
}
