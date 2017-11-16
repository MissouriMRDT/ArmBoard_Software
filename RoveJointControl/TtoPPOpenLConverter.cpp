/*
 * TorquePowerPercentConverter.cpp
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#include "TtoPPOpenLConverter.h"
#include "RoveJointUtilities.h"
#include "RoveBoard.h"

TtoPPOpenLConverter::TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, int staticMillivolts)
    : IOConverter(InputTorque, InputPowerPercent), motorType(motor_type), KT(Kt), staticMilliVolts(staticMillivolts),
      voltConverterUsed(false), VoltSensor(0), motorR_mOhm(motResistance_milliOhms)
{
  if(abs(staticMillivolts) > VOLT_MAX)
  {
    debugFault("TtoPPOpenLConverter: value of static millivolts was set higher than the system can support");
  }
}

TtoPPOpenLConverter::TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, FeedbackDevice *voltSensor)
    : IOConverter(InputTorque, InputPowerPercent), motorType(motor_type), KT(Kt), VoltSensor(voltSensor),
      voltConverterUsed(true), staticMilliVolts(0), motorR_mOhm(motResistance_milliOhms)
{
  if(voltSensor->getFeedbackType() != InputVoltage)
  {
    debugFault("TtoPPOpenLConverter: feedback device doesn't output voltage");
  }
}

long TtoPPOpenLConverter::runAlgorithm(const long input, const long oldOutput, bool * ret_OutputFinished)
{
  *ret_OutputFinished = true; //open loop, so output is always finished per se
  long newOutput;

  if(motorType == TorqueConvert_BrushedDC)
  {
    newOutput = runAlgorithmBrushedDC(input);
  }
  else
  {
    debugFault("TtoPPOpenLConverter: motor type not supported");
    newOutput = 0;
  }

  if(supportUsed && ((supportIsPersistant && newOutput == 0) || newOutput != 0))
  {
    newOutput += supportingAlgorithm->addToOutput(input, newOutput + oldOutput);
  }

  return newOutput;
}

//function to be called when class is acting as a support algorithm to another IOConverter.
long TtoPPOpenLConverter::addToOutput(const long inputValue, const long calculatedOutput)
{
  bool dummy;
  return runAlgorithm(inputValue, calculatedOutput, &dummy);
}

long TtoPPOpenLConverter::runAlgorithm(const long input, bool * ret_OutputFinished)
{
  return runAlgorithm(input, 0, ret_OutputFinished);
}

long TtoPPOpenLConverter::runAlgorithmBrushedDC(const long torque_milliNewtons)
{
  float f_motorR_mOhm = motorR_mOhm;
  float f_torque_milliNewtons = abs(torque_milliNewtons);
  float milliVoltsToApply;
  int powPercent;
  float milliVoltsAvailable;

  if(torque_milliNewtons == 0)
  {
    return 0;
  }
  else if(voltConverterUsed)
  {
    milliVoltsAvailable = VoltSensor->getFeedback();
  }
  else
  {
    milliVoltsAvailable = staticMilliVolts;
  }

  //for brushed DC, torque is just a linear scaling of voltage using the equation
  //Vapplied = (L/Kt) * (dT/dt) + (R/Kt) * T + Kw * W.
  //There's some idealization there with motor windings and such, but in general the error is fairly small.
  //As such, since it's an entirely linear equation, we can use superposition to just say
  //Vapplied_to_torque = (R/Kt) * T
  milliVoltsToApply = (((f_motorR_mOhm / 1000.0) / KT) * (f_torque_milliNewtons / 1000.0)) * 1000.0;

  if(milliVoltsToApply > milliVoltsAvailable)
  {
    powPercent = POWERPERCENT_MAX * sign(torque_milliNewtons);
  }
  else
  {
    powPercent = ((milliVoltsToApply / milliVoltsAvailable) * 100.0) * ((float)PERCENT_TO_POWERPERCENT) * sign(torque_milliNewtons);
  }

  if(powPercent > POWERPERCENT_MAX)
  {
    powPercent = POWERPERCENT_MAX;
  }
  else if(powPercent < POWERPERCENT_MIN)
  {
    powPercent = POWERPERCENT_MIN;
  }

  return powPercent;
}

