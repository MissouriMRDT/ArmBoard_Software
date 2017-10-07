/*
 * TorquePowerPercentConverter.cpp
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#include "TtoPPOpenLConverter.h"
#include "RoveBoard.h"

TtoPPOpenLConverter::TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Ki, int staticMillivolts)
    : DrivingAlgorithm(InputTorque, InputPowerPercent), motorType(motor_type), KI(Ki), staticMilliVolts(staticMillivolts),
      voltConverterUsed(false), VoltSensor(0)
{

}

TtoPPOpenLConverter::TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Ki, FeedbackDevice *voltSensor)
    : DrivingAlgorithm(InputTorque, InputPowerPercent), motorType(motor_type), KI(Ki), VoltSensor(voltSensor),
      voltConverterUsed(true), staticMilliVolts(0)
{
  if(voltSensor->getFeedbackType() != InputVoltage)
  {
    debugFault("TorquePowerPercentConverter: feedback device doesn't output voltage");
  }
}

long TtoPPOpenLConverter::runAlgorithm(const long input, bool * ret_OutputFinished)
{
  *ret_OutputFinished = true; //open loop, so output is always finished per se


}


