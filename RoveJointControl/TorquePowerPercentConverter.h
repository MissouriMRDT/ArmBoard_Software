/*
 * TorquePowerPercentConverter.h
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#ifndef ROVEJOINTCONTROL_TORQUEPOWERPERCENTCONVERTER_H_
#define ROVEJOINTCONTROL_TORQUEPOWERPERCENTCONVERTER_H_

#include "AbstractFramework.h"
#include "RoveJointUtilities.h"

typedef enum TorqueConverterMotorTypes
{
  TorqueConvert_BrushedDC
} TorqueConverterMotorTypes;

class TorquePowerPercentConverter: public DrivingAlgorithm
{
  private:
    TorqueConverterMotorTypes motorType;
    const float KI;
    const FeedbackDevice* VoltSensor;
    const bool voltConverterUsed;
    const unsigned int staticMilliVolts;


    long runAlgorithm(const long input, bool * ret_OutputFinished);

  public:
    TorquePowerPercentConverter(TorqueConverterMotorTypes motor_type, float Ki, FeedbackDevice *voltSensor);
    TorquePowerPercentConverter(TorqueConverterMotorTypes motor_type, float Ki, int staticMillivolts);

};




#endif /* ROVEJOINTCONTROL_TORQUEPOWERPERCENTCONVERTER_H_ */
