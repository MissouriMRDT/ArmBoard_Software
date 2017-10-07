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

class TtoPPOpenLConverter: public DrivingAlgorithm
{
  private:
    TorqueConverterMotorTypes motorType;
    const float KT;
    FeedbackDevice* const VoltSensor;
    const bool voltConverterUsed;
    const unsigned int staticMilliVolts;
    const int motorR_mOhm;

    long runAlgorithmBrushedDC(const long torque_milliNewtons);

    long runAlgorithm(const long input, bool * ret_OutputFinished);

  public:
    TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, FeedbackDevice *voltSensor);
    TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, int staticMillivolts);

};




#endif /* ROVEJOINTCONTROL_TORQUEPOWERPERCENTCONVERTER_H_ */
