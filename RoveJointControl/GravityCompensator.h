/*
 * GravityCompensator.h
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#ifndef ROVEJOINTCONTROL_GRAVITYCOMPENSATOR_H_
#define ROVEJOINTCONTROL_GRAVITYCOMPENSATOR_H_

#include "RoveJointUtilities.h"
#include "TtoPPOpenLConverter.h"
#include <stdint.h>
#include "RoveJointControl/AbstractFramework.h"
#include "RoveJointControl/GravityInertiaSystemStatus.h"

//class that tries to account for gravity during joint motion. Designed to act as a supporting IOConverter to another IOConverter rather than
//usually directly controlling a joint itself.
//see the README.md for more info
class GravityCompensator: public IOConverter
{
  private:

    long addToOutput(const long inputValue, const long calculatedOutput);
    long runAlgorithm(const long input, bool * ret_OutputFinished);

    uint8_t jointId;
    GravityInertiaSystemStatus* systemStatus;
    TtoPPOpenLConverter torqueConverter;
    float scalar;

    FeedbackDevice *errorTorqueSensor;
    bool useErrorComp;
    uint32_t loopsTillErrorComp;
    uint32_t acceptableError;
    uint32_t loopsLeftTillErrorComp;
    uint32_t lastError;
    uint32_t errorReadingDeadband;
    int64_t  compensationValue;

  public:

    //public constructor, when the motor's voltage line doesn't vary in voltage level.
    //Inputs:
    //        sysStatus: A pointer to the gravity inertia system status that will support this class. GravInertSysStatus will handle
    //                   crunching most of the hard math for gravity compensation while this class acts mostly as an interface between it
    //                   and the main driving algorithm. Therefore, GravInertSysStatus needs to have its update() function called periodically
    //                   so that it's periodically performing gravity calculations.
    //        motorType: What kind of motor type is attached to this joint, in the TorqueConverterMotorTypes enum
    //        Kt:        The motor's torque-to-amperage ratio, in newton-meters per amp
    //        motRes_milliohms: The average resistance of the motor at its terminals in milliohms
    //        staticMillivolts: The amount of voltage in the motor power line, in millivolts
    //        joint_id:  The ID of the joint in the system, starting from 1 and going up. What joint ID correlates to what role in reality
    //                   is defendant on GravInertSysStatus; see it for more details.
    GravityCompensator(GravityInertiaSystemStatus* sysStatus, TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, int staticMillivolts, uint8_t joint_Id);


    //public constructor, when the motor's voltage line varies in voltage level so needs a sensor to detect the voltage level.
    //Inputs:
    //        sysStatus: A pointer to the gravity inertia system status that will support this class. GravInertSysStatus will handle
    //                   crunching most of the hard math for gravity compensation while this class acts mostly as an interface between it
    //                   and the main driving algorithm. Therefore, GravInertSysStatus needs to have its update() function called periodically
    //                   so that it's periodically performing gravity calculations.
    //        motorType: What kind of motor type is attached to this joint, in the TorqueConverterMotorTypes enum
    //        Kt:        The motor's torque-to-amperage ratio, in newton-meters per amp
    //        motRes_milliohms: The average resistance of the motor at its terminals in milliohms
    //        fdev: A pointer to a feedback device that reads voltage values and will be used to determine how much voltage is on the motor's power line.
    //        joint_id:  The ID of the joint in the system, starting from 1 and going up. What joint ID correlates to what role in reality
    //                   is defendant on GravInertSysStatus; see it for more details.
    GravityCompensator(GravityInertiaSystemStatus* sysStatus, TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, FeedbackDevice* fdev, uint8_t joint_Id);

    //sets a scalar to apply to the output of the class, so you can tune it. Default is 1.
    void setScalar(float scale);

    void useErrorCompensation(FeedbackDevice *torqueDev);
    void setLoopsTillErrorCompensation(uint32_t loopsTillErrorCompensation);
    void setAcceptableErrorRange(uint32_t errorRange_milliNewtonMeters);
    void setErrorReadingDeadband(uint32_t deadband);
};





#endif /* ROVEJOINTCONTROL_GRAVITYCOMPENSATOR_H_ */
