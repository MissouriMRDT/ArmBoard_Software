/*
 * GravityCompensator.h
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#ifndef ROVEJOINTCONTROL_GRAVITYCOMPENSATOR_H_
#define ROVEJOINTCONTROL_GRAVITYCOMPENSATOR_H_

#include "abstractFramework.h"
#include "RoveJointUtilities.h"
#include "GravityInertiaSystemStatus.h"
#include "TtoPPOpenLConverter.h"
#include <stdint.h>

class GravityCompensator: public SupportingAlgorithm
{
  private:

    long addToOutput(const long inputValue, const long calculatedOutput);
    uint8_t jointId;
    GravityInertiaSystemStatus* systemStatus;
    DrivingAlgorithm* torqueConverter;

  public:

    //public constructor.
    //Inputs:
    //        sysStatus: A pointer to the gravity inertia system status that will support this class. GravInertSysStatus will handle
    //                   crunching most of the hard math for gravity compensation while this class acts mostly as an interface between it
    //                   and the main driving algorithm. Therefore, GravInertSysStatus needs to have its update() function called periodically
    //                   so that it's periodically performing gravity calculations.
    //        torquePowerPercentConverter: A pointer to a TtoPpOpenLConverter class. This class will ask it to convert the gravity's torque to
    //                                     power percent for it.
    //        joint_id:  The ID of the joint in the system, starting from 1 and going up. What joint ID correlates to what role in reality
    //                   is defendant on GravInertSysStatus; see it for more details.
    GravityCompensator(GravityInertiaSystemStatus* sysStatus, TtoPPOpenLConverter* torquePowerPercentConverter, uint8_t joint_Id);

};





#endif /* ROVEJOINTCONTROL_GRAVITYCOMPENSATOR_H_ */
