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
#include <stdint.h>

class GravityCompensator: public SupportingAlgorithm
{
  private:

    long addToOutput(const long inputValue, const long calculatedOutput);
    uint8_t jointId;
    GravityInertiaSystemStatus* systemStatus;
    DrivingAlgorithm* torqueConverter;

  public:

    GravityCompensator(GravityInertiaSystemStatus* sysStatus, DrivingAlgorithm* torquePowerPercentConverter, uint8_t joint_Id);

};





#endif /* ROVEJOINTCONTROL_GRAVITYCOMPENSATOR_H_ */
