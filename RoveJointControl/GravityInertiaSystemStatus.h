/*
 * SystemStatus.h
 *
 * Calculates the Gravity torque and inertia of the arm with values from all joints.
 */

#ifndef ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_
#define ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_

#include "AbstractFramework.h"

class GravityInertiaSystemStatus
{
private:
    // Constants.
    const double J1_CENTER_OF_GRAVITY;
    const double J1_WEIGHT;
    const double J1_LENGTH;
    const double J2_CENTER_OF_GRAVITY;
    const double J2_WEIGHT;
    const double J2_LENGTH;
    const double J3_CENTER_OF_GRAVITY;
    const double J3_WEIGHT;
    const double J3_LENGTH;
    const double J4_CENTER_OF_GRAVITY;
    const double J4_WEIGHT;
    const double J4_LENGTH;
    const double J5_CENTER_OF_GRAVITY;
    const double J5_WEIGHT;
    const double J5_LENGTH;
    const double J6_CENTER_OF_GRAVITY;
    const double J6_WEIGHT;
    const double J6_LENGTH;

    // Member variables.
    double j1Gravity;
    double j1Inertia;
    double j2Gravity;
    double j2Inertia;
    double j3Gravity;
    double j3Inertia;
    double j4Gravity;
    double j4Inertia;
    double j5Gravity;
    double j5Inertia;
    double j6Gravity;
    double j6Inertia;

protected:

public:
    // Description:     Constructor
    // Arguments:       j1CenterOfGravity - Center of gravity for joint 1.
    //                  j1Weight          - Weight for joint 1.
    //                  j1Length          - Length for joint 1.
    //                  j2CenterOfGravity - Center of gravity for joint 2.
    //                  j2Weight          - Weight for joint 2.
    //                  j2Length          - Length for joint 2.
    //                  j3CenterOfGravity - Center of gravity for joint 3.
    //                  j3Weight          - Weight for joint 3.
    //                  j3Length          - Length for joint 3.
    //                  j4CenterOfGravity - Center of gravity for joint 4.
    //                  j4Weight          - Weight for joint 4.
    //                  j4Length          - Length for joint 4.
    //                  j5CenterOfGravity - Center of gravity for joint 5.
    //                  j5Weight          - Weight for joint 5.
    //                  j5Length          - Length for joint 5.
    //                  j6CenterOfGravity - Center of gravity for joint 6.
    //                  j6Weight          - Weight for joint 6.
    //                  j6Length          - Length for joint 6.
    // Derived From:    Nothing
    GravityInertiaSystemStatus(const double j1CenterOfGravity, const double j1Weight, const double j1Length,
                 const double j2CenterOfGravity, const double j2Weight, const double j2Length,
                 const double j3CenterOfGravity, const double j3Weight, const double j3Length,
                 const double j4CenterOfGravity, const double j4Weight, const double j4Length,
                 const double j5CenterOfGravity, const double j5Weight, const double j5Length,
                 const double j6CenterOfGravity, const double j6Weight, const double j6Length);
    ~GravityInertiaSystemStatus();

    // Description: Calculates the gravity and inertia values of all joints.
    //              Uses the joint constants in the calculations, so no parameters are necessary.
    // Arguments:   None
    // Returns:     Nothing
    void update();

    // Description: Returns the calculated gravity of the desired joint.
    // Arguments:   id - the ID of the joint requesting info.
    // Returns:     the torque gravity of the given joint.
    double getGravity(uint32_t id);

    // Description: Returns the calculated inertia of the desired joint.
    // Arguments:   id - the ID of the joint requesting info.
    // Returns:     the inertia of the given joint.
    double getInertia(uint32_t id);
};

#endif /* ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_ */
