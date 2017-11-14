/*
 * SystemStatus.h
 *
 * Calculates the Gravity torque and inertia of the arm with values from all joints.
 */

#ifndef ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_
#define ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_

#include "AbstractFramework.h"
#include "Roveboard.h"
#include "RoveJointUtilities.h"

typedef enum ArmModel {gryphonArm} ArmModel;

class GravityInertiaSystemStatus
{
private:
    // Constants.
    const double GRIPPER_WEIGHT;
    const double GRIPPER_LENGTH;
    const double GRIPPER_CENTER_OF_GRAVITY;
    const double FOREARM_WEIGHT;
    const double FOREARM_LENGTH;
    const double FOREARM_CENTER_OF_GRAVITY;
    const double BICEP_WEIGHT;
	const double BICEP_LENGTH;
    const double BICEP_CENTER_OF_GRAVITY;

    FeedbackDevice* const JOINT1ANGLE;
    FeedbackDevice* const JOINT2ANGLE;
    FeedbackDevice* const JOINT3ANGLE;
    FeedbackDevice* const JOINT4ANGLE;
    FeedbackDevice* const JOINT5ANGLE;
    FeedbackDevice* const JOINT6ANGLE;




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

    float puToRad(uint32_t p_units);
    const ArmModel Model;

public:
    // Description:     Constructor
    // Arguments:       gripperCenterOfGravity - Center of gravity for gripper. Inches
    //                  gripperWeight          - Weight for gripper. Pounds
    //                  gripperLength          - Length for gripper. Inches
    //                  forearmCenterOfGravity - Center of gravity for forearm.
    //                  forearmWeight          - Weight for forearm.
    //                  forearmLength          - Length for forearm.
    //                  bicepCenterOfGravity   - Center of gravity for bicep.
    //                  bicepWeight            - Weight for bicep.
    //                  bicepLength            - Length for bicep.
    //
    // Derived From:    Nothing
    GravityInertiaSystemStatus(ArmModel model, const double GRIPPER_WEIGHT, const double GRIPPER_LENGTH, const double GRIPPER_CENTER_OF_GRAVITY,
    						   const double FOREARM_WEIGHT, const double FOREARM_LENGTH, const double FOREARM_CENTER_OF_GRAVITY,
							   const double BICEP_WEIGHT, const double BICEP_LENGTH, const double BICEP_CENTER_OF_GRAVITY,
							   FeedbackDevice* joint1, FeedbackDevice* joint2, FeedbackDevice* joint3,
							   FeedbackDevice* joint4, FeedbackDevice* joint5, FeedbackDevice* joint6);
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





