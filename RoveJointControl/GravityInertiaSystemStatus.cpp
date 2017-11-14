#include <GravityInertiaSystemStatus.h>
#include "RoveBoard.h"
#include "RoveJointUtilities.h"
#include <stdint.h>

const float FootPoundToNewtonMeter = 1.36;

GravityInertiaSystemStatus::GravityInertiaSystemStatus(ArmModel model, const double gripperWeight, const double gripperLength, const double gripperCenterOfGravity,
		   const double forearmWeight, const double forearmLength, const double forearmCenterOfGravity,
		   const double bicepWeight, const double bicepLength, const double bicepCenterOfGravity,
		   FeedbackDevice* joint1, FeedbackDevice* joint2, FeedbackDevice* joint3,
		   FeedbackDevice* joint4, FeedbackDevice* joint5, FeedbackDevice* joint6) :
		   GRIPPER_WEIGHT(gripperWeight), GRIPPER_LENGTH(gripperLength/12), GRIPPER_CENTER_OF_GRAVITY(gripperCenterOfGravity/12),
		   FOREARM_WEIGHT(forearmWeight), FOREARM_LENGTH(forearmLength/12), FOREARM_CENTER_OF_GRAVITY(forearmCenterOfGravity/12),
		   BICEP_WEIGHT(bicepWeight), BICEP_LENGTH(bicepLength/12), BICEP_CENTER_OF_GRAVITY(bicepCenterOfGravity/12),
		   JOINT1ANGLE(joint1), JOINT2ANGLE(joint2), JOINT3ANGLE(joint3), JOINT4ANGLE(joint4), JOINT5ANGLE(joint5),
		   JOINT6ANGLE(joint6), Model(model)
{

}

// Empty because there are no pointers.
GravityInertiaSystemStatus::~GravityInertiaSystemStatus()
{
}

void GravityInertiaSystemStatus::update()
{
	uint32_t joint1Feed = JOINT1ANGLE->getFeedback();
	uint32_t joint2Feed = JOINT2ANGLE->getFeedback();
	uint32_t joint3Feed = JOINT3ANGLE->getFeedback();
	uint32_t joint4Feed = JOINT4ANGLE->getFeedback();
	uint32_t joint5Feed = JOINT5ANGLE->getFeedback();
	uint32_t joint6Feed = JOINT6ANGLE->getFeedback();

	if(Model == gryphonArm)
	{
		j2Gravity = 0;
		j3Gravity = (FOREARM_WEIGHT*FOREARM_CENTER_OF_GRAVITY+GRIPPER_WEIGHT*FOREARM_LENGTH)*cosLW(puToRad(joint1Feed)+puToRad(joint3Feed)) + GRIPPER_WEIGHT*GRIPPER_CENTER_OF_GRAVITY*cosLW(puToRad(joint1Feed)+puToRad(joint3Feed))*cosLW(puToRad(joint4Feed));
		j4Gravity = GRIPPER_WEIGHT*GRIPPER_CENTER_OF_GRAVITY*sinLW(puToRad(joint5Feed))*cos(puToRad(joint1Feed)+puToRad(joint3Feed))*-sinLW(puToRad(joint4Feed));
		j5Gravity = GRIPPER_WEIGHT*GRIPPER_CENTER_OF_GRAVITY*cosLW(puToRad(joint1Feed)+puToRad(joint3Feed)+puToRad(joint5Feed));
		j6Gravity = 0;
		//depends on torque calculated for joint 3
		j1Gravity = ((FOREARM_WEIGHT+GRIPPER_WEIGHT)*BICEP_LENGTH + BICEP_WEIGHT*BICEP_CENTER_OF_GRAVITY)*cosLW(puToRad(joint1Feed))+j3Gravity;

		j3Gravity *= FootPoundToNewtonMeter * 1000;
		j4Gravity *= FootPoundToNewtonMeter * 1000;
		j5Gravity *= FootPoundToNewtonMeter * 1000;
		j1Gravity *= FootPoundToNewtonMeter * 1000;
	}
}

double GravityInertiaSystemStatus::getGravity(uint32_t id)
{
    // Return the appropriate value based on the joint.
    // id 1 corresponds to joint 1, id 2 corresponds to joint 2, etc.
    switch (id)
    {
    case 1:
        return j1Gravity;
    case 2:
        return j2Gravity;
    case 3:
        return j3Gravity;
    case 4:
        return j4Gravity;
    case 5:
        return j5Gravity;
    case 6:
        return j6Gravity;
    default:
        return 0.0;
    }
}

double GravityInertiaSystemStatus::getInertia(uint32_t id)
{
    // Return the appropriate value based on the joint.
    // id 1 corresponds to joint 1, id 2 corresponds to joint 2, etc.
    switch (id)
    {
    case 1:
        return j1Inertia;
    case 2:
        return j2Inertia;
    case 3:
        return j3Inertia;
    case 4:
        return j4Inertia;
    case 5:
        return j5Inertia;
    case 6:
        return j6Inertia;
    default:
        return 0.0;
    }
}

float GravityInertiaSystemStatus::puToRad(uint32_t p_units)
{
  float degrees = static_cast<float>(p_units)*360.0/(POS_MAX-POS_MIN);

  return radians(degrees);
}
