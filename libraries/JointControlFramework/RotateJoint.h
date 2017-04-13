#ifndef ROTATEJOINT_H_
#define ROTATEJOINT_H_

#include "JointFrameworkUtilities.h"
#include "FirmwareIncludes.h"
#include "AbstractFramework.h"

//represents a joint where two motor devices move in opposite directions to control the joint.
//Typically this causes a rotation motion, IE rotate joint
class RotateJoint : public JointInterface
{
	protected:

		OutputDevice* controller2;

	public:

		//constructor for with feedback. Feedback based algorithms are complex enough to usualyl require user initiliazation, so it's directly passed
		//in by the user as well
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//alg: the IOAlgorithm representing the closed loop algorithm being used on this joint
		//cont1: The first output device controlling the first motor on this joint
		//cont2: The second output device controlling the second motor on this joint
		//feed: The feedback device used with this joint
		RotateJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed);

		//constructor without feedback. IOAlgorithm for this joint automatically selected based on the other passed arguments
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//cont1: The first output device controlling the first motor on this joint
		//cont2: The second output device controlling the second motor on this joint
		RotateJoint(ValueType inputType, OutputDevice* cont1, OutputDevice* cont2);

		//deletes pointers used in the joint
		~RotateJoint();

		//moves two motors together so that they rotate the joint. Motors will spin the opposite direction
		//input: a long that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar
		//ranges defined in the framework's .h file
    //returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
		JointControlStatus runOutputControl(const long movement);
};

#endif