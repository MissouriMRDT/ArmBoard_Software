#ifndef SINGLEMOTORJOINT_H_
#define SINGLEMOTORJOINT_H_

#include "JointFrameworkUtilities.h"
#include "AbstractFramework.h"

//this derivation of jointInterface represents joints where a singular motor device moves the joint.
class SingleMotorJoint : public JointInterface
{
	public:

		//Constructor for the single motor joint. it is passed an inputType, feedback device pointer and output device pointer
		//With this info selects an algorithm.
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//cont: The output device controlling the motor on this joint
		SingleMotorJoint(ValueType inputType, OutputDevice * cont);

		//constructor for with feedback. Feedback based algorithms are complex enough to usualyl require user initiliazation, so it's directly passed
		//in by the user as well
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//alg: the closed loop IOalgorithm used by this joint
		//cont: The output device controlling the motor on this joint
		//feed: The feedback device used with this joint
		SingleMotorJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont, FeedbackDevice* feed);

		~SingleMotorJoint();

		//runs algorithm for movement for a singlular motor
		//input: a long that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar
		//ranges defined in the framework's .h file
    //returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
		JointControlStatus runOutputControl(const long movement);
};

#endif