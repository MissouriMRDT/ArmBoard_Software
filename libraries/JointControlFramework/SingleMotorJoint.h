#ifndef SINGLEMOTORJOINT_H_
#define SINGLEMOTORJOINT_H_

#include "JointFrameworkUtilities.h"
#include "AbstractFramework.h"

class SingleMotorJoint : public JointInterface
{
	public:

		//constructor for single motor joints without feedback.
		//Constructor automatically chooses an open loop algorithm that inputs the specified inputType and outputs the values the output device accepts
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//cont: The output device controlling the motor on this joint
		SingleMotorJoint(ValueType inputType, OutputDevice * cont);

		//constructor for single motor joints with feedback device
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//alg: The IOAlgorithm to be used by this joint
		//cont: The output device controlling the motor on this joint
		//feed: The feedback device used with this joint
		SingleMotorJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont, FeedbackDevice* feed);

		~SingleMotorJoint();

		//runs algorithm for movement for a singlular motor
		//calls the algorithm to manipulate the input and sends it to the motor device.
		//input: a long that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar
		//ranges defined in the framework's .h file
		//returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
		JointControlStatus runOutputControl(const long movement);
};

#endif