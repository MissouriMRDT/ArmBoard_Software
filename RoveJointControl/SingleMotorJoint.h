#ifndef SINGLEMOTORJOINT_H_
#define SINGLEMOTORJOINT_H_

#include "AbstractFramework.h"
#include "RoveJointUtilities.h"

class SingleMotorJoint : public JointInterface
{
	public:

    //Overview: creates joint interface without an IO Converter, IE where output is passed straight to the output device.
    //
    //Inputs:   inputType: What kind of movement this joint should be controlled by, such as speed or position input.
    //          cont: The output device controlling the first motor on this joint
		SingleMotorJoint(ValueType inputType, OutputDevice * cont);

    //Overview: creates the joint interface with an IOConverter to translate commands for the output devices.
		//
		//Inputs: inputType: What kind of movement this joint should be controlled by, such as speed or position input.
    //        alg: the closed loop IOAlgorithm used by this joint
    //        cont: The first output device controlling the first motor on this joint
		SingleMotorJoint(ValueType inputType, DrivingAlgorithm *alg, OutputDevice* cont);

		~SingleMotorJoint();

		//Overview: runs control algorithm for a singlular motor so that it moves.
		//
		//Inputs: input: a long that represents the desired movement. Value constraints and meaning depend on the inputType.
    //               For example, if this joint runs off of speed input then the values are constrained between
		//               SPEED_MIN and SPEED_MAX, and otherwise for the other types.
		//
    //returns: The status of attempting to control this joint. Such as if the output is running, if it's complete, if there was an error, etc
		JointControlStatus runOutputControl(const long movement);
    
    //tells the joint to halt. Note that this won't keep the joint from moving if called again; 
    //use disable for that
    void stop();
    
    //turns the joint off; it will stop moving until enabled
    void disableJoint();
    
    //turns the joint on after being disabled
    void enableJoint();
};

#endif
