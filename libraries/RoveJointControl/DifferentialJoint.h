#ifndef TILTJOINT_H_
#define TILTJOINT_H_

#include "AbstractFramework.h"
#include "RoveJointUtilities.h"

enum DifferentialType
{
  //This describes moving the differential joint so that the shaft moves up and down
  DifferentialTilt,

  //This describes moving the differential joint so that the shaft rotates around
  DifferentialRotate
};

class DifferentialJoint : public JointInterface
{
	protected:

		OutputDevice* controller2;
    
    //the other joint with which this joint is coupled with
    DifferentialJoint* coupledJoint;

    DifferentialType thisJointType;

    //tracks whether or not the Differential joint is coupled with another Differential joint; physically they always are,
    //but if for some reason the user wants to only use one motion on the differential joint, then they can choose
    //to not couple the joint with another.
    bool coupled;
    
	public:

    //variables to store the power of either motor; useful if this is in coupled mode, as the movements of one joint motion
    //affect the other so virtual power variables are used to track what this specific moton joint is trying
    //to move at, to compare with the other joint this is coupled with
    int motorOneVirtualPower;
    int motorTwoVirtualPower;
    
		//Overview: creates the joint interface with an IOConverter to translate commands for the output devices.
    //          Note both output devices have to have the same input type.
    //
    //Inputs:   DifferentialType: Whether this differnetial joint has tilt motion or rotate motion
		//          inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//          alg: the Driving Algorithm used by this joint
		//          cont1: The first output device controlling the first motor on this joint
		//          cont2: The second output device controlling the second motor on this joint
		DifferentialJoint(DifferentialType jointType, ValueType inputType, DrivingAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2);

		//Overview: creates joint interface without an IO Converter, IE where output is passed straight to the output device
		//
		//Input:    DifferentialType: Whether this differnetial joint has tilt motion or rotate motion
		//          Note both output devices are assumed to have the same input type
		//          inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//          cont1: The first output device controlling the first motor on this joint
		//          cont2: The second output device controlling the second motor on this joint
		DifferentialJoint(DifferentialType jointType, ValueType inputType, OutputDevice* cont1, OutputDevice* cont2);

		~DifferentialJoint();

		//Overview: runs algorithm for moving two motors together so that it moves the joint, rotating it or tilting it.
		//input:    movement, a long that represents the desired movement. Value constraints and meaning depend on the inputType.
		//          For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX
    //returns:  The status of attempting to control this joint. Such as if the output is running, if it's complete, if there was an error, etc
		JointControlStatus runOutputControl(const long movement);

    bool switchDifferentialModules(ValueType newInputType, DrivingAlgorithm* newAlgorithm, OutputDevice* newDevice1, OutputDevice* newDevice2);
    
    //Overview: couples this differential joint with another; differential joints
    //          are intertwined so need to interact with each other to properly gauge output power
    //
    //Inputs:   The other differential joint to couple this one with. If this joint is a rotate joint then the other must be a tilt
    //          and vice versa; they can't be the same type.
    void pairDifferentialJoint(DifferentialJoint* otherJoint);
    
    //Overview: tells the joint to halt. Note that this won't keep the joint from moving if called again;
    //          use disable for that
    void stop();
    
    //Overview: turns the joint off; it will stop moving until enabled
    void disableJoint();
    
    //Overview: turns the joint on after being disabled
    void enableJoint();
};

#endif
