#ifndef ABSTRACTFRAMEWORK_H_
#define ABSTRACTFRAMEWORK_H_

#include "RoveJointUtilities.h"

class JointInterface;
class IOConverter;
class OutputDevice;
class FeedbackDevice;

class DifferentialJoint;
class SingleMotorJoint;

class JointInterface
{
  protected:

    //expected input type from the user
    ValueType inType;

    //algorithm is either passed in, or created dymanically based on what is passed to the control framework interface.
    IOConverter* manip;

    //pointer to the output device instance which is passed in when creating the framework interface
    //called in runOutputControl
    OutputDevice* controller1;

    //tracks whether or not the parameters passed in the joint constructor were valid
    bool validConstruction;
    
    //tracks whether or not the user has passed in an algorithm to use; if not, commands are passed directly to output
    bool algorithmUsed;
    
    //tracks whether or not the joint has been enabled for operation
    bool enabled;

    //function that checks to see if the user put in a proper input value when calling the runOutputControl function
    //returns true if the input is in a valid range, false if it's not
    bool verifyInput(long inputToVerify);

    JointInterface()
    {
      enabled = true;
    };

  public:

    //Runs the output control for this joint, IE making it move or checking
    //to see if it needs to move, updating the output control loops, anything the joint's algorithm says.
    //Must pass an integer for this implementation.
    //returns: The status of attempting to update the controls on this joint. Such as if the output is running, if it's complete, errors, etc
    virtual JointControlStatus runOutputControl(const long movement) = 0;
    
    //replaces the current joint's algorithm component with a different one
    //input: the new type of input the joint is going to be given when runOutputControl is called
    //       the new algorithm module
    //returns: true if swap was successful, false if not and previous settings retained
    //warning: not thread safe
    bool switchModules(ValueType newInputType, IOConverter* newAlgorithm);

    //replaces the current joint's algorithm component and outputDevice components with different ones
    //input: the new type of input the joint is going to be given when runOutputControl is called
    //       the new algorithm module
    //       the new OutputDevice module
    //returns: true if swap was successful, false if not and previous settings retained
    //warning: not thread safe
    bool switchModules(ValueType newInputType, IOConverter* newAlgorithm, OutputDevice* newDevice);

    //replaces the current joint's outputDevice components with a different one
    //input: the new type of input the joint is going to be given when runOutputControl is called
    //       the new OutputDevice module
    //returns: true if swap was successful, false if not and previous settings retained
    //warning: not thread safe
    bool switchModules(ValueType newInputType, OutputDevice* newDevice);

    //tells joint stop using an IOConverter, IE values should be passes straight to the output.
    //input: the new type of input the joint is going to be given when runOutputControl is called
    //returns: true if swap was successful, false if not and previous settings retained
    //warning: not thread safe
    bool removeAlgorithm(ValueType newInputType);
    
    //tells joint to stop using an IOConverter, IE values should be passes straight to the output,
    //and swap the joint's outputDevice with a different one.
    //input: the new type of input the joint is going to be given when runOutputControl is called
    //       the new OutputDevice module
    //returns: true if swap was successful, false if not and previous settings retained
    //warning: not thread safe
    bool removeAlgorithm(ValueType newInputType, OutputDevice* newDevice);

    //tells the joint to halt. Note that this won't keep the joint from moving if called again; 
    //use disable for that
    virtual void stop() = 0;
    
    //turns the joint off; it will stop moving until enabled
    virtual void disableJoint() = 0;
    
    //turns the joint on after being disabled
    virtual void enableJoint() = 0;
};

class FeedbackDevice
{
	friend class SingleMotorJoint;
	friend class DifferentialJoint;
  friend class JointInterface;

	public:

		//returns feedback. Public for everything to see, if desired
		//returns: a value representing feedback, the range of the value depends on what input type this device returns.
		//For example, if this device returns speed feedback, the values shall be in the range between SPEED_MIN and SPEED_MAX
		virtual long getFeedback() = 0;

		ValueType fType;
};

class OutputDevice
{
	//Joint interface needs access to its functions
	friend class SingleMotorJoint;
	friend class DifferentialJoint;
	friend class JointInterface;

  public:
    
    //gets the current moving value of the device. 
    //return value depends on what value type the device operates on; a speed value for speed devices, etc.
    virtual long getCurrentMove() = 0;

	protected:

		//calls the device to move the motor, based on the passed value of movement.
		//Input: Can be values based on any of the input types as defined in the enum above, and the ranges for these values
		//should stay within the max and min constants for each type
		virtual void move(const long movement) =  0;

		//expected input that the output device wants.
		ValueType inType;

		//used for if the specific controller is mounted backwards on the motor joint.
		//True means invert the signal (backwards) False means just send the signal
		bool invert;

    //tracks whether or not the device is enabled/powered on by the joint manager
    bool enabled;
    
    OutputDevice()
    {
      enabled = true;
    };
    
    //tells device to stop moving
    virtual void stop() = 0;
    
    //tells the device to power on or off.
    virtual void setPower(bool powerOn) = 0;
};

class IOConverter
{
	friend class SingleMotorJoint;
	friend class DifferentialJoint;
  friend class JointInterface;

	protected:

    //the types of values that the algorithm takes in and gives out
    ValueType inType;
    ValueType outType;

    IOConverter(){};

		//run whatever algorithm this implements, returns value that can be directly passed to an output device
		//input: int in, representing the input values that need to be converted into output values.
		//The specific value constraints depend on the input and output types the algorithm implements; for example if an algorithm
		//is supposed to take in speed and output position, then its input is constrained by SPEED_MIN and SPEED_MAX, and its output is constrained by POS_MIN and POS_MAX
    //bool * ret_outputFinished: parameter passed by pointer, returns true if the joint has finished its controlled movement and ready to exit the control loop,
    //false if it's still in the middle of getting to its desired finished state IE in the middle of moving to a desired end position or reaching a desired end speed
    //If ret_OutputFinished returns false but input returns 0, it's an indication that an error has occured
		virtual long runAlgorithm(const long input, bool * ret_OutputFinished) = 0;
};

class ClosedLoopAlg : public IOConverter
{
  protected:

    //pointer to the feedback device used by this algorithm (typically passed as a constructor argument)
    FeedbackDevice * feedbackDev;

    //This flag tracks whether or not the feedback device given to the algorithm is a proper fit for the algorithm
    bool validConstruction;

    virtual long runAlgorithm(const long input, bool * ret_OutputFinished) = 0;
};

class OpenLoopAlg: public IOConverter
{
  protected:
    virtual long runAlgorithm(const long input, bool * ret_OutputFinished) = 0;
};

#endif
