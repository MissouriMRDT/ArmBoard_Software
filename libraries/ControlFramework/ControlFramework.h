#include "Energia.h"
/*  
 *   
 *   This is the library for the Joint Interface Framework. 
	 Framework description here: https://github.com/MST-MRDT/ArmBoardSoftware/blob/development/libraries/ControlFramework/Framework%20Overview.docx
 *   Used for controlling output devices and easily manipulating joints. The user will only ever interact with the 
     Joint Interface, which keeps track of an output device which moves that joint, and an algorithm for controlling its represented 
	 joint, and algorithm selection is automatic. 
	 
	 implemenation notes:
 *   The JointInterface class is friend classes to the OutputDevice class, IOAlgorithm class, and the FeedbackDevice class. 
 *   
 *   A outputDevice and feedback device are passed to the control framework interface as pointers and along with an input type determine the algorithm used.
     The user has no control on the selected algorithm other than through the output and feedback devices and the input from base station. 
	 Once the joint interface has been set up nothing can be changed (mainly the expected input must remain the same) so the output device will function as
	 expected
 *   
 *   In order to implement modules into the program:
     Inherit from the proper abstract class, and go down to Joint Interface's selector methods and put in the logic to select your new class accordingly.
 *   
*/  

//All the types of inputType that can be expected and selected
enum InputType{spd, pos};

//used later for maping in move commands. constants so they can be easily changed if something changes on input or expected output.
//The common inputs and outputs between classes will fall between these limits, class should not expect to take or return values outside of these
const int SPEED_MIN = -1000, SPEED_MAX = 1000; 
const int POS_MIN = 0, POS_MAX = 10000;

//feedback devices used to help determine where the arm is and what steps need to be taken
//Used by the IOAlgorithm class to perform looping. Part of the JointInterface framework
class FeedbackDevice
{
	friend class SingleMotorJoint;
	friend class TiltJoint;
	friend class RotateJoint;

	public:
	
		//blank constructor for the base class
		FeedbackDevice() {} ;

		//returns feedback. Public because all the IOAlgorithm classes need to be able to call it,
		//and friend isn't inherited so we can't just make the abstract class our friend
		virtual int getFeedback();

		//expected type of output the feedbackDevice is expected to return
		enum FeedType {};
		FeedType fType;
};
 
//Class containing the devices which move the arm and how they are controlled. Part of the joint interface framework
class OutputDevice
{
	//Joint interface needs access to its functions
	friend class SingleMotorJoint;
	friend class TiltJoint;
	friend class RotateJoint;
	
	protected:
	
		//all the controllers have different ways of movement controlled by algorithms. The algorithm will spit out the speed 
		//or position that device will use, which will always be an int.
		//The pins are taken care of directly by the controller instance. The polarity by the sign of the integer for the move command.
		virtual void move(const int movement);

		//blank constructor for the base class
		OutputDevice() {};

		//expected input that the output device wants.
		//dont confuse with inType from base station.
		InputType outType;

		//used for if the specific controller is mounted backwards on a two motor joint. 
		//Only one motor should be inverted per two motor joint
		//True means invert the signal (backwards) False means just send the signal
		bool invert;
};

//Discrete H Bridge controlled directly by the microcontroller, which has only two inputs to control forward and backwards. 
//Has two pins (one forward, one backwards). Outputs a pwm signal on one pin or another. 
class DirectDiscreteHBridge : public OutputDevice
{
	private:

		//FPWM_PIN the forward PWM and RPWM is the reverse PWM
		int FPWM_PIN, RPWM_PIN;

		//Value ranges for conversting input to expected output when sending 
		const int PWM_MIN = 0, PWM_MAX = 255;

		protected:
		//function which directly moves the device. 
		void move(const int movement);
		public:
		//constructor, user must give pin assignments with the first pin being the forward pin and the second being the reverse pin.
		//Must specify the physical orientation of the device as well, if it is mounted in reverse or not
		DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown);  

};
  
//Base class for all algorithms. Algorithms take the input from base station and do whatever computation is 
//needed to interpret the command such as controls. 
//Part of the joint interface framework
class IOAlgorithm
{

	friend class SingleMotorJoint;
	friend class TiltJoint;
	friend class RotateJoint;

	protected:
		//Constructor for the base class is empty.
		IOAlgorithm() {};

		//run whatever algorithm this implements, returns value that can be directly passed to an output device
		virtual int runAlgorithm(const int in);
};

//Algorithm used when speed is recieved from base station and speed is expected to be sent to the device without any feedback. 
//Open loop speed control.  
class SpdToSpdNoFeedAlgorithm : public IOAlgorithm
{

    protected:
      //since nothing needs to change the input (what is recieved from base station) is directly returned
      //implemented to preserve modularity and uniformity in code.
      int runAlgorithm(const int input);
};
  
//Interface for controlling the joint overall from the main program's perspective. Internally, 
//it is passed a pointer to an output device and a feedback device as well as contains the expected input type, such as speed or position. 
//From this data selects the proper IO algorithm. 
//From there, it handles all duties of controlling the joint; the main program just has to pass it a value to respond to.
//This class is supposed to be thought of as abstract; base classes determine what type of joint is used.
class JointInterface
{
	protected:
	
		//Algorithm is the same for all types of joints so it is not virtual.
		//Called in the constructor for each of the sub classes to determine which algorithm needs to be used.
		//Different selectors are used for open loop and feedback methods
		//Takes in an output device during the construction of the Joint Interface and selects an algorithm for it
		void selector(FeedbackDevice* feedback, InputType outputDeviceType, IOAlgorithm *alg)
		{
			//selects algorithm 
			return;  
		}
    
		void selector(InputType outputDeviceType, IOAlgorithm* alg)
		{
			//selects algorithm]
			if((outputDeviceType == spd) && inType == spd)
			{
				alg = new SpdToSpdNoFeedAlgorithm();
			}
			return;
		}
	  
		//expected input type from base station
		InputType inType;

		//created dymanically based on what is passed to the control framework interface. 
		//called in runOutputControl
		IOAlgorithm* manip;

		//Passed to the framework interface if feedback is used. Used when looping in certain algorithms
		FeedbackDevice* feedback;
	  
    public:
	
		//Runs the output control for this joint, IE making it move or checking 
		//feedback to see if it needs to move, whatever the algorithm for 
		//this joint is deemed to be, it runs it.
		//Must pass an integer for this implementation. Will break otherwise.
		virtual void runOutputControl(const int movement);
};
	  
//this derivation of jointInterface represents joints where a singular motor device moves the joint.
class SingleMotorJoint : public JointInterface
{
	protected:
	
		//pointer to the output device instance which is passed in when creating the framework interface
		//called in runOutputControl
		OutputDevice* controller;

	public:
	
		//Constructor for the single motor joint. it is passed an inputType, feedback device pointer and output device pointer
		//With this info selects an algorithm. 
		SingleMotorJoint(InputType inputType, OutputDevice * cont);

		//constructor for with feedback      
		SingleMotorJoint(InputType inputType, OutputDevice* cont, FeedbackDevice* feed);

		//destructor since there are pointers being used
		~SingleMotorJoint();       

		//runs algorithm for movement for a singlular motor
		void runOutputControl(const int movement);    
};
  
//Two motor device joint, where they move in the same direction to control the joint
class TiltJoint : public JointInterface
{
	protected:
	
		OutputDevice* controller1;
		OutputDevice* controller2;

	public:
	
		//Constructor for a tilt joint. it is passed an expected input type, two output devices, and possibly a feedback device
		//algorithm selection remains the same as expeted input to both the devices should be the same.
		TiltJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed);

		//constructor without feedback
		TiltJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2);	  

		//need to use a destructor to remove the pointers used
		~TiltJoint();

		//runs algorithm for moving two motors together so that it tilts the joint	  
		void runOutputControl(const int movement);
};  
  
//represents a joint where two motor devices move in opposite directions to control the joint. 
//Typically this causes a rotation motion, IE rotate joint
class RotateJoint : public JointInterface
{
	protected:
	
		OutputDevice* controller1;
		OutputDevice* controller2;

	public:
	
		// constructor for a rotating joint. pass it a input type, two output devices of the same type 
		// (dont put one device that wants a spd and another which wants a pos) and a feedback device. 
		// These are all pointers other than the input type. Algorithm selection is same for all joints.
		RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed);

		//constructor without feedback
		RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2);

		//deletes pointers used in the joint
		~RotateJoint();

		//moves two motors together so that they rotate the joint. Motors will spin the opposite direction 
		void runOutputControl(const int movement);
};
  
