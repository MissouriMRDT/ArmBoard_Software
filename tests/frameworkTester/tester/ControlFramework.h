#include "Energia.h"
#include "RoveDynamixel.h"
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
     Inherit from the proper abstract class. If you are implementing an algorithm, and that algorithm doens't use feedback, then
	 go down to Joint Interface's selector method and put in the logic to select your new algorithm accordingly.
 *   
*/  

//All the types of values that can be passed to and be returned from the clases in the control framework 
enum InputType{spd, pos};

//Constants representing the ranges for the different input types.
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
		//and friend isn't inherited so we can't just make the abstract class our friend.
		//returns: a value representing feedback, the range of the value depends on what input type this device returns.
		//For example, if this device returns speed feedback, the values shall be in the range between SPEED_MIN and SPEED_MAX
		virtual int getFeedback();

		InputType fType;
};
 
//Class containing the devices which move the arm and how they are controlled, such as motor controllers, and the hardware specifics of the devices
//such as what GPIO pins are used by the microcontroller to control them. Part of the joint interface framework
class OutputDevice
{
	//Joint interface needs access to its functions
	friend class SingleMotorJoint;
	friend class TiltJoint;
	friend class RotateJoint;
	friend class JointInterface;
  
	protected:

		//calls the device to move the motor, based on the passed value of movement.
		//Input: Can be values based on any of the input types as defined in the enum above, and the ranges for these values
		//should stay within the max and min constants for each type
		virtual void move(const int movement);

		//blank constructor for the base class
		OutputDevice() {};

		//expected input that the output device wants.
		InputType outType;

		//used for if the specific controller is mounted backwards on the motor joint.
		//True means invert the signal (backwards) False means just send the signal
		bool invert;
};

//SDC2130 motor controller. Capable of moving based on torque, speed,
//or position. Unfortunately it's kind of shitty, best we have right now
//is budging it slowly with input speed commands that output just making 
//it move position slightly. Only implemented modes are controlling it
//via pwm and moving by taking in a speed position
class Sdc2130: public OutputDevice
{
	private:
		int PWM_PIN, TX_PIN, RX_PIN;
		int pwmVal;
		
		const int PWM_MIN = 0, PWM_MAX = 255;
		const int POS_INC = 2;
		
		enum ControlTypes {Pwm, c_Serial};
		ControlTypes controlType;
	protected:
  
		//general move command. Pass in either speed or position values
		void move(const int movement);
		
		//move command if inputting speed, input movement is in the range of SPEED_MIN to SPEED_MAX
		void moveSpeed(const int movement);
		
		//move command if inputting position, input movement is in the range of POS_MIN to POS_MAX
		//void movePos(const int movement); not implemented
	public:
		//constructor for controlling it via pwm
		//inputs:
		//pwmPin: GPIO pin that's connected to the pwm input of the SDC2130 device. GPIO pin number id's are defined by energia's pinmapping
		//inType: instance of the InputType enum that defines what kind of input the device should take, currently pos or spd. The motor controller can move based on either, so pick one
		//upside down: whether or not the motor is mounted in reverse so the input values would also need to be inverted
		Sdc2130(const int pwmPin, InputType inType, bool upsideDown);
		
		//constructor for controlling it via serial
		//Sdc2130(const int txPin, const int rxPin, InputType inType, bool upsideDown); not implemented
};

//Standard dynamixel capable of wheel, joint, and multi-turn modes of operation.
//RoveDynamixel causes some issues to interface with but not worth rewriting it to work with classes better
//Note: Requires use of RoveDynamixel which is hard to read and understand. Edit at own risk.
class DynamixelController : public OutputDevice{
  private:
    //Only Important pin for dynamixel is the Tx/Rx pin other two are just power and not important logically
    int Tx_PIN;
	int Rx_PIN;
	uint32_t baudRate;
	
	//values which the dynamixel expects the speed to be between. CW = clockwise/positive/forward  CCW = counter-clockwise/negative/backwards
	const int DYNA_SPEED_CW_MAX = 1023;
	const int DYNA_SPEED_CW_MIN = 0;
	const int DYNA_SPEED_CCW_MAX = 2047;
	const int DYNA_SPEED_CCW_MIN = 1024;
	
	//need to have in order to interface with RoveDynamixel since it uses structs and pointers as opposed to classes
	Dynamixel dynamixel;
  
  protected:
	//movement command based on a speed input and wheel mode. Input should be in the range between SPEED_MIN and SPEED_MAX
	//this will be the defaut assumption, other modes with other methods of movement will be 
	//made into other functions not simply move.
    void move(const int movement);
	
  public:
  
	//constructor for a dynamixel which takes in a move type and other things needed for dynamixels
	//Sets up uart on the board and baud rate at the same time
	/*Inputs: 
	  TX -> tx pin numerical id, as defined by energia's pinmapping
	  RX -> tx pin numerical id, as defined by energia's pinmapping
	  upsideDown -> Whether or not the dyna is mounted in reverse and thus the inputs need to be reversed as well
	  type -> Instance of the DynamixelType enum that defines the dynamixel brand such as AX, MX, etc
	  id -> id of the dynamixel. Note that if you aren't sure, you'll need to use a different program to set the id yourself
	  uartIndex -> which hardware uart to use when talking to it, 0-7
	  baud -> baud rate of the serial communication
	  mode -> instance of the DynamixelMode enum that defines the dynamixel's mode such as Wheel, Joint, etc
	*/
	DynamixelController(const int Tx, const int Rx, bool upsideDown, DynamixelType type, uint8_t id, uint8_t uartIndex, uint32_t baud, DynamixelMode mode);	
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
		//input: int representing the input values that need to be converted into output values. 
		//The specific value constraints depend on the input and output types the algorithm implements; for example if an algorithm
		//is supposed to take in speed and output position, then its input is constrained by SPEED_MIN and SPEED_MAX, and its output is constrained by POS_MIN and POS_MAX
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
  
//Interface for controlling the joint overall from the main program's perspective.
//It handles all duties of controlling the joint; the main program just has to pass it a value to respond to.
//This class is supposed to be thought of as abstract; base classes determine what type of joint is used.
class JointInterface
{
	protected:
	  
		//expected input type from base station
		InputType inType;

		//algorithm is either passed in, or created dymanically based on what is passed to the control framework interface. 
		//called in runOutputControl
		IOAlgorithm* manip;

		//Passed to the framework interface if feedback is used. Used when looping in certain algorithms
		FeedbackDevice* feedback;

		//pointer to the output device instance which is passed in when creating the framework interface
		//called in runOutputControl
		OutputDevice* controller1;

		//if the constructor wasn't passed an ioalgorithm to use, then this function selects one.
		//Basic algorithms only; if it's one that uses feedback then it needs to be passed in by the user, 
		//as feedback based algorithms typically are complex enough to require user-dictated initialization
		void algorithmSelector()
		{
		  //selects algorithm]
		  if((controller1->outType == spd) && inType == spd)
		  {
			manip = new SpdToSpdNoFeedAlgorithm();
		  }
		  return;
		}
    
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
	public:
	
		//Constructor for the single motor joint. it is passed an inputType, feedback device pointer and output device pointer
		//With this info selects an algorithm. 
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//cont: The output device controlling the motor on this joint
		SingleMotorJoint(InputType inputType, OutputDevice * cont);

		//constructor for with feedback. Feedback based algorithms are complex enough to usualyl require user initiliazation, so it's directly passed 
		//in by the user as well
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//alg: the closed loop IOalgorithm used by this joint
		//cont: The output device controlling the motor on this joint
		//feed: The feedback device used with this joint  
		SingleMotorJoint(InputType inputType, IOAlgorithm *alg, OutputDevice* cont, FeedbackDevice* feed);

		//destructor since there are pointers being used
		~SingleMotorJoint();       

		//runs algorithm for movement for a singlular motor
		//input: an int that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar 
		//ranges defined in the framework's .h file
		void runOutputControl(const int movement);    
};
  
//Two motor device joint, where they move in the same direction to control the joint
class TiltJoint : public JointInterface
{
	protected:
  
		OutputDevice* controller2;

	public:
	
		//constructor for with feedback. Feedback based algorithms are complex enough to usualyl require user initiliazation, so it's directly passed 
		//in by the user as well
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//alg: the closed loop IOAlgorithm used by this joint
		//cont1: The first output device controlling the first motor on this joint
		//cont2: The second output device controlling the second motor on this joint
		//feed: The feedback device used with this joint
		TiltJoint(InputType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed);

		//constructor without feedback. IOAlgorithm automatically selected based on the other arguments
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//cont1: The first output device controlling the first motor on this joint
		//cont2: The second output device controlling the second motor on this joint
		TiltJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2);	  

		//need to use a destructor to remove the pointers used
		~TiltJoint();

		//runs algorithm for moving two motors together so that it tilts the joint	
		//input: an int that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar 
		//ranges defined in the framework's .h file  
		void runOutputControl(const int movement);
};  
  
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
		RotateJoint(InputType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed);

		//constructor without feedback. IOAlgorithm for this joint automatically selected based on the other passed arguments
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//cont1: The first output device controlling the first motor on this joint
		//cont2: The second output device controlling the second motor on this joint
		RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2);

		//deletes pointers used in the joint
		~RotateJoint();

		//moves two motors together so that they rotate the joint. Motors will spin the opposite direction 
		//input: an int that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar 
		//ranges defined in the framework's .h file
		void runOutputControl(const int movement);
};
  
