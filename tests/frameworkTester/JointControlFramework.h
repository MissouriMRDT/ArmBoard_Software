/*  Programmer: Drue Satterfield, David Strickland
 *   
 *   This is the library for the Joint Interface Framework. 
 *   Framework description here: https://github.com/MST-MRDT/ArmBoardSoftware/blob/development/libraries/ControlFramework/Framework%20Overview.docx
 *   Used for controlling output devices and easily manipulating joints. The user will only ever interact with the 
 *   Joint Interface, which keeps track of an output device which moves that joint, and an algorithm for controlling that joint.
 *   
 *   necessary libraries:
 *   This program is meant to use energia libraries as well as the RoveDynamixel library
 *   
 *   implemenation notes:
 *   The user is supposed to use the library in this fashion: 
 *   1) construct the output device class/classes representing the output device/devices (a motor controller, an h bridge, etc) used to move the joint 
 *   2) if the joint is closed loop controlled, then the user must also construct a feedback device class representing the feedback device equipped on the joint as well as 
 *      construct the IOAlgorithm class representing the closed loop algorithm they wish to use to control the joint
 *   3) Finally, construct the JointInterface class that represents this joint by passing it in the output device/devices and the constructed feedback device and closed loop algorithm, if used
 *      On top of those, the constructor also will have the user specify what kind of values they will pass the joint interface class, such as positional values or speed values. This is so
 *      that the interface knows how to properly interpret commands
 *   4) From here on out, the user should be able to simply call on the joint interface class for that joint to control the joint. 
 *   note) When I say joint interface class/IOAlgorithm/output device/feedback device, I mean their specific derived classes representing the specific thing used on the joint, the formers are
 *         all abstract superclasses.
 *         
 *   If the joint is open loop controlled -- IE there is no feedback device -- then  user has no control on the selected algorithm, the joint interface constructor selects an open loop 
 *   algorithm internally since open loop algorithms aren't particularly complex or in need of user input.
 *   
 *   In order to implement modules into the program:
 *   Inherit from the proper abstract class. 
 *   If you are implementing an algorithm, and that algorithm doens't use feedback, then
 *   go down to Joint Interface's selector method and put in the logic to select your new algorithm accordingly.
 *   Don't forget to make a constructor; if it's a device class then it should take in whatever parameters it needs to output properly such as hardware pins being used.
 *   If it's an open loop algorithm class, then the constructor likely doesn't take anything and all it needs to do internally is set the algorithm's input and output types.
 *   Also for open loop algorithm classes, their class setup must specify that JointInterface is a friend class (as the constructor should be protected since the user never needs it)
 *   If it's a closed loop algorithm class, then the constructor should take in any parameters used to configure the algorithm as well as internally set the input and output types.
 *   For closed loop algorithm classes, the constructor should be public since the user will need to construct them personally
 *   
 *   Once you do implement a new module, make sure to record it in the modules in the framework text file that should be in the framework libarary's directory
*/  

#include "Energia.h"
#include "RoveDynamixel.h"
#include <PwmReader.h>


//All the types of values that can be passed to and be returned from the clases in the control framework 
enum ValueType{spd, pos};

//the types of return statuses that can be returned from the joint interface's 'run output' methods. They are to inform 
//the caller of the status of the joint after attempting to carry out the user's command
enum JointControlStatus
{
  //user's input was outside of the range boundaries accepted, user should check the valid input ranges based on what value type they specified in the interface's construction.4
  //For instance, if the user constructed the interface to use speed values, then the inputs must be between SPEED_MIN and SPEED_MAX
  InvalidInput, 

  //user did not construct the joint interface properly. This probably means the arguments passed into the construction did not work with each other; if the user is to pass in 
  //speed values, for instance, then the algorithm used for the joint must also take speed values. And whatever type of values the algorithm outputs is the type of value that 
  //the output device used to construct the interface must use in turn. 
  InvalidConstruction,

  //no errors encountered. The joint is now in the process of running the desired output, but the joint is not yet in the desired end state and the control loop 
  //should be called again until it is. IE if the user wants the joint to move to a certain position, this is the status that shall be returned if the joint is
  //in the middle of moving that that position. If the joint is not closed loop controlled, then this status shall likely never be returned for that joint.
  OutputRunning,

  //no errors encountered. The joint has reached the desired end state, IE it has reached the user specified speed or position, etc. 
  //If the joint is open loop controlled, this is the default non-error return
  OutputComplete
};

//Constants representing the ranges for the different input types.
//The common inputs and outputs between classes will fall between these limits, class should not expect to take or return values outside of these
const int SPEED_MIN = -1000, SPEED_MAX = 1000; 
const long POS_MIN = 0, POS_MAX = 72000; //started with base of 360.00 for deg, made to 36000 to work without float math, mult by two for better resolution. Each value means 360/72000 = .005 deg per value

//class prototypes:
  //joint interface and derived classes
class JointInterface; 
class TiltJoint;
class SingleMotorJoint;
class RotateJoint;
  //IOAlgorithm and derived classes
class IOAlgorithm; 
class SpdToSpdNoFeedAlgorithm;
  //output device and derived classes
class OutputDevice; 
class DirectDiscreteHBridge;
class DynamixelController;
class Sdc2130;
  //feedback devices and derived classes
class FeedbackDevice;

//Interface for controlling the joint overall from the main program's perspective.
//It handles all duties of controlling the joint; the main program just has to pass it a value to respond to.
//This class is supposed to be thought of as abstract; base classes determine what type of joint is used.
class JointInterface
{
  protected:
    
    //expected input type from base station
    ValueType inType;

    //algorithm is either passed in, or created dymanically based on what is passed to the control framework interface. 
    //called in runOutputControl
    IOAlgorithm* manip;

    //Passed to the framework interface if feedback is used. Used when looping in certain algorithms
    FeedbackDevice* feedback;

    //pointer to the output device instance which is passed in when creating the framework interface
    //called in runOutputControl
    OutputDevice* controller1;

    //tracks whether or not the parameters passed via construction were valid
    bool validConstruction;

    //function that checks to see if the user put in a proper input value when calling the runOutputControl function
    //returns true if the input is in a valid range, false if it's not
    bool verifyInput(long inputToVerify);

    //if the constructor wasn't passed an ioalgorithm to use, then this function selects one.
    //Basic algorithms only; if it's one that uses feedback then it needs to be passed in by the user, 
    //as feedback based algorithms typically are complex enough to require user-dictated initialization
    void algorithmSelector();
    
  public:
  
    //Runs the output control for this joint, IE making it move or checking 
    //feedback to see if it needs to move, whatever the algorithm for 
    //this joint is deemed to be, it runs it.
    //Must pass an integer for this implementation. Will break otherwise.
    //returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
    virtual JointControlStatus runOutputControl(const long movement);
};

//feedback devices used to help determine where the arm is and what steps need to be taken
//Used by the IOAlgorithm class to perform looping. Part of the JointInterface framework
class FeedbackDevice
{
	friend class SingleMotorJoint;
	friend class TiltJoint;
	friend class RotateJoint;
  friend class JointInterface;

  protected:
    //blank constructor for the base class
    FeedbackDevice() {} ;
  
	public:

		//returns feedback. Public because all the IOAlgorithm classes need to be able to call it,
		//and friend isn't inherited so we can't just make the abstract class our friend.
		//returns: a value representing feedback, the range of the value depends on what input type this device returns.
		//For example, if this device returns speed feedback, the values shall be in the range between SPEED_MIN and SPEED_MAX
		virtual long getFeedback();

		ValueType fType;
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
		virtual void move(const long movement);

		//blank constructor for the base class
		OutputDevice() {};

		//expected input that the output device wants.
		ValueType inType;

		//used for if the specific controller is mounted backwards on the motor joint.
		//True means invert the signal (backwards) False means just send the signal
		bool invert;
};

  
//Base class for all algorithms. Algorithms take the input from base station and do whatever computation is 
//needed to interpret the command such as controls. 
//Part of the joint interface framework
class IOAlgorithm
{
	friend class SingleMotorJoint;
	friend class TiltJoint;
	friend class RotateJoint;
  friend class JointInterface;
  
	protected:
	
		//Constructor for the base class is empty.
		IOAlgorithm() {};

    //pointer to the feedback device used on this joint, if there is any
    FeedbackDevice * feedbackDev;
    
    //the types of values that the algorithm takes in and gives out
    ValueType inType;
    ValueType outType;

    //the types of values that the algorithm expects to take in as feedback, if feedback is used
    ValueType feedbackInType;
    
		//run whatever algorithm this implements, returns value that can be directly passed to an output device
		//input: int in, representing the input values that need to be converted into output values. 
		//The specific value constraints depend on the input and output types the algorithm implements; for example if an algorithm
		//is supposed to take in speed and output position, then its input is constrained by SPEED_MIN and SPEED_MAX, and its output is constrained by POS_MIN and POS_MAX
    //bool * ret_outputFinished: parameter passed by pointer, returns true if the joint has finished its controlled movement and ready to exit the control loop, 
    //false if it's still in the middle of getting to its desired finished state IE in the middle of moving to a desired end position or reaching a desired end speed
		virtual long runAlgorithm(const long input, bool * ret_OutputFinished);
};


                                           /******************************************************************************
                                           * 
                                           * Joint Interface derived classes
                                           * 
                                           ******************************************************************************/


    
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

		//destructor since there are pointers being used
		~SingleMotorJoint();       

		//runs algorithm for movement for a singlular motor
		//input: a long that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar 
		//ranges defined in the framework's .h file
    //returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
		JointControlStatus runOutputControl(const long movement);    
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
		TiltJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed);

		//constructor without feedback. IOAlgorithm automatically selected based on the other arguments
		//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
		//cont1: The first output device controlling the first motor on this joint
		//cont2: The second output device controlling the second motor on this joint
		TiltJoint(ValueType inputType, OutputDevice* cont1, OutputDevice* cont2);	  

		//need to use a destructor to remove the pointers used
		~TiltJoint();

		//runs algorithm for moving two motors together so that it tilts the joint	
		//input: a long that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
		//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar 
		//ranges defined in the framework's .h file  
    //returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
		JointControlStatus runOutputControl(const long movement);
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

                                          /******************************************************************************
                                           * 
                                           * Algorithm derived classes
                                           * 
                                           ******************************************************************************/


//Algorithm used when speed is recieved from base station and speed is expected to be sent to the device without any feedback. 
//Open loop speed control.  
class SpdToSpdNoFeedAlgorithm : public IOAlgorithm
{
  friend class JointInterface;
    
  protected:
    //since nothing needs to change the input (what is recieved from base station) is directly returned
    //implemented to preserve modularity and uniformity in code.
    long runAlgorithm(const long input, bool * ret_OutputFinished);

    //constructor, nothing special in particular
    SpdToSpdNoFeedAlgorithm(): IOAlgorithm()
    {
      inType = spd;
      outType = spd;
    }
};


                                           /******************************************************************************
                                           * 
                                           * Output Device derived classes
                                           * 
                                           ******************************************************************************/
                                           

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
    void move(const long movement);
    
    //move command if inputting speed, input movement is in the range of SPEED_MIN to SPEED_MAX
    void moveSpeed(const int movement);
    
    //move command if inputting position, input movement is in the range of POS_MIN to POS_MAX
    //void movePos(const long movement); not implemented
   
  public:
    //constructor for controlling it via pwm
    //inputs:
    //pwmPin: GPIO pin that's connected to the pwm input of the SDC2130 device. GPIO pin number id's are defined by energia's pinmapping
    //inType: instance of the ValueType enum that defines what kind of input the device should take, currently pos or spd. The motor controller can move based on either, so pick one
    //upside down: whether or not the motor is mounted in reverse so the input values would also need to be inverted
    Sdc2130(const int pwmPin, ValueType inType, bool upsideDown);
    
    //constructor for controlling it via serial
    //Sdc2130(const int txPin, const int rxPin, ValueType inType, bool upsideDown); not implemented
};

//Standard dynamixel capable of wheel, joint, and multi-turn modes of operation.
//RoveDynamixel causes some issues to interface with but not worth rewriting it to work with classes better
//Note: Requires use of RoveDynamixel which is hard to read and understand. Edit at own risk.
class DynamixelController : public OutputDevice
{
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
    void move(const long movement);
  
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
    void move(const long movement);
    
  public:
    //constructor, user must give pin assignments with the first pin being the forward pin and the second being the reverse pin.
    //Must specify the physical orientation of the device as well, if it is mounted in reverse or not
    DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown);  

};  

                                           /******************************************************************************
                                           * 
                                           * Feedback Device derived classes
                                           * 
                                           ******************************************************************************/

//feedback device for the MA3 encoder, 12 bit version
//note: this class uses the pwm reader library. It will not compile without it.
class Ma3Encoder12b: public FeedbackDevice
{
  private:
    uint8_t pwmMappedPin;
    const int PWM_READ_MAX = 4097;
    const int PWM_READ_MIN = 1;
    
  public:

    //constructor. Public, to be called by main before passing into joint interface
    //input: char representing which GPIO pin port connects to the encoder and an int representing the pin's number. Ex: PC_2 would be 'c', 2. Remember that the encoder outputs pwm signals,
    //so the pin it connects to must be capable of reading pwm. A list of which pins are compatible are in the pwm reader library.
    Ma3Encoder12b(uint8_t mappedPinNumber): FeedbackDevice()
    {
      initPwmRead(mappedPinNumber); //function in the pwm reader library
      pwmMappedPin = mappedPinNumber;
      fType = pos;
    }

    //gets the positional feedback from the encoder. Returns positional values from POS_MIN and POS_MAX.
    //Note that the feedback can fluctuate a bit due to encoder's natural error tolerance. If unreliable results are returned consistently,
    //taking an average of returned values might work in your favor
    long getFeedback();
};
