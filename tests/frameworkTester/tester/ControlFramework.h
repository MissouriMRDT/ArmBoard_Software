#include "Energia.h"
/*  
 *   
 *   This is the library for Control Framework which includes algorithms used for various input types and device setupts, framework for output devices, framework for Feedback Devices, and the control framework interface. 
 *   Used for controlling output devices and enables looping using feedback devices. The user will only ever interact with the control framework interface(as the name implies) and algorithm selection is automatic. 
 *   The ControlFrameworkInterface class is friend classes to the OutputDevice class, IOAlgorithm class, and the FeedbackDevice class. The Feedback Device class is also friends with the IOAlgorithms class since it is used
 *   within the algorithm. 
 *   
 *   A outputDevice and feedback device are passed to the control framework interface as pointers and along with an input type determine the algorithm used. The user has no control on the selected
 *   algorithm other than through the output and feedback devices and the input from base station. Once the control framework has been created nothing can be changed (mainly the expected input must remain the same) so
 *   the output device will function correctly.
 *   
 *   Once that is done you can call the move command in the control framework interface which will move the output device accordingly. 
 *   
*/  

  //All the types of inputType that can be expected and selected
  //only spd is implemented for now as it covers Open loop
  enum InputType{spd, pos};

  //used later for maping in move commands. constants so they can be easily changed if something changes on input or expected output.
  //The common inputs and outputs between classes will fall between these limits, class should not expect to take or return values outside of these
  const int SPEED_MIN = -1000, SPEED_MAX = 1000; 
  const int POS_MIN = 0, POS_MAX = 10000;

  //feedback devices used to help determine where the arm is and what steps need to be taken
  //Used by the IOAlgorithm class to perform looping. Part of the ControlFrameworkInterface 
  class FeedbackDevice{
    friend class SingleMotorJoint;
    friend class TiltJoint;
    friend class RotateJoint;
    public:
      //blank constructor for the base class
      FeedbackDevice() {} ;
      
      //returns feedback
      virtual int getFeeback();

      //expected type of output the feedbackDevice is expected to return
      enum FeedType {};
      FeedType fType;
  };
 
  //Class containing the devices which move the arm and how they are controlled. Part of the ControlFrameworkInterface
  class OutputDevice{
    //ControlFrameworkInterface needs access to its functions
    friend class SingleMotorJoint;
    friend class TiltJoint;
    friend class RotateJoint;
    protected:
      //all the controllers have different ways of movement controlled by algorithms. The algorithm will spit out the speed or position that device will use, which will always be an int.
      //The pins are taken care of directly by the controller instance. The polarity by the sign of the integer for the move command.
      virtual void move(const int movement);
      
      //blank constructor for the base class
      OutputDevice() {};
      
      //expected input that the output device wants.
      //dont confuse with inType from base station.
      InputType outType;
	  
	  //used for if the specific controller is mounted upside down (other conversions are take care of elsewhere)
	  //True means invert the signal (upside down) False means just send the signal
	  bool invert;
  };

  //Discrete H Bridge controlled directly by the microcontroller, which has only two inputs to control forward and backwards. 
  //Has two pins (one forward, one backwards). Outputs a pwm signal on one pin or another. Never have the pins both active at once (should be impossible in the algorithm). 
  class DirectDiscreteHBridge : public OutputDevice{
    private:
      //FPWM_PIN the forward PWM and RPWM is the reverse PWM
      int FPWM_PIN, RPWM_PIN;

      //Value ranges for conversting input to expected output when sending 
      const int PWM_MIN = 0, PWM_MAX = 255;
    public:
      //constructor, user must give pin assignments with the first pin being the forward pin and the second being the reverse pin.
	  //Must specify the physical orientation of the device as well 
      DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown);  

      //function which directly moves the device. Makes final changes to the input for controlling direction and coversion to specific schemes before sending to the device.
      void move(const int movement);
  };
  
  //Base class for all algorithms. Algorithms mainly convert between input types (recieved speed from base station and want to give a position to the device) as well as control the looping.
  //Part of the control framework interface.
  class IOAlgorithm{
    friend class SingleMotorJoint;
    friend class TiltJoint;
    friend class RotateJoint;
    protected:
      //Constructor for the base class is empty.
      IOAlgorithm() {};

      //prototype for the algorithms so that they can be called without initially knowing what the algorithm will be. 
      virtual int runAlgorithm(const int in);
  };

  //Algorithm used when speed is recieved from base station and speed is expected to be sent to the device. 
  //Used in the DirectDiscreteHBridge.
  class SpdToSpdNoFeedAlgorithm : public IOAlgorithm{
    protected:
      //since nothing needs to change the input (what is recieved from base station) is directly returned
      //implemented to preserve modularity and uniformity in code.
      int runAlgorithm(const int input);
  };
  
  //What is seen by the outside program. It is passed a pointer to an output device and a feedback device as well as contains the expected input from base station. From this data selects the proper IO algorithm.
  //Can be told to runOutputControl to send instructions to each device and have it move accordingly.
  class JointInterface{
	protected:
	  //Algorithm is the same for all types of joints so it is not virtual.
	  //Called in the constructor for each of the sub classes to determine which algorithm needs to be used.
	  //Different selectors are used for open loop and feedback methods
	  //Takes in an output device during the construction of the Joint Interface and selects an algorithm for it
	  void selector(FeedbackDevice* feedback, InputType outputDeviceType, IOAlgorithm *alg)
    {
      //selects algorithm (only one currently)
      if((outputDeviceType == spd) && inType == spd)
      {
        alg = new SpdToSpdNoFeedAlgorithm();
      }
    return;  
    }
    
    void selector(InputType outputDeviceType, IOAlgorithm* alg)
    {
      //selects algorithm (not yet implemented)
      if((outputDeviceType == spd) && inType == spd)
      {
        //*alg = new SpdToSpdFeedAlgorithm();
      }
      return;
    }
	  
      //expected input type from base station(speed or position)
      InputType inType;

      //created dymanically based on what is passed to the control framework interface. 
      //called in runOutputControl
      IOAlgorithm* manip;

      //Passed to the framework interface. Used when looping in certain algorithms
      //If no feedback device(encoder, torque sensor, ect.) is used, it is expected to be passed in as a null pointer.
      FeedbackDevice* feedback;
	  
    public:
      //pass the input recieved from base station directly to the function. Calls algorithm and move functions within this funciton.
      //Must pass an integer for this implementation. Will break otherwise.
	    virtual void runOutputControl(const int movement);
    };
	  
  class SingleMotorJoint : public JointInterface{
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
      
	  //moves a singlular motor and inverts the signal if needed
      void runOutputControl(const int movement);    
  };
  
  //Two device joint where one joint must be inverted
  //The motor knows if it must be inverted. it will invert it if the motor is upside down and again for the opposite motor so they move correctly.
  class TiltJoint : public JointInterface{
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
	  
	  //moves two motors together so that it tilts the arm (both motors move together in the same objective direction though they may spin opposite eachother in physical implementation)
	  void runOutputControl(const int movement);
  };  
  
  class RotateJoint : public JointInterface{
    protected:
	  OutputDevice* controller1;
	  OutputDevice* controller2;
	
	public:
	  //constructor for a rotating joint. pass it a input type, two output devices of the same type(dont put one device that wants a spd and another which wants a pos)
	  //and a feedback device. These are all pointers other than the input type. Algorithm selection is same for all joints.
	  RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed);
	  
	  //constructor without feedback
	  RotateJoint(InputType inputType, OutputDevice* cont1, OutputDevice* cont2);
	  
	  //deletes pointers used in the joint
	  ~RotateJoint();
	  
	  //moves two motors together so that they rotate the joint. Motors will spin the opposite direction objectively but same input can be sent to each device(if both or neither is inverted) 
	  void runOutputControl(const int movement);
  };
  
