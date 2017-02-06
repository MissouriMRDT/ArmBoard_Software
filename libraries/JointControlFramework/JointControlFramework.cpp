#include "JointControlFramework.h"


                                           /*****************************
                                            *
                                            *
                                            *     JOINT INTERFACES BELOW 
                                            *           |
                                            *           V
                                            ****************************/

//if the constructor wasn't passed an ioalgorithm to use, then this function selects one.
//Basic algorithms only; if it's one that uses feedback then it needs to be passed in by the user,
//as feedback based algorithms typically are complex enough to require user-dictated initialization
void JointInterface::algorithmSelector()
{
  //selects algorithm]
  if((controller1->inType == spd) && inType == spd)
  {
    manip = new SpdToSpdNoFeedAlgorithm();
  }
  return;
}

//function that checks to see if the user put in a proper input value when calling the runOutputControl function
//returns true if the input is in a valid range, false if it's not
bool JointInterface::verifyInput(long inputToVerify)
{
  long valueMin;
  long valueMax;

  if(inType == spd)
  {
    valueMin = SPEED_MIN;
    valueMax = SPEED_MAX;
  }
  else if(inType == pos)
  {
    valueMin = POS_MIN;
    valueMax = POS_MAX;
  }
  //if any other intypes are made, put them here
  else
  {
    valueMin = 0;
    valueMax = 0;
  }

  if(valueMin <= inputToVerify && inputToVerify <= valueMax)
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

void JointInterface::coupleJoint(JointInterface* otherJoint)
{
  //only couple the joint if it isn't already coupled to avoid infinite recursion
  if(!coupled)
  {
    coupledJoint = otherJoint;
    coupled = true;
    otherJoint -> coupleJoint(this);
  }
}

//constructor for single motor joints with feedback device
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//alg: The IOAlgorithm to be used by this joint
//cont: The output device controlling the motor on this joint
//feed: The feedback device used with this joint
SingleMotorJoint::SingleMotorJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont, FeedbackDevice* feed) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont;
	feedback = feed;
	manip = alg;
	//Serial.println("Setting feedback device");
	manip -> setFeedDevice(feed);
	//Serial.println("Feedback device complete");
  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc
  if((inputType == alg->inType) && (cont->inType == alg->outType) && (alg->feedbackInType == feed->fType))
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
  //Serial.println("Single motor joint successfully set up");
}

//constructor for single motor joints without feedback.
//Constructor automatically chooses an open loop algorithm that inputs the specified inputType and outputs the values the output device accepts
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//cont: The output device controlling the motor on this joint
//feed: The feedback device used with this joint
SingleMotorJoint::SingleMotorJoint(ValueType inputType, OutputDevice* cont) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont;

	//algorithm selected internally
	algorithmSelector();

  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc.
  //Technically this should never go wrong as long as the algorithmSelector is working properly, but it never hurts to double check. If indeed the construction
  //winds up being invalid, for debugging try checking the algorithm selector method for bugs
  if((inputType == manip->inType) && (manip->outType == cont->inType))
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

//deletes pointers used in single motor joint to 'prevent memory leaks'. Most likely not neccessary but good practice.
SingleMotorJoint::~SingleMotorJoint()
{
	delete controller1;
	delete manip;
	delete feedback;
}

//run the output algorithm for this tilt joint correctly (I mean, hopefully).
//calls the algorithm to manipulate the input and sends it to the motor device.
//input: a long that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar
//ranges defined in the framework's .h file
//returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
JointControlStatus SingleMotorJoint::runOutputControl(const long movement)
{
	long mov; //var used as interrum value since algorithm can change the value
  bool motionComplete;
  JointControlStatus returnStatus;

  if(verifyInput(movement) == false)
  {
    returnStatus = InvalidInput;
  }

  else if(validConstruction)
  {
  	//calls algorithm
  	mov = manip->runAlgorithm(movement, &motionComplete);

  	//moves device with output decided on by the algorithm
  	controller1->move(mov);

    if(motionComplete == true)
    {
      returnStatus = OutputComplete;
    }
    else
    {
      returnStatus = OutputRunning;
    }
  }

  //if this joint wasn't properly constructed, nothing is run
  else
  {
    returnStatus = InvalidConstruction;
  }

	return(returnStatus);
}

//creates the joint interface for a tilt joint with a feedback device.
//Note both output devices are assumed to have the same input type
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//alg: the IOAlgorithm used by this joint
//cont1: The first output device controlling the first motor on this joint
//cont2: The second output device controlling the second motor on this joint
//feed: The feedback device used with this joint
TiltJoint::TiltJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;
	feedback = feed;
	manip = alg;
  manip -> setFeedDevice(feed);

  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc
  if((inputType == alg->inType) && (cont1->inType == alg->outType) && (alg->feedbackInType == feed->fType) && (cont1->inType == cont2->inType))
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

//creates joint interface for a tilt joint with no feedback.
//Constructor automatically chooses an open loop algorithm that inputs the specified inputType and outputs the values the output device accepts
//Note both output devices are assumed to have the same input type
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//cont1: The first output device controlling the first motor on this joint
//cont2: The second output device controlling the second motor on this joint
TiltJoint::TiltJoint(ValueType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;

	//internally selects algorithm
	algorithmSelector();

  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, both output devices line up properly, etc
  if((inputType == manip->inType) && (manip->outType == cont1->inType) && (cont1->inType == cont2->inType))
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

//Destructor for the tilt joint since it has pointers
TiltJoint::~TiltJoint()
{
	delete controller1;
	delete controller2;
	delete manip;
	delete feedback;
}

//run the output algorithm for this tilt joint correctly (I mean, hopefully).
//calls the algorithm to manipulate the input and sends it to each controller.
//Both devices get the same command since they're supposed to move together.
//input: a long that represents the desired movement. What values this int is constrained to depends on what this joint was set up to take as an input.
//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar
//ranges defined in the framework's .h file
//returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
JointControlStatus TiltJoint::runOutputControl(const long movement)
{
  int mov; //var used as interrum value since algorithm can change the value
  bool motionComplete;
  JointControlStatus returnStatus;

  if(verifyInput(movement) == false)
  {
    returnStatus = InvalidInput;
  }
  else if(validConstruction)
  {
  	//largely a temp value to store any modifications made to the input
  	int mov;

  	//runs the algorithm on the input
  	mov = manip->runAlgorithm(movement, &motionComplete);

    motorOneSpeed = mov;
    motorTwoSpeed = mov;

    //this only happens if this joint has been coupled with another joint
    //the coupled logic will modify the speed calculated by the algorithm
    //to allow smooth tilting and rotating at the same time
    if(coupled)
    {
      motorOneSpeed += coupledJoint->motorOneSpeed;
      if (motorOneSpeed > 1000)
      {
        motorOneSpeed = 1000;
      }
      if (motorOneSpeed < -1000)
      {
        motorOneSpeed = -1000;
      }

      motorTwoSpeed += coupledJoint->motorTwoSpeed;
      if (motorTwoSpeed > 1000)
      {
        motorTwoSpeed = 1000;
      }
      if (motorTwoSpeed < -1000)
      {
        motorTwoSpeed = -1000;
      }
    }
    

  	//send to the motor move command
  	controller1->move(motorOneSpeed);

  	//both the controllers should move the arm in the same direction. send command to motor 2
  	controller2->move(motorTwoSpeed);

    if(motionComplete == true)
    {
      returnStatus = OutputComplete;
    }
    else
    {
      returnStatus = OutputRunning;
    }
  }

  //if this joint wasn't properly constructed, nothing is run
  else
  {
    returnStatus = InvalidConstruction;
  }

  return(returnStatus);
}

//constructor for the rotate joint class without feedback.
//Constructor automatically chooses an open loop algorithm that inputs the specified inputType and outputs the values the output device accepts
//Assumes both passed devices have the same input type
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//cont1: The first output device controlling the first motor on this joint
//cont2: The second output device controlling the second motor on this joint
RotateJoint::RotateJoint(ValueType inputType, OutputDevice* cont1, OutputDevice* cont2) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;

	//internally selects algorithm
	algorithmSelector();

  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, both output devices line up properly, etc
  if((inputType == manip->inType) && (manip->outType == cont1->inType) && (cont1->inType == cont2->inType))
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

//constructor for the rotate joint class with feedback.
//Assumes both passed devices have the same input type.
//inputType: What kind of movement this joint should be controlled by, such as speed or position input.
//alg: the IOAlgorithm this joint should use
//cont1: The first output device controlling the first motor on this joint
//cont2: The second output device controlling the second motor on this joint
//feed: A pointer to the feedback device used on this joint.
RotateJoint::RotateJoint(ValueType inputType, IOAlgorithm *alg, OutputDevice* cont1, OutputDevice* cont2, FeedbackDevice* feed) : JointInterface()
{
	//assignments
	inType = inputType;
	controller1 = cont1;
	controller2 = cont2;
	feedback = feed;
	manip = alg;
  manip -> setFeedDevice(feed);

  //checks to make sure the passed arguments all work with each other, that is that the algorithm's input type is the same as what the user is putting in, and
  //that the algorithm's output value type is what the output device expects to take in, etc
  if((inputType == alg->inType) && (cont1->inType == alg->outType) && (alg->feedbackInType == feed->fType) && (cont1->inType == cont2->inType))
  {
    validConstruction = true;
  }
  else
  {
    validConstruction = false;
  }
}

//rotate joint deconstructor. Deletes pointers
RotateJoint::~RotateJoint()
{
	delete controller1;
	delete controller2;
	delete manip;
	delete feedback;
}

//run the output algorithm for this tilt joint correctly (I mean, hopefully).
//calls the algorithm to manipulate the input and sends it to each controller.
//One device gets an inverted direction from the other, since they move in opposite tandem on rotate joints.
//input: a long that represents the desired movement. What values this int is constrained to depend on what this joint was set up to take as an input.
//For example, if this joint runs off of speed input then the values are constrained between SPEED_MIN and SPEED_MAX, and otherwise for the similar
//ranges defined in the framework's .h file
//returns: The status of attempting to control this joint. Such as if the output is now running, or if it's complete, or if there was an error
JointControlStatus RotateJoint::runOutputControl(const long movement)
{
  long mov; //var used as interrum value since algorithm can change the value
  bool motionComplete;
  JointControlStatus returnStatus;

  if(verifyInput(movement) == false)
  {
    returnStatus = InvalidInput;
  }
  else if(validConstruction)
  {

    //runs the algorithm on the input
    mov = manip->runAlgorithm(movement, &motionComplete);

    motorOneSpeed = mov;
    motorTwoSpeed = -mov;

    //this only happens if this joint has been coupled with another joint
    //the coupled logic will modify the speed calculated by the algorithm
    //to allow smooth tilting and rotating at the same time
    if(coupled)
    {
      motorOneSpeed += coupledJoint->motorOneSpeed;
      if (motorOneSpeed > 1000)
      {
        motorOneSpeed = 1000;
      }
      if (motorOneSpeed < -1000)
      {
        motorOneSpeed = -1000;
      }

      motorTwoSpeed += coupledJoint->motorTwoSpeed;
      if (motorTwoSpeed > 1000)
      {
        motorTwoSpeed = 1000;
      }
      if (motorTwoSpeed < -1000)
      {
        motorTwoSpeed = -1000;
      }
    }

    //send to the motor move command
    controller1->move(motorOneSpeed);

    //send command to motor 2. Since this is a rotate joint, the motors need to go in opposite directions so send the second one a negafied value
    controller2->move(motorTwoSpeed);

    if(motionComplete == true)
    {
      returnStatus = OutputComplete;
    }
    else
    {
      returnStatus = OutputRunning;
    }
  }

  //if this joint wasn't properly constructed, nothing is run
  else
  {
    returnStatus = InvalidConstruction;
  }

  return(returnStatus);
}
                                           /*****************************
                                            *
                                            *
                                            *     OUTPUT DEVICES BELOW 
                                            *           |
                                            *           V
                                            ****************************/

//constructor for a dynamixel for any mode
//Calls the init function from RoveDynamixel.h to initialize the dynamixel
/*Inputs: TX -> tx pin numerical id, as defined by energia's pinmapping
          RX -> tx pin numerical id, as defined by energia's pinmapping
          upsideDown -> Whether or not the dyna is mounted in reverse and thus the inputs need to be reversed as well
          type -> Instance of the DynamixelType enum that defines the dynamixel brand such as AX, MX, etc
          id -> id of the dynamixel. Note that if you aren't sure, you'll need to use a different program to set the id yourself
          uartIndex -> which hardware uart to use when talking to it, 0-7
          baud -> baud rate of the serial communication
          mode -> instance of the DynamixelMode enum that defines the dynamixel's mode such as Wheel, Joint, etc
*/
DynamixelController::DynamixelController(const int Tx, const int Rx, bool upsideDown, DynamixelType type, uint8_t id, uint8_t uartIndex, uint32_t baud, DynamixelMode mode) : OutputDevice()
{
  //assignments
  Tx_PIN = Tx;
  Rx_PIN = Rx;
  baudRate = baud;
  invert = upsideDown;

  if(mode == Wheel)
    inType = spd;

  else if(mode == Joint)
    inType = pos;

  //view RoveDynamixel.h for details on all functions called here
  //note: no comments in RoveDynamixel
  DynamixelInit(&dynamixel, type, id, uartIndex, baud);

  //actually sets the values correctly for the dynamixel
  DynamixelSetId(&dynamixel, id);
  DynamixelSetBaudRate(dynamixel, baud);
  DynamixelSetMode(dynamixel, mode);
}

//sends the move command for the wheel mode based on a speed, an int constrained between the SPEED_MIN and SPEED_MAX constants.
//clockwise will be considered forward and ccw is reverse
void DynamixelController::move(const long movement)
{
  //stores the error returned by the spin wheel function
  uint8_t errorMessageIgnore;
  long mov = movement;
  uint16_t send;

  //if mounted upside down then invert the signal passed to it and move accordingly
  if (invert)
  {
	  //inverts the input easily
  	mov = -mov;
  }

  //if supposed to move backwards(ccw)
  if(mov < 0)
  {
	  send = map(mov, 0, SPEED_MAX, DYNA_SPEED_CCW_MAX, DYNA_SPEED_CCW_MIN);

	  //calls spin wheel function from RoveDynamixel
    //can take up to a uint16_t which exceeds a standard int but
    errorMessageIgnore = DynamixelSpinWheel(dynamixel, send);
  }

  //if forwards (cw)
  else if(mov > 0)
  {
    send = map(mov, 0, SPEED_MAX, DYNA_SPEED_CW_MAX, DYNA_SPEED_CW_MIN);

	//calls spin wheel function from RoveDynamixel
    //can take up to a uint16_t which exceeds a standard int but
    errorMessageIgnore = DynamixelSpinWheel(dynamixel, send);
  }

  //stop
  else if(mov == 0)
  {
    //calls spin wheel function from RoveDynamixel
    //can take up to a uint16_t which exceeds a standard int but
    errorMessageIgnore = DynamixelSpinWheel(dynamixel, 0);
  }

  return;
}

/*constructor for the Sdc2130 when controlled via pwm.
  Note that inType of speed is the only one currently implemented.
  inputs:
  pwmPin: GPIO pin that's connected to the pwm input of the SDC2130 device. GPIO pin number id's are defined by energia's pinmapping
  inType: instance of the ValueType enum that defines what kind of input the device should take, currently pos or spd. The motor controller can move based on either, so pick one
  upside down: whether or not the motor is mounted in reverse so the input values would also need to be inverted
 */
Sdc2130::Sdc2130(const int pwmPin, ValueType inType, bool upsideDown): OutputDevice()
{
	PWM_PIN = pwmPin;
	inType = inType;
	invert = upsideDown;
	controlType = Pwm;
	pwmVal = 0;
}

//sdc2130 general move function. Selects the specific move function
//based on the specified movement type, such as speed or position
//Input: Can be either position or speed values constrained between SPEED_MIN and SPEED_MAX or POS_MIN and POS_MAX
void Sdc2130::move(const long movement)
{
	if(inType == spd)
	{
		moveSpeed(movement);
	}
	//position-input movement not implemented
	else if(inType == pos)
	{

	}
}

//sdc2130 movement command for moving based on a speed input, IE a value constrained between the SPEED_MIN and SPEED_MAX constants.
//Simply causes the joint to move a couple of positional ticks,
//as true speed based movement is not implemented due to strange delays in sdc2130 response
void Sdc2130::moveSpeed(const int movement)
{
	int speed = movement;

	if(invert)
	{
		speed = -speed;
	}

	if(controlType == Pwm)
	{
		if(speed > 0)
		{
			pwmVal+=POS_INC;
		}
		else
		{
			pwmVal-=POS_INC;
		}

		if(pwmVal < PWM_MIN)
		{
			pwmVal = PWM_MIN;
		}
		else if(pwmVal > PWM_MAX)
		{
			pwmVal = PWM_MAX;
		}

		PwmWrite(PWM_PIN, pwmVal);

	}

	//serial control not implemented
	else
	{

	}
}


/* Creates the device. Assigns the pins correctly.
 * int FPIN: the GPIO pin connected to the H bridge's forward transistor, pin number is defined by energia's pinmapping
 * int RPIN: the GPIO pin connected to the H bridge's forward transistor, pin number is defined by energia's pinmapping
 * bool upside down: Whether or not the motor is mounted in reverse and as such the inputs also need to be reversed
 */
DirectDiscreteHBridge::DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown) : OutputDevice()
{
	FPWM_PIN = FPIN;
	RPWM_PIN = RPIN;
	inType = spd;
	invert = upsideDown;
}

//moves by passing a pwm signal to the H bridge.
//Input: expects int values constrained between the SPEED_MIN and SPEED_MAX constants
void DirectDiscreteHBridge::move(const long movement)
{
	int mov = movement;
	int pwm = 0;

	//if mounted upside down then invert the signal passed to it and move accordingly
	if (invert)
	{
		//inverts the input easily
		mov = -mov;
	}

	//if supposed to move backwards
	if(mov < 0)
	{
		mov = abs(mov);
		pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

		//stop the transistor for the other direction -- if both were on, the h bridge would short out
		PwmWrite(FPWM_PIN, 0);
		PwmWrite(RPWM_PIN, pwm);
	}

	//if forwards
	else if(mov > 0)
	{
		pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

		//stop the transistor for the other direction -- if both were on, the h bridge would short out
		PwmWrite(FPWM_PIN, pwm);
		PwmWrite(RPWM_PIN, 0);
	}

	//stop
	else if(mov == 0)
	{
		PwmWrite(RPWM_PIN, 0);
		PwmWrite(FPWM_PIN, 0);
	}

	return;
}


//DRV8388 constructor here
// pin asignments for enable pin and phase pin, also a bool to determine the orientation of da motor
DRV8388::DRV8388 (const int EN_PIN, const int PH_PIN, bool upsideDown) : OutputDevice()
{
  ENABLE_PIN = EN_PIN;
  PHASE_PIN = PH_PIN;
  inType = spd;
  invert = upsideDown;

  pinMode(PHASE_PIN, OUTPUT);
}


//move function which passes in speed ( which is converted to phase and PWM) to move device
void DRV8388::move(const long movement)
{
  int mov = movement;
  int pwm = 0;
  
  //if mounted upside down then invert the signal passed to it and move accordingly
  if (invert)
  {
    //inverts the input easily
    mov = -mov;
  }
  
  //if supposed to move backwards
  if(mov < 0)
  {
    mov = abs(mov);
    pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);

    //set phase to 1 for "reverse" rotation
    digitalWrite(PHASE_PIN, HIGH);
    
    //pulsate enable pin to control motor
    PwmWrite(ENABLE_PIN, pwm);
  }
  
  //if forwards
  else if(mov > 0)
  {
    pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);
      
    //set phase to 0 for "forward" rotation
    digitalWrite(PHASE_PIN, LOW);
    
    //pulsate enable pin to control motor
    PwmWrite(ENABLE_PIN, pwm);
  }
  
  //stop
  else if(mov == 0)
  {
    PwmWrite(ENABLE_PIN, 0);//set enable to 0 to brake motor
    //phase don't matter
  }
  
  return;
}





                                           /*****************************
                                            *
                                            *
                                            *     ALGORITHMS BELOW 
                                            *           |
                                            *           V
                                            ****************************/

                                            
//if this IOAlgorithm uses feedback device, this function is used by the joint interface to set it, and sets the feedbackInitialized flag to true
void IOAlgorithm::setFeedDevice(FeedbackDevice* fdDev)
{
  feedbackDev = fdDev;
  feedbackInitialized = true;
}


// Constructor. 
// Input: inKI, the integer representing the PI constant Ki
//        inKP, the integer representing the PI constant Kp
//        inDt, the float value representing the time differential between calls of the runAlgorithm method. 
//        The PI Algorithm is meant to be put into a loop by the main program until it is finished, and dt represents 
//        the amount of time that passes in between the calls to the algorithm in that loop, in seconds.
PIAlgorithm::PIAlgorithm(int inKI, int inKP, float inDT) : IOAlgorithm()
{
  // Assign the values of the PIAlgorithm class to the ones provided to the constructor.
  // Sets errorSummation to be zero so that the error can be accurately accouted for for each phase of the
  // closed loop algorithm.
  // speed_minMag is not provided to the constructor, so a default value is assumed.
  KI = inKI;
  KP = inKP;
  DT = inDT;
  speed_minMag = DEFAULT_MINMAG;
  errorSummation = 0;
  inType = pos;
  outType = spd;
  feedbackInType = pos;
}

// Same as above, but if the speed_minMag is provided. speedMinMag is an int -- representing speed values -- where 
// the value passed is the slowest speed the motor is allowed to move when not simply stopping.
PIAlgorithm::PIAlgorithm(int inKI, int inKP, float inDT, int inSpeed_minMag) : IOAlgorithm()
{
  // Assign the values of the PIAlgorithm class to the ones provided to the constructor.
  // Sets errorSummation to be zero so that the error can be accurately accouted for for each phase of the
  // closed loop algorithm.
  KI = inKI;
  KP = inKP;
  DT = inDT;
  speed_minMag = inSpeed_minMag;
  errorSummation = 0;
  inType = pos;
  outType = spd;
  feedbackInType = pos;
}

// Function that converts rotation units into something that can be worked with more easily—such as degrees.
float PIAlgorithm::dist360(int pos_rotationUnits)
{
  return (static_cast<float>(pos_rotationUnits)*360.0/(POS_MAX-POS_MIN));
}

// Full function that takes a postion input (an value for the gear to move to) as well as a boolean to check if the movement
// of the gear has been succeeded. If the bool "ret_OutputFinished" is true, then
// the joint has reached its desired position and the method will return 0 speed.
// Upon being called, the method will run PI logic on the passed input and return a value of speed for how fast
// the motor controlling this joint should move
long PIAlgorithm::runAlgorithm(const long input, bool * ret_OutputFinished)
{
  // Check if the Algorithm class has actually been initialized or not. If not, kill the function.
  if (feedbackInitialized == false)
  {
    *ret_OutputFinished = false;	  
    return 0;
  }
	
  // Create local variables for the function to work with, as well as convert values to degrees.
  long posDest = input;
  long posNow = feedbackDev->getFeedback();
  float deg_posDest = dist360(posDest);
  float deg_posNow = dist360(posNow);
  float deg_disToDest = deg_posDest - deg_posNow;

  // Check if the current value of the rotation is within the margin-of-error acceptable for the location.
  // If so, set the value to be OutputFinished to be true, so that the function should not run again.
  if (-DEG_DEADBAND < deg_disToDest && deg_disToDest < DEG_DEADBAND)
  {
    *ret_OutputFinished = true;
    return 0;
  }

  // Calculate the value of how fast the motor needs to turn at its current interval
  int spd_out = (KP * deg_disToDest + KI * errorSummation);

  // Check for fringe cases if the speed out value is outside of the acceptable range,
  // forcing the value to return back into the acceptable range.
  if (spd_out > SPEED_MAX)
  {
    spd_out = SPEED_MAX;
  } else if (spd_out < SPEED_MIN)
  {
    spd_out = SPEED_MIN;
  } else if (spd_out < speed_minMag && spd_out > 0)
  {
    spd_out = speed_minMag;
  } else if (spd_out > -speed_minMag && spd_out < 0)
  {
    spd_out = -speed_minMag;
  }
  else
  {
    // Calculate and add the value to the errorSummation so that we can keep track
    // of how much of an error has been accumulated.
    errorSummation+=(deg_disToDest * DT);
  }
	
  // Ensure that the output is not finished (since it has gotten this far) so that the function
  // should be run once again.
  *ret_OutputFinished = false;
	
  // return the value of the speed we calculated
  return spd_out;
}

//speed to speed algorithm with no feedback just returns the input
//input: expects speed values, constrained between the global SPEED_MIN and SPEED_MAX values
//output: Actually just returns the same values. Meh
long SpdToSpdNoFeedAlgorithm::runAlgorithm(const long input, bool * ret_OutputFinished)
{
  //since there's no feedback, there's no output control so just automatically return true
  *ret_OutputFinished = true;
  return input;
}

                                           /*****************************
                                            *
                                            *
                                            *     FEEDBACK DEVICE(S) BELOW 
                                            *           |
                                            *           V
                                            ****************************/
                                            

//gets the positional feedback from the encoder. Returns positional values from POS_MIN and POS_MAX
long Ma3Encoder12b::getFeedback()
{
  uint32_t readOnPeriod = getOnPeriod_us(pwmMappedPin); //function part of the pwm reader library
  //values will be between PWM_READ_MIN and PWM_READ_MAX, that is 1 and 4097. Or at least they should be; if it's above there was slight comm error and it can be scaled down to the max val.
  if(readOnPeriod > PWM_READ_MAX)
  {
    readOnPeriod = PWM_READ_MAX;
  }

  //Alternatively, if the value read is 0 then it means the duty cycle is either at 0 or 100%, so if we get a 0 then we need to check the duty cycle to see which it is.
  //If it's 0%, then use min value. If it's 100%, use max value.
  else if(readOnPeriod == 0)
  {
    if(getDuty(pwmMappedPin) == 0)
    {
      readOnPeriod = PWM_READ_MIN;
    }
    else
    {
      readOnPeriod = PWM_READ_MAX;
    }
  }
  Serial.print("Encoder value: ");
  Serial.println(map(readOnPeriod, PWM_READ_MIN, PWM_READ_MAX, POS_MIN, POS_MAX));
  //scale the values from the pwm values to the common position values, IE 1-4097 to POS_MIN-POS_MAX, and return it
  return(map(readOnPeriod, PWM_READ_MIN, PWM_READ_MAX, POS_MIN, POS_MAX));
}
