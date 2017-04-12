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
	manip -> setFeedDevice(feed);

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
    
    //if motionComplete returned false but movement is 0, that's an indication that an error state occured
    if(motionComplete == false && mov == 0)
    {
      returnStatus = InvalidInput;
    }
    
    else if(motionComplete == true)
    {
      returnStatus = OutputComplete;
    }
    else
    {
      returnStatus = OutputRunning;
    }
    
  	//moves device with output decided on by the algorithm
  	controller1->move(mov);
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
    
    //if motionComplete returned false but movement is 0, that's an indication that an error state occured
    if(motionComplete == false && mov == 0)
    {
      returnStatus = InvalidInput;
    }
    
    else if(motionComplete == true)
    {
      returnStatus = OutputComplete;
    }
    else
    {
      returnStatus = OutputRunning;
    }
    
    motorOneSpeed = mov;
    motorTwoSpeed = mov;

    //this only happens if this joint has been coupled with another joint
    //the coupled logic will modify the speed calculated by the algorithm
    //to allow smooth tilting and rotating at the same time.
    //It is automatically assumed that the other joint is a rotate joint
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
   
    //if motionComplete returned false but movement is 0, that's an indication that an error state occured
    if(motionComplete == false && mov == 0)
    {
      returnStatus = InvalidInput;
    }
    
    else if(motionComplete == true)
    {
      returnStatus = OutputComplete;
    }
    else
    {
      returnStatus = OutputRunning;
    }

    motorOneSpeed = mov;
    motorTwoSpeed = -mov;

    //this only happens if this joint has been coupled with another joint
    //the coupled logic will modify the speed calculated by the algorithm
    //to allow smooth tilting and rotating at the same time.
    //It is automatically assumed that the other joint is a tilt joint
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

//constructor for VNH5019 motor driver. Inputs are pin asignments for hardware pins, also a bool to determine the orientation of da motor
VNH5019::VNH5019(const int PwmPin, const int InaPin, const int InbPin, bool upsideDown)
{
  PWM_PIN = PwmPin;
  INA_PIN = InaPin;
  INB_PIN = InbPin;
  
  inType = spd;
  invert = upsideDown;

  pinMode(INA_PIN, OUTPUT);
  pinMode(INB_PIN, OUTPUT);
  
  //brake motor by default
  digitalWrite(INA_PIN, LOW);
  digitalWrite(INB_PIN, LOW);
}

//move function which passes in speed ( which is converted to phase and PWM) to move device
void VNH5019::move(const long movement)
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

    //set InB to 0 and InA to 1 for "reverse" rotation
    digitalWrite(INA_PIN, HIGH);
    digitalWrite(INB_PIN, LOW);
    
    //pulsate enable pin to control motor
    PwmWrite(PWM_PIN, pwm);
  }
  
  //if forwards
  else if(mov > 0)
  {
    pwm = map(mov, 0, SPEED_MAX, PWM_MIN, PWM_MAX);
      
    //set InB to 1 and InA to 0 for forward rotation
    digitalWrite(INA_PIN, LOW);
    digitalWrite(INB_PIN, HIGH);
    
    //pulsate enable pin to control motor
    PwmWrite(PWM_PIN, pwm);
  }
  
  //stop
  else if(mov == 0)
  {
    PwmWrite(PWM_PIN, 0);//set all pins to 0 to brake motor
    digitalWrite(INA_PIN, LOW);
    digitalWrite(INB_PIN, LOW);
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
PIAlgorithm::PIAlgorithm(int inKP, int inKI, float inDT) : IOAlgorithm()
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
  hardStopPos1 = -1;
  hardStopPos2 = -1;
}

// Same as above, but if the speed_minMag is provided. speedMinMag is an int -- representing speed values -- where 
// the value passed is the slowest speed the motor is allowed to move when not simply stopping.
PIAlgorithm::PIAlgorithm(int inKP, int inKI, float inDT, int inSpeed_minMag) : IOAlgorithm()
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
  hardStopPos1 = -1;
  hardStopPos2 = -1;
}

// Function that converts rotation units into something that can be worked with more easily—such as degrees.
float PIAlgorithm::dist360(int pos_rotationUnits)
{
  return (static_cast<float>(pos_rotationUnits)*360.0/(POS_MAX-POS_MIN));
}

//finds the shortest path between two positions in degrees. Note that this function doesn't consider things like hard stops, so
//it shouldn't be used to find the BEST path. 
float PIAlgorithm::calcShortPath(float present, float dest)
{
  //basically, your destination is always to the left or right of you, one way will be shorter. The direct center of the two paths is at 180 degrees
  //from your starting point. Calculate the degrees to the destination by simply taking the difference between dest and present. If it's more than 180,
  //then the shorter path is to go the other direction.
  //If the destination is actually 180 degrees from the present, then either way is technically the shortest path. : ( Defaults to positive 180
  float degToDest = dest - present;
  if(abs(degToDest) > 180)
  {
    degToDest = ((360 - abs(dest - present)) * -1 * sign(degToDest));
  }
  else if(degToDest == -180) //use positive 180 if it's 180 degrees away
  {
    degToDest = 180;
  }
  
  return(degToDest);
}

//calculates the best route to the destination in distance in degrees from the current position in degrees.
//Returns IMPOSSIBLE_MOVEMENT if it can't reach the destination.
float PIAlgorithm::calcRouteToDest(float present, float dest)
{
  float shortPathToDest = calcShortPath(present, dest); //find out the quickest path to the destination in degrees
  if(shortPathToDest == 0) //if we're 0 degrees from the destination, just return now as we're done with a capital D
  {
    return 0;
  }
  //if there aren't hard stops set for this joint, the short path is fine
  else if(hardStopPos1 == -1)
  {
    return shortPathToDest;
  }
  //if there are hard stops, we gotta run some logic to make sure we choose a path without a collision
  else
  {
    //if the destination is the same space as the hard stop, that's impossible to pull off
    if(hardStopPos1 == dest || hardStopPos2 == dest)
    {
      return IMPOSSIBLE_MOVEMENT;
    }
    
    float shortPathToStop1 = calcShortPath(present, hardStopPos1);
    float shortPathToStop2 = calcShortPath(present, hardStopPos2);
    float comparedStopPath;
    float uncomparedStopPath;
    
    //we do this logic based on distances to our destination and to the hard stops. All calculations are based on placing destination, present, 
    //and hard stop positions on a 360 degree circle path.
    //There are three cases to consider. a) when the hard stops are both in the direction we want to go in
    // b) the hard stops are both in the direction we don't want to go in (easiest case)
    // c) the hard stops are split so one is to the direction we want to head and one is the other way on this 360 degree circle.
    // For reference when saying direction I refer to heading to the 'left' or 'right' of the current position on the circle.
    // Direction is referenced based on the sign of the calculated distances; if it's negative it's one way from present position, positive it's the other.
    
    //case a) check. If they are both in the direction we're heading, just use the closest one as the comparison point
    if((sign(shortPathToStop1) == sign(shortPathToStop2)) && sign(shortPathToStop1) == sign(shortPathToDest))
    {
      if(abs(shortPathToStop1) < abs(shortPathToStop2))
      {
        comparedStopPath = shortPathToStop1;
        uncomparedStopPath = shortPathToStop2;
      }
      else
      {
        comparedStopPath = shortPathToStop2;
        uncomparedStopPath = shortPathToStop1;
      }
    }
    
    //case b) check. Check to see if exactly one hard stop is in the same direction as our destination, and if it does, use it as the comparison point
    //this case will only work if it is preceeded by the check for case a) which rules out the possibility that both the hard
    //stops are in the same direction. 
    else if(sign(shortPathToStop1) == sign(shortPathToDest))
    {
      comparedStopPath = shortPathToStop1;
      uncomparedStopPath = shortPathToStop2;
    }
    else if(sign(shortPathToStop2) == sign(shortPathToDest))
    {
      comparedStopPath = shortPathToStop2;
      uncomparedStopPath = shortPathToStop1;
    }
    
    //if it was neither a) or b) then it's c). In this case, since they're all not in the direction we're travelling to the destination, it's a safe route
    else
    {
      return shortPathToDest;
    }
    
    //if it was a) or b), then we calculate distance to the hard stop in our way and the destination. If the destination is closer, we can move to it
    //without colliding
    if(abs(comparedStopPath) > abs(shortPathToDest))
    {
      return shortPathToDest;
    }
    
    //if it was a) or b) and one hard stop was in the way, then there are two scenarios left. We have to try and go the other, longer way around the circle
    //to our destination. If the other hard stop is in this direction -- case b) -- then it's impossible to reach the destination as it lies in between 
    //the two stops. But if case a) holds, then we might be able to still reach it depending on if the other hard stop or the destination is closer when
    //going the longer way. If the destination is closer, we can reach it, but if the hard stop is closer, then we can't go this way either, it's impossible
    else if(sign(uncomparedStopPath) == sign(shortPathToDest))//if direction to stop 2 isn't in the longer path we now want to try
    {
      float longUncomparedStopPath = (360 - abs(uncomparedStopPath)) * sign(uncomparedStopPath) * -1;
      float longPathToDest = (360 - abs(shortPathToDest)) * sign(shortPathToDest) * -1;
      
      if(abs(longUncomparedStopPath) > abs(longPathToDest)) //if dest is closer, we're good on this path
      {
        return longPathToDest;
      }
    }
    
    //if no good case held and returned, movement is impossible
    return IMPOSSIBLE_MOVEMENT;
  }
}

//function for specifying positions of hard stops attached to this joint, that is positions in degrees that the joint can't travel through
//To disable hard stops, set one or both to -1.
void PIAlgorithm::setHardStopPositions(float hardStopPos1_deg, float hardStopPos2_deg)
{
  if(!(hardStopPos1_deg == -1 || hardStopPos2_deg == -1))
  {
    hardStopPos1_deg = abs(hardStopPos1_deg);
    hardStopPos2_deg = abs(hardStopPos2_deg);
  }
  else
  {
    hardStopPos1_deg = -1;
    hardStopPos2_deg = -1;
  }
  
  hardStopPos1 = hardStopPos1_deg;
  hardStopPos2 = hardStopPos2_deg;
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
  
  float deg_disToDest = calcRouteToDest(deg_posNow, deg_posDest);
  
  //if the calculation returned that we can't reach the destination, return error state
  if(deg_disToDest == IMPOSSIBLE_MOVEMENT)
  {
    *ret_OutputFinished = false;
    return 0;
  }

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
  //scale the values from the pwm values to the common position values, IE 1-4097 to POS_MIN-POS_MAX, and return it
  return(map(readOnPeriod, PWM_READ_MIN, PWM_READ_MAX, POS_MIN, POS_MAX));
}

//get the positional feedback from the encoder via analogRead, not PWM for the MA3
//literally just use the Energia analogRead function
long Ma3Encoder10b::getFeedback()
{
  return analogRead(analogReadPin);
}


