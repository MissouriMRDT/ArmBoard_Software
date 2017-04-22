#include "arm.h"

/* programmers: Drue Satterfield, David Strickland, Christopher Dutcher, Jake Hasenfratz
 * Spring 17
 * 
 * The main arm program, for revision two of the arm hardware. 
 * Handles all arm processes and telemetry, based on commands from base station using the rovecomm standard.
 * 
 * For more information on the hardware, visit the MRDT ArmBoardHardware github
 * For more information on the software, visit the MRDT ArmBoardSoftware github, plus the assorted libraries used have documentation under their folders at energia\libraries
 * 
 * Hardware used: literally all of the timers and all of the pwm modules on the tiva tm4c1294ncpdt
 */


//Joint and hardware wrappers
JointInterface* joint1;
JointInterface* joint2;
JointInterface* joint3;
JointInterface* joint4;
JointInterface* joint5;
JointInterface* gripperMotor;
JointInterface* gripperServo;

PIAlgorithm* joint1Alg;
PIAlgorithm* joint2Alg;
PIAlgorithm* joint3Alg;
PIAlgorithm* joint4Alg;
PIAlgorithm* joint5Alg;

Ma3Encoder12b joint1Encoder(ENCODER1_READING_PIN);
Ma3Encoder12b joint2Encoder(ENCODER2_READING_PIN);
Ma3Encoder12b joint3Encoder(ENCODER3_READING_PIN);
Ma3Encoder12b joint4Encoder(ENCODER4_READING_PIN);
Ma3Encoder12b joint5Encoder(ENCODER5_READING_PIN);

GenPwmPhaseHBridge dev1(MOT1_PWN_PIN, HBRIDGE1_PHASE_PIN, HBRIDGE1_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev2(MOT2_PWN_PIN, HBRIDGE2_PHASE_PIN, HBRIDGE2_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev3(MOT3_PWN_PIN, HBRIDGE3_PHASE_PIN, HBRIDGE3_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev4(MOT4_PWN_PIN, HBRIDGE4_PHASE_PIN, HBRIDGE4_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev5(MOT5_PWN_PIN, HBRIDGE5_PHASE_PIN, HBRIDGE5_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge gripMotorDev(GRIPMOT_PWM_PIN, GRIPMOT_PHASE_PIN, GRIPMOT_NENABLE_PIN, false, true);
RCContinuousServo gripServoDev(GRIPPER_SERVO_PWM_PIN, false);

//variables used to control joints during closed loop control
long joint1Destination;
long joint2Destination;
long joint3Destination;
long joint4Destination;
long joint5Destination;

ControlSystems currentControlSystem; //tracks what control system arm is currently using

void setup() {} //useless

void loop() {
  CommandResult result;
  uint16_t commandId;
  size_t commandSize;
  char commandData[255]; 
  uint32_t watchdogTimer_us = 0; //increment this value everytime we don't get a command. When we've waited for a command for longer than our timeout value, stop all arm movement

  initialize(); //control devices initted in here

  delay(1000);
  
  //switchToClosedLoop(); //for debugging. RED currnetly lacks command to switch between schemes, has to be done manually
  
  while(1) //main program loop. Listen for communications from the endefector or from base station, and proceed based on that transmission
  {
    commandSize = 0;//reset variables
    commandId = 0;

    roveComm_GetMsg(&commandId, &commandSize, commandData);
    if(commandId != 0) //command packets come in 1 or 2 bytes. If it's any other size, there was probably a comm error
    {
      watchdogTimer_us = 0; //reset watchdog timer since we received a command

      switch(commandId)
      {
        case ArmStop:
          result = stopArm();
          break;
          
        case ArmJ1:
        case LY_ArmJ1:
          result = moveJ1(*(int16_t*)(commandData));
          break;

        case ArmJ2:
        case LY_ArmJ2:
          result = moveJ2(*(int16_t*)(commandData));
          break;

       case ArmJ3:
       case LY_ArmJ3:
          result = moveJ3(*(int16_t*)(commandData));
          break;

        case ArmJ4: 
        case LY_ArmJ4: 
          result = moveJ4(*(int16_t*)(commandData));
          break;

        case ArmJ5: 
        case LY_ArmJ5: 
          result = moveJ5(*(int16_t*)(commandData));
          break;

        case MoveGripper: 
        case LY_MoveGripper: 
          result = moveGripper(*(int16_t*)(commandData));
          break;

        case MoveGripServo: 
          result = moveGripper(*(int16_t*)(commandData));
          break;

        case UseOpenLoop: 
          result = switchToOpenLoop();
          break;
          
        case UseClosedLoop: 
          result = switchToClosedLoop();
          break;

        case ArmEnableAll: 
          masterPowerSet((*(bool*)(commandData)));
          allMotorsPowerSet(*(bool*)(commandData));
          break;

        case ArmEnableMain: 
          masterPowerSet(*(bool*)(commandData));
          break;

        case ArmAbsoluteAngle: 
          setArmDestinationAngles(((float*)(commandData)));
          break;

        case ArmGetPosition: 
          float currentPositions[ArmJointCount];
          getArmPositions(currentPositions);
          roveComm_SendMsg(ArmCurrentPosition, sizeof(float) * ArmJointCount, currentPositions);
          break;

        default:
          break; //do nothing if it's not a known ID
          
      } //end switch

      if(result != Success)
      {
        //todo: if there's ever any telemetry about what to do when the command isn't successful, this is where we'll send telemetry back about it
        // I'm not quite sure how to do this. Somebody else's addition would be very helpful.
      }
    }//end if(commandId != 0)

    //if no messages were recieved, increment our watchdog counter. If the counter has gone over a certain period of time since we last got a transmission, cease all movement.
    //This is to keep the arm from committing suicide on the environment/the rover if communications ever get interrupted while it's in the middle of moving
    else
    {
      uint8_t microsecondDelay = 10;
      delayMicroseconds(microsecondDelay);

      watchdogTimer_us += microsecondDelay;

      if(watchdogTimer_us >= WATCHDOG_TIMEOUT_US) //if more than our timeout period has passed, then kill arm movement
      {
        Serial.println("Timed out");
        stopArm();
        watchdogTimer_us = 0;
      }
    }//end else

    if(checkOvercurrent()) //CURRENTLY TOO BUGGY TO USE
    { 
      //masterPowerDisable();
      //Serial.println("Disabling power, OC");
      //TODO: send telemetry back to base station
    }

    //todo: Check motors for fault conditions, do stuff

  }//end while

}

//setup all crucial software processes such as rovecomm, serial, and the closed loop timer, and set up static GPIO pins.
//Also initialize the joint constructs to their initial state, IE open loop controlled
void initialize()
{
  roveComm_Begin(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  Serial.begin(9600);

  pinMode(HBRIDGE1_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE2_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE3_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE4_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE5_NFAULT_PIN,INPUT);
  pinMode(GRIPMOT_NFAULT_PIN,INPUT);

  pinMode(OC_NFAULT_PIN,INPUT);
  pinMode(POWER_LINE_CONTROL_PIN,OUTPUT);

  //all joints are initialized to open loop control format
  joint1 = new RotateJoint(spd, &dev1, &dev2);
  joint2 = new TiltJoint(spd, &dev1, &dev2);
  joint3 = new SingleMotorJoint(spd, &dev3);
  joint4 = new RotateJoint(spd, &dev4, &dev5);
  joint5 = new TiltJoint(spd, &dev4, &dev5);
  gripperMotor = new SingleMotorJoint(spd, &gripMotorDev);
  gripperServo = new SingleMotorJoint(spd, &gripServoDev);
  
  joint1 -> coupleJoint(joint2);
  joint4 -> coupleJoint(joint5);

  masterPowerSet(false);

  allMotorsPowerSet(true);

  currentControlSystem = OpenLoop;
  
  //set timer 0 to fire at a rate where the different PI algorithms will all be updated at their expected timeslice in seconds. 
  //There are 5 controls to update independently. They update one at a time, one being serviced every time the timer fires. So it takes 5 timer
  //firings for any individual control to get updated again. Meaning the timeslice of the timer itself must be one fifth of the PI algorithms overall timeslice so that 
  //when it cycles back around the overall timeslice will have passed
  setupTimer0((PI_TIMESLICE_SECONDS/5.0) * 1000000.0); //function expects microseconds
}

//checks to see if the master powerline has overcurrented.
//Returns true if so, false if not
bool checkOvercurrent()
{
  if(readMasterCurrent() > CURRENT_LIMIT)
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

//Turns on or off the main power line
CommandResult masterPowerSet(bool enable)
{
  if(enable)
  {
    digitalWrite(POWER_LINE_CONTROL_PIN, HIGH);
  }
  else
  {
    digitalWrite(POWER_LINE_CONTROL_PIN, LOW);
  }
}

//turns on or off all the motors
void allMotorsPowerSet(bool enable)
{
  if(enable)
  {
    dev1.setPower(true);
    dev2.setPower(true);
    dev3.setPower(true);
    dev4.setPower(true);
    dev5.setPower(true);
    gripMotorDev.setPower(true);
    gripServoDev.setPower(true);
  }
  else
  {
    dev1.setPower(false);
    dev2.setPower(false);
    dev3.setPower(false);
    dev4.setPower(false);
    dev5.setPower(false);
    gripMotorDev.setPower(false);
    gripServoDev.setPower(false);
  }
}

//reads the current detected in the main power line. 
//returns: Amps moving through the main power line
float readMasterCurrent()
{
  //Note this is only an estimation, as it assumes the VCC is currently 3.3V when in reality it tends to be between 3V and 3.3V
  int adc = analogRead(CURRENT_READ_PIN);
  float voltRead =((float)(adc))/1023.0*(VCC) - VCC*.1; //converts read value (from 0 to 1023) to volts (0 to VCC). Current sensor has an offset of .33V as well
  if(voltRead < 0)
  {
    return(0);
  }
  else
  {
    float ampsRead = voltRead/CURRENT_SENSOR_RATIO;
    return ampsRead;
  }
}

//stops all arm movement by disabling the main power line and disabling all motors
CommandResult stopArm()
{
  masterPowerSet(false);
  allMotorsPowerSet(false);
}

//turns on or off the motors attached to joint 1 and 2
void j12PowerSet(bool powerOn)
{
  dev1.setPower(powerOn);
  dev2.setPower(powerOn);
}

//turns on or off the motor attached to joint 3
void j3PowerSet(bool powerOn)
{
  dev3.setPower(powerOn);
}

//turns on or off the motors attached to joint 4 and 5
void j45PowerSet(bool powerOn)
{
  dev4.setPower(powerOn);
  dev5.setPower(powerOn);
}

//turns on or off the motors attached to the gripper
void gripperPowerSet(bool powerOn)
{
  gripMotorDev.setPower(powerOn);
  gripServoDev.setPower(powerOn);
}

//Sets the angles for the joints of the arm to travel to
//Input: an angle array. angles[0] = joint1 destination, etc
//Note that this will only be performed when the current control system being used is closed loop
//Note that the angles are numerically described using the joint control framework standard
CommandResult setArmDestinationAngles(float* angles)
{ 
  //angles comes in as an array
  if(currentControlSystem == ClosedLoop)
  {
    joint1Destination = angles[0];
    joint2Destination = angles[1];
    joint3Destination = angles[2];
    joint4Destination = angles[3];
    joint5Destination = angles[4];
  }
}

//moves the first joint
//note that this function only operates if open loop is currently being used; else, use the setArmDestinationAngles function for closed loop movvement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ1(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    joint1->runOutputControl(moveValue);
    if(moveValue != 0)
    {
      Serial.print("Moving j1: ");
      Serial.println(moveValue);
    }
  }
}

//moves the second joint
//note that this function only operates if open loop is currently being used; else, use the setArmDestinationAngles function for closed loop movvement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ2(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    joint2->runOutputControl(moveValue);
    if(moveValue != 0)
    {
      Serial.print("Moving j2: ");
      Serial.println(moveValue);
    }
  }
}

//moves the third joint
//note that this function only operates if open loop is currently being used; else, use the setArmDestinationAngles function for closed loop movvement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ3(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    joint3->runOutputControl(moveValue);
    if(moveValue != 0)
    {
      Serial.print("Moving j3: ");
      Serial.println(moveValue);
    }
  }
}

//moves the fourth joint
//note that this function only operates if open loop is currently being used; else, use the setArmDestinationAngles function for closed loop movvement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ4(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    joint4->runOutputControl(moveValue);
    if(moveValue != 0)
    {
      Serial.print("Moving j4: ");
      Serial.println(moveValue);
    }
  }
}

//moves the fifth joint
//note that this function only operates if open loop is currently being used; else, use the setArmDestinationAngles function for closed loop movvement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ5(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    joint5->runOutputControl(moveValue);
    if(moveValue != 0)
    {
      Serial.print("Moving j5: ");
      Serial.println(moveValue);
    }
  }
}

//moves the gripper open/closed
//note that this function only operates if open loop is currently being used; else, use the setArmDestinationAngles function for closed loop movvement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveGripper(int16_t moveValue)
{
  gripperMotor->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving j6: ");
    Serial.println(moveValue);
  }
}

//spins the gripper servo
//note that this function only operates if open loop is currently being used; else, use the setArmDestinationAngles function for closed loop movvement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveGripServo(int16_t moveValue)
{
  gripperServo->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving gripper servo: ");
    Serial.println(moveValue);
  }
}

//switches the arm over to open loop control method; this will disable closed loop functions and functionality
//while enabling open loop functions and functionality
CommandResult switchToOpenLoop()
{
  //disable closed loop interrupts before doing any operation to preserve thread safety
  TimerDisable(TIMER0_BASE, TIMER_A); 
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  delay(10);
  
  //reconstruct joint interfaces with open loop format
  delete joint1;
  delete joint2;
  delete joint3;
  delete joint4;
  delete joint5;
  joint1 = new RotateJoint(spd, &dev1, &dev2);
  joint2 = new TiltJoint(spd, &dev1, &dev2);
  joint3 = new SingleMotorJoint(spd, &dev3);
  joint4 = new RotateJoint(spd, &dev4, &dev5);
  joint5 = new TiltJoint(spd, &dev4, &dev5);

  currentControlSystem = OpenLoop;
}

//switches the arm over to closed loop control method; this will enable closed loop functions and functionality
//while disabling open loop functions and functionality
CommandResult switchToClosedLoop()
{
  currentControlSystem = ClosedLoop;
    
  //reconstruct joints with closed loop algorithms
  delete joint1;
  delete joint2;
  delete joint3;
  delete joint4;
  delete joint5;
  delete joint1Alg;
  delete joint2Alg;
  delete joint3Alg;
  delete joint4Alg;
  delete joint5Alg;
  joint1Alg = new PIAlgorithm(5,4,PI_TIMESLICE_SECONDS);
  joint2Alg = new PIAlgorithm(5,4,PI_TIMESLICE_SECONDS);
  joint3Alg = new PIAlgorithm(21,4,PI_TIMESLICE_SECONDS);
  joint4Alg = new PIAlgorithm(21,4,PI_TIMESLICE_SECONDS);
  joint5Alg = new PIAlgorithm(21,4,PI_TIMESLICE_SECONDS);
  joint1 = new RotateJoint(pos, joint1Alg, &dev1, &dev2, &joint1Encoder);
  joint2 = new TiltJoint(pos, joint2Alg, &dev1, &dev2, &joint2Encoder);
  joint3 = new SingleMotorJoint(pos, joint3Alg, &dev3, &joint3Encoder);
  joint4 = new RotateJoint(pos, joint4Alg, &dev4, &dev5, &joint4Encoder);
  joint5 = new TiltJoint(pos, joint5Alg, &dev4, &dev5, &joint5Encoder);

  //have default position destination values be the joints' current positions, so they hold still when switchover occurs until base station sends a new position to go towards
  joint1Destination = joint1Encoder.getFeedback();
  joint2Destination = joint2Encoder.getFeedback();
  joint3Destination = joint3Encoder.getFeedback();
  joint4Destination = joint4Encoder.getFeedback();
  joint5Destination = joint5Encoder.getFeedback();

  //enable closed loop interrupts, and only after devices have been properly reconstructed with the new algorithms as we 
  //don't want the timer interrupt firing while we're still modifying the classes
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A);
}

//sets up timer 0 so that it can service closed loop functionality; 
//closed loop works by periodically updating all of the joints' positional destinations on a consistent timeslice.
//Safest option for this service is to use a timer, and timer 0 is the only timer that's not in use by the rest of the program (timers 1-5 are used to read pwm).
//After setup, timer remains ready but not running. The switchToClosedLoop function turns on the timer and its interrupt, while switchToOpenLoop turns it back off.
void setupTimer0(float timeout_micros)
{
  uint32_t timerLoad = 16000000.0 * (timeout_micros/1000000.0); // clock cycle (cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes

  //enable timer hardware
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

  delay(1); //let the periph finish processing

  //set clock to internal precision clock of 16 Mhz
  TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_PIOSC);

  //configure timer for count up periodic
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  
  //set timer load based on earlier calculated value
  TimerLoadSet(TIMER0_BASE, TIMER_A, (timerLoad)); 

  //set up interrupts. The order here is actually important, TI's forums reccomend 
  //setting up new interrupts in this exact fashion
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_TIMER0A);

  //register interrupt functions 
  TimerIntRegister(TIMER0_BASE, TIMER_A, &closedLoopUpdateHandler);
  
  //enable master system interrupt
  IntMasterEnable();

  delay(1);
}

//fills a float array with the current positions of the joints. 
//Angles are numerically described using the joint control framework standard
CommandResult getArmPositions(float positions[ArmJointCount])
{
  positions[0] = joint1Encoder.getFeedback();
  positions[1] = joint2Encoder.getFeedback();
  positions[2] = joint3Encoder.getFeedback();
  positions[3] = joint4Encoder.getFeedback();
  positions[4] = joint5Encoder.getFeedback();
}

//Timer 0 periodic timeout interrupt. 
//In this interrupt, closed loop protocol is serviced by updating the arm joint's destination positions.
//The interrupt doesn't decide the destination positions; that's done by other functions. Instead, it just tells the joints 
//to go towards their predetermined positions. This is done because closed loop uses PI logic controls, and PI logic needs to be updated 
//on a consistent timeslice for its algorithm to calculate properly.
void closedLoopUpdateHandler()
{
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  static int jointUpdated = 1;
  jointUpdated += 1;
  if(jointUpdated > 5)
  {
    jointUpdated = 1;
  }
  if(jointUpdated == 1)
  {
    joint1->runOutputControl(joint1Destination);
  }
  else if(jointUpdated == 2)
  {
    joint2->runOutputControl(joint2Destination);
  }
  else if(jointUpdated == 3)
  {
    joint3->runOutputControl(joint3Destination);
  }
  else if(jointUpdated == 4)
  {
    joint4->runOutputControl(joint4Destination);
  }
  else if(jointUpdated == 5)
  {
    joint5->runOutputControl(joint5Destination);
  }
}
