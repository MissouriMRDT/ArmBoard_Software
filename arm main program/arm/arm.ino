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

PIAlgorithm joint1Alg(5,4,PI_TIMESLICE_SECONDS);
PIAlgorithm joint2Alg(5,4,PI_TIMESLICE_SECONDS);
PIAlgorithm joint3Alg(ElbowKp,ElbowKi,PI_TIMESLICE_SECONDS);
PIAlgorithm joint4Alg(21,4,PI_TIMESLICE_SECONDS);
PIAlgorithm joint5Alg(21,4,PI_TIMESLICE_SECONDS);

Ma3Encoder12b joint1Encoder(ENCODER1_READING_PIN);
Ma3Encoder12b joint2Encoder(ENCODER2_READING_PIN);
Ma3Encoder12b joint3Encoder(ENCODER3_READING_PIN);
Ma3Encoder12b joint4Encoder(ENCODER4_READING_PIN);
Ma3Encoder12b joint5Encoder(ENCODER5_READING_PIN);

GenPwmPhaseHBridge dev1(MOT1_PWN_PIN, HBRIDGE1_PHASE_PIN, HBRIDGE1_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev2(MOT2_PWN_PIN, HBRIDGE2_PHASE_PIN, HBRIDGE2_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev3(MOT3_PWN_PIN, HBRIDGE3_PHASE_PIN, HBRIDGE3_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev4(MOT4_PWN_PIN, HBRIDGE4_PHASE_PIN, HBRIDGE4_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev5(MOT5_PWN_PIN, HBRIDGE5_PHASE_PIN, HBRIDGE5_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge gripMotorDev(GRIPMOT_PWM_PIN, GRIPMOT_PHASE_PIN, GRIPMOT_NENABLE_PIN, false, true);
RCContinuousServo gripServoDev(GRIPPER_SERVO_PWM_PIN, false);

SingleMotorJoint gripperMotor(spd, &gripMotorDev);
SingleMotorJoint gripperServo(spd, &gripServoDev);

RotateJoint joint1Open(spd, &dev1, &dev2);
TiltJoint joint2Open(spd, &dev1, &dev2);
SingleMotorJoint joint3Open(spd, &dev3);
RotateJoint joint4Open(spd, &dev4, &dev5);
TiltJoint joint5Open(spd, &dev4, &dev5);

RotateJoint joint1Closed(pos, &joint1Alg, &dev1, &dev2, &joint1Encoder);
TiltJoint joint2Closed(pos, &joint2Alg, &dev1, &dev2, &joint2Encoder);
SingleMotorJoint joint3Closed(pos, &joint3Alg, &dev3, &joint3Encoder);
RotateJoint joint4Closed(pos, &joint4Alg, &dev4, &dev5, &joint4Encoder);
TiltJoint joint5Closed(pos, &joint5Alg, &dev4, &dev5, &joint5Encoder);

//variables used to control joints during closed loop control
unsigned long joint1Destination;
unsigned long joint2Destination;
unsigned long joint3Destination;
unsigned long joint4Destination;
unsigned long joint5Destination;

ControlSystems currentControlSystem; //tracks what control system arm is currently using
bool mainPowerOn;
bool m1On;
bool m2On;
bool m3On;
bool m4On;
bool m5On;
bool gripMotOn;
bool initialized = false;

void setup() {} //useless

/*main loop
after initialization, function has three responsibilities it juggles. 
 1) handle messages from base station and carry orders out. 
      a) if messages don't come in for a certain amount of time, assume line is disconnected or that the driver is no longer using the arm 
      and stop the arm from moving until more commands come in
 2) protect the arm from overcurrenting, by checking for an overcurrent condition and handling it by killing power and reporting the error to base station
 3) protect the individual motors from overcurrenting, by checking for motor fault conditions and handling it by killing power to those motors and reporting the error
*/
void loop() {

  initialize();
  delay(100);

  //switchToClosedLoop(); //for debugging. RED currnetly lacks command to switch between schemes, has to be done manually
  while(1) //main loop begin
  {
    processBaseStationCommands();

    //armOvercurrentHandling(); arm current sensing is currently buggy

    motorFaultHandling();
  }

}

//Listens for base station commands, and if any are detected carry out arm duties based off of them. 
//If none are received for a set period if time, the arm will stop movement and disable the main power line, as the assumption
//is that the line was disconnected or the user is no longer moving the arm, and we don't want the arm to keep going and kill itself against a rock or the rover; 
//the CFO does not want to pay for another arm.
//The time it takes for the arm to 'timeout' is tracked by the global constant WATCHDOG_TIMEOUT_US, in microseconds
void processBaseStationCommands()
{
  CommandResult result;
  uint16_t commandId = 0;
  size_t commandSize = 0;
  char commandData[255]; 
  static uint32_t watchdogTimer_us = 0; //increment this value everytime we don't get a command. When we've waited for a command for longer than our timeout value, stop all arm movement

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
        if(currentControlSystem != OpenLoop)
        {
          switchToOpenLoop();
        }
        result = moveJ1(*(int16_t*)(commandData));
        break;

      case ArmJ2:
      case LY_ArmJ2:
        if(currentControlSystem != OpenLoop)
        {
          switchToOpenLoop();
        }
        result = moveJ2(*(int16_t*)(commandData));
        break;

     case ArmJ3:
     case LY_ArmJ3:
        if(currentControlSystem != OpenLoop)
        {
          switchToOpenLoop();
        }
        result = moveJ3(*(int16_t*)(commandData));
        break;

      case ArmJ4: 
      case LY_ArmJ4: 
        if(currentControlSystem != OpenLoop)
        {
          switchToOpenLoop();
        }
        result = moveJ4(*(int16_t*)(commandData));
        break;

      case ArmJ5: 
      case LY_ArmJ5: 
        if(currentControlSystem != OpenLoop)
        {
          switchToOpenLoop();
        }
        result = moveJ5(*(int16_t*)(commandData));
        break;

      case MoveGripper: 
      case LY_MoveGripper: //gripper only ever operates in open loop but the rest of the system can be using other controls at the same time
        result = moveGripper(*(int16_t*)(commandData));
        break;

      case MoveGripServo: //gripper only ever operates in open loop but the rest of the system can be using other controls at the same time
        result = moveGripServo(*(int16_t*)(commandData));
        break;

      case ArmEnableAll: 
        masterPowerSet((*(bool*)(commandData)));
        allMotorsPowerSet(*(bool*)(commandData));
        break;

      case ArmEnableMain: 
        masterPowerSet(*(bool*)(commandData));
        break;

      case ArmAbsoluteAngle:
        if(currentControlSystem != ClosedLoop)
        {
          switchToClosedLoop();
        }
        setArmDestinationAngles(((float*)(commandData)));
        break;

      case ArmAbsoluteXYZ:
        if(currentControlSystem != ClosedLoop)
        {
          switchToClosedLoop();
        }
        float absoluteAngles[ArmJointCount];
        computeIK((float*)(commandData), absoluteAngles);
        setArmDestinationAngles(absoluteAngles);

      case ArmGetPosition: 
        float currentPositions[ArmJointCount]; //empty array to fill, as this command expects an entire array of positions
        getArmPositions(currentPositions);
        roveComm_SendMsg(ArmCurrentPosition, sizeof(float) * ArmJointCount, currentPositions);
        break;

      case ArmEnableJ1:
        j12PowerSet(*(bool*)commandData); //joint 1 and joint 2 are linked together
        break;

      case ArmEnableJ2:
        j12PowerSet(*(bool*)commandData); //joint 1 and joint 2 are linked together
        break;

      case ArmEnableJ3:
        j3PowerSet(*(bool*)commandData); //joint 3 is on its own
        break;

      case ArmEnableJ4:
        j45PowerSet(*(bool*)commandData); //joint 4 and joint 5 are linked together
        break;

      case ArmEnableJ5:
        j45PowerSet(*(bool*)commandData); //joint 4 and joint 5 are linked together
        break;

      case ArmEnableEndeff:
        gripperMotorPowerSet(*(bool*)commandData);
        break;

      case ArmEnableServo:
        gripperServoPowerSet(*(bool*)commandData);
        break;

      case ArmCurrentMain:
        float *armCurrent;
        *armCurrent = readMasterCurrent();
        roveComm_SendMsg(ArmCurrentPosition, sizeof(float), armCurrent);
        break;
        
      default:
        break; //do nothing if it's not a known ID
        
    } //end switch

    if(result != Success)
    {
      //todo: if there's ever any telemetry about what to do when the command isn't successful, this is where we'll send telemetry back about it
    }
  }//end if(commandId != 0)

  //if no messages were received, increment our watchdog counter. If the counter has gone over a certain period of time since we last got a transmission, cease all movement.
  //Exception is if we're in closed loop, in which case it might be normal for the arm to not get commands for long periods of time
  else if(currentControlSystem != ClosedLoop)
  {
    uint8_t microsecondDelay = 1;
    delayMicroseconds(microsecondDelay);

    watchdogTimer_us += microsecondDelay;

    if(watchdogTimer_us >= WATCHDOG_TIMEOUT_US) //if more than our timeout period has passed, then kill arm movement
    {
      Serial.println("Timeout");
      stopArm();
      watchdogTimer_us = 0;
    }
  }//end else
}

//Checks each motor for fault conditions.
//If fault conditions are found, the joint attached to the motor is disabled and an error message
//is sent to base station
void motorFaultHandling()
{
  if(digitalRead(HBRIDGE1_NFAULT_PIN) == LOW && mainPowerOn && m1On) //pins are always low when main power is off
  {
    j12PowerSet(false); //motors 1 and 2 are a part of joints 1 and 2, which are interlinked together
    roveComm_SendMsg(ArmFault, 1, (void*)ArmFault_m1);
  }
  if(digitalRead(HBRIDGE2_NFAULT_PIN) == LOW && mainPowerOn && m2On)
  {
    j12PowerSet(false); //motors 1 and 2 are a part of joints 1 and 2, which are interlinked together
    roveComm_SendMsg(ArmFault, 1, (void*)ArmFault_m2);
  }
  if(digitalRead(HBRIDGE3_NFAULT_PIN) == LOW && mainPowerOn && m3On)
  {
    j3PowerSet(false);
    roveComm_SendMsg(ArmFault, 1, (void*)ArmFault_m3);
  }
  if(digitalRead(HBRIDGE4_NFAULT_PIN) == LOW && mainPowerOn && m4On)
  {
    j45PowerSet(false); //motors 4 and 5 are a part of joints 4 and 5, which are interlinked together
    roveComm_SendMsg(ArmFault, 1, (void*)ArmFault_m4);
  }
  if(digitalRead(HBRIDGE5_NFAULT_PIN) == LOW && mainPowerOn && m5On)
  {
    j45PowerSet(false); //motors 4 and 5 are a part of joints 4 and 5, which are interlinked together
    roveComm_SendMsg(ArmFault, 1, (void*)ArmFault_m5);
  }
  if(digitalRead(GRIPMOT_NFAULT_PIN) == LOW && gripMotOn)
  {
    gripperMotorPowerSet(false);
    roveComm_SendMsg(ArmFault, 1, (void*)ArmFault_gripper);
  }
}

//Checks to see if there's an overcurrent condition. If there is, disable main power and report error to base station. 
//User must manually re-enable power
void armOvercurrentHandling()
{
  if(readMasterCurrent() > CURRENT_LIMIT)
  { 
    masterPowerSet(false);
    roveComm_SendMsg(ArmFault, 1, (void*)ArmFault_overcurrent);
  }
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

  pinMode(POWER_LINE_CONTROL_PIN,OUTPUT);

  joint1Open.coupleJoint(&joint2Open);
  joint4Open.coupleJoint(&joint5Open);

  joint1Closed.coupleJoint(&joint2Closed);
  joint4Closed.coupleJoint(&joint5Closed);
  
  //initialze to open loop control format
  switchToOpenLoop();

  masterPowerSet(false);

  allMotorsPowerSet(true);
  
  //set timer 0 to fire at a rate where the different PI algorithms will all be updated at their expected timeslice in seconds. 
  //There are 5 controls to update independently. They update one at a time, one being serviced every time the timer fires. So it takes 5 timer
  //firings for any individual control to get updated again. Meaning the timeslice of the timer itself must be one fifth of the PI algorithms overall timeslice so that 
  //when it cycles back around the overall timeslice will have passed
  setupTimer0((PI_TIMESLICE_SECONDS/5.0) * 1000000.0); //function expects microseconds

  joint3Alg.setDeadband(ElbowDeadband);
  joint3Alg.setHardStopPositions(ElbowHardStopUp, ElbowHardStopDown);
  
  joint3Encoder.setOffsetAngle(ElbowOffsetAngle);
  
  initialized = true;
}

//Turns on or off the main power line
CommandResult masterPowerSet(bool enable)
{
  Serial.print("Enable master power: ");
  Serial.println(enable);
  mainPowerOn = enable;
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

  m1On = powerOn;
  m2On = powerOn;
}

//turns on or off the motor attached to joint 3
void j3PowerSet(bool powerOn)
{
  dev3.setPower(powerOn);

  m3On = powerOn;
}

//turns on or off the motors attached to joint 4 and 5
void j45PowerSet(bool powerOn)
{
  dev4.setPower(powerOn);
  dev5.setPower(powerOn);

  m4On = powerOn;
  m5On = powerOn;
}

//turns on or off the motor attached to the gripper
void gripperMotorPowerSet(bool powerOn)
{
  gripMotorDev.setPower(powerOn);

  gripMotOn = powerOn;
}

//turns on or off the servo attached to the gripper
void gripperServoPowerSet(bool powerOn)
{
  gripServoDev.setPower(powerOn);
}

//Sets the angles for the joints of the arm to travel to
//Input: an angle array. angles[0] = joint1 destination, etc. Joints are described in floats from 0 to 360 degrees
//Note that this will only be performed when the current control system being used is closed loop
CommandResult setArmDestinationAngles(float* angles)
{ 
  //angles comes in as an array
  joint1Destination = angles[0] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0)); //convert from 0-360 float to framework's POSITION_MIN - POSITION_MAX long
  joint2Destination = angles[1] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
  joint3Destination = angles[2] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
  joint4Destination = angles[3] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
  joint5Destination = angles[4] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
}

//moves the first joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ1(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    if(moveValue > 0)
      moveValue = BaseMaxSpeed;
    else if(moveValue < 0)
      moveValue = -BaseMaxSpeed; //adjusting for base station 
    joint1Open.runOutputControl(moveValue);
  }
}

//moves the second joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ2(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    if(moveValue > 0)
      moveValue = BaseMaxSpeed;
    else if(moveValue < 0)
      moveValue = -BaseMaxSpeed; //adjusting for base station scaling
      
    joint2Open.runOutputControl(moveValue);
  }
}

//moves the third joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ3(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    joint3Open.runOutputControl(moveValue);
  }
}

//moves the fourth joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ4(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {
    joint4Open.runOutputControl(moveValue);
  }
}

//moves the fifth joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ5(int16_t moveValue)
{
  if(currentControlSystem == OpenLoop)
  {   
    joint5Open.runOutputControl(moveValue);
  }
}

//moves the gripper open/closed
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveGripper(int16_t moveValue)
{
  if(moveValue > 0)
    moveValue = 1000;
  else if(moveValue < 0)
    moveValue = -1000;
    
  gripperMotor.runOutputControl(moveValue);
}

//spins the gripper servo
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveGripServo(int16_t moveValue)
{
  gripperServo.runOutputControl(moveValue);
}

//switches the arm over to open loop control method; this will disable closed loop functions and functionality
//while enabling open loop functions and functionality
CommandResult switchToOpenLoop()
{ 
  if(initialized)
  {
    //disable closed loop interrupts before doing any operation to preserve thread safety
    TimerDisable(TIMER0_BASE, TIMER_A); 
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //have arm soft reset when switching between control schemes if power's on and we're potentially moving
    if(mainPowerOn)
    {
      masterPowerSet(false);
      delay(1000);
      masterPowerSet(true);
    }
  }
  
  currentControlSystem = OpenLoop;

  dev3.setRampUp(ElbowRampUp);
  dev3.setRampDown(ElbowRampDown);
  dev1.setRampUp(BaseRampUp);
  dev2.setRampUp(BaseRampUp);
  dev1.setRampDown(BaseRampDown);
  dev2.setRampDown(BaseRampDown);
}

//switches the arm over to closed loop control method; this will enable closed loop functions and functionality
//while disabling open loop functions and functionality
CommandResult switchToClosedLoop()
{
  currentControlSystem = ClosedLoop;
    
  //have default position destination values be the joints' current positions, so they hold still when switchover occurs until base station sends a new position to go towards
  joint1Destination = joint1Encoder.getFeedback();
  joint2Destination = joint2Encoder.getFeedback();
  joint3Destination = joint3Encoder.getFeedback();
  joint4Destination = joint4Encoder.getFeedback();
  joint5Destination = joint5Encoder.getFeedback();

  if(initialized)
  {
    //have arm soft reset when switching between control schemes if power's on and we're potentially moving
    if(mainPowerOn)
    {
      masterPowerSet(false);
      delay(1000);
      masterPowerSet(true);
    }
  
    //enable closed loop interrupts, which will begin to move the arm towards its set destinations
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
  }
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
//Angles are numerically described as 0-360 degrees
CommandResult getArmPositions(float positions[ArmJointCount])
{
  positions[0] = joint1Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN))); //getFeedback returns from POS_MAX to POS_MIN long, convert it to 0-360 degrees float
  positions[1] = joint2Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN)));
  positions[2] = joint3Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN)));
  positions[3] = joint4Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN)));
  positions[4] = joint5Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN)));
}

void computeIK(float coordinates[IKArgCount], float angles[ArmJointCount])
{
  float temp;
  float temp2;
  float tempNum;
  float tempDen;
  float cX = coordinates[0];
  float cY = coordinates[1];
  float cZ = coordinates[2];
  float gripperAngle = coordinates[3];

  float joint1Angle;
  float joint2Angle;
  float joint3Angle;
  float joint4Angle;
  float joint5Angle;
  float d = sqrt((cX*cX) + (cY*cY));
  constrain(d, 0, (ElbowLength + WristLength));
  float w = sqrt(d*d + (cZ-BaseLength)*(cZ-BaseLength));
  constrain(w, (ElbowLength-WristLength), (ElbowLength + WristLength));
  
  joint1Angle = negativeDegreeCorrection(atan2(cY, cX)); //joints will start being expressed in radians

  temp = ElbowLength*ElbowLength + w*w - WristLength*WristLength; //elbow length = R2, base length = R1, Wrist length = R3
  temp /= (2*ElbowLength*w);
  temp = acos(temp);
  temp2 = negativeDegreeCorrection(atan2(cZ-BaseLength, d));

  joint2Angle = temp + temp2;

  temp = ElbowLength*ElbowLength + WristLength*WristLength - w*w;
  temp /= (2*ElbowLength*WristLength);
  joint3Angle = acos(temp);

  joint5Angle = negativeDegreeCorrection(M_PI -joint3Angle - joint2Angle + gripperAngle); //M_PI given in math.h

  joint4Angle = joint4Encoder.getFeedback(); //joint4 not actually controlled in IK

  angles[0] = joint1Angle;
  angles[1] = joint2Angle;
  angles[2] = joint3Angle;
  angles[3] = joint4Angle;
  angles[4] = joint5Angle;

  //convert angles from 0-2pi to 0-360
  int i;
  for(i = 0; i < 5; i++)
  {
    angles[i] = angles[i] * 360.0 / (2.0*M_PI);
  }

  //make sure values are constrained
  for(i = 0; i < 5; i++)
  {
    while(angles[i] < 0)
    {
      angles[i] += 360;
    }
    while(angles[i] > 360)
    {
      angles[i] -= 360;
    }
  }
}

//converts 0 to -2pi, to 0 to 2pi
float negativeDegreeCorrection(float correctThis)
{
  while(correctThis < 0)
  {
    correctThis += 2.0*M_PI; 
  }                        

  return(correctThis);
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
    joint1Closed.runOutputControl(joint1Destination);
  }
  else if(jointUpdated == 2)
  {
    joint2Closed.runOutputControl(joint2Destination);
  }
  else if(jointUpdated == 3)
  {
    joint3Closed.runOutputControl(joint3Destination);
  }
  else if(jointUpdated == 4)
  {
    joint4Closed.runOutputControl(joint4Destination);
  }
  else if(jointUpdated == 5)
  {
    joint5Closed.runOutputControl(joint5Destination);
  }
}

