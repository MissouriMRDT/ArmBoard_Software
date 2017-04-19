#include "arm.h"


JointInterface* joint1;
JointInterface* joint2;
JointInterface* joint3;
JointInterface* joint4;
JointInterface* joint5;
JointInterface* gripperMotor;

PIAlgorithm* alg1;
PIAlgorithm* alg2;
PIAlgorithm* alg3;
PIAlgorithm* alg4;
PIAlgorithm* alg5;
PIAlgorithm* algGripper;

Ma3Encoder12b* fb1;
Ma3Encoder12b* fb2;
Ma3Encoder12b* fb3;
Ma3Encoder12b* fb4;
Ma3Encoder12b* fb5;
Ma3Encoder12b* fbGripper;

GenPwmPhaseHBridge dev1(MOT1_PWN_PIN, HBRIDGE1_PHASE_PIN, HBRIDGE1_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev2(MOT2_PWN_PIN, HBRIDGE2_PHASE_PIN, HBRIDGE2_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev3(MOT3_PWN_PIN, HBRIDGE3_PHASE_PIN, HBRIDGE3_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev4(MOT4_PWN_PIN, HBRIDGE4_PHASE_PIN, HBRIDGE4_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev5(MOT5_PWN_PIN, HBRIDGE5_PHASE_PIN, HBRIDGE5_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev6(MOT6_PWM_PIN, HBRIDGE6_PHASE_PIN, HBRIDGE6_NSLEEP_PIN, true, false);


void setup() {} //useless

void loop() {
  CommandResult result;
  uint16_t commandId;
  size_t commandSize;
  int16_t commandData;
  uint32_t watchdogTimer_us = 0; //increment this value everytime we don't get a command. When we've waited for a command for longer than our timeout value, stop all arm movement

  initialize(); //control devices initted in here

  delay(10);
  
  //masterPowerEnable(); //for debugging. Enable if base station currently isn't sending 'enable power' messages and you just want to move some motors
  
  while(1) //main program loop. Listen for communications from the endefector or from base station, and proceed based on that transmission
  {
    commandSize = 0;
    commandId = 0;
    commandData = 0;//reset variables

    roveComm_GetMsg(&commandId, &commandSize, &commandData);
    if(commandId != 0) //command packets come in 1 or 2 bytes. If it's any other size, there was probably a comm error
    {
      watchdogTimer_us = 0; //reset watchdog timer since we received a command

      if(commandId == ArmStop || commandId == LY_ArmStop)
      {
        result = stopArm();
      }
      else if(commandId == ArmJ1 || commandId == LY_ArmJ1)
      {
        result = moveJ1(commandData);
      }
      else if(commandId == ArmJ2 || commandId == LY_ArmJ2)
      {
        result = moveJ2(commandData);
      }
      else if(commandId == ArmJ3 || commandId == LY_ArmJ3)
      {
        result = moveJ3(commandData);
      }
      else if(commandId == ArmJ4 || commandId == LY_ArmJ4)
      {
        result = moveJ4(commandData);
      }
      else if(commandId == ArmJ5 || commandId == LY_ArmJ5)
      {
        result = moveJ5(commandData);
      }
      else if(commandId == MoveGripper || commandId == LY_MoveGripper)
      {
        result = moveGripper(commandData);
      }
      else if(commandId == TurnCap)
      {
        result = turnCap(commandData);
      }
      else if(commandId == OpenLoop)
      {
        result = switchToOpenLoop();
      }
      else if(commandId == ClosedLoop)
      {
        result = switchToClosedLoop();          
      }


      if(result != Success)
      {
        //todo: if there's ever any telemetry about what to do when the command isn't successful, this is where we'll send telemetry back about it
        // I'm not quite sure how to do this. Somebody else's addition would be very helpful.
      }
    }//end if

    //if no messages were recieved, increment our watchdog counter. If the counter has gone over a certain period of time since we last got a transmission, cease all movement.
    //This is to keep the arm from committing suicide on the environment/the rover if communications ever get interrupted while it's in the middle of moving
    else
    {
      uint8_t microsecondDelay = 10;
      delayMicroseconds(microsecondDelay);

      watchdogTimer_us += microsecondDelay;

      if(watchdogTimer_us >= WATCHDOG_TIMEOUT_US) //if more than our timeout period has passed, then kill arm movement
      {
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

  }//end while

}

void initialize()
{
  roveComm_Begin(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  Serial.begin(9600);
  Serial6.begin(GRIPPER_COMM_BAUD_RATE);

  pinMode(HBRIDGE1_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE2_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE3_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE4_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE5_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE6_NFAULT_PIN,INPUT);

  pinMode(OC_NFAULT_PIN,INPUT);
  pinMode(POWER_LINE_CONTROL_PIN,OUTPUT);

  joint1 = new RotateJoint(spd, &dev1, &dev2);
  joint2 = new TiltJoint(spd, &dev1, &dev2);
  joint3 = new SingleMotorJoint(spd, &dev3);
  joint4 = new RotateJoint(spd, &dev4, &dev5);
  joint5 = new TiltJoint(spd, &dev4, &dev5);
  gripperMotor = new SingleMotorJoint(spd, &dev6);

  joint1 -> coupleJoint(joint2);
  joint4 -> coupleJoint(joint5);

  masterPowerDisable();

  enableAllMotors();

  TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_PIOSC);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

  fb1 = new Ma3Encoder12b(ENCODER1_READING_PIN);
  fb2 = new Ma3Encoder12b(ENCODER2_READING_PIN);
  fb3 = new Ma3Encoder12b(ENCODER3_READING_PIN);
  fb4 = new Ma3Encoder12b(ENCODER4_READING_PIN);
  fb5 = new Ma3Encoder12b(ENCODER5_READING_PIN);
  fbGripper = new Ma3Encoder12b(ENCODERG_READING_PIN);
}

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

CommandResult masterPowerEnable()
{
  digitalWrite(POWER_LINE_CONTROL_PIN, HIGH);
}

CommandResult masterPowerDisable()
{
  digitalWrite(POWER_LINE_CONTROL_PIN, LOW);
}

void enableAllMotors()
{
  dev1.setPower(true);
  dev2.setPower(true);
  dev3.setPower(true);
  dev4.setPower(true);
  dev5.setPower(true);
  dev6.setPower(true);
}

void disableAllMotors()
{
  dev1.setPower(false);
  dev2.setPower(false);
  dev3.setPower(false);
  dev4.setPower(false);
  dev5.setPower(false);
  dev6.setPower(false);
}

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

CommandResult stopArm()
{
  joint1->runOutputControl(0);
  joint2->runOutputControl(0);
  joint3->runOutputControl(0);
  joint4->runOutputControl(0);
  joint5->runOutputControl(0);
  gripperMotor->runOutputControl(0);
}

CommandResult moveJ1(int16_t moveValue)
{
  joint1->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving j1: ");
    Serial.println(moveValue);
  }
}

CommandResult moveJ2(int16_t moveValue)
{
  joint2->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving j2 : ");
    Serial.println(moveValue);
  }
}

CommandResult moveJ3(int16_t moveValue)
{
  joint3->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving j3: ");
    Serial.println(moveValue);
  }
}

CommandResult moveJ4(int16_t moveValue)
{
  joint4->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving j4: ");
    Serial.println(moveValue);
  }
}

CommandResult moveJ5(int16_t moveValue)
{
  joint5->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving j5: ");
    Serial.println(moveValue);
  }
}

CommandResult moveGripper(int16_t moveValue)
{
  gripperMotor->runOutputControl(moveValue);
  if(moveValue != 0)
  {
    Serial.print("Moving j6: ");
    Serial.println(moveValue);
  }
}

CommandResult turnCap(int16_t moveValue)
{
  //
}

CommandResult switchToOpenLoop()
{
  
  delete joint1;
  delete joint2;
  delete joint3;
  delete joint4;
  delete joint5;
  delete gripperMotor;
  delete alg1;
  delete alg2;
  delete alg3;
  delete alg4;
  delete alg5;
  delete algGripper;
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerDisable(TIMER0_BASE, TIMER_A);
  joint1 = new RotateJoint(spd, &dev1, &dev2);
  joint2 = new TiltJoint(spd, &dev1, &dev2);
  joint3 = new SingleMotorJoint(spd, &dev3);
  joint4 = new RotateJoint(spd, &dev4, &dev5);
  joint5 = new TiltJoint(spd, &dev4, &dev5);
  gripperMotor = new SingleMotorJoint(spd, &dev6);
}

CommandResult switchToClosedLoop()
{
  delete joint1;
  delete joint2;
  delete joint3;
  delete joint4;
  delete joint5;
  delete gripperMotor;
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A);
  alg1 = new PIAlgorithm(21,4,.04);
  alg2 = new PIAlgorithm(21,4,.04);
  alg3 = new PIAlgorithm(21,4,.04);
  alg4 = new PIAlgorithm(21,4,.04);
  alg5 = new PIAlgorithm(21,4,.04);
  algGripper = new PIAlgorithm(21,4,.04);
  joint1 = new RotateJoint(spd, alg1, &dev1, &dev2, fb1);
  joint2 = new TiltJoint(spd, alg2, &dev1, &dev2, fb2);
  joint3 = new SingleMotorJoint(spd, alg3, &dev3, fb3);
  joint4 = new RotateJoint(spd, alg4, &dev4, &dev5, fb4);
  joint5 = new TiltJoint(spd, alg5, &dev4, &dev5, fb5);
  gripperMotor = new SingleMotorJoint(spd, algGripper, &dev6, fbGripper);
}


