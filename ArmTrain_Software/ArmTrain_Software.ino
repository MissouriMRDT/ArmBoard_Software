#include "ArmTrain_Software.h"

/*Declare Arrays*/
int currentJointVals[6];
int futureJointVals[6];

void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting Communication");
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);

  /*Attatch Pins to Class Objects*/

  //Joint Serial, Encoders, and PID 
  Bicep.attachJoint(BICEP_SERIAL, ENC_BICEP_TILT, ENC_BICEP_TWIST,
                    MIN_BICEP_OUTPUT_TILT, MAX_BICEP_OUTPUT_TILT, 
                    BICEP_TILT_KP, BICEP_TILT_KI, BICEP_TILT_KD,
                    MIN_BICEP_OUTPUT_TWIST, MAX_BICEP_OUTPUT_TWIST,
                    BICEP_TWIST_KP, BICEP_TWIST_KI, BICEP_TWIST_KD
                   );
  Elbow.attachJoint(ELBOW_SERIAL, ENC_ELBOW_TILT, ENC_ELBOW_TWIST,
                    MIN_ELBOW_OUTPUT_TILT, MAX_ELBOW_OUTPUT_TILT,
                    ELBOW_TILT_KP, ELBOW_TILT_KI, ELBOW_TILT_KD,
                    MIN_ELBOW_OUTPUT_TWIST, MAX_ELBOW_OUTPUT_TWIST,
                    ELBOW_TWIST_KP, ELBOW_TWIST_KI, ELBOW_TWIST_KD
                   );
  Wrist.attachJoint(WRIST_SERIAL, ENC_WRIST_TILT, ENC_WRIST_TWIST,
                    MIN_WRIST_OUTPUT_TILT, MAX_WRIST_OUTPUT_TILT,
                    WRIST_TILT_KP, WRIST_TILT_KI, WRIST_TILT_KD,
                    MIN_WRIST_OUTPUT_TWIST, MAX_WRIST_OUTPUT_TWIST,
                    WRIST_TWIST_KP, WRIST_TWIST_KI, WRIST_TWIST_KD
                   );
                   
  //Joint Limit Switches
  Bicep.attachLimitSwitches(LS_UPPER_BICEP, LS_LOWER_BICEP);  
  Elbow.attachLimitSwitches(LS_UPPER_ELBOW, LS_LOWER_ELBOW);  

  //Gripper Motor
  Gripper.attach(GRIPPER_INA, GRIPPER_INB, GRIPPER_PWM);
  
  /*Set I/O Pins*/
  
  //Limit Switches
  pinMode(LS_UPPER_BICEP, INPUT);
  pinMode(LS_LOWER_BICEP, INPUT);
  pinMode(LS_UPPER_ELBOW, INPUT);
  pinMode(LS_LOWER_ELBOW, INPUT);
  
  //Software Indicators
  pinMode(ERROR_LED, OUTPUT);
  pinMode(SW1_LED, OUTPUT);
  pinMode(SW2_LED, OUTPUT);

  /*Start Watchdog*/
  Watchdog.attach(Estop);
  Watchdog.start(500, DISABLE_BOARD_RESET);
}

void loop() 
{
 //Check to see if there is a RoveComm packet
 //If so, check what kind and execute command 
 packet = RoveComm.read();
 //Serial.println(packet.data_id);
 if(packet.data_id != 0) 
 {
  switch(packet.data_id)
    {
      case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
        //Control arm with raw velocity values
        openLoopControl();
        break;
      case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID:
        //Control the arm with raw angle values
        closedLoopControl();
        break;
      case RC_ARMBOARD_SETCLOSEDLOOPSTATE_DATA_ID:
        //Allow the Odrives to be moved/ must be used before any other joint commands
        setClosedLoop();
        break;
      case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
        //Sets gripper motor to a speed between [-1000,1000]
        int16_t* gripperSpeed = (int16_t*)packet.data;
        Gripper.drive(gripperSpeed[0]);
        break;
    }
  }
}

void setClosedLoop() 
{
  //If we set motors to closed loop control, SW1 LED turns on
  uint8_t* state;
  state = (uint8_t*)packet.data;
  if(state[0])
  {
    Serial.println("Setting to closed loop");
    Bicep.Joint.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Bicep.Joint.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Elbow.Joint.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Elbow.Joint.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Wrist.Joint.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    Wrist.Joint.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    digitalWrite(SW1_LED, HIGH);
    Serial.println("Set to closed loop");
  }
  else
  {
    Serial.println("Setting to idle state");
    Bicep.Joint.left.writeState(AXIS_STATE_IDLE);
    Bicep.Joint.right.writeState(AXIS_STATE_IDLE);
    Elbow.Joint.left.writeState(AXIS_STATE_IDLE);
    Elbow.Joint.right.writeState(AXIS_STATE_IDLE);
    Wrist.Joint.left.writeState(AXIS_STATE_IDLE);
    Wrist.Joint.right.writeState(AXIS_STATE_IDLE);
    digitalWrite(SW1_LED, LOW);
    Serial.println("Set to idle loop");
  }
}

void getPosition() 
{
  
}

void openLoopControl()
{
  int16_t* openLoopVelocityValues = (int16_t*)packet.data;
  //Check if input is above min, otherwise set motor speeds to 0
  for(int i = 0; i < 6; i++) 
  {
    if(abs(openLoopVelocityValues[i]) < MIN_SPEED) 
    {
      openLoopVelocityValues[i] = 0; 
    }
      Serial.print("Joint ");
      Serial.print(i+1);
      Serial.print(" value: ");
      Serial.println(openLoopVelocityValues[i]);
  }
  //run Joints at given velocity
  Bicep.tiltTwistDecipercent(openLoopVelocityValues[0], openLoopVelocityValues[1]);
  Elbow.tiltTwistDecipercent(openLoopVelocityValues[2], openLoopVelocityValues[3]);
  Wrist.tiltTwistDecipercent(openLoopVelocityValues[4], openLoopVelocityValues[5]);
  
  //update the ODrive watchdogs
  Bicep.Joint.left.updateWatchdog();
  Bicep.Joint.right.updateWatchdog();
  Elbow.Joint.left.updateWatchdog(); 
  Elbow.Joint.right.updateWatchdog(); 
  Wrist.Joint.left.updateWatchdog(); 
  Wrist.Joint.right.updateWatchdog();

  //Clearing the internal watchdog 
  Watchdog.clear();
  Serial.println("Wrote speeds");
}

void closedLoopControl() 
{
  float bicepTiltTwistOutput[2] = {};
  float elbowTiltTwistOutput[2] = {};
  float wristTiltTwistOutput[2] = {};

  //Print out our angle values
  float* closedLoopAngleValues = (float*)packet.data;
  Serial.println("Angle Values:");
  for(int i = 0; i < 6; i++) 
  {
    Serial.print("Joint ");
    Serial.print(i+1);
    Serial.print(" angle: ");
    Serial.println(closedLoopAngleValues[i]);
  }

  //This is for figuring out the velocity values
  Bicep.moveToPos(closedLoopAngleValues[0],closedLoopAngleValues[1],bicepTiltTwistOutput);
  Elbow.moveToPos(closedLoopAngleValues[2],closedLoopAngleValues[3],elbowTiltTwistOutput);
  Wrist.moveToPos(closedLoopAngleValues[4],closedLoopAngleValues[5],wristTiltTwistOutput);

  //If any of the joints aren't going to move, rehome them
  if( (bicepTiltTwistOutput[0] == 0) && (bicepTiltTwistOutput[1] == 0))
  {
    Bicep.rehomePosition();
  }
  if( (elbowTiltTwistOutput[0] == 0) && (elbowTiltTwistOutput[1] == 0))
  {
    Elbow.rehomePosition();
  }
  if( (wristTiltTwistOutput[0] == 0) && (wristTiltTwistOutput[1] == 0))
  {
    Wrist.rehomePosition();
  }

  //Move the joints based on PID control with given angle values
  Bicep.tiltTwistDecipercent(bicepTiltTwistOutput[0],bicepTiltTwistOutput[1]);
  Elbow.tiltTwistDecipercent(elbowTiltTwistOutput[0],elbowTiltTwistOutput[1]);
  Wrist.tiltTwistDecipercent(wristTiltTwistOutput[0],wristTiltTwistOutput[1]);

  //update the ODrive watchdogs
  Bicep.Joint.left.updateWatchdog();
  Bicep.Joint.right.updateWatchdog();
  Elbow.Joint.left.updateWatchdog(); 
  Elbow.Joint.right.updateWatchdog(); 
  Wrist.Joint.left.updateWatchdog(); 
  Wrist.Joint.right.updateWatchdog();

  //Clearing the internal watchdog 
  Watchdog.clear();
  Serial.println("Wrote speeds");

}

void Estop() 
{
  Bicep.tiltTwistDecipercent(0,0);
  Elbow.tiltTwistDecipercent(0,0);
  Wrist.tiltTwistDecipercent(0,0);
  Serial.println("Stopping Motors"); 
}