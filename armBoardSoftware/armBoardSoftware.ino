#include "armBoardSoftware.h"

/*Declare Arrays*/
int currentJointVals[6];
int futureJointVals[6];

void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting Communication");
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_ARMBOARD_PORT);

  /*Attatch Pins to Class Objects*/

  Bicep.attachJoint(BICEP_SERIAL, ENC_BICEP_TILT, ENC_BICEP_TWIST);
  Elbow.attachJoint(ELBOW_SERIAL, ENC_ELBOW_TILT, ENC_ELBOW_TWIST);
  Wrist.attachJoint(WRIST_SERIAL, ENC_WRIST_TILT, ENC_WRIST_TWIST);

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
 
  //Laser & Solenoid
  pinMode(LASER_ACTUATION, OUTPUT);
  pinMode(SOLENOID_ACTUATION, OUTPUT);
  
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
 rovecomm_packet = RoveComm.read();
 //Serial.println(rovecomm_packet.data_id);
 if(rovecomm_packet.data_id != 0) 
 {
  switch(rovecomm_packet.data_id)
    {
      case RC_ARMBOARD_MOVEOPENLOOP_DATAID:
        openLoopControl();
        break;
        
      case RC_ARMBOARD_SOLENOID_DATAID:
        //If we get a command to activate end effector, write to pin to actuate
        if(rovecomm_packet.data[0] == RC_ARMBOARD_SOLENOID_ENABLE) 
        {                                                       
          digitalWrite(SOLENOID_ACTUATION, HIGH);               
        }                                                       
          else if(rovecomm_packet.data[0] == RC_ARMBOARD_SOLENOID_DISABLE) 
        {
          digitalWrite(SOLENOID_ACTUATION, LOW);
        }
        break; 
        
      case RC_ARMBOARD_LASER_DATAID:
        //If we get a command to activate laser, write to pin to actuate
        if(rovecomm_packet.data[0] == RC_ARMBOARD_LASER_ENABLE) 
        {                                                       
          digitalWrite(LASER_ACTUATION, HIGH);               
        }                                                       
          else if(rovecomm_packet.data[0] == RC_ARMBOARD_LASER_DISABLE) 
        {
          digitalWrite(LASER_ACTUATION, LOW);
        }
        break;
      case RC_ARMBOARD_SET_CLOSED_LOOP_DATAID:
        uint8_t* state;
        state = (uint8_t*)rovecomm_packet.data;
        if(state[0])
        {
          Serial.println("Setting to closed loop");
          Bicep.Joint.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
          Bicep.Joint.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
          Elbow.Joint.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
          Elbow.Joint.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
          Wrist.Joint.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
          Wrist.Joint.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);

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
        }
        break;
      case RC_ARMBOARD_GRIPPER_DATAID:
        //Sets gripper motor to a speed between [-1000,1000]
        int16_t* gripperSpeed = (int16_t*)rovecomm_packet.data;
        Gripper.drive(gripperSpeed[0]);
        break;
    }
 }
}

void openLoopControl()
{
  int16_t* openLoopVelocityValues = (int16_t*)rovecomm_packet.data;
  //Check if input is above min, otherwise set motor speeds to 0
  for(int i = 0; i < 6; i++) 
  {
    if(abs(openLoopVelocityValues[i]) < MIN_SPEED) 
    {
      openLoopVelocityValues[i] = 0; 
    }
      Serial.print("Joint ");
      Serial.print(i);
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

  //
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
