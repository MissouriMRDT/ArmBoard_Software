#include "armBoardSoftware.h"


/*Initialize Class Objects*/

//Rovecomm
RoveCommEthernetUdp RoveComm;
struct rovecomm_packet rovecomm_packet;

//Watchdog
//TODO: Analyse functionality of current watchdog. Do we want a hard board reset after several watchdog triggers?
RoveWatchdog Watchdog;

//ODrives
RoveDifferentialJointBrushless Bicep(BICEP_SERIAL, ENC_BICEP_TILT, ENC_BICEP_TWIST, BICEP_GR, MAX_SPEED_FORWARD, MAX_SPEED_REVERSE);
RoveDifferentialJointBrushless Elbow(ELBOW_SERIAL, ENC_ELBOW_TILT, ENC_ELBOW_TWIST, ELBOW_GR, MAX_SPEED_FORWARD, MAX_SPEED_REVERSE);
RoveDifferentialJointBrushless Wrist(WRIST_SERIAL, ENC_WRIST_TILT, ENC_WRIST_TWIST, WRIST_GR, MAX_SPEED_FORWARD, MAX_SPEED_REVERSE);

//Gripper
RoveStmVnhPwm Gripper;

/*Declare Arrays*/
int currentJointVals[6];
int futureJointVals[6];

void setup() 
{
  Serial.begin(115200);
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET);
  Serial.println("Starting Communication");

  /*Attatch Pins to Class Objects*/

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
  Watchdog.start(150);
}

void loop() 
{
 //Check to see if there is a RoveComm packet
 //If so, check what kind and execute command 
 rovecomm_packet = RoveComm.read();
 if(rovecomm_packet.data_id != 0) 
 {
  switch(rovecomm_packet.data_id)
    {
      case RC_ARMBOARD_MOVEOPENLOOP_DATAID:
        openLoopControl();
        Watchdog.clear();
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
  }
  //Run Joints at given velocity
  Serial.println("Open Loops Speeds:");
  for(int i = 0; i < 6; i++)
  {
    Serial.println(openLoopVelocityValues[i]); 
  }
  Bicep.tiltTwistDecipercent(openLoopVelocityValues[0], openLoopVelocityValues[1]);
  Elbow.tiltTwistDecipercent(openLoopVelocityValues[2], openLoopVelocityValues[3]);
  Wrist.tiltTwistDecipercent(openLoopVelocityValues[4], openLoopVelocityValues[5]);
}

void Estop() 
{
  Bicep.tiltTwistDecipercent(0,0);
  Elbow.tiltTwistDecipercent(0,0);
  Wrist.tiltTwistDecipercent(0,0); 
}
