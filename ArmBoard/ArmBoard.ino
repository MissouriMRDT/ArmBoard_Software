#include "ArmBoard.h"
Servo Servo4;
Servo Servo1;
Servo Servo2;
Servo Servo3;
uint32_t currentPositions[6] = {0};
uint32_t bicepAngleVals[4] = {0};
uint32_t forearmAngleVals[2] = {0};


void setup()
{
  Serial.begin(115200);
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET);

  pinMode(LASER_CNTRL_PIN, OUTPUT);

  //This servo (4) is attached to a random pin, as this is the only way to get the actual
  //servos to work
  Servo4.attach(PM_0);
  Servo1.attach(SERVO_1_CRTL_PIN);
  Servo2.attach(SERVO_2_CRTL_PIN);
  Servo3.attach(SERVO_3_CRTL_PIN);

  Servo1.write(SERVO_1_RETRACTED); 
  Servo2.write(SERVO_2_RETRACTED);
  Servo3.write(SERVO_3_RETRACTED);
}


void loop()
{
 rovecomm_packet = RoveComm.read();
 if(rovecomm_packet.data_id != 0) 
  //Serial.println(rovecomm_packet.data_id);
 switch(rovecomm_packet.data_id)
 {
  case RC_ARMBOARD_MOVEOPENLOOP_DATAID:
    doOpenLoop();
    break;
  case RC_ARMBOARD_TOOLSELECTION_DATAID:
    toolSelection();
    break;
  case RC_ARMBOARD_LASER_DATAID:
    Serial.println("Laser");
    if(rovecomm_packet.data[0] == 1)
      digitalWrite(LASER_CNTRL_PIN, HIGH);
    else
      digitalWrite(LASER_CNTRL_PIN, LOW);
    break;
  case RC_ARMBOARD_ARMCOMMANDS_DATAID:
    parseCommand();
    break;
  case RC_ARMBOARD_BICEP_MOTORANGLES_DATAID:
  case RC_ARMBOARD_FOREARM_MOTORANGLES_DATAID:
    updatePosition();
    break;
  case RC_ARMBOARD_MOVETOANGLE_DATAID:
    doClosedLoop();
    break;
  case RC_ARMBOARD_IKINCROV_DATAID:
    Serial.println("IK INCREMENT");
    initPresentCoordinates();
    int16_t moveCommands[6];
    for(int i = 0; i<6;i++)
    {
      moveCommands[i] = (int16_t)rovecomm_packet.data[i];
    }
    incrementRoverIK(moveCommands);
    uint32_t bicepMove[4];
    bicepMove[0] = bicepAngleVals[0]; //J1
    bicepMove[1] = invertAngle(bicepAngleVals[1],true); //J2
    bicepMove[2] = bicepAngleVals[2]; //J3
    bicepMove[3] = invertAngle(bicepAngleVals[3],true); //J4
    RoveComm.writeTo(RC_ARMBOARD_BICEP_ANGLE_DATAID, 4, bicepMove, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
    uint32_t forearmMove[4];
    forearmMove[0] = forearmAngleVals[0]; //J5
    forearmMove[1] = forearmAngleVals[1]; //J6
    RoveComm.writeTo(RC_ARMBOARD_FOREARM_ANGLE_DATAID, 2, forearmMove, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
    RoveComm.writeTo(RC_ARMBOARD_GRIPPER_DATAID, 1, rovecomm_packet.data[6], 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
    RoveComm.writeTo(RC_ARMBOARD_SOLENOID_DATAID, 1, rovecomm_packet.data[7], 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
    break;
  default:
    break;
 }
 //sendPosition();
}

void doOpenLoop()
{
   int16_t bicepVals[4];
   int16_t forearmVals[3];
   Serial.println("Open Loop");
   Serial.println(rovecomm_packet.data_id);
   Serial.print("1:");Serial.println(rovecomm_packet.data[0]);
   Serial.print("2:");Serial.println(rovecomm_packet.data[1]);
   Serial.print("3:");Serial.println(rovecomm_packet.data[2]);
   Serial.print("4:");Serial.println(rovecomm_packet.data[3]);
   Serial.print("5:");Serial.println(rovecomm_packet.data[4]);
   Serial.print("6:");Serial.println(rovecomm_packet.data[5]);

   bicepVals[0] = rovecomm_packet.data[0]; //J1
   bicepVals[1] = rovecomm_packet.data[1]; //J2
   bicepVals[2] = rovecomm_packet.data[2]; //J3
   bicepVals[3] = rovecomm_packet.data[3]; //J4
   forearmVals[0] = rovecomm_packet.data[4]; //J5
   forearmVals[1] = rovecomm_packet.data[5]; //J6
   forearmVals[2] = rovecomm_packet.data[6]; //Gripper
   forearmVals[3] = rovecomm_packet.data[7]; //Nipper
   //sending the motor commands to their specific boards
   RoveComm.writeTo(RC_ARMBOARD_BICEP_DATAID, 4, bicepVals, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
   RoveComm.writeTo(RC_ARMBOARD_FOREARM_DATAID, 4, forearmVals, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
}

void doClosedLoop()
{
   Serial.println(rovecomm_packet.data[5]);
   bicepAngleVals[0] = rovecomm_packet.data[0]; //J1
   bicepAngleVals[1] = invertAngle(rovecomm_packet.data[1],true); //J2
   bicepAngleVals[2] = rovecomm_packet.data[2]; //J3
   bicepAngleVals[3] = invertAngle(rovecomm_packet.data[3],true); //J4
   forearmAngleVals[0] = rovecomm_packet.data[4]; //J5
   forearmAngleVals[1] = rovecomm_packet.data[5]; //J6

   RoveComm.writeTo(RC_ARMBOARD_BICEP_ANGLE_DATAID, 4, bicepAngleVals, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
   RoveComm.writeTo(RC_ARMBOARD_FOREARM_ANGLE_DATAID, 2, forearmAngleVals, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
}

void toolSelection()
{
 //we write the tool id to forearm so that forearm can keep track of whether the solenoid can be enabled or not
 RoveComm.writeTo(RC_ARMBOARD_TOOLSELECTION_DATAID, 1, rovecomm_packet.data[0], 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
 if(rovecomm_packet.data[0] == 0)
 {
      //Typing tool selected
      Servo1.write(SERVO_1_SELECTED);
      Servo2.write(SERVO_2_RETRACTED);
      Servo3.write(SERVO_3_RETRACTED);
 }
 else if(rovecomm_packet.data[0] == 1)
 {
      //Hex tool selected
      Servo1.write(SERVO_1_RETRACTED);
      Servo2.write(SERVO_2_SELECTED);
      Servo3.write(SERVO_3_RETRACTED);
 }
 else if(rovecomm_packet.data[0] == 2)
 {
      //Screwdriver tool selected
      Servo1.write(SERVO_1_RETRACTED);
      Servo2.write(SERVO_2_RETRACTED);
      Servo3.write(SERVO_3_SELECTED);
 }
}
uint32_t timer;
void sendPosition()
{
  if (timer > millis())
   {
    timer = millis();
   }

   if (millis() - timer > 500) 
   {
    timer = millis(); 
    RoveComm.write(RC_ARMBOARD_MOTORANGLES_DATAID, 6, currentPositions);  
   }
}

void parseCommand()
{
  Serial.println("Parsing");
  if(rovecomm_packet.data[RC_ARMBOARD_POSITION_GETENTRY] == 1)
  {
    Serial.println("Joint 1 Angle:");
    Serial.println(currentPositions[0]);
    Serial.println("Joint 2 Angle:");
    Serial.println(currentPositions[1]);
    Serial.println("Joint 3 Angle:");
    Serial.println(currentPositions[2]);
    Serial.println("Joint 4 Angle:");
    Serial.println(currentPositions[3]);
    Serial.println("Joint 5 Angle:");
    Serial.println(currentPositions[4]);
    Serial.println("Joint 6 Angle:");
    Serial.println(currentPositions[5]);
    Serial.println("sending");
    RoveComm.write(RC_ARMBOARD_MOTORANGLES_DATAID, 6, currentPositions);
  }
  else if(rovecomm_packet.data[RC_ARMBOARD_ARMCOMMANDS_SWAPP_GRIPPERENTRY] == 1)
  {
    //definitely do something, but we don't have the second gripper so nope for now
  }
}

void updatePosition()
{
  if(rovecomm_packet.data_id == RC_ARMBOARD_BICEP_MOTORANGLES_DATAID)
  {
    currentPositions[0] = rovecomm_packet.data[0];
    currentPositions[1] = invertAngle(rovecomm_packet.data[1],true);
    currentPositions[2] = rovecomm_packet.data[2];
    currentPositions[3] = invertAngle(rovecomm_packet.data[3],true);
  }
  else if(rovecomm_packet.data_id == RC_ARMBOARD_FOREARM_MOTORANGLES_DATAID)
  {
    currentPositions[4] = rovecomm_packet.data[0];
    currentPositions[5] = rovecomm_packet.data[1];
  }
}

uint32_t invertAngle(uint32_t angle, bool invert)
{
  if      (invert)  
  {
     angle = 360000 - angle;
  }

  return angle;

}
