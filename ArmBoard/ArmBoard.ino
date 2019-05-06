#include "ArmBoard.h"
Servo Servo4;
Servo Servo1;
Servo Servo2;
Servo Servo3;


void setup()
{
  Serial.begin(115200);
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET);

  pinMode(SOLENOID_CRTL_PIN, OUTPUT);

  //This servo (4) is attached to a random pin, as this is the only way to get the actual
  //servos to work
  //yaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaayyyyyyyyyyyyy
  Servo4.attach(PM_0);
  Servo1.attach(SERVO_1_CRTL_PIN);
  Servo2.attach(SERVO_2_CRTL_PIN);
  Servo3.attach(SERVO_3_CRTL_PIN);

  //Servo4.write(SERVO_1_RETRACTED);
  Servo1.write(SERVO_1_RETRACTED); //tried Servos[0].writeMicroseconds(1500); and still knothing on PM_7 pin.
  Servo2.write(SERVO_2_RETRACTED);
  Servo3.write(SERVO_3_RETRACTED);
}


void loop()
{
 rovecomm_packet = RoveComm.read();
 if(rovecomm_packet.data_id != 0) 
  Serial.println(rovecomm_packet.data_id);
 switch(rovecomm_packet.data_id)
 {
   case RC_ARMBOARD_MOVEOPENLOOP_DATAID:
    doOpenLoop();
    break;
   case RC_ARMBOARD_TOOLSELECTION_DATAID:
    toolSelection();
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
   default:
    break;
 }
 //sendPosition();
}

void doOpenLoop()
{
   int16_t bicepVals[4];
   int16_t forearmVals[3];
   Serial.println(rovecomm_packet.data_id);
   Serial.println(rovecomm_packet.data[0]);
   Serial.println(rovecomm_packet.data[1]);
   Serial.println(rovecomm_packet.data[2]);
   Serial.println(rovecomm_packet.data[3]);
   Serial.println(rovecomm_packet.data[4]);
   Serial.println(rovecomm_packet.data[5]);

   bicepVals[0] = rovecomm_packet.data[0]; //J1
   bicepVals[1] = rovecomm_packet.data[1]; //J2
   bicepVals[2] = rovecomm_packet.data[2]; //J3
   bicepVals[3] = rovecomm_packet.data[3]; //J4
   forearmVals[0] = rovecomm_packet.data[4]; //J5
   forearmVals[1] = rovecomm_packet.data[5]; //J6
   forearmVals[2] = rovecomm_packet.data[6]; //Gripper
  /*
  if(rovecomm_packet.data[7] > 0)
  {
    digitalWrite(SOLENOID_CRTL_PIN, HIGH); //Solenoid actuates
    digitalWrite(SW1_IND_PIN, HIGH);
    delay(250);
    digitalWrite(SOLENOID_CRTL_PIN, LOW); //Solenoid actuates
    digitalWrite(SW1_IND_PIN, LOW);
  }
  else
  {
    digitalWrite(SOLENOID_CRTL_PIN, LOW); //Solenoid actuates
    Serial.println("LOW");
  }*/
  

   //sending the motor commands to their specific boards
   RoveComm.writeTo(RC_ARMBOARD_BICEP_DATAID, 4, bicepVals, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
   RoveComm.writeTo(RC_ARMBOARD_FOREARM_DATAID, 3, forearmVals, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
}

void doClosedLoop()
{
   uint32_t bicepVals[4];
   uint32_t forearmVals[2];
   Serial.println(rovecomm_packet.data[5]);
   bicepVals[0] = rovecomm_packet.data[0]; //J1
   bicepVals[1] = rovecomm_packet.data[1]; //J2
   bicepVals[2] = rovecomm_packet.data[2]; //J3
   bicepVals[3] = rovecomm_packet.data[3]; //J4
   forearmVals[0] = rovecomm_packet.data[4]; //J5
   forearmVals[1] = rovecomm_packet.data[5]; //J6
   RoveComm.writeTo(RC_ARMBOARD_BICEP_ANGLE_DATAID, 4, bicepVals, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
   RoveComm.writeTo(RC_ARMBOARD_FOREARM_ANGLE_DATAID, 2, forearmVals, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
}

void toolSelection()
{
 if(rovecomm_packet.data[0] == 0)
 {
      //Typing tool selected
      Serial.println("Select servo1 tool");
      Servo1.write(SERVO_1_SELECTED);
      Servo2.write(SERVO_2_RETRACTED);
      Servo3.write(SERVO_3_RETRACTED);
 }
 else if(rovecomm_packet.data[0] == 1)
 {
      //Hex tool selected
      Serial.println("Select servo2 tool");
      Servo1.write(SERVO_1_RETRACTED);
      Servo2.write(SERVO_2_SELECTED);
      Servo3.write(SERVO_3_RETRACTED);
 }
 else if(rovecomm_packet.data[0] == 2)
 {
      //Screwdriver tool selected
      Serial.println("Select servo3 tool");
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
    currentPositions[1] = rovecomm_packet.data[1];
    currentPositions[2] = rovecomm_packet.data[2];
    currentPositions[3] = rovecomm_packet.data[3];
  }
  else if(rovecomm_packet.data_id == RC_ARMBOARD_FOREARM_MOTORANGLES_DATAID)
  {
    currentPositions[4] = rovecomm_packet.data[0];
    currentPositions[5] = rovecomm_packet.data[1];
  }
}
