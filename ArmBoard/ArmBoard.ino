#include "ArmBoard.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET);
  Serial.println("Started");
  pinMode(SOLENOID_PIN, OUTPUT);

}


void loop()
{
 rovecomm_packet = RoveComm.read();
 Serial.println("Current id: ");
 Serial.println(rovecomm_packet.data_id);
 digitalWrite(SOLENOID_PIN, HIGH);
 if(rovecomm_packet.data_id == RC_ARMBOARD_MOVEOPENLOOP_DATAID)
 {
   int16_t bicepVals[4];
   int16_t forearmVals[3];

   Serial.println("Joint 1");
   Serial.println(rovecomm_packet.data[0]);
   Serial.println("Joint 2");
   Serial.println(rovecomm_packet.data[1]);
   Serial.println("Joint 3");
   Serial.println(rovecomm_packet.data[2]);
   Serial.println("Joint 4");
   Serial.println(rovecomm_packet.data[3]);
   Serial.println("Joint 5");
   Serial.println(rovecomm_packet.data[4]);
   Serial.println("Joint 6");
   Serial.println(rovecomm_packet.data[5]);
   Serial.println("Gripper");
   Serial.println(rovecomm_packet.data[6]);
   Serial.println("Nipper");
   Serial.println(rovecomm_packet.data[7]);


   bicepVals[0] = rovecomm_packet.data[0]; //J1
   bicepVals[1] = rovecomm_packet.data[1]; //J2
   bicepVals[2] = rovecomm_packet.data[2]; //J3
   bicepVals[3] = rovecomm_packet.data[3]; //J4
   forearmVals[0] = rovecomm_packet.data[4]; //J5
   forearmVals[1] = rovecomm_packet.data[5]; //J6
   forearmVals[2] = rovecomm_packet.data[6]; //J6

   //sending the motor commands to their specific boards
   RoveComm.writeTo(RC_ARMBOARD_BICEP_DATAID, 4, bicepVals, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
   RoveComm.writeTo(RC_ARMBOARD_FOREARM_DATAID, 3, forearmVals, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);

 }
}
