#include "ArmBicep.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_BICEP_FOURTHOCTET);
  Shoulder.LeftMotor.attach(SHOULDER_LEFT_INA, SHOULDER_LEFT_INB, SHOULDER_LEFT_PWM);
  Shoulder.RightMotor.attach(SHOULDER_RIGHT_INA, SHOULDER_RIGHT_INB, SHOULDER_RIGHT_PWM);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
  Elbow.RightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Watchdog.attach(estop);
  Watchdog.start(2000);
}

void loop()
{
  rovecomm_packet = RoveComm.read();
  if(rovecomm_packet.data_id == RC_ARMBOARD_BICEP_DATAID)
  {

    Serial.println(rovecomm_packet.data_id);
    Serial.println(rovecomm_packet.data[0]);
    Serial.println(rovecomm_packet.data[1]);
    Serial.println(rovecomm_packet.data[2]);
    Serial.println(rovecomm_packet.data[3]);


    if(abs(rovecomm_packet.data[0]) < 50 && abs(rovecomm_packet.data[1]) < 50 && abs(rovecomm_packet.data[3]) < 50 && abs(rovecomm_packet.data[4]) < 50)
    {
        estop();
    }
    if(rovecomm_packet.data[0] >= 70)
    {
        Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1])*2/3, (rovecomm_packet.data[0])*2/3);
    }
    else if(rovecomm_packet.data[0] <= -70)
    {
        Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1])/3, (rovecomm_packet.data[0])/3);
    }
    else if(rovecomm_packet.data[1] >= 70)
    {
      Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1])/2, (rovecomm_packet.data[0])/2, true);
    }
    else if(rovecomm_packet.data[1] <= -70)
    {
      Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1])/2, (rovecomm_packet.data[0])/2,false,true);
    }
    if(abs(rovecomm_packet.data[2]) >= 70 || abs(rovecomm_packet.data[3]) >= 70)
        Elbow.tiltTwistDecipercent((rovecomm_packet.data[2])/2, (rovecomm_packet.data[3])/2);

    Watchdog.clear();
  }
}

void estop()
{
  Serial.println("WE ESTOPPED");
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Watchdog.clear();
}