#include "ArmBicep.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_BICEP_FOURTHOCTET);

  Shoulder.LeftMotor.attach(SHOULDER_LEFT_INA, SHOULDER_LEFT_INB, SHOULDER_LEFT_PWM);
  Shoulder.RightMotor.attach(SHOULDER_RIGHT_INA, SHOULDER_RIGHT_INB, SHOULDER_RIGHT_PWM);
  Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
  Elbow.RightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);

  Shoulder.TiltEncoder.attach(SHOULDER_TILT_ENCODER);
  Shoulder.TwistEncoder.attach(SHOULDER_TWIST_ENCODER);

  Elbow.TiltEncoder.attach(ELBOW_TILT_ENCODER);
  Elbow.TwistEncoder.attach(ELBOW_TWIST_ENCODER);

  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Shoulder.TiltEncoder.start();
  Shoulder.TwistEncoder.start();

  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Elbow.TiltEncoder.start();
  Elbow.TwistEncoder.start();

  Watchdog.attach(estop);
  Watchdog.start(1000);

}

uint32_t timer = millis();

void loop()
{
  rovecomm_packet = RoveComm.read();
  switch(rovecomm_packet.data_id)
  {
    case RC_ARMBOARD_BICEP_DATAID:
      doOpenLoop();
      break;
    default:
      break;
  }
  jointAngles[0] = Shoulder.TwistEncoder.readMillidegrees();
  jointAngles[1] = Shoulder.TiltEncoder.readMillidegrees();
  jointAngles[2] = Elbow.TiltEncoder.readMillidegrees();
  jointAngles[3] = Elbow.TwistEncoder.readMillidegrees();

  if (timer > millis())
  {
    timer = millis();
  }//end if

  if (millis() - timer > 100) 
  {
    timer = millis(); 
    readAngles();
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

void readAngles()
{
  Serial.println("Joints");
  Serial.println(jointAngles[0]);
  Serial.println(jointAngles[1]);
  Serial.println(jointAngles[2]);
  Serial.println(jointAngles[3]);
  RoveComm.writeTo(RC_ARMBOARD_BICEP_MOTORANGLES_DATAID, 4, jointAngles, 192, 168, 1, RC_ARMBOARD_FOURTHOCTET, 11000);
  Watchdog.clear();
}

void doOpenLoop()
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


