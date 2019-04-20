#include "ArmForearm.h"

void setup()
{
  //start our communications
  Serial.begin(9600);
  RoveComm.begin(RC_FOREARM_FOURTHOCTET);

  //initialize motors
  Wrist.LeftMotor.attach(WRIST_LEFT_INA, WRIST_LEFT_INB, WRIST_LEFT_PWM);
  Wrist.RightMotor.attach(WRIST_RIGHT_INA, WRIST_RIGHT_INB, WRIST_RIGHT_PWM);
  Gripper.attach(GRIPPER_INA, GRIPPER_INB, GRIPPER_PWM);
  //set motor speeds to 0, to be safe
  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);

  Wrist.TiltEncoder.attach(WRIST_TILT_ENCODER);
  Wrist.TiltEncoder.start();
  Wrist.TwistEncoder.attach(WRIST_TWIST_ENCODER);
  Wrist.TwistEncoder.start();

  Watchdog.attach(estop);
  Watchdog.start(1000);

}

uint32_t timer = millis();

void loop()
{
  rovecomm_packet = RoveComm.read();
  switch(rovecomm_packet.data_id)
  {
    case RC_ARMBOARD_FOREARM_DATAID:
      doOpenLoop();
      break;
    default:
      break;
  }
  jointAngles[0] = Wrist.TwistEncoder.readMillidegrees();
  jointAngles[1] = Wrist.TiltEncoder.readMillidegrees();

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

void readAngles()
{
  Serial.println("Joints");
  Serial.println(jointAngles[0]);
  Serial.println(jointAngles[1]);
  RoveComm.writeTo(RC_ARMBOARD_FOREARM_MOTORANGLES_DATAID, 2, jointAngles, 192, 168, 1, RC_ARMBOARD_FOURTHOCTET, 11000);
}

void doOpenLoop()
{
    if(abs(rovecomm_packet.data[0]) < 50 && abs(rovecomm_packet.data[1]) < 50 && abs(rovecomm_packet.data[3]) < 50 && abs(rovecomm_packet.data[4]) < 50)
    {
        estop();
    }
    //print telemetry
    Serial.print("Wrist tilt: ");
    Serial.println(rovecomm_packet.data[0]);
    Serial.print("Wrist twist: ");
    Serial.println(rovecomm_packet.data[1]);
    if(abs(rovecomm_packet.data[0]) >= 70 || abs(rovecomm_packet.data[1]) >= 70)
      Wrist.tiltTwistDecipercent((rovecomm_packet.data[0]*2), (rovecomm_packet.data[1]*2));

    Serial.print("Gripper value: ");
    Serial.println(rovecomm_packet.data[2]);
    Gripper.drive(rovecomm_packet.data[2]);
    Watchdog.clear();
}

void estop()
{
  Serial.println("WE ESTOPPED");
  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);
  Gripper.drive(0);
  Watchdog.clear();
}
