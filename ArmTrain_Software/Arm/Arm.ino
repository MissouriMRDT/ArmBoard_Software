#include "Arm.h"

void setup()
{
    Serial.begin(115200);
    delay(10);
    Serial.println("Arm loop begin:");
    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET);

    stop(); //set all to 0 and clear watchdog





    pinMode(J1LS_1, INPUT);
    pinMode(J1LS_2, INPUT);

    pinMode(J2LS_1, INPUT); 
    pinMode(J2LS_2, INPUT);

    pinMode(J3LS_1, INPUT);
    pinMode(J3LS_2, INPUT);

    pinMode(SW_IND_1, OUTPUT);
    //laser and sol might not need to be OUTPUT
    pinmode(LASER, OUTPUT);
    pinmode(SOL, OUTPUT);

    //current sensors analog read these pins in RoveStmVnhPwm
    pinMode(CS1, INPUT);
    pinMode(CS2, INPUT);
    pinMode(CS3, INPUT);
    pinMode(CS4, INPUT);
    pinMode(CS5, INPUT);
    pinMode(CS6, INPUT);
    pinMode(CSGR, INPUT);

    //                 A in   B in   PWM   invert bus mV scalemV cs  maxAmp     --- 12V max = scale_to_millivolts / bus_millivolts => 100.0% scale
    J1.motor1.attach( J1INA, J1INB, J1PWM, false, 12000, 12000, CS1, 6500); //6.5amp stall current of motors
    J2.motor1.attach( J2INA, J2INB, J2PWM, false, 12000, 12000, CS2, 6500);
    J3.motor1.attach( J3INA, J3INB, J3PWM, false, 12000, 12000, CS3, 6500);
    J4.motor1.attach( J4INA, J4INB, J4PWM, false, 12000, 12000, CS4, 6500);
    J5.motor1.attach( J5INA, J5INB, J5PWM, false, 12000, 12000, CS5, 6500);
    J6.motor1.attach( J6INA, J6INB, J6PWM, false, 12000, 12000, CS6, 6500);
    GRIP.motor1.attach( GRINA, GRINB, GRPWM, false, 12000, 12000, CSGR, 6500);

    //                   pin    priority    autocalibrate   offset  invert  read@0  read@360  
    J1.encoder_1.attach( PM_1, 7, false, 0, false, 0, 1000);
    J2.encoder_1.attach( PM_2, 7, false, 0, false, 0, 1000);
    J3.encoder_1.attach( PH_0, 7, false, 0, false, 0, 1000);
    J4.encoder_1.attach( PH_1, 7, false, 0, false, 0, 1000);
    J5.encoder_1.attach( PK_6, 7, false, 0, false, 0, 1000);
    J6.encoder_1.attach( PK_7, 7, false, 0, false, 0, 1000);

}

uint32_t timer = millis();

void loop()
{
    //1take in data
    //2updatePosition();
    //3closedloop

    rovecomm_packet = RoveComm.read();
    if(rovecomm_packet.data_id != 0);
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
    //case RC_ARMBOARD_BICEP_MOTORANGLES_DATAID:
    case RC_ARMBOARD_FOREARM_MOTORANGLES_DATAID: //change to arm motorangles
        updatePosition();
        break;
    case RC_ARMBOARD_MOVETOANGLE_DATAID:
        doClosedLoop();
        break;
    //case RC_ARMBOARD_IKINCROV_DATAID:
    //   Serial.println("IK INCREMENT");
    //   initPresentCoordinates();
    //   int16_t moveCommands[6];
    //   for(int i = 0; i<6;i++)
    //   {
    //     moveCommands[i] = (int16_t)rovecomm_packet.data[i];
    //   }
    //   incrementRoverIK(moveCommands);
    //   uint32_t bicepMove[4];
    //   bicepMove[0] = bicepAngleVals[0]; //J1
    //   bicepMove[1] = invertAngle(bicepAngleVals[1],true); //J2
    //   bicepMove[2] = bicepAngleVals[2]; //J3
    //   bicepMove[3] = invertAngle(bicepAngleVals[3],true); //J4
    //   RoveComm.writeTo(RC_ARMBOARD_BICEP_ANGLE_DATAID, 4, bicepMove, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
    //   uint32_t forearmMove[4];
    //   forearmMove[0] = forearmAngleVals[0]; //J5
    //   forearmMove[1] = forearmAngleVals[1]; //J6
    //   RoveComm.writeTo(RC_ARMBOARD_FOREARM_ANGLE_DATAID, 2, forearmMove, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
    //   RoveComm.writeTo(RC_ARMBOARD_GRIPPER_DATAID, 1, rovecomm_packet.data[6], 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
    //   RoveComm.writeTo(RC_ARMBOARD_SOLENOID_DATAID, 1, rovecomm_packet.data[7], 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
    //   break;
    case RC_ARMBOARD_DOLS_DATAID:
        Serial.println("DoLS");
        Serial.println(rovecomm_packet.data[0]);
        RoveComm.writeTo(RC_ARMBOARD_DOLS_DATAID, 1, rovecomm_packet.data[0], 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
        break;
    default:
        break;
    }

}

void doOpenLoop()
{
    
                             //j1 j2 j3 j4 j5 j6
    uint32_t jointAngles[6]; //0, 1, 2, 3, 4, 5
   
    Serial.println("Open Loop");
    Serial.println(rovecomm_packet.data_id);
    Serial.print("1:");Serial.println(rovecomm_packet.data[0]);
    Serial.print("2:");Serial.println(rovecomm_packet.data[1]);
    Serial.print("3:");Serial.println(rovecomm_packet.data[2]);
    Serial.print("4:");Serial.println(rovecomm_packet.data[3]);
    Serial.print("5:");Serial.println(rovecomm_packet.data[4]);
    Serial.print("6:");Serial.println(rovecomm_packet.data[5]);

    jointAngles[0] = rovecomm_packet.data[0]; //J1
    jointAngles[1] = rovecomm_packet.data[1]; //J2
    jointAngles[2] = rovecomm_packet.data[2]; //J3
    jointAngles[3] = rovecomm_packet.data[3]; //J4
    jointAngles[4] = rovecomm_packet.data[4]; //J5
    jointAngles[5] = rovecomm_packet.data[5]; //J6
    jointAngles[6] = rovecomm_packet.data[6]; //Gripper
    Serial.println(jointAngles[6]);
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

void doClosedLoop()
{
   Serial.println(rovecomm_packet.data[5]);
   jointAngles[0] = rovecomm_packet.data[0]; //J1
   jointAngles[1] = rovecomm_packet.data[1]; //J2
   jointAngles[2] = rovecomm_packet.data[2]; //J3
   jointAngles[3] = rovecomm_packet.data[3]; //J4
   jointAngles[4] = rovecomm_packet.data[4]; //J5
   jointAngles[5] = rovecomm_packet.data[5]; //J6

   RoveComm.writeTo(RC_ARMBOARD_BICEP_ANGLE_DATAID, 4, bicepAngleVals, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
   RoveComm.writeTo(RC_ARMBOARD_FOREARM_ANGLE_DATAID, 2, forearmAngleVals, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
}

void stop()
{
    J1.motor_1.DriveMotor(0);
    J2.motor_1.DriveMotor(0);
    J3.motor_1.DriveMotor(0);
    J4.motor_1.DriveMotor(0);
    J5.motor_1.DriveMotor(0);
    J6.motor_1.DriveMotor(0);
    Watchdog.clear();
}

void updatePosition()
{
    jointAngles[0] = J1.encoder_1.readMillidegrees();
    jointAngles[2] = J2.encoder_1.readMillidegrees();
    jointAngles[4] = J3.encoder_1.readMillidegrees();
    jointAngles[6] = J4.encoder_1.readMillidegrees();
    jointAngles[8] = J5.encoder_1.readMillidegrees();
    jointAngles[10] = J6.encoder_1.readMillidegrees();

    if (timer > millis())
    {
    timer = millis();
    }

    if (millis() - timer > 100) 
    {
    timer = millis(); 
    RoveComm.writeTo(RC_ARMBOARD_BICEP_MOTORANGLES_DATAID, 4, jointAngles, 192, 168, 1, RC_ARMBOARD_FOURTHOCTET, 11000);
    }
}