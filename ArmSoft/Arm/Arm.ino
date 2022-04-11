#include "Arm.h"


void setup()
{   
    Serial.begin(115200);
    Serial.println("Armboard setup");
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    digitalWrite(GREEN_LED, HIGH);
    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    
    pinMode(J1LS_1, INPUT); //Limit Switches set as input pins
    pinMode(J1LS_2, INPUT);
    pinMode(J2LS_1, INPUT); 
    pinMode(J2LS_2, INPUT);
    pinMode(J3LS_1, INPUT);
    pinMode(J3LS_2, INPUT);

    pinMode(SW_IND_1, OUTPUT);
    
    pinMode(LASER, OUTPUT); //laser and sol might not need to be OUTPUT
    pinMode(SOL, OUTPUT);

    pinMode(ENCJ1, INPUT);  //encoders
    pinMode(ENCJ2, INPUT);

    pinMode(CS1, INPUT); //current sensors analog read these pins in RoveStmVnhPwm
    pinMode(CS2, INPUT);
    pinMode(CS3, INPUT);
    pinMode(CS4, INPUT);
    pinMode(CS5, INPUT);
    pinMode(CS6, INPUT);
    pinMode(CSGR,INPUT);

    //A in   B in   PWM   invert bus mV scalemV cs  maxAmp     --- 12V max = scale_to_millivolts / bus_millivolts => 100.0% scale
    J1.motor_1.attach( J1INA, J1INB, J1PWM); //6.5amp stall current of motors
    J2.motor_1.attach( J2INA, J2INB, J2PWM);
    J3.motor_1.attach( J3INA, J3INB, J3PWM);
    //J4.motor_1.attach( J4INA, J4INB, J4PWM);
    J5.motor_1.attach( J5INA, J5INB, J5PWM);
    J6.motor_1.attach( J6INA, J6INB, J6PWM);
    GRIP.motor_1.attach( GRINA, GRINB, GRPWM);

    //                   pin    priority    autocalibrate   offset  invert  read@0  read@360  
    J1.encoder_1.attach(ENCJ1);
    J2.encoder_1.attach(ENCJ2);
    // J3.encoder_1.attach( PH_0, 7, false, 0, false, 0, 1000);
    // J4.encoder_1.attach( PH_1, 7, false, 0, false, 0, 1000);
    // J5.encoder_1.attach( PK_6, 7, false, 0, false, 0, 1000);
    // J6.encoder_1.attach( PK_7, 7, false, 0, false, 0, 1000);
 
    J1.attachLimitSwitches(J1LS_1, J1LS_2);
    J2.attachLimitSwitches(J2LS_1, J2LS_2);
    J3.attachLimitSwitches(J3LS_1, J3LS_2);

    
    J1.encoder_1.start();
    J2.encoder_1.start();
    //J3.encoder.start();
    //J4.encoder.start();
    //J5.encoder.start();
    //J6.encoder.start();

    Serial.println("watchdog setup start");
    Watchdog.attach(estop);
    WatchdogTelemetry.attach(telemetry);
    Watchdog.start(watchdogTimeout);
    WatchdogTelemetry.start(ROVECOMM_UPDATE_RATE);
    Serial.println("watchdog setup complete");

    estop();  //stops all motors and clears Watchdog
    delay(100);
    Serial.println("                       ^^:...                     ");
    Serial.println("                      JG?JYJ???77!~^:.            ");
    Serial.println("    Arm Time         ^B:.7^^?!!!!!!?JYYJ~         ");
    Serial.println("                     PY JJ ?^.^^:    .:?@~        ");
    Serial.println("  Has Begun         ^&7!B.7P P^ ~J  .: .@5        ");
    Serial.println("                     !&#P7#^7P  5#PJG~  J&:       ");
    Serial.println("                     .YJGB55G577JG!J! . .&?       ");
    Serial.println("                       .:  ..:^5@^ .. .. 5B       ");
    Serial.println("                               Y&.   ... ~@^      ");
    Serial.println("            .:^~~~^^:.        7@Y ...... .#5      ");
    Serial.println("         .~JPPGGGBGGGGPJ~   :G&7. ....... ?@^     ");
    Serial.println("       :YPY!:.. .....:~JGP^^#G:  ........ .#P     ");
    Serial.println("      ?B7.              .~Y&P  ........... 5@~    ");
    Serial.println("     ?#^  ..............  7@: ...........  J@Y    ");
    Serial.println("    :&7  ................ ?&. ............ ?@?    ");
    Serial.println("    :&~.:    .........    ^&^ .......... !^P@^    ");
    Serial.println("    7#::?Y7^.        ..^~: JY .......... !B@5     ");
    Serial.println("    Y#.  ^?5PP5YJJJY55Y?~.  !. ......... ~@&:     ");
    Serial.println("    ^@J     .^~7??7!^:.   .   .......... ~@?      ");
    Serial.println("     !&Y.                                Y#.      ");
    Serial.println("      ^G#Y~.         .~~!!~!JYJ7!~~~~?JPGG!       ");
    Serial.println("        !G&#PJ!~::..:^7YB@@##GYYY55555Y?^         ");
    Serial.println("          :!YGB##BBBGGPY?~:                       ");
    Serial.println("               ::^^^:                             ");
    
    delay(2000);
    Serial.println("3");
    digitalWrite(RED_LED, HIGH);   //3sec countdown
    delay(950);
    digitalWrite(RED_LED, LOW);
    delay(50);
    Serial.println("2");
    digitalWrite(RED_LED, HIGH);
    delay(950);
    digitalWrite(RED_LED, LOW);
    delay(50);
    Serial.println("1");
    digitalWrite(RED_LED, HIGH);
    delay(950); 
    digitalWrite(RED_LED, LOW);
    delay(50);
    Serial.println("GO");

    digitalWrite(GREEN_LED, LOW);
    
}

uint32_t timer = millis();
void loop()
{
    digitalWrite(RED_LED, HIGH);
    packet = RoveComm.read();
    //stop();
    // J1.DriveMotor(1000);
    // J2.DriveMotor(1000);
    // // fuckie J3.DriveMotor(1000);
    // J4.DriveMotor(1000);
    // J5.DriveMotor(1000);
    // // no work J6.DriveMotor(1000);
    //updatePosition();

    jointAngles[0] = J1.encoder_1.readDegrees();
    jointAngles[1] = J2.encoder_1.readDegrees();

    String ej1string = String(jointAngles[0]);
    String ej2string = String(jointAngles[1]);

    Serial.print("  J1 angle:" + ej1string);
    Serial.print("  J2 angle:" + ej2string);
    Serial.println("position updated");


    if(packet.data_id != 0)
    {
        //Serial.println(packet.data_id);
        switch(packet.data_id)
        {
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
            doOpenLoop();
            break;
        case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID: // recieve degrees, convert to physical to movement
            ClosedLoop();
            break;
        // case RC_ARMBOARD_ARMINCREMENTPOSITION_DATA_ID: //[J1, J2, J3, J4, J5, J6] (Degrees)
        //     break;
        // case RC_ARMBOARD_ARMMOVEIK_DATA_ID:             //[X, Y, Z, Y, P, R] (in)
        //     break;
        // case RC_ARMBOARD_ARMINCREMENTIKROVER_DATA_ID:   //[X, Y, Z, Y, P, R] (in)
        //     break;
        // case RC_ARMBOARD_ARMINCREMENTIKWRIST_DATA_ID:   //[X, Y, Z, Y, P, R] (in)
        //     break;
        case RC_ARMBOARD_LASERS_DATA_ID:       //[1-enable, 0-disable]
            Serial.println("Laser"); //laser is one val, probably works as is, do solinoid like this maybe
            if(packet.data[0] == 1)
                digitalWrite(LASER, HIGH);
            else
                digitalWrite(LASER, LOW);
            break;
        case RC_ARMBOARD_SOLENOID_DATA_ID:     //[1-enable, 0-disable]
            Serial.println("Solenoid"); //laser is one val, probably works as is, do solinoid like this maybe
            if(packet.data[0] == 1)
                digitalWrite(SOL, HIGH);
            else
                digitalWrite(SOL, LOW);
            break;
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID: //[Power] (-1000, 1000) (m%)
            GRIP.DriveMotor(packet.data[0]);
            Watchdog.clear();
            break;
        // case RC_ARMBOARD_MOTORCURRENTS_DATAID; // current sense telemetry
        //     motorCS[0] = analogRead(CS1);
        //     motorCS[1] = analogRead(CS2);
        //     motorCS[2] = analogRead(CS3);
        //     motorCS[3] = analogRead(CS4);
        //     motorCS[4] = analogRead(CS5);
        //     motorCS[5] = analogRead(CS6);
        //     motorCS[6] = analogRead(CSGR);
        //     motorCS[7] = 0);
        //     // RoveComm.writeTo(RC_ARMBOARD_MOTORCURRENTS_DATAID, RC_ARMBOARD_MOTORCURRENTS_DATACOUNT,
        //     //                 motorCS, RC_ROVECOMM_SUBNET_IP_FIRST_OCTET, RC_ROVECOMM_SUBNET_IP_SECOND_OCTET,
        //     //                 RC_ROVECOMM_SUBNET_IP_THIRD_OCTET, RC_MICROPIBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_UDP_PORT);
        //     Watchdog.clear();
        //     break;
            // case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID: //[0-Turn off Watchdog Override, 1-Turn on Watchdog Override]
            //     break;
            // case RC_ARMBOARD_DOLS_DATAID: //[Base Tilt Up, Base Tilt Down, Base Twist CW, Base Twist CCW, Elbow Tilt Up, Elbow Tilt Down, Elbow  Twist CW, Elbow  Twist CCW] (0-Turn off Limit Switch Override, 1-Turn on Limit Switch Override) (bitmasked)
            //     Serial.println("DoLS");
            //     Serial.println(packet.data[0]);
            //     //if true then overide to always have limitswitches as true
            //     //RoveComm.write(RC_ARMBOARD_ENCODERSTATUS_DATA_ID, RC_ARMBOARD_ENCODERSTATUS_DATA_COUNT, RC_ARMBOARD_ENCODERSTATUS_DATA_TYPE);
            //     break;
        case RC_ARMBOARD_REQUESTJOINTPOSITIONS_DATA_ID: //change to arm motorangles             //just 6 values of degrees of joints not x y z but degrees //Prompt arm for J1-6 positions
            updatePosition();
            break;
        // case RC_ARMBOARD_TOGGLEPOSITIONTELEM_DATA_ID:   //Start auto pushing arm J1-6 positions
        //     break;
        // case RC_ARMBOARD_REQUESTAXESPOSITIONS_DATA_ID:  //Prompt arm for XYZPYR Data
        //     break;
        //case RC_ARMBOARD_IKINCROV_DATAID:
        //   Serial.println("IK INCREMENT");
        //   initPresentCoordinates();
        //   int16_t moveCommands[6];
        //   for(int i = 0; i<6;i++)
        //   {
        //     moveCommands[i] = (int16_t)packet.data[i];
        //   }
        //   incrementRoverIK(moveCommands);
        //   uint32_t bicepMove[4];
        //   bicepMove[0] = bicepAngleVals[0]; //J1
        //   bicepMove[1] = invertAngle(bicepAngleVals[1],true); //J2
        //   bicepMove[2] = bicepAngleVals[2]; //J3
        //   bicepMove[3] = invertAngle(bicepAngleVals[3],true); //J4
        //   RoveComm.write(RC_ARMBOARD_BICEP_ANGLE_DATAID, 4, bicepMove, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
        //   uint32_t forearmMove[4];
        //   forearmMove[0] = forearmAngleVals[0]; //J5
        //   forearmMove[1] = forearmAngleVals[1]; //J6
        //   RoveComm.write(RC_ARMBOARD_FOREARM_ANGLE_DATAID, 2, forearmMove, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
        //   RoveComm.write(RC_ARMBOARD_GRIPPER_DATAID, 1, packet.data[6], 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
        //   RoveComm.write(RC_ARMBOARD_SOLENOID_DATAID, 1, packet.data[7], 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);
        //   break;
        default:
            break;
        }
    }
    else
    {
        //error packet
    }
    digitalWrite(RED_LED, LOW);
}

void estop() //estop includes watchdog clear
{
    J1.DriveMotor(0);
    J2.DriveMotor(0);
    J3.DriveMotor(0);
    J4.DriveMotor(0);
    Wrist.tiltTwistDrive(0,0);

    digitalWrite(LASER, LOW);
    digitalWrite(SOL, LOW);

    Watchdog.clear();  
}
void telemetry()
{
    return;
}

void doOpenLoop()
{
    
                               //j1 j2 j3 j4 j5 j6
    //uint32_t jointAngles[6]; //0, 1, 2, 3, 4, 5
    int16_t* motorSpeeds = (int16_t*)packet.data;
    Serial.println("Open Loop");
    Serial.println(packet.data_id);
    // Serial.print("1:");Serial.println(packet.data[0]);
    // Serial.print("2:");Serial.println(packet.data[1]);
    // Serial.print("3:");Serial.println(packet.data[2]);
    // Serial.print("4:");Serial.println(packet.data[3]);
    // Serial.print("5:");Serial.println(packet.data[4]);
    // Serial.print("6:");Serial.println(packet.data[5]);

    J1.DriveMotor(motorSpeeds[0]);
    J2.DriveMotor(motorSpeeds[1]);
    J3.DriveMotor(motorSpeeds[2]);
    J4.DriveMotor(motorSpeeds[3]);
    Wrist.tiltTwistDrive(motorSpeeds[4], motorSpeeds[5]);
    Watchdog.clear();
}

void toolSelection()
{
 //we write the tool id to forearm so that forearm can keep track of whether the solenoid can be enabled or not
 //RoveComm.write(RC_ARMBOARD_ENCODERSTATUS_DATA_ID, RC_ARMBOARD_ENCODERSTATUS_DATA_COUNT, (uint8_t)1);
 // used to be servo stuffs
}

void openLoop()
{
    Serial.print("1:");Serial.println(packet.data[0]);
    Serial.print("2:");Serial.println(packet.data[1]);
    Serial.print("3:");Serial.println(packet.data[2]);
    Serial.print("4:");Serial.println(packet.data[3]);
        

    if(abs(packet.data[0]) <= INPUT_DEADBAND ) packet.data[0] = 0;
    if(abs(packet.data[1]) <= INPUT_DEADBAND ) packet.data[1] = 0;
    if(abs(packet.data[2]) <= INPUT_DEADBAND ) packet.data[2] = 0;
    if(abs(packet.data[3]) <= INPUT_DEADBAND ) packet.data[3] = 0;


    J1.DriveMotor(packet.data[0]);
    J2.DriveMotor(packet.data[1]);
    J3.DriveMotor(packet.data[2]);
    J4.DriveMotor(packet.data[3]);
    J5.DriveMotor(packet.data[4]);
    J6.DriveMotor(packet.data[5]);

    Watchdog.clear();
}

void ClosedLoop()
{
    Serial.println(packet.data[5]);
    jointAngles[0] = packet.data[0]; //J1
    jointAngles[1] = packet.data[1]; //J2
    jointAngles[2] = packet.data[2]; //J3
    jointAngles[3] = packet.data[3]; //J4
    jointAngles[4] = packet.data[4]; //J5
    jointAngles[5] = packet.data[5]; //J6

    float newtargetAngles[6];
    float currentAngles[6];

    currentAngles[0] = jointAngles[0]; //store for new targets difference
    currentAngles[1] = jointAngles[1];
    currentAngles[2] = jointAngles[2];
    currentAngles[3] = jointAngles[3];
    currentAngles[4] = jointAngles[4];
    currentAngles[5] = jointAngles[5];
    currentAngles[6] = jointAngles[6];


    //parse packet
    //take array of target points
    //assign the points
    updatePosition();
    //movetoAngle( J1, , newtargetAngles, newtargetAngles[0]);    //?dont know what to put here yet? from bicep --->Shoulder, shoulderTiltTarget, shoulderTwistTarget, angles, outputs

    //do something with the output
    int J1tilt = 1;
    int J1twist = 1; //these exist so it compiles

    if(J1tilt < 0)
    {
        J1tilt = map(J1tilt, -700, 0, -900, -600);
    }
    else if(J1tilt > 0)
    {
        J1tilt = map(J1tilt, 0, 400, 250, 400);
    }
    if(J1twist < 0)
    {
        J1tilt = map(J1twist, -0, 600, 500, 700);
    }
    else if(J1twist > 0)
    {
        J1twist = map(J1twist, -600, 0, -700, -500);
    }

    Serial.println("J1 Tilt:");
    Serial.println(J1tilt);
    Serial.println("Twist:");
    Serial.println(J1twist);

}

void updatePosition()
{
    jointAngles[0] = J1.encoder_1.readDegrees();
    jointAngles[1] = J2.encoder_1.readDegrees();
    jointAngles[2] = J3.encoder_1.readDegrees();
    jointAngles[3] = J4.encoder_1.readDegrees(); //read degrees instead of milli
    jointAngles[4] = J5.encoder_1.readDegrees();
    jointAngles[5] = J6.encoder_1.readDegrees();

    //Serial.print("  J1 angle:" + jointAngles[0]);
    //Serial.print("  J2 angle:" + jointAngles[1]);
    // Serial.print("  J3 angle:" + jointAngles[2]);
    // Serial.print("  J4 angle:" + jointAngles[3]);
    // Serial.print("  J5 angle:" + jointAngles[4]);
    // Serial.print("  J6 angle:" + jointAngles[5]);

    // Serial.print("ENCJ1:" + analogRead(ENCJ1));
    // Serial.println();


    if (timer > millis())
    {
    timer = millis();
    }

    if ((millis() - timer) > 100) 
    {
    timer = millis(); 
    //RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles); //send as float
    }
}
