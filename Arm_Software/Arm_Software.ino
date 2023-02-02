#include "Arm_Software.h"

using namespace std;

void setup()
{
    Serial.begin(115200);
    ShoulderTilt.motor.attach(MotorINA_1, MotorINB_1, MotorPWM_1);
    ShoulderTwist.motor.attach(MotorINA_2, MotorINB_2, MotorPWM_2);
    ElbowTilt.motor.attach(MotorINA_3, MotorINB_3, MotorPWM_3);
    ElbowTwist.motor.attach(MotorINA_4, MotorINB_4, MotorPWM_4);
    Wrist.rightMotor.attach(MotorINA_5, MotorINB_5, MotorPWM_5);
    Wrist.leftMotor.attach(MotorINA_6, MotorINB_6, MotorPWM_6);
    Gripper.attach(MotorINA_7, MotorINB_7, MotorPWM_7);

    ShoulderTilt.encoder.attach(Encoder_ShoulderTilt, 7, false, 0);
    ShoulderTwist.encoder.attach(Encoder_ShoulderTwist, 7, false, 0);
    ElbowTilt.encoder.attach(Encoder_ElbowTilt, 7, false, 0);   // -40000
    ElbowTwist.encoder.attach(Encoder_ElbowTwist, 7, false, 0);    // -295000
    Wrist.tiltEncoder.attach(Encoder_WristTilt, 7, false, 0);      // -180000
    Wrist.twistEncoder.attach(Encoder_WristTwist, 7, false, 0);    // -180000

    ShoulderTilt.attachLimitSwitches(LimitSwitchLower_J1, LimitSwitchLower_J1);
    ShoulderTwist.attachLimitSwitches(LimitSwitchLower_J2, LimitSwitchLower_J2);
    ElbowTilt.attachLimitSwitches(LimitSwitchLower_J3, LimitSwitchLower_J3);

    ShoulderTilt.setAngleLimits(180, 45);
    ShoulderTwist.setAngleLimits(360, 0);
    ElbowTilt.setAngleLimits(200, 130);
    ElbowTwist.setAngleLimits(360, 0);

    ShoulderTilt.encoder.start();
    ShoulderTwist.encoder.start();
    ElbowTilt.encoder.start();
    ElbowTwist.encoder.start();
    Wrist.tiltEncoder.start();
    Wrist.twistEncoder.start();

    ElbowTilt.pid.attach(-1000.0, 1000.0, 150.0, 0, 0);
    ElbowTwist.pid.attach(-500.0, 500.0, 10.0, 0, 0);

    ShoulderTilt.pid.attach(-1000.0, 500.0, 100.0, 0, 0);
    ShoulderTwist.pid.attach(-1000.0, 1000.0, 20.0, 0, 0);

    Wrist.tiltPid.attach(-1000.0, 1000.0, 11.0, 0, 0);
    Wrist.twistPid.attach(-1000.0, 1000.0, 11.0, 0, 0);

    pinMode(LaserToggle, OUTPUT);
    pinMode(SolenoidToggle, OUTPUT);

    pinMode(LimitSwitchLower_J1, INPUT);
    pinMode(LimitSwitchUpper_J1, INPUT);
    pinMode(LimitSwitchLower_J2, INPUT);
    pinMode(LimitSwitchUpper_J2, INPUT);
    pinMode(LimitSwitchLower_J3, INPUT);
    pinMode(LimitSwitchUpper_J3, INPUT);

    digitalWrite(LaserToggle, HIGH);
    digitalWrite(SolenoidToggle, LOW);

    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);

    Watchdog.attach(estop);
    Watchdog.start(WatchdogTimeout);
}

void loop()
{
    parsePackets();
    if (closedloopActive) closedLoop();

    if((millis() - timer) >= ROVECOMM_UPDATE_RATE*4) {
        updatePosition();
        RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
        Serial.println(jointAngles[0]);
        //updateCoordinates();
        //RoveComm.write(RC_ARMBOARD_IKCOORDINATES_DATA_ID, RC_ARMBOARD_IKCOORDINATES_DATA_COUNT, coordinates);
        timer = millis();
    }  
}

void parsePackets()
{
    packet = RoveComm.read();

    switch (packet.data_id)
    {
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
            openLoop();
            Watchdog.clear();
            break;

        case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID:
            float* motorAngles;
            motorAngles = (float*)packet.data;

            jointTargets[0] = motorAngles[0]; 
            jointTargets[1] = motorAngles[1];
            jointTargets[2] = motorAngles[2];
            jointTargets[3] = motorAngles[3];
            jointTargets[4] = motorAngles[4];
            jointTargets[5] = motorAngles[5];
            closedloopActive = true;
            Watchdog.clear();
            break;

        case RC_ARMBOARD_ARMMOVEIK_DATA_ID:
            float* dest;
            dest = (float*)packet.data;
            dest[3] = radians(dest[3]);
            dest[4] = radians(dest[4]);
            dest[5] = radians(dest[5]);
            
            float angles[6];
            angles[0] = radians(jointAngles[0]);
            angles[1] = radians(jointAngles[1]);
            angles[2] = radians(jointAngles[2]);
            angles[3] = radians(jointAngles[3]);
            angles[4] = radians(jointAngles[4]);
            angles[5] = radians(jointAngles[5]);
            
            float IKTargets[6];
            bool valid;
            valid = inverseKinematics(dest, angles, IKTargets);

            if(valid) {
                jointTargets[0] = IKTargets[0];
                jointTargets[1] = IKTargets[1];
                jointTargets[2] = IKTargets[2];
                jointTargets[3] = degrees(IKTargets[3]);
                jointTargets[4] = degrees(IKTargets[4]);
                jointTargets[5] = degrees(IKTargets[5]);
                closedloopActive = true;
                Watchdog.clear();
            }
            break;
        
        case RC_ARMBOARD_LASERS_DATA_ID:
            if ((uint8_t)packet.data[0]) {
                digitalWrite(LaserToggle, HIGH);
            }
            else {
                digitalWrite(LaserToggle, LOW);
            }
            break;

        case RC_ARMBOARD_SOLENOID_DATA_ID:
            if ((uint8_t)packet.data[0]) {
                digitalWrite(SolenoidToggle, HIGH);
            }
            else {
                digitalWrite(SolenoidToggle, LOW);
            }
            break;

        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
            int16_t* gripperSpeed;
            gripperSpeed = (int16_t*)packet.data;
            Gripper.drive(gripperSpeed[0]);
            Watchdog.clear();
            break;

        case RC_ARMBOARD_REQUESTJOINTPOSITIONS_DATA_ID:
            RoveComm.writeReliable(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
            break;

        case RC_ARMBOARD_REQUESTAXESPOSITIONS_DATA_ID:
            RoveComm.writeReliable(RC_ARMBOARD_IKCOORDINATES_DATA_ID, RC_ARMBOARD_IKCOORDINATES_DATA_COUNT, coordinates);
            break;

        default:
            break;
    }   
}

void openLoop()
{
    closedloopActive = false;
    int16_t* motorSpeeds = (int16_t*)packet.data;

    if (!ShoulderTilt.atHardLimit(motorSpeeds[0])) ShoulderTilt.moveJoint(motorSpeeds[0]);
    if (!ShoulderTwist.atHardLimit(motorSpeeds[1])) ShoulderTwist.moveJoint(motorSpeeds[1]);
    if (!ElbowTilt.atHardLimit(motorSpeeds[2])) ElbowTilt.moveJoint(motorSpeeds[2]);

    ElbowTwist.moveJoint(motorSpeeds[3]);
    Wrist.moveDiffJoint(motorSpeeds[4], motorSpeeds[5]);
}

void updatePosition()
{
    jointAngles[0] = ShoulderTilt.encoder.readDegrees();
    jointAngles[1] = ShoulderTwist.encoder.readDegrees();
    jointAngles[2] = ElbowTilt.encoder.readDegrees();
    jointAngles[3] = ElbowTwist.encoder.readDegrees();
    jointAngles[4] = Wrist.tiltEncoder.readDegrees();
    jointAngles[5] = Wrist.twistEncoder.readDegrees();
}

void updateCoordinates()
{
    forwardKinematics(jointAngles, coordinates);
    coordinates[3] = degrees(coordinates[3]);
    coordinates[4] = degrees(coordinates[4]);
    coordinates[5] = degrees(coordinates[5]);
}

void closedLoop()
{
    float angles[2];
    float targets[2];
    float outputs[2];
    updatePosition();

    // Shoulder Tilt
    if (ShoulderTilt.atSoftLimit(jointAngles[0], jointTargets[0])) {
        outputs[0] = 0;
    }
    else {
        moveToAngle(ShoulderTilt, jointTargets[0], jointAngles[0], outputs[0]);
        if(ShoulderTilt.atHardLimit(outputs[0])) outputs[0] = 0;
    }
    ShoulderTilt.moveJoint(outputs[0]);

    // Shoulder Twist
    if (ShoulderTwist.atSoftLimit(jointAngles[1], jointTargets[1])) {
        outputs[0] = 0;
    }
    else {
        moveToAngle(ShoulderTwist, jointTargets[1], jointAngles[1], outputs[0]);
        if (ShoulderTwist.atHardLimit(outputs[0])) outputs[0] = 0;
    }
    ShoulderTwist.moveJoint(outputs[0]);
    
    // Elbow Tilt
    if (ElbowTilt.atSoftLimit(jointAngles[2], jointTargets[2])) {
        outputs[0] = 0;
    }
    else {
        moveToAngle(ElbowTilt, jointTargets[2], jointAngles[2], outputs[0]);
        if(ElbowTilt.atHardLimit(outputs[0])) outputs[0] = 0;
    }
    ElbowTilt.moveJoint(outputs[0]);

    // Elbow Twist
    if (ElbowTwist.atSoftLimit(jointAngles[3], jointTargets[3])) {
        outputs[0] = 0;
    }
    else {
        moveToAngle(ElbowTwist, jointTargets[3], jointAngles[3], outputs[0]);
        // No hard limit
    }
    ElbowTwist.moveJoint(outputs[0]);

    // Wrist Tilt and Twist
    angles[0] = jointAngles[4];
    angles[1] = jointAngles[5];
    targets[0] = jointTargets[4];
    targets[1] = jointTargets[5];
    moveToAngle(Wrist, targets, angles, outputs);
    Wrist.moveDiffJoint(outputs[0], outputs[1]);

    Watchdog.clear();
}

void moveToAngle(RoveJoint &Joint, float target, float angle, float& output)
{
    float smallerAngle = min(angle, target);
    float largerAngle = max(angle, target);

    if (largerAngle - smallerAngle > 180) {
        output = Joint.pid.incrementPid(angle, target, PidTolerance);
    }
    else {
        output = Joint.pid.incrementPid(target, angle, PidTolerance);
    }
}

void moveToAngle(RoveJointDifferential &Joint, float targets[2], float angles[2], float outputs[2])
{
    // Tilt
    float smallerAngle = min(angles[0], targets[0]);
    float largerAngle = max(angles[0], targets[0]);
    if (largerAngle - smallerAngle > 180) {
        outputs[0] = Joint.tiltPid.incrementPid(angles[0], targets[0], PidTolerance);
    }
    else {
        outputs[0] = Joint.tiltPid.incrementPid(targets[0], angles[0], PidTolerance);
    }

    // Twist
    smallerAngle = min(angles[1], targets[1]);
    largerAngle = max(angles[1], targets[1]);
    if (largerAngle - smallerAngle > 180) {
        outputs[1] = Joint.twistPid.incrementPid(angles[1], targets[1], PidTolerance);
    }
    else {
        outputs[1] = Joint.twistPid.incrementPid(targets[1], angles[1], PidTolerance);
    }
}

void estop()
{
    ShoulderTilt.moveJoint(0);
    ShoulderTwist.moveJoint(0);
    ElbowTilt.moveJoint(0);
    ElbowTwist.moveJoint(0);
    Wrist.moveDiffJoint(0, 0);
    Gripper.drive(0);
}
