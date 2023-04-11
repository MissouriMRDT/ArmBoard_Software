#include "Arm_Software.h"


void setup() {
    Serial.begin(115200);

    // Configure lasers
    pinMode(LAS, OUTPUT);

    // TODO configure in constructor of LimitSwitch
    //pinMode(LIM_1, INPUT);
    //pinMode(LIM_2, INPUT);
    //pinMode(LIM_3, INPUT);
    //pinMode(LIM_4, INPUT);
    //pinMode(LIM_5, INPUT);
    //pinMode(LIM_6, INPUT);

    // Configure encoder offsets
    Encoder1.configOffset(0);
    Encoder2.configOffset(0);
    Encoder3.configOffset(0);
    Encoder4.configOffset(0);
    Encoder5.configOffset(0);
    Encoder6.configOffset(0);

    // Attach encoder interrupts
    Encoder1.begin([]{Encoder1.handleInterrupt();});
    Encoder2.begin([]{Encoder2.handleInterrupt();});
    Encoder3.begin([]{Encoder3.handleInterrupt();});
    Encoder4.begin([]{Encoder4.handleInterrupt();});
    Encoder5.begin([]{Encoder5.handleInterrupt();});
    Encoder6.begin([]{Encoder6.handleInterrupt();});


    // Config motor output limits
    Motor1.configOutputLimits(950, -950);
    Motor2.configOutputLimits(950, -950);
    Motor3.configOutputLimits(950, -950);
    Motor4.configOutputLimits(950, -950);
    Motor5.configOutputLimits(950, -950);
    Motor6.configOutputLimits(950, -950);
    Motor7.configOutputLimits(950, -950);
    Motor8.configOutputLimits(950, -950);
    Motor9.configOutputLimits(950, -950);

    // Configure PID controllers
    PID1.configPID(0, 0, 0);
    PID2.configPID(0, 0, 0);
    PID3.configPID(0, 0, 0);
    PID4.configPID(0, 0, 0);
    PID5.configPID(0, 0, 0);
    PID6.configPID(0, 0, 0);

    
    // Attach encoders
    J1.attachEncoder(&Encoder1);
    J2.attachEncoder(&Encoder2);
    J3.attachEncoder(&Encoder3);
    J4.attachEncoder(&Encoder4);
    Wrist.attachTwistEncoder(&Encoder5);
    Wrist.attachTiltEncoder(&Encoder6);

    // Attach PID controllers
    J1.attachPID(&PID1);
    J2.attachPID(&PID2);
    J3.attachPID(&PID3);
    J4.attachPID(&PID4);
    Wrist.attachTwistPID(&PID5);
    Wrist.attachTiltPID(&PID6);

    // Attach hard limits
    //J1.attachHardLimits(&LS1, &LS2);
    //J2.attachHardLimits(&LS3, &LS4);
    //J3.attachHardLimits(&LS5, &LS6);

    // Configure soft limits
    //J1.configSoftLimits(0, 360);
    //J2.configSoftLimits(0, 360);
    //J3.configSoftLimits(0, 360);
    //J4.configSoftLimits(0, 360);

    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_ARMBOARD_MAC);
    feedWatchdog();
}


uint32_t lastWriteCoordinatesTimestamp = millis();
uint32_t lastWriteJointAnglesTimestamp = millis();

void loop() {
    uint32_t timestamp = millis();
    updateJointAngles();
    updateCoordinates();

    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        // Open loop control of J1-J6
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID: 
        {
            int16_t* data = (int16_t*) packet.data; // decipercent values for J1-J6
            closedLoopActive = false;

            openLoop(data);
            feedWatchdog();
            break;
        }

        // Closed loop control of J1-J6
        case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID:
        {
            float* data = (float*) packet.data; // target angles in degrees for J1-J6
            closedLoopActive = true;

            updateTargetAngles_Position(data);
            feedWatchdog();
            break;
        }

        // Closed loop control of J1-J6, using IK to calculate targetAngles
        case RC_ARMBOARD_ARMMOVEIK_DATA_ID:
        {
            float* data = (float*) packet.data; // Destination coordinates (x, y, z, yaw, pitch, roll), angles in degrees
            closedLoopActive = true;

            bool valid = updateTargetAngles_IK(data);
            if (!valid) Serial.println("IK destination out of range.");
            feedWatchdog();
            break;
        }

        // Open loop control of hex key
        case RC_ARMBOARD_ENDEFFECTOR_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            HexKey.drive(data);
            feedWatchdog();
            break;
        }

        // Open loop control of gripper
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            Gripper.drive(data);
            feedWatchdog();
            break;
        }
        
        // Toggle lasers on or off
        case RC_ARMBOARD_LASERS_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];
            digitalWrite(LAS, data);
            break;
        }

        // Write joint angles to RoveComm
        case RC_ARMBOARD_REQUESTJOINTPOSITIONS_DATA_ID:
        {
            RoveComm.writeReliable(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
            lastWriteJointAnglesTimestamp = timestamp;
            break;
        }

        // Write coordinates to RoveComm
        case RC_ARMBOARD_REQUESTAXESPOSITIONS_DATA_ID:
        {
            RoveComm.writeReliable(RC_ARMBOARD_IKCOORDINATES_DATA_ID, RC_ARMBOARD_IKCOORDINATES_DATA_COUNT, coordinates);
            lastWriteCoordinatesTimestamp = timestamp;
            break;
        }

        // Default
        default:
        {
            break;
        }
    }


    // Update closed loop
    if (closedLoopActive) closedLoop(timestamp);


    // Periodically write telemetry
    if ((timestamp - lastWriteCoordinatesTimestamp) >= ROVECOMM_UPDATE_RATE*4) {
        RoveComm.write(RC_ARMBOARD_IKCOORDINATES_DATA_ID, RC_ARMBOARD_IKCOORDINATES_DATA_COUNT, coordinates);
        lastWriteCoordinatesTimestamp = timestamp;
    }

    if ((timestamp - lastWriteJointAnglesTimestamp) >= ROVECOMM_UPDATE_RATE*4) {
        RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
        lastWriteJointAnglesTimestamp = timestamp;
    }


}


void updateJointAngles() {
    jointAngles[0] = Encoder1.readDegrees();
    jointAngles[1] = Encoder2.readDegrees();
    jointAngles[2] = Encoder3.readDegrees();
    jointAngles[3] = Encoder4.readDegrees();
    jointAngles[4] = Encoder5.readDegrees();
    jointAngles[5] = Encoder6.readDegrees();
}

void updateCoordinates() {
    forwardKinematics(jointAngles, coordinates);
}

void updateTargetAngles_Position(float targets[6]) {
    targetAngles[0] = targets[0];
    targetAngles[1] = targets[1];
    targetAngles[2] = targets[2];
    targetAngles[3] = targets[3];
    targetAngles[4] = targets[4];
    targetAngles[5] = targets[5];
}

bool updateTargetAngles_IK(float dest[6]) {
    bool valid = inverseKinematics(dest, jointAngles, targetAngles); 
    
    // If IK destination is invalid, hold current position
    if (!valid) {
        targetAngles[0] = jointAngles[0];
        targetAngles[1] = jointAngles[1];
        targetAngles[2] = jointAngles[2];
        targetAngles[3] = jointAngles[3];
        targetAngles[4] = jointAngles[4];
        targetAngles[5] = jointAngles[5];
    }

    return valid;
}



void openLoop(int16_t decipercents[6]) {
    J1.drive(decipercents[0]);
    J2.drive(decipercents[1]);
    J3.drive(decipercents[2]);
    J4.drive(decipercents[3]);
    Wrist.drive(decipercents[4], decipercents[5]);
}

void closedLoop(uint32_t timestamp) {
    J1.setAngle(targetAngles[0], timestamp);
    J2.setAngle(targetAngles[1], timestamp);
    J3.setAngle(targetAngles[2], timestamp);
    J4.setAngle(targetAngles[3], timestamp);
    Wrist.setAngles(targetAngles[4], targetAngles[5], timestamp);
}

void estop() {
    J1.drive(0);
    J2.drive(0);
    J3.drive(0);
    J4.drive(0);
    Wrist.drive(0, 0);
}

void feedWatchdog() {
  Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
