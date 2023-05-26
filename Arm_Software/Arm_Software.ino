#include "Arm_Software.h"

#include <cmath>


void setup() {
    Serial.begin(115200);
    Serial.println("Setup");


    // Configure laser pin
    pinMode(LAS, OUTPUT);

    // Configure buttons pins
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);
    pinMode(DIR_SW, INPUT);


    // Configure limit switch inverts
    LS1.configInvert(false);
    LS2.configInvert(false);
    LS3.configInvert(false);
    LS4.configInvert(false);
    LS5.configInvert(false);
    LS6.configInvert(false);
    LS7.configInvert(false);
    LS8.configInvert(false);
    LS9.configInvert(false);
    LS10.configInvert(false);
    //LS11.configInvert(false);
    //LS12.configInvert(false);
    BD_LS12.configInvert(false);

    // Begin bidirectional limit switches
    BD_LS12.begin([]{BD_LS12.handleInterrupt();});


    // Configure encoder inverts
    Encoder1.configInvert(true);
    Encoder2.configInvert(true);
    Encoder3.configInvert(true);
    Encoder4.configInvert(false);
    Encoder5.configInvert(false);
    Encoder6.configInvert(false);

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


    // Config motor inverts
    Motor1.configInvert(true);
    Motor2.configInvert(true);
    Motor3.configInvert(true);
    Motor4.configInvert(false);
    Motor5.configInvert(false);
    Motor6.configInvert(true);
    Motor7.configInvert(false);
    Motor8.configInvert(false);
    Motor9.configInvert(true);

    // Config motor output limits
    Motor1.configMaxOutputs(-900, 900);
    Motor2.configMaxOutputs(-900, 900);
    Motor3.configMaxOutputs(-900, 900);
    Motor4.configMaxOutputs(-900, 900);
    Motor5.configMaxOutputs(-900, 900);
    Motor6.configMaxOutputs(-900, 900);
    Motor7.configMaxOutputs(-900, 900);
    Motor8.configMaxOutputs(-900, 900);
    Motor9.configMaxOutputs(-900, 900);

    // Config motor deadbands
    Motor1.configMinOutputs(-50, 50);
    Motor2.configMinOutputs(-50, 50);
    Motor3.configMinOutputs(-30, 30);
    Motor4.configMinOutputs(-30, 30);
    Motor5.configMinOutputs(-20, 20);
    Motor6.configMinOutputs(-20, 20);
    Motor7.configMinOutputs(-10, 10);
    Motor8.configMinOutputs(-10, 10);
    Motor9.configMinOutputs(-50, 50);

    // Config motor ramp rates
    Motor1.configRampRate(10000);
    Motor9.configRampRate(10000);


    // Configure PID controllers
    PID1.configPID(130, 0, 0);
    PID2.configPID(0, 0, 0);
    PID3.configPID(90, 0, 0);
    PID4.configPID(0, 0, 0);
    PID5.configPID(0, 0, 0);
    PID6.configPID(0, 0, 0);


    // Attach encoders
    J1.attachEncoder(&Encoder1);
    J2.attachEncoder(&Encoder2);
    J3.attachEncoder(&Encoder3);
    J4.attachEncoder(&Encoder4);
    Wrist.attachTiltEncoder(&Encoder5);
    Wrist.attachTwistEncoder(&Encoder6);

    // Attach PID controllers
    J1.attachPID(&PID1);
    J2.attachPID(&PID2);
    J3.attachPID(&PID3);
    J4.attachPID(&PID4);
    Wrist.attachTiltPID(&PID5);
    Wrist.attachTwistPID(&PID6);

    // Attach hard limits
    J1.attachHardLimits(&BD_LS12.forwardSwitch(), &BD_LS12.reverseSwitch());
    J2.attachHardLimits(&LS9, &LS10);
    J3.attachHardLimits(&LS7, &LS8);


    // Configure soft limits
    //J1.configSoftLimits(90, 270);
    //J2.configSoftLimits(215, 105);
    //J3.configSoftLimits(110, 220);
    //J4.configSoftLimits(330, 30);

    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_FIRSTOCTET, RC_ARMBOARD_SECONDOCTET, RC_ARMBOARD_THIRDOCTET, RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    Serial.println("Complete");
    
    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}



void loop() {
    float timestamp = ((float) millis()) / 1000.0;
    updateJointAngles();
    updateCoordinates();

    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        // Open loop control of J1-J6
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID: 
        {
            int16_t* data = (int16_t*) packet.data; // decipercent values for J1-J6

            for (int i = 0; i < 6; i++) {
                decipercents[i] = data[i];
            }

            closedLoopActive = false;
            feedWatchdog();
            break;
        }

        // Closed loop control of J1-J6
        case RC_ARMBOARD_ARMMOVETOPOSITION_DATA_ID:
        {
            float* data = (float*) packet.data; // target angles in degrees for J1-J6

            for (int i = 0; i < 6; i++) {
                targetAngles[i] = data[i];
            }

            closedLoopActive = true;
            feedWatchdog();
            break;
        }

        // Incremental closed loop control of J1-J6
        case RC_ARMBOARD_ARMINCREMENTPOSITION_DATA_ID:
        {
            float* data = (float*) packet.data; // change in target angles, in degrees, for J1-J6

            for (int i = 0; i < 6; i++) {
                targetAngles[i] = fmod(targetAngles[i] + data[i], 360);
                if (targetAngles[i] < 0) targetAngles[i] += 360;
            }

            closedLoopActive = true;
            feedWatchdog();
            break;
        }

        // Control of J1-J6 using IK
        case RC_ARMBOARD_ARMMOVEIK_DATA_ID:
        {
            float* data = (float*) packet.data; // Destination coordinates (x, y, z, yaw, pitch, roll), angles in degrees
            
            // If IK destination is invalid, hold current position
            bool valid = inverseKinematics(data, jointAngles, targetAngles); 
            if (!valid) {
                targetAngles[0] = jointAngles[0];
                targetAngles[1] = jointAngles[1];
                targetAngles[2] = jointAngles[2];
                targetAngles[3] = jointAngles[3];
                targetAngles[4] = jointAngles[4];
                targetAngles[5] = jointAngles[5];
            }

            closedLoopActive = true;
            feedWatchdog();
            break;
        }

        // Incremental control of J1-J6 using IK, relative to rover
        case RC_ARMBOARD_ARMINCREMENTIKROVER_DATA_ID:
        {
            // TODO: support incremental IK
            break;
        }

        // Incremental control of J1-J6 using IK, relative to wrist
        case RC_ARMBOARD_ARMINCREMENTIKWRIST_DATA_ID:
        {
            // TODO: support incremental IK
            break;
        }
        
        // Toggle lasers on or off
        case RC_ARMBOARD_LASERS_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            digitalWrite(LAS, data);
            break;
        }

        // Toggle solenoid
        case RC_ARMBOARD_ENDEFFECTOR_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            extendSolenoid = data;
            break;
        }

        // Open loop control of gripper 1 and 2
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;

            decipercents[6] = data[0];
            decipercents[7] = data[1];
            feedWatchdog();
            break;
        }

        // Override watchdog
        case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            watchdogOverride = data;
            break;
        }

        // Override limit switches
        case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        {
            uint16_t data = ((uint16_t*) packet.data)[0];

            Wrist.overrideTwistForwardHardLimit(data & (1<<11));
            Wrist.overrideTwistReverseHardLimit(data & (1<<10));
            Wrist.overrideTiltForwardHardLimit(data & (1<<9));
            Wrist.overrideTiltReverseHardLimit(data & (1<<8));
            J4.overrideForwardHardLimit(data & (1<<7));
            J4.overrideReverseHardLimit(data & (1<<6));
            J3.overrideForwardHardLimit(data & (1<<5));
            J3.overrideReverseHardLimit(data & (1<<4));
            J2.overrideForwardHardLimit(data & (1<<3));
            J2.overrideReverseHardLimit(data & (1<<2));
            J1.overrideForwardHardLimit(data & (1<<1));
            J1.overrideReverseHardLimit(data & (1<<0));

            break;
        }

        // Write joint angles to RoveComm
        case RC_ARMBOARD_REQUESTJOINTPOSITIONS_DATA_ID:
        {
            RoveComm.writeReliable(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
            break;
        }

        // Toggle continuous telemetry for jointAngles and coordinates
        case RC_ARMBOARD_TOGGLEPOSITIONTELEM_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            telemetryOverride = data;
            break;
        }

        // Write coordinates to RoveComm
        case RC_ARMBOARD_REQUESTAXESPOSITIONS_DATA_ID:
        {
            RoveComm.writeReliable(RC_ARMBOARD_IKCOORDINATES_DATA_ID, RC_ARMBOARD_IKCOORDINATES_DATA_COUNT, coordinates);
            break;
        }

        // Default
        default:
        {
            break;
        }
    }


    // Buttons
    bool direction = digitalRead(DIR_SW);
    uint8_t manualButtons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    // Motor outputs
    if (closedLoopActive) {
        J1.setAngle(targetAngles[0], timestamp);
        J2.setAngle(targetAngles[1], timestamp);
        J3.setAngle(targetAngles[2], timestamp);
        J4.setAngle(targetAngles[3], timestamp);
        Wrist.setAngles(targetAngles[5], targetAngles[4], timestamp);
    }
    else {
        // J1
        if (manualButtons == 1) J1.drive((direction? 900 : -900), timestamp);
        else J1.drive(decipercents[0], timestamp);

        // J2
        if (manualButtons == 2) J2.drive((direction? 900 : -900), timestamp);
        else J2.drive(decipercents[1], timestamp);

        // J3
        if (manualButtons == 3) J3.drive((direction? 900 : -900), timestamp);
        else J3.drive(decipercents[2], timestamp);

        // J4
        if (manualButtons == 4) J4.drive((direction? 900 : -900), timestamp);
        else J4.drive(decipercents[3], timestamp);

        // J5
        if (manualButtons == 5) Wrist.drive(0, (direction? 900 : -900), timestamp);
        else if (manualButtons == 6) Wrist.drive((direction? 900 : -900), 0, timestamp);
        else Wrist.drive(decipercents[5], decipercents[4], timestamp);
    }

    // Gripper
    if (manualButtons == 7) Gripper.drive((direction? 900 : -900), timestamp);
    else Gripper.drive(decipercents[6], timestamp);

    // Hex Key
    if (manualButtons == 8) HexKey.drive((direction? 900 : -900), timestamp);
    else HexKey.drive(decipercents[7], timestamp);

    // Spare
    if (manualButtons == 9) Solenoid.drive((direction? 900 : -900), timestamp);
    else Solenoid.drive((extendSolenoid? 900 : 0), timestamp);

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



void estop() {
    if (!watchdogOverride) {
        watchdogStatus = 1;

        for (int i = 0; i < 8; i++) {
            decipercents[i] = 0;
        }
        closedLoopActive = false;
    }
}

void telemetry() {
    RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, RC_ARMBOARD_WATCHDOGSTATUS_DATA_COUNT, watchdogStatus);

    if (!telemetryOverride) {
        RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
        RoveComm.write(RC_ARMBOARD_IKCOORDINATES_DATA_ID, RC_ARMBOARD_IKCOORDINATES_DATA_COUNT, coordinates);
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
