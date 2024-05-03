#include "Arm_Software.h"

#include <cmath>


void setup() {
    Serial.begin(115200);
    Serial.println("Setup");

    // Configure buttons pins
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);

    // Initialize IO Expanders
    // 1 for input, 0 for output
    // Defaults to all inputs
    IOX1.begin();
    IOX2.begin(~((1<<IOX2_SOLENOID) | (1<<IOX2_LAS)));

    // Attach encoders
    X.attachEncoder(&Encoder1);
    Y1.attachEncoder(&Encoder2);
    Y2.attachEncoder(&Encoder3);
    Z.attachEncoder(&Encoder4);
    Pitch.attachEncoder(&Encoder5);
    Roll1.attachEncoder(&Encoder6);
    Roll2.attachEncoder(&Encoder7);

    // TODO: Attach hard limits (Needs lim switches)
    X.attachHardLimits(&LS1, &LS2);
    Y1.attachHardLimits(&LS3, &LS4);
    Y2.attachHardLimits(&LS5, &LS6);
    Z.attachHardLimits(&LS7, &LS8);
    Pitch.attachHardLimits(&LS9, &LS10);

    // Configure encoder inverts
    X.Encoder()->configInvert(false);
    Y1.Encoder()->configInvert(false);
    Y2.Encoder()->configInvert(false);
    Z.Encoder()->configInvert(false);
    Pitch.Encoder()->configInvert(false);
    Roll1.Encoder()->configInvert(false);
    Roll2.Encoder()->configInvert(false);

    // Configure encoder offsets
    X.Encoder()->configOffset(0);
    Y1.Encoder()->configOffset(0);
    Y2.Encoder()->configOffset(0);
    Z.Encoder()->configOffset(0);
    Pitch.Encoder()->configOffset(0);
    Roll1.Encoder()->configOffset(0);
    Roll2.Encoder()->configOffset(0);

    // Attach encoder interrupts, leave the same
    Encoder1.begin([]{Encoder1.handleInterrupt();});
    Encoder2.begin([]{Encoder2.handleInterrupt();});
    Encoder3.begin([]{Encoder3.handleInterrupt();});
    Encoder4.begin([]{Encoder4.handleInterrupt();});
    Encoder5.begin([]{Encoder2.handleInterrupt();});
    Encoder6.begin([]{Encoder3.handleInterrupt();});
    Encoder7.begin([]{Encoder4.handleInterrupt();});

    // Config motor inverts, reference joint
    X.Motor()->configInvert(false);
    Y1.Motor()->configInvert(false);
    Y2.Motor()->configInvert(false);
    Z.Motor()->configInvert(false);
    Pitch.Motor()->configInvert(false);
    Roll1.Motor()->configInvert(false);
    Roll2.Motor()->configInvert(false);
    Gripper1.configInvert(false);
    Gripper2.configInvert(false);
    Spare.configInvert(false);

    // Config motor output limits, reference joint
    X.Motor()->configMaxOutputs(-1000, 1000);
    Y1.Motor()->configMaxOutputs(-1000, 1000);
    Y2.Motor()->configMaxOutputs(-1000, 1000);
    Z.Motor()->configMaxOutputs(-1000, 1000);
    Pitch.Motor()->configMaxOutputs(-1000, 1000);
    Roll1.Motor()->configMaxOutputs(-1000, 1000);
    Roll2.Motor()->configMaxOutputs(-1000, 1000);
    Motor8.configMaxOutputs(-1000, 1000);
    Motor9.configMaxOutputs(-1000, 1000);
    Spare.configMaxOutputs(-1000, 1000);

    // Config motor deadbands, reference joint
    X.Motor()->configMinOutputs(-10, 10);
    Y1.Motor()->configMinOutputs(-10, 10);
    Y2.Motor()->configMinOutputs(-10, 10);
    Z.Motor()->configMinOutputs(-10, 10);
    Pitch.Motor()->configMinOutputs(-10, 10);
    Roll1.Motor()->configMinOutputs(-10, 10);
    Roll2.Motor()->configMinOutputs(-10, 10);
    Gripper1.configMinOutputs(-10, 10);
    Gripper2.configMinOutputs(-10, 10);
    Spare.configMinOutputs(-10, 10);

    // Config motor ramp rates, reference joint
    X.Motor()->configRampRate(10000);
    Y1.Motor()->configRampRate(10000);
    Y2.Motor()->configRampRate(10000);
    Z.Motor()->configRampRate(10000);
    Pitch.Motor()->configRampRate(10000);
    Roll1.Motor()->configRampRate(10000);
    Roll2.Motor()->configRampRate(10000);
    Motor8.configRampRate(10000);
    Motor9.configRampRate(10000);
    Motor10.configRampRate(10000);

    // RoveComm
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_FIRSTOCTET, RC_ARMBOARD_SECONDOCTET, RC_ARMBOARD_THIRDOCTET, RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    Serial.println("Complete");
    
    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}



void loop() {
    uint32_t timestamp = millis();

    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        case RC_ARMBOARD_OPENLOOP_DATA_ID:
        {
            int16_t *data = (int16_t*)packet.data;
            
            X_decipercent = data[0];
            Y1_decipercent = data[1];
            Y2_decipercent = data[2];
            Z_decipercent = data[3];
            Pitch_decipercent = data[4];
            
            if (activeGripper) {
                Roll1_decipercent = 0;
                Roll2_decipercent = data[5];
            } else {
                Roll2_decipercent = 0;
                Roll1_decipercent = data[5];
            }

            break;
        }

        case RC_ARMBOARD_SETPOSITION_DATA_ID:
        {
            
            break;
        }

        case RC_ARMBOARD_INCREMENTPOSITION_DATA_ID:
        {

            break;
        }

        case RC_ARMBOARD_SETIK_DATA_ID:
        {

            break;
        }

        case RC_ARMBOARD_INCREMENTIK_ROVERRELATIVE_DATA_ID:
        {

            break;
        }

        case RC_ARMBOARD_INCREMENTIK_WRISTRELATIVE_DATA_ID:
        {

            break;
        }

        case RC_ARMBOARD_LASER_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);

            laserOn = (data == 0)? false : true;
            break;
        }

        case RC_ARMBOARD_SOLENOID_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);

            extendSolenoid = (data == 0)? false : true;
            break;
        }

        case RC_ARMBOARD_GRIPPER_DATA_ID:
        {
            int16_t data = *((int16_t*) packet.data);

            switch(activeGripper) {
                case 0:
                    Gripper1_decipercent = data;
                    Gripper2_decipercent = 0;
                    break;
                case 1:
                    Gripper1_decipercent = 0;
                    Gripper2_decipercent = data;
                    break;
            }
            break;
        }

        case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            watchdogOverride = *((uint8_t*) packet.data);
            break;
        }

        case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);

            X.overrideForwardHardLimit(data & (1<<0));
            X.overrideReverseHardLimit(data & (1<<1));
            Y1.overrideForwardHardLimit(data & (1<2));
            Y1.overrideReverseHardLimit(data & (1<<3));
            Y2.overrideForwardHardLimit(data & (1<<4));
            Y2.overrideReverseHardLimit(data & (1<<5));
            Z.overrideForwardHardLimit(data & (1<<6));
            Z.overrideReverseHardLimit(data & (1<<7));
            Pitch.overrideForwardHardLimit(data & (1<<8));
            Pitch.overrideReverseHardLimit(data & (1<<9));
            break;
        }


        case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);
            
            X_Joint.calibrating = data & (1<<0);
            Y1_Joint.calibrating = data & (1<<1);
            Y2_Joint.calibrating = data & (1<<2);
            Z_Joint.calibrating = data & (1<<3);
            Pitch_Joint.calibrating = data & (1<<4);
            break;
        }

        case RC_ARMBOARD_SELECTGRIPPER_DATA_ID:
        {
            activeGripper = *((uint8_t*) packet.data);
            break;
        }

        // Default
        default:
        {
            break;
        }
    }

    // IO Expanders
    if (timestamp - lastIOX_timestamp > IOX_UPDATE_PERIOD) {
        // IO Expander 1
        uint8_t iox1_val = IOX1.read8();
        LS1.set(iox1_val & (1<<IOX1_LIM_1));
        LS2.set(iox1_val & (1<<IOX1_LIM_2));
        LS3.set(iox1_val & (1<<IOX1_LIM_3));
        LS4.set(iox1_val & (1<<IOX1_LIM_4));
        LS5.set(iox1_val & (1<<IOX1_LIM_5));
        LS6.set(iox1_val & (1<<IOX1_LIM_6));
        LS7.set(iox1_val & (1<<IOX1_LIM_7));
        LS8.set(iox1_val & (1<<IOX1_LIM_8));

        // IO Expander 2
        uint8_t iox2_val = IOX2.read8();
        LS9.set(iox2_val & (1<<IOX2_LIM_9));
        LS10.set(iox2_val & (1<<IOX2_LIM_10));
        direction = iox2_val & (1<<IOX2_DIR_SW);
        IOX2.write(IOX2_LAS, laserOn);
        IOX2.write(IOX2_SOLENOID, extendSolenoid);
    }

    if (X_Joint.calibrating) {
        X.drive(900);
        if (X.atForwardHardLimit()) {
            X.drive(0);
            X.Encoder()->setDegrees(0);
            X_Joint.calibrating = false;
            X_Joint.calibrated = true;
        }
    }


    // Buttons
    uint8_t buttons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    // Motor outputs

    // X
    // if calibrating: drive left, if at fwd lim switch, then set calibrating to false and set calibrated to true and also reset position var to 0
    //Create a struct for each joint: target pos, calibrating, and calibrated
    if (buttons == BTN_1) X.drive((direction? 900 : -900));
    else X.drive(X_decipercent);

    // Y1
    if (buttons == BTN_2) Y1.drive((direction? 900 : -900));
    else Y1.drive(Y1_decipercent);

    // Y2
    if (buttons == BTN_3) Y2.drive((direction? 900 : -900));
    else Y2.drive(Y2_decipercent);

    // Z
    if (buttons == BTN_4) Z.drive((direction? 900 : -900));
    else Z.drive(Z_decipercent);
    
    // Pitch
    if (buttons == BTN_5) Pitch.drive((direction? 900 : -900));
    else Pitch.drive(Pitch_decipercent);

    // Roll1
    if (buttons == BTN_6) Roll1.drive((direction? 900 : -900));
    else Roll1.drive(Roll1_decipercent);

    // Roll2
    if (buttons == BTN_7) Roll2.drive((direction? 900 : -900));
    else Roll2.drive(Roll2_decipercent);

    // Gripper1
    if (buttons == BTN_8) Gripper1.drive((direction? 900 : -900));
    else Gripper1.drive(Gripper1_decipercent);

    // Gripper2
    if (buttons == BTN_9) Gripper2.drive((direction? 900 : -900));
    else Gripper2.drive(Gripper2_decipercent);

    // Solenoid
    if (buttons == BTN_10) ;
    else ;

    // Laser
    if (buttons == BTN_11) ;
    else ;
}


void estop() {
    if (!watchdogOverride) {
        watchdogStatus = 1;

        closedLoopActive = false;

        X_decipercent = 0;
        Y1_decipercent = 0;
        Y2_decipercent = 0;
        Z_decipercent = 0;
        Pitch_decipercent = 0;
        Roll1_decipercent = 0;
        Roll2_decipercent = 0;
        Gripper1_decipercent = 0;
        Gripper2_decipercent = 0;
    }
}

void telemetry() {
    RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, RC_ARMBOARD_WATCHDOGSTATUS_DATA_COUNT, watchdogStatus);

    if (!telemetryOverride) {
        float positions[7] = {Encoder1.readDegrees(), Encoder2.readDegrees(), Encoder3.readDegrees(), Encoder4.readDegrees(), 0, 0, 0};
        RoveComm.write(RC_ARMBOARD_POSITIONS_DATA_ID, RC_ARMBOARD_POSITIONS_DATA_COUNT, positions);

        float coordinates[5] = {0, 0, 0, 0, 0};
        RoveComm.write(RC_ARMBOARD_COORDINATES_DATA_ID, RC_ARMBOARD_COORDINATES_DATA_COUNT, coordinates);

        uint8_t limitSwitches = (X.atForwardHardLimit() << 0) | (X.atReverseHardLimit() << 1) | (Y1.atForwardHardLimit() << 2) | (Y1.atReverseHardLimit() << 3) |
                                (Y2.atForwardHardLimit() << 4) | (Y2.atReverseHardLimit() << 5) | (Z.atForwardHardLimit() << 6) | (Z.atReverseHardLimit() << 7) | 
                                (Pitch.atForwardHardLimit() << 8) | (Pitch.atReverseHardLimit() << 9);;
        RoveComm.write(RC_ARMBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_ARMBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitches);
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
