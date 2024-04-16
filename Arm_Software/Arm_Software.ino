#include "Arm_Software.h"

#include <cmath>


void setup() {
    Serial.begin(115200);
    Serial.println("Setup");


    // Configure I/O Extender
    pinMode(SDA, BI); //How to set bidirectional?
    pinMode(SCL, OUTPUT);

    // Configure buttons pins
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);


    // Configure encoder inverts
    Encoder1.configInvert(false);
    Encoder2.configInvert(false);
    Encoder3.configInvert(false);
    Encoder4.configInvert(false);
    Encoder5.configInvert(false);
    Encoder6.configInvert(false);
    Encoder7.configInvert(false);

    // Configure encoder offsets
    Encoder1.configOffset(0);
    Encoder2.configOffset(0);
    Encoder3.configOffset(0);
    Encoder4.configOffset(0);
    Encoder5.configOffset(0);
    Encoder6.configOffset(0);
    Encoder7.configOffset(0);

    // Attach encoder interrupts
    Encoder1.begin([]{Encoder1.handleInterrupt();});
    Encoder2.begin([]{Encoder2.handleInterrupt();});
    Encoder3.begin([]{Encoder3.handleInterrupt();});
    Encoder4.begin([]{Encoder4.handleInterrupt();});
    Encoder5.begin([]{Encoder2.handleInterrupt();});
    Encoder6.begin([]{Encoder3.handleInterrupt();});
    Encoder7.begin([]{Encoder4.handleInterrupt();});

    // Config motor inverts
    Motor1.configInvert(false);
    Motor2.configInvert(false);
    Motor3.configInvert(false);
    Motor4.configInvert(false);
    Motor5.configInvert(false);
    Motor6.configInvert(false);
    Motor7.configInvert(false);
    Motor8.configInvert(false);
    Motor9.configInvert(false);
    Motor10.configInvert(false);


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
    Motor10.configMaxOutputs(-900, 900);

    // Config motor deadbands
    Motor1.configMinOutputs(-10, 10);
    Motor2.configMinOutputs(-10, 10);
    Motor3.configMinOutputs(-10, 10);
    Motor4.configMinOutputs(-10, 10);
    Motor5.configMinOutputs(-10, 10);
    Motor6.configMinOutputs(-10, 10);
    Motor7.configMinOutputs(-10, 10);
    Motor8.configMinOutputs(-10, 10);
    Motor9.configMinOutputs(-10, 10);
    Motor10.configMinOutputs(-10, 10);

    // Config motor ramp rates
    Motor1.configRampRate(10000);
    Motor2.configRampRate(10000);
    Motor3.configRampRate(10000);
    Motor4.configRampRate(10000);
    Motor5.configRampRate(10000);
    Motor6.configRampRate(10000);
    Motor7.configRampRate(10000);
    Motor8.configRampRate(10000);
    Motor9.configRampRate(10000);
    Motor10.configRampRate(10000);


    // Attach encoders //Other 3 encoders???
    X.attachEncoder(&Encoder1);
    Y1.attachEncoder(&Encoder2);
    Y2.attachEncoder(&Encoder3);
    Z.attachEncoder(&Encoder4);

    // TODO: Attach hard limits (Needs lim switches)


    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_FIRSTOCTET, RC_ARMBOARD_SECONDOCTET, RC_ARMBOARD_THIRDOCTET, RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    Serial.println("Complete");
    
    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}



void loop() {
    float timestamp = ((float) millis()) / 1000.0;

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
            Roll1_decipercent = data[5];
            Roll2_decipercent = data[6];

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
            break;
        }


        case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID:
        {

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



    // Buttons
    bool direction = digitalRead(DIR_SW); //Change for IO Extender
    uint8_t buttons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    // Motor outputs

    // X
    if (buttons == 1) X.drive((direction? 900 : -900));
    else X.drive(X_decipercent);

    // Y1
    if (buttons == 2) Y1.drive((direction? 900 : -900));
    else Y1.drive(Y1_decipercent);

    // Y2
    if (buttons == 3) Y2.drive((direction? 900 : -900));
    else Y2.drive(Y2_decipercent);

    // Z
    if (buttons == 4) Z.drive((direction? 900 : -900));
    else Z.drive(Z_decipercent);
    
    // Pitch
    if (buttons == 5) Pitch.drive((direction? 900 : -900));
    else Pitch.drive(Pitch_decipercent);

    // Roll1
    if (buttons == 6) Roll1.drive((direction? 900 : -900));
    else Roll1.drive(Roll1_decipercent);

    // Roll2
    if (buttons == 7) Roll2.drive((direction? 900 : -900));
    else Roll2.drive(Roll2_decipercent);

    // Gripper1
    if (buttons == 8) Gripper1.drive((direction? 900 : -900));
    else Gripper1.drive(Gripper1_decipercent);

    // Gripper2
    
    // Solenoid

    // Laser (Change for IO Extender)
    if (buttons == 9) digitalWrite(LAS, HIGH);
    else digitalWrite(LAS, (laserOn? HIGH : LOW));

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
                                (Y2.atForwardHardLimit() << 4) | (Y2.atReverseHardLimit() << 5) | (Z.atForwardHardLimit() << 6) | (Z.atReverseHardLimit() << 7);
        RoveComm.write(RC_ARMBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_ARMBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitches);
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
