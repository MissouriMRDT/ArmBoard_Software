#include "Arm_Software.h"

#include <cmath>


void setup() {
    // Serial debugger
    Serial.begin(115200);
    Serial.println("Setup");

    // Digital pins
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

    // Configure encoder inverts
    Encoder1.configInvert(false);
    Encoder2.configInvert(false);
    Encoder3.configInvert(false);
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
    Motor1.configInvert(false);
    Motor2.configInvert(false);
    Motor3.configInvert(false);
    Motor4.configInvert(false);
    Motor5.configInvert(false);
    Motor6.configInvert(false);
    Motor7.configInvert(false);
    Motor8.configInvert(false);

    // Config motor output limits
    Motor1.configMaxOutputs(-900, 900);
    Motor2.configMaxOutputs(-900, 900);
    Motor3.configMaxOutputs(-900, 900);
    Motor4.configMaxOutputs(-900, 900);
    Motor5.configMaxOutputs(-900, 900);
    Motor6.configMaxOutputs(-900, 900);
    Motor7.configMaxOutputs(-900, 900);
    Motor8.configMaxOutputs(-900, 900);

    // Config motor deadbands
    Motor1.configMinOutputs(-10, 10);
    Motor2.configMinOutputs(-10, 10);
    Motor3.configMinOutputs(-10, 10);
    Motor4.configMinOutputs(-10, 10);
    Motor5.configMinOutputs(-10, 10);
    Motor6.configMinOutputs(-10, 10);
    Motor7.configMinOutputs(-10, 10);
    Motor8.configMinOutputs(-10, 10);

    // Config motor ramp rates
    Motor1.configRampRate(10000);
    Motor2.configRampRate(10000);
    Motor3.configRampRate(10000);
    Motor4.configRampRate(10000);
    Motor5.configRampRate(10000);
    Motor6.configRampRate(10000);
    Motor7.configRampRate(10000);
    Motor8.configRampRate(10000);

    // Attach encoders

    // Attach hard limits

    // Configure soft limits

    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_FIRSTOCTET, RC_ARMBOARD_SECONDOCTET, RC_ARMBOARD_THIRDOCTET, RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    Serial.println("Complete");
    
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}



void loop() {

    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        case RC_ARMBOARD_OPENLOOP_DATA_ID:
        {
            
            break;
        }

    }


    // Buttons
    bool direction = digitalRead(DIR_SW);
    uint8_t buttons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);


    // Motor outputs

    // X
    if (buttons == 1) X.drive((direction? 900 : -900));
    else X.drive(XDecipercent);

    // Y1

    // Y2

    // Z
    
    // Pitch

    // Roll1

    // Roll2

    // Gripper

    // Solenoid
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
        float positions[6] = {0, 0, 0, 0, 0, 0};
        RoveComm.write(RC_ARMBOARD_POSITIONS_DATA_ID, RC_ARMBOARD_POSITIONS_DATA_COUNT, positions);

        float coordinates[6] = {0, 0, 0, 0, 0, 0};
        RoveComm.write(RC_ARMBOARD_COORDINATES_DATA_ID, RC_ARMBOARD_COORDINATES_DATA_COUNT, coordinates);
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
