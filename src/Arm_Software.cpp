#include "Arm_Software.h"

//2025 REV 1

#include <cmath>

void setup() {
    Serial.begin(115200);
    Serial.println("Setup");

    // Button pins
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);

    // IO expander pins
    IOX_TWI.begin();
    IOX1.begin();

    IOX2.begin(~((1<<IOX2_FWD_1) | (1<<IOX2_RVS_1)));
    IOX2.begin(~((1<<IOX2_FWD_2) | (1<<IOX2_RVS_2)));
    IOX2.begin(~((1<<IOX2_FWD_3) | (1<<IOX2_RVS_3)));

    IOX3.begin(~((1<<IOX3_FWD_4) | (1<<IOX3_RVS_4)));
    IOX3.begin(~((1<<IOX3_FWD_5) | (1<<IOX3_RVS_5)));
    IOX3.begin(~((1<<IOX3_FWD_6) | (1<<IOX3_RVS_6)));
    IOX3.begin(~((1<<IOX3_FWD_7) | (1<<IOX3_RVS_7)));

    // Attach encoders
    X.attachEncoder(&Encoder1);
    J2.attachEncoder(&Encoder3);
    J3.attachEncoder(&Encoder4);
    J4.attachEncoder(&Encoder5);
    Pitch.attachEncoder(&Encoder6);
    Roll.attachEncoder(&Encoder2);

    //Attach hard limits


    // Attach encoder inverts
    X.Encoder()->configinvert(true);
    J2.Encoder()->configinvert(true);
    J3.Encoder()->configinvert(true);
    J4.Encoder()->configinvert(true);
    Pitch.Encoder()->configinvert(true);
    Roll.Encoder()->configinvert(true);

    // Configrue encoder interupts
    Encoder1.begin([]{Encoder1.handleInterupt();});
    Encoder2.begin([]{Encoder2.handleInterupt();});
    Encoder3.begin([]{Encoder3.handleInterupt();});
    Encoder4.begin([]{Encoder4.handleInterupt();});
    Encoder5.begin([]{Encoder5.handleInterupt();});
    Encoder6.begin([]{Encoder6.handleInterupt();});

    // Config motor inverts, reference joint
    X.Motor()->configinvert(false);
    J2.Motor()->configInvert(false);
    J3.Motor()->configInvert(false);
    J4.Motor()->configInvert(false);
    Pitch.Motor()->configInvert(false);
    Roll.Motor()->configInvert(false);
    Gripper.configInvert(false);

    // Config motor output limits, reference joint
    X.Motor()->configMaxOutputs(-1000, 1000);
    J2.Motor()->configMaxOutputs(-1000, 1000);
    J3.Motor()->configMaxOutputs(-1000, 1000);
    J4.Motor()->configMaxOutputs(-1000, 1000);
    Pitch.Motor()->configMaxOutputs(-1000, 1000);

     // Config motor deadbands, reference joint
     X.Motor()->configMinOutputs(-200, 200); // change the values when testing
    J2.Motor()->configMinOutputs(-100, 170); // change the values when testing
    J3.Motor()->configMinOutputs(-100, 220); // change the values when testing
    J4.Motor()->configMinOutputs(-220, 190); // change the values when testing
    Pitch.Motor()->configMinOutputs(-50, 50); // change the values when testing
    Roll.Motor()->configMinOutputs(-200, 200); // change the values when testing
    Gripper.configMinOutputs(-50, 50); // change the values when testing

     // Config motor ramp rates, reference joint , change value when testing
    X.Motor()->configRampRate(10000); 
    J2.Motor()->configRampRate(10000);
    J3.Motor()->configRampRate(10000);
    J4.Motor()->configRampRate(10000);
    Pitch.Motor()->configRampRate(10000);
    Roll.Motor()->configRampRate(10000);
    Gripper.Motor()->configRampRate(10000);

     // X soft limits
    X.overrideReverseSoftLimit(true);
    X.overrideForwardSoftLimit(true);

     // J2 soft limits
    J2.configSoftLimits(0, J2.qMax);
    J2.overrideReverseSoftLimit(true);
    J2.overrideForwardSoftLimit(true);

     // J3 soft limits
    J3.configSoftLimit(J3.qMin, J3.qMax);
    J3.overrideReverseSoftLimit(true);
    J3.overrideForwardSoftLimit(true);

     // J4 soft limits
    J4.configSoftLimits(J4.qMin, J4.qMax);
    J4.overrideReverseSoftLimit(true);
    J4.overrideForwardSoftLimit(true);

    // Pitch soft limits
    Pitch.configSoftLimits(Pitch.qMin, Pitch.qMax);
    Pitch.overrideReverseSoftLimit(true);
    Pitch.overrideForwardSoftLimit(true);

    // Roll
    Roll_PID.enableContinuousFeedback(0, 360);

    // Attach PID
    X.attachPID(&X_PID);
    J2.attachPID(&J2_PID);
    J3.attachPID(&J3_PID);
    J4.attachPID(&J4_PID);
    Pitch.attachPID(&Pitch_PID);
    Roll.attachPID(&Roll_PID);
    
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
        case RC_ARMBOARD_OPENLOOP_DATA_ID:{
            // initlaize openlood data
        }
    }
}