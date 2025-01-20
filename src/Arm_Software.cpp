#include "Arm_Software.h"

// 2025 REV 1

void setup() 
{
    Serial.begin(115200);
    Serial.println("Setup");

    // Button pins
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);

    pinMode(DIR_SW, INPUT); // pullup or pulldown needed?

    // IO expander pins
    IOX_TWI.begin();
    IOX1.begin();

    IOX2.begin(~uint8_t((1 << IOX2_FWD_1) | (1 << IOX2_RVS_1) | (1 << IOX2_FWD_2) | 
                        (1 << IOX2_RVS_2) | (1 << IOX2_FWD_3) | (1 << IOX2_RVS_3)));

    IOX3.begin(~uint8_t((1 << IOX3_FWD_4) | (1 << IOX3_RVS_4) | (1 << IOX3_FWD_5) | 
                 (1 << IOX3_RVS_5) | (1 << IOX3_FWD_6) | (1 << IOX3_RVS_6) | 
                 (1 << IOX3_FWD_7) | (1 << IOX3_RVS_7)));

    // Attach encoders
    X.attachEncoder(&XEncoder);
    J2.attachEncoder(&J2Encoder);
    J3.attachEncoder(&J3Encoder);
    J4.attachEncoder(&J4Encoder);
    Pitch.attachEncoder(&PitchEncoder);
    Roll.attachEncoder(&RollEncoder);

    // Attach hard limits
    X.attachHardLimits(&LS6, &LS9);
    J2.attachHardLimits(&LS10, &LS2);
    J3.attachHardLimits(&LS3, &LS8);
    J4.attachHardLimits(&LS5, &LS7);
    Pitch.attachHardLimits(&LS4, &LS4); //CHANGE which is fwd/rvs based on direction

    // Attach encoder inverts
    X.Encoder()->configInvert(false); //CHANGE
    J2.Encoder()->configInvert(true);
    J3.Encoder()->configInvert(true);
    J4.Encoder()->configInvert(false);
    Pitch.Encoder()->configInvert(false);
    Roll.Encoder()->configInvert(false);

    // Attach encoder offsets
    J2.Encoder()->configOffset(-121.82); //Subtract 360 if greater than 180
    J3.Encoder()->configOffset(346.03); //Subtract 360 if greater than 180
    J4.Encoder()->configOffset(280.46);
    Pitch.Encoder()->configOffset(262.44);

    // Configrue encoder interupts
    J2Encoder.begin([]{J2Encoder.handleInterrupt();});
    J3Encoder.begin([]{J3Encoder.handleInterrupt();});
    J4Encoder.begin([]{J4Encoder.handleInterrupt();});
    PitchEncoder.begin([]{PitchEncoder.handleInterrupt();});
    XEncoder.begin([]{XEncoder.handleInterrupt();});
    RollEncoder.begin([]{RollEncoder.handleInterrupt();});

    // Config motor inverts, reference joint
    X.Motor()->configInvert(false); //CHANGE
    J2.Motor()->configInvert(false);
    J3.Motor()->configInvert(false);
    J4.Motor()->configInvert(false);
    Pitch.Motor()->configInvert(false);
    Roll.Motor()->configInvert(true);
    Gripper.configInvert(false);
    Spare.configInvert(false);

    // Config motor output limits, reference joint
    X.Motor()->configMaxOutputs(-1000, 1000); //CHANGE
    J2.Motor()->configMaxOutputs(-1000, 1000);
    J3.Motor()->configMaxOutputs(-1000, 1000);
    J4.Motor()->configMaxOutputs(-1000, 1000);
    Pitch.Motor()->configMaxOutputs(-1000, 1000);
    Gripper.configMaxOutputs(-1000, 1000);
    Spare.configMaxOutputs(-1000, 1000);

    // Config motor deadbands, reference joint
    X.Motor()->configMinOutputs(-200, 200);    //CHANGE: PID deci% floor depending on arm config to prevent arm falling because of gravity
    J2.Motor()->configMinOutputs(-100, 170);  
    J3.Motor()->configMinOutputs(-100, 220);  
    J4.Motor()->configMinOutputs(-220, 190);  
    Pitch.Motor()->configMinOutputs(-50, 50); 
    Roll.Motor()->configMinOutputs(-200, 200);
    Gripper.configMinOutputs(-50, 50);       
    Spare.configMinOutputs(-50, 50);  

    // Config motor ramp rates, reference joint
    X.Motor()->configRampRate(10000); //CHANGE
    J2.Motor()->configRampRate(10000);
    J3.Motor()->configRampRate(10000);
    J4.Motor()->configRampRate(10000);
    Pitch.Motor()->configRampRate(10000);
    Roll.Motor()->configRampRate(10000);
    Gripper.configRampRate(10000);
    Spare.configRampRate(10000);

    // X soft limits
    X.configSoftLimits(X_REV_LIM, X_FWD_LIM);
    X.overrideReverseSoftLimit(true);
    X.overrideForwardSoftLimit(true);

    // J2 soft limits
    J2.configSoftLimits(J2_REV_LIM, J2_FWD_LIM);
    J2.overrideReverseSoftLimit(true);
    J2.overrideForwardSoftLimit(true);

    // J3 soft limits
    J3.configSoftLimits(J3_REV_LIM, J3_FWD_LIM);
    J3.overrideReverseSoftLimit(true);
    J3.overrideForwardSoftLimit(true);

    // J4 soft limits
    J4.configSoftLimits(J4_REV_LIM, J4_FWD_LIM);
    J4.overrideReverseSoftLimit(true);
    J4.overrideForwardSoftLimit(true);

    // Pitch soft limits
    Pitch.configSoftLimits(PITCH_REV_LIM, PITCH_FWD_LIM);
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
    RoveComm.begin(RC_ARMBOARD_IPADDRESS);
    Serial.println("Complete");

    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}

void loop() 
{
    // currentMode = CLOSED_LOOP;
    // RollState.qTarget = 90;

    uint32_t timestamp = millis();

    // Parse RoveComm packets
    static RoveCommPacket packet;
    RoveComm.read(packet);
    switch (packet.dataId) {
    case RC_ARMBOARD_OPENLOOP_DATA_ID:
    {
        // Set joint decipercent
        int16_t *data = (int16_t *)packet.data;
        XState.decipercent = data[0];
        J2State.decipercent = data[1];
        J3State.decipercent = data[2];
        J4State.decipercent = data[3];
        PitchState.decipercent = data[4];
        RollState.decipercent = data[5];

        currentMode = OPEN_LOOP;
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_SETPOSITION_DATA_ID:
    {
        float *data = (float*) packet.data;
        XState.qTarget = data[0];
        J2State.qTarget = data[1];
        J3State.qTarget = data[2];
        J4State.qTarget = data[3];
        PitchState.qTarget = data[4];
        RollState.qTarget = data[5];

        currentMode = CLOSED_LOOP;
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_INCREMENTPOSITION_DATA_ID:
    {
        float *data = (float*) packet.data;
        XState.qTarget += data[0];
        J2State.qTarget += data[1];
        J3State.qTarget += data[2];
        J4State.qTarget += data[3];
        PitchState.qTarget += data[4];
        RollState.qTarget += data[5];
        
        currentMode = CLOSED_LOOP;
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_LASER_DATA_ID:
    {
        uint8_t data = *((uint8_t *)packet.data);
        laserOn = (data == 0) ? false : true;
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_SOLENOID_DATA_ID:
    {
        uint8_t data = *((uint8_t *)packet.data);
        extendSolenoid = (data == 0) ? false : true;
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_GRIPPER_DATA_ID:
    {
        int16_t data = *((int16_t*) packet.data);
        GripperDecipercent = data;
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID:
    {
        watchdogOverride = *((uint8_t*) packet.data);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
    {
        uint16_t data = *((uint16_t*) packet.data);

        X.overrideForwardHardLimit(data & (1 << 0));
        X.overrideReverseHardLimit(data & (1 << 1));
        J2.overrideForwardHardLimit(data & (1 << 2));
        J2.overrideReverseHardLimit(data & (1 << 3));
        J3.overrideForwardHardLimit(data & (1 << 4));
        J3.overrideReverseHardLimit(data & (1 << 5));
        J4.overrideForwardHardLimit(data & (1 << 6));
        J4.overrideReverseHardLimit(data & (1 << 7));
        Pitch.overrideForwardHardLimit(data & (1 << 8));
        Pitch.overrideReverseHardLimit(data & (1 << 9));
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_SOFTLIMITOVERRIDE_DATA_ID:
    {
        uint16_t data = *((uint16_t*) packet.data);

        X.overrideForwardSoftLimit(data & (1 << 0));
        X.overrideReverseSoftLimit(data & (1 << 1));
        J2.overrideForwardSoftLimit(data & (1 << 2));
        J2.overrideReverseSoftLimit(data & (1 << 3));
        J3.overrideForwardSoftLimit(data & (1 << 4));
        J3.overrideReverseSoftLimit(data & (1 << 5));
        J4.overrideForwardSoftLimit(data & (1 << 6));
        J4.overrideReverseSoftLimit(data & (1 << 7));
        Pitch.overrideForwardSoftLimit(data & (1 << 8));
        Pitch.overrideReverseSoftLimit(data & (1 << 9));
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID:
    {
        uint8_t data = *((uint8_t*) packet.data);
        if(data & (1<<1)) Roll.Encoder()->setDegrees(0);
        Xcalibrating = data & (1 << 0);
        feedWatchdog();
        break;
    }
    }

    // IO Expanders
    if (timestamp - lastIOX_timestamp > IOX_UPDATE_PERIOD) {
        lastIOX_timestamp = timestamp;

        // IO Expander 1
        uint8_t iox1_val = IOX1.read8();
        LS1.set(iox1_val & (1 << IOX1_LIM_1));
        LS2.set(iox1_val & (1 << IOX1_LIM_2));
        LS3.set(iox1_val & (1 << IOX1_LIM_3));
        LS4.set(iox1_val & (1 << IOX1_LIM_4));
        LS5.set(iox1_val & (1 << IOX1_LIM_5));
        LS6.set(iox1_val & (1 << IOX1_LIM_6));
        LS7.set(iox1_val & (1 << IOX1_LIM_7));
        LS8.set(iox1_val & (1 << IOX1_LIM_8));

        // IO Expander 2
        uint8_t iox2_val = IOX2.read8();
        LS9.set(iox2_val & (1 << IOX2_LIM_9));
        LS10.set(iox2_val & (1 << IOX2_LIM_10));

    }

    direction = digitalRead(DIR_SW);
    buttonInput = (digitalRead(B_ENC_3) << 3) | (digitalRead(B_ENC_2) << 2) | (digitalRead(B_ENC_1) << 1) | (digitalRead(B_ENC_0) << 0);

    // Motor Outputs
    Serial.println();
    Serial.printf("X: ");
    updateJoint(X,XState,BTN_X);
    Serial.printf("J2: ");
    updateJoint(J2,J2State,BTN_J2);
    Serial.printf("J3: ");
    updateJoint(J3,J3State,BTN_J3);
    Serial.printf("J4: ");
    updateJoint(J4,J4State,BTN_J4);
    Serial.printf("Pt: ");
    updateJoint(Pitch,PitchState,BTN_PITCH);
    Serial.printf("Rl: ");
    updateJoint(Roll,RollState,BTN_ROLL);

    updateMotor(Gripper,GripperDecipercent,BTN_GRIPPER);
    updateMotor(Spare,SpareDecipercent,BTN_SPARE);

    // Solenoid
    if (buttonInput == BTN_SOL) setSolenoid(true);
    else setSolenoid(extendSolenoid);

    // Laser
    if (buttonInput == BTN_LAS) setLaser(true);
    else setLaser(laserOn);
    
}

void estop() 
{
    if (!watchdogOverride)
    {
        watchdogStatus = 1;

        XState.decipercent = 0;
        J2State.decipercent = 0;
        J3State.decipercent = 0;
        J4State.decipercent = 0;
        PitchState.decipercent = 0;
        RollState.decipercent = 0;
        GripperDecipercent = 0;
    }
}

void telemetry() 
{
    RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);
    
    if(!telemetryOverride) {
    float positions[6] = {X.Encoder()->readDegrees(), J2.Encoder()->readDegrees(), J3.Encoder()->readDegrees(), J4.Encoder()->readDegrees(), 
                          Pitch.Encoder()->readDegrees(), Roll.Encoder()->readDegrees()};

    RoveComm.write(RC_ARMBOARD_POSITIONS_DATA_ID, RC_ARMBOARD_POSITIONS_DATA_COUNT, positions);
    }

    //Add telemetry data as needed
}

void setSolenoid(bool extend)
{
    digitalWrite(Solenoid, extend? HIGH:LOW);
}

void setLaser(bool on)
{
    digitalWrite(LAS,on? HIGH:LOW);
}

void updateJoint(RoveJoint &joint, JointState &state, uint8_t button) 
{
    state.qMotor = joint.Encoder()->readDegrees();

    Serial.print(state.qMotor, 2);
    Serial.printf("    ");

    if(buttonInput == button){
        // override soft limits; drive joint @ default decipercent, turn soft limits back on
        joint.overrideReverseSoftLimit(true);
        joint.overrideForwardSoftLimit(true);
        joint.drive((direction? -900 : 900));
        joint.overrideReverseSoftLimit(false);
        joint.overrideForwardSoftLimit(false);
    } else if (Xcalibrating){
        // calibrates x joint only
        if(X.atReverseHardLimit()) {
            X.overrideReverseSoftLimit(false);
            X.overrideForwardSoftLimit(false);
            X.drive(0);
            X.Encoder()->setDegrees(0);
            Xcalibrating = false;
            Xcalibrated = true;
        } else {
            X.overrideReverseSoftLimit(true);
            X.overrideForwardSoftLimit(true);
            X.drive(-900);
        }
    } else if (currentMode == CLOSED_LOOP || currentMode == INVERSE_KINEMATICS) {
        if (Xcalibrated) joint.setAngle(state.qTarget);
        else joint.drive(0); //When drive(0), drive at floor deci% instead, make func, or just add to base deci%
    } else {
        joint.drive(state.decipercent);
    }
}

void updateMotor(RoveMotor &motor, int16_t decipercent, uint8_t button) 
{
    if (buttonInput == button) motor.drive((direction ? -900 : 900));
    else motor.drive(decipercent);
}

void feedWatchdog() 
{
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}

void HoldCurrentPosition() 
{
    //Calculate wristPos
    //Calculate gripperPos
    //Calculate target angles
    //Calculate spherical wrist pos
}