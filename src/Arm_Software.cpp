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
    X.attachHardLimits(&LS9, &LS6);
    J2.attachHardLimits(&LS2, &LS10);
    J3.attachHardLimits(&LS8, &LS3);
    J4.attachHardLimits(&LS7, &LS5);
    Pitch.attachHardLimits(&LS4, &LS4);

    // Attach encoder inverts
    X.Encoder()->configInvert(false);
    J2.Encoder()->configInvert(true);
    J3.Encoder()->configInvert(true);
    J4.Encoder()->configInvert(false);
    Pitch.Encoder()->configInvert(false);
    Roll.Encoder()->configInvert(false);

    J2.Encoder()->configNegativeDegrees(true);
    J3.Encoder()->configNegativeDegrees(true);

    // Attach encoder offsets
    J2.Encoder()->configOffset(-121.82);
    J3.Encoder()->configOffset(346.03);
    J4.Encoder()->configOffset(280.46); //280.46
    Pitch.Encoder()->configOffset(98.44); //262.44 //82.44

    // Configrue encoder interupts
    J2Encoder.begin([]{J2Encoder.handleInterrupt();});
    J3Encoder.begin([]{J3Encoder.handleInterrupt();});
    J4Encoder.begin([]{J4Encoder.handleInterrupt();});
    PitchEncoder.begin([]{PitchEncoder.handleInterrupt();});
    XEncoder.begin([]{XEncoder.handleInterrupt();});
    RollEncoder.begin([]{RollEncoder.handleInterrupt();});

    // Config motor inverts, reference joint
    X.Motor()->configInvert(true);
    J2.Motor()->configInvert(true);
    J3.Motor()->configInvert(true);
    J4.Motor()->configInvert(true);
    Pitch.Motor()->configInvert(true);
    Roll.Motor()->configInvert(false);
    Gripper.configInvert(true);
    Spare.configInvert(true);

    // Config motor output limits, reference joint
    X.Motor()->configMaxOutputs(-1000, 1000);
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
    Roll.Motor()->configMinOutputs(-50, 50);
    Gripper.configMinOutputs(-50, 50);       
    Spare.configMinOutputs(-50, 50);  

    // Config motor ramp rates, reference joint
    X.Motor()->configRampRate(10000);
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
    J4.overrideReverseSoftLimit(false);
    J4.overrideForwardSoftLimit(false);

    // Pitch soft limits
    Pitch.configSoftLimits(PITCH_REV_LIM, PITCH_FWD_LIM);
    Pitch.overrideReverseSoftLimit(false);
    Pitch.overrideForwardSoftLimit(false);

    Roll_PID.enableContinuousFeedback(0, 360);
    J4_PID.configOutputLimits(-1023, 1023); //PID can calculate a too high deci%, limit deci% to +/-1023
    J4_PID.configIZone(10);
    Pitch_PID.configIZone(10);

    J4_PID.configOffset(-90);
    Pitch_PID.configOffset(90);

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

    // J4.overrideForwardHardLimit(true);
    // J4.overrideReverseHardLimit(true);
}

void loop() 
{   
    Xcalibrated = true;

    if ((PitchState.qMotor < 270) && (PitchState.qMotor > 90)) {
        Pitch.overrideForwardHardLimit(true);
        Pitch.overrideReverseHardLimit(false);
    } else {
        Pitch.overrideForwardHardLimit(false);
        Pitch.overrideReverseHardLimit(true);
    }

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
    // Serial.printf("X: ");
    // updateJoint(X,XState,BTN_X);
    // Serial.printf("J2: ");
    // updateJoint(J2,J2State,BTN_J2);
    // Serial.printf("J3: ");
    // updateJoint(J3,J3State,BTN_J3);
    Serial.printf("J4: ");
    updateJoint(J4,J4State,BTN_J4);
    Serial.printf("Pt: ");
    updateJoint(Pitch,PitchState,BTN_PITCH);
    // Serial.printf("Rl: ");
    // updateJoint(Roll,RollState,BTN_ROLL);

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

    bool isMoving = true;

    Serial.printf(" m: ");
    Serial.print(state.qMotor, 2);
    Serial.printf(" t: ");
    Serial.print(state.qTarget, 2);
    Serial.printf(" d: ");
    Serial.print(state.decipercent, 10);
    

    if(buttonInput == button){
        joint.drive((direction? 900 : -900));
    } else if (Xcalibrating){
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
    } else if ((currentMode == CLOSED_LOOP || currentMode == INVERSE_KINEMATICS) && Xcalibrated) {
        joint.setAngle(state.qTarget);
    } else if (state.decipercent != 0) {
        joint.drive(state.decipercent);
    } else {
        isMoving = false;
    }

    if (!overrideGravityFighting) {
        if (isMoving) {
            state.hasMoved = true;
            state.qHold = state.qMotor;
        } else {
            if (state.hasMoved) {
                joint.setAngle(state.qHold);
                // Serial.printf(" h: ");
                // Serial.print(state.qHold, 2);
            }
        }
    }
    // Serial.printf(" SL: ");
    // Serial.print(joint.atForwardSoftLimit(), 1);
    // Serial.print(joint.atReverseSoftLimit(), 1);
    // Serial.printf("    ");
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

    XState.qTarget = XState.qMotor;
    J2State.qTarget = J2State.qMotor;
    J3State.qTarget = J3State.qMotor;
    J4State.qTarget = J4State.qMotor;
    PitchState.qTarget = PitchState.qMotor;
    RollState.qTarget = RollState.qMotor;
}