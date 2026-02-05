#include "Arm_Software.h"

#include <Arduino.h>

// 2026 DEV

void setup() 
{

    Serial.begin(115200);
    while(!Serial);
    Serial.println("Setup");

    // Button pins
    pinMode(BTN_1, INPUT_PULLUP);
    pinMode(BTN_2, INPUT_PULLUP);
    pinMode(BTN_3, INPUT_PULLUP);
    pinMode(BTN_4, INPUT_PULLUP);
    pinMode(BTN_5, INPUT_PULLUP);
    pinMode(BTN_6, INPUT_PULLUP);
    pinMode(BTN_7, INPUT_PULLUP);
    pinMode(BTN_8, INPUT_PULLUP);
    pinMode(BTN_1_SERVO, INPUT_PULLUP);
    pinMode(BTN_2_SERVO, INPUT_PULLUP);
    pinMode(BTN_3_SERVO, INPUT_PULLUP);
    pinMode(BTN_4_SERVO, INPUT_PULLUP);
    pinMode(BTN_LASER, INPUT_PULLUP);
    pinMode(BTN_LIN_SERVO, INPUT_PULLUP);
    pinMode(DIR_SW, INPUT_PULLUP);

    LinearServo.attach(LINEAR_SERVO);
    CameraOnePan.attach(SERVO_1);
    CameraOneTilt.attach(SERVO_2);
    CameraTwoPan.attach(SERVO_3);
    CameraTwoTilt.attach(SERVO_4);
    // CacheServo.attach(nullptr); NOT RIGHT NUMBER

    CAN_CHANNEL.begin(ACAN_T4_Settings{125'000});

    // delay(1000); To let settings and serial connect before sending all initial smoco configs over CAN
    // while(!Serial)

    //Set Low pass smoothing factor
    xMotor.setLowPassSmoothingFactor(INT16_MAX);
    J2Motor.setLowPassSmoothingFactor(INT16_MAX);
    J3Motor.setLowPassSmoothingFactor(INT16_MAX);
    J4Motor.setLowPassSmoothingFactor(INT16_MAX);
    PitchMotor.setLowPassSmoothingFactor(INT16_MAX);
    RollMotor.setLowPassSmoothingFactor(INT16_MAX);
    GripperMotor.setLowPassSmoothingFactor(INT16_MAX);

    //Set PID gains
    xMotor.setPID(1, 0, 0);
    J2Motor.setPID(1, 0, 0);
    J3Motor.setPID(1, 0, 0);
    J4Motor.setPID(1, 0, 0);
    PitchMotor.setPID(1, 0, 0);
    RollMotor.setPID(1, 0, 0); 
    GripperMotor.setPID(1, 0, 0);

    //Set soft limits
    xMotor.setSoftLimitPosition(X_REV_LIM, X_FWD_LIM);
    J2Motor.setSoftLimitPosition(J2_REV_LIM, J2_FWD_LIM);
    J3Motor.setSoftLimitPosition(J3_REV_LIM, J3_FWD_LIM);
    J4Motor.setSoftLimitPosition(J4_REV_LIM, J4_FWD_LIM);
    PitchMotor.setSoftLimitPosition(PITCH_REV_LIM, PITCH_FWD_LIM);
    RollMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    GripperMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);

    // RoveComm
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_IPADDRESS);
    Serial.println("Complete");

    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}

void loop() 
{   
    UpdateFromRoveComm();
    receiveCANMessages();
    UpdateArm();
}

void estop() 
{
    if (!watchdogOverride)
    {
        watchdogStatus = 1;

        xMotor.stopAndReset();    
        J2Motor.stopAndReset();
        J3Motor.stopAndReset();
        J4Motor.stopAndReset();
        PitchMotor.stopAndReset();
        RollMotor.stopAndReset();
        GripperMotor.stopAndReset();

        IKMode = false;

    }
}

void telemetry() 
{
    // RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);
    
    if(!telemetryOverride) {

        float positions[7] = {
            xMotor.m_position,
            J2Motor.m_position,
            J3Motor.m_position,
            J4Motor.m_position,
            PitchMotor.m_position,
            RollMotor.m_position,
            (PitchMotor.m_position * cosf(J4Motor.m_position*DEG2RAD)) + (J2Motor.m_position + J3Motor.m_position)
        };
        // Serial.println();
        // Serial.print(positions[6]);
        RoveComm.write(RC_ARMBOARD_POSITION_DATA_ID, RC_ARMBOARD_POSITION_DATA_COUNT, positions);

        uint16_t limitsTriggered = 0;
        if(xMotor.m_limitSwitchA) limitsTriggered |= (1 << 0);
        if(xMotor.m_limitSwitchB) limitsTriggered |= (1 << 1);

        if(J2Motor.m_limitSwitchA) limitsTriggered |= (1 << 2);
        if(J2Motor.m_limitSwitchB) limitsTriggered |= (1 << 3);

        if(J3Motor.m_limitSwitchA) limitsTriggered |= (1 << 4);
        if(J3Motor.m_limitSwitchB) limitsTriggered |= (1 << 5);

        if(J4Motor.m_limitSwitchA) limitsTriggered |= (1 << 6);
        if(J4Motor.m_limitSwitchB) limitsTriggered |= (1 << 7);

        if(PitchMotor.m_limitSwitchA) limitsTriggered |= (1 << 8);
        if(PitchMotor.m_limitSwitchB) limitsTriggered |= (1 << 9);
        RoveComm.write(RC_ARMBOARD_LIMITSWITCH_DATA_ID, RC_ARMBOARD_LIMITSWITCH_DATA_COUNT, &limitsTriggered);

        uint16_t softLimitsTriggered = 0;
        if(xMotor.m_softLimitAPosition) softLimitsTriggered |= (1 << 0);
        if(xMotor.m_softLimitBPosition) softLimitsTriggered |= (1 << 1);

        if(J2Motor.m_softLimitAPosition) softLimitsTriggered |= (1 << 2);
        if(J2Motor.m_softLimitBPosition) softLimitsTriggered |= (1 << 3);

        if(J3Motor.m_softLimitAPosition) softLimitsTriggered |= (1 << 4);
        if(J3Motor.m_softLimitBPosition) softLimitsTriggered |= (1 << 5);

        if(J4Motor.m_softLimitAPosition) softLimitsTriggered |= (1 << 6);
        if(J4Motor.m_softLimitBPosition) softLimitsTriggered |= (1 << 7);

        if(PitchMotor.m_softLimitAPosition) softLimitsTriggered |= (1 << 8);
        if(PitchMotor.m_softLimitBPosition) softLimitsTriggered |= (1 << 9);
        RoveComm.write(RC_ARMBOARD_SOFTLIMIT_DATA_ID, RC_ARMBOARD_SOFTLIMIT_DATA_COUNT, &softLimitsTriggered);

        /*
        //xMotor.ping();

        uint16_t pingData[6] = {0,0,0,0,0,0};
        pingData[0] = xMotor.m_pingTime;


        RoveComm.write(RC_ARMBOARD_SMOCOPING_DATA_ID,RC_ARMBOARD_SMOCOPING_DATA_COUNT,pingData);
        Serial.println("pingdata sent !");
        Serial.println(xMotor.m_pingTime);
        */
    }

    //Add telemetry data as needed
}

void setLaser(bool on) { digitalWrite(LASER,on? HIGH:LOW); }

void feedWatchdog() 
{
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}

void UpdateFromRoveComm()
{
    static RoveCommPacket packet;
    RoveComm.read(packet);

    switch (packet.dataId) {
        case RC_ARMBOARD_OPENLOOP_DATA_ID:
        {
            int16_t *packetData = (int16_t*) packet.data;

            // Set motor motorState control mode to open loop
            // if m_currentMode != OPEN_LOOP m_resendParameters = false;

            XState.m_resendParameters = (XState.m_currentMode  == MotorState::OPEN_LOOP) ? true : false;
            J2State.m_resendParameters = (J2State.m_currentMode == MotorState::OPEN_LOOP) ? true : false;
            J3State.m_resendParameters = (J3State.m_currentMode == MotorState::OPEN_LOOP) ? true : false;
            J4State.m_resendParameters = (J4State.m_currentMode == MotorState::OPEN_LOOP) ? true : false;
            PitchState.m_resendParameters = (PitchState.m_currentMode == MotorState::OPEN_LOOP) ? true : false;
            RollState.m_resendParameters = (RollState.m_currentMode == MotorState::OPEN_LOOP) ? true : false;


            XState.m_currentMode = MotorState::OPEN_LOOP;
            J2State.m_currentMode = MotorState::OPEN_LOOP;
            J3State.m_currentMode = MotorState::OPEN_LOOP;
            J4State.m_currentMode = MotorState::OPEN_LOOP;
            PitchState.m_currentMode = MotorState::OPEN_LOOP;
            RollState.m_currentMode = MotorState::OPEN_LOOP;

            xMotor.m_dutyCycle = packetData[0]; //limit switch comes from basestation
            J2Motor.m_dutyCycle = packetData[1];
            J3Motor.m_dutyCycle = packetData[2];
            J4Motor.m_dutyCycle = packetData[3];
            PitchMotor.m_dutyCycle = packetData[4];
            RollMotor.m_dutyCycle = packetData[5];

            // XState.setDutyCycle(packetData[0]);
            // J2State.setDutyCycle(packetData[1]);
            // J3State.setDutyCycle(packetData[2]);
            // J4State.setDutyCycle(packetData[3]);
            // PitchState.setDutyCycle(packetData[4]);
            // RollState.setDutyCycle(packetData[5]);

            //XState.setControlMode(0);
            //J2State.setControlMode(0);
            //J3State.setControlMode(0);
            //J4State.setControlMode(0);
            //PitchState.setControlMode(0);
            //RollState.setControlMode(0);

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_TARGETANGLE_DATA_ID:
        {
            float *packetData = (float*) packet.data; //Float not i32?
            //XState.m_targetAng(packetData[0]);
            //J2State.setTargetAngle(packetData[1]);
            //J3State.setTargetAngle(packetData[2]);
            //J4State.setTargetAngle(packetData[3]);
            //PitchState.setTargetAngle(packetData[4]);
            //RollState.setTargetAngle(packetData[5]);

            XState.m_currentMode = MotorState::TARGET_ANGLE;
            J2State.m_currentMode = MotorState::TARGET_ANGLE;
            J3State.m_currentMode = MotorState::TARGET_ANGLE;
            J4State.m_currentMode = MotorState::TARGET_ANGLE;
            PitchState.m_currentMode = MotorState::TARGET_ANGLE;
            RollState.m_currentMode = MotorState::TARGET_ANGLE;

            XState.m_targetAngleInDegrees = packetData[0];
            J2State.m_targetAngleInDegrees = packetData[1];
            J3State.m_targetAngleInDegrees = packetData[2];
            J4State.m_targetAngleInDegrees = packetData[3];
            PitchState.m_targetAngleInDegrees = packetData[4];
            RollState.m_targetAngleInDegrees = packetData[5];

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_GRIPPEROPENLOOP_DATA_ID: 
        {
            int16_t *packetData = (int16_t*) packet.data;
            GripperDutyCycle = packetData[0];

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_IKPOSITION_DATA_ID:
        {           
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_LINEARSERVO_DATA_ID:
        {
            uint8_t packetData = *((uint8_t *)packet.data);
            linearServoTarget = packetData;
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_LASER_DATA_ID: 
        {
            uint8_t packetData = *((uint8_t *)packet.data);
            laserOn = (packetData == 0) ? false : true;
            /*
            if(laserOn){
                xMotor.ping();
            }
            */
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_CACHE_DATA_ID:
        {
            uint8_t packetData = *((uint8_t *)packet.data);
            
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
            uint16_t packetData = *((uint16_t *)packet.data);
            
            //x+ data & (1 << 0) 
            //x- data & (1 << 1)
            //j2+ data & (1 << 2)
            //j2- data & (1 << 3)
            //j3+ data & (1 << 4)
            //j3- data & (1 << 5)
            //j4+ data & (1 << 6)
            //j4- data & (1 << 7)
            //p+ data & (1 << 8)
            //p- data & (1 << 9)

            // Switch byte order if it doesn't work; pretty sure most-significant bit is X+
            xMotor.m_ignoreLimit = (packetData & (1 << 0) || packetData & (1 << 1));
            J2Motor.m_ignoreLimit = (packetData & (1 << 2) || packetData & (1 << 3));
            J3Motor.m_ignoreLimit = (packetData & (1 << 4) || packetData & (1 << 5));
            J4Motor.m_ignoreLimit = (packetData & (1 << 6) ||  packetData & (1 << 7));
            PitchMotor.m_ignoreLimit = (packetData & (1 << 8) || packetData & (1 << 9));

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_CLOSEDLOOPOVERRIDE_DATA_ID:
        {
            uint8_t packetData = *((uint8_t *)packet.data);

            //x data & (1 << 0)
            //j2 data & (1 << 1)
            //j3 data & (1 << 2)
            //j4 data & (1 << 3)
            //p data & (1 << 4)
            //r data & (1 << 5)
            //XState.setClosedLoopOverride(packetData & (1 << 0));
            //J2State.setClosedLoopOverride(packetData & (1 << 1));
            //J3State.setClosedLoopOverride(packetData & (1 << 2));
            //J4State.setClosedLoopOverride(packetData & (1 << 3));
            //PitchState.setClosedLoopOverride(packetData & (1 << 4));
            //RollState.setClosedLoopOverride(packetData & (1 << 5));

            XState.m_closedLoopOverride = packetData & (1 << 0);
            J2State.m_closedLoopOverride = packetData & (1 << 0);
            J3State.m_closedLoopOverride = packetData & (1 << 0);
            J4State.m_closedLoopOverride = packetData & (1 << 0);
            PitchState.m_closedLoopOverride = packetData & (1 << 0);
            RollState.m_closedLoopOverride = packetData & (1 << 0);

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID:
        {
            uint8_t packetData = *((uint8_t *)packet.data);

            //x data & (1 << 0)
            //roll data & (1 << 1)

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SOFTLIMITOVERRIDE_DATA_ID:
        {
            uint16_t packetData = *((uint16_t *)packet.data);

            //x+ data & (1 << 0) 
            //x- data & (1 << 1)
            //j2+ data & (1 << 2)
            //j2- data & (1 << 3)
            //j3+ data & (1 << 4)
            //j3- data & (1 << 5)
            //j4+ data & (1 << 6)
            //j4- data & (1 << 7)
            //p+ data & (1 << 8)
            //p- data & (1 << 9)
            xMotor.m_softLimitAPosition = (packetData & (1 << 0) ? INT32_MIN : X_REV_LIM);
            xMotor.m_softLimitBPosition = (packetData & (1 << 1) ? INT32_MAX : X_FWD_LIM);

            J2Motor.m_softLimitAPosition = (packetData & (1 << 2) ? INT32_MIN : J2_REV_LIM);
            J2Motor.m_softLimitBPosition = (packetData & (1 << 3) ? INT32_MAX : J2_FWD_LIM);

            J3Motor.m_softLimitAPosition = (packetData & (1 << 4) ? INT32_MIN : J3_REV_LIM);
            J3Motor.m_softLimitBPosition = (packetData & (1 << 5) ? INT32_MAX : J3_FWD_LIM);

            J4Motor.m_softLimitAPosition = (packetData & (1 << 6) ? INT32_MIN : J4_REV_LIM);
            J4Motor.m_softLimitBPosition = (packetData & (1 << 7) ? INT32_MAX : J4_FWD_LIM);

            PitchMotor.m_softLimitAPosition = (packetData & (1 << 8) ? INT32_MIN : PITCH_REV_LIM);
            PitchMotor.m_softLimitBPosition = (packetData & (1 << 9) ? INT32_MAX : PITCH_FWD_LIM);

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_ARMGIMBAL1_DATA_ID:
        {
            int16_t *packetData = (int16_t*) packet.data;

            CameraOnePan.write(packetData[0]);
            CameraOneTilt.write(packetData[1]);

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_ARMGIMBAL2_DATA_ID:
        {
            int16_t *packetData = (int16_t*) packet.data;

            CameraTwoPan.write(packetData[0]);
            CameraTwoTilt.write(packetData[1]);

            feedWatchdog();
            break;
        }

        // Telemetry packets
        case RC_ARMBOARD_POSITION_DATA_ID:
        {

            break;
        }
        case RC_ARMBOARD_LIMITSWITCH_DATA_ID:
        {
            break;
        }
        case RC_ARMBOARD_SOFTLIMIT_DATA_ID:
        {
            break;
        }
        case RC_ARMBOARD_SMOCOPING_DATA_ID:
        {
            break;
        }

    }

}

void UpdateArm()
{
    direction = digitalRead(DIR_SW);

    XState.updateMotor(digitalRead(BTN_LASER) == false, direction);
    J2State.updateMotor(digitalRead(BTN_2), direction);
    J3State.updateMotor(digitalRead(BTN_3), direction);
    J4State.updateMotor(digitalRead(BTN_4), direction);
    PitchState.updateMotor(digitalRead(BTN_5), direction);
    RollState.updateMotor(digitalRead(BTN_6), direction);

    if(digitalRead(BTN_7)) GripperMotor.driveOpenLoop(direction ? -900 : 900, false);
    else GripperMotor.driveOpenLoop(GripperDutyCycle, false);

    // Linear Servo
    if (digitalRead(BTN_LIN_SERVO)) LinearServo.write(direction ? 180 : 0);
    else LinearServo.write(linearServoTarget);

    // Laser
    /*if (digitalRead(BTN_LASER)) setLaser(true);
    else setLaser(laserOn);
    */
}

void receiveCANMessages() {
    /*
    CANMessage receivedMessage;
    while(CAN_CHANNEL.available()){
    CAN_CHANNEL.receive(receivedMessage);
Serial.printf("Received CAN packet with ID %d\n", receivedMessage.id);
        if (0) {
            Serial.println("Sending packet to VESC");
            //vesc_process_can_frame(msg.id, msg.data, msg.len);
        } else {
            Serial.printf("Sending packet to Smoco (%d)\n", receivedMessage.id & 0xF);
            if ((receivedMessage.id & 0xF) == 13) {
                Serial.printf("ERROR:::%d:::\n", ((SmocoCANMessage *)receivedMessage.data)->commandError.commandID);
            }
    xMotor.sync(receivedMessage);
    */
    
    /*

    switch (receivedMessage.id >> 8) {
        case X_ID:
            xMotor.sync(receivedMessage);
            break;
        case J2_ID:
            J2Motor.sync(receivedMessage);
            break;
        case J3_ID:
            J3Motor.sync(receivedMessage);
            break;
        case J4_ID:
            J4Motor.sync(receivedMessage);
            break;
        case PITCH_ID:
            PitchMotor.sync(receivedMessage);
            break;
        case ROLL_ID:
            RollMotor.sync(receivedMessage);
            break;
        case GRIPPER_ID:
            GripperMotor.sync(receivedMessage);
            break;
        default:
            break;
    }
            */
}