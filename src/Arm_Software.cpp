#include "Arm_Software.h"

#include <Arduino.h>

template <typename... Args> unsigned long long bitmask(Args... as) {
    using unused = int[];
    unsigned long long ret{0ull};
    (void)unused{0, (ret >>= 1, ret |= (as ? 1ull : 0ull) << (sizeof...(as) - 1), 0)...};
    return ret;
}

void setup() {

    Serial.begin(115200);
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

    ACAN_T4_Settings settings{SMOCO_CAN_BAUD_RATE};
    CAN_CHANNEL.begin(settings);

    // delay(1000); To let settings and serial connect before sending all initial smoco configs over CAN
    // while(!Serial)
    // delay(3000); // wait for smocos to start up

    // Set PID gains
    XMotor.setPID(0.7, 0, 0);
    J2Motor.setPID(0.7, 0, 0);
    J3Motor.setPID(0.7, 0, 0);
    J4Motor.setPID(0.7, 0, 0);
    PitchMotor.setPID(0.7, 0, 0);
    RollMotor.setPID(0.7, 0, 0);
    GripperMotor.setPID(0.7, 0, 0);

    // Set soft limits
    XMotor.setSoftLimitPosition(X_REV_LIM, X_FWD_LIM);
    J2Motor.setSoftLimitPosition(J2_REV_LIM, J2_FWD_LIM);
    J3Motor.setSoftLimitPosition(J3_REV_LIM, J3_FWD_LIM);
    J4Motor.setSoftLimitPosition(J4_REV_LIM, J4_FWD_LIM);
    PitchMotor.setSoftLimitPosition(PITCH_REV_LIM, PITCH_FWD_LIM);
    RollMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    GripperMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);

    // Set ramp rates
    XMotor.setRampRate(100.0);
    J2Motor.setRampRate(100.0);
    J3Motor.setRampRate(100.0);
    J4Motor.setRampRate(100.0);
    PitchMotor.setRampRate(100.0);
    RollMotor.setRampRate(100.0);
    GripperMotor.setRampRate(100.0);

    // RoveComm
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_IPADDRESS);
    Serial.println("Complete");

    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}

void loop() {
    updateFromRoveComm();
    receiveCANMessages();
    updateArm();
}

void estop() {
    if (!watchdogOverride) {
        watchdogStatus = 1;

        XMotor.driveOpenLoop(0);
        J2Motor.driveOpenLoop(0);
        J3Motor.driveOpenLoop(0);
        J4Motor.driveOpenLoop(0);
        PitchMotor.driveOpenLoop(0);
        RollMotor.driveOpenLoop(0);
        GripperMotor.driveOpenLoop(0);
    }
}

void telemetry() {
    // RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);

    if (!telemetryOverride) {
        // TODO: calculate in and deg from encoder
        float positions[8] = {
            encToDeg(XMotor.getPosition(), 0, X_ENC_PER_IN),
            encToDeg(J2Motor.getPosition(), J2_ZERO, J2_ENC_PER_DEG),
            encToDeg(J3Motor.getPosition(), J3_ZERO, J3_ENC_PER_DEG),
            encToDeg(J4Motor.getPosition(), J4_ZERO, J4_ENC_PER_DEG),
            encToDeg(PitchMotor.getPosition(), PITCH_ZERO, PITCH_ENC_PER_DEG),
            encToDeg(RollMotor.getPosition(), rollZero, ROLL_ENC_PER_DEG),
            0, // TODO: calculate Y
            0, // TODO: calculate Z
        };
        RoveComm.write(RC_ARMBOARD_POSITION_DATA_ID, RC_ARMBOARD_POSITION_DATA_COUNT, positions);

        uint16_t limitsTriggered = bitmask(
            XMotor.getLimitSwitchA(), XMotor.getLimitSwitchB(), J2Motor.getLimitSwitchA(), J2Motor.getLimitSwitchB(),
            J3Motor.getLimitSwitchA(), J3Motor.getLimitSwitchB(), J4Motor.getLimitSwitchA(), J4Motor.getLimitSwitchB(),
            PitchMotor.getLimitSwitchA(), PitchMotor.getLimitSwitchB());
        RoveComm.write(RC_ARMBOARD_LIMITSWITCH_DATA_ID, RC_ARMBOARD_LIMITSWITCH_DATA_COUNT, &limitsTriggered);

        uint16_t softLimitsTriggered =
            bitmask(XMotor.getSoftLimitA(), XMotor.getSoftLimitB(), J2Motor.getSoftLimitA(), J2Motor.getSoftLimitB(),
                    J3Motor.getSoftLimitA(), J3Motor.getSoftLimitB(), J4Motor.getSoftLimitA(), J4Motor.getSoftLimitB(),
                    PitchMotor.getSoftLimitA(), PitchMotor.getSoftLimitB());
        RoveComm.write(RC_ARMBOARD_SOFTLIMIT_DATA_ID, RC_ARMBOARD_SOFTLIMIT_DATA_COUNT, &softLimitsTriggered);

        XMotor.ping();
        J2Motor.ping();
        J3Motor.ping();
        J4Motor.ping();
        PitchMotor.ping();
        RollMotor.ping();
        GripperMotor.ping();

        uint16_t pingData[7] = {XMotor.getPingTime(),      J2Motor.getPingTime(),    J3Motor.getPingTime(),
                                J4Motor.getPingTime(),     PitchMotor.getPingTime(), RollMotor.getPingTime(),
                                GripperMotor.getPingTime()};
        RoveComm.write(RC_ARMBOARD_SMOCOPING_DATA_ID, RC_ARMBOARD_SMOCOPING_DATA_COUNT, pingData);
    }

    // Add telemetry data as needed
}

void setLaser(bool on) { digitalWrite(LASER, on ? HIGH : LOW); }

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}

void updateFromRoveComm() {
    static RoveCommPacket packet;
    RoveComm.read(packet);

    switch (packet.dataId) {
    case RC_ARMBOARD_OPENLOOP_DATA_ID: {
        // Set motor motorState control mode to open loop
        // if m_currentMode != OPEN_LOOP m_resendParameters = false;

        XMotor.driveOpenLoop(packet.i16data[0]);
        J2Motor.driveOpenLoop(packet.i16data[1]);
        J3Motor.driveOpenLoop(packet.i16data[2]);
        J4Motor.driveOpenLoop(packet.i16data[3]);
        PitchMotor.driveOpenLoop(packet.i16data[4]);
        RollMotor.driveOpenLoop(packet.i16data[5]);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_TARGETANGLE_DATA_ID: {

        XMotor.driveTargetPosition(degToEnc(packet.fdata[0], 0, X_ENC_PER_IN), 0.05 * 1024);
        J2Motor.driveTargetPosition(degToEnc(packet.fdata[1], J2_ZERO, J2_ENC_PER_DEG), 0.05 * 1024);
        J3Motor.driveTargetPosition(degToEnc(packet.fdata[2], J3_ZERO, J3_ENC_PER_DEG), 0.05 * 1024);
        J4Motor.driveTargetPosition(degToEnc(packet.fdata[3], J4_ZERO, J4_ENC_PER_DEG), 0.05 * 1024);
        PitchMotor.driveTargetPosition(degToEnc(packet.fdata[4], PITCH_ZERO, PITCH_ENC_PER_DEG), 0.05 * 1024);
        RollMotor.driveTargetPosition(degToEnc(packet.fdata[5], rollZero, ROLL_ENC_PER_DEG), 0.05 * 1024);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_GRIPPEROPENLOOP_DATA_ID: {
        GripperMotor.driveOpenLoop(packet.i16data[0]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_IKPOSITION_DATA_ID: {
        // TODO
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_LINEARSERVO_DATA_ID: {
        LinearServo.write(packet.i8data[0]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_LASER_DATA_ID: {
        setLaser(packet.u8data[0]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_CACHE_DATA_ID: {
        uint8_t packetData = *((uint8_t *)packet.data);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID: {
        watchdogOverride = *((uint8_t *)packet.data);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID: {

        // not effective until next drive command!
        int16_t limits = packet.i16data[0];
        XMotor.configIgnoreLimits(limits & (1 << 0), limits & (1 << 1));
        J2Motor.configIgnoreLimits(limits & (1 << 2), limits & (1 << 3));
        J3Motor.configIgnoreLimits(limits & (1 << 4), limits & (1 << 5));
        J4Motor.configIgnoreLimits(limits & (1 << 6), limits & (1 << 7));
        PitchMotor.configIgnoreLimits(limits & (1 << 8), limits & (1 << 9));
        break;
    }
    case RC_ARMBOARD_CLOSEDLOOPOVERRIDE_DATA_ID: {

        // TODO: State dependent (applies to IK mode, makes certain axes run in openloop with speed 0)

        // x data & (1 << 0)
        // j2 data & (1 << 1)
        // j3 data & (1 << 2)
        // j4 data & (1 << 3)
        // p data & (1 << 4)
        // r data & (1 << 5)

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID: {

        // x data & (1 << 0)
        // roll data & (1 << 1)
        if (packet.u8data[0] & (1 << 0)) {
            XMotor.calibratePosition(-INT16_MAX / 2, 0);
        } else if (packet.u8data[0] & (1 << 1)) {
            rollZero = RollMotor.getPosition();
        }

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_SOFTLIMITOVERRIDE_DATA_ID: {
        uint16_t packetData = packet.i16data[0];
        XMotor.setSoftLimitPosition(packetData & (1 << 0) ? INT32_MIN : X_REV_LIM,
                                    packetData & (1 << 1) ? INT32_MAX : X_FWD_LIM);
        J2Motor.setSoftLimitPosition(packetData & (1 << 2) ? INT32_MIN : X_REV_LIM,
                                     packetData & (1 << 3) ? INT32_MAX : J2_FWD_LIM);
        J3Motor.setSoftLimitPosition(packetData & (1 << 4) ? INT32_MIN : X_REV_LIM,
                                     packetData & (1 << 5) ? INT32_MAX : J3_FWD_LIM);
        J4Motor.setSoftLimitPosition(packetData & (1 << 6) ? INT32_MIN : J4_REV_LIM,
                                     packetData & (1 << 7) ? INT32_MAX : J4_FWD_LIM);
        PitchMotor.setSoftLimitPosition(packetData & (1 << 8) ? INT32_MIN : PITCH_REV_LIM,
                                        packetData & (1 << 9) ? INT32_MAX : PITCH_FWD_LIM);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_ARMGIMBAL1_DATA_ID: {
        CameraOnePan.write(packet.i16data[0]);
        CameraOneTilt.write(packet.i16data[1]);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_ARMGIMBAL2_DATA_ID: {
        CameraTwoPan.write(packet.i16data[0]);
        CameraTwoTilt.write(packet.i16data[1]);

        feedWatchdog();
        break;
    }
    }
}

void updateArm() {
    /*direction = digitalRead(DIR_SW);

    if (digitalRead(BTN_7)) GripperMotor.driveOpenLoop(direction ? -900 : 900, false);

    // Linear Servo
    if (digitalRead(BTN_LIN_SERVO)) LinearServo.write(direction ? 180 : 0);

    // Laser
    if (digitalRead(BTN_LASER))
        setLaser(true);
    else
        setLaser(laserOn);*/
}

void receiveCANMessages() {
    CANMessage receivedMessage;
    while (CAN_CHANNEL.available()) {
        if (CAN_CHANNEL.receive(receivedMessage)) {
            // Serial.printf("ID %x CMD %x RTR %s LEN %d\n", receivedMessage.id >> 4, receivedMessage.id & 0xF, receivedMessage.rtr ? "R" : "D", receivedMessage.len);
            XMotor.sync(receivedMessage);
            J2Motor.sync(receivedMessage);
            J3Motor.sync(receivedMessage);
            J4Motor.sync(receivedMessage);
            PitchMotor.sync(receivedMessage);
            RollMotor.sync(receivedMessage);
            GripperMotor.sync(receivedMessage);
        }
    }
}

float encToDeg(int32_t enc, int32_t encZero, float encPerDeg, bool reversed) {
    return (enc - encZero) / encPerDeg * (reversed ? -1 : 1);
}

int32_t degToEnc(float deg, int32_t encZero, float encPerDeg, bool reversed) {
    return (reversed ? -1 : 1) * (deg * encPerDeg) + encZero;
}
