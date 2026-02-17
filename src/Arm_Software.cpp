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

    // Set angle conversions
    XMotor.configAngleConversion(0, X_ENC_PER_IN);
    J2Motor.configAngleConversion(J2_ZERO, J2_ENC_PER_DEG);
    J3Motor.configAngleConversion(J3_ZERO, J3_ENC_PER_DEG);
    J4Motor.configAngleConversion(J4_ZERO, J4_ENC_PER_DEG);
    J5Motor.configAngleConversion(J5_ZERO, J5_ENC_PER_DEG);
    J6Motor.configAngleConversion(0, J6_ENC_PER_DEG);

    // Set PID gains
    XMotor.setPID(0.02, 0, 0);
    J2Motor.setPID(0.55, 0, 0.001);
    J3Motor.setPID(0.25, 0, 0);
    J4Motor.setPID(0.03, 0, 0);
    J5Motor.setPID(0.08, 0, 0.001);
    J6Motor.setPID(0.2, 0, 0);
    GripperMotor.setPID(0.2, 0, 0);

    // Set soft limits
    XMotor.setSoftLimitPosition(X_REV_LIM, X_FWD_LIM);
    J2Motor.setSoftLimitPosition(J2_REV_LIM, J2_FWD_LIM);
    J3Motor.setSoftLimitPosition(J3_REV_LIM, J3_FWD_LIM);
    J4Motor.setSoftLimitPosition(J4_REV_LIM, J4_FWD_LIM);
    J5Motor.setSoftLimitPosition(J5_REV_LIM, J5_FWD_LIM);
    J6Motor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    GripperMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);

    // Set ramp rates
    XMotor.setRampRate(100.0);
    J2Motor.setRampRate(100.0);
    J3Motor.setRampRate(100.0);
    J4Motor.setRampRate(100.0);
    J5Motor.setRampRate(100.0);
    J6Motor.setRampRate(100.0);
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
        J5Motor.driveOpenLoop(0);
        J6Motor.driveOpenLoop(0);
        GripperMotor.driveOpenLoop(0);
    }
}

void telemetry() {
    // RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);

    if (!telemetryOverride) {
        JointPositions angles = {
            XMotor.getAngle(),
            J2Motor.getAngle(),
            J3Motor.getAngle(),
            J4Motor.getAngle(),
            J5Motor.getAngle(),
            J6Motor.getAngle(),
        };
        Vector gripperPos = IK::CalculateForwardTransform(angles) * Vector{0, 0, 0};
        float positions[9] = {
            XMotor.getAngle(),
            J2Motor.getAngle(),
            J3Motor.getAngle(),
            J4Motor.getAngle(),
            J5Motor.getAngle(),
            J6Motor.getAngle(),
            gripperPos.x,
            gripperPos.y,
            gripperPos.z,
        };
        RoveComm.write(RC_ARMBOARD_POSITION_DATA_ID, RC_ARMBOARD_POSITION_DATA_COUNT, positions);

        uint16_t limitsTriggered = bitmask(
            XMotor.getLimitSwitchA(), XMotor.getLimitSwitchB(), J2Motor.getLimitSwitchA(), J2Motor.getLimitSwitchB(),
            J3Motor.getLimitSwitchA(), J3Motor.getLimitSwitchB(), J4Motor.getLimitSwitchA(), J4Motor.getLimitSwitchB(),
            J5Motor.getLimitSwitchA(), J5Motor.getLimitSwitchB());
        RoveComm.write(RC_ARMBOARD_LIMITSWITCH_DATA_ID, RC_ARMBOARD_LIMITSWITCH_DATA_COUNT, &limitsTriggered);

        uint16_t softLimitsTriggered =
            bitmask(XMotor.getSoftLimitA(), XMotor.getSoftLimitB(), J2Motor.getSoftLimitA(), J2Motor.getSoftLimitB(),
                    J3Motor.getSoftLimitA(), J3Motor.getSoftLimitB(), J4Motor.getSoftLimitA(), J4Motor.getSoftLimitB(),
                    J5Motor.getSoftLimitA(), J5Motor.getSoftLimitB());
        RoveComm.write(RC_ARMBOARD_SOFTLIMIT_DATA_ID, RC_ARMBOARD_SOFTLIMIT_DATA_COUNT, &softLimitsTriggered);

        XMotor.ping();
        J2Motor.ping();
        J3Motor.ping();
        J4Motor.ping();
        J5Motor.ping();
        J6Motor.ping();
        GripperMotor.ping();

        uint16_t pingData[7] = {XMotor.getPingTime(),      J2Motor.getPingTime(),    J3Motor.getPingTime(),
                                J4Motor.getPingTime(),     J5Motor.getPingTime(), J6Motor.getPingTime(),
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
        currentMode = ControlMode::OPEN_LOOP;

        XMotor.driveOpenLoop(packet.i16data[0]);
        J2Motor.driveOpenLoop(packet.i16data[1]);
        J3Motor.driveOpenLoop(packet.i16data[2]);
        J4Motor.driveOpenLoop(packet.i16data[3]);
        J5Motor.driveOpenLoop(packet.i16data[4]);
        J6Motor.driveOpenLoop(packet.i16data[5]);

        targetAngles.X = XMotor.getAngle();
        targetAngles.J2 = J2Motor.getAngle();
        targetAngles.J3 = J3Motor.getAngle();
        targetAngles.J4 = J4Motor.getAngle();
        targetAngles.J5 = J5Motor.getAngle();
        targetAngles.J6 = J6Motor.getAngle();

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_TARGETANGLE_DATA_ID: {
        currentMode = ControlMode::CLOSED_LOOP;

        targetAngles.X = packet.fdata[0];
        targetAngles.J2 = packet.fdata[1];
        targetAngles.J3 = packet.fdata[2];
        targetAngles.J4 = packet.fdata[3];
        targetAngles.J5 = packet.fdata[4];
        targetAngles.J6 = packet.fdata[5];

        XMotor.driveTargetAngle(targetAngles.X, 0.05f);
        J2Motor.driveTargetAngle(targetAngles.J2, 0.05f);
        J3Motor.driveTargetAngle(targetAngles.J3, 0.05f);
        J4Motor.driveTargetAngle(targetAngles.J4, 0.05f);
        J5Motor.driveTargetAngle(targetAngles.J5, 0.05f);
        J6Motor.driveTargetAngle(targetAngles.J6, 0.05f);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_TARGETANGLEINCREMENT_DATA_ID: {
        if (currentMode != ControlMode::CLOSED_LOOP) {
            targetAngles.X = XMotor.getAngle();
            targetAngles.J2 = J2Motor.getAngle();
            targetAngles.J3 = J3Motor.getAngle();
            targetAngles.J4 = J4Motor.getAngle();
            targetAngles.J5 = J5Motor.getAngle();
            targetAngles.J6 = J6Motor.getAngle();
        }
        currentMode = ControlMode::CLOSED_LOOP;

        targetAngles.X += packet.fdata[0];
        targetAngles.J2 += packet.fdata[1];
        targetAngles.J3 += packet.fdata[2];
        targetAngles.J4 += packet.fdata[3];
        targetAngles.J5 += packet.fdata[4];
        targetAngles.J6 += packet.fdata[5];
        XMotor.driveTargetAngle( targetAngles.X, 0.05f);
        J2Motor.driveTargetAngle(targetAngles.J2, 0.05f);
        J3Motor.driveTargetAngle(targetAngles.J3, 0.05f);
        J4Motor.driveTargetAngle(targetAngles.J4, 0.05f);
        J5Motor.driveTargetAngle(targetAngles.J5, 0.05f);
        J6Motor.driveTargetAngle(targetAngles.J6, 0.05f);
        break;
    }
    // case RC_ARMBOARD_IKPOSITION_DATA_ID:
    case RC_ARMBOARD_IKPOSITIONINCREMENT_DATA_ID: {
                // SIM CODE:
        wristTarget.x += packet.fdata[0];
		wristTarget.y += packet.fdata[1];
		wristTarget.z += packet.fdata[2];
		TransfMatrix targetPose = 
		Translation(0, 0, wristTarget.z)
		* Translation(0, wristTarget.y, 0)
		* Translation(wristTarget.x, 0, 0);
		wristRotation = Rotation(0, M_PI_2, 0); // wrist facing forward
		targetPose = targetPose * wristRotation;
		
        targetAngles.X = XMotor.getAngle();
        targetAngles.J2 = J2Motor.getAngle();
        targetAngles.J3 = J3Motor.getAngle();
        targetAngles.J4 = J4Motor.getAngle();
        targetAngles.J5 = J5Motor.getAngle();
        targetAngles.J6 = J6Motor.getAngle();
        if (IK::CalculateInverseKinematics(targetPose, targetAngles)) {
            targetAngles.J4 = packet.fdata[4];
            targetAngles.J5 = packet.fdata[5];
            targetAngles.J6 = packet.fdata[6];
            if (
                XMotor.isAngleWithinLimits(targetAngles.X)
                && J2Motor.isAngleWithinLimits(targetAngles.J2)
                && J3Motor.isAngleWithinLimits(targetAngles.J3)
                && J4Motor.isAngleWithinLimits(targetAngles.J4)
                && J5Motor.isAngleWithinLimits(targetAngles.J5)
                && J6Motor.isAngleWithinLimits(targetAngles.J6)
            ) {
                XMotor.driveTargetAngle( targetAngles.X, 0.05f);
                J2Motor.driveTargetAngle(targetAngles.J2, 0.05f);
                J3Motor.driveTargetAngle(targetAngles.J3, 0.05f);
                J4Motor.driveTargetAngle(targetAngles.J4, 0.05f);
                J5Motor.driveTargetAngle(targetAngles.J5, 0.05f);
                J6Motor.driveTargetAngle(targetAngles.J6, 0.05f);
            } else {
                // std::cout << "IK outside limits" << std::endl;
            }
        } else {
            // std::cout << "IK failed" << std::endl;
        }
        break;
    }
    case RC_ARMBOARD_IKPOSEINCREMENT_DATA_ID: {
        // SIM CODE:
        wristTarget.x += packet.fdata[0];
		wristTarget.y += packet.fdata[1];
		wristTarget.z += packet.fdata[2];
		TransfMatrix targetPose = 
		Translation(0, 0, wristTarget.z)
		* Translation(0, wristTarget.y, 0)
		* Translation(wristTarget.x, 0, 0);
		wristRotation = Rotation(0, packet.fdata[3], 0) * wristRotation;
		wristRotation = Rotation(0, 0, packet.fdata[4]) * wristRotation;
		wristRotation = Rotation(packet.fdata[5], 0, 0) * wristRotation;
		targetPose = targetPose * wristRotation;
		
        targetAngles.X = XMotor.getAngle();
        targetAngles.J2 = J2Motor.getAngle();
        targetAngles.J3 = J3Motor.getAngle();
        targetAngles.J4 = J4Motor.getAngle();
        targetAngles.J5 = J5Motor.getAngle();
        targetAngles.J6 = J6Motor.getAngle();
        if (IK::CalculateInverseKinematics(targetPose, targetAngles)) {
            if (
                XMotor.isAngleWithinLimits(targetAngles.X)
                && J2Motor.isAngleWithinLimits(targetAngles.J2)
                && J3Motor.isAngleWithinLimits(targetAngles.J3)
                && J4Motor.isAngleWithinLimits(targetAngles.J4)
                && J5Motor.isAngleWithinLimits(targetAngles.J5)
                && J6Motor.isAngleWithinLimits(targetAngles.J6)
            ) {
                XMotor.driveTargetAngle( targetAngles.X, 0.05f);
                J2Motor.driveTargetAngle(targetAngles.J2, 0.05f);
                J3Motor.driveTargetAngle(targetAngles.J3, 0.05f);
                J4Motor.driveTargetAngle(targetAngles.J4, 0.05f);
                J5Motor.driveTargetAngle(targetAngles.J5, 0.05f);
                J6Motor.driveTargetAngle(targetAngles.J6, 0.05f);
            } else {
                // std::cout << "IK outside limits" << std::endl;
            }
        } else {
            // std::cout << "IK failed" << std::endl;
        }
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
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID: {
        watchdogOverride = packet.u8data[0];

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
        J5Motor.configIgnoreLimits(limits & (1 << 8), limits & (1 << 9));
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
        // j6 data & (1 << 1)
        if (packet.u8data[0] & (1 << 0)) {
            XMotor.calibratePosition(-INT16_MAX / 2, 0);
        } else if (packet.u8data[0] & (1 << 1)) {
            J6Zero = J6Motor.getPosition();
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
        J5Motor.setSoftLimitPosition(packetData & (1 << 8) ? INT32_MIN : J5_REV_LIM,
                                        packetData & (1 << 9) ? INT32_MAX : J5_FWD_LIM);

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
            // Serial.printf("ID %x CMD %x RTR %s LEN %d\n", receivedMessage.id >> 4, receivedMessage.id & 0xF,
            // receivedMessage.rtr ? "R" : "D", receivedMessage.len);
            XMotor.sync(receivedMessage);
            J2Motor.sync(receivedMessage);
            J3Motor.sync(receivedMessage);
            J4Motor.sync(receivedMessage);
            J5Motor.sync(receivedMessage);
            J6Motor.sync(receivedMessage);
            GripperMotor.sync(receivedMessage);
        }
    }
}
