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
    J6Motor.configAngleConversion(J6Zero, J6_ENC_PER_DEG);

    // Set PID gains
    XMotor.setPID(0.05, 0, 0);
    J2Motor.setPID(0.55, 0, 0.001);
    J3Motor.setPID(0.25, 0, 0);
    J4Motor.setPID(0.03, 0, 0);
    J5Motor.setPID(0.08, 0, 0.001);
    J6Motor.setPID(0.2, 0, 0);
    GripperMotor.setPID(0.2, 0, 0);

    // Set soft limits
    XMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
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
}

void estop() {
    if (!watchdogOverride) {
        watchdogStatus = 1;
        driveOpenLoop(0, 0, 0, 0, 0, 0);
    }
}

void telemetry() {
    // RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);
    Serial.printf("X: %d\n", XMotor.getPosition());

    if (!telemetryOverride) {
        JointPositions angles = getJointPositions();
        Vector gripperPos = getGripperCoordinates();
        // Convert Dr Gant space to BaseStation space
        // Z -> X
        // Y -> Y
        // X -> Z
        float positions[9] = {angles.X,  angles.J2,    angles.J3,    angles.J4,   angles.J5,
                              angles.J6, gripperPos.z, gripperPos.y, gripperPos.x};
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

        uint16_t pingData[7] = {(uint16_t)XMotor.getPingTime(),      (uint16_t)J2Motor.getPingTime(),
                                (uint16_t)J3Motor.getPingTime(),     (uint16_t)J4Motor.getPingTime(),
                                (uint16_t)J5Motor.getPingTime(),     (uint16_t)J6Motor.getPingTime(),
                                (uint16_t)GripperMotor.getPingTime()};
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
        driveOpenLoop(packet.i16data[0], packet.i16data[1], packet.i16data[2], packet.i16data[3], packet.i16data[4],
                      packet.i16data[5]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_TARGETANGLE_DATA_ID: {
        driveTargetAngles(packet.fdata[0], packet.fdata[1], packet.fdata[2], packet.fdata[3], packet.fdata[4],
                          packet.fdata[5]);

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_TARGETANGLEINCREMENT_DATA_ID: {
        incrementTargetAngles(packet.fdata[0], packet.fdata[1], packet.fdata[2], packet.fdata[3], packet.fdata[4],
                              packet.fdata[5]);
        break;
    }
    case RC_ARMBOARD_IKPOSITION_DATA_ID: {
        // X -> Z
        // Y -> Y
        // Z -> X
        TransfMatrix targetPose = Translation(packet.fdata[2], packet.fdata[1], packet.fdata[1]) *
                                  Rotation(packet.fdata[5], packet.fdata[4], packet.fdata[3]);
        driveInverseKinematics(targetPose);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_IKPOSEINCREMENT_DATA_ID: {
        // X -> Z
        // Y -> Y
        // Z -> X
        incrementInverseKinematicsPose(packet.fdata[2], packet.fdata[1], packet.fdata[0], packet.fdata[5],
                                       packet.fdata[4], packet.fdata[3]);
        break;
    }
    case RC_ARMBOARD_IKPOSITIONINCREMENT_DATA_ID: {
        // X -> Z
        // Y -> Y
        // Z -> X
        incrementInverseKinematicsPosition(packet.fdata[2], packet.fdata[1], packet.fdata[0], packet.fdata[5],
                                           packet.fdata[4], packet.fdata[3]);
        break;
    }
    case RC_ARMBOARD_GRIPPEROPENLOOP_DATA_ID: {
        GripperMotor.driveOpenLoop(packet.i16data[0]);
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
        }
        if (packet.u8data[0] & (1 << 1)) {
            J6Zero = J6Motor.getPosition();
            J6Motor.configAngleConversion(J6Zero, J6_ENC_PER_DEG);
        }

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_SOFTLIMITOVERRIDE_DATA_ID: {
        softLimitOverride(packet.u16data[0]);
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

// Drive joints with given powers
void driveOpenLoop(int16_t XDuty, int16_t J2Duty, int16_t J3Duty, int16_t J4Duty, int16_t J5Duty, int16_t J6Duty) {
    if (currentMode != ControlMode::OPEN_LOOP) Serial.println("SETTING TO OPEN LOOP");
    currentMode = ControlMode::OPEN_LOOP;
    XMotor.driveOpenLoop(XDuty);
    J2Motor.driveOpenLoop(J2Duty);
    J3Motor.driveOpenLoop(J3Duty);
    J4Motor.driveOpenLoop(J4Duty);
    J5Motor.driveOpenLoop(J5Duty);
    J6Motor.driveOpenLoop(J6Duty);
}

// Drive joints to target angles
void driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle) {
    if (currentMode != ControlMode::CLOSED_LOOP) Serial.println("SETTING TO CLOSED LOOP");
    currentMode = ControlMode::CLOSED_LOOP;
    XMotor.driveTargetAngle(XAngle, 0.05);
    J2Motor.driveTargetAngle(J2Angle, 0.05);
    J3Motor.driveTargetAngle(J3Angle, 0.05);
    J4Motor.driveTargetAngle(J4Angle, 0.05);
    J5Motor.driveTargetAngle(J5Angle, 0.05);
    J6Motor.driveTargetAngle(J6Angle, 0.05);
}

// Increment joint angles
void incrementTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle) {
    if (currentMode != ControlMode::CLOSED_LOOP) {
        driveTargetAngles(XMotor.getAngle(), J2Motor.getAngle(), J3Motor.getAngle(), J4Motor.getAngle(),
                          J5Motor.getAngle(), J6Motor.getAngle());
    }
    currentMode = ControlMode::CLOSED_LOOP;
    XMotor.driveTargetAngle(XMotor.getTargetAngle() + XAngle, 0.05);
    J2Motor.driveTargetAngle(J2Motor.getTargetAngle() + J2Angle, 0.05);
    J3Motor.driveTargetAngle(J3Motor.getTargetAngle() + J3Angle, 0.05);
    J4Motor.driveTargetAngle(J4Motor.getTargetAngle() + J4Angle, 0.05);
    J5Motor.driveTargetAngle(J5Motor.getTargetAngle() + J5Angle, 0.05);
    J6Motor.driveTargetAngle(J6Motor.getTargetAngle() + J6Angle, 0.05);
}


void incrementInverseKinematicsPosition(float x, float y, float z, float j4, float j5, float j6) {
    if (currentMode != ControlMode::IK_WRIST) {
        JointPositions levelAngles = getJointPositions();
        // wrist center
        TransfMatrix currentPose = IK::CalculateForwardTransform(levelAngles);
        Vector wristCoords = currentPose.getTranslation() - currentPose.getRotation() * ((WRIST_LENGTH+GRIPPER_LENGTH)*BASIS_Z);
        // gripper center
        gripperTarget = wristCoords + (WRIST_LENGTH+GRIPPER_LENGTH)*BASIS_X;
        // compute what the angles would be
        IK::CalculateInverseKinematics(Translation(gripperTarget.x, gripperTarget.y, gripperTarget.z)*Rotation(0, M_PI_2, 0), levelAngles);
        // compute what the angles should be
        j4j5j6Target = {
            J4Motor.getAngle() - levelAngles.J4,
            J5Motor.getAngle() - levelAngles.J5,
            J6Motor.getAngle() - levelAngles.J6
        };
        Serial.println("SETTING TO WRIST CONTROL");
    }
    currentMode = ControlMode::IK_WRIST;

    TransfMatrix targetPose = Translation(gripperTarget.x + x, gripperTarget.y + y, gripperTarget.z + z) // gripper coords
                                * Rotation(0, M_PI_2, 0); // wrist facing forward
    // calculate IK up to wrist
    JointPositions angles = getJointPositions();
    if (!IK::CalculateInverseKinematics(targetPose, angles)) return;

    // set back to current target angles
    JointPositions currentAngles = getJointPositions();
    
    angles.J4 += j4j5j6Target.x + j4;
    angles.J5 += j4j5j6Target.y + j5;
    angles.J6 += j4j5j6Target.z + j6;
    
    if (!isPositionWithinLimits(angles)) return;

    gripperTarget.x += x;
    gripperTarget.y += y;
    gripperTarget.z += z;
    j4j5j6Target.x += j4;
    j4j5j6Target.y += j5;
    j4j5j6Target.z += j6;

    XMotor.driveTargetAngle(angles.X, 0.05);
    J2Motor.driveTargetAngle(angles.J2, 0.05);
    J3Motor.driveTargetAngle(angles.J3, 0.05);
    J4Motor.driveTargetAngle(angles.J4, 0.05);
    J5Motor.driveTargetAngle(angles.J5, 0.05);
    J6Motor.driveTargetAngle(angles.J6, 0.05);

}

void incrementInverseKinematicsPose(float tx, float ty, float tz, float rx, float ry, float rz) {
    if (currentMode != ControlMode::IK_POSE) {
        TransfMatrix currentPose = IK::CalculateForwardTransform(getJointPositions());
        gripperTarget = currentPose.getTranslation();
        wristRotation = currentPose.getRotation();
        Serial.println("SETTING TO POSE CONTROL");
    }
    currentMode = ControlMode::IK_POSE;

    TransfMatrix targetPose = Translation(gripperTarget.x + tx, gripperTarget.y + ty, gripperTarget.z + tz)
    // incrementally rotate wrist in world space
    * Rotation(0, ry * M_PI / 180, 0) // rotate about Y
    * Rotation(rx * M_PI / 180, 0, 0) // rotate about X
    * Rotation(0, 0, rz * M_PI / 180) // rotate about Z
    * wristRotation;

    driveInverseKinematics(targetPose);
}

void driveInverseKinematics(const TransfMatrix& targetPose) {
    currentMode = ControlMode::IK_POSE;
    JointPositions angles = getJointPositions();
    if (!IK::CalculateInverseKinematics(targetPose, angles)) return;
    if (!isPositionWithinLimits(angles)) return;
    gripperTarget = targetPose.getTranslation();
    wristRotation = targetPose.getRotation();
    XMotor.driveTargetAngle(angles.X, 0.05);
    J2Motor.driveTargetAngle(angles.J2, 0.05);
    J3Motor.driveTargetAngle(angles.J3, 0.05);
    J4Motor.driveTargetAngle(angles.J4, 0.05);
    J5Motor.driveTargetAngle(angles.J5, 0.05);
    J6Motor.driveTargetAngle(angles.J6, 0.05);
}

void limitSwitchOverride(uint16_t bitmask) {
    XMotor.configIgnoreLimits(bitmask & (1 << 0), bitmask & (1 << 1));
    J2Motor.configIgnoreLimits(bitmask & (1 << 2), bitmask & (1 << 3));
    J3Motor.configIgnoreLimits(bitmask & (1 << 4), bitmask & (1 << 5));
    J4Motor.configIgnoreLimits(bitmask & (1 << 6), bitmask & (1 << 7));
    J5Motor.configIgnoreLimits(bitmask & (1 << 8), bitmask & (1 << 9));
}

// Configure soft limits
void softLimitOverride(uint16_t bitmask) {
    XMotor.setSoftLimitPosition(bitmask & (1 << 0) ? INT32_MIN : X_REV_LIM, //
                                bitmask & (1 << 1) ? INT32_MAX : X_FWD_LIM);
    J2Motor.setSoftLimitPosition(bitmask & (1 << 2) ? INT32_MIN : X_REV_LIM,
                                 bitmask & (1 << 3) ? INT32_MAX : J2_FWD_LIM);
    J3Motor.setSoftLimitPosition(bitmask & (1 << 4) ? INT32_MIN : X_REV_LIM,
                                 bitmask & (1 << 5) ? INT32_MAX : J3_FWD_LIM);
    J4Motor.setSoftLimitPosition(bitmask & (1 << 6) ? INT32_MIN : J4_REV_LIM,
                                 bitmask & (1 << 7) ? INT32_MAX : J4_FWD_LIM);
    J5Motor.setSoftLimitPosition(bitmask & (1 << 8) ? INT32_MIN : J5_REV_LIM,
                                 bitmask & (1 << 9) ? INT32_MAX : J5_FWD_LIM);
}

bool isPositionWithinLimits(const JointPositions& angles) {
    return XMotor.isAngleWithinLimits(angles.X) && J2Motor.isAngleWithinLimits(angles.J2) &&
           J3Motor.isAngleWithinLimits(angles.J3) && J4Motor.isAngleWithinLimits(angles.J4) &&
           J5Motor.isAngleWithinLimits(angles.J5) && J6Motor.isAngleWithinLimits(angles.J6);
}

JointPositions getJointPositions() {
    return {
        XMotor.getAngle(),  J2Motor.getAngle(), J3Motor.getAngle(),
        J4Motor.getAngle(), J5Motor.getAngle(), J6Motor.getAngle(),
    };
}

Vector getGripperCoordinates() {
    JointPositions angles = getJointPositions();
    return IK::CalculateForwardTransform(angles) * ORIGIN;
}
