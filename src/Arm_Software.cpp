#include "Arm_Software.h"
#include "InverseKinematics.h"

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

    LinearServo.attach(LINEAR_SERVO, 600, 2000);
    LinearServo.write(0);
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
    XMotor.setPID(0.005, 0, 0);
    J2Motor.setPID(0.007, 0, 0);
    J3Motor.setPID(0.005, 0, 0);
    J4Motor.setPID(0.003, 0, 0.15);
    J5Motor.setPID(0.004, 0, 0.1);
    J6Motor.setPID(0.005, 0, 0);
    GripperMotor.setPID(0.02, 0, 0);

    XMotor.setDutyCycleRange(0, INT16_MAX, 0, INT16_MIN);
    J2Motor.setDutyCycleRange(0, INT16_MAX, 0, INT16_MIN);
    J3Motor.setDutyCycleRange(0, INT16_MAX, 0, INT16_MIN);
    J4Motor.setDutyCycleRange(1000, INT16_MAX, -1000, INT16_MIN);
    J5Motor.setDutyCycleRange(2000, INT16_MAX, -2000, INT16_MIN);
    J6Motor.setDutyCycleRange(4200, INT16_MAX, -4200, INT16_MIN);
    GripperMotor.setDutyCycleRange(0, INT16_MAX, 0, INT16_MIN);

    // Set soft limits
    XMotor.setSoftLimitPosition(X_REV_LIM, X_FWD_LIM);
    J2Motor.setSoftLimitPosition(J2_REV_LIM, J2_FWD_LIM);
    J3Motor.setSoftLimitPosition(J3_REV_LIM, J3_FWD_LIM);
    J4Motor.setSoftLimitPosition(J4_REV_LIM, J4_FWD_LIM);
    J5Motor.setSoftLimitPosition(J5_REV_LIM, J5_FWD_LIM);
    J6Motor.setSoftLimitPosition(J6_REV_LIM, J6_FWD_LIM);
    GripperMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);

    // Set ramp rates
    XMotor.setRampRate(1.0);
    J2Motor.setRampRate(1.0);
    J3Motor.setRampRate(1.0);
    J4Motor.setRampRate(1.0);
    J5Motor.setRampRate(1.0);
    J6Motor.setRampRate(1.0);
    GripperMotor.setRampRate(1.0);

    // RoveComm
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_IPADDRESS);
    Serial.println("Complete");

    feedWatchdog();
    nextTelemetry = millis();
    nextPing = millis();
}

void loop() {
    // Serial.printf("%05d\t%05d\t%05d\t%05d\t%05d\t%05d\t\n", XMotor.getPosition(), J2Motor.getPosition(),
    //               J3Motor.getPosition(), J4Motor.getPosition(), J5Motor.getPosition(), J6Motor.getPosition());
    handleButtons();
    if (getButtonsPressed() == 0) {
        updateFromRoveComm();
    }
    receiveCANMessages();
    if (millis() >= nextPing) {
        pingJoints();
        nextPing += PING_PERIOD;
    }
    if (millis() >= nextTelemetry) {
        telemetry();
        nextTelemetry += TELEMETRY_PERIOD;
    }
}

void estop() {
    watchdogStatus = 1;
    GripperMotor.driveOpenLoop(0);

    switch (currentMode) {
    case ControlMode::OPEN_LOOP:
        driveOpenLoop(0, 0, 0, 0, 0, 0);
        break;
    case ControlMode::IK_WRIST:
    case ControlMode::IK_POSE:
    case ControlMode::CLOSED_LOOP:
        driveTargetAngles(XMotor.getAngle(), J2Motor.getAngle(), J3Motor.getAngle(), J4Motor.getAngle(),
                          J5Motor.getAngle(), J6Motor.getAngle());
        setControlMode(ControlMode::CLOSED_LOOP);
        break;
    }
}

void telemetry() {
    // RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);

    JointPositions angles = getJointPositions();
    Vector gripperPos = getGripperCoordinates();
    float positions[9] = {angles.X,  angles.J2,    angles.J3,    angles.J4,   angles.J5,
                          angles.J6, gripperPos.x, gripperPos.y, gripperPos.z};
    RoveComm.write(RC_ARMBOARD_POSITION_DATA_ID, RC_ARMBOARD_POSITION_DATA_COUNT, positions);

    float targets[9] = {XMotor.getTargetAngle(),  J2Motor.getTargetAngle(), J3Motor.getTargetAngle(),
                        J4Motor.getTargetAngle(), J5Motor.getTargetAngle(), J6Motor.getTargetAngle(),
                        gripperTarget.x,          gripperTarget.y,          gripperTarget.z};
    RoveComm.write(RC_ARMBOARD_TARGET_DATA_ID, RC_ARMBOARD_TARGET_DATA_COUNT, targets);

    uint16_t limitsTriggered =
        bitmask(XMotor.getLimitSwitchForward(), XMotor.getLimitSwitchReverse(), J2Motor.getLimitSwitchForward(),
                J2Motor.getLimitSwitchReverse(), J3Motor.getLimitSwitchForward(), J3Motor.getLimitSwitchReverse(),
                J4Motor.getLimitSwitchForward(), J4Motor.getLimitSwitchReverse(), J5Motor.getLimitSwitchForward(),
                J5Motor.getLimitSwitchReverse());
    RoveComm.write(RC_ARMBOARD_LIMITSWITCH_DATA_ID, limitsTriggered);

    uint16_t softLimitsTriggered =
        bitmask(XMotor.getSoftLimitForward(), XMotor.getSoftLimitReverse(), J2Motor.getSoftLimitForward(),
                J2Motor.getSoftLimitReverse(), J3Motor.getSoftLimitForward(), J3Motor.getSoftLimitReverse(),
                J4Motor.getSoftLimitForward(), J4Motor.getSoftLimitReverse(), J5Motor.getSoftLimitForward(),
                J5Motor.getSoftLimitReverse());
    RoveComm.write(RC_ARMBOARD_SOFTLIMIT_DATA_ID, softLimitsTriggered);

    uint16_t pingData[7] = {(uint16_t)XMotor.getPingTime(),      (uint16_t)J2Motor.getPingTime(),
                            (uint16_t)J3Motor.getPingTime(),     (uint16_t)J4Motor.getPingTime(),
                            (uint16_t)J5Motor.getPingTime(),     (uint16_t)J6Motor.getPingTime(),
                            (uint16_t)GripperMotor.getPingTime()};
    RoveComm.write(RC_ARMBOARD_SMOCOPING_DATA_ID, RC_ARMBOARD_SMOCOPING_DATA_COUNT, pingData);
}

void setLaser(bool on) { digitalWrite(LASER, on ? HIGH : LOW); }

void feedWatchdog() {
    watchdogStatus = 0;
    if (!watchdogOverride) {
        Watchdog.begin(estop, WATCHDOG_TIMEOUT);
    }
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
        TransfMatrix targetPose = Translation(packet.fdata[0], packet.fdata[1], packet.fdata[2]) *
                                  Rotation(packet.fdata[3], packet.fdata[4], packet.fdata[5]);
        driveInverseKinematics(targetPose);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_IKWORLDINCREMENT_DATA_ID: {
        incrementInverseKinematicsWorldPose(packet.fdata[0], packet.fdata[1], packet.fdata[2], packet.fdata[3],
                                            packet.fdata[4], packet.fdata[5]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_IKTOOLINCREMENT_DATA_ID: {
        incrementInverseKinematicsToolPose(packet.fdata[0], packet.fdata[1], packet.fdata[2], packet.fdata[3],
                                           packet.fdata[4], packet.fdata[5]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_IKWRISTINCREMENT_DATA_ID: {
        incrementInverseKinematicsWrist(packet.fdata[0], packet.fdata[1], packet.fdata[2], packet.fdata[3],
                                        packet.fdata[4], packet.fdata[5]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_GRIPPEROPENLOOP_DATA_ID: {
        GripperMotor.driveOpenLoop(packet.i16data[0]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_LINEARSERVO_DATA_ID: {
        LinearServo.write(packet.u8data[0]);
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
        if (watchdogOverride) {
            Watchdog.end();
        } else {
            Watchdog.begin(estop, WATCHDOG_TIMEOUT);
        }

        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID: {

        // not effective until next drive command!
        int16_t limits = packet.i16data[0];
        XMotor.setIgnoreLimit(limits & (1 << 0), limits & (1 << 1));
        J2Motor.setIgnoreLimit(limits & (1 << 2), limits & (1 << 3));
        J3Motor.setIgnoreLimit(limits & (1 << 4), limits & (1 << 5));
        J4Motor.setIgnoreLimit(limits & (1 << 6), limits & (1 << 7));
        J5Motor.setIgnoreLimit(limits & (1 << 8), limits & (1 << 9));
        break;
    }
    case RC_ARMBOARD_CLOSEDLOOPOVERRIDE_DATA_ID: {

        // TODO: State dependent (applies to IK mode, makes certain axes run in openloop with speed 0)
        
        setOpenLoopOverride(packet.u16data[0]);
        feedWatchdog();
        break;
    }
    case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID: {
        estop();

        // x data & (1 << 0)
        // j6 data & (1 << 1)
        if (packet.u8data[0] & (1 << 0)) {
            XMotor.calibrateAngle(INT16_MIN / 2, -7.06);
            uint32_t timeout = millis() + 10000;
            while (!XMotor.getCalibrated() && millis() < timeout) {
                feedWatchdog();
                receiveCANMessages();
                delay(100);
            }
            estop(); // this will reset the target angles if in closed loop
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

void pingJoints() {
    XMotor.ping();
    J2Motor.ping();
    J3Motor.ping();
    J4Motor.ping();
    J5Motor.ping();
    J6Motor.ping();
    GripperMotor.ping();
}

void setOpenLoopOverride(int16_t bitmask){
    closedLoopOverride = bitmask;
}

// Drive joints with given powers
void driveOpenLoop(int16_t XDuty, int16_t J2Duty, int16_t J3Duty, int16_t J4Duty, int16_t J5Duty, int16_t J6Duty) {
    setControlMode(ControlMode::OPEN_LOOP);
    XMotor.driveOpenLoop(XDuty);
    J2Motor.driveOpenLoop(J2Duty);
    J3Motor.driveOpenLoop(J3Duty);
    J4Motor.driveOpenLoop(J4Duty);
    J5Motor.driveOpenLoop(J5Duty);
    J6Motor.driveOpenLoop(J6Duty);
}

// Drive joints to target angles
void driveTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle) {
    setControlMode(ControlMode::CLOSED_LOOP);
    XMotor.driveTargetAngle(XAngle, 0);
    J2Motor.driveTargetAngle(J2Angle, 0);
    J3Motor.driveTargetAngle(J3Angle, 0);
    J4Motor.driveTargetAngle(J4Angle, 0);
    J5Motor.driveTargetAngle(J5Angle, 0);
    J6Motor.driveTargetAngle(J6Angle, 0);
}

// Increment joint angles
void incrementTargetAngles(float XAngle, float J2Angle, float J3Angle, float J4Angle, float J5Angle, float J6Angle) {
    setControlMode(ControlMode::CLOSED_LOOP);
    XMotor.driveTargetAngle(XMotor.getTargetAngle() + XAngle, 0);
    J2Motor.driveTargetAngle(J2Motor.getTargetAngle() + J2Angle, 0);
    J3Motor.driveTargetAngle(J3Motor.getTargetAngle() + J3Angle, 0);
    J4Motor.driveTargetAngle(J4Motor.getTargetAngle() + J4Angle, 0);
    J5Motor.driveTargetAngle(J5Motor.getTargetAngle() + J5Angle, 0);
    J6Motor.driveTargetAngle(J6Motor.getTargetAngle() + J6Angle, 0);
}

void incrementInverseKinematicsWrist(float x, float y, float z, float j4, float j5, float j6) {
    setControlMode(ControlMode::IK_WRIST);

    TransfMatrix targetPose =
        Translation(gripperTarget.x + x, gripperTarget.y + y, gripperTarget.z + z) // gripper coords
        * Rotation(0, M_PI, 0);                                                    // wrist facing forward
    // calculate IK up to wrist
    JointPositions levelAngles = getJointPositions();
    // pretend the last 3 angles are always zero so that only one solution is chosen
    levelAngles.J4 = 0;
    levelAngles.J5 = 0;
    levelAngles.J6 = 0;
    if (!IK::CalculateInverseKinematics(targetPose, levelAngles)) return;

    JointPositions newAngles = {levelAngles.X, levelAngles.J2, levelAngles.J3, levelAngles.J4 + j4j5j6Target.x + j4,
                                (levelAngles.J5 * cosf((levelAngles.J4 + j4j5j6Target.x + j4) * M_PI / 180)) + j4j5j6Target.y + j5,
                                // levelAngles.J5 + j4j5j6Target.y + j5,
                                levelAngles.J6 + j4j5j6Target.z + j6};

    if (!isPositionWithinLimits(newAngles)) return;

    gripperTarget = targetPose.getTranslation();
    wristRotation = targetPose.getRotation();
    j4j5j6Target.x += j4;
    j4j5j6Target.y += j5;
    j4j5j6Target.z += j6;

    XMotor.driveTargetAngle(newAngles.X, 0);
    J2Motor.driveTargetAngle(newAngles.J2, 0);
    J3Motor.driveTargetAngle(newAngles.J3, 0);
    J4Motor.driveTargetAngle(newAngles.J4, 0);
    J5Motor.driveTargetAngle(newAngles.J5, 0);
    J6Motor.driveTargetAngle(newAngles.J6, 0);
}

void incrementInverseKinematicsWorldPose(float tx, float ty, float tz, float rx, float ry, float rz) {
    setControlMode(ControlMode::IK_POSE);

    TransfMatrix targetPose = Translation(gripperTarget.x + tx, gripperTarget.y + ty, gripperTarget.z + tz)
                              // incrementally rotate wrist in world space
                              * Rotation(0, ry * M_PI / 180, 0) // rotate about Y (yaw)
                              * Rotation(rx * M_PI / 180, 0, 0) // rotate about X (pitch)
                              * Rotation(0, 0, rz * M_PI / 180) // rotate about Z (roll)
                              * wristRotation;

    driveInverseKinematics(targetPose);
}

void incrementInverseKinematicsToolPose(float tx, float ty, float tz, float rx, float ry, float rz) {
    setControlMode(ControlMode::IK_POSE);

    TransfMatrix newWristRotation = wristRotation
                                    * Rotation(0, ry * M_PI / 180, 0)  // rotate about Y (yaw)
                                    * Rotation(rx * M_PI / 180, 0, 0)  // rotate about X (pitch)
                                    * Rotation(0, 0, rz * M_PI / 180); // rotate about Z (roll)
    TransfMatrix targetPose = Translation(gripperTarget.x, gripperTarget.y, gripperTarget.z) * newWristRotation;
    Vector newGripperTarget = targetPose * Vector{tx, ty, tz};

    targetPose = Translation(newGripperTarget.x, newGripperTarget.y, newGripperTarget.z) * newWristRotation;

    driveInverseKinematics(targetPose);
}

void driveInverseKinematics(const TransfMatrix& targetPose) {
    setControlMode(ControlMode::IK_POSE);
    JointPositions angles = getJointPositions();
    if (!IK::CalculateInverseKinematics(targetPose, angles)) return;
    if (!isPositionWithinLimits(angles)) return;
    gripperTarget = targetPose.getTranslation();
    wristRotation = targetPose.getRotation();
    XMotor.driveTargetAngle(angles.X, 0);
    J2Motor.driveTargetAngle(angles.J2, 0);
    J3Motor.driveTargetAngle(angles.J3, 0);
    J4Motor.driveTargetAngle(angles.J4, 0);
    J5Motor.driveTargetAngle(angles.J5, 0);
    J6Motor.driveTargetAngle(angles.J6, 0);
}

void limitSwitchOverride(uint16_t bitmask) {
    XMotor.setIgnoreLimit(bitmask & (1 << 0), bitmask & (1 << 1));
    J2Motor.setIgnoreLimit(bitmask & (1 << 2), bitmask & (1 << 3));
    J3Motor.setIgnoreLimit(bitmask & (1 << 4), bitmask & (1 << 5));
    J4Motor.setIgnoreLimit(bitmask & (1 << 6), bitmask & (1 << 7));
    J5Motor.setIgnoreLimit(bitmask & (1 << 8), bitmask & (1 << 9));
}

// Configure soft limits
void softLimitOverride(uint16_t bitmask) {
    XMotor.setSoftLimitPosition(bitmask & (1 << 0) ? INT32_MIN : X_FWD_LIM, //
                                bitmask & (1 << 1) ? INT32_MAX : X_REV_LIM);
    J2Motor.setSoftLimitPosition(bitmask & (1 << 2) ? INT32_MIN : J2_FWD_LIM,
                                 bitmask & (1 << 3) ? INT32_MAX : J2_REV_LIM);
    J3Motor.setSoftLimitPosition(bitmask & (1 << 4) ? INT32_MIN : J3_FWD_LIM,
                                 bitmask & (1 << 5) ? INT32_MAX : J3_REV_LIM);
    J4Motor.setSoftLimitPosition(bitmask & (1 << 6) ? INT32_MIN : J4_FWD_LIM,
                                 bitmask & (1 << 7) ? INT32_MAX : J4_REV_LIM);
    J5Motor.setSoftLimitPosition(bitmask & (1 << 8) ? INT32_MIN : J5_FWD_LIM,
                                 bitmask & (1 << 9) ? INT32_MAX : J5_REV_LIM);
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

uint64_t getButtonsPressed() {

    uint64_t ret = (!digitalRead(BTN_1) << BTN_1) | (!digitalRead(BTN_2) << BTN_2) | (!digitalRead(BTN_3) << BTN_3) |
                   (!digitalRead(BTN_4) << BTN_4) | (!digitalRead(BTN_5) << BTN_5) | (!digitalRead(BTN_6) << BTN_6) |
                   (!digitalRead(BTN_7) << BTN_7) | (!digitalRead(BTN_8) << BTN_8);
    return ret;
}

void handleButtons() {
    XButton.update();
    J2Button.update();
    J3Button.update();
    J4Button.update();
    J5Button.update();
    J6Button.update();
    GripperButton.update();

    // if somoeone codes this function in a more condensed way show me how so I can learn 'Malakhi Rivera & Drew
    // Fundaburg UwU'

    if (XButton.fallingEdge()) {
        estop();
        if (digitalRead(DIR_SW) == 0) {
            XMotor.driveOpenLoop(INT16_MAX * 0.5);
        }
        if (digitalRead(DIR_SW) == 1) {
            XMotor.driveOpenLoop(INT16_MIN * 0.5);
        }
    }

    if (XButton.risingEdge()) {
        estop();
    }

    if (J2Button.fallingEdge()) {
        estop();
        if (digitalRead(DIR_SW) == 0) {
            J2Motor.driveOpenLoop(INT16_MAX * 0.3);
        }
        if (digitalRead(DIR_SW) == 1) {
            J2Motor.driveOpenLoop(INT16_MIN * 0.3);
        }
    }

    if (J2Button.risingEdge()) {
        estop();
    }

    if (J3Button.fallingEdge()) {
        estop();
        if (digitalRead(DIR_SW) == 0) {
            J3Motor.driveOpenLoop(INT16_MAX * 0.5);
        }
        if (digitalRead(DIR_SW) == 1) {
            J3Motor.driveOpenLoop(INT16_MIN * 0.5);
        }
    }

    if (J3Button.risingEdge()) {
        estop();
    }

    if (J4Button.fallingEdge()) {
        estop();
        if (digitalRead(DIR_SW) == 0) {
            J4Motor.driveOpenLoop(INT16_MAX * 0.5);
        }
        if (digitalRead(DIR_SW) == 1) {
            J4Motor.driveOpenLoop(INT16_MIN * 0.5);
        }
    }

    if (J4Button.risingEdge()) {
        estop();
    }

    if (J5Button.fallingEdge()) {
        estop();
        if (digitalRead(DIR_SW) == 0) {
            J5Motor.driveOpenLoop(INT16_MAX * 0.5);
        }
        if (digitalRead(DIR_SW) == 1) {
            J5Motor.driveOpenLoop(INT16_MIN * 0.5);
        }
    }

    if (J5Button.risingEdge()) {
        estop();
    }

    if (J6Button.fallingEdge()) {
        estop();
        if (digitalRead(DIR_SW) == 0) {
            J6Motor.driveOpenLoop(INT16_MAX * 0.5);
        }
        if (digitalRead(DIR_SW) == 1) {
            J6Motor.driveOpenLoop(INT16_MIN * 0.5);
        }
    }

    if (J6Button.risingEdge()) {
        estop();
    }

    if (GripperButton.fallingEdge()) {
        estop();
        if (digitalRead(DIR_SW) == 0) {
            GripperMotor.driveOpenLoop(INT16_MAX * 0.2);
        }
        if (digitalRead(DIR_SW) == 1) {
            GripperMotor.driveOpenLoop(INT16_MIN * 0.2);
        }
    }

    if (GripperButton.risingEdge()) {
        estop();
    }
}

void setControlMode(ControlMode newMode) {
    if (currentMode == newMode) {
        return;
    }
    currentMode = newMode;
    switch (newMode) {
    case ControlMode::OPEN_LOOP:
        Serial.println("SETTING TO OPEN LOOP");
        break;
    case ControlMode::CLOSED_LOOP:
        Serial.println("SETTING TO CLOSED LOOP");
        driveTargetAngles(XMotor.getAngle(), J2Motor.getAngle(), J3Motor.getAngle(), J4Motor.getAngle(),
                          J5Motor.getAngle(), J6Motor.getAngle());
        break;
    case ControlMode::IK_WRIST: {
        JointPositions levelAngles = getJointPositions();
        // wrist center
        TransfMatrix currentPose = IK::CalculateForwardTransform(levelAngles);
        Vector wristCoords = currentPose.getTranslation() - currentPose.getRotation() * (DH_6.d*BASIS_Z);
        // gripper center
        gripperTarget = wristCoords + DH_6.d*-BASIS_Z;
        // compute what the angles would be
        levelAngles.J4 = 0;
        levelAngles.J5 = 0;
        levelAngles.J6 = 0;
        IK::CalculateInverseKinematics(Translation(gripperTarget.x, gripperTarget.y, gripperTarget.z) * Rotation(0, M_PI, 0), levelAngles);
        levelAngles.J5 *= cosf((levelAngles.J4 + J4Motor.getAngle()) * M_PI / 180);
        // compute what the angles should be
        j4j5j6Target = {J4Motor.getAngle() - levelAngles.J4, J5Motor.getAngle() - levelAngles.J5, J6Motor.getAngle() - levelAngles.J6};
        Serial.println("SETTING TO WRIST CONTROL");
        break;
    }
    case ControlMode::IK_POSE: {
        TransfMatrix currentPose = IK::CalculateForwardTransform(getJointPositions());
        gripperTarget = currentPose.getTranslation();
        wristRotation = currentPose.getRotation();
        Serial.println("SETTING TO POSE CONTROL");
        break;
    }
    }
}
