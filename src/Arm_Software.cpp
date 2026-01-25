#include "Arm_Software.h"

// 2026 DEV

void setup() 
{

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

    ACAN_T4_Settings settings(125000); //I think i'm missing things

    //Set Low pass smoothing factor
    xMotor.setLowPassSmoothingFactor(INT16_MAX);
    J2Motor.setLowPassSmoothingFactor(INT16_MAX);
    J3Motor.setLowPassSmoothingFactor(INT16_MAX);
    J4Motor.setLowPassSmoothingFactor(INT16_MAX);
    PitchMotor.setLowPassSmoothingFactor(INT16_MAX);
    RollMotor.setLowPassSmoothingFactor(INT16_MAX);
    GripperMotor.setLowPassSmoothingFactor(INT16_MAX);
    SpareMotor.setLowPassSmoothingFactor(INT16_MAX);

    //Set PID gains, Need to make gains vars or store somewhere
    xMotor.setPID(1, 0, 0);
    J2Motor.setPID(1, 0, 0);
    J3Motor.setPID(1, 0, 0);
    J4Motor.setPID(1, 0, 0);
    PitchMotor.setPID(1, 0, 0);
    RollMotor.setPID(1, 0, 0);
    GripperMotor.setPID(1, 0, 0);
    SpareMotor.setPID(1, 0, 0);

    //Set soft limits
    xMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    J2Motor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    J3Motor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    J4Motor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    PitchMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    RollMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    GripperMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    SpareMotor.setSoftLimitPosition(INT32_MIN, INT32_MAX); 

    // RoveComm
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_IPADDRESS);
    Serial.println("Complete");

    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}

void loop() 
{   

    feedWatchdog(); //REMOVE
    InitiallySyncTargets();
    UpdateLimits();
    UpdateFromRoveComm();
    UpdateArm();
    
}

void estop() 
{
    if (!watchdogOverride)
    {
        watchdogStatus = 1;

        xMotor.openLoopDrive(0, false);    //Stop and reset?
        J2Motor.openLoopDrive(0, false);
        J3Motor.openLoopDrive(0, false);
        J4Motor.openLoopDrive(0, false);
        PitchMotor.openLoopDrive(0, false);
        RollMotor.openLoopDrive(0, false);
        GripperMotor.openLoopDrive(0, false);

        IKMode = false;

    }
}

void telemetry() 
{
    // :(
    // RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);
    
    if(!telemetryOverride) {

        float positions[7] = {
            XJoint.Encoder()->readDegrees(),
            J2Joint.Encoder()->readDegrees(),
            J3Joint.Encoder()->readDegrees(),
            J4Joint.Encoder()->readDegrees(),
            PitchJoint.Encoder()->readDegrees(),
            RollJoint.Encoder()->readDegrees(),
            (PitchJoint.Encoder()->readDegrees() * cosf(J4Joint.Encoder()->readDegrees()*DEG2RAD)) + (J2Joint.Encoder()->readDegrees() + J3Joint.Encoder()->readDegrees())
        };
        // Serial.println();
        // Serial.print(positions[6]);
        RoveComm.write(RC_ARMBOARD_POSITIONS_DATA_ID, RC_ARMBOARD_POSITIONS_DATA_COUNT, positions);

        float coords[6] = {
            CartesianCoords.x,
            CartesianCoords.y,
            CartesianCoords.z,
            J4Joint.Encoder()->readDegrees(),
            PitchJoint.Encoder()->readDegrees()
        };
        RoveComm.write(RC_ARMBOARD_COORDINATES_DATA_ID, RC_ARMBOARD_COORDINATES_DATA_COUNT, coords);
    
        uint16_t limitsTriggered = 0;
        if (XJoint.atForwardHardLimit()) limitsTriggered |= (1 << 0);
        if (XJoint.atReverseHardLimit()) limitsTriggered |= (1 << 1);
        if (J2Joint.atForwardHardLimit()) limitsTriggered |= (1 << 2);
        if (J2Joint.atReverseHardLimit()) limitsTriggered |= (1 << 3);
        if (J3Joint.atForwardHardLimit()) limitsTriggered |= (1 << 4);
        if (J3Joint.atReverseHardLimit()) limitsTriggered |= (1 << 5);
        if (J4Joint.atForwardHardLimit()) limitsTriggered |= (1 << 6);
        if (J4Joint.atReverseHardLimit()) limitsTriggered |= (1 << 8);
        if (PitchJoint.atForwardHardLimit()) limitsTriggered |= (1 << 9);
        RoveComm.write(RC_ARMBOARD_LIMITSWITCHTRIGGERED_DATA_ID, limitsTriggered);
    }

    //Add telemetry data as needed
}

void setSolenoid(bool extend) { digitalWrite(Solenoid, extend? HIGH:LOW); }

void setLaser(bool on) { digitalWrite(LASER,on? HIGH:LOW); }

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

void CalibrateX()
{

    if(XJoint.atReverseHardLimit()) {
        XState.overrideReverseSoftLimit(false);
        XState.overrideForwardSoftLimit(false);
        XJoint.drive(0);
        XJoint.Encoder()->setDegrees(0.01);
        Xcalibrating = false;
        Xcalibrated = true;
        XState.setTarget(0);
        Serial.printf("X Calibrated!");
        XState.overrideClosedLoop(false);
    } else {
        XState.overrideReverseSoftLimit(true);
        XState.overrideForwardSoftLimit(true);
        XJoint.drive(-900);
        Serial.printf("X Calibrating...");
    }

}

void InitiallySyncTargets() //Also calc xyz
{

    if (firstLoop)
    {

        XState.setMotor();
        J2State.setMotor();
        J3State.setMotor();
        J4State.setMotor();
        PitchState.setMotor();
        RollState.setMotor();

        XState.setTarget(XState.getMotorAngle());
        J2State.setTarget(J2State.getMotorAngle());
        J3State.setTarget(J3State.getMotorAngle());
        J4State.setTarget(J4State.getMotorAngle());
        PitchState.setTarget(PitchState.getMotorAngle());
        RollState.setTarget(RollState.getMotorAngle());

        CalculateForwardKinematics();
        
    }
    firstLoop = false;

}

void UpdateFromRoveComm()
{
    static RoveCommPacket packet;
    RoveComm.read(packet);

    switch (packet.dataId) {
        case RC_ARMBOARD_OPENLOOP_DATA_ID:
        {
            int16_t *data = (int16_t*) packet.data;
            xMotor.openLoopDrive(data[0], false); //limit switch comes from basestation
            J2Motor.openLoopDrive(data[1], false);
            J3Motor.openLoopDrive(data[2], false);
            J4Motor.openLoopDrive(data[3], false);
            PitchMotor.openLoopDrive(data[4], false);
            RollMotor.openLoopDrive(data[5], false);

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_TARGETANGLE_DATA_ID:
        {
            float *data = (float*) packet.data; //Float not i32?
            xMotor.setJointAngle(data[0], 1, false); //limit switch comes from basestation
            J2Motor.setJointAngle(data[1], 1, false);
            J3Motor.setJointAngle(data[2], 1, false);
            J4Motor.setJointAngle(data[3], 1, false);
            PitchMotor.setJointAngle(data[4], 1, false);
            RollMotor.setJointAngle(data[5], 1, false);

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_GRIPPEROPENLOOP_DATA_ID: 
        {
            int16_t *data = (int16_t*) packet.data;
            GripperMotor.drive(data[0]);
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
            uint8_t data = *((uint8_t *)packet.data);
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
        case RC_ARMBOARD_CACHE_DATA_ID:
        {
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

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_CLOSEDLOOPOVERRIDE_DATA_ID:
        {

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID:
        {

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SOFTLIMITOVERRIDE_DATA_ID:
        {
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_ARMGIMBAL1_DATA_ID:
        {
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_ARMGIMBAL2_DATA_ID:
        {
            feedWatchdog();
            break;
        }
        
    }

}

void UpdateArm() //Only used for buttons
{

    // underMode = false; //J3State.getMotorAngle() > 0;
    if (J3State.getMotorAngle() >= 0) IKMode = false; // dont put it out of IK mode if pos, IK will throw out invalid solution bc of IK J3 soft limits (so remove this line)

    direction = digitalRead(DIR_SW);
    buttonInput = (digitalRead(B_ENC_3) << 3) | (digitalRead(B_ENC_2) << 2) | (digitalRead(B_ENC_1) << 1) | (digitalRead(B_ENC_0) << 0);

    if (Xcalibrating || buttonInput) IKMode = false;

    // Motor Outputs
    if (!Xcalibrated) XState.overrideClosedLoop(true);
    if (Xcalibrating) CalibrateX();
    else XState.updateJoint(buttonInput, direction);

    J2State.updateJoint(buttonInput, direction);
    J3State.updateJoint(buttonInput, direction);

    J4State.updateJoint(buttonInput, direction);
    PitchState.updateJoint(buttonInput, direction);
    RollState.updateJoint(buttonInput, direction);

    updateMotor(Gripper,GripperDecipercent,BTN_GRIPPER);
    updateMotor(Spare,SpareDecipercent,BTN_SPARE);

    // Solenoid
    if (buttonInput == BTN_SOL) setSolenoid(true);
    else setSolenoid(extendSolenoid);

    // Laser
    if (buttonInput == BTN_LAS) setLaser(true);
    else setLaser(laserOn);

}

void CalculateInverseKinematics() 
{
    float q1, q2, q3, qP;

	//Calculate target angles using IK
	q1 = CartesianCoords.x;
	q3 = RAD2DEG*acos((pow(CartesianCoords.z,2)+pow(CartesianCoords.y,2)-pow(J2_LENGTH,2)-pow(J3_LENGTH,2))/(2*J2_LENGTH*J3_LENGTH));

	// if (underMode) q2 = RAD2DEG*(atan2(CartesianCoords.y, CartesianCoords.z) - atan2(J3_LENGTH*sin(q3*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(q3*DEG2RAD))));
	// else q2 = RAD2DEG*(atan2(CartesianCoords.y, CartesianCoords.z) + atan2(J3_LENGTH*sin(q3*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(q3*DEG2RAD))));
	
    q2 = RAD2DEG*(atan2(CartesianCoords.y, CartesianCoords.z) + atan2(J3_LENGTH*sin(q3*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(q3*DEG2RAD))));

	// q3 = underMode? q3 : -q3;
	q3 = -q3;

    qP = PitchControl - (q2 + q3) * cosf(J4State.getMotorAngle()*DEG2RAD);
    qP = PitchState.bound360Degrees(qP);

	// Check if calculated angle is invalid and limit movement
	if (!(XState.isInSafeZone(q1) && J2State.isInSafeZone(q2) && J3State.isInSafeZone(q3) && PitchState.isInSafeZone(qP) && Xcalibrated)) {
        CalculateForwardKinematics();
        return;
    }

    XState.setTarget(q1);
    J2State.setTarget(q2);
    J3State.setTarget(q3);
    PitchState.setTarget(qP);

}

void UpdateLimits()
{
    if (Xcalibrated)
    {
        //Gimbal Masts
        if ((XState.getMotorAngle() < 2.0) && (J2State.getMotorAngle() < 92.0)  && (J2State.getMotorAngle() > 90.0)) {
            J2State.setForwardLimit(J2State.getMotorAngle());
            XState.setForwardLimit(X_FWD_LIM);
            XState.setReverseLimit(X_REV_LIM);
        } else if ((XState.getMotorAngle() > 6.0) && (J2State.getMotorAngle() < 92.0) && (J2State.getMotorAngle() > 90.0)) {
            J2State.setForwardLimit(J2State.getMotorAngle());
            XState.setForwardLimit(X_FWD_LIM);
            XState.setReverseLimit(X_REV_LIM);
        } else if ((J2State.getMotorAngle() > 90.0) && (XState.getMotorAngle() < 2.0) && (XState.getMotorAngle() > 1.8)) {
            XState.setReverseLimit(XState.getMotorAngle());
            J2State.setForwardLimit(J2_FWD_LIM);
        } else if ((J2State.getMotorAngle() > 90.0) && (XState.getMotorAngle() > 6.0) && (XState.getMotorAngle() < 6.2)) {
            XState.setForwardLimit(XState.getMotorAngle());
            J2State.setForwardLimit(J2_FWD_LIM);
        } else {
            XState.setForwardLimit(X_FWD_LIM);
            XState.setReverseLimit(X_REV_LIM);
            J2State.setForwardLimit(J2_FWD_LIM);
        }

        // BUG: THESE LIMITS OVERRIDE GIMBAL MAST LIMITS, chase doesnt need these
        //Wheels
        // float yPos = (J2_LENGTH * sin(J2State.getMotorAngle()*DEG2RAD)) + (J3_LENGTH * sin((J3State.getMotorAngle() + J2State.getMotorAngle())*DEG2RAD));
        // if ((yPos < -100.0) && (XState.getMotorAngle() < 2.0) && (XState.getMotorAngle() > 1.8))
        // {
        //     XState.setReverseLimit(XState.getMotorAngle());
        // }
        // else if ((yPos < -100.0) && (XState.getMotorAngle() > 6.0) && (XState.getMotorAngle() < 6.2))
        // {
        //     XState.setForwardLimit(XState.getMotorAngle());
        // }
        // else
        // {
        //     XState.setForwardLimit(X_FWD_LIM);
        //     XState.setReverseLimit(X_REV_LIM);
        // }

    }

    if (IKMode)
    {
        // if (underMode) {
        //     J3State.setForwardLimit(J3_FWD_LIM);
        //     J3State.setReverseLimit(J3_MID_LIM);
        // } else {
        //     J3State.setForwardLimit(-J3_MID_LIM);
        //     J3State.setReverseLimit(J3_REV_LIM);
        // }

        J3State.setForwardLimit(-J3_MID_LIM);
        J3State.setReverseLimit(J3_REV_LIM);
    } else {
        J3State.setForwardLimit(J3_FWD_LIM);
        J3State.setReverseLimit(J3_REV_LIM);
    }

    if ((PitchState.getMotorAngle() < 270) && (PitchState.getMotorAngle() > 90)) {
        PitchJoint.overrideForwardHardLimit(true);
        PitchJoint.overrideReverseHardLimit(false);
    } else {
        PitchJoint.overrideForwardHardLimit(false);
        PitchJoint.overrideReverseHardLimit(true);
    }

}

void CalculateForwardKinematics()
{

    CartesianCoords = {0,0,0};
	CartesianCoords = CartesianCoords * (Translate(0, 0, J3_LENGTH) * Rotate(J3State.getMotorAngle()*DEG2RAD, 0, 0) * Translate(0, 0, J2_LENGTH) * Rotate(J2State.getMotorAngle()*DEG2RAD, 0, 0) * Translate(XState.getMotorAngle(), 0, 0));
    CartesianCoords.y *= -1;

    PitchControl = PitchState.getMotorAngle() + ((J2State.getMotorAngle() + J3State.getMotorAngle()) * cosf(J4State.getMotorAngle()*DEG2RAD));

}