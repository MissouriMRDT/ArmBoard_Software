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
    UpdateFromIOX();
    UpdateArm();
    
}

void estop() 
{
    if (!watchdogOverride)
    {
        watchdogStatus = 1;

        XState.overrideClosedLoop(true);
        J2State.overrideClosedLoop(true);
        J3State.overrideClosedLoop(true);
        J4State.overrideClosedLoop(true);
        PitchState.overrideClosedLoop(true);
        RollState.overrideClosedLoop(true);

        IKMode = false;

    }
}

void telemetry() 
{
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

void setLaser(bool on) { digitalWrite(LAS,on? HIGH:LOW); }

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
        case RC_ARMBOARD_SETINDIVIDUALSPEEDS_DATA_ID: //done
        {
            // Set joint decipercent
            int16_t *data = (int16_t *)packet.data;
            XState.setDecipercent(data[0]);
            J2State.setDecipercent(data[1]);
            J3State.setDecipercent(data[2]);
            J4State.setDecipercent(data[3]);
            PitchState.setDecipercent(data[4]);
            RollState.setDecipercent(data[5]);

            XState.setControlMode(0);
            J2State.setControlMode(0);
            J3State.setControlMode(0);
            J4State.setControlMode(0);
            PitchState.setControlMode(0);
            RollState.setControlMode(0);

            IKMode = false;

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SETJOINTSPEED_DATA_ID: //done
        {
            int16_t *data = (int16_t*) packet.data;
            switch (data[0])
            {
                case X:
                    XState.setDecipercent(data[1]);
                    XState.setControlMode(0);
                    break;
                case J2:
                    J2State.setDecipercent(data[1]);
                    J2State.setControlMode(0);
                    break;
                case J3:
                    J3State.setDecipercent(data[1]);
                    J3State.setControlMode(0);
                    break;
                case J4:
                    J4State.setDecipercent(data[1]);
                    J4State.setControlMode(0);
                    break;
                case PITCH:
                    PitchState.setDecipercent(data[1]);
                    PitchState.setControlMode(0);
                    break;
                case ROLL:
                    RollState.setDecipercent(data[1]);
                    RollState.setControlMode(0);
                    break;
            }

            IKMode = false;

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SETINDIVIDUALTARGETANGLES_DATA_ID: //done
        {
            float *data = (float*) packet.data;
            XState.setTarget(data[0]);
            J2State.setTarget(data[1]);
            J3State.setTarget(data[2]);
            J4State.setTarget(data[3]);
            PitchState.setTarget(data[4]);
            RollState.setTarget(data[5]);

            XState.setControlMode(1);
            J2State.setControlMode(1);
            J3State.setControlMode(1);
            J4State.setControlMode(1);
            PitchState.setControlMode(1);
            RollState.setControlMode(1);

            IKMode = false;
            
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SETJOINTTARGETANGLE_DATA_ID: //done
        {
            float *data = (float*) packet.data;

            switch (uint8_t(data[0]))
            {
                case X:
                    XState.setTarget(data[1]);
                    XState.setControlMode(1);
                    break;
                case J2:
                    J2State.setTarget(data[1]);
                    J2State.setControlMode(1);
                    break;
                case J3:
                    J3State.setTarget(data[1]);
                    J3State.setControlMode(1);
                    break;
                case J4:
                    J4State.setTarget(data[1]);
                    J4State.setControlMode(1);
                    break;
                case PITCH:
                    PitchState.setTarget(data[1]);
                    PitchState.setControlMode(1);
                    break;
                case ROLL:
                    RollState.setTarget(data[1]);
                    RollState.setControlMode(1);
                    break;
            }

            IKMode = false;

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_INCREMENTINDIVIDUALTARGETANGLES_DATA_ID:
        {
            float *data = (float*) packet.data;

            XState.incrementTarget(data[0]);
            J2State.incrementTarget(data[1]);
            J3State.incrementTarget(data[2]);
            J4State.incrementTarget(data[3]);
            PitchState.incrementTarget(data[4]);
            RollState.incrementTarget(data[5]);

            XState.setControlMode(1);
            J2State.setControlMode(1);
            J3State.setControlMode(1);
            J4State.setControlMode(1);
            PitchState.setControlMode(1);
            RollState.setControlMode(1);

            IKMode = false;
            
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_INCREMENTJOINTTARGETANGLE_DATA_ID:
        {
            float *data = (float*) packet.data;
            switch (uint8_t(data[0]))
            {
                case X:
                    XState.incrementTarget(data[1]);
                    XState.setControlMode(1);
                    break;
                case J2:
                    J2State.incrementTarget(data[1]);
                    J2State.setControlMode(1);
                    break;
                case J3:
                    J3State.incrementTarget(data[1]);
                    J3State.setControlMode(1);
                    break;
                case J4:
                    J4State.incrementTarget(data[1]);
                    J4State.setControlMode(1);
                    break;
                case PITCH:
                    PitchState.incrementTarget(data[1]);
                    PitchState.setControlMode(1);
                    break;
                case ROLL:
                    RollState.incrementTarget(data[1]);
                    RollState.setControlMode(1);
                    break;
            }

            IKMode = false;

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SETIKPOSITION_DATA_ID:
        {
            if (!IKMode)
            {
                IKMode = true;
                CalculateForwardKinematics();
            }

            float *data = (float*) packet.data;

            CartesianCoords.x = data[0];
            CartesianCoords.y = data[1];
            CartesianCoords.z = data[2];

            J4State.setTarget(data[3]);
            PitchControl = data[4];
            RollState.setTarget(data[5]);

            CalculateInverseKinematics();
            
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_INCREMENTIKPOSITION_DATA_ID:
        {
            if (!IKMode)
            {
                IKMode = true;
                CalculateForwardKinematics();
            }

            float *data = (float*) packet.data;

            CartesianCoords.x += data[0];
            CartesianCoords.y += data[1];
            CartesianCoords.z += data[2];

            J4State.incrementTarget(data[3]);
            PitchControl += data[4];
            RollState.incrementTarget(data[5]);

            CalculateInverseKinematics();

            feedWatchdog();
            break;      
        }
        case RC_ARMBOARD_SETLOCKMODEPOSITION_DATA_ID:
        {
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_INCREMENTLOCKMODEPOSITION_DATA_ID:
        {
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_LASER_DATA_ID: //done
        {
            uint8_t data = *((uint8_t *)packet.data);
            laserOn = (data == 0) ? false : true;
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SOLENOID_DATA_ID: //done
        {
            uint8_t data = *((uint8_t *)packet.data);
            extendSolenoid = (data == 0) ? false : true;
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SETGRIPPERSPEED_DATA_ID: //done
        {
            int16_t *data = (int16_t*) packet.data;
            GripperDecipercent = data[0];
            SpareDecipercent = data[1];
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID: //done
        {
            watchdogOverride = *((uint8_t*) packet.data);
            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID: //done
        {
            uint16_t data = *((uint16_t*) packet.data);

            XJoint.overrideForwardHardLimit(data & (1 << 0));
            XJoint.overrideReverseHardLimit(data & (1 << 1));
            J2Joint.overrideForwardHardLimit(data & (1 << 2));
            J2Joint.overrideReverseHardLimit(data & (1 << 3));
            J3Joint.overrideForwardHardLimit(data & (1 << 4));
            J3Joint.overrideReverseHardLimit(data & (1 << 5));
            J4Joint.overrideForwardHardLimit(data & (1 << 6));
            J4Joint.overrideReverseHardLimit(data & (1 << 7));
            PitchJoint.overrideForwardHardLimit(data & (1 << 8));
            PitchJoint.overrideReverseHardLimit(data & (1 << 8));

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_CLOSEDLOOPOVERRIDE_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);

            XState.overrideClosedLoop(data & (1 << 0));
            J2State.overrideClosedLoop(data & (1 << 1));
            J3State.overrideClosedLoop(data & (1 << 2));
            J4State.overrideClosedLoop(data & (1 << 3));
            PitchState.overrideClosedLoop(data & (1 << 4));
            RollState.overrideClosedLoop(data & (1 << 5));

            IKMode = false;

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);

            if(data & (1<<1)) RollJoint.Encoder()->setDegrees(0);
            if (J2State.getMotorAngle() < 90.0) Xcalibrating = data & (1 << 0);

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_SOFTLIMITOVERRIDE_DATA_ID:
        {
            uint16_t data = *((uint16_t*) packet.data);

            XState.overrideForwardSoftLimit(data & (1 << 0));
            XState.overrideReverseSoftLimit(data & (1 << 1));
            J2State.overrideForwardSoftLimit(data & (1 << 2));
            J2State.overrideReverseSoftLimit(data & (1 << 3));
            J3State.overrideForwardSoftLimit(data & (1 << 4));
            J3State.overrideReverseSoftLimit(data & (1 << 5));
            J4State.overrideForwardSoftLimit(data & (1 << 6));
            J4State.overrideReverseSoftLimit(data & (1 << 7));
            PitchState.overrideForwardSoftLimit(data & (1 << 8));
            PitchState.overrideReverseSoftLimit(data & (1 << 9));

            feedWatchdog();
            break;
        }
        case RC_ARMBOARD_ESTOP_DATA_ID:
        {
            estop();

            feedWatchdog();
            break;
        }
    }

}

void UpdateFromIOX()
{

    uint32_t timestamp = millis();

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

}

void UpdateArm() 
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