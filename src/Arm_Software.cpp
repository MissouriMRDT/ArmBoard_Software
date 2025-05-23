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
    XJoint.attachEncoder(&XEncoder);
    J2Joint.attachEncoder(&J2Encoder);
    J3Joint.attachEncoder(&J3Encoder);
    J4Joint.attachEncoder(&J4Encoder);
    PitchJoint.attachEncoder(&PitchEncoder);
    RollJoint.attachEncoder(&RollEncoder);

    // Attach hard limits
    XJoint.attachHardLimits(&LS3, &LS4);
    J2Joint.attachHardLimits(&LS10, &LS9);
    J3Joint.attachHardLimits(&LS8, &LS7);
    J4Joint.attachHardLimits(&LS6, &LS5);
    PitchJoint.attachHardLimits(&LS2, &LS2);

    // Attach encoder inverts
    XJoint.Encoder()->configInvert(false);
    J2Joint.Encoder()->configInvert(true);
    J3Joint.Encoder()->configInvert(true);
    J4Joint.Encoder()->configInvert(false);
    PitchJoint.Encoder()->configInvert(false);
    RollJoint.Encoder()->configInvert(false);

    J2Joint.Encoder()->configNegativeDegrees(true);
    J3Joint.Encoder()->configNegativeDegrees(true);

    // Attach encoder offsets
    J2Joint.Encoder()->configOffset(-177.89); //125.94
    J3Joint.Encoder()->configOffset(45.53); //346.03
    J4Joint.Encoder()->configOffset(265.46); //280.46
    PitchJoint.Encoder()->configOffset(249.44); //98.44 197.43

    // Configrue encoder interupts
    J2Encoder.begin([]{J2Encoder.handleInterrupt();});
    J3Encoder.begin([]{J3Encoder.handleInterrupt();});
    J4Encoder.begin([]{J4Encoder.handleInterrupt();});
    PitchEncoder.begin([]{PitchEncoder.handleInterrupt();});
    XEncoder.begin([]{XEncoder.handleInterrupt();});
    RollEncoder.begin([]{RollEncoder.handleInterrupt();});

    // Config motor inverts, reference joint
    XJoint.Motor()->configInvert(false);
    J2Joint.Motor()->configInvert(false);
    J3Joint.Motor()->configInvert(true);
    J4Joint.Motor()->configInvert(true);
    PitchJoint.Motor()->configInvert(true);
    RollJoint.Motor()->configInvert(false);
    Gripper.configInvert(true);
    Spare.configInvert(true);

    // Config motor output limits, reference joint
    XJoint.Motor()->configMaxOutputs(-1000, 1000);
    J2Joint.Motor()->configMaxOutputs(-1000, 1000);
    J3Joint.Motor()->configMaxOutputs(-1000, 1000);
    J4Joint.Motor()->configMaxOutputs(-1000, 1000);
    PitchJoint.Motor()->configMaxOutputs(-1000, 1000);
    Gripper.configMaxOutputs(-1000, 1000);
    Spare.configMaxOutputs(-1000, 1000);

    // Config motor deadbands, reference joint
    XJoint.Motor()->configMinOutputs(-200, 200);    //CHANGE: PID deci% floor depending on arm config to prevent arm falling because of gravity
    J2Joint.Motor()->configMinOutputs(-100, 170);  
    J3Joint.Motor()->configMinOutputs(-100, 220);  
    J4Joint.Motor()->configMinOutputs(-220, 190);  
    PitchJoint.Motor()->configMinOutputs(-50, 50); 
    RollJoint.Motor()->configMinOutputs(-50, 50);
    Gripper.configMinOutputs(-50, 50);       
    Spare.configMinOutputs(-50, 50);  

    // Config motor ramp rates, reference joint
    XJoint.Motor()->configRampRate(10000);
    J2Joint.Motor()->configRampRate(10000);
    J3Joint.Motor()->configRampRate(10000);
    J4Joint.Motor()->configRampRate(10000);
    PitchJoint.Motor()->configRampRate(10000);
    RollJoint.Motor()->configRampRate(10000);
    Gripper.configRampRate(10000);
    Spare.configRampRate(10000);


    XState.overrideForwardSoftLimit(true);
    XState.overrideReverseSoftLimit(true);

    PitchState.overrideReverseSoftLimit(true);
    PitchState.overrideForwardSoftLimit(true);

    RollState.overrideReverseSoftLimit(true);
    RollState.overrideForwardSoftLimit(true);

    RollPID.enableContinuousFeedback(0, 360);
    J4PID.configOutputLimits(-1023, 1023); //PID can calculate a too high deci%, limit deci% to +/-1023
    J4PID.configIZone(10);
    PitchPID.configIZone(10);

    J4PID.configOffset(-90);
    PitchPID.configOffset(90);

    // Attach PID
    XJoint.attachPID(&XPID);
    J2Joint.attachPID(&J2PID);
    J3Joint.attachPID(&J3PID);
    J4Joint.attachPID(&J4PID);
    PitchJoint.attachPID(&PitchPID);
    RollJoint.attachPID(&RollPID);

    J4State.setBoundTo360(true);
    PitchState.setBoundTo360(true);
    RollState.setBoundTo360(true);

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