#include "Arm_Software.h"

#include <cmath>


void setup() {
    Serial.begin(115200);
    Serial.println("Setup");

    // Configure buttons pins
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);

    // Initialize IO Expanders
    // 1 for input, 0 for output
    // Defaults to all inputs
    IOX_TWI.begin();
    IOX1.begin();
    IOX2.begin(~((1<<IOX2_SOLENOID) | (1<<IOX2_LAS)));

    // Attach encoders
    X.attachEncoder(&Encoder1);
    Y1.attachEncoder(&Encoder2);
    Y2.attachEncoder(&Encoder3);
    Z.attachEncoder(&Encoder4);
    Pitch.attachEncoder(&Encoder7);
    Roll1.attachEncoder(&Encoder5);
    Roll2.attachEncoder(&Encoder6);

    X.attachHardLimits(&LS2, &LS1);
    Y1.attachHardLimits(&LS3, &LS4);
    Y2.attachHardLimits(&LS5, &LS6);
    Z.attachHardLimits(&LS7, &LS8);
    Pitch.attachHardLimits(&LS9, &LS10);

    // Configure encoder inverts
    X.Encoder()->configInvert(true);
    Y1.Encoder()->configInvert(false);
    Y2.Encoder()->configInvert(false);
    Z.Encoder()->configInvert(false);
    Pitch.Encoder()->configInvert(true);
    Roll1.Encoder()->configInvert(false);
    Roll2.Encoder()->configInvert(true);

    // Attach encoder interrupts, leave the same
    Encoder1.begin([]{Encoder1.handleInterrupt();});
    Encoder2.begin([]{Encoder2.handleInterrupt();});
    Encoder3.begin([]{Encoder3.handleInterrupt();});
    Encoder4.begin([]{Encoder4.handleInterrupt();});
    Encoder5.begin([]{Encoder5.handleInterrupt();});
    Encoder6.begin([]{Encoder6.handleInterrupt();});
    Encoder7.begin([]{Encoder7.handleInterrupt();});

    // Config motor inverts, reference joint
    X.Motor()->configInvert(false);
    Y1.Motor()->configInvert(false);
    Y2.Motor()->configInvert(false);
    Z.Motor()->configInvert(false);
    Pitch.Motor()->configInvert(false);
    Roll1.Motor()->configInvert(false);
    Roll2.Motor()->configInvert(true);
    Gripper1.configInvert(false);
    Gripper2.configInvert(false);
    Spare.configInvert(false);

    // Config motor output limits, reference joint
    X.Motor()->configMaxOutputs(-1000, 1000);
    Y1.Motor()->configMaxOutputs(-1000, 1000);
    Y2.Motor()->configMaxOutputs(-1000, 1000);
    Z.Motor()->configMaxOutputs(-1000, 1000);
    Pitch.Motor()->configMaxOutputs(-1000, 1000);
    Roll1.Motor()->configMaxOutputs(-1000, 1000);
    Roll2.Motor()->configMaxOutputs(-1000, 1000);
    Motor8.configMaxOutputs(-1000, 1000);
    Motor9.configMaxOutputs(-1000, 1000);
    Spare.configMaxOutputs(-1000, 1000);

    // Config motor deadbands, reference joint
    X.Motor()->configMinOutputs(-200, 200);
    Y1.Motor()->configMinOutputs(-100, 170);
    Y2.Motor()->configMinOutputs(-100, 220);
    Z.Motor()->configMinOutputs(-220, 190);
    Pitch.Motor()->configMinOutputs(-50, 50);
    Roll1.Motor()->configMinOutputs(-200, 200);
    Roll2.Motor()->configMinOutputs(-130, 130);
    Gripper1.configMinOutputs(-50, 50);
    Gripper2.configMinOutputs(-50, 50);
    Spare.configMinOutputs(-50, 50);

    // Config motor ramp rates, reference joint
    X.Motor()->configRampRate(10000);
    Y1.Motor()->configRampRate(10000);
    Y2.Motor()->configRampRate(10000);
    Z.Motor()->configRampRate(10000);
    Pitch.Motor()->configRampRate(10000);
    Roll1.Motor()->configRampRate(10000);
    Roll2.Motor()->configRampRate(10000);
    Gripper1.configRampRate(10000);
    Gripper2.configRampRate(10000);
    Spare.configRampRate(10000);

    // X soft limits
    //X.configSoftLimits(0, 8); 
    X.overrideReverseSoftLimit(true);
    X.overrideForwardSoftLimit(true);

    // Y1 soft limits
    Y1.configSoftLimits(0, Y1_MAX);
    Y1.overrideReverseSoftLimit(true);
    Y1.overrideForwardSoftLimit(true);

    // Y2 soft limits
    Y2.configSoftLimits(0, Y2_MAX);
    Y2.overrideReverseSoftLimit(true);
    Y2.overrideForwardSoftLimit(true);

    // Z soft limits
    Z.configSoftLimits(0, 7.625);
    Z.overrideReverseSoftLimit(true);
    Z.overrideForwardSoftLimit(true);

    // Pitch soft limits
    // Pitch.configForwardSoftLimit(310);
    // Pitch.configReverseSoftLimit(0);
    // Pitch encoder is absolute, so set predefined offset without calibration
    Pitch_state.calibrated = true;
    Pitch.Encoder()->configOffset(0);

    // Roll
    Roll1_PID.enableContinuousFeedback(0, 360);
    Roll2_PID.enableContinuousFeedback(0, 360);

    // Attach PID
    X.attachPID(&X_PID);
    Y1.attachPID(&Y1_PID);
    Y2.attachPID(&Y2_PID);
    Z.attachPID(&Z_PID);
    Pitch.attachPID(&Pitch_PID);
    Roll1.attachPID(&Roll1_PID);
    Roll2.attachPID(&Roll2_PID);
    
    // RoveComm
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_ARMBOARD_FIRSTOCTET, RC_ARMBOARD_SECONDOCTET, RC_ARMBOARD_THIRDOCTET, RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    Serial.println("Complete");
    
    feedWatchdog();
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}



void loop() {
    uint32_t timestamp = millis();

    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        case RC_ARMBOARD_OPENLOOP_DATA_ID:
        {
            int16_t *data = (int16_t*)packet.data;
            
            X_state.decipercent = data[0];
            Y1_state.decipercent = data[1];
            Y2_state.decipercent = data[2];
            Z_state.decipercent = data[3];
            Pitch_state.decipercent = data[4];
            
            if (activeGripper) {
                Roll1_state.decipercent = 0;
                Roll2_state.decipercent = data[5];
            } else {
                Roll1_state.decipercent = data[5];
                Roll2_state.decipercent = 0;
            }

            closedLoopActive = false;
            feedWatchdog();

            break;
        }

        case RC_ARMBOARD_SETPOSITION_DATA_ID:
        {
            float *data = (float*) packet.data;

            X_state.target = data[0];
            Y1_state.target = data[1];
            Y2_state.target = data[2];
            Z_state.target = data[3];
            Pitch_state.target = data[4];

            if (activeGripper) {
                Roll2_state.target = data[5];
            } else {
                Roll1_state.target = data[5];
            }

            closedLoopActive = true;
            feedWatchdog();

            break;
        }

        case RC_ARMBOARD_INCREMENTPOSITION_DATA_ID:
        {
            float *data = (float*) packet.data;

            X_state.target += data[0];
            Y1_state.target += data[1];
            Y2_state.target += data[2];
            Z_state.target += data[3];
            Pitch_state.target += data[4];

            if (activeGripper) {
                Roll2_state.target += data[5];
            } else {
                Roll1_state.target += data[5];
            }

            closedLoopActive = true;
            feedWatchdog();

            break;
        }

        case RC_ARMBOARD_SETIK_DATA_ID:
        {

            break;
        }

        case RC_ARMBOARD_INCREMENTIK_ROVERRELATIVE_DATA_ID:
        {

            break;
        }

        case RC_ARMBOARD_INCREMENTIK_WRISTRELATIVE_DATA_ID:
        {

            break;
        }

        case RC_ARMBOARD_LASER_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);

            laserOn = (data == 0)? false : true;
            break;
        }

        case RC_ARMBOARD_SOLENOID_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);

            extendSolenoid = (data == 0)? false : true;
            break;
        }

        case RC_ARMBOARD_GRIPPER_DATA_ID:
        {
            int16_t data = *((int16_t*) packet.data);

            switch(activeGripper) {
                case 0:
                    Gripper1_decipercent = data;
                    Gripper2_decipercent = 0;
                    break;
                case 1:
                    Gripper1_decipercent = 0;
                    Gripper2_decipercent = data;
                    break;
            }
            break;
        }

        case RC_ARMBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            watchdogOverride = *((uint8_t*) packet.data);
            break;
        }

        case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        {
            uint16_t data = *((uint16_t*) packet.data);

            X.overrideForwardHardLimit(data & (1<<0));
            X.overrideReverseHardLimit(data & (1<<1));
            Y1.overrideForwardHardLimit(data & (1<<2));
            Y1.overrideReverseHardLimit(data & (1<<3));
            Y2.overrideForwardHardLimit(data & (1<<4));
            Y2.overrideReverseHardLimit(data & (1<<5));
            Z.overrideForwardHardLimit(data & (1<<6));
            Z.overrideReverseHardLimit(data & (1<<7));
            Pitch.overrideForwardHardLimit(data & (1<<8));
            Pitch.overrideForwardSoftLimit(data & (1<<8));
            Pitch.overrideReverseHardLimit(data & (1<<9));
            Pitch.overrideReverseSoftLimit(data & (1<<9));
            break;
        }


        case RC_ARMBOARD_CALIBRATEENCODER_DATA_ID:
        {
            uint8_t data = *((uint8_t*) packet.data);
            
            // Linear joints need to calibrate
            X_state.calibrating = data & (1<<0);
            Y1_state.calibrating = data & (1<<1);
            Y2_state.calibrating = data & (1<<2);
            Z_state.calibrating = data & (1<<3);

            // Pitch joint has no calibration

            // Roll joint calibration is manual
            if (data & (1<<5)) {
                Roll1_state.calibrated = true;
                Roll1.Encoder()->setDegrees(0);
            }
            if (data & (1<<6)) {
                Roll2_state.calibrated = true;
                Roll2.Encoder()->setDegrees(0);
            }
            
            break;
        }

        case RC_ARMBOARD_SELECTGRIPPER_DATA_ID:
        {
            activeGripper = *((uint8_t*) packet.data);
            break;
        }

        case 8069:
        {
            // data[0]: joint to use
            // data[1-3]: kP, kI, kD
            float *data = (float*) packet.data;

            // RoveComm only allows packets of one data type, so round float to nearest integer
            uint8_t jointId = (uint8_t) (data[0] + 0.5);

            RovePIDController *pid = nullptr;
            switch (jointId) {
                case 1: pid = &X_PID; break;
                case 2: pid = &Y1_PID; break;
                case 3: pid = &Y2_PID; break;
                case 4: pid = &Z_PID; break;
                case 5: pid = &Pitch_PID; break;
                case 6: pid = &Roll1_PID; break;
                case 7: pid = &Roll2_PID; break;
            }

            if (pid != nullptr) {
                pid->configPID(data[1], data[2], data[3]);
            }

            break;
        }

        case RC_ARMBOARD_SOFTLIMITOVERRIDE_DATA_ID:
        {
            uint16_t data = *((uint16_t*) packet.data);

            X.overrideForwardSoftLimit(data & (1<<0));
            X.overrideReverseSoftLimit(data & (1<<1));
            Y1.overrideForwardSoftLimit(data & (1<<2));
            Y1.overrideReverseSoftLimit(data & (1<<3));
            Y2.overrideForwardSoftLimit(data & (1<<4));
            Y2.overrideReverseSoftLimit(data & (1<<5));
            Z.overrideForwardSoftLimit(data & (1<<6));
            Z.overrideReverseSoftLimit(data & (1<<7));
            Pitch.overrideForwardSoftLimit(data & (1<<8));
            Pitch.overrideReverseSoftLimit(data & (1<<9));
        }
    }


    // IO Expanders
    if (timestamp - lastIOX_timestamp > IOX_UPDATE_PERIOD) {
        lastIOX_timestamp = timestamp;

        // IO Expander 1
        uint8_t iox1_val = IOX1.read8();
        LS1.set(iox1_val & (1<<IOX1_LIM_1));
        LS2.set(iox1_val & (1<<IOX1_LIM_2));
        LS3.set(iox1_val & (1<<IOX1_LIM_3));
        LS4.set(iox1_val & (1<<IOX1_LIM_4));
        LS5.set(iox1_val & (1<<IOX1_LIM_5));
        LS6.set(iox1_val & (1<<IOX1_LIM_6));
        LS7.set(iox1_val & (1<<IOX1_LIM_7));
        LS8.set(iox1_val & (1<<IOX1_LIM_8));

        // IO Expander 2
        uint8_t iox2_val = IOX2.read8();
        LS9.set(iox2_val & (1<<IOX2_LIM_9));
        LS10.set(iox2_val & (1<<IOX2_LIM_10));
        direction = iox2_val & (1<<IOX2_DIR_SW);
    }

    // Buttons
    buttons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    // Motor outputs
    updateJoint(X, X_state, BTN_1);
    updateJoint(Y1, Y1_state, BTN_2, true, Y1_MAX);
    updateJoint(Y2, Y2_state, BTN_3, true, Y2_MAX);
    updateJoint(Z, Z_state, BTN_4);
    updateJoint(Pitch, Pitch_state, BTN_5);
    updateJoint(Roll1, Roll1_state, BTN_6);
    updateJoint(Roll2, Roll2_state, BTN_7);

    updateMotor(Gripper1, Gripper1_decipercent, BTN_8);
    updateMotor(Gripper2, Gripper2_decipercent, BTN_9);
    updateMotor(Spare, 0, BTN_10);

    // Solenoid
    if (buttons == BTN_11) setSolenoid(true);
    else setSolenoid(extendSolenoid);

    // Laser
    setLaser(laserOn);

    float ZHeight = ((Y1.Encoder()->readDegrees()) + ((Y2.Encoder()->readDegrees()) - 4));

    // Update X and Y Soft Limits to prevent Z axis from smashing
    if (X_state.calibrated && Y1_state.calibrated && Y2_state.calibrated) {
        if (ZHeight > 14.5) {
          X.configSoftLimits(0, 8);
          if ((ZHeight < 15) && ((X.Encoder()->readDegrees()) < 3)) {
            Y1.configSoftLimits(Y1.Encoder()->readDegrees(), Y1_MAX);
            Y2.configSoftLimits(Y2.Encoder()->readDegrees(), Y2_MAX);
          } else {
            Y1.configSoftLimits(0, Y1_MAX);
            Y2.configSoftLimits(0, Y2_MAX);
          }
        } else {
          X.configSoftLimits(3, 8);
          Y1.configSoftLimits(0, Y1_MAX);
          Y2.configSoftLimits(0, Y2_MAX);
        }
   }

    //Zero Pitch when lemon switch is pressed
    if (Pitch.atReverseHardLimit()) {
      Pitch.Encoder()->setDegrees(0);
    }

}


void updateJoint(RoveJoint &joint, JointState &state, uint8_t button, bool calibrateUp, float position) {
    if (buttons == button) {
        joint.overrideReverseSoftLimit(true);
        joint.overrideForwardSoftLimit(true);
        joint.drive((direction? -900 : 900));
        joint.overrideReverseSoftLimit(false);
        joint.overrideForwardSoftLimit(false);
    } else if (state.calibrating) {
        if (joint.atForwardHardLimit() || joint.atReverseHardLimit()) {
            joint.overrideReverseSoftLimit(false);
            joint.overrideForwardSoftLimit(false);
            joint.drive(0);
            joint.Encoder()->setDegrees(position);
            state.calibrating = false;
            state.calibrated = true;
        } else {
            joint.overrideReverseSoftLimit(true);
            joint.overrideForwardSoftLimit(true);
            calibrateUp? joint.drive(900) : joint.drive(-900);
        }
    } else if (closedLoopActive) {
        if (state.calibrated) {
            joint.setAngle(state.target);
        } else {
            joint.drive(0);
        }
    } else {
        joint.drive(state.decipercent);
    }
}

void updateMotor(RoveMotor &motor, int16_t decipercent, uint8_t button) {
    uint8_t buttons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    if (buttons == button) {
        motor.drive((direction? -900 : 900));
    } else {
        motor.drive(decipercent);
    }
}


inline void setSolenoid(bool extend) {
    IOX2.write(IOX2_SOLENOID, extend? HIGH : LOW);
}

inline void setLaser(bool on) {
    IOX2.write(IOX2_LAS, on? HIGH : LOW);
}


void estop() {
    if (!watchdogOverride) {
        watchdogStatus = 1;

        closedLoopActive = false;

        X_state.decipercent = 0;
        Y1_state.decipercent = 0;
        Y2_state.decipercent = 0;
        Z_state.decipercent = 0;
        Pitch_state.decipercent = 0;
        Roll1_state.decipercent = 0;
        Roll2_state.decipercent = 0;
        Gripper1_decipercent = 0;
        Gripper2_decipercent = 0;
    }
}

void telemetry() {
    RoveComm.write(RC_ARMBOARD_WATCHDOGSTATUS_DATA_ID, RC_ARMBOARD_WATCHDOGSTATUS_DATA_COUNT, watchdogStatus);

    RoveComm.write(69, 1, ZHeight);

    if (!telemetryOverride) {
        float positions[7] = {X.Encoder()->readDegrees(), Y1.Encoder()->readDegrees(), Y2.Encoder()->readDegrees(), Z.Encoder()->readDegrees(),
                                Pitch.Encoder()->readDegrees(), Roll1.Encoder()->readDegrees(), Roll2.Encoder()->readDegrees()};
        RoveComm.write(RC_ARMBOARD_POSITIONS_DATA_ID, RC_ARMBOARD_POSITIONS_DATA_COUNT, positions);

        // float coordinates[5] = {0, 0, 0, 0, 0};
        // RoveComm.write(RC_ARMBOARD_COORDINATES_DATA_ID, RC_ARMBOARD_COORDINATES_DATA_COUNT, coordinates);

        // uint8_t limitSwitches = (X.atForwardHardLimit() << 0) | (X.atReverseHardLimit() << 1) | (Y1.atForwardHardLimit() << 2) | (Y1.atReverseHardLimit() << 3) |
        //                         (Y2.atForwardHardLimit() << 4) | (Y2.atReverseHardLimit() << 5) | (Z.atForwardHardLimit() << 6) | (Z.atReverseHardLimit() << 7) | 
        //                         (Pitch.atForwardHardLimit() << 8) | (Pitch.atReverseHardLimit() << 9);
        // RoveComm.write(RC_ARMBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_ARMBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitches);
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
