#include "ArmTrain_Software.h"

//Motor Buttons
const uint8_t MOTOR_1 = PD_7;
const uint8_t MOTOR_2 = PA_6;
const uint8_t MOTOR_3 = PM_4;
const uint8_t MOTOR_4 = PM_5;
const uint8_t MOTOR_5 = PD_0;
const uint8_t MOTOR_6 = PN_2;

void setup() 
{
    pinMode(MOTOR_1,INPUT); //MOTORS 1-6
    pinMode(MOTOR_2,INPUT);
    pinMode(MOTOR_3,INPUT);
    pinMode(MOTOR_4,INPUT);
    pinMode(MOTOR_5,INPUT);
    pinMode(MOTOR_6,INPUT);

    pinMode(LS_LOWER_BICEP,OUTPUT); //LS 1-4
    pinMode(LS_UPPER_BICEP,OUTPUT);
    pinMode(LS_LOWER_ELBOW,OUTPUT);
    pinMode(LS_UPPER_ELBOW,OUTPUT);

    pinMode(ENC_WRIST_TILT,OUTPUT); //ENC 1-6
    pinMode(ENC_WRIST_TWIST,OUTPUT);
    pinMode(ENC_ELBOW_TILT,OUTPUT);
    pinMode(ENC_ELBOW_TWIST,OUTPUT);
    pinMode(ENC_BICEP_TILT,OUTPUT);
    pinMode(ENC_BICEP_TWIST,OUTPUT);

    pinMode(SW1_LED,OUTPUT); //SW 1-3
    pinMode(SW2_LED,OUTPUT);
    pinMode(ERROR_LED,OUTPUT);

    pinMode(GRIPPER_INA,OUTPUT); //GRIPPER
    pinMode(GRIPPER_INB,OUTPUT);
    pinMode(GRIPPER_PWM,OUTPUT);

    pinMode(PC_4,INPUT); //Rx left
    pinMode(PC_5,OUTPUT); //Tx left
    pinMode(PK_0,INPUT); //Rx mid
    pinMode(PK_1,OUTPUT); //Tx mid
    pinMode(PP_0,INPUT); //Rx right
    pinMode(PP_1,OUTPUT); //Tx right
}

void loop() 
{
    if(digitalRead(MOTOR_1)==HIGH) //motor 1
    {
        digitalWrite(LS_LOWER_BICEP,HIGH); //ls1
        digitalWrite(LS_UPPER_BICEP,HIGH); //ls2
        digitalWrite(LS_LOWER_ELBOW,HIGH); //ls3
        digitalWrite(LS_UPPER_ELBOW,HIGH); //ls4
    }
    if(digitalRead(MOTOR_2)==HIGH) //motor 2
    {
        digitalWrite(SW1_LED,HIGH); //sw1
        digitalWrite(SW2_LED,HIGH); //sw2
        digitalWrite(ERROR_LED,HIGH); //sw3
    }
    if(digitalRead(MOTOR_3)==HIGH) //motor 3
    {
        analogWrite(ENC_WRIST_TILT,255); //enc1
        analogWrite(ENC_WRIST_TWIST,255); //enc2
        analogWrite(ENC_ELBOW_TILT,255); //enc3
        analogWrite(ENC_ELBOW_TWIST,255); //enc4
        analogWrite(ENC_BICEP_TILT,255); //enc5
        analogWrite(ENC_BICEP_TWIST,255); //enc6
    }
    if(digitalRead(MOTOR_4)==HIGH) //motor 4
    {
        digitalWrite(PC_5,HIGH); //Tx left
        digitalWrite(PK_1,HIGH); //Tx mid
        digitalWrite(PP_1,HIGH); //Tx right
    }
    else
    {
        digitalWrite(LS_LOWER_BICEP,LOW); //ls1
        digitalWrite(LS_UPPER_BICEP,LOW); //ls2
        digitalWrite(LS_LOWER_ELBOW,LOW); //ls3
        digitalWrite(LS_UPPER_ELBOW,LOW); //ls4

        digitalWrite(SW1_LED,LOW); //sw1
        digitalWrite(SW2_LED,LOW); //sw2
        digitalWrite(ERROR_LED,LOW); //sw3

        analogWrite(ENC_WRIST_TILT,0); //enc1
        analogWrite(ENC_WRIST_TWIST,0); //enc2
        analogWrite(ENC_ELBOW_TILT,0); //enc3
        analogWrite(ENC_ELBOW_TWIST,0); //enc4
        analogWrite(ENC_BICEP_TILT,0); //enc5
        analogWrite(ENC_BICEP_TWIST,0); //enc6

        digitalWrite(PC_5,LOW); //Tx left
        digitalWrite(PK_1,LOW); //Tx mid
        digitalWrite(PP_1,LOW); //Tx right
    }
}


