#include "ArmTrain_Software.h"


//Pin definitions

//Motor Buttons
const uint8_t MOTOR_1 = PE_0;
const uint8_t MOTOR_2 = PE_1;
const uint8_t MOTOR_3 = PE_2;
const uint8_t MOTOR_4 = PE_3;
const uint8_t MOTOR_5 = PD_7;
const uint8_t MOTOR_6 = PA_6;
const uint8_t MOTOR_7 = PM_4;

//Motor Inputs
const uint8_t IN_A_1 = PC_6;
const uint8_t IN_B_1 = PE_5;
const uint8_t IN_A_2 = PD_3;
const uint8_t IN_B_2 = PC_7;
const uint8_t IN_A_3 = PB_2;
const uint8_t IN_B_3 = PB_3;
const uint8_t IN_A_4 = PD_4;
const uint8_t IN_B_4 = PD_5;
const uint8_t IN_A_5 = PQ_0;
const uint8_t IN_B_5 = PP_4;
const uint8_t IN_A_6 = PN_5;
const uint8_t IN_B_6 = PN_4;
const uint8_t IN_A_7 = PP_1;
const uint8_t IN_B_7 = PP_0;

//Motor PWM
const uint8_t PWM_1 = PH_3;
const uint8_t PWM_2 = PD_1;
const uint8_t PWM_3 = PD_0;
const uint8_t PWM_4 = PD_2;
const uint8_t PWM_5 = PM_7;
const uint8_t PWM_6 = PA_5;
const uint8_t PWM_7 = PM_0;

//Encoders
const uint8_t ENC_1 = PM_1;
const uint8_t ENC_2 = PM_2;
const uint8_t ENC_3 = PH_0;
const uint8_t ENC_4 = PH_1;
const uint8_t ENC_5 = PK_6;
const uint8_t ENC_6 = PK_7;

//Limit Switches
const uint8_t LIM_1 = PP_5;
const uint8_t LIM_2 = PA_7;
const uint8_t LIM_3 = PQ_2;
const uint8_t LIM_4 = PQ_4;

//External LEDS
const uint8_t LED_1 = PM_6;
const uint8_t LED_2 = PP_3;

#define NUM_INPUT 8
#define NUM_OUTPUT 21

uint8_t OUTPUT_PINS[NUM_OUTPUT] = {};

uint8_t INPUT_PINS[NUM_INPUT] = {MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, MOTOR_5, MOTOR_6, MOTOR_7};

void setup() 
{
    //All input pins
    for(int i = 0; i<NUM_INPUT; i++)
    {
        pinMode(INPUT_PINS[i], INPUT);
    }

    //All output pins
    for(int i = 0; i < NUM_OUTPUT; i++)
    {
        pinMode(OUTPUT_PINS[i], OUTPUT);
    }


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


