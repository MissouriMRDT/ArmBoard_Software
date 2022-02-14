#include "ArmTrain_Software.h"

#define NUM_INPUT 17
#define NUM_OUTPUT 25

bool laser_on = false;
bool soleniod_on = false;
uint8_t OUTPUT_PINS[NUM_OUTPUT] = {
    IN_A_1, IN_A_2, IN_A_3, IN_A_4, IN_A_5, IN_A_6, IN_A_7,
    IN_B_1, IN_B_2, IN_B_3, IN_B_4, IN_B_5, IN_B_6, IN_B_7, 
    PWM_1, PWM_2, PWM_3, PWM_4, PWM_5, PWM_6, PWM_7,
    LED_1, LED_2,
    LASER_EN, SOLENIOD_EN
};

uint8_t INPUT_PINS[NUM_INPUT] = {
    MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, MOTOR_5, MOTOR_6, MOTOR_7, 
    ENC_1, ENC_2, ENC_3, ENC_4, ENC_5, ENC_6,
    LIM_1, LIM_2, LIM_3, LIM_4
};

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


    //UART setup
    pinMode(PC_4,INPUT); //Rx left
    pinMode(PC_5,OUTPUT); //Tx left
    pinMode(PK_0,INPUT); //Rx mid
    pinMode(PK_1,OUTPUT); //Tx mid
    pinMode(PP_0,INPUT); //Rx right
    pinMode(PP_1,OUTPUT); //Tx right
}

void loop() 
{
  //I have no clue what this shit does
  //Best guess is it tests the buttons and the individual unit tests that need to be done.
  
  //Poll keys for press detection.
  bool m_1_press = digital_read(MOTOR_1);
  bool m_2_press = digital_read(MOTOR_2);
  bool m_3_press = digital_read(MOTOR_3);
  bool m_4_press = digital_read(MOTOR_4);
  bool m_5_press = digital_read(MOTOR_5);
  bool m_6_press = digital_read(MOTOR_6);
  bool m_7_press = digital_read(MOTOR_7);

  if(m_1_press)
  {
    //do first set of tests   
  }

  if(m_2_press)
  {
    //do second set of tests
  }

  if(m_3_press)
  {
    //do third set of tests
  }

  if(m_4_press)
  {
    //Toggle Laser
    if(laser_on)
    {
      digitalWrite(LASER_EN, HIGH);
    }
    else
    {
      digitalWrite(LASER_EN, LOW);
    }
    laser_on = !laser_on;
  }

  if(m_5_press)
  {
    //Toggle Solenoid
    if(soleniod_on)
    {
      digitalWrite(SOLENIOD_EN, HIGH);
    }
    else
    {
      digitalWrite(SOLENIOD_EN, LOW);
    }
    soleniod_on = !soleniod_on;
  }
  
  if(m_6_press)
  {
    //do 6th set of tests
  }

  if(m_7_press)
  {
    //do 7th set of tests
  }
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


