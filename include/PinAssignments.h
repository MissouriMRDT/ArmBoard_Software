#ifndef PINASSIGNMENTS_H
#define PINASSIGNMENTS_H

//2025 REV 1

#include <Wire.h>

// Switch Pin
#define DIR_SW          33

// Motor Pins

#define M8_FWD          11
#define M8_RVS          12


#define M1_PWM          2
#define M2_PWM          3
#define M3_PWM          4
#define M4_PWM          5
#define M5_PWM          6
#define M6_PWM          7
#define M7_PWM          8
#define M8_PWM          36

#define M1_CS           14
#define M2_CS           15
#define M3_CS           16
#define M4_CS           17
#define M5_CS           23
#define M6_CS           20
#define M7_CS           21
#define M8_CS           22

// Laser and Solenoid
#define LAS             13
#define Solenoid        10

// I/O Expander
#define SDA             18
#define SCL             19
#define IOX_TWI         Wire

#define IOX1_LIM_1      0
#define IOX1_LIM_2      1
#define IOX1_LIM_3      2
#define IOX1_LIM_4      3
#define IOX1_LIM_5      4
#define IOX1_LIM_6      5
#define IOX1_LIM_7      6
#define IOX1_LIM_8      7

#define IOX2_LIM_9      0
#define IOX2_LIM_10     1
#define IOX2_FWD_1      2
#define IOX2_RVS_1      3
#define IOX2_FWD_2      4
#define IOX2_RVS_2      5
#define IOX2_FWD_3      6
#define IOX2_RVS_3      7

#define IOX3_FWD_4      0
#define IOX3_RVS_4      1
#define IOX3_FWD_5      2
#define IOX3_RVS_5      3
#define IOX3_FWD_6      4
#define IOX3_RVS_6      5
#define IOX3_FWD_7      6
#define IOX3_RVS_7      7


// Encoders
#define ENC_1A          41
#define ENC_1B          40
#define ENC_2A          39
#define ENC_2B          38
#define ABS_1           27
#define ABS_2           26
#define ABS_3           25
#define ABS_4           24

// Buttons
#define B_ENC_0         28
#define B_ENC_1         29
#define B_ENC_2         34
#define B_ENC_3         35

#define BTN_1           10 // mag wire
#define BTN_2           1
#define BTN_3           2
#define BTN_4           3
#define BTN_5           4
#define BTN_6           5
#define BTN_7           6
#define BTN_8           7
#define BTN_LAS         8
#define BTN_SOL         9

// Can Transceiver, ask Adam again if this is how we assign pin for CAN rx and tx
#define CANRX           0
#define CANTX           1



#endif