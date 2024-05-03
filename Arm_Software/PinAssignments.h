#ifndef PINASSIGNMENTS_H
#define PINASSIGNMENTS_H

#include <Wire.h>


// Motor Pins
#define M1_FWD          0
#define M1_RVS          1
#define M2_FWD          7
#define M2_RVS          8

#define M3_FWD          28
#define M3_RVS          29
#define M4_FWD          3
#define M4_RVS          4

#define M5_FWD          13
#define M5_RVS          14
#define M6_FWD          15
#define M6_RVS          5 // mag wire

#define M7_FWD          33 // mag wire
#define M7_RVS          22
#define M8_FWD          2
#define M8_RVS          6
#define M9_FWD          9
#define M9_RVS          10
#define M10_FWD         12
#define M10_RVS         11


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
#define IOX2_LAS        2
#define IOX2_SOLENOID   3
#define IOX2_DIR_SW     4


// Encoders
#define ENC_1A          41
#define ENC_1B          40
#define ENC_2A          39
#define ENC_2B          38

#define ENC_3A          27
#define ENC_3B          26
#define ENC_4A          25
#define ENC_4B          24

#define ENC_5A          30
#define ENC_5B          31
#define ENC_6A          32
#define ENC_6B          23

#define ENC_7A          34
#define ENC_7B          35


// Buttons
#define B_ENC_0         20
#define B_ENC_1         21
#define B_ENC_2         37
#define B_ENC_3         36

#define BTN_1              11 // mag wire
#define BTN_2              1
#define BTN_3              2
#define BTN_4              3
#define BTN_5              4
#define BTN_6              5
#define BTN_7              6
#define BTN_8              7
#define BTN_9              8
#define BTN_10             9
#define BTN_11             10



#endif