#ifndef PINASSIGNMENTS_H
#define PINASSIGNMENTS_H

#include <cstdint>


// Motor Pins
const uint8_t FWD_PWM_1 = 9;
const uint8_t RVS_PWM_1 = 8;
const uint8_t FWD_PWM_2 = 7;
const uint8_t RVS_PWM_2 = 6;
const uint8_t FWD_PWM_3 = 0;
const uint8_t RVS_PWM_3 = 1;

const uint8_t FWD_PWM_4 = 2;
const uint8_t RVS_PWM_4 = 3;
const uint8_t FWD_PWM_5 = 23;
const uint8_t RVS_PWM_5 = 22;
const uint8_t FWD_PWM_6 = 24;
const uint8_t RVS_PWM_6 = 25;

const uint8_t FWD_PWM_7 = 37;
const uint8_t RVS_PWM_7 = 36;
const uint8_t FWD_PWM_8 = 28;
const uint8_t RVS_PWM_8 = 29;
const uint8_t FWD_PWM_9 = 5;
const uint8_t RVS_PWM_9 = 4;


// Limit Switch Pins
const uint8_t LIM_1 = 14;
const uint8_t LIM_2 = 13;
const uint8_t LIM_3 = 16;
const uint8_t LIM_4 = 15;
const uint8_t LIM_5 = 18;
const uint8_t LIM_6 = 17;

const uint8_t LIM_7 = 20;
const uint8_t LIM_8 = 19;
const uint8_t LIM_9 = 10;
const uint8_t LIM_10 = 21;
const uint8_t LIM_11 = 12;
const uint8_t LIM_12 = 11;


// Encoder Pins
const uint8_t ENC_1 = 41;
const uint8_t ENC_2 = 40;
const uint8_t ENC_3 = 39;
const uint8_t ENC_4 = 38;
const uint8_t ENC_5 = 35;
const uint8_t ENC_6 = 34;


// Button Pins
const uint8_t B_ENC_0 = 30;
const uint8_t B_ENC_1 = 27;
const uint8_t B_ENC_2 = 31;
const uint8_t B_ENC_3 = 32;

const uint8_t DIR_SW = 33;


// Laser Pin
const uint8_t LAS = 26;


#endif