#ifndef PID_H
#define  PID_H

#include "main.h"

typedef struct {

    // add more members if needed
    float Kp;     // proportional gain
    float Ki;     // integral gain
    float Kd;     // derivative gain
    float error;
    float dt;
    float target;
    float integral;
    float prevError;
    float output;    // controller output
    float tolerance;
    uint32_t prevTime;
} PID;
float PID_Update(float measurement, PID pid, float setpoint);
void PID_init(float kp, float ki, float kd, float target, float tolerance, PID pid);




#endif
