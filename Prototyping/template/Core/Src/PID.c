#include "PID.h"
#include <math.h>



// PID struct for storing gains, integrals, and previous error



void PID_init(float kp, float ki, float kd, float target, float toleratnce, PID pid) {
	pid.Kp = kp;
	pid.Ki = ki;
	pid.Kd = kd;
	pid.target = target;
	pid.error = 0.0;
	pid.dt = 0.0;
	pid.integral = 0.0;
	pid.prevError = 0.0;
	pid.output = 0.0;
	pid.tolerance = 0.0;
	pid.prevTime = 0.0;
}


float PID_Update(float measurement, PID pid, float setpoint) {
	pid.dt = HAL_GetTick() - pid.prevTime;
	pid.target = setpoint;
	pid.error = (fabsf(setpoint - measurement) < pid.tolerance) ? 0 : (setpoint - measurement);


	float pOut = pid.Kp * pid.error;
	pid.integral += pid.error * pid.dt;
	float iOut = pid.Ki * pid.integral;
	float derivative = (pid.error - pid.prevError) / pid.dt;
	float dOut = pid.Kd * derivative;
	pid.prevError = pid.error;
	pid.prevTime = HAL_GetTick();
	return pOut + iOut + dOut;


}
