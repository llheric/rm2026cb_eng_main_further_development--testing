#ifndef CONTROLLER_FACADE_H_
#define CONTROLLER_FACADE_H_

#include "controller_pid.h"
#include "controller_pvi.h"
#include "motor_provider.h"
#include "imu_provider.h"
#include "control_runtime.h"

typedef struct {
	DJIMotor *motor;
	float v_scale;
	float x_scale;
	uint8_t feed_v;
	uint8_t feed_x;
	
	float Kp, Ki, Kd;
	float Ts;
	float out_limit;
	float integral_limit;
	pid_mode_t mode;
	uint8_t prefer_pull;
	
	volatile int16_t *slot;
	
	PID        *pid_obj;
	Controller *ctrl_obj;
	DJIMotorProvCtx *prov_ctx;
	MeasProvider    *provider;
}PIDDJIMotorBindingCfg;

bool pid_djimotor_bind(PIDDJIMotorBindingCfg *c);

#endif

