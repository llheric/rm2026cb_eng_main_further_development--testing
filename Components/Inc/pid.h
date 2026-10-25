#ifndef _PID_H
#define _PID_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float last_err;
  float integral;
  float integral_limit;
	float integral_threshold;
	float output_deadband;
  float max_out;
	float last_delta_err;
	float filter_k;
	float out;
} PID_Legacy;

static inline void PID_init_legacy(PID_Legacy* pid, float kp, float ki, float kd, float integral_limit, float integral_threshold, float max_out, float output_deadband,float filter_k) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->integral_threshold = integral_threshold;
  pid->max_out = max_out;
  pid->output_deadband = output_deadband;
	pid->last_delta_err = 0;
	pid->last_err = 0;
	pid->filter_k = filter_k;
}

static inline void PID_reset_legacy(PID_Legacy* pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0;
}

float PID_calc_legacy(PID_Legacy *pid, float measure, float target);
float PID_dry_calc_legacy(PID_Legacy* pid, float measure, float target);

#endif
