#ifndef _FSFC_H
#define _FSFC_H

typedef struct{
	float 		ka; // proportion coefficient, like kp in PID
	float 		kw; // differential, like kd in PID
	float 		ki; // integral, like ki in PID
	float 		filter_k;
	float 		last_err;
  float 		integral;
	float 		integral_limit;
	float 		max_out;
	float 		out;
	float 		last_angleout;
	float			ka_OUT;
	float			ki_OUT;
	float			kw_OUT;
}FSFC;


static inline void FSFC_Init(FSFC* fsfc, float ka, float kw, float ki, float integral_limit, float max_out, float filter_k){
	fsfc->ka = ka;
	fsfc->kw = kw;
	fsfc->ki = ki;
	fsfc->integral_limit = integral_limit;
	fsfc->max_out = max_out;
	fsfc->filter_k = filter_k;
	fsfc->last_err = 0.0f;
	fsfc->out = 0.0f;
	fsfc->last_angleout = 0.0f;
	fsfc->integral = 0.0f;
}

static inline void FSFC_Reset(FSFC* fsfc, float ka, float kw, float ki, float integral_limit, float max_out, float filter_k){
	fsfc->ka = ka;
	fsfc->kw = kw;
	fsfc->ki = ki;
	fsfc->integral_limit = integral_limit;
	fsfc->max_out = max_out;
	fsfc->filter_k = filter_k;
	fsfc->integral = 0;
}


float FSFC_calc(FSFC* fsfc, float angle_get, float angle_set, float w_get, float w_set);
#endif
