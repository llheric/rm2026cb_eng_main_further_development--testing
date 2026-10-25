#include "FSFC.h"
#include <math.h>
static inline void absLimit(float *in, float max){
	if(*in > max){
		*in = max;
	}
	else if(*in < -max){
		*in = -max;
	}
}

/**
* @breif   a more convenient substitute for double-loop PID control
* @param   fsfc:		container of necessary coefficient
*					 w_get:		speed_get
*					 w_set:		speed_set
* @return  out:		output
* @note	   
*/
float FSFC_calc(FSFC* fsfc, float angle_get, float angle_set, float w_get, float w_set){
	float a_err = angle_set - angle_get;
	float w_err = w_set - w_get;
	float k = fsfc->filter_k;
	fsfc->integral += a_err;
	absLimit(&fsfc->integral, fsfc->integral_limit);
	
	fsfc->ki_OUT = fsfc->integral * fsfc->ki;
	fsfc->ka_OUT = a_err * fsfc->ka;
	fsfc->kw_OUT = w_err * fsfc->kw;
	
	fsfc->ka_OUT = k * fsfc->ka_OUT + (1 - k) * fsfc->last_angleout;
	
	fsfc->last_err = a_err;
	float out = fsfc->ki_OUT + fsfc->ka_OUT + fsfc->kw_OUT;

	absLimit(&out, fsfc->max_out);
	
	fsfc->out = out;
	fsfc->last_angleout = fsfc->ka_OUT;
	return out;
}
