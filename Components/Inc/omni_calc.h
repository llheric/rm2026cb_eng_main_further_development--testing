#ifndef _OMNI_CALC
#define _OMNI_CALC

#include "func_lib.h"
#include "robot_conf.h"
#include "os.h"
#include "chassis.h"

void omni_calc(float vx, float vy,float vw,float *speed);
void omni_inverse_calc(float vx, float vy,float vw,float *speed);

#endif

