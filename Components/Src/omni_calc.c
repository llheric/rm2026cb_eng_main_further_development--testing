#include "omni_calc.h"

extern control_mode_t control_mode;

void omni_calc(float vx, float vy,float vw,float *speed){
	static float rotate_ratio=0;//((_CHASSIS_WHEELBASE+_CHASSIS_WHEELTRACK)/2.0f)/RADIAN_COEF;
	static float wheel_rpm_ratio = 60.0f/(_CHASSIS_PERIMETER*M3508_DECELE_RATIO);

	VAL_LIMIT(vx,-_CHASSIS_MAX_VX_SPEED, _CHASSIS_MAX_VX_SPEED);
	VAL_LIMIT(vy,-_CHASSIS_MAX_VY_SPEED, _CHASSIS_MAX_VY_SPEED);
	VAL_LIMIT(vw,-_CHASSIS_MAX_VR_SPEED, _CHASSIS_MAX_VR_SPEED);

	float wheel_rpm[5];
	wheel_rpm[MOTOR_ID_FORWARD_LEFT]=(vy-vx-vw*rotate_ratio) * wheel_rpm_ratio * -WHEEL_DIR;//FORWARD-LEFT
	wheel_rpm[MOTOR_ID_FORWARD_RIGHT]=(vy+vx-vw*rotate_ratio) * wheel_rpm_ratio * -WHEEL_DIR;//FORWARD-RIGHT
	wheel_rpm[MOTOR_ID_BACKWARD_RIGHT]=(vy-vx+vw*rotate_ratio) * wheel_rpm_ratio * WHEEL_DIR;//BACK-RIGHT
	wheel_rpm[MOTOR_ID_BACKWARD_LEFT]=(vy+vx+vw*rotate_ratio) * wheel_rpm_ratio * WHEEL_DIR;//BACK-LEFT

	for(int i=1;i<=4;++i) speed[i]=wheel_rpm[i];
}


void omni_inverse_calc(float vx, float vy,float vw,float *speed){
	static float rotate_ratio=((_CHASSIS_WHEELBASE+_CHASSIS_WHEELTRACK)/2.0f)/RADIAN_COEF;
	static float wheel_rpm_ratio = 60.0f/(_CHASSIS_PERIMETER*M3508_DECELE_RATIO);


	vx = 0.5 * (speed[MOTOR_ID_FORWARD_RIGHT] - speed[MOTOR_ID_FORWARD_LEFT]) / (wheel_rpm_ratio * -WHEEL_DIR);
	vy = 0.5 * (speed[MOTOR_ID_FORWARD_LEFT] - speed[MOTOR_ID_BACKWARD_LEFT]) / (wheel_rpm_ratio * -WHEEL_DIR);
	vw = 0.5 * (- speed[MOTOR_ID_FORWARD_RIGHT] - speed[MOTOR_ID_BACKWARD_LEFT]) / (wheel_rpm_ratio * -WHEEL_DIR);
}