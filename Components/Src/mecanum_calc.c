#include "mecanum_calc.h"

void mecanum_calc(float vx, float vy, float vw, float *speed){
  static float rotate_ratio=((_CHASSIS_WHEELBASE+_CHASSIS_WHEELTRACK)/2.0f)/RADIAN_COEF;
  static float wheel_rpm_ratio = 60.0f/(_CHASSIS_PERIMETER*M3508_DECELE_RATIO );
  
	#if SELF_MADE_MEC
	VAL_LIMIT(vx, -_CHASSIS_MAX_VX_SPEED * chassis.spd_ratio, _CHASSIS_MAX_VX_SPEED * chassis.spd_ratio);  //mm/s
  VAL_LIMIT(vy, -_CHASSIS_MAX_VY_SPEED * chassis.spd_ratio, _CHASSIS_MAX_VY_SPEED * chassis.spd_ratio);  //mm/s
  VAL_LIMIT(vw, -_CHASSIS_MAX_VR_SPEED * chassis.spd_ratio, _CHASSIS_MAX_VR_SPEED * chassis.spd_ratio);  //deg/s
  
	#else
  VAL_LIMIT(vx, -_CHASSIS_MAX_VX_SPEED, _CHASSIS_MAX_VX_SPEED);  //mm/s
  VAL_LIMIT(vy, -_CHASSIS_MAX_VY_SPEED, _CHASSIS_MAX_VY_SPEED);  //mm/s
  VAL_LIMIT(vw, -_CHASSIS_MAX_VR_SPEED, _CHASSIS_MAX_VR_SPEED);  //deg/s
  #endif
	
	
  float wheel_rpm[5];
  //float   max = 0;
  
  wheel_rpm[3] = ( -vx - vy + vw * rotate_ratio) * wheel_rpm_ratio * WHEEL_DIR;   //  back- left   //
  wheel_rpm[4] = ( -vx + vy + vw * rotate_ratio) * wheel_rpm_ratio * WHEEL_DIR;	 // forward- left  
	// these wheels are reversed due to sysmetry
  wheel_rpm[1] = ( vx + vy + vw * rotate_ratio) * wheel_rpm_ratio * WHEEL_DIR;  // forward right    
  wheel_rpm[2] = ( vx - vy + vw * rotate_ratio) * wheel_rpm_ratio * WHEEL_DIR;		// back -right   

  for (int i = 1; i <= 4; i++) speed[i] = wheel_rpm[i];
	//for (int i = 1; i <= 4; i++) speed[i+4] = vw * rotate_ratio* wheel_rpm_ratio * WHEEL_DIR;
}

