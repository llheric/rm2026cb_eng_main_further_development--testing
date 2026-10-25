#ifndef IMU_PROVIDER_H_
#define IMU_PROVIDER_H_

#include "controller_base.h"
#include "meas_provider.h"
#include "imu.h"

typedef enum {
	IMU_AXIS_YAW,
	IMU_AXIS_PITCH,
	IMU_AXIS_ROLL,
} imu_axis_t;

typedef struct {
	imu_axis_t axis;
	float x_scale;
	float v_scale;
} IMUProvCtx;

void imu_ctx_init(
	IMUProvCtx *ctx,
	imu_axis_t axis,
	float x_scale, float v_scale
);
MeasProvider make_imu_provider(IMUProvCtx *ctx);
	
static inline void get_std_angle(float *yaw, float *pitch, float *roll, float *yaw_spd, float *pitch_spd, float *roll_spd){
	*yaw   = -attitude.totalYaw;
	*pitch =  attitude.pitch;
	*roll  =  attitude.roll;
	
	*yaw_spd   = -attitude.gyroYaw;
	*pitch_spd =  attitude.gyroPitch;
	*roll_spd  =  attitude.gyroRoll;
}


#endif

