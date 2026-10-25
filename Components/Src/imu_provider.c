#include "imu_provider.h"

static bool imu_read(void *ctx, meas_sample_t *out) {
	IMUProvCtx *c = (IMUProvCtx*)ctx;
	if (!c || !out) return false;
	
	float yaw, pitch, roll, dyaw, dpitch, droll;
	get_std_angle(&yaw, &pitch, &roll, &dyaw, &dpitch, &droll);
	
	float x = 0.0f, v = 0.0f;
	switch (c->axis) {
		case IMU_AXIS_YAW:   x = yaw;   v = dyaw;   break;
		case IMU_AXIS_PITCH: x = pitch; v = dpitch; break;
		case IMU_AXIS_ROLL:  x = roll;  v = droll;  break;
		default: return false;
	}
	
	out->x_meas = x * c->x_scale;
	out->v_meas = v * c->v_scale;
	out->mask   = CTRL_MASK_X_MEAS | CTRL_MASK_V_MEAS;
	return true;
}

static const MeasProviderVTable IMU_PROV_VT = {
	.read = imu_read,
};

void imu_ctx_init(
	IMUProvCtx *ctx,
	imu_axis_t axis,
	float x_scale, float v_scale
) {
	(*ctx) = (IMUProvCtx){
		.axis = axis,
		.x_scale = x_scale,
		.v_scale = v_scale,
	};
}

MeasProvider make_imu_provider(IMUProvCtx *ctx) {
	MeasProvider p = { .vt = &IMU_PROV_VT, .ctx = ctx };
	return p;
}
