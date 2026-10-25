#ifndef MOTOR_PROVIDER_H_
#define MOTOR_PROVIDER_H_

#include "controller_base.h"
#include "meas_provider.h"
#include "dji_motor.h"
#include "dm_motor.h"

// ================================ DJI Motor Provider ================================

typedef struct {
	volatile DJIMotor *m;
	float v_scale;
	float x_scale;
	uint8_t feed_v : 1;
	uint8_t feed_x : 1;
} DJIMotorProvCtx;

void djimotor_ctx_init(
	DJIMotorProvCtx *ctx,
	DJIMotor *m,
	float v_scale, float x_scale,
	uint8_t feed_v, uint8_t feed_x
);
MeasProvider make_djimotor_provider(DJIMotorProvCtx *ctx);

// ================================ DM Motor Provider ================================

typedef struct {
	volatile DMMotor *m;
	float v_scale;
	float x_scale;
	uint8_t feed_v : 1;
	uint8_t feed_x : 1;
} DMMotorProvCtx;

void dmmotor_ctx_init(
	DMMotorProvCtx *ctx,
	DMMotor *m,
	float v_scale, float x_scale,
	uint8_t feed_v, uint8_t feed_x
);
MeasProvider make_dmmotor_provider(DMMotorProvCtx *ctx);

#endif

