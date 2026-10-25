#include "motor_provider.h"

// ================================ DJI Motor Provider ================================

static bool djimotor_read(void *ctx, meas_sample_t *out) {
	DJIMotorProvCtx *c = (DJIMotorProvCtx*)ctx;
	if (!c || !c->m || !out) return false;
	
	float v = (float)c->m->speed;
	float x = (float)c->m->totalAngle;
	out->mask = 0;
	
	if (c->feed_v && c->v_scale != 0.0f) {
		out->v_meas = v * c->v_scale;
		out->mask |= CTRL_MASK_V_MEAS;
	}
	if (c->feed_x && c->x_scale != 0.0f) {
		out->x_meas = x * c->x_scale;
		out->mask |= CTRL_MASK_X_MEAS;
	}
	
	return out->mask != 0;
}

static const MeasProviderVTable DJIMOTOR_PROV_VT = {
    .read = djimotor_read,
};

void djimotor_ctx_init(
	DJIMotorProvCtx *ctx,
	DJIMotor *m,
	float v_scale, float x_scale,
	uint8_t feed_v, uint8_t feed_x
) {
	(*ctx) = (DJIMotorProvCtx){
		.m = m, .v_scale = v_scale, .x_scale = x_scale,
		.feed_v = feed_v, .feed_x = feed_x
	};
}

MeasProvider make_djimotor_provider(DJIMotorProvCtx *ctx) {
	MeasProvider p = { .vt = &DJIMOTOR_PROV_VT, .ctx = ctx };
	return p;
}


// ================================ DM Motor Provider ================================

static bool dmmotor_read(void *ctx, meas_sample_t *out) {
	DMMotorProvCtx *c = (DMMotorProvCtx*)ctx;
	if (!c || !c->m || !out) return false;
	
	float v = c->m->status.vel;
	float x = c->m->status.totalpos;
	out->mask = 0;
	
	if (c->feed_v && c->v_scale != 0.0f) {
		out->v_meas = v * c->v_scale;
		out->mask |= CTRL_MASK_V_MEAS;
	}
	if (c->feed_x && c->x_scale != 0.0f) {
		out->x_meas = x * c->x_scale;
		out->mask |= CTRL_MASK_X_MEAS;
	}
	
	return out->mask != 0;
}

static const MeasProviderVTable DMMOTOR_PROV_VT = {
    .read = dmmotor_read,
};

void dmmotor_ctx_init(
	DMMotorProvCtx *ctx,
	DMMotor *m,
	float v_scale, float x_scale,
	uint8_t feed_v, uint8_t feed_x
) {
	(*ctx) = (DMMotorProvCtx){
		.m = m, .v_scale = v_scale, .x_scale = x_scale,
		.feed_v = feed_v, .feed_x = feed_x
	};
}

MeasProvider make_dmmotor_provider(DMMotorProvCtx *ctx) {
	MeasProvider p = { .vt = &DMMOTOR_PROV_VT, .ctx = ctx };
	return p;
}
