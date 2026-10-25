/*
 * PID controller implementation
 * -----------------------------
 * This file contains the PID controller logic that adapts the generic
 * Controller interface (ControllerVTable) to a PID algorithm. The .c
 * functions are documented with plain English comments explaining the
 * behavior and key implementation choices.
 */

#include "controller_pid.h"

/* Store the provided reference structure into the PID context. */
static void pid_set_reference(void *ctx, const ctrl_ref_t *ref) {
	PID *p = (PID*)ctx;
	if (!p || !ref) return ;
	p->ref = *ref;
}

/* Copy a measurement sample into the PID's internal measurement buffer. */
static void pid_ingest_measurement(void *ctx, const ctrl_meas_t *meas) {
	PID *p = (PID*)ctx;
	if (!p || !meas) return ;
	p->meas = *meas;
}

/* Reset internal PID state (integrator, previous error, and output). */
static void pid_reset(void *ctx) {
	PID *p = (PID*)ctx;
	if (!p) return ;
	p->integral = 0.0f;
	p->err_prev = 0.0f;
	p->out = 0.0f;
}

/* Enable or disable the PID controller. */
static void pid_enable(void *ctx, bool en) {
	PID *p = (PID*)ctx;
	if (!p) return ;
	p->enabled = en;
}

/* PID main step function.
 * - Optionally pull measurements from a provider when configured.
 * - Compute error depending on configured mode (position or velocity).
 * - Integrate (with anti-windup), compute derivative, and form control output.
 * - Clamp the final output to configured limits.
 */
static void pid_step(void *ctx) {
	PID *p = (PID*)ctx;
	if (!p) return ;
	if (!p->enabled) {
		p->out = 0.0f;
		return ;
	}
    
	/* Pull measurements from provider when in pull mode. */
	meas_sample_t s;
	if (p->provider && p->prefer_pull && meas_provider_read(p->provider, &s)) {
		if (s.mask & CTRL_MASK_V_MEAS) p->meas.v_meas = s.v_meas;
		if (s.mask & CTRL_MASK_X_MEAS) p->meas.x_meas = s.x_meas;
		p->meas.mask = s.mask;
	}
    
	/* Compute error according to mode. */
	float err = 0.0f;
	switch (p->mode) {
		case PID_PLANT_POSITION: {
			const float xr = (p->ref.mask  & CTRL_MASK_X_REF)  ? p->ref.x_ref   : 0.0f;
			const float xm = (p->meas.mask & CTRL_MASK_X_MEAS) ? p->meas.x_meas : 0.0f;
			err = xr - xm;
			break;
		}
		case PID_PLANT_VELOCITY: {
			const float vr = (p->ref.mask  & CTRL_MASK_V_REF)  ? p->ref.v_ref   : 0.0f;
			const float vm = (p->meas.mask & CTRL_MASK_V_MEAS) ? p->meas.v_meas : 0.0f;
			err = vr - vm;
			break;
		}
		default:
			return;
	}
	
	const PIDParams par = p->params;
	
	if (p->reset_integral) {
		p->integral = 0.0f;
		p->err_prev = 0.0f;
		p->reset_integral = 0;
	}
    
	/* Integral update with anti-windup. */
	p->integral += par.Ki * err * p->Ts;
	p->integral = clamp(p->integral, -par.integral_limit, +par.integral_limit);
    
	/* Derivative (finite difference) and compute PID output. */
	const float derr = (err - p->err_prev) / (p->Ts > 0.0f ? p->Ts : 1.0f);
	float u = par.Kp * err + p->integral + par.Kd * derr;
	p->out = clamp(u, -par.out_limit, +par.out_limit);
    
	/* Save error for next derivative computation. */
	p->err_prev = err;
}

/* Return last computed output. Safe for callers to query even if PID is NULL. */
static float pid_get_output(void *ctx) {
	PID *p = (PID*)ctx;
	return p ? p->out : 0.0f;
}

const ControllerVTable PID_VT = {
	.set_reference 			= pid_set_reference,
	.ingest_measurement = pid_ingest_measurement,
	.step 							= pid_step,
	.get_output 				= pid_get_output,
	.reset 							= pid_reset,
	.enable							= pid_enable,
};

void pid_init(
	PID *p,
	PIDParams params,
	float Ts, pid_mode_t mode
) {
	/* Initialize PID configuration and clear runtime state. */
	if (!p) return ;
	memset(p, 0, sizeof(*p));
    
	p->params = params;
	if (p->params.out_limit < 0.0f) p->params.out_limit = 0.0f;
	if (p->params.integral_limit < 0.0f) p->params.integral_limit = 0.0f;
	
	p->Ts = Ts;
	p->mode = mode;
	p->enabled = false;
	p->ref.mask  = 0u;
	p->meas.mask = 0u;
	
	p->provider = NULL;
	p->prefer_pull = 0;
}

Controller make_pid(PID *pid) {
	Controller c = { .vt = &PID_VT, .ctx = pid, .enabled = false };
	return c;
}

void pid_bind_provider(PID *p, MeasProvider *prov, uint8_t prefer_pull) {
	/* Attach an external measurement provider. When `prefer_pull` is set,
	 * pid_step() will attempt to read new samples from the provider.
	 */
	if (!p) return ;
	p->provider = prov;
	p->prefer_pull = prefer_pull;
}
