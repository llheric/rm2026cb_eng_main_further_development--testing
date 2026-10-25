#include "controller_pvi.h"

static void pvi_set_reference(void *ctx, const ctrl_ref_t *ref) {
	PVI *p = (PVI*)ctx;
	if (!p || !ref) return ;
	p->ref = *ref;
}

static void pvi_ingest_measurement(void *ctx, const ctrl_meas_t *meas) {
	PVI *p = (PVI*)ctx;
	if (!p || !meas) return ;
	p->meas = *meas;
}

static void pvi_reset(void *ctx) {
	PVI *p = (PVI*)ctx;
	if (!p) return ;
	p->integral  = 0.0f;
	p->xerr_prev = 0.0f;
	p->out = 0.0f;
}

static void pvi_enable(void *ctx, bool en) {
	PVI *p = (PVI*)ctx;
	if (!p) return ;
	p->enabled = en;
}

static void pvi_step(void *ctx) {
	PVI *p = (PVI*)ctx;
	if (!p) return ;
	if (!p->enabled) {
		p->out = 0.0f;
		return ;
	}
	
	meas_sample_t s;
	if (p->provider && p->prefer_pull && meas_provider_read(p->provider, &s)) {
		if (s.mask & CTRL_MASK_V_MEAS) p->meas.v_meas = s.v_meas;
		if (s.mask & CTRL_MASK_X_MEAS) p->meas.x_meas = s.x_meas;
		p->meas.mask = s.mask;
	}
    
	float xerr = 0.0f, verr = 0.0f;
	const float xr = (p->ref.mask  & CTRL_MASK_X_REF)  ? p->ref.x_ref   : 0.0f;
	const float xm = (p->meas.mask & CTRL_MASK_X_MEAS) ? p->meas.x_meas : 0.0f;
	const float vr = (p->ref.mask  & CTRL_MASK_V_REF)  ? p->ref.v_ref   : 0.0f;
	const float vm = (p->meas.mask & CTRL_MASK_V_MEAS) ? p->meas.v_meas : 0.0f;
	
	xerr = xr - xm;
	verr = vr - vm;
	
	const PVIParams par = p->params;
	
	if (p->reset_integral) {
		p->integral = 0.0f;
		p->xerr_prev = 0.0f;
		p->reset_integral = 0;
	}
    
	p->integral += par.Ki * xerr * p->Ts;
	p->integral = clamp(p->integral, -par.integral_limit, +par.integral_limit);
    
	float u = par.Kp * xerr + par.Kv * verr + p->integral;
	p->out = clamp(u, -par.out_limit, +par.out_limit);
	
	p->xerr_prev = xerr;
}

static float pvi_get_output(void *ctx) {
	PVI *p = (PVI*)ctx;
	return p ? p->out * p->u_scale : 0.0f;
}

const ControllerVTable PVI_VT = {
	.set_reference 			= pvi_set_reference,
	.ingest_measurement = pvi_ingest_measurement,
	.step 							= pvi_step,
	.get_output 				= pvi_get_output,
	.reset 							= pvi_reset,
	.enable							= pvi_enable,
};

void pvi_init(
	PVI *p,
	PVIParams params,
	float Ts, float u_scale
) {
	if (!p) return ;
	memset(p, 0, sizeof(*p));
    
	p->params = params;
	if (p->params.out_limit < 0.0f) p->params.out_limit = 0.0f;
	if (p->params.integral_limit < 0.0f) p->params.integral_limit = 0.0f;
	
	p->Ts = Ts;
	p->u_scale = u_scale;
	p->enabled = false;
	p->ref.mask  = 0u;
	p->meas.mask = 0u;
	
	p->provider = NULL;
	p->prefer_pull = 0;
}

Controller make_pvi(PVI *pvi) {
	Controller c = { .vt = &PVI_VT, .ctx = pvi, .enabled = false };
	return c;
}

void pvi_bind_provider(PVI *p, MeasProvider *prov, uint8_t prefer_pull) {
	if (!p) return ;
	p->provider = prov;
	p->prefer_pull = prefer_pull;
}
