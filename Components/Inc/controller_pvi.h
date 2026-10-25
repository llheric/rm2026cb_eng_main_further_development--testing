#ifndef CONTROLLER_PVI_H_
#define CONTROLLER_PVI_H_

#include "controller_base.h"
#include "meas_provider.h"

typedef struct {
	float Kp, Kv, Ki;
	float out_limit;
	float integral_limit;
} PVIParams;

typedef struct {
	PVIParams params;
	float Ts;
	
	ctrl_ref_t  ref;
	ctrl_meas_t meas;
    
	float integral;
	float xerr_prev;
	float out;
	float u_scale;
	bool enabled;
	
	uint8_t reset_integral;
	
	MeasProvider *provider;
	uint8_t prefer_pull;
} PVI;

void pvi_init(
	PVI *p,
	PVIParams params,
	float Ts, float u_scale
);
	
Controller make_pvi(PVI *pvi);
	
void pvi_bind_provider(PVI *p, MeasProvider *prov, uint8_t prefer_pull);
	
static inline void pvi_set_params(PVI *p, const PVIParams params, int reset_integral) {
	if (!p) return ;
	p->params = params;
	if (reset_integral) p->reset_integral = 1;
}

#endif

