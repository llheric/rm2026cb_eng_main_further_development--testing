#ifndef MEAS_PROVIDER_H_
#define MEAS_PROVIDER_H_

#include "stdint.h"
#include "stdbool.h"
#include "control_utils.h"
/*
typedef struct {
	float x_meas;
	float v_meas;
	uint32_t mask;
} meas_sample_t;
*/
typedef struct MeasProviderVTable {
	bool (*read)(void *ctx, meas_sample_t *out);
} MeasProviderVTable;

typedef struct MeasProvider {
	const MeasProviderVTable *vt;
	void *ctx;
} MeasProvider;

static inline bool meas_provider_read(MeasProvider *p, meas_sample_t *out) {
	return (p && p->vt && p->vt->read) ? p->vt->read(p->ctx, out) : false;
}

#endif

