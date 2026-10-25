#ifndef CONTROLLER_BASE_H_
#define CONTROLLER_BASE_H_

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#define CONTROLLER_MAX 16

typedef enum {
	CTRL_MASK_X_REF  = 1u << 0, // x reference valid
	CTRL_MASK_V_REF  = 1u << 1, // v reference valid
	CTRL_MASK_X_MEAS = 1u << 2, // x measure valid
	CTRL_MASK_V_MEAS = 1u << 3, // v measure valid
} ctrl_mask_t;

typedef struct {
	/** @brief Position reference (if mask indicates valid). */
	float x_ref;
	/** @brief Velocity reference (if mask indicates valid). */
	float v_ref;
	/** @brief Bitfield describing which fields are valid (ctrl_mask_t). */
	uint32_t mask;
} ctrl_ref_t;

typedef struct {
	/** @brief Measured position (if mask indicates valid). */
	float x_meas;
	/** @brief Measured velocity (if mask indicates valid). */
	float v_meas;
	/** @brief Bitfield describing which measurement fields are valid. */
	uint32_t mask;
} ctrl_meas_t;


typedef struct ControllerVTable {
	/** @brief Set reference values (position/velocity) for the controller. */
	void (*set_reference)(void *ctx, const ctrl_ref_t *ref);
	/** @brief Provide measurement samples (position/velocity) to the controller. */
	void (*ingest_measurement)(void *ctx, const ctrl_meas_t *meas);
	/** @brief Advance the controller state by one step (called each control tick). */
	void (*step)(void *ctx);
	/** @brief Query the controller's current floating-point output. */
	float (*get_output)(void *ctx);

	/** @brief Reset internal controller state (integrators, filters, etc.). */
	void (*reset)(void *ctx);
	/** @brief Enable or disable the controller; implementation may start/stop internal logic. */
	void (*enable)(void *ctx, bool en);
} ControllerVTable;

typedef struct Controller{
	/** @brief Pointer to the controller virtual table (function pointers). */
	const ControllerVTable *vt;
	/** @brief Opaque pointer to controller-specific context/state. */
	void *ctx;
	/** @brief Whether the controller is enabled and should be stepped. */
	bool enabled;
} Controller;


typedef struct {
	/** @brief Fixed-size array of registered controller pointers. */
	Controller *list[CONTROLLER_MAX];
	/** @brief Number of controllers currently registered. */
	uint8_t count;
} ControllerRegistry;

/** @brief Initialize a ControllerRegistry structure.
 *
 * Sets `count` to zero and clears the internal list.
 */
void controller_registry_init(ControllerRegistry *r);

/** @brief Register a controller into the registry.
 *
 * @return true on success, false if the registry is full.
 */
bool controller_register(ControllerRegistry *r, Controller *c);

/** @brief Step all registered controllers that are enabled.
 *
 * This calls the `step` function from each Controller's vtable if present.
 */
void controller_step_all(ControllerRegistry *r);

/** @brief Reset all registered controllers by calling their `reset` vtable.
 */
void controller_reset_all(ControllerRegistry *r);

// helper functions

/** @brief Clamp a floating-point value to a range [lo, hi]. */
/*static inline float clamp(float x, float lo, float hi) {
	return x < lo ? lo : (x > hi ? hi : x);
}
*/
/** @brief Enable or disable a controller and notify its vtable. */
static inline void controller_enable(Controller *c, bool en) {
	if (!c) return;
	c->enabled = en;
	if (c->vt && c->vt->enable) c->vt->enable(c->ctx, en);
}

/** @brief Convenience helper: set only velocity reference on a controller. */
static inline void controller_set_vref(Controller *c, float vref) {
	if (!c) return ;
	ctrl_ref_t r = { .x_ref = 0.0f, .v_ref = vref, .mask = CTRL_MASK_V_REF };
	if (c->vt && c->vt->set_reference) c->vt->set_reference(c->ctx, &r);
}

/** @brief Convenience helper: set only position reference on a controller. */
static inline void controller_set_xref(Controller *c, float xref) {
	if (!c) return ;
	ctrl_ref_t r = { .x_ref = xref, .v_ref = 0.0f, .mask = CTRL_MASK_X_REF };
	if (c->vt && c->vt->set_reference) c->vt->set_reference(c->ctx, &r);
}

/** @brief Convenience helper: set both position and velocity references. */
static inline void controller_set_pvref(Controller *c, float pref, float vref) {
	if (!c) return ;
	ctrl_ref_t r = { .x_ref = pref, .v_ref = vref, .mask = CTRL_MASK_X_REF | CTRL_MASK_V_REF };
	if (c->vt && c->vt->set_reference) c->vt->set_reference(c->ctx, &r);
}



/** @brief Convenience helper: provide only a velocity measurement. */
static inline void controller_ingest_vmeas(Controller *c, float vmeas) {
	if (!c) return ;
	ctrl_meas_t m = { .x_meas = 0.0f, .v_meas = vmeas, .mask = CTRL_MASK_V_MEAS };
	if (c->vt && c->vt->ingest_measurement) c->vt->ingest_measurement(c->ctx, &m);
}

/** @brief Convenience helper: provide only a position measurement. */
static inline void controller_ingest_xmeas(Controller *c, float xmeas) {
	if (!c) return ;
	ctrl_meas_t m = { .x_meas = xmeas, .v_meas = 0.0f, .mask = CTRL_MASK_X_MEAS };
	if (c->vt && c->vt->ingest_measurement) c->vt->ingest_measurement(c->ctx, &m);
}

/** @brief Convenience helper: provide both position and velocity measurements. */
static inline void controller_ingest_pvmeas(Controller *c, float pmeas, float vmeas) {
	if (!c) return ;
	ctrl_meas_t m = { .x_meas = pmeas, .v_meas = vmeas, .mask = CTRL_MASK_X_MEAS | CTRL_MASK_V_MEAS };
	if (c->vt && c->vt->ingest_measurement) c->vt->ingest_measurement(c->ctx, &m);
}

#endif // CONTROLLER_BASE_H_
