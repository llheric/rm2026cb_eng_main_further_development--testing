#include "controller_base.h"

void controller_registry_init(ControllerRegistry *r) {
	/* Initialize the registry: clear count and nullify entries so that
	 * subsequent operations see a known empty state. */
	r->count = 0;
	for (uint8_t i = 0; i < CONTROLLER_MAX; i++) {
		r->list[i] = NULL;
	}
}

bool controller_register(ControllerRegistry *r, Controller *c) {
	/* Add controller pointer to first free slot. Return false if full. */
	if (r->count >= CONTROLLER_MAX) return false;
	r->list[r->count++] = c;
	return true;
}

void controller_step_all(ControllerRegistry *r) {
	/* Iterate all registered controllers and invoke their step function
	 * if they are enabled. Controllers that are NULL or lack a step
	 * implementation are skipped.
	 */
	for (uint8_t i = 0; i < r->count; i++) {
		Controller *c = r->list[i];
		if (c && c->enabled && c->vt && c->vt->step) {
			c->vt->step(c->ctx);
		}
	}
}

void controller_reset_all(ControllerRegistry *r) {
	/* Call reset on every registered controller that provides a reset
	 * implementation. This is typically used when the system enters a safe
	 * state to clear integrators and internal memory.
	 */
	for (uint8_t i = 0; i < r->count; i++) {
		Controller *c = r->list[i];
		if (c && c->vt && c->vt->reset) {
			c->vt->reset(c->ctx);
		}
	}
}
