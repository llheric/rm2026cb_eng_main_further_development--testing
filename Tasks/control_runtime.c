#include "control_runtime.h"
#include "controller.h"
#include "control_utils.h"
#include "board_comm.h"
#include "dm_example.h"

ControlRuntime runtime;

static inline int16_t f2i16_sat(float x) {
	/*
	 * Convert a float to a signed 16-bit integer with saturation. This
	 * guarantees we never overflow the 16-bit signed range when converting
	 * a controller floating-point output into the integer slot used by
	 * motor transmit code.
	 */
	if (x > 32767.0f) return 32767;
	if (x < -32768.0f) return -32768;
	return (int16_t)x;
}

void control_runtime_init(void) {
	ControlRuntime *rt = &runtime;
	
	/* Initialize runtime state: no bindings and an empty controller
	 * registry. The registry keeps track of controllers so they can be
	 * stepped together by controller_step_all().
	 */
	rt->bind_count_int16 = 0;
	rt->bind_count_float = 0;
	controller_registry_init(&rt->reg);
}

bool control_runtime_bind_int16(Controller *c, volatile int16_t *slot, float out_limit) {
	ControlRuntime *rt = &runtime;
	
	/* Validate inputs and capacity first. The runtime requires both a
	 * controller and a destination slot pointer. We also ensure the
	 * binding table isn't full.
	 */
	if (!c || !slot) return false;
	if (rt->bind_count_int16 >= CTRL_BINDING_MAX) return false;

	/* Register the controller with the shared registry. If registration
	 * fails (for example the controller is already registered or memory
	 * constraints), we bail out.
	 */
	if (!controller_register(&rt->reg, c)) return false;
    
	/* Store the binding information. Note: runtime does not copy the
	 * controller object — it stores the pointer, so the caller must
	 * ensure controller lifetime exceeds the runtime usage.
	 */
	CtrlBindingInt16 *b = &rt->bindings_int16[rt->bind_count_int16++];
	b->ctrl = c;
	b->slot = slot;
	b->out_limit = out_limit;
	return true;
}

bool control_runtime_bind_float(Controller *c, volatile float *slot, float out_limit) {
	ControlRuntime *rt = &runtime;
	
	/* Validate inputs and capacity first. The runtime requires both a
	 * controller and a destination slot pointer. We also ensure the
	 * binding table isn't full.
	 */
	if (!c || !slot) return false;
	if (rt->bind_count_float >= CTRL_BINDING_MAX) return false;

	/* Register the controller with the shared registry. If registration
	 * fails (for example the controller is already registered or memory
	 * constraints), we bail out.
	 */
	if (!controller_register(&rt->reg, c)) return false;
    
	/* Store the float binding information. */
	CtrlBindingFloat *b = &rt->bindings_float[rt->bind_count_float++];
	b->ctrl = c;
	b->slot = slot;
	b->out_limit = out_limit;
	return true;
}

void control_runtime_step() {
	ControlRuntime *rt = &runtime;
	
	/* Step all registered controllers. This will typically update internal
	 * controller states (e.g., integrate PID terms) and prepare outputs.
	 */
	controller_step_all(&rt->reg);

	/* Iterate over bindings and write each controller's output to its
	 * bound slot. For safety we check that the controller, its virtual
	 * table and get_output function are present. The float output is
	 * clamped using the binding's out_limit before conversion.
	 */
	for (uint8_t i = 0; i < rt->bind_count_int16; i++) {
		CtrlBindingInt16 *b = &rt->bindings_int16[i];
		Controller  *c = b->ctrl;
		if (!c || !c->vt || !c->vt->get_output) continue;
        
		/* Ask controller for its current floating-point output. The
		 * controller is expected to expose a get_output(ctx) function
		 * that returns the desired control effort in engineering units.
		 */
		float u = c->vt->get_output(c->ctx);

		/* Apply a local safety clamp to prevent excessive outputs. The
		 * runtime enforces [-out_limit, +out_limit] before converting to
		 * an integer slot value.
		 * Note: `clamp()` is an external utility; ensure it's available
		 * in the build (commonly provided in utils or math helpers).
		 */
		u = clamp(u, -b->out_limit, +b->out_limit);

		/* Convert with saturation and write into the target slot. The
		 * slot is typically used by lower-level code that packages motor
		 * commands for transmission. Declaring the slot volatile allows
		 * safe concurrent reads by transmit tasks/ISRs.
		 */
		*(b->slot) = f2i16_sat(u);
	}

	/* Process float bindings: write controller outputs directly as floats
	 * without conversion to int16.
	 */
	for (uint8_t i = 0; i < rt->bind_count_float; i++) {
		CtrlBindingFloat *b = &rt->bindings_float[i];
		Controller  *c = b->ctrl;
		if (!c || !c->vt || !c->vt->get_output) continue;
        
		/* Get floating-point output and apply clamping. */
		float u = c->vt->get_output(c->ctx);
		u = clamp(u, -b->out_limit, +b->out_limit);

		/* Write directly to the float slot. */
		*(b->slot) = u;
	}
}

void control_runtime_flush() {
	/* If the system is not in manual_online (i.e., it's offline or in a
	 * safe state), reset all controllers and zero all bound output
	 * slots. This ensures motors stop when the system is commanded
	 * offline. `manual_online` is an external flag defined elsewhere.
	 */
	//if (!arm_motion_allowed()) {
	if (!arm_motion_allowed()) {
		ControlRuntime *rt = &runtime;
		controller_reset_all(&rt->reg);
		for (uint8_t i = 0; i < rt->bind_count_int16; i++) {
			CtrlBindingInt16 *b = &rt->bindings_int16[i];
			*(b->slot) = 0;
		}
		for (uint8_t i = 0; i < rt->bind_count_float; i++) {
			CtrlBindingFloat *b = &rt->bindings_float[i];
			*(b->slot) = 0.0f;
		}
	}
    
	/* Trigger CAN motor current transmission for both CAN buses. These
	 * functions will read the integer slots and send appropriate CAN
	 * messages. Side effect: initiates communication over the motor bus.
	 */
	DJIMotor_SendAll();
	if (arm_control_ready) DMMotor_SendAllCommands();
	
	/* Performance test logging: every 5 control cycles, log controller
	 * outputs and references for analysis. This block is conditionally
	 * compiled based on PERFORMANCE_TEST_ON flag.
	 */
	#if PERFORMANCE_TEST_ON
	static uint8_t cycle_count = 0;
	
	extern Controller ctrl_yaw, ctrl_pitch;
	extern gimbal_log_data_t gimbal_log_data;
	
	cycle_count ++;
	if (cycle_count < 5) return ;
	
	cycle_count = 0;
	
	ControlRuntime *rt = &runtime;
	for (uint8_t i = 0; i < rt->bind_count_int16; i++) {
		CtrlBindingInt16 *b = &rt->bindings_int16[i];
		Controller  *c = b->ctrl;
		if (c == &ctrl_yaw) {
			gimbal_log_data.ctrl_yaw_out = c->vt->get_output(c->ctx) / 16384.0f;
			PVI *ctx = (PVI*)c->ctx;
			gimbal_log_data.y_pos_ref  = ctx->ref.x_ref;
			gimbal_log_data.y_spd_ref  = ctx->ref.v_ref;
			gimbal_log_data.y_pos_meas = ctx->meas.x_meas;
			gimbal_log_data.y_spd_meas = ctx->meas.v_meas;
		}
		if (c == &ctrl_pitch) {
			gimbal_log_data.ctrl_pitch_out = c->vt->get_output(c->ctx);
			PVI *ctx = (PVI*)c->ctx;
			gimbal_log_data.p_pos_ref  = ctx->ref.x_ref;
			gimbal_log_data.p_spd_ref  = ctx->ref.v_ref;
			gimbal_log_data.p_pos_meas = ctx->meas.x_meas;
			gimbal_log_data.p_spd_meas = ctx->meas.v_meas;
		}
	}
	gimbal_log_data.flag = performance_test_is_on();
	gimbal_log_data.time_stamp = time();
	send_gimbal_log_data(&gimbal_log_data);
	#endif
}

void control_runtime_task(void *args) {
	while (1) {
		/* Perform one control step and flush outputs, then sleep for the
		 * configured control timestep. The delay function blocks the
		 * current task for CTRL_TS_MS milliseconds — typical usage is to
		 * run this function as a dedicated periodic RTOS task.
		 */
		if (arm_control_ready) control_runtime_step();
		control_runtime_flush();
		delay(CTRL_TS_MS);
	}
}

