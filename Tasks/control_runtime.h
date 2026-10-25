#ifndef CONTROL_RUNTIME_H_
#define CONTROL_RUNTIME_H_

#include "controller_base.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "os.h"
#include "comm.h"

#define CTRL_TS_MS 2
#define CTRL_TS (CTRL_TS_MS*0.001f)

#define CTRL_BINDING_MAX 8

/** @brief Single binding between a Controller and an integer output slot.
 *
 * The runtime stores an array of these bindings and writes each controller's
 * (saturated) output into the provided `slot` every control step.
 */
typedef struct {
	/** @brief Pointer to the controller instance to be stepped by the runtime.
	 *
	 * The runtime does not take ownership of the controller. The caller must
	 * ensure the controller instance remains valid for the lifetime of the
	 * binding.
	 */
	Controller *ctrl;

	/** @brief Pointer to the integer output slot where the controller output
	 *  will be written.
	 *
	 * This is typically a shared variable tied to a motor current/command
	 * value. Declared volatile because it may be read by other tasks or
	 * interrupt handlers (for example the CAN transmit path).
	 */
	volatile int16_t *slot;

	/** @brief Positive bound applied to the floating-point controller output
	 *  before conversion and writing into `slot`.
	 */
	float out_limit;
} CtrlBindingInt16;

/** @brief Single binding between a Controller and a float output slot.
 *
 * Similar to CtrlBinding but writes the controller output directly as a float
 * without saturation to int16.
 */
typedef struct {
	/** @brief Pointer to the controller instance to be stepped by the runtime. */
	Controller *ctrl;

	/** @brief Pointer to the float output slot where the controller output
	 *  will be written.
	 */
	volatile float *slot;

	/** @brief Positive bound applied to the floating-point controller output
	 *  before writing into `slot`.
	 */
	float out_limit;
} CtrlBindingFloat;

/** @brief Control runtime container.
 *
 * Holds the controller registry and a fixed-size table of bindings.
 */
typedef struct {
	ControllerRegistry reg; /**< Controller registry used for stepping */
	CtrlBindingInt16 bindings_int16[CTRL_BINDING_MAX]; /**< Int16 binding table */
	uint8_t bind_count_int16; /**< Number of active int16 bindings */
	CtrlBindingFloat bindings_float[CTRL_BINDING_MAX]; /**< Float binding table */
	uint8_t bind_count_float; /**< Number of active float bindings */
} ControlRuntime;

/** @brief Initialize the control runtime.
 *
 * Must be called once at system startup before calling other runtime APIs.
 */
void control_runtime_init(void);

/** @brief Bind a Controller instance to an int16_t output slot.
 *
 * This function is specifically for binding controllers whose output should be
 * converted to int16_t with saturation (typical for motor current commands).
 *
 * @param c Pointer to a previously created Controller instance (must be non-null).
 * @param slot Pointer to an int16_t location where the runtime will write the
 *             saturated controller output (must be non-null).
 * @param out_limit Positive float bound used to clamp the controller's
 *                  floating-point output before conversion to int16.
 * @return true if binding and registration succeeded, false on invalid input,
 *         full binding table, or failed registration.
 */
bool control_runtime_bind_int16(Controller *c, volatile int16_t *slot, float out_limit);

/** @brief Bind a Controller instance to a float output slot.
 *
 * This function binds controllers whose output should remain as floating-point
 * values without conversion to int16_t.
 *
 * @param c Pointer to a previously created Controller instance (must be non-null).
 * @param slot Pointer to a float location where the runtime will write the
 *             clamped controller output (must be non-null).
 * @param out_limit Positive float bound used to clamp the controller's
 *                  floating-point output.
 * @return true if binding and registration succeeded, false on invalid input,
 *         full binding table, or failed registration.
 */
bool control_runtime_bind_float(Controller *c, volatile float *slot, float out_limit);

/** @brief Main periodic task entry for the control runtime.
 *
 * This function implements the infinite loop that steps all registered
 * controllers, writes outputs to their bound slots and triggers transmission
 * of motor current commands. It is intended to be run as a FreeRTOS task or
 * similar periodic thread.
 *
 * @param args Task argument (unused).
 */
void control_runtime_task(void *args);

#endif

