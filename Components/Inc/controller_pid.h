#ifndef CONTROLLER_PID_H_
#define CONTROLLER_PID_H_

#include "controller_base.h"
#include "meas_provider.h"

typedef enum {
	PID_PLANT_POSITION,
	PID_PLANT_VELOCITY,
} pid_mode_t;

typedef struct {
	/** @brief Proportional, integral, derivative gains. */
	float Kp, Ki, Kd;
	/** @brief Absolute output limit (applied to controller output). */
	float out_limit;
	/** @brief Absolute limit for the integral term to avoid windup. */
	float integral_limit;
} PIDParams;

typedef struct {
	/** @brief PID parameters (gains, limits). */
	PIDParams params;
	/** @brief Control timestep in seconds (Ts). */
	float Ts;
	/** @brief PID operating mode (position vs velocity plant). */
	pid_mode_t mode;
    
	/** @brief Last set reference values. */
	ctrl_ref_t  ref;
	/** @brief Last ingested measurements. */
	ctrl_meas_t meas;
    
	/** @brief Internal integral accumulator. */
	float integral;
	/** @brief Previous error value used for derivative term. */
	float err_prev;
	/** @brief Last computed output (float). */
	float out; 
	/** @brief Whether the PID controller is enabled. */
	bool  enabled;
	
	uint8_t reset_integral;
    
	/** @brief Optional measurement provider (pull model). */
	MeasProvider *provider;
	/** @brief If non-zero prefer pulling measurements from provider in step(). */
	uint8_t prefer_pull;
} PID;

/** @brief Initialize a PID structure with given parameters.
 *
 * This zeroes internal state and sets gains, timestep and limits.
 */
void pid_init(
	PID *p,
	PIDParams params,
	float Ts, pid_mode_t mode
);

/** @brief Create a Controller wrapper around a PID context.
 *
 * Returns a Controller value whose vtable points to the PID implementation
 * and whose ctx points to the provided PID struct.
 */
Controller make_pid(PID *pid);

/** @brief Bind a measurement provider to the PID instance.
 *
 * If `prefer_pull` is non-zero, the PID step function will attempt to pull
 * samples from the provider before computing the control law.
 */
void pid_bind_provider(PID *p, MeasProvider *prov, uint8_t prefer_pull);
    
/** @brief PID controller vtable instance (defined in controller_pid.c). */
extern const ControllerVTable PID_VT;
	
static inline void pid_set_params(PID *p, const PIDParams params, int reset_integral) {
	if (!p) return ;
	p->params = params;
	if (reset_integral) p->reset_integral = 1;
}

#endif
