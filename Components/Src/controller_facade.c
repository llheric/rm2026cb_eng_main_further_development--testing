#include "controller_facade.h"

bool pid_djimotor_bind(PIDDJIMotorBindingCfg *c) {
	if (
		!c || !c->motor ||
		!c->pid_obj ||!c->ctrl_obj ||
		!c->prov_ctx || !c->provider
	) return false;
	
	PIDParams params = (PIDParams){
		.Kp = c->Kp, .Ki = c->Ki, .Kd = c->Kd,
		.out_limit = c->out_limit, .integral_limit = c->integral_limit
	};
	
	pid_init(c->pid_obj, params, c->Ts, c->mode);
	
	djimotor_ctx_init(
		c->prov_ctx, c->motor,
		c->v_scale, c->x_scale, c->feed_v, c->feed_x
	);
	*(c->provider) = make_djimotor_provider(c->prov_ctx);
	
	pid_bind_provider(c->pid_obj, c->provider, c->prefer_pull);
	
	*(c->ctrl_obj) = make_pid(c->pid_obj);
	controller_enable(c->ctrl_obj, true);
	
	control_runtime_bind_int16(c->ctrl_obj, c->slot, c->out_limit);
	
	return true;
}

