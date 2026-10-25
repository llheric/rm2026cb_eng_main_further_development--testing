#include "planner_trap.h"
#include "controller_base.h"
static void trap_pos_set_goal(void *ctx, const ctrl_goal_t *goal) {
	TrapPosPlanner *p = (TrapPosPlanner*)ctx;
	if (!p || !goal) return ;

	// Set per-goal limits, defaulting to overall limits
	p->v_goal_max = p->v_max;
	p->a_goal_max = p->a_max;
	
	if (goal->mask & CTRL_GOAL_V_MAX) p->v_goal_max = fabsf(goal->v_max);
	if (goal->mask & CTRL_GOAL_A_MAX) p->a_goal_max = fabsf(goal->a_max);

	if (goal->mask & CTRL_GOAL_X_TARGET) p->x_goal = goal->x_target;

	if (goal->mask & CTRL_GOAL_V_TARGET) p->v_goal = goal->v_target;
	else p->v_goal = 0.0f; // Default to zero if not specified

	p->has_goal = true;
}

static void trap_pos_step(void *ctx) {
	TrapPosPlanner *p = (TrapPosPlanner*)ctx;
	if (!p || !p->has_goal) return ;

	const float Ts = p->Ts;
	const float a_max = p->a_goal_max;
	const float v_max = p->v_goal_max;

	float dx = p->x_goal - p->x_ref;
	float dv = p->v_goal - p->v_ref;
	if (fabsf(dx) < p->pos_tolerance && fabsf(dv) < p->vel_tolerance) {
		p->x_ref = p->x_goal;
		p->v_ref = p->v_goal;
		p->has_goal = false;
		return ;
	}

	/* Determine motion direction based on position error.
	 * When position error is near zero, use velocity error for direction.
	 */
	float dir = (fabsf(dx) > p->pos_tolerance) ? 
		((dx > 0.0f) ? +1.0f : -1.0f) :
		((dv > 0.0f) ? +1.0f : -1.0f);

	/* Try to accelerate in the direction of the goal. */
	float v_candidate = p->v_ref + dir * a_max * Ts;
	v_candidate = clamp(v_candidate, -v_max, v_max);

	/* Calculate stopping distance from current candidate velocity to goal velocity.
	 * Formula: d = (v_candidate² - v_goal²) / (2 * a_max)
	 * This accounts for non-zero target velocities.
	 */
	float v_cand_sq = v_candidate * v_candidate;
	float v_goal_sq = p->v_goal * p->v_goal;
	float dx_stop = a_max > 0.0f ? fabsf(v_cand_sq - v_goal_sq) / (2.0f * a_max) : 0.0f;

	/* If we need to start braking to reach the goal, limit velocity. */
	if (dx_stop > fabsf(dx)) {
		/* Compute the maximum velocity magnitude we can have now to stop at the goal.
		 * Formula: v_brake = sqrt(v_goal² + 2 * a_max * |dx|)
		 */
		float v_brake_sq = v_goal_sq + 2.0f * a_max * fabsf(dx);
		float v_brake = sqrtf(fmaxf(0.0f, v_brake_sq));
		
		/* The brake velocity should have the same sign as the position error
		 * to ensure we're moving toward the goal during braking.
		 */
		v_candidate = dir * v_brake;
		v_candidate = clamp(v_candidate, -v_max, v_max);
	}

	p->v_ref = v_candidate;
	p->x_ref += p->v_ref * Ts;

	dx = p->x_goal - p->x_ref;
	dv = p->v_goal - p->v_ref;
	if (fabsf(dx) < p->pos_tolerance && fabsf(dv) < p->vel_tolerance) {
		p->x_ref = p->x_goal;
		p->v_ref = p->v_goal;
		p->has_goal = false;
	}
}

static void trap_pos_get_ref(void *ctx, ctrl_ref_t *out) {
	TrapPosPlanner *p = (TrapPosPlanner*)ctx;
	if (!out) return ;
	if (!p) { (*out) = (ctrl_ref_t){0}; return ; }

	out->x_ref = p->x_ref;
	out->v_ref = p->v_ref;
	out->mask = CTRL_MASK_X_REF | CTRL_MASK_V_REF;
}

static void trap_pos_reset(void *ctx) {
	TrapPosPlanner *p = (TrapPosPlanner*)ctx;
	if (!p) return ;

	p->v_ref = 0.0f;
	p->has_goal = false;
}

static void trap_pos_enable(void *ctx, bool en) {
	TrapPosPlanner *p = (TrapPosPlanner*)ctx;
	if (!p) return ;
	if (!en) { p->v_ref = 0.0f; p->has_goal = false; }
}

static const CtrlPlannerVTable TRAP_POS_VT = {
	.set_goal  = trap_pos_set_goal,
	.step      = trap_pos_step,
	.get_ref   = trap_pos_get_ref,
	.reset     = trap_pos_reset,
	.enable    = trap_pos_enable,
};

void trap_pos_planner_init(
	TrapPosPlanner *p,
	float v_max, float a_max,
	float Ts
) {
	if (!p) return ;
	memset(p, 0, sizeof(*p));

	p->v_max = fabsf(v_max);
	p->a_max = fabsf(a_max);
	p->Ts    = Ts;

	p->x_ref = 0.0f;
	p->v_ref = 0.0f;

	p->x_goal = 0.0f;
	p->v_goal = 0.0f;
	p->has_goal = false;

	// Default per-goal limits to overall limits
	p->v_goal_max = v_max;
	p->a_goal_max = a_max;

	p->pos_tolerance = 1e-3f;
	p->vel_tolerance = 1e-3f;
}

CtrlPlanner make_trap_pos_planner(TrapPosPlanner *p) {
	CtrlPlanner planner = { .ctx = p, .vt = &TRAP_POS_VT, .enabled = false, };
	return planner;
}
