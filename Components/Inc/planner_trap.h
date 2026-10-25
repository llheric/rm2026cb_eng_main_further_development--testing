#ifndef PLANNER_TRAP_H_
#define PLANNER_TRAP_H_

#include "planner_base.h"

typedef struct
{
    /** @brief Maximum velocity and acceleration for the trapezoidal profile. */
    float v_max, a_max;
    /** @brief Control timestep in seconds. */
    float Ts;

    /** @brief Current position for the trapezoidal profile. */
    float x_ref;
    /** @brief Current velocity for the trapezoidal profile. */
    float v_ref;

    /** @brief Goal position for the trapezoidal profile. */
    float x_goal;
    /** @brief Goal velocity for the trapezoidal profile. */
    float v_goal;
    /** @brief Whether a goal is currently set. */
    bool has_goal;

    /** @brief Per-goal maximum velocity and acceleration (overrides defaults if non-zero). */
    float v_goal_max;
    /** @brief Per-goal maximum acceleration (overrides default if non-zero). */
    float a_goal_max;

    /** @brief Tolerances for reaching the goal. */
    float pos_tolerance;
    /** @brief Velocity tolerance for reaching the goal. */
    float vel_tolerance;
} TrapPosPlanner;

void trap_pos_planner_init(
    TrapPosPlanner *p,
    float v_max, float a_max,
    float Ts);
CtrlPlanner make_trap_pos_planner(TrapPosPlanner *p);

#endif
