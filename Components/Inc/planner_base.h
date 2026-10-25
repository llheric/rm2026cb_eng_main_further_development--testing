#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include "control_utils.h"
#include "controller_base.h"
typedef struct CtrlPlannerVTable
{
    /** @brief Set a new goal for the planner to reach. */
    void (*set_goal)(void *ctx, const ctrl_goal_t *goal);
    /** @brief Advance the planner state by one step. */
    void (*step)(void *ctx);
    /** @brief Get the current reference (position/velocity) from the planner. */
    void (*get_ref)(void *ctx, ctrl_ref_t *out);

    /** @brief Reset the planner state. */
    void (*reset)(void *ctx);
    /** @brief Enable or disable the planner. */
    void (*enable)(void *ctx, bool en);
} CtrlPlannerVTable;

typedef struct CtrlPlanner
{
    /** @brief Pointer to the planner virtual table (function pointers). */
    const CtrlPlannerVTable *vt;
    /** @brief Opaque pointer to planner-specific context/state. */
    void *ctx;
    /** @brief Whether the planner is enabled and should be stepped. */
    bool enabled;
} CtrlPlanner;

/** @brief Set a new goal for the planner to reach. */
static inline void planner_enable(CtrlPlanner *p, bool en)
{
    if (!p)
        return;
    p->enabled = en;
    if (p->vt && p->vt->enable)
        p->vt->enable(p->ctx, en);
}

/** @brief Advance the planner state by one step. */
static inline void planner_step(CtrlPlanner *p)
{
    if (!p || !p->enabled || !p->vt || !p->vt->step)
        return;
    p->vt->step(p->ctx);
}

/** @brief Set a new goal for the planner to reach. */
static inline void planner_set_goal(CtrlPlanner *p, const ctrl_goal_t *goal)
{
    if (!p || !p->vt || !p->vt->set_goal || !goal)
        return;
    p->vt->set_goal(p->ctx, goal);
}

#endif
