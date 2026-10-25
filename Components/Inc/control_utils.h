#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"

#if 0
typedef enum
{
    CTRL_MASK_X_REF = 1u << 0,  // x reference valid
    CTRL_MASK_V_REF = 1u << 1,  // v reference valid
    CTRL_MASK_X_MEAS = 1u << 2, // x measure valid
    CTRL_MASK_V_MEAS = 1u << 3, // v measure valid
} ctrl_mask_t;

typedef struct
{
    /** @brief Position reference (if mask indicates valid). */
    float x_ref;
    /** @brief Velocity reference (if mask indicates valid). */
    float v_ref;
    /** @brief Bitfield describing which fields are valid (ctrl_mask_t). */
    uint32_t mask;
} ctrl_ref_t;

typedef struct
{
    /** @brief Measured position (if mask indicates valid). */
    float x_meas;
    /** @brief Measured velocity (if mask indicates valid). */
    float v_meas;
    /** @brief Bitfield describing which measurement fields are valid. */
    uint32_t mask;
} ctrl_meas_t;
#endif
typedef struct
{
    /** @brief Position and velocity measurement sample. */
    float x_meas;
    /** @brief Velocity measurement sample. */
    float v_meas;
    /** @brief Bitfield describing which measurement fields are valid. */
    uint32_t mask;
} meas_sample_t;

typedef enum
{
    CTRL_GOAL_X_TARGET = 1u << 0, /**< Target position x_target is valid */
    CTRL_GOAL_V_TARGET = 1u << 1, /**< Target velocity v_target is valid */

    CTRL_GOAL_V_MAX = 1u << 2, /**< Per-goal velocity limit v_max is valid */
    CTRL_GOAL_A_MAX = 1u << 3, /**< Per-goal acceleration limit a_max is valid */
    CTRL_GOAL_J_MAX = 1u << 4, /**< Per-goal jerk limit j_max is valid */

    CTRL_GOAL_T_TOTAL = 1u << 5, /**< Total time for the motion profile t_total is valid */
} ctrl_goal_mask_t;

typedef struct
{
    /** @brief Target position (if mask indicates valid). */
    float x_target;
    /** @brief Target velocity (if mask indicates valid). */
    float v_target;

    /** @brief Maximum velocity for this goal (if mask indicates valid). */
    float v_max;
    /** @brief Maximum acceleration for this goal (if mask indicates valid). */
    float a_max;
    /** @brief Maximum jerk for this goal (if mask indicates valid). */
    float j_max;

    /** @brief Total time for the motion profile to reach this goal (if mask indicates valid). */
    float t_total;

    /** @brief Bitfield describing which fields are valid (ctrl_goal_mask_t). */
    uint32_t mask;
} ctrl_goal_t;

// helper functions

/** @brief Clamp a floating-point value to a range [lo, hi]. */
static inline float clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

#endif
