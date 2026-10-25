#ifndef DM_EXAMPLE_TASK_H
#define DM_EXAMPLE_TASK_H

#include "os.h"
#include "dm_motor.h"
#include "controller_facade.h"

void dm_motor_task(void *args);

// Keil Watch diagnostics. Status array uses motor ID, not mapping index.
extern volatile uint8_t arm_control_ready;
extern volatile uint8_t arm_init_motor_id;
extern volatile uint8_t arm_init_failed_id;
extern volatile uint8_t arm_init_ok_mask;
extern volatile HAL_StatusTypeDef arm_init_status[7];
uint8_t arm_motion_allowed(void);

typedef struct
{
    uint8_t slave_motor_id;
    uint8_t master_data_source;
    float k;
    float b;
} MotorMapping;

typedef enum
{
    STATE_INIT,
    STATE_ONLINE,
    STATE_OFFLINE,
    STATE_ERROR,
	// STATE_AUTO_STORE,
    // STATE_AUTO_TAKE
} SystemState;

typedef struct
{
    float continuous_deg;
    float last_angle;
    int32_t rounds;
} SlaveState;

typedef struct
{
    float kp;
    float ki;
    float kv;
    float alpha;
} MotorPviParams;

typedef struct
{
    float max_vel;
    float max_acc;
    float dt;
} PlannerParams;


typedef struct {
    uint8_t active;
    float soft_max_vel;
    float soft_max_acc;
    float exit_threshold;
} SoftStartState;

#endif
