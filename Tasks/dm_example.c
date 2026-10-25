#include "dm_example.h"
#include "robot_conf.h"
#include "planner_trap.h"
#include "dm_motor.h"
#include <string.h>
#include "board_comm.h"

#define SLAVE_MOTOR_COUNT 6
const uint8_t slave_motor_ids[SLAVE_MOTOR_COUNT] = {1, 2, 3, 4, 5, 6};
float delta_prev_master[SLAVE_MOTOR_COUNT] = {0};
float delta_prev_target[SLAVE_MOTOR_COUNT] = {0};
uint8_t delta_initialized[SLAVE_MOTOR_COUNT] = {0};
uint8_t grip = 0;

MotorMapping motor_mappings[SLAVE_MOTOR_COUNT] = {
	{1, 1, 0.00529412f, 253.94116f},
	{2, 2, -0.011123658f, 269.2862f},
	{3, 3, 0.0041f, 0.0f},
	{4, 4, 0.0413064f, -173.8927f},
	{5, 5, 0.0457f, 36.058f},
	{6, 6, -0.01035f, 373.950f}
};
 
MotorPviParams motor_pvi_params[SLAVE_MOTOR_COUNT] = {
	{0.18f, 0.000f, 0.08f, 0.39f}, // 1   不用管0.39，没有使用到
	{0.11f, 0.000f, 0.07f, 0.39f}, // 2
	{0.035f, 0.000f, 0.025f, 0.39f}, // 3  
	{7.5f, 0.000f, 8.3f, 0.39f},	 // 4
	{4.0f, 0.000f, 2.3f, 0.39f},	 // 5
	{2.5f, 0.000f, 3.0f, 0.39f}		 // 6
};

PlannerParams planner_params[SLAVE_MOTOR_COUNT] = {
	{720.0f, 1080.0f, 0.005f}, // 电机ID 1
	{720.0f, 1080.0f, 0.005f}, // 电机ID 2
	{720.0f, 1080.0f, 0.005f}, //	电机ID 3
	{120.0f, 346.2f, 0.005f},	// 电机ID 4
	{180.0f, 346.2f, 0.005f},	// 电机ID 5
	{120.0f, 720.0f, 0.005f}	// 电机ID 6
};

const float master_ranges[7] = {
	0,
	2 * 32766.0f,
	0.0f,
	0.0f,
	0.0f,
	344.0f,
	0.0f
};

SoftStartState soft_start_states[SLAVE_MOTOR_COUNT] = {
	{0, 10.0f, 10.0f, 6.00f},
	{0, 10.0f, 10.0f, 6.0f},
	{0, 10.0f, 10.0f, 6.0f},
	{0, 10.0f, 34.0f, 6.0f},
	{0, 10.0f, 34.0f, 6.0f},
	{0, 10.0f, 34.0f, 6.0f}
};

volatile SystemState current_state = STATE_INIT;
volatile uint8_t arm_control_ready = 0;
volatile uint8_t arm_init_motor_id = 0;
volatile uint8_t arm_init_failed_id = 0;
volatile uint8_t arm_init_ok_mask = 0;
volatile HAL_StatusTypeDef arm_init_status[7];

static uint8_t motor_selected(int idx)
{
    return (ARM_ACTIVE_MOTOR_MASK & (1u << idx)) != 0;
}

uint8_t arm_motion_allowed(void)
{
    return arm_control_ready && control_cmd_trans.control_protection;
}

TrapPosPlanner follow_planners[SLAVE_MOTOR_COUNT];
CtrlPlanner follow_ctrl_planners[SLAVE_MOTOR_COUNT];
ctrl_goal_t follow_goals[SLAVE_MOTOR_COUNT];
ctrl_ref_t follow_refs[SLAVE_MOTOR_COUNT];
PVI follow_dms[SLAVE_MOTOR_COUNT];
PVIParams follow_params[SLAVE_MOTOR_COUNT];
Controller ctrl_dms[SLAVE_MOTOR_COUNT];
MeasProvider dm_provs[SLAVE_MOTOR_COUNT];
DMMotorProvCtx follow_ctxs[SLAVE_MOTOR_COUNT];
extern user_defined_data_t custom_controller;
float uniformed_V_Ref[SLAVE_MOTOR_COUNT];
SlaveState slave_states[SLAVE_MOTOR_COUNT];


// 判断自定义控制器是否有效
uint8_t is_custom_controller_valid(void)
{
	if (custom_controller.J1 != 0 &&
		custom_controller.J2 != 0 &&
		custom_controller.J3 != 0 &&
		custom_controller.J4 != 0 &&
		custom_controller.J5 != 0 &&
		custom_controller.J6 != 0)
	{
		return 1;
	}
	return 0;
}

// 将角度归一化到 [0, 360)
float normalize_angle_0_360(float angle)
{
	while (angle < 0.0f)
		angle += 360.0f;
	while (angle >= 360.0f)
		angle -= 360.0f;
	return angle;
}

float convert_to_continuous_deg(float current_angle, float last_angle, int32_t *rounds)
{
	float diff = current_angle - last_angle;
	if (diff < -300.0f)
		(*rounds)++;
	else if (diff > 300.0f)
		(*rounds)--;
	return (*rounds) * 360.0f + current_angle;
}


// 获取自定义控制器角度
float get_master_angle(uint8_t mapping_idx)
{
	float raw_angle = 0.0f;
	uint8_t src = motor_mappings[mapping_idx].master_data_source;  

	// 自定义控制器无效时返回 0（不运动）	
	if (!is_custom_controller_valid())
		return 0.0f;

	switch (src)
	{
	case 1: raw_angle = custom_controller.J4; break;
	case 2: raw_angle = custom_controller.J5; break;
	case 3: raw_angle = custom_controller.J6; break;
	case 4: raw_angle = custom_controller.J2; break;
	case 5: raw_angle = custom_controller.J1; break;
	case 6: raw_angle = custom_controller.J3; break;
  default: return 0.0f;
	
	}

	// 为每个数据源维护状态
	static float last_raw[7] = {0};
	static float offset[7] = {0};
	static uint8_t initialized[7] = {0};

	if (!initialized[src])
	{
		last_raw[src] = raw_angle;
		offset[src] = 0.0f;
		initialized[src] = 1;
		return raw_angle;
	}

	float range = master_ranges[src];
	float half_range = range / 2.0f;
	float diff = raw_angle - last_raw[src];

	if (diff > half_range)
	{
		offset[src] -= range;
	}
	else if (diff < -half_range)
	{
		offset[src] += range;
	}

	last_raw[src] = raw_angle;
	return raw_angle + offset[src];
}

float map_master_to_slave_angle(float master_angle, uint8_t mapping_idx)
{
	return motor_mappings[mapping_idx].k * master_angle + motor_mappings[mapping_idx].b;
}


// PVI 跟随初始化
bool motor_follow_init_single(int idx)
{
	uint8_t slave_id = slave_motor_ids[idx];
	float raw_angle = DMMotors_CAN1[slave_id].status.pos;
	float angle_to_use = raw_angle;

	slave_states[idx].rounds = 0;
	slave_states[idx].last_angle = angle_to_use;
	slave_states[idx].continuous_deg = angle_to_use;

	dmmotor_ctx_init(&follow_ctxs[idx], &DMMotors_CAN1[slave_id], 1, 1, 1, 1);
	dm_provs[idx] = make_dmmotor_provider(&follow_ctxs[idx]);

	follow_params[idx] = (PVIParams){
		.Kp = motor_pvi_params[idx].kp,
		.Ki = motor_pvi_params[idx].ki,
		.Kv = motor_pvi_params[idx].kv,
		.out_limit = DMMotors_CAN1[slave_id].info.T_max,
		.integral_limit = DMMotors_CAN1[slave_id].info.T_max / 2,
	}; 

	pvi_init(&follow_dms[idx], follow_params[idx], 0.005f, 1);
	pvi_bind_provider(&follow_dms[idx], &dm_provs[idx], 1);
	ctrl_dms[idx] = make_pvi(&follow_dms[idx]);

	if (!control_runtime_bind_float(&ctrl_dms[idx],
        &DMMotors_CAN1[slave_id].command.target_tor,
        DMMotors_CAN1[slave_id].info.T_max)) return false;
	controller_set_pvref(&ctrl_dms[idx], DMMotors_CAN1[slave_id].status.totalpos, 0.0f);
	controller_enable(&ctrl_dms[idx], true);
	return true;
}


// 规划跟随初始化
void planner_follow_init_single(int idx)
{
	uint8_t slave_id = slave_motor_ids[idx];

	trap_pos_planner_init(&follow_planners[idx],
						  planner_params[idx].max_vel,
						  planner_params[idx].max_acc,
						  planner_params[idx].dt);

	slave_states[idx].continuous_deg = convert_to_continuous_deg(
		DMMotors_CAN1[slave_id].status.pos,
		slave_states[idx].last_angle,
		&slave_states[idx].rounds);
	slave_states[idx].last_angle = DMMotors_CAN1[slave_id].status.pos;

	follow_planners[idx].x_ref = slave_states[idx].continuous_deg;
	follow_planners[idx].v_ref = DMMotors_CAN1[slave_id].status.vel * 57.3f;

	follow_ctrl_planners[idx] = make_trap_pos_planner(&follow_planners[idx]);
	planner_enable(&follow_ctrl_planners[idx], true);

	float master_angle = get_master_angle(idx);
	float slave_target_angle;

	if (slave_id == 3)
	{
		// 增量控制：初始目标设为当前位置
		slave_target_angle = slave_states[idx].continuous_deg;
		delta_prev_master[idx] = master_angle;
		delta_prev_target[idx] = slave_target_angle;
		delta_initialized[idx] = 1;
	}
	else
	{
		// 绝对控制
		slave_target_angle = map_master_to_slave_angle(master_angle, idx);
	}

	follow_goals[idx] = (ctrl_goal_t){
		.mask = CTRL_GOAL_X_TARGET | CTRL_GOAL_V_TARGET,
		.x_target = slave_target_angle,
		.v_target = 0,
	};
	planner_set_goal(&follow_ctrl_planners[idx], &follow_goals[idx]);

	soft_start_states[idx].active = 0;
}


// 全部电机跟随初始化
bool motor_follow_init_all(void)
{
    for (int i = 0; i < SLAVE_MOTOR_COUNT; ++i) {
        if (!motor_selected(i)) continue;
        arm_init_motor_id = slave_motor_ids[i];
        if (!motor_follow_init_single(i)) {
            arm_init_status[slave_motor_ids[i]] = HAL_ERROR;
            arm_init_failed_id = slave_motor_ids[i];
            return false;
        }
        planner_follow_init_single(i);
    }
    return true;
}


// 单电机跟随更新
void motor_follow_update_single(int idx)
{
	uint8_t slave_id = slave_motor_ids[idx];
	float raw_angle = DMMotors_CAN1[slave_id].status.pos;
	float angle_to_use = raw_angle;

	slave_states[idx].continuous_deg = convert_to_continuous_deg(
		angle_to_use,
		slave_states[idx].last_angle,
		&slave_states[idx].rounds);
	slave_states[idx].last_angle = angle_to_use;

	float master_angle = get_master_angle(idx);
	float slave_target_angle;

	if (slave_id == 3)
	{
		if (!delta_initialized[idx])
		{
			// 离线后首次上线，重新初始化基准
			delta_prev_master[idx] = master_angle;
			delta_prev_target[idx] = slave_states[idx].continuous_deg;
			delta_initialized[idx] = 1;
			slave_target_angle = slave_states[idx].continuous_deg;
		}
		else
		{
			float delta = master_angle - delta_prev_master[idx];
			slave_target_angle = delta_prev_target[idx] + motor_mappings[idx].k * delta;
			delta_prev_master[idx] = master_angle;
			delta_prev_target[idx] = slave_target_angle;
		}
	}
	else
	{
		slave_target_angle = map_master_to_slave_angle(master_angle, idx);
	}

	follow_goals[idx] = (ctrl_goal_t){
		.mask = CTRL_GOAL_X_TARGET | CTRL_GOAL_V_TARGET,
		.x_target = slave_target_angle,
		.v_target = 0,
	};

	SoftStartState *ss = &soft_start_states[idx];
	float error = fabsf(slave_target_angle - slave_states[idx].continuous_deg);

	if (ss->active && error <= ss->exit_threshold)
	{
		ss->active = 0;
	}

	if (ss->active)
	{
		follow_goals[idx].mask |= CTRL_GOAL_V_MAX | CTRL_GOAL_A_MAX;
		follow_goals[idx].v_max = ss->soft_max_vel;
		follow_goals[idx].a_max = ss->soft_max_acc;
	}

	planner_set_goal(&follow_ctrl_planners[idx], &follow_goals[idx]);

	if (follow_planners[idx].has_goal)
	{
		planner_step(&follow_ctrl_planners[idx]);
		follow_ctrl_planners[idx].vt->get_ref(follow_ctrl_planners[idx].ctx, &follow_refs[idx]);
		uniformed_V_Ref[idx] = DMMotors_CAN1[slave_motor_ids[idx]].status.vel * 57.3f;
		controller_set_pvref(&ctrl_dms[idx],
							 follow_refs[idx].x_ref,
							 follow_refs[idx].v_ref / 57.3f);
	}
}


// 全部电机更新
void motor_follow_update_all(void)
{
	for (int i = 0; i < SLAVE_MOTOR_COUNT; i++)
		if (motor_selected(i)) motor_follow_update_single(i);
}



void handle_offline_state(void)
{
	for (int i = 0; i < SLAVE_MOTOR_COUNT; i++)
	{
		if (!motor_selected(i)) continue;
		uint8_t slave_id = slave_motor_ids[i];
		float raw_angle = DMMotors_CAN1[slave_id].status.pos;
		float angle_to_use = raw_angle;
		/*
				// 仅对电机 ID 2 进行归一化修正
				if (slave_id == 2) {
					angle_to_use = normalize_angle_0_360(raw_angle);
				}
		*/

		slave_states[i].continuous_deg = convert_to_continuous_deg(
			angle_to_use,
			slave_states[i].last_angle,
			&slave_states[i].rounds);
		slave_states[i].last_angle =  angle_to_use;

		follow_planners[i].x_ref = slave_states[i].continuous_deg;
		follow_planners[i].v_ref = DMMotors_CAN1[slave_id].status.vel;

		follow_goals[i] = (ctrl_goal_t){
			.mask = CTRL_GOAL_X_TARGET | CTRL_GOAL_V_TARGET,
			.x_target = slave_states[i].continuous_deg,
			.v_target = 0,
		};
		planner_set_goal(&follow_ctrl_planners[i], &follow_goals[i]);

		controller_set_pvref(&ctrl_dms[i], slave_states[i].continuous_deg, 0.0f);

		if (slave_id == 3)
		{
			delta_initialized[i] = 0;
		}
	}
}


void stop_all_motors(void)
{
	for (int i = 0; i < SLAVE_MOTOR_COUNT; i++)
	{
		uint8_t slave_id = slave_motor_ids[i];
		DMMotor_SetTorque(&DMMotors_CAN1[slave_id], 0);
	}
}


/* Initialization is attempted once after reset. Failure stays latched. */
static void arm_init_abort(void)
{
    arm_control_ready = 0;
    stop_all_motors();
    for (int i = 0; i < SLAVE_MOTOR_COUNT; ++i) {
        if (motor_selected(i)) DMMotor_Disable(USE_CAN1, slave_motor_ids[i]);
    }
    current_state = STATE_ERROR;
}

static void arm_start_follow(void)
{
    handle_offline_state();
    for (int i = 0; i < SLAVE_MOTOR_COUNT; ++i)
        if (motor_selected(i)) soft_start_states[i].active = 1;
    current_state = STATE_ONLINE;
}

void state_machine_step(void)
{
    switch (current_state) {
    case STATE_INIT:
        arm_control_ready = 0;
        arm_init_ok_mask = 0;
        arm_init_failed_id = 0;
        for (int i = 0; i < 7; ++i) arm_init_status[i] = HAL_BUSY;
        for (int i = 0; i < SLAVE_MOTOR_COUNT; ++i) {
            uint8_t id = slave_motor_ids[i];
            if (!motor_selected(i)) continue;
            arm_init_motor_id = id;
            arm_init_status[id] = DMMotor_Enable(USE_CAN1, id);
            if (arm_init_status[id] != HAL_OK) {
                if (!arm_init_failed_id) arm_init_failed_id = id;
                DMMotor_Disable(USE_CAN1, id);
                continue;
            }
            arm_init_ok_mask |= (1u << i);
        }
        if (arm_init_failed_id) {
            arm_init_abort();
            break;
        }
        if (!motor_follow_init_all()) {
            arm_init_abort();
            break;
        }
        // Publish to the sender only after all selected controllers exist.
        for (int i = 0; i < SLAVE_MOTOR_COUNT; ++i) {
            if (!motor_selected(i)) continue;
            uint8_t id = slave_motor_ids[i];
            if (!DMMotor_Register(&DMMotors_CAN1[id])) {
                arm_init_status[id] = HAL_ERROR;
                arm_init_failed_id = id;
                arm_init_abort();
                return;
            }
        }
        arm_init_motor_id = 0;
        arm_control_ready = 1;
        current_state = STATE_OFFLINE;
        if (arm_motion_allowed()) arm_start_follow();
        break;

    case STATE_ONLINE:
        if (arm_motion_allowed()) {
            motor_follow_update_all();
        } else {
            stop_all_motors();
            handle_offline_state();
            current_state = STATE_OFFLINE;
        }
        break;

    case STATE_OFFLINE:
        stop_all_motors();
        if (arm_motion_allowed()) arm_start_follow();
        else if (arm_control_ready) handle_offline_state();
        break;

    case STATE_ERROR:
        stop_all_motors();
        // Do not enter ONLINE after failed initialization, even if enabled.
        break;

    default:
        arm_init_abort();
        break;
    }
}


void dm_motor_task(void *args)
{
	while (1)
	{
		state_machine_step();
		delay(5);
	}
}