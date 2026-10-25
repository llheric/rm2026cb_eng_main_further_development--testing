#include "main_control.h"
#include "performance_test.h"

gimbal_command_t gimbal_cmd;
extern chassis_cmd_trans_t chassis_cmd_trans;

extern observer_t obs;
extern uint32_t auto_receive_time;

keyboard_t keyboard;

button_t fn1_btn;
button_t fn2_btn;
button_t pause_btn;
button_t trigger_btn;

button_t key_w_btn;
button_t key_s_btn;
button_t key_a_btn;
button_t key_d_btn;
button_t key_ctrl_btn;
button_t key_shift_btn;
button_t key_q_btn;
button_t key_e_btn;
button_t key_c_btn;
button_t key_v_btn;
button_t mouse_left_btn;
button_t mouse_right_btn;

DJIMotorConfig yaw_cfg, pitch_cfg;
DJIMotorConfig fric_l_cfg, fric_r_cfg, feeder_cfg;

// internal, information of mode
main_control_mode_t main_control_mode;

// ****************************************** Begin of Init ******************************************//
void main_control_init() {
	main_control_mode.control_mode = OFF_MODE;
	
	main_control_mode.gimbal_mode = GIMBAL_IDLE;
	main_control_mode.chassis_mode = CHASSIS_IDLE;
	main_control_mode.shoot_17mm_mode = SHOOT_17MM_IDLE;
	
	main_control_mode.cha_gim_mode = CHASSIS_GIMBAL_SEPERATE;
	main_control_mode.rotate_mode = ROTATE_OFF;
	main_control_mode.fric_mode = FRIC_OFF;
	main_control_mode.shooter_mode = SHOOT_OFF;
	
	create_uint16_btn(&fn1_btn, &vt_data.controller.fn_1);
	create_uint16_btn(&fn2_btn, &vt_data.controller.fn_2);
	create_uint16_btn(&pause_btn, &vt_data.controller.pause);
	create_uint8_btn(&trigger_btn, &vt_data.controller.trigger);
	
	create_uint8_btn(&key_w_btn, &keyboard.key_w);
	create_uint8_btn(&key_s_btn, &keyboard.key_s);
	create_uint8_btn(&key_a_btn, &keyboard.key_a);
	create_uint8_btn(&key_d_btn, &keyboard.key_d);
	create_uint8_btn(&key_ctrl_btn,  &keyboard.key_ctrl);
	create_uint8_btn(&key_shift_btn, &keyboard.key_shift);
	create_uint8_btn(&key_q_btn, &keyboard.key_q);
	create_uint8_btn(&key_e_btn, &keyboard.key_e);
	create_uint8_btn(&key_c_btn, &keyboard.key_c);
	create_uint8_btn(&key_v_btn, &keyboard.key_v);
	
	create_uint8_btn(&mouse_left_btn, &keyboard.mouse_left);
	create_uint8_btn(&mouse_right_btn, &keyboard.mouse_right);
	
	pitch_cfg = (DJIMotorConfig){ // CAN1 1 pitch
		.escId = 1, .type = DJI_MOTOR_GM6020,
		.bus = DJI_MOTOR_BUS_CAN1,
		.ctrlMode = DJI_MOTOR_CTRL_CURRENT};
	DJIMotor_Add(&pitch_cfg);
		
	yaw_cfg = (DJIMotorConfig){ // CAN2 2 yaw
		.escId = 2, .type = DJI_MOTOR_GM6020,
		.bus = DJI_MOTOR_BUS_CAN2,
		.ctrlMode = DJI_MOTOR_CTRL_CURRENT}; 
	DJIMotor_Add(&yaw_cfg);
		
	fric_l_cfg = (DJIMotorConfig){ // CAN1 fric wheel
		.escId = 1, .type = DJI_MOTOR_M3508,
		.bus = DJI_MOTOR_BUS_CAN1,
		.ctrlMode = DJI_MOTOR_CTRL_CURRENT};
	DJIMotor_Add(&fric_l_cfg);
		
	fric_r_cfg = (DJIMotorConfig){ // CAN1 fric wheel
		.escId = 2, .type = DJI_MOTOR_M3508,
		.bus = DJI_MOTOR_BUS_CAN1,
		.ctrlMode = DJI_MOTOR_CTRL_CURRENT};
	DJIMotor_Add(&fric_r_cfg);
		
	feeder_cfg = (DJIMotorConfig){ // CAN2 feeder
		.escId = 1, .type = DJI_MOTOR_M2006,
		.bus = DJI_MOTOR_BUS_CAN2,
		.ctrlMode = DJI_MOTOR_CTRL_CURRENT};
	DJIMotor_Add(&feeder_cfg);
	
	DJIMotor_BuildAll();
}

// ****************************************** End of Init ******************************************//


// ****************************************** Begin of Helpers ******************************************//

void update_keyboard_status(void) {
	keyboard.key_w = KEY_W ? 1 : 0;
	keyboard.key_s = KEY_S ? 1 : 0;
	keyboard.key_a = KEY_A ? 1 : 0;
	keyboard.key_d = KEY_D ? 1 : 0;
	keyboard.key_q = KEY_Q ? 1 : 0;
	keyboard.key_e = KEY_E ? 1 : 0;
	keyboard.key_c = KEY_C ? 1 : 0;
	keyboard.key_v = KEY_V ? 1 : 0;
	keyboard.key_ctrl  = KEY_CTRL  ? 1 : 0;
	keyboard.key_shift = KEY_SHIFT ? 1 : 0;
	
	keyboard.mouse_left  = vt_data.mouse_key.mouse_left  ? 1 : 0;
	keyboard.mouse_right = vt_data.mouse_key.mouse_right ? 1 : 0;
}

// ****************************************** End of Helpers ******************************************//


// ****************************************** Begin of Mode Switch ******************************************//

void control_mode_switch(void) {
	if(!manual_online && 0) main_control_mode.control_mode = OFF_MODE;
	
	else {
		switch(vt_data.controller.mode_sw){
			case SW_RIGHT:
				main_control_mode.control_mode = CONTROLLER_MODE2;
				break;
			
			case SW_MID:
				main_control_mode.control_mode = CONTROLLER_MODE1;
				break;
			
			case SW_LEFT:
				main_control_mode.control_mode = MOUSE_KEY_MODE;
				break;
			
			default:
				main_control_mode.control_mode = OFF_MODE;
				break;
		}
	}
}


void gimbal_mode_switch(void) {	
	switch (main_control_mode.control_mode) {
		case CONTROLLER_MODE1:
		case CONTROLLER_MODE2:
			if (main_control_mode.gimbal_mode != GIMBAL_AUTO_AIM &&
				  main_control_mode.gimbal_mode != GIMBAL_CONTROLLER) {
				main_control_mode.gimbal_mode = GIMBAL_CONTROLLER;
			}
		
			if (main_control_mode.fric_mode == FRIC_OFF && trigger_btn.rise) {
				if (main_control_mode.gimbal_mode == GIMBAL_CONTROLLER && aimbot_online) {
					main_control_mode.gimbal_mode = GIMBAL_AUTO_AIM;
				} else {
					main_control_mode.gimbal_mode = GIMBAL_CONTROLLER;
				}
			}
			break;
		case MOUSE_KEY_MODE:
			#if PERFORMANCE_TEST_ON
			if (trigger_btn.rise) {
				performance_test_start();
			}
			if (pause_btn.rise) {
				performance_test_stop();
				performance_test_reset();
			}
			#endif
		
			if (mouse_right_btn.level && aimbot_online) {
				main_control_mode.gimbal_mode = GIMBAL_AUTO_AIM;
			} else {
				main_control_mode.gimbal_mode = GIMBAL_MOUSE_KEY_CONTROL;
			}
			break;
		default:
			main_control_mode.gimbal_mode = GIMBAL_IDLE;
			break;
	}
}

void chassis_mode_switch(void) {
	switch (main_control_mode.control_mode) {
		case CONTROLLER_MODE1:
		case CONTROLLER_MODE2:
			main_control_mode.chassis_mode = CHASSIS_CONTROLLER;
			break;
		case MOUSE_KEY_MODE:
			main_control_mode.chassis_mode = CHASSIS_MOUSE_KEY_CONTROL;
			break;
		default:
			main_control_mode.chassis_mode = CHASSIS_IDLE;
			break;
	}
}

void shoot_17mm_mode_switch(void) {
	switch (main_control_mode.control_mode) {
		case CONTROLLER_MODE1:
		case CONTROLLER_MODE2:
			if (main_control_mode.shooter_mode == SHOOT_AUTO_AIMBOT) {
				main_control_mode.shoot_17mm_mode = SHOOT_17MM_AUTO_AIM;
			} else {
				main_control_mode.shoot_17mm_mode = SHOOT_17MM_CONTROLLER;
			}
			break;
		case MOUSE_KEY_MODE:
			main_control_mode.shoot_17mm_mode = SHOOT_17MM_MOUSE_KEY_CONTROL;
			break;
		default:
			main_control_mode.shoot_17mm_mode = SHOOT_17MM_IDLE;
			break;
	}
}

void cha_gim_mode_switch(void) {
	switch (main_control_mode.control_mode) {
		case CONTROLLER_MODE1:
			main_control_mode.cha_gim_mode = CHASSIS_GIMBAL_SEPERATE;
			break;
		case CONTROLLER_MODE2:
			main_control_mode.cha_gim_mode = CHASSIS_GIMBAL_FOLLOW_0_DEG;
			break;
		case MOUSE_KEY_MODE:
			if (key_v_btn.rise) {
				if (main_control_mode.cha_gim_mode == CHASSIS_GIMBAL_SEPERATE) {
					main_control_mode.cha_gim_mode = CHASSIS_GIMBAL_FOLLOW_0_DEG;
				} else {
					main_control_mode.cha_gim_mode = CHASSIS_GIMBAL_SEPERATE;
				}
			}
			break;
		default:
			main_control_mode.cha_gim_mode = CHASSIS_GIMBAL_SEPERATE;
			break;
	}
}

void rotate_mode_switch(void) {
	switch (main_control_mode.control_mode) {
		case CONTROLLER_MODE1:
		case CONTROLLER_MODE2:
			if (fn1_btn.rise) {
				if (main_control_mode.rotate_mode == ROTATE_OFF) {
					main_control_mode.rotate_mode = ROTATE_SLOW;
				} else {
					main_control_mode.rotate_mode = ROTATE_OFF;
				}
			}
			break;
		case MOUSE_KEY_MODE:
			if (key_c_btn.rise) {
				if (main_control_mode.rotate_mode == ROTATE_OFF) {
					main_control_mode.rotate_mode = ROTATE_SLOW;
				} else {
					main_control_mode.rotate_mode = ROTATE_OFF;
				}
			}
			break;
		default:
			main_control_mode.rotate_mode = ROTATE_OFF;
			break;
	}
}

void fric_mode_switch(void) {
	switch (main_control_mode.control_mode) {
		case CONTROLLER_MODE1:
		case CONTROLLER_MODE2:
			if (fn2_btn.rise) {
				if (main_control_mode.fric_mode == FRIC_OFF) {
					main_control_mode.fric_mode = FRIC_ON;
				} else {
					main_control_mode.fric_mode = FRIC_OFF;
				}
			}
			break;
		case MOUSE_KEY_MODE:
			if (key_e_btn.rise) {
				if (main_control_mode.fric_mode == FRIC_OFF) {
					main_control_mode.fric_mode = FRIC_ON;
				} else {
					main_control_mode.fric_mode = FRIC_OFF;
				}
			}
			break;
		default:
			main_control_mode.fric_mode = FRIC_OFF;
			break;
	}
}

void shooter_mode_switch(void) {
	switch (main_control_mode.control_mode) {
		case CONTROLLER_MODE1:
		case CONTROLLER_MODE2:
			if (pause_btn.rise) {
				if (main_control_mode.shooter_mode == SHOOT_OFF) {
					main_control_mode.shooter_mode = SHOOT_SINGLE;
				} else if (main_control_mode.shooter_mode == SHOOT_SINGLE) {
					main_control_mode.shooter_mode = SHOOT_AUTO;
				} else if (main_control_mode.shooter_mode == SHOOT_AUTO && 
									 main_control_mode.gimbal_mode == GIMBAL_AUTO_AIM) {
					main_control_mode.shooter_mode = SHOOT_AUTO_AIMBOT;
				} else {
					main_control_mode.shooter_mode = SHOOT_OFF;
				}
			}
			break;
		case MOUSE_KEY_MODE:
			if (key_q_btn.rise) {
				if (main_control_mode.shooter_mode == SHOOT_SINGLE) {
					main_control_mode.shooter_mode = SHOOT_AUTO;
				} else {
					main_control_mode.shooter_mode = SHOOT_SINGLE;
				}
			}
			break;
		default:
			main_control_mode.shooter_mode = SHOOT_OFF;
			break;
	}
}

// ****************************************** End of Mode Switch ******************************************//


// ****************************************** Begin of Target Send ******************************************//
void gimbal_target_send(void) {
	// target of gimbal is based on current angle
	
	#if PERFORMANCE_TEST_ON
	if (main_control_mode.control_mode != MOUSE_KEY_MODE) {
		performance_test_stop();
		performance_test_reset();
	}
	
	performance_test_set_yaw_sine(5.0f, 4.0f, 0.0f);
	if (performance_test_is_on()) {
		main_control_mode.gimbal_mode = GIMBAL_PERFORMANCE_TEST;
	}
	#endif
    
	switch (main_control_mode.gimbal_mode) {
		case GIMBAL_CONTROLLER:
			gimbal_cmd.yaw_angle_set -= vt_data.controller.ch_3 * YAW_INC / 4.0f;
			gimbal_cmd.yaw_speed_set = 0;
			gimbal_cmd.pitch_angle_set += vt_data.controller.ch_2 * PIT_INC / 4.0f;
			gimbal_cmd.pitch_speed_set = 0;
			gimbal_cmd.auto_mode = 0;
			break;
		case GIMBAL_AUTO_AIM: {
			if (auto_aimbot.dist < 0.0f || auto_aimbot.enemy_id == 0) {
				gimbal_cmd.yaw_angle_set += 0;
				gimbal_cmd.yaw_speed_set = 0;
				gimbal_cmd.pitch_angle_set += 0;
				gimbal_cmd.pitch_speed_set = 0;
				gimbal_cmd.auto_mode = 0;	
			} else {
				uint32_t cur_time = time();
				float delta_time = (float)(cur_time - auto_receive_time) / 1000.0f;
				if (delta_time < 0.0f || delta_time > 1.0f) delta_time = 0.0f;
				
				gimbal_cmd.yaw_angle_set = auto_aimbot.yaw + delta_time * auto_aimbot.yaw_spd * RADIAN_TO_DEGREE;
				gimbal_cmd.yaw_speed_set = auto_aimbot.yaw_spd;
				gimbal_cmd.pitch_angle_set = auto_aimbot.pitch + delta_time * auto_aimbot.pitch_spd * RADIAN_TO_DEGREE;
				gimbal_cmd.pitch_speed_set = auto_aimbot.pitch_spd;
				gimbal_cmd.auto_mode = 1;
			}
			break;
		}
		case GIMBAL_MOUSE_KEY_CONTROL:
			gimbal_cmd.yaw_angle_set -= (float)(vt_data.mouse_key.mouse_x) * YAW_INC / 500.0f;
			gimbal_cmd.yaw_speed_set = 0;
			gimbal_cmd.pitch_angle_set += (float)(vt_data.mouse_key.mouse_y) * PIT_INC / 300.0f;
			gimbal_cmd.pitch_speed_set = 0;
			gimbal_cmd.auto_mode = 0;
			break;
		case GIMBAL_PERFORMANCE_TEST: {
			uint32_t cur_time = time();
			float delta_time = (float)(cur_time - cur_ref.update_time) / 1000.0f;
			if (delta_time < 0.0f || delta_time > 1.0f) delta_time = 0.0f;
			
			gimbal_cmd.yaw_angle_set = cur_ref.y_pos_tar + delta_time * cur_ref.y_spd_tar * RADIAN_TO_DEGREE;
			gimbal_cmd.yaw_speed_set = cur_ref.y_spd_tar;
			gimbal_cmd.pitch_angle_set = cur_ref.p_pos_tar + delta_time * cur_ref.p_spd_tar * RADIAN_TO_DEGREE;
			gimbal_cmd.pitch_speed_set = cur_ref.p_spd_tar;
			gimbal_cmd.auto_mode = 1;
			break;
		}
		case GIMBAL_IDLE:
		default:
			gimbal_cmd.yaw_angle_set += 0;
			gimbal_cmd.yaw_speed_set = 0;
			gimbal_cmd.pitch_angle_set += 0;
			gimbal_cmd.pitch_speed_set = 0;
			gimbal_cmd.auto_mode = 0;
			break;
	}
	
	VAL_LIMIT(gimbal_cmd.pitch_angle_set,PITCH_IMU_LOWER_BOUND,PITCH_IMU_UPPER_BOUND);
	gimbal_set_command(&gimbal_cmd);
}

void chassis_target_send(void) {
	switch (main_control_mode.chassis_mode) {
		case CHASSIS_CONTROLLER:
			chassis_cmd_trans.ch_0 = (int16_t)(vt_data.controller.ch_0*32767.0f);
			chassis_cmd_trans.ch_1 = (int16_t)(vt_data.controller.ch_1*32767.0f);
			chassis_cmd_trans.cha_gim_status = main_control_mode.cha_gim_mode;
			chassis_cmd_trans.rotate_status = main_control_mode.rotate_mode;
			chassis_cmd_trans.wheel = vt_data.controller.wheel - 1024;
			break;
		case CHASSIS_MOUSE_KEY_CONTROL: {
			float ch_0 = 0.0f, ch_1 = 0.0f;
			if (key_w_btn.level) ch_1 += 1.0f;
			if (key_s_btn.level) ch_1 -= 1.0f;
			if (key_a_btn.level) ch_0 -= 1.0f;
			if (key_d_btn.level) ch_0 += 1.0f;
			
			chassis_cmd_trans.ch_0 = (int16_t)(ch_0*32767.0f);
			chassis_cmd_trans.ch_1 = (int16_t)(ch_1*32767.0f);
			chassis_cmd_trans.cha_gim_status = main_control_mode.cha_gim_mode;
			chassis_cmd_trans.rotate_status = main_control_mode.rotate_mode;
			chassis_cmd_trans.wheel = key_shift_btn.level * 660;
			break;
		}
		default:
			chassis_cmd_trans.ch_0 = (int16_t)(0.f);
			chassis_cmd_trans.ch_1 = (int16_t)(0.f);
			chassis_cmd_trans.cha_gim_status = CHASSIS_GIMBAL_SEPERATE;
			chassis_cmd_trans.rotate_status = ROTATE_OFF;
			chassis_cmd_trans.wheel = 0;
			break;
	}
}


void shoot_17mm_target_send(void) {
	/*
	main_control_mode.shooter_mode = SHOOT_SINGLE;
	// for shoot latency measure
	*/
	
	if(main_control_mode.fric_mode == FRIC_OFF) {
		shoot_17mm_stop_fric();
		shoot_17mm_stop_shooter();
		return ;
	}
	
	shoot_17mm_set_fric(1);
	
	switch (main_control_mode.shoot_17mm_mode) {
		case SHOOT_17MM_CONTROLLER:
			if (main_control_mode.shooter_mode == SHOOT_OFF) {
				shoot_17mm_stop_shooter();
			} else if (main_control_mode.shooter_mode == SHOOT_SINGLE) {
				if(trigger_btn.rise) {
					shoot_17mm_set_shooter(1);
					
					// measure shoot latency
					obs.triggered = 1;
					obs.trigger_ts = time();
				}
			} else if (main_control_mode.shooter_mode == SHOOT_BURST3) {
				// placeholder
				if(trigger_btn.rise) shoot_17mm_set_shooter(3);
			} else if (main_control_mode.shooter_mode == SHOOT_AUTO) {
				if (trigger_btn.rise) shoot_17mm_set_shooter(AUTO_INF);
				if (trigger_btn.fall) shoot_17mm_stop_shooter();
			}
			break;
		case SHOOT_17MM_AUTO_AIM:
			if (
				auto_aimbot.shoot == 1 && // tmp
				fabsf(auto_aimbot.yaw - attitude.totalYaw) < atan2f(0.05f, auto_aimbot.dist) &&
				fabsf(auto_aimbot.pitch - attitude.pitch)  < atan2f(0.05f, auto_aimbot.dist)
			) {
				shoot_17mm_set_shooter(AUTO_INF);
			} else if (
				auto_aimbot.shoot == 2 &&
				fabsf(auto_aimbot.yaw - attitude.totalYaw) < atan2f(0.05f, auto_aimbot.dist) &&
				fabsf(auto_aimbot.pitch - attitude.pitch)  < atan2f(0.05f, auto_aimbot.dist)
			) {
				shoot_17mm_set_shooter(AUTO_INF);
			} else {
				shoot_17mm_stop_shooter();
			}
			break;
		case SHOOT_17MM_MOUSE_KEY_CONTROL:
			if (main_control_mode.shooter_mode == SHOOT_OFF) {
				shoot_17mm_stop_shooter();
			} else if (main_control_mode.shooter_mode == SHOOT_SINGLE) {
				if (mouse_left_btn.rise) shoot_17mm_set_shooter(1);
			} else if (main_control_mode.shooter_mode == SHOOT_AUTO) {
				if (mouse_left_btn.rise) shoot_17mm_set_shooter(AUTO_INF);
				if (mouse_left_btn.fall) shoot_17mm_stop_shooter();
			}
			break;
		default:
			break;
	}
	
}

// ****************************************** End of Target Send ******************************************//

// ****************************************** Begin of Main Task ******************************************//
void main_control_task(void * args) {
	main_control_init();
	
	while(1) {
		update_keyboard_status();
		update_button_status();
		
		// switch mode of whole mode
		control_mode_switch();
		
		//task mode switch 
		gimbal_mode_switch();
		chassis_mode_switch();
		shoot_17mm_mode_switch();
		
		cha_gim_mode_switch();
		rotate_mode_switch();
		fric_mode_switch();
		shooter_mode_switch();
		
		// send target of components
		gimbal_target_send();
		chassis_target_send();
		shoot_17mm_target_send();
		
		delay(3);
	}
}

// ****************************************** End of Main Task ******************************************//
