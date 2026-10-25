#include "dji_example.h"
#include "controller.h"
#include <math.h>

// ?????(?????),???????????
#define TOTAL_RANGE 1803500   // ???????????

int16_t left_limit = 0;
int16_t right_limit = 0;
uint8_t limits_found = 0;

PIDParams dji_params;
PID pid_dji;
Controller ctrl_dji;

DJIMotorProvCtx dji_ctx;
MeasProvider dji_prov;

DJIMotorConfig dji_cfg;

// ???????
void dji_find_left_limit(void) {
    int16_t start_pos, last_pos, current_pos;
    uint32_t last_time, now;
    float dt;
    int stall_count;
    int16_t limit_pos;

    start_pos = CAN1_Motors[3].totalAngle;
    last_pos = start_pos;
    last_time = HAL_GetTick();
    stall_count = 0;
    controller_set_vref(&ctrl_dji, -2000.0f);  // ???? 100 dps
    while (1) {
        delay(5);
        current_pos = CAN1_Motors[3].totalAngle;
        int16_t current_current = CAN1_MotorCurrents[3];  // ??????
        now = HAL_GetTick();
        dt = (now - last_time) / 1000.0f;
        if (dt > 0) {
            if (current_current>=1100 ) {  // ???????????
                stall_count++;
                if (stall_count > 5) {  // ????
                    limit_pos = current_pos;
                    break;
                }
            } else {
                stall_count = 0;
            }
        }
        last_pos = current_pos;
        last_time = now;

        // ??????,???????
        if (abs(start_pos - current_pos) > TOTAL_RANGE * 2) {
            limit_pos = current_pos;
            break;
        }
    }
    left_limit = limit_pos;
    right_limit = left_limit + TOTAL_RANGE;  // ??????????

    // ??
    controller_set_vref(&ctrl_dji, 0);
    delay(10);

    limits_found = 1;

    // ??:?????
    // int16_t mid = (left_limit + right_limit) / 2;
    // ??????????(?)
}

void dji_example_init(void) {
    // Initialize DJI Motor
    dji_cfg = (DJIMotorConfig){
        .escId = 3, .type = DJI_MOTOR_M2006,
        .bus = DJI_MOTOR_BUS_CAN1,
        .ctrlMode = DJI_MOTOR_CTRL_CURRENT};
    DJIMotor_Add(&dji_cfg);
        
    // Build all motors
    DJIMotor_BuildAll();
    
    // Initialize PID Controller
    dji_params = (PIDParams){.Kp=2.0f,  .Ki=0.0f, .Kd=0.5f, .out_limit=9500.0f, .integral_limit=4000.0f};
    pid_init(&pid_dji, dji_params, 0.005f, PID_PLANT_VELOCITY);
    
    // Initialize Measurement Provider
    djimotor_ctx_init(&dji_ctx, &CAN1_Motors[3], 1.0f, 0.0f, 1, 0);
    dji_prov = make_djimotor_provider(&dji_ctx);
    
    // Bind Provider to PID
    pid_bind_provider(&pid_dji, &dji_prov, 1);
    
    // Create Controller from PID
    ctrl_dji = make_pid(&pid_dji);
    
    // Enable Controller
    controller_enable(&ctrl_dji, true);
    
    // Bind Controller to Control Runtime
    control_runtime_bind_int16(&ctrl_dji, &CAN1_MotorCurrents[3], 1400.0f);
}

#define BOUNDARY_NEAR 3000   // ???????(?????),???????
#define SAFE_SPEED 500        // ???????????

void dji_example_update(void) {
			if(control.online)
        controller_set_vref(&ctrl_dji, 9000 * control.channel[3]);
			else
				controller_set_vref(&ctrl_dji, 0);
}

void dji_motor_task(void *args) {
    dji_example_init();
    //dji_find_left_limit();
    while (1) {
        dji_example_update();
        delay(5);
    }
}