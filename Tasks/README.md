# New frame of sentry 2025

## 主控
### 相关文件
头文件：Tasks/main_control.h 源文件：Tasks/main_control.c

相关：发射，底盘，云台，通讯

### 机器人模式

    // states of control mode of the whole robot
    typedef enum{
        OFF_MODE,
        CONTROLLER_MODE1,
        CONTROLLER_MODE2,
        NAVIGATION_MODE,
        FIXEDPOINT_MODE,
        CV_DEBUG_MODE,
        DEFENSE_MODE,
        CV_PID_ADJUST_MODE,
        PITCH_GRA_COM_ADJUST_MODE,
    } control_mode_e;

    // states of chassis control mode
    typedef enum {
        CONTROLLER_ALONE, // controlled by controller
        TOP_WITH_CONTROLLER, // controlled by controller while topping
        TOP_WITH_SLAM, // controlled by SLAM while topping
        SLAM_ALONE, // controlled by SLAM
        TOP_ALONE, // only top
        CHASSIS_OFF, // off
    } chassis_control_mode_e;

    // states of shoot mode
    typedef enum{
        OFF,
        SINGLE,
        CONTINUE,
        SHOOT_BLOCK, // prwheel back set when it is blocked
    }shoot_mode_e;

    // states of gimbal mode
    typedef enum {
        CONTROLLER_CONTROLLED, 
        KEEP_STILL,
    } gimbal_mode_e;

    // internal, information of mode including all modes
    typedef struct {
        // informtion about mode
        control_mode_e control_mode;
        
        chassis_control_mode_e chassis_control_mode;
        
        int fric_on; // whether friction wheel is on
        
        shoot_mode_e shoot_mode;
        
        gimbal_mode_e gimbal_mode;
        
        // helping variables which determine shoot mode
        int shoot_allow; // only for single shoot
        int heat_spare; // whether heat is spare for shoot
        int back_set_flag; // whether prwheel need to back set
        
        uint32_t prwheel_back_set_start_time; // start time of prwheel back set
    } main_control_mode_t;

### 电机控制器

    // internal, controller of motor including pid, fsfc and feedforward
    typedef struct {
        PID wheel_pids[5];
        
        PID left_friction_pid;
        PID right_friction_pid;
        
        PID prwheel_pid;
        
        FSFC yaw_fsfc;
        
        FSFC pitch_fsfc;
        float pitch_gravity_compensation_current; // current of pitch gravity compensation (feedforward)
    } motor_controller_t;

### 配置

    // for controller
    /// L: control.triSwitch[0]  R: control.triSwitch[1]
    #define L    0
    #define R    1

    /// for 3 states of control.triSwitch
    #define UP   1
    #define MID  3
    #define DOWN 2 

    // for prwheel back set
    #define BACK_SET_CURRENT 2000 // motor current for prwheel back set
    #define BACK_SET_TIME  200 // the duation of prwheel back set

    // distinct speed target of prwheel
    #define PRWHEEL_EXPECTED_SPD_FAST 200 // for fast continue shoot
    #define PRWHEEL_EXPECTED_SPD_SLOW 120 // for slow continue shoot
    #define PRWHEEL_EXPECTED_SPD_SINGLE 60 // for single shoot
    #define PRWHEEL_EXPECTED_SPD_FOR_CV_DEBUG 40 // for single shoot when CV_DEBUG

    #define FRIC_SPD_30 7050 // speed of friction wheel before deceleration

    #define YAW_INC -250.0f / 360.0f // increase speed of yaw when controlled by contoller
    #define PIT_INC -1.0f / 360.0f * 8192.0f // increase speed of pitch when controlled by contoller

### 概览
主要控制流程是：主模式切换，各部件模式切换（其中发射模式需要额外依赖于当前的发射状态），发送各部件状态的目标值，由各部件计算相应电机的目标值，接受各部件的计算结果并通过电机控制器控制电机。

## 发射
### 相关文件
头文件：Tasks/compo_shoot.h 源文件：Tasks/compo_shoot.c

相关：主控，通讯

### 主要接口
初始化：
    `
    void compo_shoot_init(void);
    `

计算电机目标值：
    `
    void compo_shoot_task(void);
    `

更新状态：
    `
    void update_shoot_state(void);
    `

反馈状态：
    `
    void update_shoot_state_feedback(void);
    `

反馈电机目标值：
    `
    void update_shoot_control_feedback(void);
    `

接受裁判系统通讯：
    `
    void handler_bullet(uint8_t* );
    `

控制接口：

    // external, commander of shoot
    typedef struct {
        int prwheel_back_set; // whether prwheel need to back set
        
        int fric_speed_set; // rpm before deceleration
        int prwheel_speed_set; // rpm after deceleration
    } sender_to_shoot_t;
    extern sender_to_shoot_t sender_to_shoot;
    

状态反馈接口:
    
    // external, responsor of shoot about state
    typedef struct {
        int block_flag; // whether it is checked that prwheeel is blocked
        
        int	bullet_count; // counter of fired bullet
        
        float	cur_local_heat; // current local heat
    } receiver_from_shoot_state_t;
    extern receiver_from_shoot_state_t receiver_from_shoot_state;
    

电机目标值反馈接口：

    // external, responser of shoot about control
    typedef struct {
        int left_friction_speed_set; // rpm before deceleration
        int right_frction_speed_set; // rpm before deceleration
        
        float prwheel_speed_set; // when prwheel doesn't need to back set, it is used, unit: rpm after deceleration
        
        int16_t left_friction_speed_get; // rpm before deceleration
        int16_t right_friction_speed_get; // rpm before deceleration
        
        int16_t prwheel_speed_get; // rpm after deceleration
    } receiver_from_shoot_control_t;
    extern receiver_from_shoot_control_t receiver_from_shoot_control; 

### 配置

    // for checking shoot block
    #define BLOCK_TIME_THRESHOLD 70 // threshold of block time to determine that prwheel is blocked

    // for count_fired_bullet to determine that a bullet is fired
    #define FRIC_SPEED_DECREASE_INTEGRAL_THRESHOLD -6 // threshold of continuous decrease of fric speed
    #define FRIC_SPEED_DECREASE_THRESHOLD 9 // threshold of decrease of fric speed relative to fric speed target
    #define DIFFERENTIAL_THRESHOLD 0.1 // threshold of change of fric speed which is counted into fric_speed_decrease_integral

## 底盘
### 相关文件
头文件：Tasks/compo_chassis.h 源文件：Tasks/compo_chassis.c

相关：主控

### 主要接口
初始化：

    extern void compo_chassis_init(void);

计算电机目标值：

    extern void compo_chassis_task(void);

反馈：

    extern void update_chassis_feedback(void);

控制接口：

    // command for topping
    typedef struct{
        int level;
        int direction;
        int	enable;
        
        float speed; //deg/s
        // for correcting the angle between the direction of one armor and gimbal in chassisAngle()
        float	compensation;
        // to make sure that when topping, robot is at the original position
        float vx_bias; //mm/s
        float vy_bias; //mm/s
    }top_set_t;

    // external, commander of chassis
    typedef struct {
        // total speed of chassis but relative to gimbal
        float vx_set; //mm/s
        float vy_set; //mm/s
        top_set_t top_set;
    } sender_to_chassis_t;
    extern sender_to_chassis_t sender_to_chassis;

反馈接口：

    // external, responser of chassis
    typedef struct {
        float wheel_speed_set[5];
        float wheel_speed_get[5];
    } receiver_from_chassis_t;
    extern receiver_from_chassis_t receiver_from_chassis;

### 配置

    #define ACC_ALLOWED 14.0f // acceleration limit of chassis
    #define DEC_ALLOWED 15.0f // deceleration limit of chassis

    #define YAW_OFFSET_FORWARD 3583 // 0-8192, yaw value when gimbal is directed at armor 0, which is the positive forward direction of chassis

## 云台
### 相关文件
头文件：Tasks/compo_gimbal.h 源文件：Tasks/compo_gimbal.c

相关：主控

### 主要接口
初始化：

    extern void compo_gimbal_init();

计算电机目标值：

    extern void compo_gimbal_task();

反馈：

    extern void update_gimbal_feedback();

控制接口：

    // external, commander of gimbal
    typedef struct {
        float yaw_angle_set; // unit: deg
        float yaw_speed_set; // rad/s
        
        float pitch_angle_set; // unit: motor
        float pitch_speed_set; // motor/s
    } sender_to_gimbal_t;
    extern sender_to_gimbal_t sender_to_gimbal;

反馈接口：

    // external, responser of gimbal
    typedef struct {
        float yaw_angle_set; // unit: deg
        float yaw_speed_set; // rad/s
        
        float pitch_angle_set; // unit: motor
        float pitch_speed_set; // motor/s
        
        float yaw_angle_get; // unit: deg
        float yaw_speed_get; // rad/s
        
        float pitch_angle_get; // unit: motor
        float pitch_speed_get; // motor/s
    } receiver_from_gimbal_t;
    extern receiver_from_gimbal_t receiver_from_gimbal;

### 配置

    #define PITCH_IMU_UP_BOUND 14*DEGREE_TO_MOTOR // maximum pitch angle, unit: motor
    #define PITCH_IMU_LOW_BOUND -26*DEGREE_TO_MOTOR // minimum pitch angle, unit: motor

## 其他配置
### 相关文件
头文件：Tasks/robot_conf.h

### 配置

    #define SENTRY 1

    // whether switch to navigation mode
    #define NAVIGATION_ON 0

    // whether use defence mode instead of fixedpoint mode
    #define DEFENSE_MODE_ON 1

    // some special mode
    #define CV_DEBUG 0
    #define SHOW_MODE 0

    // for adjustment
    #define SHOOT_TEST 0
    #define CV_PID_ADJUST 0
    #define SLOW_TOP_COMPESATION_ADJUST 0
    #define FAST_TOP_COMPESATION_ADJUST 0
    #define WHEEL_PID_ADJUST 0

    // for new function test
    #define PITCH_GRA_COM_TEST 1

    #define PI          3.1416f
    #define SQUARE_ROOT_OF_2_MULTIPLIED_BY_2 0.7071f 
    #define RADIAN_COEF 57.3f // 180/3.14
    #define DEGREE_TO_MOTOR  22.7556f // 8192/360
    #define DEGREE_TO_RADIAN 0.0174f // 3.14f/180.0f

    #define _WHEEL_RADIUS      72.0f  // the radius of wheel(mm)
    #define _WHEEL_PERIMETER   358.0f // the perimeter of wheel(mm)

    #define _CHASSIS_WHEELTRACK  380.0f // wheel track distance(mm)
    #define _CHASSIS_WHEELBASE   230.0f // wheel base distance(mm)
    #define _CHASSIS_RADIUS      480.0f // radius of chassis(mm)

    #define FRIC_DIR -1
    #define PRWHEEL_DIR -1
    #define YAW_DIR 1
    #define PITCH_DIR -1

    #define M3508_MAX_WHEEL_RPM 8740 // rotor speed
    #define M3508_DECELE_RATIO 19.0f // M3508_decelerate_ratio
    #define M2006_REDUCTION_RATIO 36.0f // M2006_decelerate_ratio
    #define GM6020_MAX_CODE_DISC_VALUE 8192
    #define GM6020_CODE_DISC_VALUE_CHANGE_PER_90_DEGREE 2048

    #define FRIC1_ID 2 // left friction wheel
    #define FRIC2_ID 1 // right friction wheel
    #define PRWHEEL_ID 3 // puller wheel
    #define PITCH_ID 6
    #define YAW_ID 5

    /* chassis maximum translation speed, unit is mm/s */
    #define _CHASSIS_MAX_VX_SPEED 1000.0f
    #define _CHASSIS_MAX_VY_SPEED 1000.0f
    /* chassis maximum rotation speed, unit is degree/s */
    #define _CHASSIS_MAX_VR_SPEED 920.0f

    #define HEAT_PER_SHOOT 10


