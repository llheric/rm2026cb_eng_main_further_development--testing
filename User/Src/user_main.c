#include "user_main.h"
#include "os.h"
#include "robot_conf.h"
#include "control_runtime.h"

#include "dji_example.h"
#include "dm_example.h"
#include "comm.h"
#include "board_comm.h"

/*******************************************************************
 * >>>>>>>>>>>>>>>>>>>>>>>>>>> NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<<<< *
 *******************************************************************
 * You are allowed to do whatever you like to this file as long as *
 * the User_Init() function is kept here, and the header file      *
 * user_main.h are included.                                       *
 *                                                                 *
 * You are supposed to wrap your main codes in functions, not      *
 * placing them into User_Init(). This function is only meant to   *
 * be the place where you init your global variables, create and   *
 * start your own threads. Make sure you exit it as soon as possi- *
 * ble. NEVER should there be any infinite loops.                  *
 *                                                                 *
 * If you need to define any macros or global variables, please    *
 * create your own header file under the group Application/main.   *
 * So should your own source file (*.c) be.                        *
 *                                                                 *
 * If you have any problem, feel free to open an issue on GitLab.  *
 *                                                                 *
 * Happy coding, RoboMasters :)                                    *
 *                                                                 *
 *******************************************************************/
 
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	huart->TxXferCount = 1;
}
 
void User_Init(void){
	control_runtime_init();
	osThreadCreate("control_runtime_task", control_runtime_task, NULL, osPriorityAboveNormal, 256);
	
 	//osThreadCreate("dji_motor_task", dji_motor_task, NULL, osPriorityNormal, 128);
 	osThreadCreate("dm_motor_task",  dm_motor_task,  NULL, osPriorityNormal, 128);
	Comm_init();
	board_comm_init();
}
