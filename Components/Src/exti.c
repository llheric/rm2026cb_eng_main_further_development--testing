#include "exti.h"
#include "imu.h"
#include "main.h"
#include "ist8310driver.h"

__weak void KEY_Triggered(){
}

__weak void SWITCH_Triggered(){
	//shoot.delay = time() - shoot.start_time;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INT1_ACCEL_Pin)
	{
		accel_update_flag |= 1 << IMU_DR_SHIFTS;
		accel_temp_update_flag |= 1 << IMU_DR_SHIFTS;
		if(imu_start_dma_flag)
		{
			imu_cmd_spi_dma();
		}
	}
	else if(GPIO_Pin == INT1_GYRO_Pin)
	{
		gyro_update_flag |= 1 << IMU_DR_SHIFTS;
		if(imu_start_dma_flag)
		{
			imu_cmd_spi_dma();
		}
	}
	else if(GPIO_Pin == DRDY_IST8310_Pin)
	{
		mag_update_flag |= 1 << IMU_DR_SHIFTS;

		if(mag_update_flag &= 1 << IMU_DR_SHIFTS)
		{
			mag_update_flag &= ~(1<< IMU_DR_SHIFTS);
			mag_update_flag |= (1 << IMU_SPI_SHIFTS);

			
			// ist8310_read_mag(ist8310_real_data.mag);
			// Data Unreliable
		}
	}
	else if(GPIO_Pin == KEY_Pin){
		KEY_Triggered();
	}
//	else if(GPIO_Pin == SWITCH_Pin){
//	  SWITCH_Triggered();
//	}
}
