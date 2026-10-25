# RM2022CBF

A Framework for RoboMaster C Board Rev. 2022

**IMPORTANT: READ THIS DOCUMENT BEFORE USING THIS DEVELOPING FRAMEWORK!**

# **<span style="color:red;">仓库名里的2022起到底层框架版本标识作用，别他妈的把你车组项目名里的框架版本年份跟着赛季年份乱改！</span>**

## Usage:

Write thread functions containing a dead loop.

Put any thread you want in user_main.c.

Compile & Flash

## Components:

### OS

A simplified layer of FreeRTOS's thread. Out of the real-usage concern, this layer only gives thread creation method. By 

>osThreadId_t osThreadCreate(char const *name, osThreadFunc_t func, void *arg, osPriority_t priority, uint32_t stackSize)

where name can be whatever you want, as long as they are not the same as each other, func can be any void type function that has a dead loop within, priority can be selected from enum osPriority_t, and stackSize can be any number on 2's power(large than 32 is suggested).

### LED

LED component offers an easy way to control the light strenth and color. 

>void LED_SetRGB(uint16_t R_value, uint16_t G_value, uint16_t B_value)

This function will change the color and light strength according to the value you give, ranging from 0 to 65535. After booting up, the default task in main.c will control the light with breathing light effect with cyan color. Any call to LED_SetRGB() will stop the breathing light effect and set the light to the status given by user permanently.

>void LED_Off()

This function shuts off the light.

### Controller

No functions should be used outside of this file. Users may get controller's data by global struct "control".Structure of control are the same as this in ABL.

### EXTI

This file overrides GPIO_EXTI_Callback in HAL, defines external interrupt's reactions. The key's press callback is provided via a __weak function KEY_Triggered(), you should redefine it outside of this file. 

DO NOT MODIFY any thing inside unless you know what you are doing!

### PID

This component provides a PID tools to use. See pid.h for details.

### PWM

This component provides a PWM interface. The "PWM" sign on the board is abstracted as pwm[1]. See pwm.h for details.

### Motor

This component provides an interface to DJI motor control and sensor. You can get motor info by CAN1_Motors[n], n ranges from 1 to 12.
Inside CAN1_Motors[n], you can get the current, speed, temprature, angle of the motor. "totalAngle" records the angle delta compared with booting status, which may be useful in gimbal controlling.

### USB

This component provides an sample for USB virtual com sending. Logs can be output through there.
Now the firmware will print "Welcome to Magician Universal Development Platform" each second.
Receiving callback is 

>static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)

in usbd_cdc_if.c. 

Comm is not included in this firmware, so this callback is left blank and ready for editing.

### IMU

This components offers data detected by the IMU. By default, imu.attitude will contain all the attitude data you want. However, you can override the data processing function by redefining IMU_DataFrame_Handler to process the data the first time a new data is measured.

**Warning: For IMU now is implemented through DMA, so DMA callback is re-written. Each time you generate code with STM32CubeMX, don't forget to comment the**

> HAL_DMA_IRQHandler(&hdma_spi1_rx);

**of DMA2_Stream2_IRQHandler in stm32f4xx_it.c.**

### Laser

This component offers an interface for operating the on-board 5V output.
>void Laser_5V_Enable()

>void Laser_5V_Disable()

### Status Monitor

This component provides an interface for monitoring input voltage, in-chip temprature and hardware version. The first two data will be periodically got by DefaultTask in main.c. The data will be put in struct "core". 

The percentage of a 6S battery correspoding to the current voltage is also calculated and put in "core".

### FIFO

A FIFO for upper applications.


## Global Sensor Data Overview

### Temperature:

IMU: bmi088_real_data.temp

Chip: core.temperature

### Attitude:

Yaw: imu.attitude.yaw

Pitch: imu.attitude.pitch

Roll: imu.attitude.roll

### Voltage:

Input: core.batt_voltage
