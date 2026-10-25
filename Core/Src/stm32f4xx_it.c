/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller.h"
#include "imu.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "communicate.h"
#include "board_communicate.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define __MY_UART_CLEAR_IDLE_FLAG(__UART__) \
do { \
  /* read SR then DR will clean idle flag */ \
  __IO uint32_t tmpreg = 0x00U; \
  tmpreg = (__UART__)->SR; \
  tmpreg = (__UART__)->DR; \
  UNUSED(tmpreg); \
} while(0)

#define __MY_USART3_DMA_PAUSE_RX() \
do { \
  /* disable USART DMA receiver */ \
  USART3->CR3 &= ~USART_CR3_DMAR; \
  /* disable DMA stream */ \
  DMA1_Stream1->CR &= ~DMA_SxCR_EN; \
} while (0)

#define __MY_USART3_DMA_RESET_RX() \
do { \
  /* reset number of data register */ \
  DMA1_Stream1->NDTR = 40; \
  /* ensure destination pointer */ \
  DMA1_Stream1->M0AR = (uint32_t) rc_buf; \
  /* clear all IT flags of the DMA stream */ \
  DMA1->LIFCR |= 0x00000f40UL; \
  /* enable DMA stream */ \
  DMA1_Stream1->CR |= DMA_SxCR_EN; \
  /* enable USART DMA receiver */ \
  USART3->CR3 |= USART_CR3_DMAR; \
} while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
  uint16_t StdId;
  uint8_t FMI;
  uint8_t motorId;
  
  if ((CAN1->IER & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U) {
    /* Check if message is still pending */
    if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U) {
      StdId = (CAN1->sFIFOMailBox[0].RIR >> CAN_TI0R_STID_Pos) & 0x7ff;
      /* let filter match index decide what to do */
      FMI = (CAN1->sFIFOMailBox[0].RDTR >> CAN_RDT0R_FMI_Pos) & 0xff;
      switch (FMI) {
        case 0: /* first filter is for DJI motor */
					if (StdId >= 0x0400) {
						comm_receiver(StdId, (uint8_t*) &CAN1->sFIFOMailBox[0].RDLR);
				  }
					else if(StdId < 0x100) {
						DMMotor_ESCDataUnpack(USE_CAN1, (uint8_t*) &CAN1->sFIFOMailBox[0].RDLR);
					}
					else{
						motorId = StdId - ESC_BASE_ID;
						if (motorId > 12) break;
						DJIMotor_ESCDataUnpack((uint8_t*) &CAN1->sFIFOMailBox[0].RDLR, CAN1_Motors + motorId);
					}
          break;
      }
      /* release Rx FIFO 0 */
      SET_BIT(CAN1->RF0R, CAN_RF0R_RFOM0);
      /* preemptive exit */
      return;
    }
  }
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	
	if ((USART1->SR & UART_FLAG_IDLE) && (USART1->CR1 & UART_IT_IDLE & UART_IT_MASK)) {
		/* clear flag */
		__MY_UART_CLEAR_IDLE_FLAG(USART1);
		HAL_UART_IdleCallback(&huart1);
		return;
	}
	
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if ((USART3->SR & UART_FLAG_IDLE) && (USART3->CR1 & UART_IT_IDLE & UART_IT_MASK)) {
    /* clear flag */
    __MY_UART_CLEAR_IDLE_FLAG(USART3);
		if(control.signalSource == SIGNAL_CONTROLLER){
			// For controller doesn't packed in protocol, hence we can directly process them
			if (DMA1_Stream1->CR & DMA_SxCR_EN) {
				__MY_USART3_DMA_PAUSE_RX();
				if (DMA1_Stream1->NDTR == RC_BUF_SIZE - 18) {
					_lastRCPacketTime = HAL_GetTick();
					notifyRCDataUnpacker();
				}
				__MY_USART3_DMA_RESET_RX();
			}else{
				HAL_UART_Abort(&huart3);
				HAL_UART_Receive_DMA(&huart3,(uint8_t *)rc_buf,RC_BUF_SIZE);
			}
		}
	else // For video transmitter, we can verify the packet by protocol, so we process it with comm
		HAL_UART_IdleCallback(&huart3);
    return;
  }

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	DMA2_Stream2_IRQHandler_IMU();
	return; // Prevent warning when statement below is uncommented
	// ==============================ALERT===================================
	// Comment the MX code below after each time generating code in CubeMX!!!
	// ======================================================================
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
  uint16_t StdId;
  uint8_t FMI;
  uint8_t motorId;
  
  if ((CAN2->IER & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U) {
    /* Check if message is still pending */
    if ((CAN2->RF0R & CAN_RF0R_FMP0) != 0U) {
      StdId = (CAN2->sFIFOMailBox[0].RIR >> CAN_TI0R_STID_Pos) & 0x7ff;
      /* let filter match index decide what to do */
      FMI = (CAN2->sFIFOMailBox[0].RDTR >> CAN_RDT0R_FMI_Pos) & 0xff;
      switch (FMI) {
        case 0: /* first filter is for DJI motor */
					if (StdId >= 0x0400) {
						comm_receiver(StdId, (uint8_t*) &CAN2->sFIFOMailBox[0].RDLR);
				  }
          else if(StdId < 0x100) {
					  DMMotor_ESCDataUnpack(USE_CAN2, (uint8_t*) &CAN2->sFIFOMailBox[0].RDLR);
					}
					else if(StdId == 0x211) {
						//CAP_Unpack((uint8_t*) &CAN2->sFIFOMailBox[0].RDLR);
					}
					else {
						motorId = StdId - ESC_BASE_ID;
            if (motorId > 12) break;
						DJIMotor_ESCDataUnpack((uint8_t*) &CAN2->sFIFOMailBox[0].RDLR, CAN2_Motors + motorId);
          }
          break;
      }
      /* release Rx FIFO 0 */
      SET_BIT(CAN2->RF0R, CAN_RF0R_RFOM0);
      /* preemptive exit */
      return;
    }
  }
  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	
	if ((USART6->SR & UART_FLAG_IDLE) && (USART6->CR1 & UART_IT_IDLE & UART_IT_MASK)) {
		/* clear flag */
		__MY_UART_CLEAR_IDLE_FLAG(USART6);
		HAL_UART_IdleCallback(&huart6);
		return;
	}
	
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
