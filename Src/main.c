/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "flash_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t jump_flag=0;
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	SCB->VTOR = 0x08010000U; //Vec
//	uint32_t Data_Orig=0x0fff012f;
//	uint32_t a;
	uint8_t Data[]={0xAA,0x55,0xAA,0x54,0xAA,0x53,0xAA,0x52,0xAA,0x51,0xAA,0x50,0xAA,0x51,0xAA,0x52,0xAA,0x53,0xAA,0x54,0xAA,0x55,};
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);
		  uint8_t key = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
//	__enable_irq();//使能中断

  /* USER CODE BEGIN Init */
     /* 设置中断表起始地址 */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
//	FLASH_If_Erase(APPLICATION_ADDRESS);
	
  /* USER CODE BEGIN 2 */
//  if(FLASH_If_Erase(APPLICATION_ADDRESS)==0)
//	printf("***************用户flash擦除成功****\r\n");//擦除了内存区40k的数据
//	else
//	printf("***************用户flash擦除失败****\r\n");
//	
//	if(FLASH_If_Write(APPLICATION_ADDRESS, (uint32_t*)Data, 4)==FLASHIF_OK)
//  printf("***************用户flash写入成功****\r\n");//擦除了内存区40k的数据
//	else
//	printf("***************用户flash写入失败****\r\n");
//	printf("取地址上第一个的数据:%x\r\n",*(__IO uint32_t*)(APPLICATION_ADDRESS+4));
//	Jump2APP(APPLICATION_ADDRESS);
  /* USER CODE END 2 */
//	__disable_irq();
//	HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
//		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
//		HAL_NVIC_DisableIRQ(TIM3_IRQn);
//    __set_FAULTMASK(1) ;
		printf("取地址上第一个的数据:%x\r\n",(__IO uint32_t)(SCB->ICSR));
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		  hcan_driver_send_single_frame(0x12345,Data,8);
////	hcan_driver_send_multi_frame(0x12345,Data,4);
//		hcan_driver_query_bus_status();
//	printf("***************控制器1****\r\n");
//			printf("***************控制器2****\r\n");
//		printf("can status is=%d\r\n",HAL_CAN_GetError(&hcan1));
//		HAL_Delay(1000);
////		
//		hcan_packet_transmit(PRIORITY_STATUS,DEV_CENTRAL_CONTROLER,DEV_MOTOR_CONTROLER,1,HCAN_FRAME_NORMAL,Data,8);
//    hcan_loop();
//					printf("***************控制器3****\r\n");
//				HAL_Delay(1000);
		if(jump_flag!=0)
		{
			printf("跳转APP运行\r\n");

		HAL_CAN_MspDeInit(&hcan1);
		HAL_UART_MspDeInit(&huart5);
		HAL_TIM_Base_MspDeInit(&htim2);
		HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
		HAL_DeInit();
//		HAL_FLASH_Lock();
//		NVIC_SystemReset();
//		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
//		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
//		HAL_NVIC_DisableIRQ(TIM3_IRQn);
		HAL_NVIC_DisableIRQ(SysTick_IRQn);
		HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);
				Jump2APP(APPLICATION_ADDRESS);
		}
    /* USER CODE END WHILE */
// HAL_UART_Receive(&huart5, &key, 1, 0xFFFFFFFF);
		if(key=='5' )
		{
			printf("跳转APP运行\r\n");

		HAL_CAN_MspDeInit(&hcan1);
		HAL_UART_MspDeInit(&huart5);
		HAL_TIM_Base_MspDeInit(&htim2);
		HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
		HAL_DeInit();
//		HAL_FLASH_Lock();
//		NVIC_SystemReset();
//		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
//		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
//		HAL_NVIC_DisableIRQ(TIM3_IRQn);
		HAL_NVIC_DisableIRQ(SysTick_IRQn);
		HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);
				Jump2APP(APPLICATION_ADDRESS);
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
