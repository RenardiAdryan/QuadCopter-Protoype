/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
 /*
 *  Created on: Sep 14, 2019
 *      Author: Renardi Adryantoro P
 				Polteknik elektronika negeri suarabaya
 				Computer Engineering 2016
*/



/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GlobalVariable.h"
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
varPID GyroYAW,GyroPITCH,GyroROLL;
int yawMAX,pitchMAX,rollMAX;
int prevMode;
char flagdroneARM;
char flagparsingIMU,flagparsingReceiver;
uint8_t data1[60];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void buzzControl(int repeate,int delay){
static int rep;
rep = repeate;
digitalWrite(BuzzPORT,BuzzPIN,LOW);
	while(rep){
		digitalWrite(BuzzPORT,BuzzPIN,HIGH);
		HAL_Delay(delay);
		digitalWrite(BuzzPORT,BuzzPIN,LOW);
		HAL_Delay(delay);
	rep--;
	}
	digitalWrite(BuzzPORT,BuzzPIN,LOW);
}

void ledControl(int repeate,int delay){

static int rep;
rep = repeate;
digitalWrite(LED1PORT,LED1PIN,LOW);
	while(rep){
		digitalWrite(LED1PORT,LED1PIN,HIGH);
		HAL_Delay(delay);
		digitalWrite(LED1PORT,LED1PIN,LOW);
		HAL_Delay(delay);
	rep--;
	}
	digitalWrite(LED1PORT,LED1PIN,LOW);
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  InitializeTimer();
  InitSerialIMU_DMA();
  InitSerialReceiver_DMA();
  HAL_Delay(2500);

  resetIMU();
  digitalWrite(GPIOB,GPIO_PIN_5,HIGH);
    HAL_Delay(200);
    digitalWrite(GPIOB,GPIO_PIN_5,LOW);
    HAL_Delay(100);
    digitalWrite(GPIOB,GPIO_PIN_5,HIGH);
    HAL_Delay(50);
    digitalWrite(GPIOB,GPIO_PIN_5,LOW);

 //==Pid constanta===
  GyroYAW.kp = 0.02;   GyroYAW.kd = 0.00125; GyroYAW.ki = 0.000;

  GyroROLL.kp = 0.08; GyroROLL.kd = 0.0155; GyroROLL.ki = 0.00;

  GyroPITCH.kp = 0.06;GyroPITCH.kd = 0.015;GyroPITCH.ki = 0.0025;
 //==================

  //PID Setpoint Initiate
  GyroYAW.setPoint = GyroROLL.setPoint = GyroPITCH.setPoint = 0;
  //==================
  //init Essensial Variable
  yawReceive   = 400;
  pitchReceive = 50;
  rollReceive  = 50;


  //Bounding to ESC
  int InitializePulse = 1000;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,InitializePulse);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,InitializePulse);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,InitializePulse);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,InitializePulse);
	  HAL_Delay(2000);


  digitalWrite(GPIOB,GPIO_PIN_5,HIGH);
  HAL_Delay(200);
  digitalWrite(GPIOB,GPIO_PIN_5,LOW);
  HAL_Delay(200);
  digitalWrite(GPIOB,GPIO_PIN_5,HIGH);
  HAL_Delay(100);
  digitalWrite(GPIOB,GPIO_PIN_5,LOW);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  //Button 1
		  if(digitalRead(BUTTON1PORT,BUTTON1PIN)==LOW){
			  HAL_Delay(1);
			  while(digitalRead(BUTTON1PORT,BUTTON1PIN)==LOW){}
			  if(digitalRead(BUTTON1PORT,BUTTON1PIN)==HIGH){ flagdroneARM=LOW; resetIMU();HAL_Delay(500);buzzControl(5,150);}
		  }
		  //BUtton 1 END

		  //Button 2
		  if(digitalRead(BUTTON2PORT,BUTTON2PIN)==LOW){
			  HAL_Delay(1);
			  while(digitalRead(BUTTON2PORT,BUTTON2PIN)==LOW){}
			  if(digitalRead(BUTTON2PORT,BUTTON2PIN)==HIGH){
				  if(flagdroneARM ==LOW) {flagdroneARM=HIGH;buzzControl(2,300);}
				  else if(flagdroneARM ==HIGH){ flagdroneARM=LOW;buzzControl(1,200);}
			  }
		  }
		  //BUtton 2 END

		  //Speed Mode
		 if(speedMode != prevMode){
			 if(speedMode == 1){pitchMAX=rollMAX = 5;buzzControl(1,100);}
			 else if(speedMode == 2){pitchMAX=rollMAX = 10;buzzControl(2,100);}
			 else if(speedMode == 3){pitchMAX=rollMAX = 15;buzzControl(3,100);}
			 else if(speedMode == 4){pitchMAX=rollMAX = 20;buzzControl(4,100);}
			 else if(speedMode == 0){pitchMAX=rollMAX = 0;buzzControl(1,300);}
			 prevMode = speedMode;
		 }
		 //SpeedMode end

		 // led notification
		 if(flagparsingReceiver !=1 || flagparsingIMU != 1){
			 if(flagparsingReceiver ==1 && flagparsingIMU !=1 ){ledControl(1,50);}
			 else if(flagparsingReceiver !=1 && flagparsingIMU ==1 ){ledControl(1,100);}
			 else if(flagparsingReceiver !=1 && flagparsingIMU !=1 ){ledControl(1,200);}
		 }
		 else digitalWrite(LED1PORT,LED1PIN,HIGH);
		 // led notification end


//		sprintf((char*)data1,"%d | %f | %f |\n",dataPitch,GyroPITCH.error,GyroPITCH.output);
//		HAL_UART_Transmit(&huart1,data1, sizeof(data1), 10);



		//sprintf((char*)data1,"%d | %.2f| %.2f | %.2f | %d | %d |\n",dataPitch,GyroPITCH.setPoint,GyroPITCH.error,GyroPITCH.output,RPMmotor1,RPMmotor2);
		// HAL_UART_Transmit(&huart1,data1, sizeof(data1), 10);
		// HAL_Delay(10);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (72000000/(50 * 20000))-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==huart1.Instance){
		Parsing_Receiver();
	}
	if(huart->Instance==huart3.Instance){
		Parsing_IMU();
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
