/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*I2C address of Gyro Module*/
#define gyroAddr 0b11010000

/*NB Low First Byte in received data*/

/*CAN IDs*/
#define BUFFER_BLOCK1	0x101
#define BUFFER_BLOCK2	0x102
#define BUFFER_BLOCK3	0x103

#define GPS_BLOCK1		0x104
#define GPS_BLOCK2		0x105
#define GPS_BLOCK3		0x106

#define GYRO_BLOCK1		0x107
#define GYRO_BLOCK2		0x108

#define FWHEEL_BLOCK1	0x109
#define FWHEEL_BLOCK2	0x110

#define RWHEEL_BLOCK1	0x111
#define RWHEEL_BLOCK2	0x112

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
//GYRO variables
uint8_t accelData[6];
uint8_t gyroData[6];

//CAN variables
CAN_RxHeaderTypeDef receiveHeader;
CAN_TxHeaderTypeDef transmitHeader;
CAN_FilterTypeDef sFilter;
CAN_FilterTypeDef sFilter2;
uint8_t can_received_data[8];
uint8_t can_tx_payload[8];
uint32_t mailbox;

//Pot variables
uint16_t left_pot=0;
uint16_t right_pot=0;

//Steering Encoder
uint8_t steeringEncoder=0;				//TO-DO

//TIMING variables
uint32_t currTime = 0;
uint32_t lastTime = 0;

//Vel variables
uint16_t left_count=0;
uint8_t left_status=0;
uint16_t right_count=0;
uint8_t right_status=0;
uint8_t deltaT=0;
uint32_t startTime=0;
uint8_t resetStartTime=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void filterSettings(void);
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
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /*Set Filter*/
    filterSettings();

    HAL_CAN_Start(&hcan1);

    /*Enable interrupt*/
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

    /*Set transmitHeader fields*/
    transmitHeader.StdId=FWHEEL_BLOCK1;
    transmitHeader.ExtId = 0x01;
    transmitHeader.IDE = CAN_ID_STD;
    transmitHeader.RTR = CAN_RTR_DATA;
    transmitHeader.TransmitGlobalTime = DISABLE;

    /*I2C Settings*/
    if(HAL_I2C_IsDeviceReady(&hi2c1, gyroAddr, 10, 1000)==HAL_OK); //hi2c, DevAddress, Trials, Timeout

    uint8_t accelReg = 0x3B;
    uint8_t gyroReg = 0x43;

    /*Wake up module*/
    uint8_t wakeUp[2];
    wakeUp[0] = 0x6B;
    wakeUp[1] = 0x00;

    /*setup accel scale*/
    uint8_t setAccelReg[2];
    setAccelReg[0]=0x1C;
    setAccelReg[1]=0b00001000;		//set accel to 4g

    if(HAL_I2C_Master_Transmit(&hi2c1, gyroAddr, wakeUp, sizeof(wakeUp), 1000)!=HAL_OK);
    if(HAL_I2C_Master_Transmit(&hi2c1, gyroAddr, setAccelReg, sizeof(setAccelReg), 1000)!=HAL_OK);

    currTime=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*Read Pot data every 10ms*/
	  	  if((currTime - lastTime) >= 10){
	  			lastTime = currTime;

	  			//ADC ROUTINE
	  			HAL_ADC_Start(&hadc1);
	  			HAL_ADC_Start(&hadc2);

	  			left_pot=HAL_ADC_GetValue(&hadc1);
	  			right_pot=HAL_ADC_GetValue(&hadc2);

	  			HAL_ADC_Stop(&hadc1);
	  			HAL_ADC_Stop(&hadc2);

	              /*GYROSCOPE*/
	  			//Request data from register 0x3B (Accel)
	  			if(HAL_I2C_Master_Transmit(&hi2c1, gyroAddr, &accelReg, 1, 10)!=HAL_OK);
	  			if(HAL_I2C_Master_Receive(&hi2c1, gyroAddr, accelData, 6, 10)!=HAL_OK);

	  			//Request data from register 0x43 (Gyro)
	  			if(HAL_I2C_Master_Transmit(&hi2c1, gyroAddr, &gyroReg, 1, 10)!=HAL_OK);
	  			if(HAL_I2C_Master_Receive(&hi2c1, gyroAddr, gyroData, 6, 10)!=HAL_OK);
	  	  }

	  	  currTime=HAL_GetTick();

	  	  /*Time period to consider in the vel calculation*/
	  	  deltaT=currTime-startTime;

	  	  if(resetStartTime){
	  		  resetStartTime = 0;
	  		  startTime = currTime;
	  	  }

	  	  /*
	  	   * TO-DO: ENCODER VOLANTE
	  	   * */

    /* USER CODE END WHILE */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : rightVel_Pin */
  GPIO_InitStruct.Pin = rightVel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rightVel_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : leftVel_Pin */
  GPIO_InitStruct.Pin = leftVel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(leftVel_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void filterSettings(void){
	sFilter.FilterBank = 0;
	sFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilter.FilterIdHigh = 0;
	sFilter.FilterIdLow = 0;
	sFilter.FilterMaskIdHigh = 0;
	sFilter.FilterMaskIdLow = 0;
	sFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilter.FilterActivation = ENABLE;
	sFilter.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilter);
}

/*CAN Communication*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &receiveHeader, can_received_data) != HAL_OK){
		/* ERROR HANDLER */
	}else{

		//SEND POT LEFT DATA      TODO: unire right e left pot
		if(receiveHeader.StdId==FWHEEL_BLOCK1 && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_REMOTE){
			transmitHeader.StdId=FWHEEL_BLOCK1;
			transmitHeader.DLC=4;
			can_tx_payload[0]=left_pot&0xFF;
			can_tx_payload[1]=right_pot&0xFF;
			can_tx_payload[2]=(left_pot>>8)|((right_pot>>4)&0xF0);
			can_tx_payload[3]=ADC_GET_RESOLUTION(&hadc1);
			HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_tx_payload, &mailbox);
		}

        /*SEND POT RIGHT DATA
		if(receiveHeader.StdId==RWHEEL_BLOCK1 && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_REMOTE){
			transmitHeader.StdId=RWHEEL_BLOCK1;
			transmitHeader.DLC=4;
			can_tx_payload[0]=left_pot&0xFF;
			can_tx_payload[1]=right_pot&0xFF;
			can_tx_payload[2]=(left_pot>>8)|((right_pot>>4)&0xF0);
			can_tx_payload[3]=ADC_GET_RESOLUTION(&hadc1);

			HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_tx_payload, &mailbox);
		}*/

		//SEND VEL DATA + STEERING_ENC
		else if(receiveHeader.StdId==FWHEEL_BLOCK2 && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_REMOTE){
			transmitHeader.StdId=FWHEEL_BLOCK2;
			transmitHeader.DLC=5;

			can_tx_payload[0]=left_count;
			can_tx_payload[1]=right_count;
			can_tx_payload[2]=(left_count>>8)|((right_count>>4)&0xF0);
			can_tx_payload[3]=deltaT;
			can_tx_payload[4]=steeringEncoder;

			left_count=0;
			right_count=0;
			resetStartTime=1;

			HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_tx_payload, &mailbox);
		}else{
			/*DO NOTHING*/
		}

        //SEND ACCEL DATA
		if(receiveHeader.StdId==GYRO_BLOCK1 && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_REMOTE){
			transmitHeader.StdId=GYRO_BLOCK1;
			transmitHeader.DLC=6;

			can_tx_payload[0]=accelData[0];
			can_tx_payload[1]=accelData[1];
			can_tx_payload[2]=accelData[2];
			can_tx_payload[3]=accelData[3];
			can_tx_payload[4]=accelData[4];
			can_tx_payload[5]=accelData[5];

			HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_tx_payload, &mailbox);
		}

		//SEND GYRO DATA
		else if(receiveHeader.StdId==GYRO_BLOCK2 && receiveHeader.IDE==CAN_ID_STD && receiveHeader.RTR==CAN_RTR_REMOTE){
			transmitHeader.StdId=GYRO_BLOCK2;
			transmitHeader.DLC=6;

			can_tx_payload[0]=gyroData[0];
			can_tx_payload[1]=gyroData[1];
			can_tx_payload[2]=gyroData[2];
			can_tx_payload[3]=gyroData[3];
			can_tx_payload[4]=gyroData[4];
			can_tx_payload[5]=gyroData[5];

			HAL_CAN_AddTxMessage(&hcan1, &transmitHeader, can_tx_payload, &mailbox);
		}else{
			/*DO NOTHING*/
		}
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
