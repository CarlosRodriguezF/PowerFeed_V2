/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "STM32_I2C_Display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RIGHT 1			//Definition RIGHT to 1
#define LEFT 0			//Definition RIGHT to 1
#define CLK_FREQ_T2	42000000		//Frequency for the Clock, used on TIM2
#define ACC_UPDATE_RATIO 50			//RAtio for Acceleration update in ms (MAX 1 Second)
#define HIGH_SPEED_INCREMENT 50		//Increment where Target speed if higher than min speed gap
#define LOW_SPEED_INCREMENT 1		//Increment where Target speed if higher than min speed gap
#define MIN_SPEED_GAP 30			//Difference between target and current speed to use HIGH_SPEED_INCREMENT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/*Variables for Encoder Read*/
int32_t encoder_value = 0x7FFF;			//Current Encoder Value
int32_t old_encoder_value = 0x7FFF;		//Previous Encoder Value
uint16_t en_invert = 0;					//Variable to storage the value of Invert for ENABLE PIN
uint16_t dir_invert = 0;				//Variable to invert the value of Invert for DIR PIN
uint16_t motor_stepsrev = 1600;			//Variable to storage the steps/rev for the motor
uint16_t leadscrew_pitch = 2;			//VAriable to storage the pitch for the leadscrew in mm/rev
uint16_t current_speed = 0;				//Variable for the current speed
uint16_t target_speed = 0;				//Variable for the target_speed
/* FLAG */
uint16_t update_speed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
int32_t Encoder_Read(int32_t *old_value);
void LCD_Write_Number(int32_t value, int32_t col_pos, int32_t raw_pos);
void Motor_Enable(uint16_t invert);
void Motor_Disable(uint16_t invert);
void Motor_Direction(uint16_t direction, uint16_t invert);
void Motor_Speed_RPM(uint16_t speed);
uint16_t Motor_Speed_Update(uint16_t *current_speed, uint16_t *target_speed);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  //Encoder Initialization
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LiquidCrystal_I2C(0x4E, 20, 4);
  lcdBegin();
  lcdSetCursor(1,1);
  lcdPrint("Power Feed V2.0");
  lcdBacklight();
  lcdClear();

  while (1)
  {
	  lcdClear();
	  HAL_Delay(10);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  TIM1->CNT = 0x7FFF;		//Initialization CNT in middle value to avoid Over/Under flow
  TIM1->SR = ~(1UL << 0);	//Clear UIF flag
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */
	float TIM11_period_ms = (float)ACC_UPDATE_RATIO/1000;		//Period to load into the timer, calculated from Define
	uint16_t TIM11_preescaler = 642;							//Preescaler, max 1 second
	uint16_t TIM11_ARR;
	TIM11_ARR = ( (float) (CLK_FREQ_T2/(TIM11_preescaler+1))*TIM11_period_ms );	//Calculation value for ARR register to set correct period

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = TIM11_preescaler;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = TIM11_ARR;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENABLE_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE_Pin DIR_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_LEFT_Pin SW_RIGHT_Pin EN_SW_Pin */
  GPIO_InitStruct.Pin = SW_LEFT_Pin|SW_RIGHT_Pin|EN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim11 ){
		update_speed = 1;
	}
}


/**
  * @brief Encoder Steps Read Function
  * @param old_value - Pointer to the value storaged as latest encoder value
  * @retval encoder_steps - Number of steps increased or decreased on the encoder
  */
int32_t Encoder_Read(int32_t *old_value)
{
	int32_t encoder_steps;

	if (TIM1->SR & (1 << 0)){		//If overflow or underflow occurs reset the CNT value
		TIM1->SR = ~(1UL << 0);		//Reset UIF bit
		TIM1->CNT = 0x7FFF;			//Reload CNT register to ox7FFF
		return 0;					//Return 0
	}

	uint16_t encoder_value = TIM1->CNT;		//Variable to storage the CNT register value
	if ( ( encoder_value - *old_value >= 2 ) || ( encoder_value - *old_value <= -2 ) ){		//If the value in the encoder register changed (At least 2, to avoid glitches) calculate increment
		encoder_steps = (*old_value - encoder_value)/2;	//Divide by 2 is needed due to increments by two on the encoder
		*old_value = encoder_value;			//Reload the old_value
		return encoder_steps;				//Return the increments, can be positive or negative
	}else{
		return 0;							//Return 0 in case no changes
	}
}

/**
  * @brief Function to write number into LCD, will clear the fields not used.
  * @param 	value - value which expected to be writen into the LCD
  * 		col_pos - column position for the number
  * 		row_pos - raw position for the number
  * @retval
  */
void LCD_Write_Number(int32_t value, int32_t col_pos, int32_t raw_pos)
{
	char str[10];
	sprintf(str, "%ld", value);
	if (value > 0){
		if (value < 10){
			lcdSetCursor(col_pos+1,raw_pos);
			lcdPrint(" ");
			lcdSetCursor(col_pos,raw_pos);
			lcdPrint(str);
		}else if (value < 100){
			lcdSetCursor(col_pos+1,raw_pos);
			lcdPrint("  ");
			lcdSetCursor(col_pos,raw_pos);
			lcdPrint(str);
		}else if (value < 1000){
			lcdSetCursor(col_pos+1,raw_pos);
			lcdPrint("   ");
			lcdSetCursor(col_pos,raw_pos);
			lcdPrint(str);
		}
	}else if (value < 0) {
		if (value > -10){
			lcdSetCursor(col_pos+2,raw_pos);
			lcdPrint(" ");
			lcdSetCursor(col_pos,raw_pos);
			lcdPrint(str);
		}else if (value > -100){
			lcdSetCursor(col_pos+2,raw_pos);
			lcdPrint("  ");
			lcdSetCursor(col_pos,raw_pos);
			lcdPrint(str);
		}else if (value > -1000){
			lcdSetCursor(col_pos+2,raw_pos);
			lcdPrint("   ");
			lcdSetCursor(col_pos,raw_pos);
			lcdPrint(str);
		}
	}else{
		lcdSetCursor(col_pos,raw_pos);
		lcdPrint("  ");
		lcdSetCursor(col_pos,raw_pos);
		lcdPrint("0");
	}
}

/**
  * @brief Function to Enable EN signal for Motor Driver
  * @param 	invert - variable to invert the EN pin logic
  * @retval
  */
void Motor_Enable(uint16_t invert){
	HAL_GPIO_WritePin(GPIOA, ENABLE_Pin, (GPIO_PIN_SET^invert));	//Enable Motor, XOR with SET to invert it if selected
}

/**
  * @brief Function to Disable EN signal for Motor Driver
  * @param 	invert - variable to invert the EN pin logic
  * @retval
  */
void Motor_Disable(uint16_t invert){
	HAL_GPIO_WritePin(GPIOA, ENABLE_Pin, (GPIO_PIN_RESET^invert));	//Disable Motor, XOR with SET to invert it if selected
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

/**
  * @brief Function to select the direction of the motor
  * @param 	direction - variable to set the direction of the motor
  * 		invert - variable to invert the DIR pin logic
  * @retval
  */
void Motor_Direction(uint16_t direction, uint16_t invert){
	if ( direction == RIGHT ){
		HAL_GPIO_WritePin(GPIOA, DIR_Pin, (GPIO_PIN_SET^invert));	//Disable Motor, XOR with SET to invert it if selected
	}else if (direction == LEFT){
		HAL_GPIO_WritePin(GPIOA, DIR_Pin, (GPIO_PIN_RESET^invert));	//Disable Motor, XOR with SET to invert it if selected
	}

}

/**
  * @brief Function to select the speed of the motor in RPM
  * @param 	rpm - Speed value in RPM it is wanted
  * @retval
  */
void Motor_Speed_RPM(uint16_t speed){
	float ARR_value_temp = 0;
	uint32_t ARR_value;
	ARR_value_temp = ((60 * (float) CLK_FREQ_T2)/(speed*motor_stepsrev));	//Calculation Value to load in ARR
	ARR_value = (uint32_t) ARR_value_temp;	//Uint32 casting
	TIM2->ARR = ARR_value+1;				//Load ARR + 1
	TIM2->CCR1 = (uint32_t) (ARR_value+1)/2;	//Load CCR1 to have always 50% Duty Cycle
	if ((TIM2->CR1 & (1 << 0)) ^ (1 << 0)){			//Checking if the Timer is already enabled, if not, enable it
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	}
}

/**
  * @brief Function to update the speed of the motor following the acceleration
  * @param 	current_speed - Current speed of the motor
  * 		target_speed - Target speed of the motor
  * @retval updated_speed - Updated speed of the motor
  */
uint16_t Motor_Speed_Update(uint16_t *current_speed, uint16_t *target_speed){
	if (*target_speed > *current_speed){			//Speed Incrementation
		if( (*target_speed - *current_speed) > MIN_SPEED_GAP ){		//If gap is bigger than MIN_SPEED_GAP, increment higher
			*current_speed = *current_speed + HIGH_SPEED_INCREMENT;	//Increment Speed
		}else{
			*current_speed = *current_speed + LOW_SPEED_INCREMENT;	//If the gap is lower, then increase lower
		}
	}else if (*target_speed < *current_speed){		//Speed lowering
		if( (*current_speed - *target_speed) > MIN_SPEED_GAP ){		//If gap is bigger than MIN_SPEED_GAP, decrease higher
			*current_speed = *current_speed - HIGH_SPEED_INCREMENT;	//Increment Speed
		}else{
			*current_speed = *current_speed - LOW_SPEED_INCREMENT;	//If the gap is lower, then decrease lower
		}
	}else if (*target_speed == *current_speed){		//Speed match

	}
	return *current_speed;
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
