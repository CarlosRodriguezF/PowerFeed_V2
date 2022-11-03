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
#include "stdlib.h"
#include "STM32_I2C_Display.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0
#define FAIL -1
#define MID 2
#define RIGHT 1						//Definition RIGHT to 1
#define LEFT 0						//Definition LEFT to 0
#define CLK_FREQ_T2	42000000		//Frequency for the Clock, used on TIM2
#define ACC_UPDATE_RATIO 50			//RAtio for Acceleration update in ms (MAX 1 Second)
#define ACC_TIME 1000				//Acceleration time in ms
#define DEBOUNCING_TIME 100			//TIme for checking debouncing FLAG (MAX 1 Second)
#define HIGH_SPEED_INCREMENT 10		//Increment where Target speed if higher than min speed gap
#define LOW_SPEED_INCREMENT 1		//Increment where Target speed if higher than min speed gap
#define MIN_SPEED_GAP 30			//Difference between target and current speed to use HIGH_SPEED_INCREMENT
/*Defines for State Machine*/
#define INITIALIZATION	 0			//Initialization state
#define STANDBY			 3			//Standby State no Movement
#define MOVE_RIGHT		 1			//Move right state
#define MOVE_LEFT		 2			//Move left state

#define STEP_NORMAL 0				//Define to select STEP NORMAL
#define STEP_x10 	1				//Define to select step mode x10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/*Variables for Encoder Read*/
int32_t encoder_value = 0x7FFF;			//Current Encoder Value
int32_t old_encoder_value = 0x7FFF;		//Previous Encoder Value

/*State Machine variables*/
uint16_t state = INITIALIZATION;				//Variable for the main state machine
uint16_t previous_state = INITIALIZATION;		//Variable for the previous state machine

uint16_t step_mode = STEP_NORMAL;		//Select mode of increase steps

/*Configuration variables*/
uint16_t en_invert = 0;					//Variable to storage the value of Invert for ENABLE PIN
uint16_t dir_invert = 0;				//Variable to invert the value of Invert for DIR PIN
uint16_t motor_stepsrev = 1600;			//Variable to storage the steps/rev for the motor
float leadscrew_pitch = 2;				//Variable to storage the pitch for the leadscrew in mm/rev
uint16_t MAX_FEEDRATE = 500;			//Maximum Feedrate on mm/min

/*General variables*/
int16_t current_feedrate = 0;			//Variable for the current feedrate
int16_t target_feedrate = 200;			//Variable for the target feedrate
int16_t display_feedrate = 200;			//Variable to display the target feedrate into the screen
uint16_t sw_status = 0;

/* FLAGS*/
uint16_t update_speed = 0;				//Flag to update the speed
uint16_t lcd_update = 0;				//Flag to update the LCD content
uint16_t debouncing_en_sw = 0;			//Flag for debouncing (Time select with time 10)
uint16_t debouncing = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
int32_t Encoder_Read(void);
void LCD_Write_Number(int32_t value, int32_t col_pos, int32_t row_pos);
void Motor_Enable(uint16_t invert);
void Motor_Disable(uint16_t invert);
void Motor_Direction(uint16_t direction, uint16_t invert);
void Motor_Speed_RPM(uint16_t speed);
uint16_t Motor_Speed_Update(uint16_t *current_speed, uint16_t *target_speed);
void LCD_Write_Feedrate(int32_t feedrate, int32_t col_pos, int32_t row_pos);
int16_t Switch_Status_Read(void);
void Update_Feedrate(int16_t *feedrate);
int16_t Encoder_Switch_Status_Read(void);
void Motor_Update_Feedrate(int16_t *current_feed, int16_t *target_feed);
uint16_t Motor_Feedrate_Update(int16_t *current_feedrate, int16_t *target_feedrate);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Electronic Leadscrew Feedrate
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
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  LiquidCrystal_I2C(0x4E, 20, 4);	//Initialization of LCD (Select your LCD address)
  lcdBegin();
  lcdSetCursor(2,1);
  lcdPrint("Power Feed V2.0");
  HAL_Delay(2000);
  lcdClear();
  lcd_update = FALSE;				//LCD has been updated

  /* Encoder Initialization */
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  /* Initialization Timers */
  HAL_TIM_Base_Start_IT(&htim11);	//Timer for debouncing
  HAL_TIM_Base_Start_IT(&htim10);	//Timer for acceleration update
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  switch (state)
	  {
	  	  case INITIALIZATION:	//Initialization state
	  		  if ( Switch_Status_Read() != MID ){	//If the switch is not in MID state, report error
	  			  lcdSetCursor(8,1);				//Print Error message
	  			  lcdPrint("ERROR!");
	  			  lcdSetCursor(3,2);
	  			  lcdPrint("Release Switch");
	  		  }else{								//If the Switch is in MID position print the default screen
	  			  lcdClear();
				  lcdSetCursor(0,0);
				  lcdPrint("Feed Rate:");
				  lcdSetCursor(0,1);
				  lcdPrint("Mode: STOP ");
				  LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
				  state = STANDBY;								//Go to standby
	  		  }
	  		  break;
	  	  case STANDBY:		//Standby state (Not movement, switch in the middle)
	  		Update_Feedrate(&target_feedrate);				//Update the feedrate from encoder
	  		display_feedrate = target_feedrate;				//Update variable to display the feedrate
	  		LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
	  		if ( Encoder_Switch_Status_Read() ){			//Check if the encoder is pressed to change the step mode
	  			if (step_mode == STEP_NORMAL){
	  				step_mode = STEP_x10;
	  			}else if(step_mode == STEP_x10){
	  				step_mode = STEP_NORMAL;
	  			}
	  		}
	  		if (lcd_update){				//Update the LCD coming from others states
	  			lcdSetCursor(0,1);
	  			lcdPrint("Mode: STOP ");
	  			lcd_update = FALSE;			//Reset flag for LCD Update
	  		}
	  		if (previous_state != STANDBY){		//If previous status is Standby the enable motor and direction
	  			previous_state = STANDBY;		//Change previous state to current one
	  		}
	  		if ( ( Switch_Status_Read() == RIGHT ) && ( current_feedrate == 0 ) ){		//Check if the switch is on right mode
	  			previous_state = STANDBY;		//Setting previous state to STANDBY
	  			state = MOVE_RIGHT;				//Change state to RIGHT
	  			lcd_update = TRUE;				//Set flag for LCD update
	  		}else if ( ( Switch_Status_Read() == LEFT ) && ( current_feedrate == 0 ) ){	//Check if the switch is on left mode
	  			previous_state = STANDBY;		//Setting previous state to STANDBY
	  			state = MOVE_LEFT;				//Change state to RIGHT
	  			lcd_update = TRUE;				//Set flag for LCD update
	  		}
	  		  break;
	  	  case MOVE_RIGHT:	//Right state, movement to the RIGHT
	  		  sw_status = Switch_Status_Read();		//Read the switch
	  		  if ( sw_status == RIGHT ){			//If it is on right position, update the feedrate target comming from others modes
	  			  if (target_feedrate == 0){
	  				  target_feedrate = display_feedrate;
	  			  }
	  			  Update_Feedrate(&target_feedrate);				//Update the feedrate from encoder
		  		  if (display_feedrate != target_feedrate){			//Check if the feedrate changed to update LCD
		  			  display_feedrate = target_feedrate;
		  			  LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
		  		  }
	  		  }else if ( sw_status == LEFT){		//If it is on left position, change to left, set feedrate to zero
		  		target_feedrate = 0;
			  	if ( current_feedrate == 0 ){		//If motor is stopped then move to left status
			  		state = MOVE_LEFT;				//Change state to left
			  		lcd_update = TRUE;				//Set flag to update display
			  		Motor_Disable(en_invert);		//Disable Motor
			  		target_feedrate = display_feedrate;	//Update feedrate
			  		break;							//Exit this state
			  	}
	  		  }else if ( sw_status == MID ){		//If it is on Mid position, change to STOP or STANDBY status
	  			target_feedrate = 0;
	  			if ( current_feedrate == 0 ){
	  				state = STANDBY;				//Change state to standby
	  				lcd_update = TRUE;				//Set flag to update display
	  				Motor_Disable(en_invert);		//Disable Motor
	  				target_feedrate = display_feedrate;	//Update feedrate
	  				break;							//Exit this state
	  			}
	  		  }
	  		  if (lcd_update){				//Update the LCD coming from others states
	  			  lcdSetCursor(0,1);
	  			  lcdPrint("Mode: RIGHT");
	  			  lcd_update = FALSE;			//Reset flag for LCD Update
	  		  }
	  		  if (previous_state != MOVE_RIGHT){		//If previous status is Standby the enable motor and direcction
				  Motor_Direction(RIGHT, dir_invert);	//Set direction to right
				  Motor_Enable(en_invert);				//Enable Motor
				  previous_state = MOVE_RIGHT;			//Change previous state to current one
	  		  }
	  		  if (update_speed){					//Update speed if the flag is set
	  			  current_feedrate = Motor_Feedrate_Update(&current_feedrate, &target_feedrate);
				  update_speed = 0;					//Reset the update_speed flag
	  		  }
	  		  if ( Encoder_Switch_Status_Read() ){	//Check if the encoder is pressed to change the step mode
	  			  if (step_mode == STEP_NORMAL){
	  				  step_mode = STEP_x10;
	  			  }else if(step_mode == STEP_x10){
	  				  step_mode = STEP_NORMAL;
	  			  }
	  		  }
	  		  break;
	  	  case MOVE_LEFT:
	  		  sw_status = Switch_Status_Read();		//Read the switch
	  		  if ( sw_status == LEFT ){				//If it is on left position, update the feedrate target comming from others modes
	  			  if (target_feedrate == 0){
	  				  target_feedrate = display_feedrate;
	  			  }
	  			  Update_Feedrate(&target_feedrate);				//Update the feedrate from encoder
		  		  if (display_feedrate != target_feedrate){			//Check if the feedrate changed to update LCD
		  			  display_feedrate = target_feedrate;
		  			  LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
		  		  }
	  		  }else if ( sw_status == RIGHT){		//If it is on right position, change to right, set feedrate to zero
		  		target_feedrate = 0;
		  		if ( current_feedrate == 0 ){		//If motor is stopped then move to right status
		  			state = MOVE_RIGHT;				//Change state to right
		  			lcd_update = TRUE;				//Set flag to update display
		  			Motor_Disable(en_invert);		//Disable Motor
		  			target_feedrate = display_feedrate;	//Update feedrate
		  			break;							//Exit this state
		  		}
	  		  }else if ( sw_status == MID ){		//If it is on Mid position, change to STOP or STANDBY status
	  			target_feedrate = 0;
	  			if ( current_feedrate == 0 ){
	  				state = STANDBY;				//Change state to standby
	  				lcd_update = TRUE;				//Set flag to update display
	  				Motor_Disable(en_invert);		//Disable Motor
	  				target_feedrate = display_feedrate;	//Update feedrate
	  				break;							//Exit this state
	  			}
	  		  }
	  		  if (lcd_update){				//Update the LCD comming from others states
	  			lcdSetCursor(0,1);
	  			lcdPrint("Mode: LEFT ");
	  			lcd_update = FALSE;			//Reset flag for LCD Update
	  		  }
	  		  if (previous_state != MOVE_LEFT){		//If previous status is Standby the enable motor and direcction
	  		  Motor_Direction(LEFT, dir_invert);	//Set direction to left
	  		  Motor_Enable(en_invert);				//Enable Motor
	  		  previous_state = MOVE_LEFT;			//Change previous state to current one
	  		  }
	  		  if (update_speed){					//Update speed if the flag is set
	  			  current_feedrate = Motor_Feedrate_Update(&current_feedrate, &target_feedrate);
				  update_speed = 0;					//Reset the update_speed flag
	  		  }
	  		  if ( Encoder_Switch_Status_Read() ){	//Check if the encoder is pressed to change the step mode
	  			  if (step_mode == STEP_NORMAL){
	  				  step_mode = STEP_x10;
	  			  }else if(step_mode == STEP_x10){
	  				  step_mode = STEP_NORMAL;
	  			  }
	  		  }
	  		  break;
	  }
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */
	float TIM10_period_ms = (float)DEBOUNCING_TIME/1000;		//Period to load into the timer, calculated from Define
	uint16_t TIM10_preescaler = 642;							//Preescaler, max 1 second
	uint16_t TIM10_ARR;
	TIM10_ARR = ( (float) (CLK_FREQ_T2/(TIM10_preescaler+1))*TIM10_period_ms );	//Calculation value for ARR register to set correct period
  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 642;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = TIM10_ARR;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
	if (htim == &htim11 ){		//Checking if the IRQ is from Timer11 Acceleration
		update_speed = 1;
	}else if ( htim == &htim10 ){	//Checking if the IRQ is from Timer10 Debouncing
		if (debouncing_en_sw){		//If debouncing enable, increment counter
			debouncing = debouncing + 1;
		}else if (!debouncing_en_sw){	//If debouncing disable reset counter
			debouncing = 0;
		}
	}
}


/**
  * @brief Encoder Steps Read Function
  * @param old_value - Pointer to the value storage as latest encoder value
  * @retval encoder_steps - Number of steps increased or decreased on the encoder
  */
int32_t Encoder_Read(void)
{
	int32_t encoder_steps;
	static int32_t old_value = 0x7FFF;		//Initialisation old_value variable

	if (TIM1->SR & (1 << 0)){		//If overflow or underflow occurs reset the CNT value
		TIM1->SR = ~(1UL << 0);		//Reset UIF bit
		TIM1->CNT = 0x7FFF;			//Reload CNT register to ox7FFF
		old_value = 0x7FFF;
		return 0;					//Return 0
	}

	uint16_t encoder_value = TIM1->CNT;		//Variable to storage the CNT register value
	if ( ( encoder_value - old_value >= 2 ) || ( encoder_value - old_value <= -2 ) ){		//If the value in the encoder register changed (At least 2, to avoid glitches) calculate increment
		encoder_steps = (old_value - encoder_value)/2;	//Divide by 2 is needed due to increments by two on the encoder
		old_value = encoder_value;			//Reload the old_value
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
void LCD_Write_Number(int32_t value, int32_t col_pos, int32_t row_pos)
{
	char str[10];					//Variable to storage the string
	sprintf(str, "%ld", value);		//Convert number to string
	if (value > 0){					//If value is positive
		if (value < 10){			//If value is lower than 10
			lcdSetCursor(col_pos+1,row_pos);
			lcdPrint(" ");
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value < 100){		//If value is lower than 100
			lcdSetCursor(col_pos+2,row_pos);
			lcdPrint(" ");
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value < 1000){	//If value is lower than 1000
			lcdSetCursor(col_pos+3,row_pos);
			lcdPrint(" ");
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}
	}else if (value < 0) {			//If value is negative
		if (value > -10){			//If value is higher than -10
			lcdSetCursor(col_pos+2,row_pos);
			lcdPrint(" ");
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value > -100){	//If value is higher than -100
			lcdSetCursor(col_pos+2,row_pos);
			lcdPrint("  ");
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value > -1000){	//If value is higher than -1000
			lcdSetCursor(col_pos+2,row_pos);
			lcdPrint("   ");
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}
	}else{		// If value is Zero, print 0
		lcdSetCursor(col_pos,row_pos);
		lcdPrint("  ");
		lcdSetCursor(col_pos,row_pos);
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
  * @param 	direction - variable to set the direction of the motor (RIGHT or LEFT)
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
	if ((TIM2->CR1 & (1 << 0)) ^ (1 << 0)){			//Checking if the Timer is already enabled, if not, enable it
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	//Enable Timer2 on PWM Mode
	}
	if (speed > 0){
	ARR_value_temp = ((60 * (float) CLK_FREQ_T2)/(speed*motor_stepsrev));	//Calculation Value to load in ARR
	ARR_value = (uint32_t) ARR_value_temp;	//Uint32 casting
	TIM2->ARR = ARR_value+1;				//Load ARR + 1
	TIM2->CCR1 = (uint32_t) (ARR_value+1)/2;	//Load CCR1 to have always 50% Duty Cycle
	}else{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);	//Disable Timer2 on PWM Mode
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
		HAL_Delay(1);
	}
	return *current_speed;
}

/**
  * @brief Function to update the feedrate of the motor following the acceleration
  * @param 	current_feedrate - Current feedrate
  * 		target_speed - Target feedrate
  * @retval updated_feedrate - Updated feedrate
  */
uint16_t Motor_Feedrate_Update(int16_t *current_feedrate, int16_t *target_feedrate){
	static uint16_t prev_target_feedrate;	//Static Variable to storage the previous target feedrate
	static uint16_t acc_step_increment;		//Static Variable to storage the acceleration step increment
	static uint16_t diff_feedrate;			//Static variable to storage the delta feedrate

	uint16_t acc_step = ceil(ACC_TIME/ACC_UPDATE_RATIO);	//Calculation acc_step with the desired time in ms, and the refresh ratio on acc update
															//This will provide the number of steps we need to do to accelerate on the desired ACC_TIME

	diff_feedrate = *target_feedrate - *current_feedrate;	//Calculation for the Delta feedrate

	if (prev_target_feedrate != *target_feedrate){			//If the storaged feedrate changed since last time
		prev_target_feedrate = *target_feedrate;				//Reload the prev_target_feedrate with latest value
		diff_feedrate = abs(*target_feedrate - *current_feedrate);	//Re-calculate the delta feedrate
		acc_step_increment = ceil(diff_feedrate / acc_step);	//Re-calculate the acceleration step increment, so we take the Delta feedrate and divide it by
																//the numbers of step required to meeting the timing
	}

	if ( *current_feedrate < *target_feedrate){			//If the current feedrate is lower than the target, accelerate
		if (  (acc_step_increment >= diff_feedrate) || (!acc_step_increment) ){	//If the step increment is lower than the delta feedrate then we increase by one to reach the target
																				//Or if the acceleration step increment is zero (If the delta feedrate is lower than the acceleration step)
			*current_feedrate = *current_feedrate + 1;	//Increment by one
		}else{											//If the delta feedrate is bigger than acc_step and the the step incrmenet is lower than the delta feedrate the we increase
			*current_feedrate = *current_feedrate + acc_step_increment;
		}
	}else if (*current_feedrate == *target_feedrate){	//If the target is achieved

	}else if ( *current_feedrate > *target_feedrate ){	//If the current feedrate is higher than the target, decelerate
		if (  (acc_step_increment >= diff_feedrate) || (!acc_step_increment) ){	//If the step increment is lower than the delta feedrate then we increase by one to reach the target
																						//Or if the acceleration step increment is zero (If the delta feedrate is lower than the acceleration step)
			*current_feedrate = *current_feedrate - 1;	//Decrement by one
		}else{											//If the delta feedrate is bigger than acc_step and the the step incrmenet is lower than the delta feedrate the we increase
			*current_feedrate = *current_feedrate - acc_step_increment;
		}
	}

	uint16_t rpm = *current_feedrate/leadscrew_pitch;	//Calculation for RPM with the leadscrew pitch
	Motor_Speed_RPM(rpm);	//Set the motor speed

	return *current_feedrate;
}

/**
  * @brief Function to write number into LCD the feedrate in the desired position
  * @param 	feedrate - feedrate value which expected to be writen into the LCD
  * 		col_pos - column position for the number
  * 		row_pos - raw position for the number
  * @retval
  */
void LCD_Write_Feedrate(int32_t feedrate, int32_t col_pos, int32_t row_pos){
	static int32_t saved_feedrate;
	if ( saved_feedrate != feedrate ){					//Print only if the feedrate changed
		LCD_Write_Number(feedrate,col_pos,row_pos);		//Write the number in the desired position
		lcdPrint("mm/min ");							//Adding mm/min
		saved_feedrate = feedrate;						//Updating Feedrate Saved
	}
}

/**
  * @brief Function to read the value for the switchs
  * @param	- NONE
  * @retval	- Switch Status RIGHT, LEFT, MID, FAIL
  */
int16_t Switch_Status_Read(void){
	int16_t switch_right, switch_left, sw_status;
	switch_right = HAL_GPIO_ReadPin(SW_RIGHT_GPIO_Port, SW_RIGHT_Pin);	//Storage value of RIGHT pin
	switch_left = HAL_GPIO_ReadPin(SW_LEFT_GPIO_Port, SW_LEFT_Pin);		//Storage value of LEFT pin
	if ( !switch_right & !switch_left ){	//If both are enabled at same time, return FAIL
		sw_status = FAIL;
	}else if( switch_right & switch_left ){	//If none are enabled return MID point
		sw_status = MID;
	}else if ( ( !switch_right ) & switch_left ){	//If right is enable return RIGHT
		sw_status = RIGHT;
	}else if ( switch_right & ( !switch_left ) ){	//If left is enable return LEFT
		sw_status = LEFT;
	}
	return sw_status;
}

/**
  * @brief Update Feedrate Function
  * @param old_value - Pointer to the value storaged as latest feedrate value
  * @retval new_feedrate - Updated Feedrate
  */
void Update_Feedrate(int16_t *feedrate){
	if (step_mode == STEP_NORMAL){
		*feedrate += Encoder_Read();	//Update Feedrate
	}else if (step_mode == STEP_x10){
		*feedrate += ( 10 * Encoder_Read());	//Update Feedrate
	}
	if (*feedrate <= 0 ){	//Limit the min value to zero
		*feedrate = 1;
	}else if (*feedrate > MAX_FEEDRATE){
		*feedrate = MAX_FEEDRATE;
	}
}

/**
  * @brief Function to read the value for the encoder switch
  * @param	- NONE
  * @retval	- Encoder Switch Status
  */
int16_t Encoder_Switch_Status_Read(void){
	static uint16_t temp_debouncing = 0;	//Temporal variable to storage the debouncing
	static uint16_t previous_en_sw_status;	//Variable to storage the previous status of the encoder switch
	uint16_t en_sw_status;

	if ( ( !HAL_GPIO_ReadPin(EN_SW_GPIO_Port, EN_SW_Pin) ) && ( !debouncing_en_sw ) ){	//If encoder is pressed and debouncing not enable
		debouncing_en_sw = TRUE;	//Enable debouncing
		temp_debouncing = debouncing;	//Set temporal variable to TRUE
		en_sw_status = FALSE;			//SW status still disable waiting debouncing time
	}else if ( ( !HAL_GPIO_ReadPin(EN_SW_GPIO_Port, EN_SW_Pin) ) && ( temp_debouncing+2 <= debouncing )){ //If encoder still pressed and debouncing +2 already passed
		en_sw_status = FALSE;	//Status still FALSE
		debouncing_en_sw = FALSE;	//Disable debouncing
		previous_en_sw_status = TRUE;	//Set previous status of enable TRUE
	}else if( ( HAL_GPIO_ReadPin(EN_SW_GPIO_Port, EN_SW_Pin) ) && ( previous_en_sw_status ) ){
		en_sw_status = TRUE;	//If button released then send status TRUE
		previous_en_sw_status = FALSE;	//Reset variable of previous status
	}else{
		en_sw_status = FALSE;	//In case other condition, send FALSE
	}
	return en_sw_status;
}

/**
  * @brief Function to update the speed of the motor following the acceleration
  * @param 	current_feed - Current feedrate of the motor
  * 		target_feed - Target feedrate of the motor
  * @retval TBD
  */
void Motor_Update_Feedrate(int16_t *current_feed, int16_t *target_feed){
	uint16_t target_rpm_val, current_rpm_val, new_current_rpm_val;
	target_rpm_val = *target_feed/leadscrew_pitch;	//Conversion Feedrate + Leadscrew pitch to RPM
	current_rpm_val = ceil(*current_feed/leadscrew_pitch);	//Calculation current RPM from feedrate + leadscrew pitch
	new_current_rpm_val = Motor_Speed_Update(&current_rpm_val,&target_rpm_val);	//Update motor speed RPM
	Motor_Speed_RPM(new_current_rpm_val);	//Set the speed in the Motor
	*current_feed = new_current_rpm_val*leadscrew_pitch;	//Re update the feedrate from new RPM updated value
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
