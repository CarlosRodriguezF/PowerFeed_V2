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
#include "ee.h"
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
#define BOTH 2						//Definition BOTH to 2
#define NONE 3						//Definition NONE to 3
#define RESTART 10					//Definition to restarting
#define READ	11					//Definition for READ
#define SUBFIXNO 0					//Value to dont print subfix
#define SUBFIXMMMIN 1				//Value to print mm/ subfix
#define TRUE_HOLD 3					//VAriable to check if buttom is holded pressed
#define CLK_FREQ_T2	42000000		//Frequency for the Clock, used on TIM2
#define DEBOUNCING_TIME 100			//TIme for checking debouncing FLAG in ms (MAX 1 Second)
#define TIMER9_PERIOD 100			//TIme for TIMER9 period FLAG in ms (MAX 1 Second)
#define	SW_HOLD_TIME 20				//Multiplier x100 to obtain the time we want to keep the button pressed to being hold pressing
#define HIGH_SPEED_INCREMENT 10		//Increment where Target speed if higher than min speed gap
#define LOW_SPEED_INCREMENT 1		//Increment where Target speed if higher than min speed gap
#define MIN_SPEED_GAP 30			//Difference between target and current speed to use HIGH_SPEED_INCREMENT
#define ACC_UPDATE_RATIO 50			//RAtio for Acceleration update in ms (MAX 1 Second)
#define CHAR_BUFF_SIZE 4			//Buffer for float to char conversion
#define TIM11_preescaler 642		//Preescaler for TIM11, max 1 second
#define ARROW_REFRESH 7 			//Times x100ms for Arrow Refresh (Example if 5 then 5x100ms = 500ms)
#define SCROLLING_TEXT 2			//Times x100ms for Text Scrolling movement (Example if 5 then 5x100ms = 500ms)
//Definition absolute max/min values (Limiting as well the configuration)
#define MAX_FAST_MOVEMENT_FEEDRATE 700	//Maximum feedrate for fas movement
#define MAX_LIMIT_FEEDRATE	700			//Value as limit for feedrate
#define MIN_ACCELERATION_TIME 1000		//Value for the minimum acceleration time
#define MAX_ACCELERATION_TIME 5000		//Value for the maximum acceleration time
#define MIN_ACC_UPDATE_RATIO 20			//Value for the minimum update acceleration ratio
#define MAX_ACC_UPDATE_RATIO 300		//Value for the maximum update acceleration ratio
#define MAX_MOTOR_STEPREV 15000		//Max step/rev for the motor
#define MAX_LEADSCREWPITCH	10		//Mas value for leadscrew pitch

#define STEP_NORMAL 0				//Define to select STEP NORMAL
#define STEP_x10 	1				//Define to select step mode x10


/*Defines for State Machine*/
#define INITIALIZATION	 0			//Initialization state
#define STANDBY			 3			//Standby State no Movement
#define MOVE_RIGHT		 1			//Move right state
#define MOVE_LEFT		 2			//Move left state
#define CONFIGURATION	 4			//Configuration Mode
#define E_STOP_SETTING   5			//E-STOP configuration Mode


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/*Variables for Encoder Read*/
int32_t current_encoder_value = 0x7FFF;			//Variable for Current Encoder Value
int32_t old_encoder_value = 0x7FFF;				//Variable for Previous Encoder Value

/*State Machine variables*/
uint16_t state = INITIALIZATION;				//Variable for the main state machine
uint16_t previous_state = INITIALIZATION;		//Variable for the previous state machine
uint16_t configuration_status = 0;				//Variable for the configuration Menu
uint16_t step_mode = STEP_NORMAL;		//Select mode of increase steps
uint16_t estop_status = NONE;

/* Custom characters*/
uint8_t char_left_arrow[] = {
  0x01,
  0x03,
  0x07,
  0x0F,
  0x0F,
  0x07,
  0x03,
  0x01
};

uint8_t char_left_empty_arrow[] = {
  0x01,
  0x03,
  0x05,
  0x09,
  0x09,
  0x05,
  0x03,
  0x01
};

uint8_t char_right_empty_arrow[] = {
  0x10,
  0x18,
  0x14,
  0x12,
  0x12,
  0x14,
  0x18,
  0x10
};

uint8_t char_right_arrow[] = {
  0x10,
  0x18,
  0x1C,
  0x1E,
  0x1E,
  0x1C,
  0x18,
  0x10
};

//Structure to storage and load the parameters
typedef struct
{
     uint16_t en_invert;
     uint16_t dir_invert;
     uint16_t motor_stepsrev;
     float leadscrew_pitch;
     uint16_t max_feedrate;
     uint16_t fast_movement_feedrate;
     uint16_t acc_time;
     uint16_t acc_update_ratio;
     uint16_t initial_feedrate;
     uint16_t first_load;
}__attribute__((packed, aligned(1))) str_parameters;

str_parameters parameter;
str_parameters default_parameter;
str_parameters *struct_ptr;
str_parameters *default_struct_ptr;

/*General variables*/
int16_t current_feedrate = 0;			//Variable for the current feedrate
int16_t target_feedrate = 200;			//Variable for the target feedrate
int16_t display_feedrate = 200;			//Variable to display the target feedrate into the screen
uint16_t sw_status = 0;					//Variable to storage the  sw status
uint16_t encoder_sw_status = 0;			//Variable to storage the encoder sw status
uint16_t aux_sw_status = 0;				//Variable to storage the aux sw status
uint32_t delay100ms_counter = 0;		//Variable for delay counter 1 = 100ms
uint32_t old_delay100ms_counter = 0;	//Variable to storage the previous delay_counter
uint16_t save_bool = FALSE;				//Variable to save or not the parameters after config
int32_t pulses_counter = 0;				//Variable to storage the pulses
int32_t saved_pulses_counter = 0;		//Variable to storage the pulses and save them
int32_t limit_pulses_counter = 0;		//Variable to storage the limit of pulses
int32_t elimit_pulses_right = 0;		//Variable to storage the limit of right position
int32_t elimit_pulses_left = 0;			//Variable to storage the limit of left position



/* FLAGS*/
uint16_t update_speed = 0;				//Flag to update the speed
uint16_t lcd_update = 0;				//Flag to update the LCD content
uint16_t debouncing_en_sw = 0;			//Flag for debouncing (Time select with time 10)
uint16_t debouncing_aux_sw = 0;			//Flag for aux_switch debouncing (Time select with time 10)
uint16_t debouncing = 0;				//Counter for debouncing
uint16_t aux_debouncing = 0;			//Counter 2 for debouncing aux_switch
uint16_t arrow_flag_counter = 0;		//Counter for the arrow flag
uint16_t arrow_flag = 0;				//Flag to print arrow
uint16_t scrolling_flag = 0;			//Flag for scrolling
uint16_t scrolling_flag_counter = 0;	//Flag for scrolling
uint8_t estop_activated_flag = 0;		//Flag for e-stop activation

/*Messages*/
char eStopText[] = "Activate Power Feed or Keep push to CANCEL - ";
char eStopText_2_Right[] = "Keep push to Set RIGHT Limit - ";
char eStopText_2_Left[] = "Keep push to Set LEFT Limit - ";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM9_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
int32_t Encoder_Read(void);
void LCD_Write_Number(int32_t value, int32_t col_pos, int32_t row_pos, int32_t subfix);
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
int16_t Aux_Switch_Status_Read(void);
void LCD_Write_Float_Number(float value, int32_t col_pos, int32_t row_pos);
static char * _float_to_char(float x, char *p);
void LCD_Write_Float_Number(float float_char, int32_t col_pos_float, int32_t row_pos_float);
uint16_t Save_Parameter_Data(str_parameters*);
uint16_t Read_Parameter_Data(str_parameters*);
void Write_Arrow(uint16_t arrowMode, uint16_t eStop_Mode);
void Write_Text_Scrolling(char*, uint16_t row, uint16_t col, uint16_t enable);
uint32_t Step_Tracking(uint16_t status);


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
	//Initialization Struct
	parameter.en_invert = 0;
	parameter.dir_invert = 0;
	parameter.motor_stepsrev = 0;
	parameter.leadscrew_pitch = 0;
	parameter.max_feedrate = 0;
	parameter.fast_movement_feedrate = 0;
	parameter.acc_time = 0;
	parameter.acc_update_ratio = 0;
	parameter.initial_feedrate = 0;
	parameter.first_load = 0;
	//Default values for parameters
	default_parameter.en_invert = 0;
	default_parameter.dir_invert = 0;
	default_parameter.motor_stepsrev = 1600;
	default_parameter.leadscrew_pitch = 2;
	default_parameter.max_feedrate = 500;
	default_parameter.fast_movement_feedrate = 500;
	default_parameter.acc_time = 1000;
	default_parameter.acc_update_ratio = 50;
	default_parameter.initial_feedrate = 50;
	default_parameter.first_load = 0;
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
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  ee_init();
  LiquidCrystal_I2C(0x4E, 20, 4);	//Initialization of LCD (Select your LCD address)
  lcdBegin();
  lcdSetCursor(2,1);
  lcdPrint("Power Feed V2.0");
  lcd_update = FALSE;				//LCD has been updated
  lcdCreateChar(0, char_left_arrow);
  lcdCreateChar(1, char_left_empty_arrow);
  lcdCreateChar(2, char_right_arrow);
  lcdCreateChar(3, char_right_empty_arrow);

  /* Encoder Initialization */
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  /* Initialization Timers */
  HAL_TIM_Base_Start_IT(&htim11);	//Timer for debouncing
  HAL_TIM_Base_Start_IT(&htim10);	//Timer for acceleration update
  HAL_TIM_Base_Start_IT(&htim9);	//Timer for general 100ms counter

  //Asignation pointers for structures
  struct_ptr = &parameter;
  default_struct_ptr = &default_parameter;

  //Delay to show initial screen and meanwhile check if encoder is pressed long
  old_delay100ms_counter = delay100ms_counter; //Update delay counter
  while(old_delay100ms_counter+30 >= delay100ms_counter){
	  if (Encoder_Switch_Status_Read() == TRUE_HOLD){	//If encoder is hold enter in configuration
		  state = CONFIGURATION;
		  lcd_update=TRUE;
		  break;
	  }else{
		  state = INITIALIZATION;	//If not, enter into Initialization
	  }
  }
  lcdClear();

  //Reading parameters from flash memory, if it is first time booting enter into configuration mode
  if (!Read_Parameter_Data(struct_ptr)){	//Read values from Flash (Emulated EEprom), if return is FALSE, then show error loading and load default values
	  lcdSetCursor(1,1);
	  lcdPrint("ERROR LOADING DATA");
	  lcdSetCursor(1,2);
	  lcdPrint("LOADING DEFAULT");
	  *struct_ptr = *default_struct_ptr;	//Load default values
	  HAL_Delay(2000);
	  lcdClear();
  }
  if (parameter.first_load != 0){	//If it is first booting, load default and go to config.
	  *struct_ptr = *default_struct_ptr;	//Load default values
	  state = CONFIGURATION;		//GO to config for initial configuration
	  lcd_update=TRUE;
  }

  float TIM11_period_ms_init = (float)parameter.acc_update_ratio/1000;		//Period to load into the timer, calculated from Define
  uint16_t TIM11_ARR_init;
  TIM11_ARR_init = ( (float) (CLK_FREQ_T2/(TIM11_preescaler+1))*TIM11_period_ms_init );	//Calculation value for ARR register to set correct period
  TIM11->ARR = TIM11_ARR_init;

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
		  		  target_feedrate = parameter.initial_feedrate;
		  		  display_feedrate = target_feedrate;
				  LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
				  Motor_Disable(parameter.en_invert);			//Disable the Motor at startup.
				  Write_Arrow(STANDBY, estop_status);
				  state = STANDBY;								//Go to standby
	  		  }
	  		  break;
	  	  case STANDBY:		//Standby state (Not movement, switch in the middle)
	  		  /*
  			  if ( previous_state == MOVE_RIGHT ){
  				  if ( estop_status == RIGHT ){
  					estop_status = BOTH;
  					limit_pulses_counter = saved_pulses_counter;
  				  }else if( estop_status == LEFT ){
  					estop_status = BOTH;
  					limit_pulses_counter = saved_pulses_counter;
  				  }
  			  }else if ( previous_state == MOVE_LEFT ){
  			  }*/

	  		  if (lcd_update){				//Update the LCD coming from others states
	  			  lcdSetCursor(0,1);
	  			  lcdPrint("Mode: STOP ");
	  			  lcd_update = FALSE;			//Reset flag for LCD Update
	  			  lcdSetCursor(0,2);
	  			  lcdPrint("                  ");
	  			  Write_Arrow(STANDBY, estop_status);
	  		  }
	  		  Update_Feedrate(&target_feedrate);				//Update the feedrate from encoder
	  		  display_feedrate = target_feedrate;				//Update variable to display the feedrate
	  		  LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
	  		  encoder_sw_status = Encoder_Switch_Status_Read();
	  		  if ( encoder_sw_status == TRUE ){			//Check if the encoder is pressed to change the step mode
	  			  if (step_mode == STEP_NORMAL){
	  				  step_mode = STEP_x10;
	  			  }else if(step_mode == STEP_x10){
	  				step_mode = STEP_NORMAL;
	  			  }
	  		  }else if ( encoder_sw_status == TRUE_HOLD ){	//If Encoder is holded, enter into E-Stop Setting mode
	  			  if ( estop_status == RIGHT ){
	  				estop_status = BOTH;
	  				elimit_pulses_right = saved_pulses_counter;
	  				Write_Arrow(STANDBY, estop_status);
	  			  }else if( estop_status == LEFT ){
	  				estop_status = BOTH;
	  				elimit_pulses_left = saved_pulses_counter;
	  				Write_Arrow(STANDBY, estop_status);
	  			  }else if( estop_status == NONE ){
					previous_state = STANDBY;		//Setting previous state to STANDBY
					state = E_STOP_SETTING;			//Change state to ESTOP Menu
					lcd_update = TRUE;				//Set flag for LCD update
	  			  }else if ( estop_status == BOTH ){
	  				estop_status = NONE;
	  				Write_Arrow(STANDBY, estop_status);
	  			  }
	  		  }
	  		  if (previous_state != STANDBY){		//If previous status is not Standby set previous status as standby
	  			  previous_state = STANDBY;		//Change previous state to current one
	  			  Step_Tracking(DISABLE);
	  		  }

	  		  if ( estop_status == RIGHT ){
				  if (scrolling_flag){
					  Write_Text_Scrolling(eStopText_2_Right, 2, 0, TRUE);
					  scrolling_flag = FALSE;
				  }
	  		  }else if ( estop_status == LEFT ){
				  if (scrolling_flag){
					  Write_Text_Scrolling(eStopText_2_Left, 2, 0, TRUE);
					  scrolling_flag = FALSE;
				  }
	  		  }else{
	  			  Write_Text_Scrolling(eStopText_2_Right, 2, 0, FALSE);
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
	  		  encoder_sw_status = Encoder_Switch_Status_Read();
	  		  aux_sw_status = Aux_Switch_Status_Read();
	  		  if ( encoder_sw_status == TRUE ){	//Check if the encoder is pressed to change the step mode
	  			  if (step_mode == STEP_NORMAL){
	  				  step_mode = STEP_x10;
	  			  }else if(step_mode == STEP_x10){
	  				  step_mode = STEP_NORMAL;
	  			  }
	  		  }
	  		  if (  aux_sw_status == TRUE_HOLD ){
		  		target_feedrate = parameter.fast_movement_feedrate;
	  		  }else if ( aux_sw_status == FALSE ){
		  		target_feedrate = display_feedrate;
	  		  }
	  		  sw_status = Switch_Status_Read();		//Read the switch
	  		  if ( sw_status == RIGHT ){			//If it is on right position, update the feedrate target comming from others modes
	  			  if (target_feedrate == 0){
	  				  target_feedrate = display_feedrate;
	  			  }
	  			  if ( aux_sw_status != TRUE_HOLD ){
					  Update_Feedrate(&target_feedrate);				//Update the feedrate from encoder
					  if (display_feedrate != target_feedrate){			//Check if the feedrate changed to update LCD
						  display_feedrate = target_feedrate;
						  LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
					  }
	  			  }
	  		  }else if ( sw_status == LEFT){		//If it is on left position, change to left, set feedrate to zero
		  		target_feedrate = 0;
			  	if ( current_feedrate == 0 ){		//If motor is stopped then move to left status
			  		state = MOVE_LEFT;				//Change state to left
			  		lcd_update = TRUE;				//Set flag to update display
			  		Motor_Disable(parameter.en_invert);		//Disable Motor
			  		target_feedrate = display_feedrate;	//Update feedrate
			  		pulses_counter = Step_Tracking(DISABLE);
			  		saved_pulses_counter = saved_pulses_counter + pulses_counter;
			  		estop_activated_flag = FALSE;
			  		break;							//Exit this state
			  	}
	  		  }else if ( sw_status == MID ){		//If it is on Mid position, change to STOP or STANDBY status
	  			target_feedrate = 0;
	  			if ( current_feedrate == 0 ){
	  				state = STANDBY;				//Change state to standby
	  				lcd_update = TRUE;				//Set flag to update display
	  				Motor_Disable(parameter.en_invert);		//Disable Motor
	  				target_feedrate = display_feedrate;	//Update feedrate
	  				pulses_counter = Step_Tracking(DISABLE);
	  				saved_pulses_counter = saved_pulses_counter + pulses_counter;
	  				estop_activated_flag = FALSE;
	  				break;							//Exit this state
	  			}
	  		  }

	  		  pulses_counter = Step_Tracking(READ);
	  		  saved_pulses_counter = saved_pulses_counter + pulses_counter;
	  		  if ( estop_status == BOTH ){
				  if ( elimit_pulses_right < 0){
					  if ( saved_pulses_counter <= elimit_pulses_right){
						  target_feedrate = 0;
						  if ( current_feedrate == 0 ){
							  Motor_Disable(parameter.en_invert);		//Disable Motor
							  target_feedrate = display_feedrate;		//Update feedrate
							  estop_activated_flag = TRUE;
							  lcdSetCursor(0,2);
							  lcdPrint("E-STOP Reached");
							  Write_Arrow(STANDBY, estop_status);
						  }
					  }
				  }else if ( elimit_pulses_right >= 0 ){
					  if ( saved_pulses_counter >= elimit_pulses_right){
						  target_feedrate = 0;
						  if ( current_feedrate == 0 ){
							  Motor_Disable(parameter.en_invert);		//Disable Motor
							  target_feedrate = display_feedrate;		//Update feedrate
							  estop_activated_flag = TRUE;
							  lcdSetCursor(0,2);
							  lcdPrint("E-STOP Reached");
							  Write_Arrow(STANDBY, estop_status);
						  }
					  }
				  }
	  		  }

	  		  if (lcd_update){				//Update the LCD coming from others states
	  			  lcdSetCursor(0,1);
	  			  lcdPrint("Mode: RIGHT");
	  			  lcd_update = FALSE;			//Reset flag for LCD Update
	  		  }
	  		  if (previous_state != MOVE_RIGHT){		//If previous status is Standby the enable motor and direcction
				  Motor_Direction(RIGHT, parameter.dir_invert);	//Set direction to right
				  Motor_Enable(parameter.en_invert);				//Enable Motor
				  previous_state = MOVE_RIGHT;			//Change previous state to current one
				  Step_Tracking(ENABLE);
	  		  }
	  		  if (update_speed){					//Update speed if the flag is set
	  			  current_feedrate = Motor_Feedrate_Update(&current_feedrate, &target_feedrate);
				  update_speed = 0;					//Reset the update_speed flag
	  		  }
	  		  if ( ( arrow_flag ) && ( estop_activated_flag == FALSE )){
	  			  Write_Arrow(RIGHT, estop_status);
	  			  arrow_flag = FALSE;
	  		  }
	  		  break;
	  	  case MOVE_LEFT:	//Left state, movement to the Left
	  		  encoder_sw_status = Encoder_Switch_Status_Read();
	  		  aux_sw_status = Aux_Switch_Status_Read();
	  		  if ( encoder_sw_status == TRUE ){	//Check if the encoder is pressed to change the step mode
	  			  if (step_mode == STEP_NORMAL){
	  				  step_mode = STEP_x10;
	  			  }else if(step_mode == STEP_x10){
	  				  step_mode = STEP_NORMAL;
	  			  }
	  		  }
	  		  if (  aux_sw_status == TRUE_HOLD ){
	  			  target_feedrate = parameter.fast_movement_feedrate;
	  		  }else if ( aux_sw_status == FALSE ){
	  			  target_feedrate = display_feedrate;
	  		  }
	  		  sw_status = Switch_Status_Read();		//Read the switch
	  		  if ( sw_status == LEFT ){				//If it is on left position, update the feedrate target comming from others modes
	  			  if (target_feedrate == 0){
	  				  target_feedrate = display_feedrate;
	  			  }
	  			  if ( aux_sw_status != TRUE_HOLD ){
					  Update_Feedrate(&target_feedrate);				//Update the feedrate from encoder
					  if (display_feedrate != target_feedrate){			//Check if the feedrate changed to update LCD
						  display_feedrate = target_feedrate;
						  LCD_Write_Feedrate(display_feedrate, 11, 0);	//Print the default speed
					  }
	  			  }
	  		  }else if ( sw_status == RIGHT){		//If it is on right position, change to right, set feedrate to zero
		  		target_feedrate = 0;
		  		if ( current_feedrate == 0 ){		//If motor is stopped then move to right status
		  			state = MOVE_RIGHT;				//Change state to right
		  			lcd_update = TRUE;				//Set flag to update display
		  			Motor_Disable(parameter.en_invert);		//Disable Motor
		  			target_feedrate = display_feedrate;	//Update feedrate
		  			pulses_counter = Step_Tracking(DISABLE);
		  			saved_pulses_counter = saved_pulses_counter - pulses_counter;
		  			estop_activated_flag = FALSE;
		  			break;							//Exit this state
		  		}
	  		  }else if ( sw_status == MID ){		//If it is on Mid position, change to STOP or STANDBY status
	  			target_feedrate = 0;
	  			if ( current_feedrate == 0 ){
	  				state = STANDBY;				//Change state to standby
	  				lcd_update = TRUE;				//Set flag to update display
	  				Motor_Disable(parameter.en_invert);		//Disable Motor
	  				target_feedrate = display_feedrate;	//Update feedrate
	  				pulses_counter = Step_Tracking(DISABLE);
	  				saved_pulses_counter = saved_pulses_counter - pulses_counter;
	  				estop_activated_flag = FALSE;
	  				break;							//Exit this state
	  			}
	  		  }

	  		  pulses_counter = Step_Tracking(READ);
	  		  saved_pulses_counter = saved_pulses_counter - pulses_counter;
	  		  if ( estop_status == BOTH ){
				  if ( elimit_pulses_left < 0){
					  if ( saved_pulses_counter <= elimit_pulses_left){
						  target_feedrate = 0;
						  if ( current_feedrate == 0 ){
							  Motor_Disable(parameter.en_invert);		//Disable Motor
							  target_feedrate = display_feedrate;		//Update feedrate
							  estop_activated_flag = TRUE;
							  lcdSetCursor(0,2);
							  lcdPrint("E-STOP Reached");
							  Write_Arrow(STANDBY, estop_status);
						  }
					  }
				  }else if ( elimit_pulses_left >= 0 ){
					  if ( saved_pulses_counter <= elimit_pulses_left){
						  target_feedrate = 0;
						  if ( current_feedrate == 0 ){
							  Motor_Disable(parameter.en_invert);		//Disable Motor
							  target_feedrate = display_feedrate;		//Update feedrate
							  estop_activated_flag = TRUE;
							  lcdSetCursor(0,2);
							  lcdPrint("E-STOP Reached");
							  Write_Arrow(STANDBY, estop_status);
						  }
					  }
				  }
	  		  }


	  		  if (lcd_update){				//Update the LCD comming from others states
	  			lcdSetCursor(0,1);
	  			lcdPrint("Mode: LEFT ");
	  			lcd_update = FALSE;			//Reset flag for LCD Update
	  		  }
	  		  if (previous_state != MOVE_LEFT){		//If previous status is Standby the enable motor and direcction
	  		  Motor_Direction(LEFT, parameter.dir_invert);	//Set direction to left
	  		  Motor_Enable(parameter.en_invert);				//Enable Motor
	  		  previous_state = MOVE_LEFT;			//Change previous state to current one
	  		  Step_Tracking(ENABLE);
	  		  }
	  		  if (update_speed){					//Update speed if the flag is set
	  			  current_feedrate = Motor_Feedrate_Update(&current_feedrate, &target_feedrate);
				  update_speed = 0;					//Reset the update_speed flag
	  		  }
	  		  if ( ( arrow_flag ) && ( estop_activated_flag == FALSE )){
	  			  Write_Arrow(LEFT, estop_status);
	  			  arrow_flag = FALSE;
	  		  }
	  		  break;
	  	  case E_STOP_SETTING:
	  		  if (lcd_update){				//Update the LCD coming from others states
	  			  lcdSetCursor(0,1);
	  			  lcdPrint("Mode: ESTOP");
	  			  lcd_update = FALSE;			//Reset flag for LCD Update
	  			  Write_Arrow(STANDBY, estop_status);
	  		  }
	  		  if (scrolling_flag){
	  			Write_Text_Scrolling(eStopText, 2, 0, TRUE);
	  			scrolling_flag = FALSE;
	  		  }
	  		  encoder_sw_status = Encoder_Switch_Status_Read();
	  		  if ( encoder_sw_status == TRUE ){			//Check if the encoder is pressed to change the step mode

	  		  }else if ( encoder_sw_status == TRUE_HOLD ){	//If Encoder is holded, exit E-Stop Setting mode
	  			  previous_state = E_STOP_SETTING;		//Setting previous state to STANDBY
	  			  state = STANDBY;					//Change state to RIGHT
	  			  lcd_update = TRUE;				//Set flag for LCD update
	  			  Write_Text_Scrolling(eStopText, 2, 0, FALSE);	//Remove the test scrolling
	  			  estop_status = NONE;				//Set e-stop status to NONE (Not activated)
	  			  Write_Arrow(STANDBY, estop_status);
	  		  }
	  		  if (previous_state != E_STOP_SETTING){		//If previous status is Standby the enable motor and direction
	  			  previous_state = E_STOP_SETTING;		//Change previous state to current one
	  		  }
	  		  if ( ( Switch_Status_Read() == RIGHT ) && ( current_feedrate == 0 ) ){		//Check if the switch is on right mode
	  			  previous_state = E_STOP_SETTING;		//Setting previous state to E-STOP
	  			  state = MOVE_RIGHT;				//Change state to RIGHT
	  			  estop_status = RIGHT;				//Change e-stop status to RIGHT
	  			  lcd_update = TRUE;				//Set flag for LCD update
	  			  elimit_pulses_left = saved_pulses_counter;	//Set the limit on the left side
	  			  Write_Arrow(STANDBY, estop_status);	//Print arrow with left limit enable
	  			  Write_Text_Scrolling(eStopText, 2, 0, FALSE);	//Remove the test scrolling
	  		  }else if ( ( Switch_Status_Read() == LEFT ) && ( current_feedrate == 0 ) ){	//Check if the switch is on left mode
	  			  previous_state = E_STOP_SETTING;		//Setting previous state to E-STOP
	  			  state = MOVE_LEFT;				//Change state to RIGHT
	  			  estop_status = LEFT;				//Change e-stop status to LEFT
	  			  lcd_update = TRUE;				//Set flag for LCD update
	  			  elimit_pulses_right = saved_pulses_counter;	//Set the limit on the right side
	  			  Write_Arrow(STANDBY, estop_status);	//Print arrow with right limit enable
	  			  Write_Text_Scrolling(eStopText, 2, 0, FALSE);	//Remove the test scrolling
	  		  }
	  		  break;
	  	  case CONFIGURATION:	//State for Configuration Menu
	  		  switch (configuration_status)
	  		  {
				  case 0:	//Enable PIN configuration
					  if (lcd_update){	//Check if is needed to update LCD
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Enable Pin Inverted:");
						  if ( parameter.en_invert == FALSE){		//If parameter is FALSE Print not inverted
							lcdSetCursor(0, 2);
							lcdPrint("NON_INVERTED");
						  }else{									//If parameter is TRUE Print inverted
							lcdSetCursor(0, 2);
							lcdPrint("INVERTED    ");
						  }
						lcd_update = FALSE;	//Reset LCD Update flag
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if (!parameter.en_invert){					//Change the value
							lcdSetCursor(0, 2);
							lcdPrint("INVERTED    ");
							parameter.en_invert = TRUE;
						  }else{
							lcdSetCursor(0, 2);
							lcdPrint("NOT_INVERTED");
							parameter.en_invert = FALSE;
						  }
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 1:	//Dir PIN configuration
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Dir Pin Inverted:");
						  if ( parameter.dir_invert == FALSE){	//If parameter is FALSE Print not inverted
							lcdSetCursor(0, 2);
							lcdPrint("NON_INVERTED");
						  }else{									//If parameter is TRUE Print inverted
							lcdSetCursor(0, 2);
							lcdPrint("INVERTED    ");
						  }
						lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if (!parameter.dir_invert){					//Change the value
							lcdSetCursor(0, 2);
							lcdPrint("INVERTED    ");
							parameter.dir_invert = TRUE;
						  }else{
							lcdSetCursor(0, 2);
							lcdPrint("NOT_INVERTED");
							parameter.dir_invert = FALSE;
						  }
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 2:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Motor Steps per Rev:");
						  LCD_Write_Number(parameter.motor_stepsrev, 0, 2, SUBFIXNO);
						  lcdSetCursor(8, 2);
						  lcdPrint("(pulse/rev)");
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  parameter.motor_stepsrev = parameter.motor_stepsrev + ( 100*(current_encoder_value - old_encoder_value) );
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if ( parameter.motor_stepsrev < 100 || ( parameter.motor_stepsrev >= (MAX_MOTOR_STEPREV + 20000) )){	//Limitation value of MAX_MOTOR_STEPREV
							  parameter.motor_stepsrev = 100;
						  }else if ( parameter.motor_stepsrev > MAX_MOTOR_STEPREV){
							  parameter.motor_stepsrev = MAX_MOTOR_STEPREV;
						  }
						  LCD_Write_Number(parameter.motor_stepsrev, 0, 2, SUBFIXNO);
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 3:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Leadscrew Pitch:");
						  LCD_Write_Float_Number(parameter.leadscrew_pitch,0,2);
						  lcdSetCursor(10, 2);
						  lcdPrint("(mm/rev)");
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  parameter.leadscrew_pitch = parameter.leadscrew_pitch + (float)( 0.01*(current_encoder_value - old_encoder_value) );
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if ( parameter.leadscrew_pitch < 0.01){						//Limitation value of Leadscrewpitch
							  parameter.leadscrew_pitch = 0.01;
						  }else if ( parameter.leadscrew_pitch > MAX_LEADSCREWPITCH){
							  parameter.leadscrew_pitch = MAX_LEADSCREWPITCH;
						  }
						  LCD_Write_Float_Number(parameter.leadscrew_pitch,0,2);
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 4:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Maximum Feedrate:");
						  LCD_Write_Number(parameter.max_feedrate, 0, 2, SUBFIXNO);
						  lcdSetCursor(10, 2);
						  lcdPrint("(mm/min)");
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  parameter.max_feedrate = parameter.max_feedrate + ( 50*(current_encoder_value - old_encoder_value) );
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if ( parameter.max_feedrate < 100 || ( parameter.max_feedrate >= (MAX_LIMIT_FEEDRATE + 20000) ) ){	//Limitation value of max_feedrate
							  parameter.max_feedrate = 100;
						  }else if ( parameter.max_feedrate > MAX_LIMIT_FEEDRATE){
							  parameter.max_feedrate = MAX_LIMIT_FEEDRATE;
						  }
						  LCD_Write_Number(parameter.max_feedrate, 0, 2, SUBFIXNO);
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 5:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Fast Mov Feedrate:");
						  LCD_Write_Number(parameter.fast_movement_feedrate, 0, 2, SUBFIXNO);
						  lcdSetCursor(10, 2);
						  lcdPrint("(mm/min)");
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  parameter.fast_movement_feedrate = parameter.fast_movement_feedrate + ( 50*(current_encoder_value - old_encoder_value) );
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if ( parameter.fast_movement_feedrate < 50 || ( parameter.fast_movement_feedrate >= (MAX_FAST_MOVEMENT_FEEDRATE + 20000) )){	//Limitation value of fastmovement_feedrate
							  parameter.fast_movement_feedrate = 50;
						  }else if ( parameter.fast_movement_feedrate > MAX_FAST_MOVEMENT_FEEDRATE){
							  parameter.fast_movement_feedrate = MAX_FAST_MOVEMENT_FEEDRATE;
						  }
						  LCD_Write_Number(parameter.fast_movement_feedrate, 0, 2, SUBFIXNO);
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 6:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Initial Feedrate:");
						  LCD_Write_Number(parameter.initial_feedrate, 0, 2, SUBFIXNO);
						  lcdSetCursor(10, 2);
						  lcdPrint("(mm/min)");
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  parameter.initial_feedrate = parameter.initial_feedrate + ( 10*(current_encoder_value - old_encoder_value) );
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if ( parameter.initial_feedrate < 10 || ( parameter.initial_feedrate >= (parameter.max_feedrate + 20000) )){	//Limitation value of initial_feedrate
							  parameter.initial_feedrate = 10;
						  }else if ( parameter.initial_feedrate > parameter.max_feedrate){
							  parameter.initial_feedrate = parameter.max_feedrate;
						  }
						  LCD_Write_Number(parameter.initial_feedrate, 0, 2, SUBFIXNO);
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 7:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Acc Time:");
						  LCD_Write_Number(parameter.acc_time, 0, 2, SUBFIXNO);
						  lcdSetCursor(10, 2);
						  lcdPrint("(ms)");
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  parameter.acc_time = parameter.acc_time + ( 100*(current_encoder_value - old_encoder_value) );
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if ( ( parameter.acc_time < MIN_ACCELERATION_TIME ) || ( parameter.acc_time >= (MAX_ACCELERATION_TIME + 20000) )){	//Limitation value of MIN_ACCELERATION_TIME
							  parameter.acc_time = MIN_ACCELERATION_TIME;
						  }else if ( parameter.acc_time > MAX_ACCELERATION_TIME){
							  parameter.acc_time = MAX_ACCELERATION_TIME;
						  }
						  LCD_Write_Number(parameter.acc_time, 0, 2, SUBFIXNO);
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 8:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Acc Update Ratio:");
						  LCD_Write_Number(parameter.acc_update_ratio, 0, 2, SUBFIXNO);
						  lcdSetCursor(10, 2);
						  lcdPrint("(ms)");
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  parameter.acc_update_ratio = parameter.acc_update_ratio + ( 10*(current_encoder_value - old_encoder_value) );
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if ( ( parameter.acc_update_ratio < MIN_ACC_UPDATE_RATIO ) || ( parameter.acc_update_ratio >= (MAX_ACC_UPDATE_RATIO + 20000) )){	//Limitation value of MAX_ACC_UPDATE_RATIO
							  parameter.acc_update_ratio = MIN_ACC_UPDATE_RATIO;
						  }else if ( parameter.acc_update_ratio > MAX_ACC_UPDATE_RATIO){
							  parameter.acc_update_ratio = MAX_ACC_UPDATE_RATIO;
						  }
						  LCD_Write_Number(parameter.acc_update_ratio, 0, 2, SUBFIXNO);
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 9:
					  if (lcd_update){
						  old_encoder_value += Encoder_Read();	//Update Encoder Value
						  lcdClear();
						  lcdSetCursor(3, 0);
						  lcdPrint("CONFIGURATION");
						  lcdSetCursor(0, 1);
						  lcdPrint("Save Configuration");
						  if ( save_bool == FALSE){		//If parameter is FALSE Print exit without save
							lcdSetCursor(0, 2);
							lcdPrint("EXIT WITHOUT SAVE");
						  }else{									//If parameter is TRUE Print Save Parameters
							lcdSetCursor(0, 2);
							lcdPrint("SAVE PARAMETERS  ");
						  }
						  lcd_update = FALSE;
					  }
					  current_encoder_value += Encoder_Read();		//Check if encoder changed
					  if ( current_encoder_value != old_encoder_value){	//If changed, change the value
						  old_encoder_value = current_encoder_value;	//Update Encoder variable
						  if (save_bool){					//Change the value
							lcdSetCursor(0, 2);
							lcdPrint("EXIT WITHOUT SAVE");
							save_bool = FALSE;
						  }else{
							lcdSetCursor(0, 2);
							lcdPrint("SAVE PARAMETERS  ");
							save_bool = TRUE;
						  }
					  }
					  if ( Encoder_Switch_Status_Read() == TRUE){	//If encoder is pressed, continue to next parameter
						configuration_status += 1;
						lcd_update = TRUE;
					  }
					  break;
				  case 10:
					  if (save_bool == TRUE){
						  parameter.first_load = 0;		//Change byte load to 0 to avoid entering again
						  if (Save_Parameter_Data(struct_ptr)){		//Save data into flash memory
							  lcdClear();
							  lcdSetCursor(5,1);
							  lcdPrint("DATA SAVED");
							  HAL_Delay(1500);
						  }else{									//If it fails return ERROR
							  lcdClear();
							  lcdSetCursor(0,1);
							  lcdPrint("ERROR DEFAULT VALUES");
							  lcdSetCursor(7,2);
							  lcdPrint("LOADED");
							  HAL_Delay(1500);
						  }
					  }else{
						  lcdClear();								//If selection was not to save data
						  lcdSetCursor(3, 1);
						  lcdPrint("DATA NOT SAVED");
						  HAL_Delay(2000);
					  }
					  state = INITIALIZATION;
					  break;
				  }
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_9;
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
	//Timer for ENCODER
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
	//Timer for PULSE
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
	//Timer for RPM Counter
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
	//Timer for Pulse Counter
  /* USER CODE END TIM5_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 5;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */
	float TIM9_period_ms = (float)TIMER9_PERIOD/1000;		//Period to load into the timer, calculated from Define
	uint16_t TIM9_preescaler = 642;							//Preescaler, max 1 second
	uint16_t TIM9_ARR;
	TIM9_ARR = ( (float) (CLK_FREQ_T2/(TIM9_preescaler+1))*TIM9_period_ms );	//Calculation value for ARR register to set correct period
  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = TIM9_preescaler;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = TIM9_ARR;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  htim10.Init.Prescaler = TIM10_preescaler;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EXTRA_OUT_GPIO_Port, EXTRA_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : EXTRA_OUT_Pin */
  GPIO_InitStruct.Pin = EXTRA_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EXTRA_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin ENABLE_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENDSTOP_RIGHT_Pin ENDSTOP_LEFT_Pin EXTRA_INPUT_Pin */
  GPIO_InitStruct.Pin = ENDSTOP_RIGHT_Pin|ENDSTOP_LEFT_Pin|EXTRA_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_LEFT_Pin SW_RIGHT_Pin SEC_SW_Pin EN_SW_Pin */
  GPIO_InitStruct.Pin = SW_LEFT_Pin|SW_RIGHT_Pin|SEC_SW_Pin|EN_SW_Pin;
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
		//Debouncing for encoder switch
		if (debouncing_en_sw){		//If debouncing enable, increment counter
			debouncing = debouncing + 1;
		}else if (!debouncing_en_sw){	//If debouncing disable reset counter
			debouncing = 0;
		}
		//Debouncing for auxiliary switch
		if (debouncing_aux_sw){		//If debouncing enable, increment counter
			aux_debouncing = aux_debouncing + 1;
		}else if (!debouncing_aux_sw){	//If debouncing disable reset counter
			aux_debouncing = 0;
		}
	}else if( htim == &htim9 ){
		delay100ms_counter += 1;
		arrow_flag_counter += 1;
		scrolling_flag_counter += 1;
		if ( arrow_flag_counter == ARROW_REFRESH){
			arrow_flag_counter = 0;
			arrow_flag = TRUE;
		}
		if (scrolling_flag_counter == SCROLLING_TEXT){
			scrolling_flag_counter = 0;
			scrolling_flag = TRUE;
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
void LCD_Write_Number(int32_t value, int32_t col_pos, int32_t row_pos, int32_t subfix)
{
	char str[10];					//Variable to storage the string
	sprintf(str, "%ld", value);		//Convert number to string
	if (value > 0){					//If value is positive
		if (value < 10){			//If value is lower than 10
			lcdSetCursor(col_pos+1,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint(" ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value < 100){		//If value is lower than 100
			lcdSetCursor(col_pos+2,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint(" ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value < 1000){	//If value is lower than 1000
			lcdSetCursor(col_pos+3,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint(" ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value < 10000){	//If value is lower than 10000
			lcdSetCursor(col_pos+4,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint(" ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value < 100000){	//If value is lower than 100000
			lcdSetCursor(col_pos+5,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint(" ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}
	}else if (value < 0) {			//If value is negative
		if (value > -10){			//If value is higher than -10
			lcdSetCursor(col_pos+2,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint(" ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value > -100){	//If value is higher than -100
			lcdSetCursor(col_pos+2,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint("   ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}else if (value > -1000){	//If value is higher than -1000
			lcdSetCursor(col_pos+2,row_pos);
			if ( subfix == SUBFIXNO ){
				lcdPrint("   ");
			}else if ( subfix == SUBFIXMMMIN ){
				lcdPrint("mm/min ");
			}
			lcdSetCursor(col_pos,row_pos);
			lcdPrint(str);
		}
	}else{		// If value is Zero, print 0
		lcdSetCursor(col_pos,row_pos);
		if ( subfix == SUBFIXNO ){
			lcdPrint("  ");
		}else if ( subfix == SUBFIXMMMIN ){
			lcdPrint("mm/min");
		}
		lcdSetCursor(col_pos,row_pos);
		lcdPrint("0");
	}
}

/**
  * @brief Function to convert float number to char
  * @param 	x - float value which expected to be converted
  * 		p - char used for the conversion
  * @retval Pointer to the char in which the value will be loaded
  */
static char * _float_to_char(float x, char *p) {
    char *s = p + CHAR_BUFF_SIZE; // go to end of buffer
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    if (x < 0) { // take care of negative numbers
        decimals = (int)(x * -100) % 100; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    } else { // positive numbers
        decimals = (int)(x * 100) % 100;
        units = (int)x;
    }

    *--s = (decimals % 10) + '0';
    decimals /= 10; // repeat for as many decimal places as you need
    *--s = (decimals % 10) + '0';
    *--s = '.';

    while (units > 0) {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    if (x < 0) *--s = '-'; // unary minus sign for negative numbers
    return s;
}

/**
  * @brief Function to write float number on the LCD
  * @param 	float_char - char expected to be writen
  * 		col_pos - column position for the number
  * 		row_pos - raw position for the number
  * @retval
  */
void LCD_Write_Float_Number(float float_char, int32_t col_pos_float, int32_t row_pos_float){
	char float2char[CHAR_BUFF_SIZE+1];
	float2char[CHAR_BUFF_SIZE] = '\0';
	char float1[4];						//Initialization array to save the output
	char *ptr = float1;
	if (float_char < 1 ){				//If float is lower than 1 print an extra 0 on the left
		lcdSetCursor(col_pos_float, row_pos_float);
		lcdPrint("0");
		ptr = _float_to_char(float_char,&float2char[0]);
		lcdPrint(ptr);
	}else{
		ptr = _float_to_char(float_char,&float2char[0]);
		lcdSetCursor(col_pos_float, row_pos_float);
		lcdPrint(ptr);
	}
}


/**
  * @brief Function to Enable EN signal for Motor Driver
  * @param 	invert - variable to invert the EN pin logic
  * @retval
  */
void Motor_Enable(uint16_t invert){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, (GPIO_PIN_SET^invert));	//Enable Motor, XOR with SET to invert it if selected
}

/**
  * @brief Function to Disable EN signal for Motor Driver
  * @param 	invert - variable to invert the EN pin logic
  * @retval
  */
void Motor_Disable(uint16_t invert){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, (GPIO_PIN_RESET^invert));	//Disable Motor, XOR with SET to invert it if selected
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
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (GPIO_PIN_SET^invert));	//Disable Motor, XOR with SET to invert it if selected
	}else if (direction == LEFT){
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (GPIO_PIN_RESET^invert));	//Disable Motor, XOR with SET to invert it if selected
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
	ARR_value_temp = ((60 * (float) CLK_FREQ_T2)/(speed*parameter.motor_stepsrev));	//Calculation Value to load in ARR
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

	uint16_t acc_step = ceil(parameter.acc_time/parameter.acc_update_ratio);	//Calculation acc_step with the desired time in ms, and the refresh ratio on acc update
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

	uint16_t rpm = *current_feedrate/parameter.leadscrew_pitch;	//Calculation for RPM with the leadscrew pitch
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
		LCD_Write_Number(feedrate,col_pos,row_pos, SUBFIXMMMIN);		//Write the number in the desired position
		//lcdPrint("mm/min ");							//Adding mm/min
		saved_feedrate = feedrate;						//Updating Feedrate Saved
	}
}

/**
  * @brief Function to read the value for the switchs
  * @param	- NONE
  * @retval	- Switch Status RIGHT, LEFT, MID, FAIL
  */
int16_t Switch_Status_Read(void){
	int16_t switch_right, switch_left, sw_status = 0;
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
	}else if (*feedrate > parameter.max_feedrate){
		*feedrate = parameter.max_feedrate;
	}
}

/**
  * @brief Function to read the value for the encoder switch
  * @param	- NONE
  * @retval	- Encoder Switch Status TRUE, FALSE or TRUE_HOLD
  */
int16_t Encoder_Switch_Status_Read(void){
	static uint16_t temp_debouncing = 0;	//Temporal variable to storage the debouncing
	static uint16_t previous_en_sw_status;	//Variable to storage the previous status of the encoder switch
	uint16_t en_sw_status = 0;
	uint16_t encoder_sw_read_value;

	encoder_sw_read_value = HAL_GPIO_ReadPin(EN_SW_GPIO_Port, EN_SW_Pin);

	if ( ( !encoder_sw_read_value ) && ( !debouncing_en_sw ) ){	//If encoder is pressed and debouncing not enable
		debouncing_en_sw = TRUE;	//Enable debouncing
		temp_debouncing = debouncing;	//Load value from debouncing
		en_sw_status = FALSE;			//SW status still disable waiting debouncing time
	}else if ( ( !encoder_sw_read_value ) && ( temp_debouncing+2 <= debouncing )){ //If encoder still pressed and debouncing +2 already passed
		en_sw_status = FALSE;	//Status still FALSE
		//debouncing_en_sw = FALSE;	//Disable debouncing
		previous_en_sw_status = TRUE;	//Set previous status of enable TRUE
		if ( temp_debouncing + SW_HOLD_TIME <= debouncing ){	//If we keep the button pressed more than the time defines
			en_sw_status = TRUE_HOLD;
			previous_en_sw_status = TRUE_HOLD;
			temp_debouncing = debouncing;
			debouncing_en_sw = FALSE;	//Disable debouncing
		}
	}else if( ( encoder_sw_read_value )){
		if ( previous_en_sw_status == TRUE_HOLD ){	//If previous status was HOLD< do not report push
			en_sw_status = FALSE;
			debouncing_en_sw = FALSE;	//Enable debouncing
		}else if ( previous_en_sw_status == TRUE ){
			en_sw_status = TRUE;	//If button released then send status TRUE
			debouncing_en_sw = FALSE;	//Disable debouncing
		}else{
			en_sw_status = FALSE;
			debouncing_en_sw = FALSE;	//Disable debouncing
		}
		previous_en_sw_status = FALSE;	//Reset variable of previous status
	}else{
		//en_sw_status = FALSE;	//In case other condition, send FALSE
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
	target_rpm_val = *target_feed/parameter.leadscrew_pitch;	//Conversion Feedrate + Leadscrew pitch to RPM
	current_rpm_val = ceil(*current_feed/parameter.leadscrew_pitch);	//Calculation current RPM from feedrate + leadscrew pitch
	new_current_rpm_val = Motor_Speed_Update(&current_rpm_val,&target_rpm_val);	//Update motor speed RPM
	Motor_Speed_RPM(new_current_rpm_val);	//Set the speed in the Motor
	*current_feed = new_current_rpm_val*parameter.leadscrew_pitch;	//Re update the feedrate from new RPM updated value
}

/**
  * @brief Function to read the value for auxiliary switch
  * @param	- NONE
  * @retval	- Aux Switch Status TRUE, FALSE or TRUE_HOLD
  */
int16_t Aux_Switch_Status_Read(void){
	static uint16_t aux_temp_debouncing = 0;	//Temporal variable to storage the debouncing
	static uint16_t previous_aux_sw_status;	//Variable to storage the previous status of the encoder switch
	uint16_t aux_sw_status = 0;
	uint16_t aux_sw_read_value;

	aux_sw_read_value = HAL_GPIO_ReadPin(SEC_SW_GPIO_Port, SEC_SW_Pin);

	if ( ( !aux_sw_read_value ) && ( !debouncing_aux_sw ) ){	//If encoder is pressed and debouncing not enable
		debouncing_aux_sw = TRUE;	//Enable debouncing
		aux_temp_debouncing = aux_debouncing;	//Load value from debouncing
		aux_sw_status = FALSE;			//SW status still disable waiting debouncing time
	}else if ( ( !aux_sw_read_value ) && ( aux_temp_debouncing + 2 <= aux_debouncing ) && (debouncing_aux_sw) ){ //If encoder still pressed and debouncing +2 already passed
		aux_sw_status = FALSE;	//Status still FALSE
		//debouncing_aux_sw = FALSE;	//Disable debouncing
		previous_aux_sw_status = TRUE;	//Set previous status of enable TRUE
		if ( aux_temp_debouncing + SW_HOLD_TIME <= aux_debouncing ){	//If we keep the button pressed more than the time defines
			aux_sw_status = TRUE_HOLD;
			previous_aux_sw_status = TRUE_HOLD;
		}
	}else if( ( aux_sw_read_value )){
		if ( previous_aux_sw_status == TRUE_HOLD ){	//If previous status was HOLD< do not report push
			aux_sw_status = FALSE;
		}else if ( previous_aux_sw_status == TRUE ){
			aux_sw_status = TRUE;	//If button released then send status TRUE
			debouncing_aux_sw = FALSE;	//Disable debouncing
		}else{
			aux_sw_status = FALSE;
			debouncing_aux_sw = FALSE;	//Disable debouncing
		}
		previous_aux_sw_status = FALSE;	//Reset variable of previous status
	}else{
		//aux_sw_status = FALSE;	//In case other condition, send FALSE
	}
	return aux_sw_status;
}

/**
  * @brief Function to save the data into flash (EEPROM Emulation)
  * @param	- Pointer to the struct to be saved
  * @retval
  */
uint16_t Save_Parameter_Data(str_parameters *struct_ptr){
	  union save_union   //Union to copy the data from Structure and separate values on bytes
	  {
	    str_parameters temp;
	    uint8_t bytes[sizeof(*struct_ptr)];
	  }save_union_par;

	  save_union_par.temp = *struct_ptr; //Copy values from value struct to uniun

	  for (uint8_t i=0; i<sizeof(*struct_ptr); i++)  //Transfers values to Union
	  {
	    ee_writeToRam(i, 1, &save_union_par.bytes[i]);
	  }
	  return ee_commit();
}

/**
  * @brief Function to read the data from flash (EEPROM Emulation)
  * @param	- Pointer to the struct in which the data needs to be loaded
  * @retval
  */
uint16_t Read_Parameter_Data(str_parameters *struct_ptr){
	union save_union   //Union to copy the data from Structure and separate values on bytes
	{
		str_parameters temp;
		uint8_t bytes[sizeof(*struct_ptr)];
	}save_union_par;

	uint16_t status_read;
	uint16_t read_fail = 1;
	uint8_t data;             //Temporal variable to copy each data from EEPROM to Union
	for (uint8_t i=0; i<sizeof(*struct_ptr); i++)  //Transfers values to Union
	{
		status_read = ee_read(i, 1, &data);
		save_union_par.bytes[i] = data;   //Save data on union
		if (!status_read){
			read_fail = 0;
		}
	}
	*struct_ptr = save_union_par.temp;
	return read_fail;
}

/**
* @brief Function to write moviung arrow in the LCD, the function needs to be used together with the arrow counter
* @param - 	arrowMode : Select the Mode of the Arrow, RIGHT, LEFT or STANDBY
* 			eStop_Mode : Value for the mode in which the e-stop is working; RIGHT, LEFT, NONE, BOTH
* @retval
*/
void Write_Arrow(uint16_t arrowMode, uint16_t eStop_Mode){
	static uint16_t arrowCounter = 0;
	static u_int16_t previousarrowMode;

	if ( eStop_Mode == NONE ){		//Check in which e-stop mode are we and write the end arrows accordingly
		lcdSetCursor(12, 1);
		lcdWrite(1);
		lcdSetCursor(19, 1);
		lcdWrite(3);
	}else if ( eStop_Mode == LEFT ){
		lcdSetCursor(12, 1);
		lcdWrite(1);
		lcdSetCursor(19, 1);
		lcdWrite(2);
	}else if ( eStop_Mode == RIGHT ){
		lcdSetCursor(12, 1);
		lcdWrite(0);
		lcdSetCursor(19, 1);
		lcdWrite(3);
	}else if ( eStop_Mode == BOTH ){
		lcdSetCursor(12, 1);
		lcdWrite(0);
		lcdSetCursor(19, 1);
		lcdWrite(2);
	}
	if ( previousarrowMode != arrowMode){	//If previously arrowMode changed, clean the screen
		lcdSetCursor(13, 1);
		lcdPrint("      ");
		arrowCounter = 0;
	}
	if (arrowMode == RIGHT){		//If arrowMode is right, print moving arrow to the right
		if ( arrowCounter == 5 ){	//If the arrow is longer than 4 then write the end
			lcdSetCursor( (13 + arrowCounter ) , 1);
			lcdPrint(">");
		}else if ( arrowCounter == 6 ){	//If the arrow is full size, then clean
			lcdSetCursor(13, 1);
			lcdPrint("      ");
			arrowCounter = 0;
			lcdSetCursor( (13 + arrowCounter ) , 1);
			lcdPrint("-");
		}else if ( arrowCounter < 5 ){	//If the arrow is lower than 5 then print one more line
			lcdSetCursor( (13 + arrowCounter ) , 1);
			lcdPrint("-");
		}
	}else if (arrowMode == LEFT){	//If arrowMode is left, print moving arrow to the left
		if ( arrowCounter == 5 ){	//If the arrow is longer than 4 then write the end
			lcdSetCursor( (18 - arrowCounter ) , 1);
			lcdPrint("<");
		}else if ( arrowCounter == 6 ){	//If the arrow is full size, then clean
			lcdSetCursor(13, 1);
			lcdPrint("      ");
			arrowCounter = 0;
			lcdSetCursor( (18 - arrowCounter ) , 1);
			lcdPrint("-");
		}else if ( arrowCounter < 5 ){	//If the arrow is lower than 5 then print one more line
			lcdSetCursor( (18 - arrowCounter ) , 1);
			lcdPrint("-");
		}
	}else if (arrowMode == STANDBY){

	}
	arrowCounter++;		//Increment the arrow counter
	previousarrowMode = arrowMode;	//Set the previous arrow mode to the current one
}

/**
* @brief Function to write string scrolling, used together with scrolling flag to move the text
* @param - 	inputText : String which wanted to be printer
* 			row : row in which we want to start printing
* 			row : column in which we want to start printing
* 			enable : Variable to enable or disable/clean scrolling text; TRUE or FALSE
* @retval
*/
void Write_Text_Scrolling(char *inputText, uint16_t row, uint16_t col, uint16_t enable){
	uint16_t textSize = strlen(inputText);
	uint16_t availableSize = 19 - col;
	static uint16_t offset;
	static uint16_t scrollingStatus = FALSE;
	uint16_t h = 0;
	char temp_char[2] = {' ', '\0'};		//Need to add NULL character to write only ONE char each time
	char *ptr = inputText;

	if ( (scrollingStatus == FALSE) && (enable == TRUE) ){	//If scrolling was disable but Enable request arrived, print text
		for ( uint16_t i = 0; i < availableSize; i++ ){		//Print the size which fit in the available size
			lcdSetCursor(col + i, row);		//Increase the cursor
			temp_char[0] = *(ptr + i);		//Transfer to the temp variable the character to print, printing one by one
			lcdPrint(temp_char);			//Print each character
		}
		scrollingStatus = TRUE;				//Setting the scrolling status to enable
		offset = 1;							//Initualization offset
	}else if ( (scrollingStatus == TRUE) && (enable == TRUE) ) {	//If scrolling was enable already and request is to keep it enabled, scroll
		for ( uint16_t e = 0; e < availableSize; e++ ){				//Print the text on the available size
			lcdSetCursor(col + e, row);								//Set the cursor
			if (offset >= ( textSize - availableSize ) ){			//If we move the text already all the way
				if ( (e + offset) < textSize){						//Print the text size within the place
					temp_char[0] = *(ptr + (e + offset) );
					lcdPrint(temp_char);
				}else{												//Once finished start printing the starting of the test at the end
					temp_char[0] = *(ptr + h);
					lcdPrint(temp_char);
					h++;
				}
			}else{
				temp_char[0] = *(ptr + (e + offset) );				//If there is still spots available print the char
				lcdPrint(temp_char);
			}
		}
		offset++;		//Increase offset by one
		h = 0;			//Reset the h increment variable
		if (offset == textSize){	//If we go trough all text size, restart offset variable
			offset = 0;
		}
	}else if ( (scrollingStatus == TRUE) && (enable == FALSE) ) {	//If request to stop the scrolling and was previously activated, clean
		scrollingStatus = FALSE;	//Set scrolling status to disabled
		h = 0;	//Reset increment variable
		offset = 0;	//Reset offset
		for ( uint16_t i = 0; i < availableSize; i++ ){	//Print blank character in all the desired locations
			lcdSetCursor(col + i, row);
			lcdPrint(" ");
		}
	}else {

	}

}

/**
* @brief Function to track the steps, read the feedback on number of steps applied to the motor
* @param -	status: Select status; DISABLE, ENABLE, READ
* @retval
*/
uint32_t Step_Tracking(uint16_t status){
	static uint16_t step_tracking_status = DISABLE;
	uint32_t steps;
	static uint32_t previous_steps = 0;

	steps = TIM5->CNT - previous_steps;
	previous_steps = TIM5->CNT;

	if ( ( step_tracking_status == DISABLE ) && ( status == ENABLE ) ){
		TIM5->CNT = 0;	//Re-start counter from zero
		HAL_TIM_Base_Start(&htim5);		//Enable timer for pulse count
		step_tracking_status = ENABLE;
	}else if ( ( step_tracking_status == ENABLE ) && ( status == DISABLE ) ){
		HAL_TIM_Base_Stop(&htim5);		//Disable timer for pulse count
		TIM5->CNT = 0;	//Re-start counter to zero
		step_tracking_status = DISABLE;
	}
	return steps;
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
