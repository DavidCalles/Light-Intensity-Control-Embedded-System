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
  TIM 1: PWM, interrupt enabled each INTERRUPT_PERIOD microseconds
  TIM 2: ADC trigger, no interrupt, 1ms fixed sampling period
  TIM 3: Encoder, no interrupt
  ADC: triggered by tim2 and moved with DMA

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "HD44780_F4.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	INACTIVE = 0, ACTIVE
}STATE;

typedef struct {
	STATE state;
	uint32_t current;
	uint32_t target;
	uint8_t flag;
}VIRTUAL_TIMER;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// -- Motion sensor pins -- //
#define MOTION_SENSOR_PIN GPIO_PIN_11
#define MOTION_SENSOR_PORT GPIOC

#define MAX_STEPS_CURTAIN 10000

#define WAKEUP_TIME 15 // time in seconds

#define LIGHT_EXTERNAL_RESISTANCE 82000.0F
#define ADC_REFERENCE_VOLTAGE 3.3F

// -- LCD BACKGROUND LIGHT PIN -- //
#define LCD_BACKGROUND_PIN GPIO_PIN_10
#define LCD_BACKGROUND_PORT GPIOA

#define INTERRUPT_PERIOD 50000U // uSeconds
#define PULSE_MULTIPLIER 0.2

#define DC_MOTOR_IN1_PIN GPIO_PIN_1
#define DC_MOTOR_IN2_PIN GPIO_PIN_0
#define DC_MOTOR_ENABLE_PIN GPIO_PIN_8

#define DC_MOTOR_PORT GPIOA

#define FORWARD 1
#define REVERSE -1
#define STOP 0

#define DESIRED_LIGHT_INTENSITY 10000 // Resistance
#define CURTAIN_REFRESH_TIME 3000
#define CURTAIN_MOVEMENT_TIME 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint16_t periodEncoder = 0xFFFF;
uint32_t lightRaw = 0;
float lightResistance = 0;
volatile GPIO_PinState motion = 0;
VIRTUAL_TIMER wakeupTimer;
volatile STATE systemState = ACTIVE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

static int TIM3_Encoder_Init(void);
// -- Virtual timers functions -- //
void VirtualTimer_Init(VIRTUAL_TIMER *);
void VirtualTimer_Increment(VIRTUAL_TIMER *);
void VirtualTimer_Restart(VIRTUAL_TIMER *);
void VirtualTimer_Disable(VIRTUAL_TIMER *);
uint8_t VirtualTimer_Finished(VIRTUAL_TIMER *);

// -- Light intensity measurement functions -- //
float RawADC2Resistance(uint32_t);

// -- Encoder functions -- //
uint16_t Encoder(void);

void SetMotorDirection(int);
void SystemStandby(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  //MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // Timer Initialization
  HAL_TIM_Base_Start(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);
  TIM3_Encoder_Init();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  SetMotorDirection(FORWARD);
  // Circular sampling from 1 channel every 1ms
  HAL_ADC_Start_DMA(&hadc1, &lightRaw, 1);
  VirtualTimer_Init(&wakeupTimer);
  printf("Sensors and UART setup successful! \n\r");

  /*Initialize LCD display*/
  HD44780_Init();
  // Set background ON
  HAL_GPIO_WritePin(LCD_BACKGROUND_PORT, LCD_BACKGROUND_PIN, GPIO_PIN_SET);
  HAL_Delay(1000);
  HD44780_ClrScr();
  HD44780_GotoXY(0,0);
  HD44780_PutStr("Light Intensity");
  HD44780_GotoXY(0,1);
  HD44780_PutStr("Control System");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(CURTAIN_REFRESH_RATE);
	  printf("LIGHT: adc-%ld, R-%.2fk\n\r", lightRaw, lightResistance/1000);
	  printf("MOTION: %d\n\r", (uint16_t)motion);
	  printf("WAKEUP: %d\n\r", (uint16_t)systemState);

	  if(systemState){
		  if(lightResistance < DESIRED_LIGHT_INTENSITY){
			  // Close curtains
			  SetMotorDirection(FORWARD);
			  HD44780_ClrScr();
			HD44780_GotoXY(0,0);
			HD44780_PutStr("Closing");
			HD44780_GotoXY(0,1);
			HD44780_PutStr("Curtains");
		  }
		  else if(lightResistance > DESIRED_LIGHT_INTENSITY){
			  // Open curtains
			  SetMotorDirection(REVERSE);
			  HD44780_ClrScr();
			HD44780_GotoXY(0,0);
			HD44780_PutStr("Opening");
			HD44780_GotoXY(0,1);
			HD44780_PutStr("Curtains");
		  }
		  else{
			  // Leave curtains as they are
			  SetMotorDirection(STOP);
			  HD44780_ClrScr();
			HD44780_GotoXY(0,0);
			HD44780_PutStr("Just");
			HD44780_GotoXY(0,1);
			HD44780_PutStr("there");
		  }
	  }
	  else{
		  SystemStandby();
	  }
	  HAL_Delay(CURTAIN_MOVEMENT_TIME);
	  SetMotorDirection(STOP);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = INTERRUPT_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = INTERRUPT_PERIOD*PULSE_MULTIPLIER; // 50% duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = HAL_RCC_GetPCLK2Freq() / 10000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 LD2_Pin PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static int TIM3_Encoder_Init(void){

HAL_StatusTypeDef rc;
GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Configure these timer pins for ENCODER */
	// Channel 1
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // Channel 2
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Initialize timer for ENCODER*/
  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = periodEncoder;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.RepetitionCounter = 0;
  rc = HAL_TIM_Base_Init(&htim3);
  if (rc != HAL_OK)
  {
	  printf("Failed to initialize Timer 3 Base, “ ”rc=%u\n", rc);
	  return -1;
  }

  TIM_Encoder_InitTypeDef encoderConfig;
  encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  encoderConfig.IC1Polarity = 0;
  encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler = 0;
  encoderConfig.IC1Filter = 3;
  encoderConfig.IC2Polarity = 0;
  encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler = 0;
  encoderConfig.IC2Filter = 3;
  rc = HAL_TIM_Encoder_Init(&htim3, &encoderConfig);
  if (rc != HAL_OK)
  {
	  printf("Failed to initialize Timer 3 Encoder, "
			  "rc=%u\n",
			  rc);
	  return -1;
  }
  rc = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  if (rc != HAL_OK)
  {
	  printf("Failed to start Timer 3 Encoder, "
			  "rc=%u\n",
			  rc);
	  return -1;
  }
  rc = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  if (rc != HAL_OK)
  {
	  printf("Failed to start Timer 3 Encoder, "
			  "rc=%u\n",
			  rc);
	  return -1;
  }
  //HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
  return 0;
}

void VirtualTimer_Init(VIRTUAL_TIMER *vtim){
	vtim ->state = ACTIVE;
	vtim ->target = (uint32_t)WAKEUP_TIME * 1000000 / INTERRUPT_PERIOD;
	vtim ->flag = 0;
	vtim ->current = 0;
}
void VirtualTimer_Increment(VIRTUAL_TIMER *vtim){
	if((vtim ->current) < (vtim ->target)){
		vtim ->current++;
	}
	else{
		vtim ->current = 0;
		vtim ->flag = 1;
	}
}
void VirtualTimer_Restart(VIRTUAL_TIMER *vtim){
	vtim ->current = 0;
	vtim ->flag = 0;
}
uint8_t VirtualTimer_Finished(VIRTUAL_TIMER *vtim){
	return (vtim ->flag);
}
void VirtualTimer_Disable(VIRTUAL_TIMER *vtim){
	vtim ->state = INACTIVE;
	vtim ->target = 0;

	vtim ->current = 0;
}

float RawADC2Resistance(uint32_t rawADC){
	float SensVoltage = rawADC*ADC_REFERENCE_VOLTAGE/4095;
	return ((SensVoltage*LIGHT_EXTERNAL_RESISTANCE)/(ADC_REFERENCE_VOLTAGE-SensVoltage));
}

uint16_t Encoder()
{
  uint16_t currentEnc = __HAL_TIM_GET_COUNTER(&htim3);

  printf("%d\n", currentEnc);

  return currentEnc;
}

void SetMotorDirection(int dir){
	if(dir > 0){
		HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
	}
	else if(dir < 0){
		HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
	}
}

void SystemStandby(void){
	HD44780_ClrScr();
	HD44780_GotoXY(5,0);
	HD44780_PutStr("Standby");
	HD44780_GotoXY(7,1);
	HD44780_PutStr("Mode");

	SetMotorDirection(STOP);
}

// Used to check sampling frequency
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	// Raw adc to sensor resistance conversion
	lightResistance = RawADC2Resistance(lightRaw);
}

/*--------------------------------------------------------------------------
*	Name:			HAL_TIM_PeriodElapsedCallback
*	Description:	Callback when Tim3 interrupt occurs.
*	Parameters:		TIM_HandleTypeDef* htim: timer handler
*	Returns:		void
---------------------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // debugging


	// Sample of motion sensor (always more than a second)
	motion = HAL_GPIO_ReadPin(MOTION_SENSOR_PORT, MOTION_SENSOR_PIN);
	// A second has passed
	VirtualTimer_Increment(&wakeupTimer);
	// Feed the wakeup timer if there is movement
	if (motion){
		VirtualTimer_Restart(&wakeupTimer);
		systemState = ACTIVE;
	} // Stop system if wake up timer is done
	else if(VirtualTimer_Finished(&wakeupTimer)){
		systemState = INACTIVE;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
