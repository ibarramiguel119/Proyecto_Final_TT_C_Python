/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
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

#define VALOR_0 65
#define VALOR_PI 300

#define VALOR_0r 65
#define VALOR_PIr 300

#define ENABLE_PIN GPIO_PIN_3
#define MS0_PIN GPIO_PIN_5
#define MS1_PIN GPIO_PIN_1
#define MS2_PIN GPIO_PIN_3

#define VELOCIDAD 0.5

#define bufersize 1
#define BUFFER_SIZE 256

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint8_t byte;
uint8_t buffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;
uint8_t bufferOverflowFlag = 0; // Bandera para indicar desbordamiento




GPIO_TypeDef* GPIO_PORT_LED = GPIOE;   // Puerto GPIO donde está conectado el LED
uint16_t GPIO_PIN_LED = GPIO_PIN_3;     // Pin GPIO que controla el LED

uint8_t stringToSend[] = "Hola";  // Define la cadena que deseas enviar
uint8_t prueba_1[] = "Flag_1";  // Define la cadena que deseas enviar
uint8_t tx1_buffer[20]="Welcome to stm32\n\r";
uint8_t tx2_buffer[20]="Welcome \n\r";
uint8_t rx1_buffer;
uint8_t received_data;


char q1[BUFFER_SIZE] = {0};
char q2[BUFFER_SIZE] = {0};
char q3[BUFFER_SIZE] = {0};
char q4[BUFFER_SIZE] = {0};
char q5[BUFFER_SIZE] = {0};

volatile uint8_t UASART = 0;


uint32_t radianes_a_valor(float radianes) {
    // Ajusta los radianes negativos a su equivalente positivo en el rango de 0 a 2PI
    if (radianes < 0) {
        radianes +=3.1416;
    }

    // Normaliza el valor de radianes en el rango de 0 a PI
    if (radianes > M_PI) {
        radianes = M_PI;
    }

    return VALOR_0r + (uint32_t)((VALOR_PIr - VALOR_0r) * (radianes / M_PI));
}
uint32_t milimetros_a_pasos(float milimetros) {
    // Calcular el número de pasos necesarios para mover la distancia en milímetros
    float pasos_por_mm = 200.0 / 8.0; // 200 pasos por 8 mm
    return (uint32_t)(fabs(milimetros) * pasos_por_mm);
}
/* USER CODE END PV */


/* USER CODE BEGIN 4 */
volatile uint8_t motor_running = 1;// Variable to control motor state
volatile uint8_t motor_running1 = 1;

volatile int pasos_retroceso = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) {
        motor_running = 0; // Stop the motor when the interrupt occurs
    }

    if (GPIO_Pin == GPIO_PIN_13) {
    	motor_running1 = 0; // Stop the motor when the interrupt occurs
    }
}

int paso_actual_q1 = 0;
int paso_actual_q2 = 4250;
int paso_actual_q3 = 0;

float q1_float;
float q4_float;
int q2_int;
int q3_int;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

void processBuffer(uint8_t *buffer, uint16_t length);
void A4988_Setup();
void mover_motorq1(float radianes);
void motor_control(void);
void motor_control1(void);
void Home (void);
void mover_motorq2_mm(float milimetros);
void mover_motorq3_mm(float milimetros);

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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //HAL_UART_Receive_IT(&huart3, &rx1_buffer, sizeof(rx1_buffer));
  HAL_UART_Receive_IT(&huart1,&byte,bufersize);

  A4988_Setup();
  Home();

    //char q1[]="1.5707";
  	//char q2[]="150";
  	//char q3[]="0";
  	//char q4[]="1.5707";

  	// Conversión de q1 y q4 a float
  	//q1_float = atof(q1);
  	//q4_float = atof(q4);

  	// Conversión de q2 y q3 a int (truncando los valores decimales)
  	//q2_int = (int)atof(q2);
  	//q3_int = (int)atof(q3);

  	 //mover_motorq1(q1_float);
  	 //mover_motorq2_mm(q2_int);
  	 //mover_motorq3_mm(q3_int);
  	 //TIM1->CCR2 = radianes_a_valor(q4_float);

  	 //HAL_Delay(2000);

  	 //strcpy(q1, "2.8274");
  	 //strcpy(q2, "155");

  	 // Conversión de q1 y q4 a float
  	 //q1_float = atof(q1);


  	 // Conversión de q2 y q3 a int (truncando los valores decimales)
  	 //q2_int = (int)atof(q2);


  	 //mover_motorq1(q1_float);
  	// mover_motorq2_mm(q2_int);
  	 //mover_motorq3_mm(q3_int);
  	// TIM1->CCR2 = radianes_a_valor(q4_float);

  	 //HAL_Delay(2000);

  	 //strcpy(q1, "4.084");
  	 //strcpy(q2, "145");


  	 // Conversión de q1 y q4 a float
  	 //q1_float = atof(q1);


  	 // Conversión de q2 y q3 a int (truncando los valores decimales)
  	 //q2_int = (int)atof(q2);

  	 //mover_motorq1(q1_float);
  	 //mover_motorq2_mm(q2_int);
  	 //mover_motorq3_mm(q3_int);
  	 //TIM1->CCR2 = radianes_a_valor(q4_float);

  	 //HAL_Delay(2000);

  	//strcpy(q1, "1.5707");
  	//strcpy(q2, "140");
  	//strcpy(q3, "48");
  	//strcpy(q4, "1.74533");


  	// Conversión de q1 y q4 a float
  	//q1_float = atof(q1);
  	//q4_float = atof(q4);


  	// Conversión de q2 y q3 a int (truncando los valores decimales)
  	//q2_int = (int)atof(q2);
  	//q3_int = (int)atof(q3);

  	//mover_motorq1(q1_float);
  	//mover_motorq2_mm(q2_int);
  	//mover_motorq3_mm(q3_int);
  	//TIM1->CCR2 = radianes_a_valor(q4_float);

  	//HAL_Delay(2000);

  	//strcpy(q1, "3.1415");
  	//strcpy(q2, "145");
  	//strcpy(q3, "95");
  	//strcpy(q4, "1.8326");

  	// Conversión de q1 y q4 a float
  	//q1_float = atof(q1);
  	//q4_float = atof(q4);

  	// Conversión de q2 y q3 a int (truncando los valores decimales)
  	//q2_int = (int)atof(q2);
  	//q3_int = (int)atof(q3);

  	//mover_motorq1(q1_float);
  	//mover_motorq2_mm(q2_int);
  	//mover_motorq3_mm(q3_int);
  	//TIM1->CCR2 = radianes_a_valor(q4_float);

  	//HAL_Delay(2000);

  	//strcpy(q1, "2.82743");
  	//strcpy(q2, "165");
  	//strcpy(q3, "179");
  	//strcpy(q4, "1.91986");

  	// Conversión de q1 y q4 a float
  	//q1_float = atof(q1);
  	//q4_float = atof(q4);

  	// Conversión de q2 y q3 a int (truncando los valores decimales)
//  	q2_int = (int)atof(q2);
//  	q3_int = (int)atof(q3);
//
//  	mover_motorq1(q1_float);
//  	mover_motorq2_mm(q2_int);
//  	mover_motorq3_mm(q3_int);
//  	TIM1->CCR2 = radianes_a_valor(q4_float);
//
//  	HAL_Delay(2000);
//  TIM1->CCR2 = 181;
//  TIM1->CCR4 = 183;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	     //q1_float = atof(q1);
	     q1_float = 1.57;
	     q4_float = 2.2;

	      	// Conversión de q2 y q3 a int (truncando los valores decimales)
	     //q2_int = (int)atof(q2);
	     q2_int =200;
	     q3_int =220;

	     //q1_float = atof(q1);


	      	// Conversión de q2 y q3 a int (truncando los valores decimales)
	     //q2_int = (int)atof(q2);
	     //q3_int = (int)atof(q3);
	     q4_float =1.5;


	     mover_motorq1(q1_float);
	     mover_motorq2_mm(q2_int);
	     mover_motorq3_mm(q3_int);
	     TIM1->CCR2 = radianes_a_valor(q4_float);
	     UASART=1;


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim1.Init.Prescaler = 64;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE5 PE9 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA5
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_LED, GPIO_PIN_SET); // Enciende el LED
        HAL_UART_Transmit(&huart1,&byte,1, 100); // Envía la cadena a través de UART

        // Almacenar el byte recibido en el buffer si no es '>'
        if (byte != 62) // 62 es el código ASCII para '>'
        {

            if (bufferIndex < BUFFER_SIZE)
            {

                buffer[bufferIndex++] = byte;

            }
            else
            {
                // Manejar el caso de desbordamiento del buffer
                bufferOverflowFlag = 1; // Establecer la bandera de desbordamiento
                bufferIndex = 0; // Opcional: restablecer el índice del buffer
            }
        }
        else
        {
            // Aquí puedes manejar el caso cuando se recibe '>'
            // Por ejemplo, procesar el buffer y restablecer bufferIndex
        	 //HAL_UART_Transmit(&huart1, prueba_1, sizeof(prueba_1) - 1, 100);
        	 //HAL_UART_Transmit(&huart1, buffer,bufferIndex, 100);// Envía la cadena a través de UART
            processBuffer(buffer, bufferIndex);
            bufferIndex = 0;
        }

        HAL_UART_Receive_IT(&huart1, &byte, 1);


        // Vuelve a habilitar la recepción por interrupción

    }
}

void processBuffer(uint8_t *buffer, uint16_t length)
{
    if (bufferOverflowFlag)
    {
        // Manejar el desbordamiento del buffer
        // Por ejemplo, enviar un mensaje de error o realizar acciones correctivas
        HAL_UART_Transmit(&huart1, (uint8_t *)"Buffer overflow\n", 16, 100);
        bufferOverflowFlag = 0; // Restablecer la bandera de desbordamiento
        return;
    }

    // Variables para almacenar las partes separadas
//    char q1[BUFFER_SIZE] = {0};
//    char q2[BUFFER_SIZE] = {0};
//    char q3[BUFFER_SIZE] = {0};
//    char q4[BUFFER_SIZE] = {0};

    // Punteros para la división de la cadena
    char *ptr = (char *)buffer;
    char *start = ptr;
    char *end = strchr(start, 'a');

    if (end != NULL)
    {
        strncpy(q1, start, end - start);
        start = end + 1;
        end = strchr(start, 'b');

        if (end != NULL)
        {
            strncpy(q2, start, end - start);
            start = end + 1;
            end = strchr(start, 'c');

            if (end != NULL)
            {
                strncpy(q3, start, end - start);
                start = end + 1;
                end = strchr(start, 'd');

                if(end != NULL){
                	 strncpy(q4, start, end - start);
                	 start = end + 1;
                	 strcpy(q5, start);
                }


            }
        }
    }



    // Enviar cada parte a través de UART para verificar
    HAL_UART_Transmit(&huart1, (uint8_t *)q1, strlen(q1), 100); // 0 puntos desfazados
    //HAL_UART_Transmit(&huart1, (uint8_t *)q2, strlen(q2), 100); // 5 puntos desfazados
    //HAL_UART_Transmit(&huart1, (uint8_t *)q3, strlen(q3), 100); // 2 puntos malos
    //HAL_UART_Transmit(&huart1, (uint8_t *)q4, strlen(q4), 100); // Enviar q4 si hay datos
}



void A4988_Setup() {
    // Configurar pines de modo (MS0, MS1, MS2) para medio paso
    HAL_GPIO_WritePin(GPIOE, MS0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, MS1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, MS2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, ENABLE_PIN, GPIO_PIN_RESET);
}

void Home (void){
	TIM1->CCR4 = 180;
	TIM1->CCR2 =183;
	mover_motorq1(0);
	motor_control();
	motor_control1();
}

void motor_control(void) {
    while (motor_running) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        for (int i = 0; i < 1000 && motor_running; i++) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(VELOCIDAD);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(VELOCIDAD);
        }
        if (!motor_running) break; // Check if motor_running is false

        HAL_Delay(500);
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    for (int i = 0; i < 2500; i++) {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    	HAL_Delay(VELOCIDAD);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	HAL_Delay(VELOCIDAD);

    	paso_actual_q2--;
    }
    HAL_Delay(500);
    motor_running = 1;
}

void motor_control1(void) {
    while (motor_running1) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
        for (int i = 0; i < 1000 && motor_running1; i++) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_Delay(VELOCIDAD);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_Delay(VELOCIDAD);
        }
        HAL_Delay(10);
        if (!motor_running1) break; // Check if motor_running is false
        HAL_Delay(500);

    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    for (int i = 0; i < 70; i++) {
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    	HAL_Delay(VELOCIDAD);
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    	HAL_Delay(VELOCIDAD);
    }
    HAL_Delay(500);

    motor_running1 = 1;
}

void mover_motorq1(float radianes) {
    // Convertir radianes a pasos
    int pasos = (int)((radianes / (2 * M_PI)) * 400);

    // Calcular el nuevo paso deseado
    int nuevo_paso = pasos;

    // Calcular la diferencia de pasos
    int diferencia_pasos = nuevo_paso - paso_actual_q1;

    if (diferencia_pasos > 0) {
        // Movimiento hacia adelante
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        for (int i = 0; i < diferencia_pasos; i++) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_Delay(VELOCIDAD);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_Delay(VELOCIDAD);
        }
    } else if (diferencia_pasos < 0) {
        // Movimiento hacia atrás
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
        for (int i = 0; i < diferencia_pasos; i++) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_Delay(VELOCIDAD);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_Delay(VELOCIDAD);
        }
    }

    // Actualizar el paso actual
    paso_actual_q1 = nuevo_paso;

    HAL_Delay(1000);
}

void mover_motorq2_mm(float milimetros) {
    // Limitar el rango de movimiento entre 0 y 200 mm
	milimetros=milimetros-100;
    if (milimetros < 0) {
        milimetros = 0;
    } else if (milimetros > 170) {
        milimetros = 170;
    }

    // Convertir milímetros a pasos
    uint32_t pasos = milimetros_a_pasos(milimetros);

    // Calcular la diferencia de pasos respecto a la posición actual
    int diferencia_pasos = pasos - paso_actual_q2;

    if (diferencia_pasos != 0) {
        if (diferencia_pasos > 0) {
            // Movimiento hacia adelante
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // Dirección positiva
        } else {
            // Movimiento hacia atrás
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Dirección negativa
            diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
        }

        // Mover el motor la cantidad de pasos necesarios
        for (int i = 0; i < diferencia_pasos; i++) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(VELOCIDAD);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(VELOCIDAD);
        }

        // Actualizar el paso actual
        paso_actual_q2 = pasos;
    }

    HAL_Delay(1000); // Pequeña pausa después de mover el motor
}

void mover_motorq3_mm(float milimetros) {
    // Limitar el rango de movimiento entre 0 y 200 mm
	if (milimetros < 0) milimetros = 0;
	if (milimetros > 400) milimetros = 400;

	// Convertir el punto de referencia de 100 mm a 0 mm para los cálculos
	//milimetros -= 100;

	// Convertir milímetros a pasos
    uint32_t pasos = milimetros_a_pasos(milimetros);

    // Calcular la nueva posición deseada
    int nuevo_paso = pasos;

    // Calcular la diferencia de pasos
    int diferencia_pasos = nuevo_paso - paso_actual_q3;

    if (diferencia_pasos > 0) {
        // Movimiento hacia adelante
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // Dirección positiva
        for (int i = 0; i < diferencia_pasos; i++) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_Delay(VELOCIDAD);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_Delay(VELOCIDAD);
        }
    } else if (diferencia_pasos < 0) {
        // Movimiento hacia atrás
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); // Dirección negativa
        diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
        for (int i = 0; i < diferencia_pasos; i++) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_Delay(VELOCIDAD);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_Delay(VELOCIDAD);
        }
    }

    // Actualizar el paso actual
    paso_actual_q3 = nuevo_paso;

    HAL_Delay(1000);
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
