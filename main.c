//USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h> // Incluir librería matemática
#include <stdio.h> // Incluir librería estándar para sprintf
#include <stdlib.h> // Incluir librería estándar para itoa
#include <string.h> // Incluir librería estándar para strlen

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#define MAX_OUTPUT 50
#define BUFFER_SIZE 1000  // Ajusta este tamaño según tus necesidades
#define bufersize 1

#define TOLERANCE 0.001
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


char buffer_1[50];
char buffer_output_px[50];
char buffer_measirement[50];
char buffer_corrected_length_px[50];
char buffer_corrected_length_py[50];
char buffer_corrected_length_pz[50];
char buffer_error[50];



char buffer_q1[50];
char buffer_q2[50];
char buffer_q3[50];



uint8_t byte;
uint8_t buffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;
uint8_t bufferOverflowFlag = 0; // Bandera para indicar desbordamiento




float errors_px[BUFFER_SIZE];
float times_px[BUFFER_SIZE];
float errors_py[BUFFER_SIZE];
float times_py[BUFFER_SIZE];
float errors_pz[BUFFER_SIZE];
float times_pz[BUFFER_SIZE];




float errors_q1[BUFFER_SIZE];
float times_q1[BUFFER_SIZE];
float errors_q2[BUFFER_SIZE];
float times_q2[BUFFER_SIZE];
float errors_q3[BUFFER_SIZE];
float times_q3[BUFFER_SIZE];
float errors_q4[BUFFER_SIZE];
float times_q4[BUFFER_SIZE];
float errors_q5[BUFFER_SIZE];
float times_q5[BUFFER_SIZE];


int index_px = 0;
int index_py = 0;
int index_pz = 0;


int index_q1 = 0;
int index_q2 = 0;
int index_q3 = 0;
int index_q4 = 0;
int index_q5 = 0;

uint8_t byte;
GPIO_TypeDef* GPIO_PORT_LED = GPIOE;   // Puerto GPIO donde está conectado el LED
uint16_t GPIO_PIN_LED = GPIO_PIN_3;     // Pin GPIO que controla el LED



char q1[BUFFER_SIZE] = {'0'};
char q2[BUFFER_SIZE] = {'1','1','2'};
char q3[BUFFER_SIZE] = {'0'};
char q4[BUFFER_SIZE] = {'1','.','5','7','0','7'};
char q5[BUFFER_SIZE] = {'1','.','5','7','0','7'};



float control_output_px;
float control_output_py;
float control_output_pz;


float control_output_q1;
float control_output_q2;
float control_output_q3;
float control_output_q4;
float control_output_q5;



float error_x;
float error_y;
float error_z;


float error_q1;
float error_q2;
float error_q3;
float error_q4;
float error_q5;




float q1_float=0;
float q2_float=0;
float q3_float=0;
float q4_float=0;
float q5_float=0;



char buffer_error[50];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void float_to_string(float value, char* buffer, int precision);
void cinematica_inversa(double px, double py, double pz, double *q1_out, double *q2_out, double *q3_out);
void cinematica_directa(double q1, double q2, double q3, double *px, double *py, double *pz);
void LeerAngulosSensor(double *q1, double *q2, double *q3);
void ControlarMotor(double q1, double q2, double q3);
void processBuffer(uint8_t *buffer, uint16_t length);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


int is_zero(float value) {
    return fabs(value) < TOLERANCE;
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float setpoint;     // Desired or target value
    float integral;     // Integral term accumulation
    float prevError;    // Previous error value (for derivative calculation)
    float prevTime;     // Previous time stamp (for derivative calculation)
} PID_Controller;


void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->prevError = 0;
    pid->prevTime = HAL_GetTick();
}

float clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}




float PID_Update(PID_Controller *pid,float setpoint , float measurement,const char* identifier,float *errors, float *times, int *index,float *output, float *error) {
    float currentTime = HAL_GetTick();
    float dt = (currentTime - pid->prevTime) / 1000; // Convertir a segundos
    *error = setpoint - measurement;

    // Proporcional
    float P_out = pid->Kp * (*error);

    // Integral
    pid->integral += (*error) * dt;
    float I_out = pid->Ki * pid->integral;

    // Derivativo
    float derivative = ((*error) - pid->prevError) / dt;
    float D_out = pid->Kd * derivative;

    // Total Output
    *output = P_out + I_out + D_out;
    *output = clamp(*output, -MAX_OUTPUT, MAX_OUTPUT); // Ajusta MAX_OUTPUT según tu sistema

    // Guardar el error y el tiempo actuales para la próxima iteración
    pid->prevError = (*error)-0.001;
    pid->prevTime = currentTime;



    // Almacenar el error y el tiempo
    if (*index < BUFFER_SIZE) {
        errors[*index] = *error;
        times[*index] = currentTime / 1000.0; // Convertir a segundos
        (*index)++;
    }


    // Convertir error y tiempo a cadena y transmitir por UART
    char buffer_error[20];
    char buffer_time[20];
    float_to_string(*error, buffer_error, 3);
    float_to_string(currentTime / 1000.0, buffer_time, 3);
    sprintf(buffer_1, "Medida error %s: %s, Tiempo: %s\r\n", identifier, buffer_error, buffer_time);

     //Transmite la cadena anterior
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

}

/* USER CODE END 0 */


///Control de la pocision
PID_Controller pid_px;
PID_Controller pid_py;
PID_Controller pid_pz;


//Control de las articulacciones

PID_Controller pid_q1;
PID_Controller pid_q2;
PID_Controller pid_q3;
PID_Controller pid_q4;
PID_Controller pid_q5;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);
  HAL_TIM_Base_Start_IT(&htim2);


  HAL_UART_Receive_IT(&huart1,&byte,bufersize);





///////Valores para el control de posicion PX,PY Y PZ

  float setpoint_px=120;
  float setpoint_py=14;
  float setpoint_pz=50;

  // Inicializa el controlador PID
  PID_Init(&pid_px, 1, 0, 0.0011, setpoint_px);
  PID_Init(&pid_py, 1, 0, 0.0011, setpoint_py);
  PID_Init(&pid_pz, 1, 0, 0.0011, setpoint_pz);


///////Valore para el control de Articulaccion q1 , q2, q3 y q4

  double q1_1, q2_1, q3_1;
  double new_px=0,new_py=0, new_pz=0;


  float q1_float1 =120;////Solo prueba no tiene efecto q1_float solo se puedo para no borrar la declaracion de la estructura
  float q2_float1 =100;
  float q3_float1 =40;
  float q4_float1 =15;
  float q5_float1 =30;

  PID_Init(&pid_q1, 1, 0, 0.0011,q1_float1);
  PID_Init(&pid_q2, 1, 0, 0.0011,q2_float1);
  PID_Init(&pid_q3, 1, 0, 0.0011,q3_float1);
  PID_Init(&pid_q4, 1, 0, 0.0011,q4_float1);
  PID_Init(&pid_q5, 1, 0, 0.0011,q5_float1);


  float corrected_length_q1=0;
  float corrected_length_q2=0;
  float corrected_length_q3=0;
  float corrected_length_q4=0;
  float corrected_length_q5=0;


  /* USER CODE BEGIN WHILE */
  while (1){

	  	  	 sprintf(buffer_1, "Valores de articulaciones q1:\r\n");
	  	    // Transmite la cadena anterior
	  	     HAL_UART_Transmit(&huart3, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);
	  	        // Transmite valores de control de salida
	  	     HAL_UART_Transmit(&huart3, (uint8_t *)q1, strlen(q1), 100);
	  	     HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	  	     sprintf(buffer_1, "Valores de articulaciones q2:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart3, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart3, (uint8_t *)q2, strlen(q2), 100);
	  	   	 HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);


	  	     sprintf(buffer_1, "Valores de articulaciones q3:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart3, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart3, (uint8_t *)q3, strlen(q3), 100);
	  	   	 HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);


	  	     sprintf(buffer_1, "Valores de articulaciones q4:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart3, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart3, (uint8_t *)q4, strlen(q4), 100);
	  	   	 HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	  	     sprintf(buffer_1, "Valores de articulaciones q5:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart3, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart3, (uint8_t *)q5, strlen(q5), 100);
	  	   	 HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  	   	 q1_float = atof(q1);
	  	   	 q2_float = atof(q2);
	  	   	 q3_float = atof(q3);
	  	   	 q4_float = atof(q4);
	  	   	 //q5_float = atof(q5);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////CONTROL DE ARTICULACION q1
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  	     //q1_float = atof(q1);
	  	   	 PID_Update(&pid_q1,q1_float,corrected_length_q1,"q1",errors_q1, times_q1, &index_q1,&control_output_q1,&error_q1);
	  	   	 corrected_length_q1 = corrected_length_q1 + control_output_q1;
	  	   	 // Simulación de un proceso que cambia la longitud medida (por ejemplo, movimiento de un actuador)
	  	     //q1_float += 0.1; // Simulación de cambio en la longitud medida

/////////////CONTROL DE ARTICULACION q2
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  	   	 //q2_float = atof(q2);
	  	   	 PID_Update(&pid_q2,q2_float,corrected_length_q2,"q2",errors_q2, times_q2, &index_q2,&control_output_q2,&error_q2);
	  	   	 corrected_length_q2 = corrected_length_q2 + control_output_q2;
	  	   	 // Simulación de un proceso que cambia la longitud medida (por ejemplo, movimiento de un actuador)
	  	   	 q2_float += 0.1; // Simulación de cambio en la longitud medida

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////CONTROL DE ARTICULACION q3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  	   	 //q3_float = atof(q3);
	  	   	 PID_Update(&pid_q3,q2_float,corrected_length_q3,"q3",errors_q3, times_q3, &index_q3,&control_output_q3,&error_q3);
	  	   	 corrected_length_q3 = corrected_length_q3 + control_output_q3;
	  	   	 q3_float += 0.1; // Simulación de cambio en la longitud medida


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////CONTROL DE ARTICULACION q4
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  	   	 //q4_float = atof(q4);
	  	   	 PID_Update(&pid_q4,q4_float,corrected_length_q4,"q4",errors_q4, times_q4, &index_q4,&control_output_q4,&error_q4);
	  	   	 corrected_length_q4 = corrected_length_q4 + control_output_q4;
	  	   	 // Simulación de un proceso que cambia la longitud medida (por ejemplo, movimiento de un actuador)
	  	   	 q4_float += 0.1; // Simulación de cambio en la longitud medida


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////CONTROL DE ARTICULACION q5
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  	   	 //q5_float = atof(q5);
	  	   	 //PID_Update(&pid_q5,q5_float,corrected_length_q5,"q5",errors_q5, times_q5, &index_q5,&control_output_q5,&error_q5);
	  	   	 //corrected_length_q5 = corrected_length_q5 + control_output_q5;
	  	   	 // Simulación de un proceso que cambia la longitud medida (por ejemplo, movimiento de un actuador)
	  	   	 //q5_float += 0.1; // Simulación de cambio en la longitud medida

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	  	   	 PID_Update(&pid_px,120, new_px,"px",errors_px, times_px, &index_px,&control_output_px,&error_x);
	        float corrected_length_px =  new_px + control_output_px;
	        // Simulación de un proceso que cambia la longitud medida (por ejemplo, movimiento de un actuador)
	        new_px += 0.1; // Simulación de cambio en la longitud medida

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	        PID_Update(&pid_py,50, new_py,"py",errors_py, times_py, &index_py,&control_output_py,&error_y);

	        float corrected_length_py = new_py + control_output_py;
	        // Simulación de un proceso que cambia la longitud medida (por ejemplo, movimiento de un actuador)
	        new_py += 0.1; // Simulación de cambio en la longitud medida



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	        PID_Update(&pid_pz,10,new_pz,"pz",errors_pz, times_pz, &index_pz,&control_output_pz,&error_z);

	        float corrected_length_pz = new_pz + control_output_pz;
	        	        // Simulación de un proceso que cambia la longitud medida (por ejemplo, movimiento de un actuador)
	        new_pz += 0.1; // Simulación de cambio en la longitud medida





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	        cinematica_inversa(corrected_length_px, corrected_length_py,corrected_length_pz, &q1_1, &q2_1, &q3_1);

	        cinematica_directa(q1_1, q2_1, q3_1, &new_px, &new_py, &new_pz);



	        if (is_zero(error_q1) && is_zero(error_q2) && is_zero(error_q3) && is_zero(error_q4)) {
	            char singleChar = '1';
	            HAL_UART_Transmit(&huart1, (uint8_t*)&singleChar, 1, HAL_MAX_DELAY);
	        }

	        HAL_Delay(1500); // Retardo de 1 segundo (1000 milisegundos)





  }

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
  RCC_OscInitStruct.HSICalibrationValue = 64;
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
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void) {
    // Enable the TIM2 clock
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Timer clock configuration structure
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};

    // Timer master configuration structure
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    // Set the instance of TIM2
    htim2.Instance = TIM2;

    // Configure the timer parameters
    htim2.Init.Prescaler = 8399;  // Adjust the prescaler according to your clock frequency
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999;     // Adjust the period to achieve the desired frequency (100ms in this case)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // Initialize the TIM2 base unit
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();  // Error handling if initialization fails
    }

    // Configure the clock source
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();  // Error handling if clock configuration fails
    }

    // Configure the master mode (triggering and synchronization)
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();  // Error handling if master configuration fails
    }
}





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



static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}



void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}




void float_to_string(float value, char* buffer, int precision) {
    int int_part = (int)value;
    float frac_part = value - int_part;
    int frac_part_int = (int)(frac_part * pow(10, precision));

    if (value < 0) {
        *buffer++ = '-';
        int_part = -int_part;
        frac_part_int = -frac_part_int;
    }

    itoa(int_part, buffer, 10);
    while (*buffer != '\0') buffer++;
    *buffer++ = '.';
    itoa(frac_part_int, buffer, 10);
}



//Funciones para calcular las cinematicas
void cinematica_inversa(double px, double py, double pz, double *q1_out, double *q2_out, double *q3_out){
	 // Calcular q1
	*q1_out = atan(-px / py) + M_PI;
    // Calcular q2
    // Calcular q2
    *q2_out = px * sin(*q1_out) - py * cos(*q1_out);

    // Asignar pz al parámetro de salida q3
    *q3_out = pz;
}

void cinematica_directa(double q1, double q2, double q3, double *px, double *py, double *pz){
	//calcualar px
	*px=q2*sin(q1);

	//calcular py
	*py=-q2*cos(q1);

	// calcualr pz

	*pz=q3;

}




void ControlarMotor(double q1, double q2, double q3) {

	char buffer_1[50];

	sprintf(buffer_1, "Moviendo motores :\r\n");
	//Tranmite la cadena anterior
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

}


/////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_LED, GPIO_PIN_SET); // Enciende el LED
        //HAL_UART_Transmit(&huart3,&byte,1, 100); // Envía la cadena a través de UART



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
        HAL_UART_Transmit(&huart3, (uint8_t *)"Buffer overflow\n", 16, 100);
        bufferOverflowFlag = 0; // Restablecer la bandera de desbordamiento
        return;
    }


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
    //HAL_UART_Transmit(&huart3, (uint8_t *)q1, strlen(q1), 100); // 0 puntos desfazados
    //HAL_UART_Transmit(&huart3, (uint8_t *)q2, strlen(q2), 100); // 5 puntos desfazados
    //HAL_UART_Transmit(&huart1, (uint8_t *)q3, strlen(q3), 100); // 2 puntos malos
    //HAL_UART_Transmit(&huart1, (uint8_t *)q4, strlen(q4), 100); // Enviar q4 si hay datos

}


////



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
