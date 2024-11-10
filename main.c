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

#define VALOR_0 65
#define VALOR_PI 295

#define ENABLE_PIN_q1 GPIO_PIN_1
#define MS0_PIN_q1 GPIO_PIN_9
#define MS1_PIN_q1 GPIO_PIN_7
#define MS2_PIN_q1 GPIO_PIN_5
#define STEP_q1 GPIO_PIN_3
#define DIR_q1 GPIO_PIN_6

#define ENABLE_PIN_q2 GPIO_PIN_4
#define MS0_PIN_q2 GPIO_PIN_2
#define MS1_PIN_q2 GPIO_PIN_0
#define MS2_PIN_q2 GPIO_PIN_11
#define STEP_q2 GPIO_PIN_15
#define DIR_q2 GPIO_PIN_11

#define ENABLE_PIN_q3 GPIO_PIN_9
#define MS0_PIN_q3 GPIO_PIN_9
#define MS1_PIN_q3 GPIO_PIN_7
#define MS2_PIN_q3 GPIO_PIN_15
#define STEP_q3 GPIO_PIN_13
#define DIR_q3 GPIO_PIN_11

#define VELOCIDAD 0.5
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



char q1[BUFFER_SIZE] = {0};
char q2[BUFFER_SIZE] = {'1','1','0'};
char q3[BUFFER_SIZE] = {0};
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
void A4988_q1();
void A4988_q2();
void A4988_q3();
void Home (void);
void Home_q2(void);
void Home_q3(void);
void mover_motorq1_rad(float radianes);
void mover_motorq2_mm(float milimetros);
void mover_motorq3_mm(float milimetros);
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
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);
}

uint32_t radianes_a_valor(float radianes) {
    // Ajusta los radianes negativos a su equivalente positivo en el rango de 0 a 2PI
    if (radianes < 0) {
        radianes += M_PI;
    }

    // Normaliza el valor de radianes en el rango de 0 a PI
    if (radianes > M_PI) {
        radianes = M_PI;
    }

    return VALOR_0 + (uint32_t)((VALOR_PI - VALOR_0) * (radianes / M_PI));
}

uint32_t milimetros_a_pasos(float milimetros) {
    // Calcular el número de pasos necesarios para mover la distancia en milímetros
    float pasos_por_mm = 200.0 / 8.0; // 200 pasos por 8 mm
    return (uint32_t)(fabs(milimetros) * pasos_por_mm);
}

volatile uint8_t FC_Home_q2 = 1;// Variable to control motor state
volatile uint8_t FC_Home_q3 = 1;
volatile uint8_t Paro_emergencia = 1;
volatile uint8_t Contador = 0;

volatile int pasos_retroceso = 0;

volatile int bandera = 0;

int paso_actual_q1 = 0;
int paso_actual_q2 = 5350;
int paso_actual_q3 = 0;

float q1_float;
float q4_float;
float q5_float;
int q2_int;
int q3_int;

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



  HAL_UART_Receive_IT(&huart3,&byte,bufersize);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  A4988_q1();
  A4988_q2();
  A4988_q3();
  //Home();

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

	  	  	 sprintf(buffer_1, "Valores de articulaciones q1  Prueba de recepccion:\r\n");
	  	    // Transmite la cadena anterior
	  	     HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);
	  	     // Transmite valores de control de salida
	  	     HAL_UART_Transmit(&huart1, (uint8_t *)q1, strlen(q1), 100);
	  	     HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	  	     sprintf(buffer_1, "Valores de articulaciones q2:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart1, (uint8_t *)q2, strlen(q2), 100);
	  	   	 HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);


	  	     sprintf(buffer_1, "Valores de articulaciones q3:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart1, (uint8_t *)q3, strlen(q3), 100);
	  	   	 HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);


	  	     sprintf(buffer_1, "Valores de articulaciones q4:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart1, (uint8_t *)q4, strlen(q4), 100);
	  	   	 HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	  	     sprintf(buffer_1, "Valores de articulaciones q5:\r\n");
	  	  	  	    // Transmite la cadena anterior
	  	  	 HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

	  	     HAL_UART_Transmit(&huart1, (uint8_t *)q5, strlen(q5), 100);
	  	   	 HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  	   	 q1_float = atof(q1);
	  	   	 q2_float = atof(q2);
	  	   	 q3_float = atof(q3);
	  	   	 q4_float = atof(q4);
	  	   	 q5_float = atof(q5);



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
	  	   	 PID_Update(&pid_q3,q3_float,corrected_length_q3,"q3",errors_q3, times_q3, &index_q3,&control_output_q3,&error_q3);
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
	  	    mover_motorq1_rad(corrected_length_q1);
	  	    HAL_Delay(100);
	  	    mover_motorq2_mm(corrected_length_q2);
	  	    HAL_Delay(100);
	  	    mover_motorq3_mm(corrected_length_q3);
	  	    HAL_Delay(200);

	  	    TIM2->CCR4 = radianes_a_valor(corrected_length_q4); //q4
	  	    HAL_Delay(200);
	  	    TIM2->CCR2 = radianes_a_valor(q5_float); //q5
	  	    HAL_Delay(1000);






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



	        if (is_zero(error_q1) && is_zero(error_q2) && is_zero(error_q3) && is_zero(error_q4) && is_zero(error_q5)) {
	            char singleChar = '1';
	            HAL_UART_Transmit(&huart3, (uint8_t*)&singleChar, 1, HAL_MAX_DELAY);
	        }

	        HAL_Delay(1500); // Retardo de 1 segundo (1000 milisegundos)





  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}





static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
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
  huart3.Init.BaudRate = 230400;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC7 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE9 PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD13 PD15 PD0
                           PD2 PD4 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_7) {
    	FC_Home_q2 = 0;
    }
    if (GPIO_Pin == GPIO_PIN_9) {
    	FC_Home_q3 = 0;
    }
    if (GPIO_Pin == GPIO_PIN_11) { //rojo
    	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
    	HAL_GPIO_WritePin(GPIOE, ENABLE_PIN_q1, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOD, ENABLE_PIN_q2, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_q3, GPIO_PIN_SET);
    	Paro_emergencia = 0;
    }
    if (GPIO_Pin == GPIO_PIN_13) { //verde
    	Paro_emergencia = Paro_emergencia + 1;
    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
    	HAL_GPIO_WritePin(GPIOE, ENABLE_PIN_q1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, ENABLE_PIN_q2, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_q3, GPIO_PIN_RESET);
    	Paro_emergencia = 1;
    }
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
    if (huart->Instance == USART3)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_LED, GPIO_PIN_SET); // Enciende el LED
        sprintf(buffer_1, "Prueba de ejecuccion:\r\n");
       	HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);

       	sprintf(buffer_1, "Byte recibido: %c\r\n", byte); // Imprime el valor en decimal
       	HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);




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
        	// Añadir terminador nulo para imprimir el buffer completo como cadena
        	buffer[bufferIndex] = '\0';

        	// Imprimir el contenido del buffer recibido a través de UART1
        	HAL_UART_Transmit(&huart1, (uint8_t *)"Buffer recibido: ", 17, HAL_MAX_DELAY);
        	HAL_UART_Transmit(&huart1, buffer, bufferIndex, HAL_MAX_DELAY); // Envía el contenido del buffer
        	HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
            processBuffer(buffer, bufferIndex);
            bufferIndex = 0;
        }

        HAL_UART_Receive_IT(&huart3, &byte, 1);





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
    sprintf(buffer_1, "Valores de articulaciones prueba de recepcion de datos q1:\r\n");
   	HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)q1, strlen(q1), 100); // 0 puntos desfazados
    //HAL_UART_Transmit(&huart3, (uint8_t *)q2, strlen(q2), 100); // 5 puntos desfazados
    //HAL_UART_Transmit(&huart1, (uint8_t *)q3, strlen(q3), 100); // 2 puntos malos
    //HAL_UART_Transmit(&huart1, (uint8_t *)q4, strlen(q4), 100); // Enviar q4 si hay datos

}

void A4988_q1(){
	HAL_GPIO_WritePin(GPIOE, ENABLE_PIN_q1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MS0_PIN_q1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MS1_PIN_q1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MS2_PIN_q1, GPIO_PIN_RESET);
}

void A4988_q2(){
	HAL_GPIO_WritePin(GPIOD, ENABLE_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS0_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS1_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS2_PIN_q2, GPIO_PIN_RESET);
}

void A4988_q3(){
	HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS0_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS1_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS2_PIN_q3, GPIO_PIN_RESET);
}

void Home (void){
	Home_q2();
	Home_q3();
	TIM2->CCR2 = radianes_a_valor(M_PI/2); //q5
	TIM2->CCR4 = radianes_a_valor(M_PI/2); //q4
}

void Home_q2(void){
	while(FC_Home_q2){
		HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_RESET);  //Retroceso
		for (int i = 0; i < 100000 && FC_Home_q2; i++) {
			HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
			HAL_Delay(VELOCIDAD);
			HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
			HAL_Delay(VELOCIDAD);
		}
		if (!FC_Home_q2) break;
		HAL_Delay(500);
	}

	HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_SET); //Avance
	for (int i = 0; i < 2500; i++) {
		HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
		HAL_Delay(VELOCIDAD);
		HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
		HAL_Delay(VELOCIDAD);
		paso_actual_q2--;
	}
	HAL_Delay(500);
	FC_Home_q2 = 1;
}

void Home_q3(void){
	while(FC_Home_q3){
		HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_RESET);  //Abajo
		for (int i = 0; i < 100000 && FC_Home_q3; i++) {
			HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
			HAL_Delay(VELOCIDAD);
			HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
			HAL_Delay(VELOCIDAD);
		}
		if (!FC_Home_q3) break;
		HAL_Delay(500);
	}

	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_SET); //Arriba
	for (int i = 0; i < 80; i++) {
		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
		HAL_Delay(VELOCIDAD);
		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
		HAL_Delay(VELOCIDAD);
	}
	HAL_Delay(500);
	FC_Home_q3 = 1;
}

void mover_motorq1_rad(float radianes){

    int pasos = (int)((radianes / (2 * M_PI)) * 400);
    int nuevo_paso = pasos;
    int diferencia_pasos = nuevo_paso - paso_actual_q1;

    if (diferencia_pasos > 0) {
        // Movimiento hacia adelante
    	HAL_GPIO_WritePin(GPIOD, DIR_q1, GPIO_PIN_RESET); //Antihorario
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

//    if(radianes == (2*M_PI))
//    {
//    	radianes = 0;
//    }

    else if (diferencia_pasos < 0) {
        // Movimiento hacia atrás
    	HAL_GPIO_WritePin(GPIOD, DIR_q1, GPIO_PIN_SET); //Horario
    	diferencia_pasos = -diferencia_pasos;
    	for (int i = 0; i < diferencia_pasos ; i++) {
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    paso_actual_q1 = nuevo_paso;
    HAL_Delay(500);
}

void mover_motorq2_mm(float milimetros){

	//milimetros = milimetros - 500;

    if (milimetros < 0) {
        milimetros = 0;
    }
    else if (milimetros > 210) {
        milimetros = 210;
    }

    uint32_t pasos = milimetros_a_pasos(milimetros);
    int diferencia_pasos = pasos - paso_actual_q2;

    if (diferencia_pasos != 0) {
        if (diferencia_pasos > 0) {
        	 HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_RESET); //Retroceso
        }
        else {
        	HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_SET); //Avance
            diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
        }

        for (int i = 0; i < diferencia_pasos; i++) {
        	HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
        	HAL_Delay(VELOCIDAD);
        	HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
        	HAL_Delay(VELOCIDAD);
        }

        paso_actual_q2 = pasos;
    }

    HAL_Delay(500);
}

void mover_motorq3_mm(float milimetros){

	if (milimetros < 0) {
		milimetros = 0;
	}
	else if (milimetros > 215) {
		milimetros = 215 ;
	}

    uint32_t pasos = milimetros_a_pasos(milimetros);
    int nuevo_paso = pasos;
    int diferencia_pasos = nuevo_paso - paso_actual_q3;

    if (diferencia_pasos > 0) {
    	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_SET); //Arriba
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    else if (diferencia_pasos < 0) {
    	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_RESET);  //Abajo
    	diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    paso_actual_q3 = nuevo_paso;
    HAL_Delay(500);
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
