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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motor.h"

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

Motor motors[2];
uint16_t adc_buffer[1] = {0};

#define MOTOR_L			0
#define MOTOR_R			1

#define MOTOR_L_DIR		1.f
#define MOTOR_R_DIR		-1.f

#define SERVO_TIM_COMP_MIN	200
#define SERVO_TIM_COMP_MAX	400

#define LINE_THRESHOLD		400

#define MOTOR_FORWARD_SPEED	0.0005f
#define MOTOR_LINE_KP		0.000001f

uint16_t servo_pos = SERVO_TIM_COMP_MIN;
uint8_t servo_dir = 0;

uint16_t transition_01 = SERVO_TIM_COMP_MIN;
uint16_t transition_10 = SERVO_TIM_COMP_MAX;

uint8_t prev_line = 0;
uint8_t curr_line = 0;

float line_setpoint = 300;
float line_value = 0;
float line_error = 0;

float motor_l_speed = 0.f;
float motor_r_speed = 0.f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void calc() {
	line_value = (transition_01 + transition_10)/2;

	line_error = line_setpoint - line_value;

	motor_l_speed = MOTOR_FORWARD_SPEED - MOTOR_LINE_KP*line_error;
	motor_r_speed = MOTOR_FORWARD_SPEED + MOTOR_LINE_KP*line_error;

	Motor_SetSpeed(&motors[MOTOR_L], motor_l_speed*MOTOR_L_DIR);
	Motor_SetSpeed(&motors[MOTOR_R], motor_r_speed*MOTOR_R_DIR);
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  Motor_Init(&motors[MOTOR_L], &htim3, TIM_CHANNEL_4, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);
  Motor_Init(&motors[MOTOR_R], &htim3, TIM_CHANNEL_3, GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_5);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 1);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1) {

	  if(servo_pos<=SERVO_TIM_COMP_MIN) {
		  servo_dir = 0;

		  /*while(adc_buffer[0]>LINE_THRESHOLD);
		  while(adc_buffer[0]<LINE_THRESHOLD);

		  calc();*/
	  }

	  if(servo_pos>=SERVO_TIM_COMP_MAX) {
		  servo_dir = 1;

		  /*while(adc_buffer[0]>LINE_THRESHOLD);
		  while(adc_buffer[0]<LINE_THRESHOLD);

		  calc();*/
	  }

	  if(servo_dir)
		  servo_pos--;
	  else
		  servo_pos++;

	  servo_pos = (servo_pos>SERVO_TIM_COMP_MAX) ? SERVO_TIM_COMP_MAX : servo_pos;
	  servo_pos = (servo_pos<SERVO_TIM_COMP_MIN) ? SERVO_TIM_COMP_MIN : servo_pos;

	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, servo_pos);

	  curr_line = (adc_buffer[0]<LINE_THRESHOLD);

	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, curr_line);

	  if(!prev_line && curr_line) {
		  transition_01 = servo_pos;
		  //transition_10 = servo_pos;

		  calc();
	  } else if(prev_line && !curr_line) {
		  transition_10 = servo_pos;

		  servo_dir = !servo_dir;

		  calc();
	  }

	  //HAL_Delay(1);

	  prev_line = curr_line;

	  uint32_t val = 15000;
	  while(val--);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

