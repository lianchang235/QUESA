/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INA226.h"
#include <string.h>
#include <stdio.h>
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TELEMETRY_PERIOD_MS 100
#define UP_FRAME_LEN 55
uint8_t rx_buf[23];
uint8_t frame_buf[23];
volatile uint8_t rx_len = 0;
volatile uint8_t frame_received = 0;
float thrust[4] = {0};      
float servo_angle = 90.0f; 
const float duty_table[30] = {
    6.0f, 6.1f, 6.2f, 6.3f, 6.4f, 6.5f, 6.6f, 6.7f, 6.8f, 6.9f,
    7.0f, 7.1f, 7.2f, 7.3f, 7.4f, 7.5f, 7.6f, 7.7f, 7.8f, 7.9f,
    8.0f, 8.1f, 8.2f, 8.3f, 8.4f, 8.5f, 8.6f, 8.7f, 8.8f, 8.9f
};
const float force_table[30] = {
    -780.0f, -689.0f, -605.0f, -510.0f, -440.0f, -354.0f, -290.0f, -225.0f, -174.0f, -118.0f,
    -70.0f, -42.0f, -16.0f, -2.0f, 0.0f, 0.0f, 0.0f, 5.0f, 33.0f, 84.0f,
    154.0f, 238.0f, 333.0f, 448.0f, 569.0f, 700.0f, 826.0f, 982.0f, 1100.0f, 1250.0f
};
uint8_t calc_checksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++){sum += data[i];}
		return sum;
}


void parse_down_frame(uint8_t *frame)
{
    memcpy(thrust, frame + 2, 16);
    memcpy(&servo_angle, frame + 18, 4);
}


float thrust_to_duty(float thrust_n)
{
    float force_g = thrust_n * 102.0f;

    if (force_g<=force_table[0]){return duty_table[0];}
    if (force_g>=force_table[29]){return duty_table[29];}
    for (uint8_t i = 0; i < 30 - 1; i++)
    {			
        if (force_g>=force_table[i]&&force_g<=force_table[i + 1])
        {
            float f1 = force_table[i];
            float f2 = force_table[i + 1];
            float d1 = duty_table[i];
            float d2 = duty_table[i + 1];
            if ((f2 - f1) == 0.0f){return d1;}
            return d1 + (force_g - f1) * (d2 - d1) / (f2 - f1);
        }
    }

    return 7.5f;
}



uint16_t duty_to_pwm_compare(float duty)
{
    uint32_t arr;
    if (duty < 6.0f){duty = 6.0f;}
    if (duty > 8.9f){duty = 8.9f;}
    arr=__HAL_TIM_GET_AUTORELOAD(&htim2);
    return (uint16_t)(duty*(arr + 1)/100.0f);
		
}


void send_up_frame(float duty[4], uint16_t pwm[4])
{
    uint8_t frame[UP_FRAME_LEN];
    uint16_t index = 0;
    frame[index++] = 0xAF;
    frame[index++] = 0xFA;
    memcpy(frame + index,&INA226_Info.current , sizeof(float));
    index += sizeof(float);
    memcpy(frame + index, &INA226_Info.vbus, sizeof(float));
    index += sizeof(float);
    memcpy(frame+index, &INA226_Info.power, sizeof(float));
    index += sizeof(float);
    memcpy(frame+index, thrust, 4 * sizeof(float));
    index += 4 * sizeof(float);
    memcpy(frame + index, duty, 4 * sizeof(float));
    index += 4 * sizeof(float);
		memcpy(frame + index, pwm, 4 * sizeof(uint16_t));
    index += 4 * sizeof(uint16_t);
    frame[index] = calc_checksum(frame, index);
    HAL_UART_Transmit(&huart3, frame, UP_FRAME_LEN, 100);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_len==0&&rx_buf[0]!=0xFA)
        {
            HAL_UART_Receive_IT(&huart3,rx_buf, 1);
            return;
        }

        if (rx_len ==1&&rx_buf[1]!=0xAF)
        {
            rx_len = 0;
            HAL_UART_Receive_IT(&huart3, rx_buf, 1);
            return;
        }

        rx_len++;

        if (rx_len==23)
        {
            if(calc_checksum(rx_buf, 22)== rx_buf[22])
            {
                memcpy(frame_buf,rx_buf,23);
                frame_received=1;
            }

            rx_len=0;
        }

        HAL_UART_Receive_IT(&huart3,rx_buf+rx_len, 1);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	  float duty[4] = {7.5f, 7.5f, 7.5f, 7.5f};
    uint16_t pwm[4] = {1500, 1500, 1500, 1500};
    float demo_angle = 90.0f;
    uint8_t demo_step = 0;
    //float demo_list[] = {-6.0f, -3.0f, 0.0f, 3.0f, 6.0f, 9.0f};
		uint8_t ina226_ret = 0;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); 
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1500);
		HAL_UART_Receive_IT(&huart3, rx_buf, 1);
		ina226_ret = INA226_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    INA226_ReadInfo();
		if (frame_received)
		{
    frame_received = 0;
    parse_down_frame(frame_buf);
		}
    for (int i = 0; i < 4; i++)
    {
        duty[i] = thrust_to_duty(thrust[i]);
        pwm[i] = duty_to_pwm_compare(duty[i]);
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm[0]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm[1]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm[2]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm[3]);
    send_up_frame(duty, pwm);

		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
