/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char buffer[50];
uint8_t i, flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void drill();
void stop();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void forward_Bluetooth()
{
	HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, 1);
	HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, 0);
	while(HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin)
			&& HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin)
			&& HAL_GPIO_ReadPin(DRILL_GPIO_Port, DRILL_Pin)
			&& flag==0)
	{
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		  HAL_Delay(1);
	}
}

void backward_Bluetooth()
{
	HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, 1);
	HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, 1);
	while(HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin)
			&& HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin)
			&& HAL_GPIO_ReadPin(DRILL_GPIO_Port, DRILL_Pin)
			&& flag==0
			&& !HAL_GPIO_ReadPin(END_SWITCH_GPIO_Port, END_SWITCH_Pin))
	{
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		  HAL_Delay(1);
	}
}

uint8_t string_compare(char array1[], char array2[], uint16_t lenght)
{
	 uint8_t comVAR=0, i;
	 for(i=0;i<lenght;i++)
	   	{
	   		  if(array1[i]==array2[i])
	   	  		  comVAR++;
	   	  	  else comVAR=0;
	   	}
	 if (comVAR==lenght)
		 	return 1;
	 else 	return 0;
}

void Message_handler()
{
	flag=0;
	i=0;

	if(string_compare(buffer, "forward", strlen("forward")))
	{
		memset(buffer, 0, sizeof(buffer));
		forward_Bluetooth();
	}else
	if(string_compare(buffer, "backward", strlen("backward")))
	{
		memset(buffer, 0, sizeof(buffer));
		backward_Bluetooth();
	}else
	if(string_compare(buffer, "drill", strlen("drill")))
	{
		memset(buffer, 0, sizeof(buffer));
		drill();
	}else
	if(string_compare(buffer, "stop", strlen("stop")))
	{
		memset(buffer, 0, sizeof(buffer));
		stop();
	}
}

void stop()
{
	HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, 0);
	while(!HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin))
		HAL_Delay(10);
	while(!HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
		HAL_Delay(10);
	while(!HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin))
		HAL_Delay(10);
	while(!HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
		HAL_Delay(10);
	while(!HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin))
		HAL_Delay(10);
	while(!HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
		HAL_Delay(10);
}

void backward()
{
	HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, 1);
	HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, 1);
	while(!HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin)
			&& flag==0
		    && HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin)
			&& !HAL_GPIO_ReadPin(END_SWITCH_GPIO_Port, END_SWITCH_Pin))
	{
		for(uint16_t i=0; i<50; i++)
		{
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		  HAL_Delay(1);
		}
	}
}

void forward()
{
	HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, 1);
	HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, 0);
	while(!HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin)
			&& flag==0
			&& HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
	{
		for(uint16_t i=0; i<50; i++)
		{
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		  HAL_Delay(1);
		}
	}
}

void calibrate_axis()
{
	  HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, 0);
	  while(HAL_GPIO_ReadPin(END_SWITCH_GPIO_Port, END_SWITCH_Pin)
			  && flag==0
			  && HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin)
	  	  	  && HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
	  {
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		  HAL_Delay(1);
	  }

	  HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, 1);
	  while(!HAL_GPIO_ReadPin(END_SWITCH_GPIO_Port, END_SWITCH_Pin)
			  && flag==0
			  && HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin)
			  && HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
	  {
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		  HAL_Delay(1);
	  }
}

void drill()
{
	HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, 1);
	calibrate_axis();

	HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, 0);

	for(uint16_t i=0; i<6400; i++)
	{
	  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
	  HAL_Delay(1);
	  if(!HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin)
			  || !HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin)
			  || flag==1)
		  break;
	}


	calibrate_axis();
	HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, 0);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE BEGIN 2 */
  calibrate_axis();

  memset(buffer, 0, sizeof(buffer));
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if(!HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin))
		  forward();

	  if(!HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
		  backward();

	  if(!HAL_GPIO_ReadPin(DRILL_GPIO_Port, DRILL_Pin))
		  drill();

	  if(!HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin) && !HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin))
		  stop();

	  if(flag==1)
		  Message_handler();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIRECTION_Pin|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAIN_MOTOR_GPIO_Port, MAIN_MOTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIRECTION_Pin STEP_Pin */
  GPIO_InitStruct.Pin = DIRECTION_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : END_SWITCH_Pin DRILL_Pin BACKWARD_Pin FORWARD_Pin */
  GPIO_InitStruct.Pin = END_SWITCH_Pin|DRILL_Pin|BACKWARD_Pin|FORWARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MAIN_MOTOR_Pin */
  GPIO_InitStruct.Pin = MAIN_MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MAIN_MOTOR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
