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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ws2812b_multi_strip_driver.h"
#include "led_matrix.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint8_t LED_strips[MAX_SUPPORTED_NUM_OF_STRIPS][MAX_SUPPORTED_LEDS_IN_STRIP][NUM_OF_CFG_BYTES_PER_LED];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void SnakeTask(void)
{
	volatile int led_id, strip_id;

		uint32_t cycle_cntr=0;
		LD2_GPIO_Port->ODR |= LD2_Pin;
		for (;;)
		{
			for (strip_id=0; strip_id < MAX_ACTIVE_STRIPS; strip_id++)
			{
				for (led_id=(MAX_LEDS_IN_STRIP-1); led_id!=0; led_id--)
				{
					LED_strips[strip_id][led_id][0] = LED_strips[0][led_id-1][0];
					LED_strips[strip_id][led_id][1] = LED_strips[0][led_id-1][1];
					LED_strips[strip_id][led_id][2] = LED_strips[0][led_id-1][2];
				}
			}
			if ((cycle_cntr%15)<8)
			{
				uint8_t power = ((cycle_cntr%15 == 0) || (cycle_cntr%15 == 7)) ? 1 :
								((cycle_cntr%15 == 1) || (cycle_cntr%15 == 6)) ? 5 : 50;
				for (strip_id=0; strip_id < MAX_ACTIVE_STRIPS; strip_id++)
				{
					if ((cycle_cntr%45) < 15)
					{
						LED_strips[led_id][0][GREEN] = power;
						LED_strips[led_id][0][RED]   = 0;
						LED_strips[led_id][0][BLUE]  = 0;
					}
					else if ((cycle_cntr%45) < 30)
					{
						LED_strips[led_id][0][GREEN] = 0;
						LED_strips[led_id][0][RED]   = power;
						LED_strips[led_id][0][BLUE]  = 0;
					}
					else
					{
						LED_strips[led_id][0][GREEN] = 0;
						LED_strips[led_id][0][RED]   = 0;
						LED_strips[led_id][0][BLUE]  = power;
					}
				}
			}
			else
			{
				for (strip_id=0; strip_id < MAX_ACTIVE_STRIPS; strip_id++)
				{
					LED_strips[led_id][0][GREEN] = 0;
					LED_strips[led_id][0][RED] = 0;
					LED_strips[led_id][0][BLUE] = 0;
				}
			}
			update_GPIO_all_strips_mask(GPIO_PIN_10 | GPIO_PIN_5 | GPIO_PIN_6);
			update_driver_mask(GPIOB_PORT);
			drive_port_strips();
			wait_x_msec(50);
			cycle_cntr++;
		}
}

void BreatheTask(void)
{
	#define POWER_STEPS 25
	volatile int power_idx, strip_id, led_id;

		LD2_GPIO_Port->ODR |= LD2_Pin;
		for (;;)
		{
			for (power_idx=1; power_idx<POWER_STEPS; power_idx++)
			{
				for (strip_id=0; strip_id < MAX_ACTIVE_STRIPS; strip_id++)
				{
					uint8_t g_pwr, r_pwr, b_pwr;
					g_pwr = strip_id==1 ? power_idx*2 : 0;
					r_pwr = strip_id==1 ? power_idx*2 : 0;
					b_pwr = strip_id==0 ? power_idx*2 : 0;
					for (led_id=0; led_id<MAX_LEDS_IN_STRIP; led_id++)
					{
						LED_strips[strip_id][led_id][GREEN] = g_pwr;
						LED_strips[strip_id][led_id][RED]   = r_pwr;
						LED_strips[strip_id][led_id][BLUE]  = b_pwr;
					}
				}
				update_GPIO_all_strips_mask(GPIO_PIN_5);
				update_driver_mask(GPIOB_PORT);
				drive_port_strips();
				wait_x_msec(20);
			}
			for (power_idx=POWER_STEPS; power_idx>0; power_idx--)
			{
				for (strip_id=0; strip_id < MAX_ACTIVE_STRIPS; strip_id++)
				{
					uint8_t g_pwr, r_pwr, b_pwr;
					g_pwr = strip_id==1 ? power_idx*2 : 0;
					r_pwr = strip_id==1 ? power_idx*2 : 0;
					b_pwr = strip_id==0 ? power_idx*2 : 0;

					for (led_id=0; led_id<MAX_LEDS_IN_STRIP; led_id++)
					{
						LED_strips[strip_id][led_id][GREEN] = g_pwr;
						LED_strips[strip_id][led_id][RED]   = r_pwr;
						LED_strips[strip_id][led_id][BLUE]  = b_pwr;

					}
				}
				update_GPIO_all_strips_mask(GPIO_PIN_5);
				update_driver_mask(GPIOB_PORT);
				drive_port_strips();
				wait_x_msec(20);
			}

		}
}

void MatrixTask(void)
{

	//volatile int power_idx, strip_id, led_id;
	led_mat_t matrix;
	uint8_t bar_id, step;
	LD2_GPIO_Port->ODR |= LD2_Pin;

	init_mat(&matrix, 16, 32, 1, 0);
	for (bar_id=8; bar_id< matrix.num_of_bars; bar_id++)
		conf_bar(&matrix, bar_id, BAR_DIR_DOWN, 0, 32, 1);
	while(1)
	{
		for (bar_id=0; bar_id< matrix.num_of_bars; bar_id++)
		{
			set_bar(&matrix, bar_id, 0, 50, 50, 50);
		}
		wait_x_msec(2000);
		for (step=0; step<16; step++)
		{
			for (bar_id=0; bar_id< matrix.num_of_bars; bar_id++)
			{
				inc_bar(&matrix, bar_id, (bar_id%4) + 1, ((bar_id%3)==0) ? 150 : 0, ((bar_id%3)==1) ? 150 : 0, ((bar_id%3)==2) ? 150 : 0);
			}
			wait_x_msec(2000);
		}
		for (bar_id=0; bar_id< matrix.num_of_bars; bar_id++)
		{
			uint8_t bar_level = bar_id < 4  ? 10 :
								bar_id < 8  ? 14 :
								bar_id < 12 ? 10 : 14;
			set_bar(&matrix, bar_id, bar_level, 100, 75, 75);
		}
		wait_x_msec(2000);
		for (step=0; step<8; step++)
		{
			for (bar_id=0; bar_id< matrix.num_of_bars; bar_id++)
			{
				dec_bar(&matrix, bar_id, (bar_id%4) + 1);
			}
			wait_x_msec(2000);
		}

	}
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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MatrixTask();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 270;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
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
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
