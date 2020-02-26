
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f7xx_hal.h"
#include "rom_bank_0.h"

/* USER CODE BEGIN Includes */

#define SNES_IRQ_OFF 0xFFFF0008U
#define MISC_PORT_REG_WR GPIOA->BSRR
#define TRANS_SNES_2_CART 0xFFFF000CU
#define SNES_CART_OFF 0xFFFF0010U

#define MISC_PORT_REG_WR GPIOA->BSRR
#define MISC_PORT_REG_RD GPIOA->IDR
#define ADDR_PORT_REG_RD GPIOF->IDR
#define DATA_PORT_REG_WR GPIOD->BSRR

#define RST_VEC_LOC_LO 0x0000FFFCU
#define RST_VEC_LOC_HI 0x0000FFFDU
#define RST_VEC_BYT_LO 0xFFFF0000U
#define RST_VEC_BYT_HI 0xFFFF0088U
#define NMI_VEC_LOC_LO 0x0000FFEAU
#define NMI_VEC_LOC_HI 0x0000FFEBU
#define NMI_VEC_BYT_LO 0xFFFF0000U
#define NMI_VEC_BYT_HI 0xFFFF001EU

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t temp = 0;
uint16_t adc_temp = 0;
volatile uint8_t adc[16];
volatile uint32_t adc_32[8];

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__disable_irq();


  /* USER CODE END 1 */

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start_DMA(&hadc1,adc_32,8);
	
	IWDG->KR = ((uint16_t)0xAAAA);  // Disable WDG
	GPIOA->BSRR = 0xFFFF0008U;      // Hold SNES with RESET LOW; Disable Interrupt Request Line; Transceiver CART to SNES
	GPIOD->BSRR = 0xFFFF0000U;      // Reset Data Lines
	//__enable_irq();
	
	// Release the SNES
	/*Configure Reset Pin to Floating Input */
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = SNES_RESET_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	while(GPIOF->IDR != RST_VEC_LOC_LO){}
  DATA_PORT_REG_WR = RST_VEC_BYT_LO;
  while(GPIOF->IDR != RST_VEC_LOC_HI){}
  DATA_PORT_REG_WR = RST_VEC_BYT_HI;
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		
		if((MISC_PORT_REG_RD & SNES_CART_OFF))
    {
      MISC_PORT_REG_WR = TRANS_SNES_2_CART; // TRANSCEIVER SNES TO CART
      while(MISC_PORT_REG_RD & SNES_CART_OFF){}
      MISC_PORT_REG_WR = SNES_IRQ_OFF; // TRANSCEIVER CART TO SNES
    }
		else
		{
		  //MISC_PORT_REG_WR = SNES_IRQ_OFF; // TRANSCEIVER CART TO SNES
			temp = (ADDR_PORT_REG_RD & 0x00007FFFU);
			
			if(temp < 0x4FFFU)
			{
				DATA_PORT_REG_WR = (0xFFFF0000 | rom_bank_0[temp]);
			}
			else
			{
				if(temp == 0x7FEAU)
					DATA_PORT_REG_WR = (0xFFFF0000 | rom_bank_0[0x4FEAU]);
				
				if(temp == 0x7FEBU)
				{
					DATA_PORT_REG_WR = (0xFFFF0000 | rom_bank_0[0x4FEBU]);
					while(!(MISC_PORT_REG_RD & SNES_CART_OFF)){}
					MISC_PORT_REG_WR = TRANS_SNES_2_CART; // TRANSCEIVER SNES TO CART
					
					/* Pitch 1: 0x4405 0x4404 */
					adc_32[0] = ((~adc_32[0]) & 0x0FFFU);
					rom_bank_0[17413] = (uint8_t)((adc_32[0] / 8) >> 8);
					rom_bank_0[17412] = (uint8_t)((adc_32[0] / 8) & 0xFF);
					
					/* Pitch 2: 0x4410 0x440F */
					adc_32[1] = ((~adc_32[1]) & 0x0FFFU);
					rom_bank_0[17424] = (uint8_t)((adc_32[1] / 8) >> 8);
					rom_bank_0[17423] = (uint8_t)((adc_32[1] / 8) & 0xFF);
						
					/* Pitch 3: 0x441B 0x441A */
					adc_32[2] = ((~adc_32[2]) & 0x0FFFU);
					rom_bank_0[17435] = (uint8_t)((adc_32[2] / 8) >> 8);
					rom_bank_0[17434] = (uint8_t)((adc_32[2] / 8) & 0xFF);
						
					/* Pitch 4: 0x4426 0x4425 */
					adc_32[3] = ((~adc_32[3]) & 0x0FFFU);
					rom_bank_0[17446] = (uint8_t)((adc_32[3] / 8) >> 8);
					rom_bank_0[17445] = (uint8_t)((adc_32[3] / 8) & 0xFF);
						
					/* Volume 1: 0x4402 (0x4403) */
					adc_32[4] = ((~adc_32[4]) & 0x0FFFU);
					rom_bank_0[0x4402] = (uint8_t)((adc_32[4] / 1) >> 5) | 0x0FU;
					rom_bank_0[0x4403] = (uint8_t)((adc_32[4] / 1) >> 5) | 0x0FU;
					
					/* Volume 2: 0x440D (0x440E) */
					adc_32[5] = ((~adc_32[5]) & 0x0FFFU);
					rom_bank_0[0x440D] = (uint8_t)((adc_32[5] / 1) >> 5) | 0x0FU;
					rom_bank_0[0x440E] = (uint8_t)((adc_32[5] / 1) >> 5) | 0x0FU;
						
					/* Volume 3: 0x4418 (0x4419) */
					adc_32[6] = ((~adc_32[6]) & 0x0FFFU);
					rom_bank_0[0x4418] = (uint8_t)((adc_32[6] / 1) >> 5) | 0x0FU;
					rom_bank_0[0x4419] = (uint8_t)((adc_32[6] / 1) >> 5) | 0x0FU;
						
					/* Volume 4: 0x4423 (0x4424) */
					adc_32[7] = ((~adc_32[7]) & 0x0FFFU);
					rom_bank_0[0x4423] = (uint8_t)((adc_32[7] / 1) >> 5) | 0x0FU;
					rom_bank_0[0x4424] = (uint8_t)((adc_32[7] / 1) >> 5) | 0x0FU;
						
							
					while(MISC_PORT_REG_RD & SNES_CART_OFF){}
					MISC_PORT_REG_WR = SNES_IRQ_OFF; // TRANSCEIVER CART TO SNES

				}
			}
		}
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRAN_OE_INV_Pin|TRAN_DIR_Pin|SNES_IRQ_INV_Pin|SNES_RESET_INV_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DAT0_Pin|DAT1_Pin|DAT2_Pin|DAT3_Pin 
                          |DAT4_Pin|DAT5_Pin|DAT6_Pin|DAT7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ADR0_Pin ADR1_Pin ADR2_Pin ADR3_Pin 
                           ADR4_Pin ADR5_Pin ADR6_Pin ADR7_Pin 
                           ADR8_Pin ADR9_Pin ADR10_Pin ADR11_Pin 
                           ADR12_Pin ADR13_Pin ADR14_Pin ADR15_Pin */
  GPIO_InitStruct.Pin = ADR0_Pin|ADR1_Pin|ADR2_Pin|ADR3_Pin 
                          |ADR4_Pin|ADR5_Pin|ADR6_Pin|ADR7_Pin 
                          |ADR8_Pin|ADR9_Pin|ADR10_Pin|ADR11_Pin 
                          |ADR12_Pin|ADR13_Pin|ADR14_Pin|ADR15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TRAN_OE_INV_Pin TRAN_DIR_Pin SNES_IRQ_INV_Pin */
  GPIO_InitStruct.Pin = TRAN_OE_INV_Pin|TRAN_DIR_Pin|SNES_IRQ_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SNES_CART_INV_Pin SNES_RD_INV_Pin SNES_WR_INV_Pin SNES_CLK_Pin */
  GPIO_InitStruct.Pin = SNES_CART_INV_Pin|SNES_RD_INV_Pin|SNES_WR_INV_Pin|SNES_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SNES_RESET_INV_Pin */
  GPIO_InitStruct.Pin = SNES_RESET_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SNES_RESET_INV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DAT0_Pin DAT1_Pin DAT2_Pin DAT3_Pin 
                           DAT4_Pin DAT5_Pin DAT6_Pin DAT7_Pin */
  GPIO_InitStruct.Pin = DAT0_Pin|DAT1_Pin|DAT2_Pin|DAT3_Pin 
                          |DAT4_Pin|DAT5_Pin|DAT6_Pin|DAT7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
