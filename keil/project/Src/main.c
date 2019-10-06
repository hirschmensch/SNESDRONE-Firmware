
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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "rom_bank_0.h"

#define SNES_IRQ_OFF 0xFFFF0008U
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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t i = 0;
uint16_t temp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile uint8_t adc[16];
volatile uint32_t adc_32[8];
volatile uint32_t adc_prev11_32;
volatile uint32_t adc_prev21_32;
volatile uint32_t adc_prev31_32;
volatile uint32_t adc_prev41_32;
volatile uint32_t adc_prev51_32;
volatile uint32_t adc_prev61_32;
volatile uint32_t adc_prev71_32;
volatile uint32_t adc_prev81_32;
volatile uint32_t adc_prev91_32;
volatile uint32_t adc_prev101_32;
volatile uint32_t adc_prev111_32;
volatile uint32_t adc_prev121_32;
volatile uint32_t adc_now1_32;

volatile uint32_t adc_prev12_32;
volatile uint32_t adc_prev22_32;
volatile uint32_t adc_prev32_32;
volatile uint32_t adc_prev42_32;
volatile uint32_t adc_prev52_32;
volatile uint32_t adc_prev62_32;
volatile uint32_t adc_prev72_32;
volatile uint32_t adc_prev82_32;
volatile uint32_t adc_prev92_32;
volatile uint32_t adc_prev102_32;
volatile uint32_t adc_prev112_32;
volatile uint32_t adc_prev122_32;
volatile uint32_t adc_now2_32;

volatile uint32_t adc_prev13_32;
volatile uint32_t adc_prev23_32;
volatile uint32_t adc_prev33_32;
volatile uint32_t adc_prev43_32;
volatile uint32_t adc_prev53_32;
volatile uint32_t adc_prev63_32;
volatile uint32_t adc_prev73_32;
volatile uint32_t adc_prev83_32;
volatile uint32_t adc_prev93_32;
volatile uint32_t adc_prev103_32;
volatile uint32_t adc_prev113_32;
volatile uint32_t adc_prev123_32;
volatile uint32_t adc_now3_32;

volatile uint32_t adc_prev14_32;
volatile uint32_t adc_prev24_32;
volatile uint32_t adc_prev34_32;
volatile uint32_t adc_prev44_32;
volatile uint32_t adc_prev54_32;
volatile uint32_t adc_prev64_32;
volatile uint32_t adc_prev74_32;
volatile uint32_t adc_prev84_32;
volatile uint32_t adc_prev94_32;
volatile uint32_t adc_prev104_32;
volatile uint32_t adc_prev114_32;
volatile uint32_t adc_prev124_32;
volatile uint32_t adc_now4_32;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

		//adc[0] = adc_32[0];
	/*
		adc[1] = adc_32[0] & 0xFF;
		adc[2] = adc_32[1] >> 8;
		adc[3] = adc_32[1] & 0xFF;
		adc[4] = adc_32[2] >> 8;
		adc[5] = adc_32[2] & 0xFF;
		adc[6] = adc_32[3] >> 8;
		adc[7] = adc_32[3] & 0xFF;
*/
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start_DMA(&hadc1,adc_32,8);
	
	
	//adc[0] = 0x01;
	//adc[1] = 0x00;
	
	adc[2] = 0x00;
	adc[3] = 0x00;
	/*
	adc[4] = 0x01;
	adc[5] = 0x00;
	adc[6] = 0x01;
	adc[7] = 0x00;
	*/
	
	
	__disable_irq();
	
	IWDG->KR = ((uint16_t)0xAAAA);	// Watchdog no bark

  MISC_PORT_REG_WR = SNES_IRQ_OFF; // TRANSCEIVER CART TO SNES (and disable IRQ line)

  while(GPIOF->IDR != RST_VEC_LOC_LO){}
  DATA_PORT_REG_WR = RST_VEC_BYT_LO;
  while(GPIOF->IDR != RST_VEC_LOC_HI){}
  DATA_PORT_REG_WR = RST_VEC_BYT_HI;
	DATA_PORT_REG_WR = RST_VEC_BYT_HI;
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
						
						
						adc_prev11_32 = adc_prev21_32;
						adc_prev21_32 = adc_prev31_32;
						adc_prev31_32 = adc_prev41_32;
						adc_prev41_32 = adc_prev51_32;
						adc_prev51_32 = adc_prev61_32;
						adc_prev61_32 = adc_prev71_32;
						adc_prev71_32 = adc_prev81_32;
						adc_prev81_32 = adc_prev91_32;
						adc_prev91_32 = adc_prev101_32;
						adc_prev101_32 = adc_prev111_32;
						adc_prev111_32 = adc_prev121_32;
						adc_prev121_32 = adc_32[0];
						
						adc_now1_32 = (adc_prev11_32 + adc_prev21_32 + adc_prev31_32 + adc_prev41_32 + adc_prev51_32 + adc_prev61_32 + adc_prev71_32 + adc_prev81_32 + adc_prev91_32 + adc_prev101_32 + adc_prev111_32 + adc_prev121_32)/12;
						
						//rom_bank_0[17413] = adc[0];					// Freq1 HI
				//rom_bank_0[17412] = adc[1];					// Freq1 LO
						//adc_32[0] = (adc_32[0] + adc_prev_32[0]) / 2;
						//adc_prev_32[0] = adc_32[0];

						rom_bank_0[17413] = (uint8_t)((adc_now1_32 / 1) >> 8);
						rom_bank_0[17412] = (uint8_t)((adc_now1_32 / 1) & 0xFF);
						
						
						adc_prev12_32 = adc_prev22_32;
						adc_prev22_32 = adc_prev32_32;
						adc_prev32_32 = adc_prev42_32;
						adc_prev42_32 = adc_prev52_32;
						adc_prev52_32 = adc_prev62_32;
						adc_prev62_32 = adc_prev72_32;
						adc_prev72_32 = adc_prev82_32;
						adc_prev82_32 = adc_prev92_32;
						adc_prev92_32 = adc_prev102_32;
						adc_prev102_32 = adc_prev112_32;
						adc_prev112_32 = adc_prev122_32;
						adc_prev122_32 = adc_32[1];
											
						adc_now2_32 = (adc_prev12_32 + adc_prev22_32 + adc_prev32_32 + adc_prev42_32 + adc_prev52_32 + adc_prev62_32 + adc_prev72_32 + adc_prev82_32 + adc_prev92_32 + adc_prev102_32 + adc_prev112_32 + adc_prev122_32)/12;
						
						
				//rom_bank_0[17424] = adc[2];					// Freq2 HI
				//rom_bank_0[17423] = adc[3];					// Freq2 LO
				
				    adc_prev13_32 = adc_prev23_32;
						adc_prev23_32 = adc_prev33_32;
						adc_prev33_32 = adc_prev43_32;
						adc_prev43_32 = adc_prev53_32;
						adc_prev53_32 = adc_prev63_32;
						adc_prev63_32 = adc_prev73_32;
						adc_prev73_32 = adc_prev83_32;
						adc_prev83_32 = adc_prev93_32;
						adc_prev93_32 = adc_prev103_32;
						adc_prev103_32 = adc_prev113_32;
						adc_prev113_32 = adc_prev123_32;
						adc_prev123_32 = adc_32[2];
						
						adc_now3_32 = (adc_prev13_32 + adc_prev23_32 + adc_prev33_32 + adc_prev43_32 + adc_prev53_32 + adc_prev63_32 + adc_prev73_32 + adc_prev83_32 + adc_prev93_32 + adc_prev103_32 + adc_prev113_32 + adc_prev123_32)/12;

						rom_bank_0[17424] = (uint8_t)((adc_now2_32 / 1) >> 8);
						rom_bank_0[17423] = (uint8_t)((adc_now2_32 / 1) & 0xFF);
						
				//rom_bank_0[17435] = adc[4];					// Freq3 HI
				//rom_bank_0[17434] = adc[5];					// Freq3 LO

						rom_bank_0[17435] = (uint8_t)((adc_now3_32 / 8) >> 8);
						rom_bank_0[17434] = (uint8_t)((adc_now3_32 / 8) & 0xFF);
						
				//						rom_bank_0[17446] = adc[6];					// Freq4 HI
				//rom_bank_0[17445] = adc[7];					// Freq4 LO
				
				adc_prev14_32 = adc_prev24_32;
						adc_prev24_32 = adc_prev34_32;
						adc_prev34_32 = adc_prev44_32;
						adc_prev44_32 = adc_prev54_32;
						adc_prev54_32 = adc_prev64_32;
						adc_prev64_32 = adc_prev74_32;
						adc_prev74_32 = adc_prev84_32;
						adc_prev84_32 = adc_prev94_32;
						adc_prev94_32 = adc_prev104_32;
						adc_prev104_32 = adc_prev114_32;
						adc_prev114_32 = adc_prev124_32;
						adc_prev124_32 = adc_32[3];
						
						adc_now4_32 = (adc_prev14_32 + adc_prev24_32 + adc_prev34_32 + adc_prev44_32 + adc_prev54_32 + adc_prev64_32 + adc_prev74_32 + adc_prev84_32 + adc_prev94_32 + adc_prev104_32 + adc_prev114_32 + adc_prev124_32)/12;

				rom_bank_0[17446] = (uint8_t)((adc_now4_32 / 8) >> 8);
						rom_bank_0[17445] = (uint8_t)((adc_now4_32 / 8) & 0xFF);
						
						adc[0] = (adc_now1_32 >> 4) & 0xFFU;
						
						if (adc[0] < 0x0FU)
							adc[0] = 0x0FU;
						
						if (adc[0] > 0x7FU)
							adc[0] = 0x7FU;
						
						rom_bank_0[17410] = adc[0];
						rom_bank_0[17411] = adc[0];
						/*
						adc[1] = adc_now2_32 >> 4;
						
						if (adc[1] < 0x0FU)
							adc[1] = 0x0F;
						
						if (adc[1] > 0x7FU)
							adc[1] = 0x7F;
						
						rom_bank_0[17421] = adc[1];
						
						rom_bank_0[17422] = adc[1];
						
						adc[2] = adc_now3_32 >> 4;
						
						if (adc[2] < 0x0FU)
							adc[2] = 0x0F;
						
						if (adc[2] > 0x7FU)
							adc[2] = 0x7F;
						
						rom_bank_0[17432] = adc[2];
						rom_bank_0[17433] = adc[2];
						
						adc[3] = adc_now4_32 >> 4;
						
						if (adc[3] < 0x0FU)
							adc[3] = 0x0F;
						
						if (adc[3] > 0x7FU)
							adc[3] = 0x7F;
						
						rom_bank_0[17443] = adc[3];
						rom_bank_0[17444] = adc[3];
						*/
						
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
  RCC_OscInitStruct.PLL.PLLN = 180;
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
