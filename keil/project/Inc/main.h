/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ADR0_Pin GPIO_PIN_0
#define ADR0_GPIO_Port GPIOF
#define ADR1_Pin GPIO_PIN_1
#define ADR1_GPIO_Port GPIOF
#define ADR2_Pin GPIO_PIN_2
#define ADR2_GPIO_Port GPIOF
#define ADR3_Pin GPIO_PIN_3
#define ADR3_GPIO_Port GPIOF
#define ADR4_Pin GPIO_PIN_4
#define ADR4_GPIO_Port GPIOF
#define ADR5_Pin GPIO_PIN_5
#define ADR5_GPIO_Port GPIOF
#define ADR6_Pin GPIO_PIN_6
#define ADR6_GPIO_Port GPIOF
#define ADR7_Pin GPIO_PIN_7
#define ADR7_GPIO_Port GPIOF
#define ADR8_Pin GPIO_PIN_8
#define ADR8_GPIO_Port GPIOF
#define ADR9_Pin GPIO_PIN_9
#define ADR9_GPIO_Port GPIOF
#define ADR10_Pin GPIO_PIN_10
#define ADR10_GPIO_Port GPIOF
#define POT2_Pin GPIO_PIN_0
#define POT2_GPIO_Port GPIOC
#define POT3_Pin GPIO_PIN_1
#define POT3_GPIO_Port GPIOC
#define POT4_Pin GPIO_PIN_2
#define POT4_GPIO_Port GPIOC
#define POT5_Pin GPIO_PIN_3
#define POT5_GPIO_Port GPIOC
#define TRAN_OE_INV_Pin GPIO_PIN_1
#define TRAN_OE_INV_GPIO_Port GPIOA
#define TRAN_DIR_Pin GPIO_PIN_2
#define TRAN_DIR_GPIO_Port GPIOA
#define SNES_IRQ_INV_Pin GPIO_PIN_3
#define SNES_IRQ_INV_GPIO_Port GPIOA
#define SNES_CART_INV_Pin GPIO_PIN_4
#define SNES_CART_INV_GPIO_Port GPIOA
#define SNES_RD_INV_Pin GPIO_PIN_5
#define SNES_RD_INV_GPIO_Port GPIOA
#define SNES_WR_INV_Pin GPIO_PIN_6
#define SNES_WR_INV_GPIO_Port GPIOA
#define SNES_RESET_INV_Pin GPIO_PIN_7
#define SNES_RESET_INV_GPIO_Port GPIOA
#define POT6_Pin GPIO_PIN_4
#define POT6_GPIO_Port GPIOC
#define POT7_Pin GPIO_PIN_5
#define POT7_GPIO_Port GPIOC
#define POT0_Pin GPIO_PIN_0
#define POT0_GPIO_Port GPIOB
#define POT1_Pin GPIO_PIN_1
#define POT1_GPIO_Port GPIOB
#define ADR11_Pin GPIO_PIN_11
#define ADR11_GPIO_Port GPIOF
#define ADR12_Pin GPIO_PIN_12
#define ADR12_GPIO_Port GPIOF
#define ADR13_Pin GPIO_PIN_13
#define ADR13_GPIO_Port GPIOF
#define ADR14_Pin GPIO_PIN_14
#define ADR14_GPIO_Port GPIOF
#define ADR15_Pin GPIO_PIN_15
#define ADR15_GPIO_Port GPIOF
#define ADR16_Pin GPIO_PIN_0
#define ADR16_GPIO_Port GPIOG
#define ADR17_Pin GPIO_PIN_1
#define ADR17_GPIO_Port GPIOG
#define GPIO0_Pin GPIO_PIN_11
#define GPIO0_GPIO_Port GPIOB
#define GPIO1_Pin GPIO_PIN_12
#define GPIO1_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_13
#define GPIO2_GPIO_Port GPIOB
#define GPIO3_Pin GPIO_PIN_14
#define GPIO3_GPIO_Port GPIOB
#define USR_LED_Pin GPIO_PIN_15
#define USR_LED_GPIO_Port GPIOB
#define ADR18_Pin GPIO_PIN_2
#define ADR18_GPIO_Port GPIOG
#define ADR19_Pin GPIO_PIN_3
#define ADR19_GPIO_Port GPIOG
#define ADR20_Pin GPIO_PIN_4
#define ADR20_GPIO_Port GPIOG
#define ADR21_Pin GPIO_PIN_5
#define ADR21_GPIO_Port GPIOG
#define ADR22_Pin GPIO_PIN_6
#define ADR22_GPIO_Port GPIOG
#define ADR23_Pin GPIO_PIN_7
#define ADR23_GPIO_Port GPIOG
#define SNES_CLK_Pin GPIO_PIN_8
#define SNES_CLK_GPIO_Port GPIOA
#define BTN0_Pin GPIO_PIN_9
#define BTN0_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_10
#define BTN1_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_11
#define BTN2_GPIO_Port GPIOA
#define BTN3_Pin GPIO_PIN_12
#define BTN3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCK_Pin GPIO_PIN_14
#define SWCK_GPIO_Port GPIOA
#define DAT0_Pin GPIO_PIN_0
#define DAT0_GPIO_Port GPIOD
#define DAT1_Pin GPIO_PIN_1
#define DAT1_GPIO_Port GPIOD
#define DAT2_Pin GPIO_PIN_2
#define DAT2_GPIO_Port GPIOD
#define DAT3_Pin GPIO_PIN_3
#define DAT3_GPIO_Port GPIOD
#define DAT4_Pin GPIO_PIN_4
#define DAT4_GPIO_Port GPIOD
#define DAT5_Pin GPIO_PIN_5
#define DAT5_GPIO_Port GPIOD
#define DAT6_Pin GPIO_PIN_6
#define DAT6_GPIO_Port GPIOD
#define DAT7_Pin GPIO_PIN_7
#define DAT7_GPIO_Port GPIOD

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
