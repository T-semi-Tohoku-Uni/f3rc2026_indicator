/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Seg2_PC13_Pin GPIO_PIN_13
#define Seg2_PC13_GPIO_Port GPIOC
#define BarLED_PC0_Pin GPIO_PIN_0
#define BarLED_PC0_GPIO_Port GPIOC
#define BarLED_PC1_Pin GPIO_PIN_1
#define BarLED_PC1_GPIO_Port GPIOC
#define Sounder1_PC2_Pin GPIO_PIN_2
#define Sounder1_PC2_GPIO_Port GPIOC
#define TapeLED_PC3_Pin GPIO_PIN_3
#define TapeLED_PC3_GPIO_Port GPIOC
#define BarLED_PA0_Pin GPIO_PIN_0
#define BarLED_PA0_GPIO_Port GPIOA
#define BarLED_PA1_Pin GPIO_PIN_1
#define BarLED_PA1_GPIO_Port GPIOA
#define BarLED_PA4_Pin GPIO_PIN_4
#define BarLED_PA4_GPIO_Port GPIOA
#define BarLED_PA5_Pin GPIO_PIN_5
#define BarLED_PA5_GPIO_Port GPIOA
#define BarLED_PA6_Pin GPIO_PIN_6
#define BarLED_PA6_GPIO_Port GPIOA
#define BarLED_PA7_Pin GPIO_PIN_7
#define BarLED_PA7_GPIO_Port GPIOA
#define Seg1_PB0_Pin GPIO_PIN_0
#define Seg1_PB0_GPIO_Port GPIOB
#define Seg1_PB1_Pin GPIO_PIN_1
#define Seg1_PB1_GPIO_Port GPIOB
#define Seg1_PB2_Pin GPIO_PIN_2
#define Seg1_PB2_GPIO_Port GPIOB
#define Seg2_PB10_Pin GPIO_PIN_10
#define Seg2_PB10_GPIO_Port GPIOB
#define Seg2_PB11_Pin GPIO_PIN_11
#define Seg2_PB11_GPIO_Port GPIOB
#define Seg2_PB12_Pin GPIO_PIN_12
#define Seg2_PB12_GPIO_Port GPIOB
#define Seg2_PB13_Pin GPIO_PIN_13
#define Seg2_PB13_GPIO_Port GPIOB
#define Seg2_PB14_Pin GPIO_PIN_14
#define Seg2_PB14_GPIO_Port GPIOB
#define Seg2_PB15_Pin GPIO_PIN_15
#define Seg2_PB15_GPIO_Port GPIOB
#define Sounder2_PC6_Pin GPIO_PIN_6
#define Sounder2_PC6_GPIO_Port GPIOC
#define Sounder3_PC7_Pin GPIO_PIN_7
#define Sounder3_PC7_GPIO_Port GPIOC
#define SW1_PC9_Pin GPIO_PIN_9
#define SW1_PC9_GPIO_Port GPIOC
#define BarLED_PA8_Pin GPIO_PIN_8
#define BarLED_PA8_GPIO_Port GPIOA
#define BarLED_PA9_Pin GPIO_PIN_9
#define BarLED_PA9_GPIO_Port GPIOA
#define buzzer_PA10_Pin GPIO_PIN_10
#define buzzer_PA10_GPIO_Port GPIOA
#define SW2_PC10_Pin GPIO_PIN_10
#define SW2_PC10_GPIO_Port GPIOC
#define SW3_PC11_Pin GPIO_PIN_11
#define SW3_PC11_GPIO_Port GPIOC
#define SWtact_PC12_Pin GPIO_PIN_12
#define SWtact_PC12_GPIO_Port GPIOC
#define Board_LED_Pin GPIO_PIN_2
#define Board_LED_GPIO_Port GPIOD
#define Seg1_PB4_Pin GPIO_PIN_4
#define Seg1_PB4_GPIO_Port GPIOB
#define Seg1_PB5_Pin GPIO_PIN_5
#define Seg1_PB5_GPIO_Port GPIOB
#define Seg1_PB6_Pin GPIO_PIN_6
#define Seg1_PB6_GPIO_Port GPIOB
#define Seg1_PB7_Pin GPIO_PIN_7
#define Seg1_PB7_GPIO_Port GPIOB
#define Seg1_PB8_Pin GPIO_PIN_8
#define Seg1_PB8_GPIO_Port GPIOB
#define Seg2_PB9_Pin GPIO_PIN_9
#define Seg2_PB9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define Sounder1Mode 1 // tim1
#define Sounder2Mode 3 // tim3
#define Sounder3Mode 8 // tim8

void BarLED_Display(uint8_t num);
int BarLED_ToggleAnimate(uint8_t one_zero);

// 7セグメント表示
void SevenSeg_Display_AllOff();
void SevenSeg_Display_Hyphen();
void SevenSeg_Display_Number(int8_t num, uint8_t segNo);
void SevenSeg_Display_Slider(uint8_t num);
void SevenSeg_Display_Alphabet_idx(uint8_t index, uint8_t segNo);
void SevenSeg_Display_Alphabet_char(char alphabet, uint8_t segNo);

int SevenSeg_ToggleAnimate_Number1(uint8_t one_zero);
int SevenSeg_ToggleAnimate_Number2(uint8_t one_zero);
int SevenSeg_ToggleAnimate_Slider(uint8_t one_zero);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
