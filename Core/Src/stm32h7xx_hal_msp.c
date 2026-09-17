/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_hal_msp.c
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
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
  *
  * Stripped down for the ADC / hysteretic-controller work. Only HAL_MspInit()
  * is left. Removed relative to the CubeMX version:
  *
  *   - the extern declarations of hdma_dac1_ch1, hdma_dac1_ch2, hdma_hrtim1_a
  *     and hdma_tim7_up, which lived in the old main.c and no longer exist -
  *     that is what the "undefined reference to hdma_..." link errors were
  *   - HAL_ADC_MspInit / MspDeInit
  *   - HAL_DAC_MspInit / MspDeInit
  *   - HAL_HRTIM_MspInit / MspPostInit / MspDeInit
  *   - HAL_TIM_Base_MspInit / MspDeInit and HAL_TIM_MspPostInit
  *
  * Dropping those is safe: the application file configures every peripheral
  * clock and pin it uses itself (ASP_AdcClockConfig, ASP_GpioConfig and the
  * per-timer config functions, or their ADE_ equivalents). The HAL falls back
  * to its own weak empty MspInit, which now does nothing - which is correct,
  * because there is nothing left for it to do.
  *
  * If CubeMX regenerates this file it will put all of those back, and the link
  * will break again in exactly the same way. Re-apply this file after any
  * regeneration, or disable DAC1, COMP1, COMP2, HRTIM1, TIM1, TIM2, TIM6 and
  * TIM7 in the .ioc so CubeMX stops emitting them.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
