///*
// * main3.c
// *
// *  Created on: Sep 4, 2026
// *      Author: shivam
// */
//
//
///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2026 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "sine.h"
//#include "spwm_lut.h"
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* Kernel clock feeding TIM2 / TIM6 / TIM7 (APB1 timer clock) */
//#define TIMER_KERNEL_CLK_HZ   240000000UL
//
///* Samples per period in the sine and SPWM lookup tables */
//#define LUT_STEPS             128UL
//
///* HRTIM timer A time base, in high-resolution ticks */
//#define HRTIM_TA_PERIOD       960U
//
///* With PrescalerRatio DIV1 a compare value must stay inside
//   [0x60, PERIOD - 0x60]. A value at the 0x60 floor can be missed by the
//   compare unit, which latches the output high and kills further CMP1 DMA
//   requests, so the duty ramp is kept well clear of both ends. */
//#define HRTIM_CMP_FLOOR       0x60U
//#define HRTIM_CMP_MIN         (HRTIM_TA_PERIOD / 5U)   /* 192 ticks, 20 % duty */
//#define HRTIM_CMP_STEP        (HRTIM_TA_PERIOD / 15U)  /*  64 ticks, ~6.7 % */
//
///* Duty-cycle steps streamed into HRTIM timer A and B CMP1 by DMA */
//#define HRTIM_LUT_SIZE        10U
//
///* Phase delay of the second output (TB1) behind the first (TA1), in ticks.
//   480 ticks = HRTIM_TA_PERIOD / 2 = 1 us at the 500 kHz / 480 MHz operating
//   point. Change this one value to retune the shift. */
//#define HRTIM_PHASE_TICKS     (HRTIM_TA_PERIOD / 2U)
//
///* HRTIM1_CHB1 pin. Confirm the mapping for your package in the datasheet
//   alternate-function table before building; PC8 / AF1 is the common one. */
//#define HRTIM_CHB1_PORT       GPIOC
//#define HRTIM_CHB1_PIN        GPIO_PIN_8
//#define HRTIM_CHB1_AF         GPIO_AF1_HRTIM1
//
///* Free DMA1 stream for the timer B duty stream (0..3 are already in use by
//   DAC1 CH1/CH2, TIM7_UP and HRTIM1_A) */
//#define HRTIM_CHB_DMA_STREAM  DMA1_Stream4
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//COMP_HandleTypeDef hcomp1;
//COMP_HandleTypeDef hcomp2;
//
//DAC_HandleTypeDef hdac1;
//DMA_HandleTypeDef hdma_dac1_ch1;
//DMA_HandleTypeDef hdma_dac1_ch2;
//
//HRTIM_HandleTypeDef hhrtim;
//DMA_HandleTypeDef hdma_hrtim1_a;
//
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim6;
//TIM_HandleTypeDef htim7;
//DMA_HandleTypeDef hdma_tim7_up;
//
///* USER CODE BEGIN PV */
//
///* Second HRTIM duty stream. Configured entirely in code below, so no CubeMX
//   round trip is needed to add the HRTIM1_B DMA request. */
//DMA_HandleTypeDef hdma_hrtim1_b;
//
///* Duty-cycle ramp streamed into both HRTIM_A CMP1 and HRTIM_B CMP1,
//   20 % up to 80 % in ~6.7 % steps.
//   Every entry must sit inside [HRTIM_CMP_FLOOR, HRTIM_TA_PERIOD - HRTIM_CMP_FLOOR];
//   a value outside that window never produces a compare match, which latches the
//   output at one level and stops further CMP1 DMA requests. */
//const uint32_t hrtim_lut[HRTIM_LUT_SIZE] = {
//    HRTIM_CMP_MIN + (0U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (1U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (2U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (3U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (4U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (5U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (6U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (7U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (8U * HRTIM_CMP_STEP),
//    HRTIM_CMP_MIN + (9U * HRTIM_CMP_STEP)
//};
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_COMP1_Init(void);
//static void MX_DAC1_Init(void);
//static void MX_COMP2_Init(void);
//static void MX_TIM6_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_TIM7_Init(void);
//static void MX_HRTIM_Init(void);
///* USER CODE BEGIN PFP */
//void Set_PWM_Carrier_Frequency(uint32_t target_hz);
//void Set_SPWM_Frequency(uint32_t target_hz);
//void Set_Sine_Frequency(uint32_t target_hz);
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///**
//  * @brief  Compute an ARR value for a target output rate.
//  * @param  target_hz Desired output frequency in Hz.
//  * @param  steps     LUT steps per output period (1 for a plain carrier).
//  * @retval ARR clamped to 16 bits, or 0 when the request is invalid.
//  */
//static uint32_t Timer_CalcArr(uint32_t target_hz, uint32_t steps)
//{
//  if ((target_hz == 0U) || (steps == 0U))
//  {
//    return 0U;
//  }
//
//  uint32_t ticks = TIMER_KERNEL_CLK_HZ / (steps * target_hz);
//
//  /* Guard the ticks - 1 subtraction against underflow on unreachable rates */
//  if (ticks < 2U)
//  {
//    ticks = 2U;
//  }
//
//  uint32_t arr = ticks - 1U;
//
//  /* TIM6 and TIM7 are 16-bit, so clamp to the narrower limit */
//  if (arr > 0xFFFFU)
//  {
//    arr = 0xFFFFU;
//  }
//
//  return arr;
//}
//
///**
//  * @brief  Set the TIM2 PWM carrier frequency.
//  */
//void Set_PWM_Carrier_Frequency(uint32_t target_hz)
//{
//  uint32_t arr = Timer_CalcArr(target_hz, 1U);
//
//  if (arr == 0U)
//  {
//    return;
//  }
//
//  TIM2->ARR = arr;
//  TIM2->EGR |= TIM_EGR_UG;    /* Load the new ARR immediately */
//}
//
///**
//  * @brief  Set the SPWM envelope frequency paced by TIM7.
//  */
//void Set_SPWM_Frequency(uint32_t target_hz)
//{
//  uint32_t arr = Timer_CalcArr(target_hz, LUT_STEPS);
//
//  if (arr == 0U)
//  {
//    return;
//  }
//
//  TIM7->ARR = arr;
//  TIM7->EGR |= TIM_EGR_UG;
//}
//
///**
//  * @brief  Set the DAC sine frequency paced by TIM6.
//  */
//void Set_Sine_Frequency(uint32_t target_hz)
//{
//  uint32_t arr = Timer_CalcArr(target_hz, LUT_STEPS);
//
//  if (arr == 0U)
//  {
//    return;
//  }
//
//  TIM6->ARR = arr;
//  TIM6->EGR |= TIM_EGR_UG;
//}
//
///**
//  * @brief  Configure the HRTIM1_CHB1 pin as an alternate-function output.
//  * @note   CubeMX only knows about CHA1, so the second output pin is set up
//  *         here. The GPIOC clock is already enabled by MX_GPIO_Init().
//  */
//static void HRTIM_ChB1_GpioInit(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  GPIO_InitStruct.Pin       = HRTIM_CHB1_PIN;
//  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull      = GPIO_NOPULL;
//  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = HRTIM_CHB1_AF;
//  HAL_GPIO_Init(HRTIM_CHB1_PORT, &GPIO_InitStruct);
//}
//
///**
//  * @brief  Arm one circular DMA stream that walks hrtim_lut into a timing
//  *         unit's CMP1 register, and enable that unit's CMP1 DMA request.
//  * @param  hdma        Stream handle with Instance and Init.Request already set.
//  * @param  timer_index HRTIM_TIMERINDEX_TIMER_x of the target timing unit.
//  * @note   Every transfer parameter is pinned down here rather than left to the
//  *         CubeMX stream page: MEMORY_TO_PERIPH, memory incrementing, fixed
//  *         peripheral address, WORD on both sides because CMP1xR is 32-bit, and
//  *         CIRCULAR - in DMA_NORMAL the stream would stop after HRTIM_LUT_SIZE
//  *         transfers and park CMP1 on the last entry as a fixed duty cycle.
//  */
//static void HRTIM_StartDutyStream(DMA_HandleTypeDef *hdma, uint32_t timer_index)
//{
//  hdma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
//  hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
//  hdma->Init.MemInc              = DMA_MINC_ENABLE;
//  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//  hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
//  hdma->Init.Mode                = DMA_CIRCULAR;
//  hdma->Init.Priority            = DMA_PRIORITY_HIGH;
//  hdma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
//
//  if (HAL_DMA_Init(hdma) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  if (HAL_DMA_Start(hdma,
//                    (uint32_t)spwm_lut_5a_1k_per960,
//                    (uint32_t)&HRTIM1->sTimerxRegs[timer_index].CMP1xR,
//                    500) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* HAL_HRTIM_WaveformTimerConfig() only caches DMARequests in TimerParam[];
//     TIMxDIER is written by HAL_HRTIM_WaveformCountStart_DMA(). The streams are
//     armed by hand, so enable the CMP1 DMA request explicitly - without this no
//     request is ever generated and CMP1 keeps its initial value. Written
//     straight to the register because the __HAL_HRTIM_TIMER_ENABLE_DMA() macro
//     is not present in every HAL revision. */
//  HRTIM1->sTimerxRegs[timer_index].TIMxDIER |= HRTIM_TIM_DMA_CMP1;
//}
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* Enable the CPU Cache */
//
//  /* Enable I-Cache---------------------------------------------------------*/
//  SCB_EnableICache();
//
//  /* Enable D-Cache---------------------------------------------------------*/
//  SCB_EnableDCache();
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_TIM1_Init();
//  MX_COMP1_Init();
//  MX_DAC1_Init();
//  MX_COMP2_Init();
//  MX_TIM6_Init();
//  MX_TIM2_Init();
//  MX_TIM7_Init();
//  MX_HRTIM_Init();
//  /* USER CODE BEGIN 2 */
//
//  /* ---- Timing setup ----------------------------------------------------- */
//  Set_Sine_Frequency(500);
//  Set_SPWM_Frequency(1000);
//
//  /* ---- TIM2 carrier, duty stream paced by TIM7 -------------------------- */
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//
//  if (HAL_DMA_Start(&hdma_tim7_up,
//                    (uint32_t)spwm_lut,
//                    (uint32_t)&TIM2->CCR1,
//                    LUT_STEPS) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  __HAL_TIM_ENABLE_DMA(&htim7, TIM_DMA_UPDATE);
//  HAL_TIM_Base_Start(&htim7);
//
//  /* ---- HRTIM interleaved pair: TA1 leading, TB1 delayed ----------------- */
//  HRTIM_ChB1_GpioInit();
//
//  /* Timer A stream: the handle comes from CubeMX with Instance and Request
//     already set for HRTIM1_A. */
//  HRTIM_StartDutyStream(&hdma_hrtim1_a, HRTIM_TIMERINDEX_TIMER_A);
//
//  /* Timer B stream: built here so no CubeMX change is needed. Both streams walk
//     the same hrtim_lut at one entry per period, so the two outputs always carry
//     the same duty, offset in time by HRTIM_PHASE_TICKS. */
//  hdma_hrtim1_b.Instance     = HRTIM_CHB_DMA_STREAM;
//  hdma_hrtim1_b.Init.Request = DMA_REQUEST_HRTIM_TIMER_B;
//  HRTIM_StartDutyStream(&hdma_hrtim1_b, HRTIM_TIMERINDEX_TIMER_B);
//
//  HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1);
//
//  /* The master carries the phase reference, so it has to run as well */
//  HAL_HRTIM_WaveformCountStart(&hhrtim, HRTIM_TIMERID_MASTER |
//                                        HRTIM_TIMERID_TIMER_A |
//                                        HRTIM_TIMERID_TIMER_B);
//
//  /* ---- Power stage bring-up sequence ------------------------------------ */
//  HAL_GPIO_WritePin(startup_GPIO_Port, startup_Pin, GPIO_PIN_SET);
//  HAL_Delay(5);
//
//  HAL_GPIO_WritePin(pb5_GPIO_Port, pb5_Pin, GPIO_PIN_SET);
//
//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)SineTable128upper,
//                    LUT_STEPS, DAC_ALIGN_12B_R);
//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t *)SineTable128lower,
//                    LUT_STEPS, DAC_ALIGN_12B_R);
//
//  HAL_COMP_Start(&hcomp1);
//  HAL_COMP_Start(&hcomp2);
//
//  HAL_TIM_Base_Start(&htim6);
//  HAL_Delay(1);
//  HAL_GPIO_WritePin(pb5_GPIO_Port, pb5_Pin, GPIO_PIN_RESET);
//
//  HAL_Delay(7);
//  HAL_GPIO_WritePin(startup_GPIO_Port, startup_Pin, GPIO_PIN_RESET);
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Supply configuration update enable
//  */
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
//
//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 480;
//  RCC_OscInitStruct.PLL.PLLP = 2;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
//  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief COMP1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_COMP1_Init(void)
//{
//
//  /* USER CODE BEGIN COMP1_Init 0 */
//
//  /* USER CODE END COMP1_Init 0 */
//
//  /* USER CODE BEGIN COMP1_Init 1 */
//
//  /* USER CODE END COMP1_Init 1 */
//  hcomp1.Instance = COMP1;
//  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH1;
//  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
//  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
//  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
//  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
//  hcomp1.Init.Mode = COMP_POWERMODE_MEDIUMSPEED;
//  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
//  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
//  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN COMP1_Init 2 */
//
//  /* USER CODE END COMP1_Init 2 */
//
//}
//
///**
//  * @brief COMP2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_COMP2_Init(void)
//{
//
//  /* USER CODE BEGIN COMP2_Init 0 */
//
//  /* USER CODE END COMP2_Init 0 */
//
//  /* USER CODE BEGIN COMP2_Init 1 */
//
//  /* USER CODE END COMP2_Init 1 */
//  hcomp2.Instance = COMP2;
//  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH2;
//  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
//  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
//  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
//  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
//  hcomp2.Init.Mode = COMP_POWERMODE_MEDIUMSPEED;
//  hcomp2.Init.WindowMode = COMP_WINDOWMODE_COMP1_INPUT_PLUS_COMMON;
//  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
//  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN COMP2_Init 2 */
//
//  /* USER CODE END COMP2_Init 2 */
//
//}
//
///**
//  * @brief DAC1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_DAC1_Init(void)
//{
//
//  /* USER CODE BEGIN DAC1_Init 0 */
//
//  /* USER CODE END DAC1_Init 0 */
//
//  DAC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN DAC1_Init 1 */
//
//  /* USER CODE END DAC1_Init 1 */
//
//  /** DAC Initialization
//  */
//  hdac1.Instance = DAC1;
//  if (HAL_DAC_Init(&hdac1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** DAC channel OUT1 config
//  */
//  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
//  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
//  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
//  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
//  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
//  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** DAC channel OUT2 config
//  */
//  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN DAC1_Init 2 */
//
//  /* USER CODE END DAC1_Init 2 */
//
//}
//
///**
//  * @brief HRTIM Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_HRTIM_Init(void)
//{
//
//  /* USER CODE BEGIN HRTIM_Init 0 */
//
//  /* USER CODE END HRTIM_Init 0 */
//
//  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
//  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
//  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
//  HRTIM_OutputCfgTypeDef pOutputCfg = {0};
//
//  /* USER CODE BEGIN HRTIM_Init 1 */
//
//  /* USER CODE END HRTIM_Init 1 */
//  hhrtim.Instance = HRTIM1;
//  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
//  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
//  if (HAL_HRTIM_Init(&hhrtim) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* The master timer is the phase reference: its period event launches TA1 and
//     its CMP1 event launches TB1 one HRTIM_PHASE_TICKS later. */
//  pTimeBaseCfg.Period = HRTIM_TA_PERIOD;
//  pTimeBaseCfg.RepetitionCounter = 0x00;
//  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
//  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
//  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
//  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
//  pTimerCfg.DMASrcAddress = 0x0000;
//  pTimerCfg.DMADstAddress = 0x0000;
//  pTimerCfg.DMASize = 0x1;
//  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
//  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
//  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
//  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
//  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
//  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
//  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
//  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
//  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Master CMP1 is the phase-shift instant for the second output */
//  pCompareCfg.CompareValue = HRTIM_PHASE_TICKS;
//  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* ---- Timer A: leading output TA1, launched by the master period ------- */
//  pTimeBaseCfg.Period = HRTIM_TA_PERIOD;
//  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
//  pTimerCfg.DMARequests = HRTIM_TIM_DMA_CMP1;
//  pTimerCfg.DMASrcAddress = (uint32_t)hrtim_lut;
//  pTimerCfg.DMADstAddress = (uint32_t)&hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR;
//  pTimerCfg.DMASize = HRTIM_LUT_SIZE;
//  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
//  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
//  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
//  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
//  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
//  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
//  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
//  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
//  /* Reset by the same master event that sets the output, so the counter reads 0
//     at the rising edge and CMP1 is the on-time in ticks with no wrap limit.
//     That reset, not the timer period, is what ends each cycle, so the preload
//     registers must also transfer on reset or the duty would freeze. */
//  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
//  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED;
//  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pCompareCfg.CompareValue = HRTIM_CMP_MIN;
//  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
//  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERPER;
//  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
//  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
//  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
//  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
//  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
//  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
//  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* ---- Timer B: same duty, launched HRTIM_PHASE_TICKS later ------------- */
//  pTimeBaseCfg.Period = HRTIM_TA_PERIOD;
//  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pTimerCfg.DMASrcAddress = (uint32_t)hrtim_lut;
//  pTimerCfg.DMADstAddress = (uint32_t)&hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR;
//  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP1;
//  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pCompareCfg.CompareValue = HRTIM_CMP_MIN;
//  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERCMP1;
//  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
//  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN HRTIM_Init 2 */
//
//  /* USER CODE END HRTIM_Init 2 */
//  HAL_HRTIM_MspPostInit(&hhrtim);
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 0;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 1199;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 600;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.BreakFilter = 0;
//  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
//  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
//  sBreakDeadTimeConfig.Break2Filter = 0;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//  HAL_TIM_MspPostInit(&htim1);
//
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 0;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 1199;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 600;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//  HAL_TIM_MspPostInit(&htim2);
//
//}
//
///**
//  * @brief TIM6 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM6_Init(void)
//{
//
//  /* USER CODE BEGIN TIM6_Init 0 */
//
//  /* USER CODE END TIM6_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM6_Init 1 */
//
//  /* USER CODE END TIM6_Init 1 */
//  htim6.Instance = TIM6;
//  htim6.Init.Prescaler = 0;
//  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim6.Init.Period = 374;
//  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM6_Init 2 */
//
//  /* USER CODE END TIM6_Init 2 */
//
//}
//
///**
//  * @brief TIM7 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM7_Init(void)
//{
//
//  /* USER CODE BEGIN TIM7_Init 0 */
//
//  /* USER CODE END TIM7_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM7_Init 1 */
//
//  /* USER CODE END TIM7_Init 1 */
//  htim7.Instance = TIM7;
//  htim7.Init.Prescaler = 0;
//  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim7.Init.Period = 65535;
//  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM7_Init 2 */
//
//  /* USER CODE END TIM7_Init 2 */
//
//}
//
///**
//  * Enable DMA controller clock
//  */
//static void MX_DMA_Init(void)
//{
//
//  /* DMA controller clock enable */
//  __HAL_RCC_DMA1_CLK_ENABLE();
//
//  /* DMA interrupt init */
//  /* DMA1_Stream0_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
//  /* DMA1_Stream1_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
//  /* DMA1_Stream2_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
//  /* DMA1_Stream3_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  /* USER CODE BEGIN MX_GPIO_Init_1 */
//
//  /* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOE_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(startup_GPIO_Port, startup_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, pb5_Pin|GPIO_PIN_8, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PF3 */
//  GPIO_InitStruct.Pin = GPIO_PIN_3;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : startup_Pin */
//  GPIO_InitStruct.Pin = startup_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(startup_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PD14 */
//  GPIO_InitStruct.Pin = GPIO_PIN_14;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PD15 */
//  GPIO_InitStruct.Pin = GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : pb5_Pin PB8 */
//  GPIO_InitStruct.Pin = pb5_Pin|GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /**/
//  HAL_I2CEx_EnableFastModePlus(SYSCFG_PMCR_I2C_PB8_FMP);
//
//  /* USER CODE BEGIN MX_GPIO_Init_2 */
//
//  /* USER CODE END MX_GPIO_Init_2 */
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//#ifdef USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
