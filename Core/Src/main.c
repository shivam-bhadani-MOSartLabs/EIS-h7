///* USER CODE BEGIN Header */
/////**
////  ******************************************************************************
////  * @file           : main.c
////  * @brief          : Main program body
////  ******************************************************************************
////  * @attention
////  *
////  * Copyright (c) 2026 STMicroelectronics.
////  * All rights reserved.
////  *
////  * This software is licensed under terms that can be found in the LICENSE file
////  * in the root directory of this software component.
////  * If no LICENSE file comes with this software, it is provided AS-IS.
////  *
////  ******************************************************************************
////  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "sine.h"
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
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;
//ADC_HandleTypeDef hadc2;
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
////////
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void PeriphCommonClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_DAC1_Init(void);
//static void MX_TIM6_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_TIM7_Init(void);
//static void MX_HRTIM_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_ADC2_Init(void);
///* USER CODE BEGIN PFP */
//////////uint32_t adc_buffer[1024] __attribute__((section(".AXI_SRAM")));
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//const uint32_t hrtim_lut[10] = {
//		96, 96, 96*2, 96*3, 96*4, 96*5, 96*6, 96*7, 96*8, 96*9
//};
//void user_delay(void){
//	for(int i=0;i<10000000;i++)
//		__asm("nop");
//}
//
//void Set_PWM_Carrier_Frequency(uint32_t target_hz)
//{
//    if (target_hz == 0) return;
//    uint32_t calculated_arr = (240000000UL / target_hz) - 1;
//    if (calculated_arr > 0xFFFF) calculated_arr = 0xFFFF;
//    TIM2->ARR = calculated_arr;
//    TIM2->EGR |= TIM_EGR_UG;
//}
//
//void Set_SPWM_Frequency(uint32_t target_hz)
//{
//    if (target_hz == 0) return;
//
//    // 240 MHz clock / 128 LUT steps
//    uint32_t calculated_arr = (240000000UL / (128 * target_hz)) - 1;
//
//    if (calculated_arr > 0xFFFF) {
//        calculated_arr = 0xFFFF; // Clamp to 16-bit max
//    }
//
//    TIM7->ARR = calculated_arr;
//    TIM7->EGR |= TIM_EGR_UG; // Force update
//}
//void Set_Sine_Frequency(uint32_t target_hz)
//{
//  // Prevent division by zero errors
//  if (target_hz == 0) return;
//
//  // H7 APB1 Timer Clock Bus baseline (240 MHz) divided by 128 LUT steps = 1,875,000
//  uint32_t calculated_arr = (240000000UL / (128 * target_hz)) - 1;
//
//  // Clamp checking: Ensure ARR fits in a 16-bit timer register (TIM6 is 16-bit)
//  if (calculated_arr > 0xFFFF) {
//      calculated_arr = 0xFFFF;
//  }
//
//  // Write directly to the timer's shadow auto-reload register
//  TIM6->ARR = calculated_arr;
//
//  /* Force an immediate generation event so the timer grabs the new ARR value
//     instantly rather than waiting for the previous long period to end */
//  TIM6->EGR |= TIM_EGR_UG;
//}
//
////uint8_t dac_index = 0;
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
//
//
//
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
////////
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* Configure the peripherals common clocks */
//  PeriphCommonClock_Config();
//
//  /* USER CODE BEGIN SysInit */
////////
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_TIM1_Init();
//  MX_DAC1_Init();
//  MX_TIM6_Init();
//  MX_TIM2_Init();
//  MX_TIM7_Init();
//  MX_HRTIM_Init();
//  MX_ADC1_Init();
//  MX_ADC2_Init();
//  /* USER CODE BEGIN 2 */
//  Set_Sine_Frequency(500);
////  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, SineTable128[0]);
////  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, SineTable128[0]);
////  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
////  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
////  SCB_CleanDCache_by_Addr((uint32_t*)SineTable128upper, sizeof(SineTable128upper));
////  SCB_CleanDCache_by_Addr((uint32_t*)SineTable128lower, sizeof(SineTable128lower));
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  Set_SPWM_Frequency(1000);
////  Set_PWM_Carrier_Frequency(500000);
////  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)spwm_lut, 128);
//
////
////    // 3. Manually start the DMA:
////    // Source = spwm_lut (Memory)
////    // Destination = TIM2->CCR1 (Duty Cycle Register)
//  if (HAL_DMA_Start(&hdma_hrtim1_a, (uint32_t)hrtim_lut, (uint32_t)&hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR, 10) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
////     3. Enable the physical pin output for TA1
//    HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TA1);
//
////     4. Start the internal high-speed counter
//    HAL_HRTIM_WaveformCountStart(&hhrtim, HRTIM_TIMERID_TIMER_A);
//
//  if (HAL_DMA_Start(&hdma_tim7_up, (uint32_t)spwm_lut, (uint32_t)&TIM2->CCR1, 128) != HAL_OK)
//    {
//        Error_Handler(); // If you hit this, your DMA configuration in CubeMX is invalid.
//    }
//
////    // 4. Tell TIM7 it is allowed to trigger the DMA
//  __HAL_TIM_ENABLE_DMA(&htim7, TIM_DMA_UPDATE);
////
////    // 5. Start the TIM7 pacing timer
//  HAL_TIM_Base_Start(&htim7);
//  HAL_GPIO_WritePin(startup_GPIO_Port, startup_Pin, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//  HAL_Delay(5);
//  HAL_GPIO_WritePin(pb5_GPIO_Port, pb5_Pin, GPIO_PIN_SET);
//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)SineTable128upper, 128, DAC_ALIGN_12B_R);
//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)SineTable128lower, 128, DAC_ALIGN_12B_R);
////  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // MOE handled internally
//  HAL_COMP_Start(&hcomp1);
//  HAL_COMP_Start(&hcomp2);
//
////  HAL_Delay(10);              // Let DAC output settle before COMP sees it
//  HAL_TIM_Base_Start(&htim6);
//  HAL_Delay(1);
//  HAL_GPIO_WritePin(pb5_GPIO_Port, pb5_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(pb5_GPIO_Port, pb5_Pin, GPIO_PIN_RESET);
//
//
//  HAL_Delay(7);
//  HAL_GPIO_WritePin(startup_GPIO_Port, startup_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_SET);
////  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
////  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//
//  // Kickstart the ADC in Interrupt mode!
////    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
////    {
////        /* Initialization Error */
////        Error_Handler();
////    }
//
//
//    // Start BOTH ADCs simultaneously using Dual-Mode DMA
//    // Note: Data arrives packed (ADC1 sample in lower 16 bits, ADC2 in upper 16 bits)
////    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buffer, 1024) != HAL_OK)
////    {
////        Error_Handler();
////    }
////    if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
////      {
////          Error_Handler();
////      }
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//
//
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
////	  __WFI();
////    HAL_GPIO_TogglePin(user_gpio_GPIO_Port,user_gpio_Pin);
////    HAL_GPIO_TogglePin(vstpin_GPIO_Port,vstpin_Pin);
////    user_delay();
////    HAL_GPIO_WritePin(user_gpio_GPIO_Port, user_gpio_Pin, GPIO_PIN_SET);
////    user_delay();
////    HAL_GPIO_WritePin(user_gpio_GPIO_Port, user_gpio_Pin, GPIO_PIN_RESET);
//	  /* 🟢 Write the next LUT voltage point to the internal DAC register */
////	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, SineTable128[dac_index]+H);
////	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, SineTable128[dac_index]-H);
//
//	      /* Advance the index wrapper */
////	      dac_index = (dac_index + 1) % 128;
//
//	      /* Delay pacing to match your 50Hz envelope target (156 microseconds) */
//	      /* For precise timing, wrapping this inside a simple basic timer interrupt (like TIM2) is ideal */
////	      for(int i = 0; i < 31; i++) {
////	              __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
////	              while(!__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE));
////	          }
//
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
//  * @brief Peripherals Common Clock Configuration
//  * @retval None
//  */
//void PeriphCommonClock_Config(void)
//{
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//  /** Initializes the peripherals clock
//  */
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
//  PeriphClkInitStruct.PLL2.PLL2M = 2;
//  PeriphClkInitStruct.PLL2.PLL2N = 50;
//  PeriphClkInitStruct.PLL2.PLL2P = 2;
//  PeriphClkInitStruct.PLL2.PLL2Q = 2;
//  PeriphClkInitStruct.PLL2.PLL2R = 2;
//  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
//  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
//  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
//  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief ADC1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC1_Init(void)
//{
//
//  /* USER CODE BEGIN ADC1_Init 0 */
//////
//  /* USER CODE END ADC1_Init 0 */
//
//  ADC_MultiModeTypeDef multimode = {0};
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC1_Init 1 */
//////
//  /* USER CODE END ADC1_Init 1 */
//
//  /** Common config
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
//  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
//  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc1.Init.LowPowerAutoWait = DISABLE;
//  hadc1.Init.ContinuousConvMode = DISABLE;
//  hadc1.Init.NbrOfConversion = 1;
//  hadc1.Init.DiscontinuousConvMode = DISABLE;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
//  hadc1.Init.OversamplingMode = DISABLE;
//  hadc1.Init.Oversampling.Ratio = 1;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure the ADC multi-mode
//  */
//  multimode.Mode = ADC_MODE_INDEPENDENT;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_10;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  sConfig.OffsetSignedSaturation = DISABLE;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC1_Init 2 */
//////
//  /* USER CODE END ADC1_Init 2 */
//
//}
//
///**
//  * @brief ADC2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC2_Init(void)
//{
//
//  /* USER CODE BEGIN ADC2_Init 0 */
//////
//  /* USER CODE END ADC2_Init 0 */
//
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC2_Init 1 */
//////
//  /* USER CODE END ADC2_Init 1 */
//
//  /** Common config
//  */
//  hadc2.Instance = ADC2;
//  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
//  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
//  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc2.Init.LowPowerAutoWait = DISABLE;
//  hadc2.Init.ContinuousConvMode = DISABLE;
//  hadc2.Init.NbrOfConversion = 1;
//  hadc2.Init.DiscontinuousConvMode = DISABLE;
//  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
//  hadc2.Init.OversamplingMode = DISABLE;
//  hadc2.Init.Oversampling.Ratio = 1;
//  if (HAL_ADC_Init(&hadc2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_10;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  sConfig.OffsetSignedSaturation = DISABLE;
//  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC2_Init 2 */
//////
//  /* USER CODE END ADC2_Init 2 */
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
////////
//  /* USER CODE END DAC1_Init 0 */
//
//  DAC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN DAC1_Init 1 */
////////
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
//  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
//  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
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
////////
//  /* USER CODE END HRTIM_Init 1 */
//  hhrtim.Instance = HRTIM1;
//  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
//  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
//  if (HAL_HRTIM_Init(&hhrtim) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pTimeBaseCfg.Period = 0xFF;
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
//  pTimeBaseCfg.Period = 960;
//  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
//  pTimerCfg.DMARequests = HRTIM_TIM_DMA_UPD;
//  pTimerCfg.DMASrcAddress = (uint32_t)hrtim_lut;
//  pTimerCfg.DMADstAddress = (uint32_t)&hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR;
//  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
//  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
//  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
//  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
//  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
//  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
//  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
//  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
//  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
//  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
//  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pCompareCfg.CompareValue = 480;
//  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
//  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
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
//  /* USER CODE BEGIN HRTIM_Init 2 */
////////
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
////////
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
////////
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
////////
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
////////
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
////////
//  /* USER CODE END TIM6_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM6_Init 1 */
////////
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
////////
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
////////
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
////////
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
//  /*Configure GPIO pin : PB0 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
////////
//  /* USER CODE END MX_GPIO_Init_2 */
//}
//
///* USER CODE BEGIN 4 */
////////
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
