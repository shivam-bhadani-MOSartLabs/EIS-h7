///**
//  ******************************************************************************
//  * @file    adc_rate_probe.c
//  * @brief   ONE ADC, no DAC, 8-bit, PF3-gated one-shot capture that MEASURES
//  *          its own sample rate on the target. NUCLEO-H753ZI.
//  *
//  * Standalone: has its own main(). Exclude adc_sine_pwm.c, adc_dac_echo.c,
//  * adc_dac_passthru.c and adc_dac_single.c from the build while using this one.
//  *
//  *   PC0    input,  ADC1_INP10 (Arduino A1)
//  *   PF3    output, HIGH for exactly the capture window - the scope marker
//  *
//  * WHY THIS FILE EXISTS
//  * --------------------
//  * The previous version derived the sample rate from the configuration - TIM8
//  * period, ADC clock, cycles per conversion - and the derivation said 4.0 MSPS
//  * while the captured waveform said 2.0. A derivation cannot settle that.
//  *
//  * So this version times itself. TIM2 is a free-running 32-bit counter at
//  * 240 MHz; its value is latched when the capture is armed and again when the
//  * DMA has taken the last sample, and the elapsed tick count goes into the
//  * descriptor. The host then computes
//  *
//  *     measured rate = timer_clk_hz * sample_count / elapsed_ticks
//  *
//  * which is the real, achieved conversion rate to about 1 part in sample_count.
//  * No scope, no assumptions.
//  *
//  * TWO MODES
//  * ---------
//  *   CAP_MODE_TRIGGERED   TIM8 TRGO paces the ADC at SAMPLE_RATE_HZ. Use this
//  *                        to check whether a chosen rate is actually achieved.
//  *   CAP_MODE_FREERUN     No timer. ContinuousConvMode, conversions back to
//  *                        back at whatever the hardware can do. Use this to
//  *                        find the CEILING - the measured rate IS the answer
//  *                        to "what is the maximum conversion rate".
//  *
//  * Run FREERUN once, read the measured rate, then set SAMPLE_RATE_HZ below it
//  * and run TRIGGERED to confirm the trigger is honoured every time.
//  *
//  * WHY A TRIGGERED RATE COMES OUT EXACTLY HALF
//  * -------------------------------------------
//  * A trigger that arrives while a regular conversion is still in progress is
//  * IGNORED - the ADC does not queue it. So if a conversion occupies even a
//  * nanosecond more than the trigger period, every second trigger is discarded
//  * and the achieved rate is exactly half the requested one. Exactly half, with
//  * every register reading exactly right. That is the failure mode that made
//  * 4 MSPS come out as 2 MSPS, and the only cure is real margin between the
//  * conversion time and the trigger period - which FREERUN measures for you.
//  *
//  * CONVERSION TIME - WHY 8 BITS IS SO MUCH FASTER
//  * ----------------------------------------------
//  * A conversion is sampling + tSAR, and tSAR depends on the resolution:
//  *
//  *       bits    tSAR       min conversion (with 1.5 cycles of sampling)
//  *       16      8.5 cyc    10.0 cycles
//  *       14      7.5        9.0
//  *       12      6.5        8.0
//  *       10      5.5        7.0
//  *        8      4.5        6.0
//  *
//  * At the 50 MHz ADC clock set up below, 6.0 cycles is 120 ns, so 8 bits has a
//  * theoretical ceiling around 8.3 MSPS against 12 bits' 6.25 MSPS. 4 MSPS at
//  * 8 bits is not marginal at all: the budget is 250 ns = 12.5 ADC cycles for a
//  * conversion that needs 6.0, so 6.5 cycles of the budget are spare and can be
//  * spent on a longer sampling aperture instead (see ADC_SAMPLETIME).
//  *
//  * SILICON REVISION MATTERS FOR THE ADC CLOCK
//  * ------------------------------------------
//  * Rev V parts take adc_ker_ck up to 50 MHz; rev Y is capped at 36 MHz. The
//  * descriptor reports REV_ID (0x2003 = rev V, 0x1003 = rev Y) so the host can
//  * tell you whether ADC_CLK_HZ below is legal for the part on the board. If it
//  * says rev Y, set ADC_CLK_HZ to 33333333 (PLL2P = 12).
//  *
//  * DATA FORMAT
//  * -----------
//  * The DMA moves halfwords, always, whatever the resolution: the ADC data
//  * register is 16-bit and 2 bytes per sample costs nothing at these rates. At
//  * 8 bits the codes are 0..255 right-aligned in the halfword, so one LSB is
//  * 3300 / 256 = 12.89 mV. Anything above 0xFF means the resolution did not
//  * take effect.
//  *
//  * READING THE BUFFER
//  * ------------------
//  * Descriptor at 0x30000000 (64 bytes), samples from 0x30000040. Or run
//  * adc_capture.py, which resets the target, reads the descriptor and prints
//  * both the requested and the MEASURED rate.
//  *
//  * DEBUG ACCESS
//  * ------------
//  * The program ends in a plain busy loop, not __WFI(), and it sets the DBGMCU
//  * sleep/stop/standby debug bits. A CPU in CSleep gates its own debug clock,
//  * and a hot-plug connect to a sleeping H7 fails with "Unable to read device id
//  * from ROM table" - which is a connection failure, not a capture failure. This
//  * program stays awake so the buffer can always be read.
//  *
//  * BUILD
//  * -----
//  * stm32h7xx_hal_conf.h needs HAL_ADC_MODULE_ENABLED. stm32h7xx_hal_adc.c and
//  * stm32h7xx_hal_adc_ex.c must be in the project. HAL_DAC_MODULE_ENABLED is no
//  * longer needed - there is no DAC in this program.
//  *
//  * Resources: ADC1, DMA2_Stream0, TIM2 (timebase), TIM8 (trigger, TRIGGERED
//  *            mode only), PLL2P, PF3, MPU region 0, 64 + 2*CAP_SAMPLES bytes
//  *            at the base of SRAM1.
//  ******************************************************************************
//  */
//
//#include "stm32h7xx_hal.h"
//#include <stdbool.h>
//
///* ==========================================================================
// * Parameters
// * ========================================================================== */
//
//#define CAP_MODE_TRIGGERED      0U
//#define CAP_MODE_FREERUN        1U
//
///* Start with FREERUN to find the ceiling, then switch to TRIGGERED. */
//#define CAP_MODE                CAP_MODE_FREERUN
//
///* 8, 10, 12, 14 or 16. Sets both the ADC resolution and the descriptor field
//   the host uses to scale codes to volts. */
//#define ADC_RES_BITS           16U
//
///* adc_ker_ck. 50 MHz is the rev V ceiling; use 33333333 on rev Y.
//   Must divide 400 MHz (the PLL2 VCO set up below) exactly. */
//#define ADC_CLK_HZ              50000000UL
//
///* Sampling time. 1.5 cycles is the minimum and gives the shortest conversion;
//   lengthen it if the source impedance needs a wider aperture and the rate
//   budget allows. At 50 MHz: 1.5 cyc = 30 ns, 2.5 cyc = 50 ns, 8.5 cyc = 170 ns.
//   8-bit settling needs about 6 time constants, and the internal path is
//   roughly 50 ohm + 8 pF, so 30 ns tolerates about 600 ohm of source
//   impedance and 170 ns about 3.5 kilohm. */
//#define ADC_SAMPLETIME          ADC_SAMPLETIME_1CYCLE_5
//#define ADC_SAMPLE_HALFCYCLES   3U      /* keep in step: 1.5 cyc = 3 halves */
//
///* TRIGGERED mode only: one TIM8 trigger = one conversion, so this IS the
//   requested rate. Must divide 240 MHz exactly. 240/30 = 8.0 MSPS,
//   240/32 = 7.5, 240/40 = 6.0, 240/48 = 5.0, 240/60 = 4.0. */
//#define SAMPLE_RATE_HZ          4000000UL
//
///* Window length. In TRIGGERED mode this is the real PF3 width; in FREERUN the
//   sample count is what matters and PF3 comes out at whatever the hardware
//   takes, which is the measurement. Longer windows measure the rate more
//   precisely - the error is about one sample in sample_count. */
//#define CAPTURE_US              1000U
//
///* 240 MHz timer kernel clock: 480 MHz SYSCLK, HCLK /2, APB /2, timer x2. */
//#define TIMER_CLK_HZ            240000000UL
//
//#define CAP_SAMPLES             ((uint32_t)(((uint64_t)CAPTURE_US * \
//                                             SAMPLE_RATE_HZ) / 1000000ULL))
//
///* ==========================================================================
// * Derived timing, checked at compile time
// * ========================================================================== */
//
///* tSAR in half ADC cycles, by resolution. */
//#if   ADC_RES_BITS == 16U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_16B
//  #define ADC_TSAR_HALFCYCLES   17U
//#elif ADC_RES_BITS == 14U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_14B
//  #define ADC_TSAR_HALFCYCLES   15U
//#elif ADC_RES_BITS == 12U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_12B
//  #define ADC_TSAR_HALFCYCLES   13U
//#elif ADC_RES_BITS == 10U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_10B
//  #define ADC_TSAR_HALFCYCLES   11U
//#elif ADC_RES_BITS == 8U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_8B
//  #define ADC_TSAR_HALFCYCLES   9U
//#else
//  #error "ADC_RES_BITS must be 8, 10, 12, 14 or 16"
//#endif
//
//#define ADC_CONV_HALFCYCLES     (ADC_SAMPLE_HALFCYCLES + ADC_TSAR_HALFCYCLES)
//
///* Nominal conversion time in picoseconds. No casts: this is evaluated by
//   the preprocessor in the #if below, and #if rejects a cast. */
//#define ADC_CONV_PS             ((ADC_CONV_HALFCYCLES * 1000000000000ULL) \
//                                 / (2ULL * ADC_CLK_HZ))
//
///* PLL2 VCO is fixed at 400 MHz below, so P follows from the wanted clock. */
//#define PLL2_VCO_HZ             400000000UL
//#define PLL2P_DIV               (PLL2_VCO_HZ / ADC_CLK_HZ)
//
//_Static_assert(PLL2_VCO_HZ % ADC_CLK_HZ == 0U,
//               "ADC_CLK_HZ must divide the 400 MHz PLL2 VCO exactly - "
//               "50000000 (P=8), 40000000 (P=10) or 33333333 is not exact, "
//               "use 400000000/P");
//_Static_assert(PLL2P_DIV >= 2U && PLL2P_DIV <= 128U, "PLL2P out of range");
//_Static_assert(ADC_CLK_HZ <= 50000000UL,
//               "adc_ker_ck above the rev V 50 MHz maximum");
//
//_Static_assert(TIMER_CLK_HZ % SAMPLE_RATE_HZ == 0U,
//               "SAMPLE_RATE_HZ must divide 240 MHz exactly or TIM8 cannot "
//               "produce it - the achieved rate would be 240e6/round(ARR+1)");
//
//_Static_assert((CAP_SAMPLES * 2U) + 64U <= 32768U,
//               "capture window exceeds the 32 KB non-cacheable MPU region - "
//               "raise Mpu_Config()'s region size to match");
//_Static_assert(CAP_SAMPLES >= 16U, "capture window too short to be useful");
//
///* Only a warning's worth of information, and only meaningful in TRIGGERED
//   mode: if the nominal conversion does not fit the trigger period the rate
//   WILL come out as a submultiple. It is not an error because the whole point
//   of this program is to measure what actually happens. */
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//  #if (ADC_CONV_PS > (1000000000000ULL / SAMPLE_RATE_HZ))
//    #warning "nominal conversion time exceeds the trigger period - expect the \
//measured rate to come out at a submultiple of SAMPLE_RATE_HZ"
//  #endif
//#endif
//
///* ==========================================================================
// * Buffer layout
// * ==========================================================================
// * SRAM1 (D2 domain), because the CubeIDE linker script puts .bss in DTCMRAM at
// * 0x20000000, which no DMA controller can reach. Placed by hand so the project
// * needs no linker change - nothing else may use the bottom of SRAM1.
// *
// * The CPU reads it and the DMA writes it, so the MPU marks the region
// * non-cacheable (Mpu_Config, before the D-cache is enabled). Without that a
// * read would return stale cache lines instead of conversions.
// */
//#define CAP_BASE_ADDR           0x30000000UL
//#define CAP_HDR_ADDR            (CAP_BASE_ADDR)
//#define CAP_DATA_ADDR           (CAP_BASE_ADDR + 64UL)
//
//#define CAP_MAGIC               0xADC0CAFEUL
//#define CAP_VERSION             3UL
//
//#define CAP_STATUS_IDLE         0UL
//#define CAP_STATUS_RUNNING      1UL
//#define CAP_STATUS_COMPLETE     2UL
//#define CAP_STATUS_TIMEOUT      3UL
//
///* 64 bytes, which keeps the sample array 32-byte aligned. Words 0..7 are
//   unchanged from version 2; everything the rate measurement needs is new. */
//typedef struct
//{
//  uint32_t magic;             /* CAP_MAGIC once the descriptor is valid      */
//  uint32_t version;           /* 3                                           */
//  uint32_t sample_rate_hz;    /* REQUESTED rate; 0 in FREERUN mode           */
//  uint32_t sample_count;
//  uint32_t pulse_us;          /* requested window                            */
//  uint32_t adc_bits;
//  uint32_t vref_mv;
//  uint32_t status;            /* CAP_STATUS_*                                */
//  uint32_t timer_clk_hz;      /* TIM2 counter clock                          */
//  uint32_t elapsed_ticks;     /* TIM2 ticks the capture actually took        */
//  uint32_t adc_clk_hz;        /* adc_ker_ck, read back from the RCC          */
//  uint32_t mode;              /* CAP_MODE_*                                  */
//  uint32_t sample_bytes;      /* 2                                           */
//  uint32_t rev_id;            /* 0x2003 rev V, 0x1003 rev Y                  */
//  uint32_t adc_isr;           /* ADC1 ISR at the end - bit 4 is OVR          */
//  uint32_t conv_halfcycles;   /* nominal conversion, in half ADC cycles      */
//} cap_hdr_t;
//
//_Static_assert(sizeof(cap_hdr_t) == 64U, "descriptor must stay 64 bytes");
//
//static volatile cap_hdr_t * const cap_hdr = (volatile cap_hdr_t *)CAP_HDR_ADDR;
//static volatile uint16_t  * const cap_buf = (volatile uint16_t *)CAP_DATA_ADDR;
//
///* PF3: high for exactly the capture window. */
//#define GATE_PORT               GPIOF
//#define GATE_PIN                GPIO_PIN_3
//
//static ADC_HandleTypeDef hadc1;
//static DMA_HandleTypeDef hdma_adc;   /* DMA2_Stream0: ADC1 DR -> cap_buf */
//static TIM_HandleTypeDef htim2;      /* 32-bit timebase for the measurement */
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//static TIM_HandleTypeDef htim8;      /* sample trigger */
//#endif
//
//void Error_Handler(void);
//
///* ==========================================================================
// * MPU
// * ========================================================================== */
//
//static void Mpu_Config(void)
//{
//  MPU_Region_InitTypeDef mpu = {0};
//
//  HAL_MPU_Disable();
//
//  mpu.Enable           = MPU_REGION_ENABLE;
//  mpu.Number           = MPU_REGION_NUMBER0;
//  mpu.BaseAddress      = CAP_BASE_ADDR;
//  mpu.Size             = MPU_REGION_SIZE_32KB;
//  mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
//  mpu.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
//  mpu.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
//  mpu.IsShareable      = MPU_ACCESS_SHAREABLE;
//  mpu.TypeExtField     = MPU_TEX_LEVEL1;
//  mpu.SubRegionDisable = 0x00;
//  mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
//  HAL_MPU_ConfigRegion(&mpu);
//
//  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
//}
//
///* ==========================================================================
// * Clocks
// * ========================================================================== */
//
//static void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef osc = {0};
//  RCC_ClkInitTypeDef clk = {0};
//
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
//  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  osc.HSEState       = RCC_HSE_BYPASS;
//  osc.PLL.PLLState   = RCC_PLL_ON;
//  osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
//  osc.PLL.PLLM       = 4;
//  osc.PLL.PLLN       = 480;
//  osc.PLL.PLLP       = 2;
//  osc.PLL.PLLQ       = 4;
//  osc.PLL.PLLR       = 2;
//  osc.PLL.PLLRGE     = RCC_PLL1VCIRANGE_1;
//  osc.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
//  osc.PLL.PLLFRACN   = 0;
//  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
//                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
//                     | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
//  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
//  clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
//  clk.AHBCLKDivider  = RCC_HCLK_DIV2;
//  clk.APB3CLKDivider = RCC_APB3_DIV2;
//  clk.APB1CLKDivider = RCC_APB1_DIV2;
//  clk.APB2CLKDivider = RCC_APB2_DIV2;
//  clk.APB4CLKDivider = RCC_APB4_DIV2;
//  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief PLL2P at ADC_CLK_HZ, routed to the ADC.
//  *
//  * HSE 8 MHz / M 4 = 2 MHz reference, N 200 -> 400 MHz VCO, P from
//  * PLL2P_DIV. At the default 50 MHz one ADC cycle is 20 ns, so a 6-cycle
//  * 8-bit conversion is 120 ns. PLL1 is untouched.
//  *
//  * HAL_ADC_Init() reads this clock back and sets ADC_CR BOOST from it, which
//  * is mandatory above 25 MHz on rev V - so the PLL2 has to be configured
//  * BEFORE Adc_Config(), not after.
//  */
//static void AdcClock_Config(void)
//{
//  RCC_PeriphCLKInitTypeDef p = {0};
//
//  p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
//  p.PLL2.PLL2M      = 4;
//  p.PLL2.PLL2N      = 200;
//  p.PLL2.PLL2P      = PLL2P_DIV;
//  p.PLL2.PLL2Q      = 2;
//  p.PLL2.PLL2R      = 2;
//  p.PLL2.PLL2RGE    = RCC_PLL2VCIRANGE_1;
//  p.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
//  p.PLL2.PLL2FRACN  = 0;
//  p.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
//
//  if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///* ==========================================================================
// * Peripherals
// * ========================================================================== */
//
//static void Gpio_Config(void)
//{
//  GPIO_InitTypeDef g = {0};
//
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//
//  /* PC0 = ADC1_INP10. */
//  g.Pin  = GPIO_PIN_0;
//  g.Mode = GPIO_MODE_ANALOG;
//  g.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &g);
//
//  /* PF3 = the capture-window marker. Driven low BEFORE being made an output,
//     so it cannot glitch high while it is being configured. MEDIUM speed is
//     plenty for a window of microseconds and rings far less than VERY_HIGH. */
//  HAL_GPIO_WritePin(GATE_PORT, GATE_PIN, GPIO_PIN_RESET);
//  g.Pin       = GATE_PIN;
//  g.Mode      = GPIO_MODE_OUTPUT_PP;
//  g.Pull      = GPIO_NOPULL;
//  g.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//  g.Alternate = 0;
//  HAL_GPIO_Init(GATE_PORT, &g);
//}
//
///**
//  * @brief The one stream: ADC1 DR into the buffer, one shot.
//  *
//  * HALFWORD at both ends whatever the resolution - the ADC data register is
//  * 16-bit and the codes are right-aligned in it. NORMAL mode, not circular:
//  * the window fills once and stops, which is what makes the record coherent
//  * and what lets NDTR mark the exact end of the capture for the timing
//  * measurement.
//  */
//static void Dma_Config(void)
//{
//  __HAL_RCC_DMA2_CLK_ENABLE();
//
//  hdma_adc.Instance                 = DMA2_Stream0;
//  hdma_adc.Init.Request             = DMA_REQUEST_ADC1;
//  hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
//  hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
//  hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
//  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//  hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
//  hdma_adc.Init.Mode                = DMA_NORMAL;
//  hdma_adc.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
//  hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
//  if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc);
//
//  /* HAL_ADC_Start_DMA() arms a transfer-complete interrupt, so the vector has
//     to be serviced. It only has to be acknowledged. */
//  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
//}
//
///**
//  * @brief ADC1 on PC0, one channel, DMA one-shot.
//  *
//  * TRIGGERED: external trigger TIM8 TRGO, rising edge, ContinuousConvMode off,
//  * so each trigger starts exactly one conversion.
//  *
//  * FREERUN: software start with ContinuousConvMode on, so one ADSTART begins a
//  * chain of back-to-back conversions at the hardware maximum. The DMA takes
//  * CAP_SAMPLES of them and stops; conversions past that point overwrite DR and
//  * set OVR, which is harmless and is why the ADC is stopped straight after.
//  */
//static void Adc_Config(void)
//{
//  ADC_ChannelConfTypeDef ch = {0};
//  ADC_MultiModeTypeDef   mm = {0};
//
//  __HAL_RCC_ADC12_CLK_ENABLE();
//
//  hadc1.Instance                      = ADC1;
//  hadc1.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
//  hadc1.Init.Resolution               = ADC_RESOLUTION;
//  hadc1.Init.ScanConvMode             = ADC_SCAN_DISABLE;
//  hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
//  hadc1.Init.LowPowerAutoWait         = DISABLE;
//  hadc1.Init.NbrOfConversion          = 1;
//  hadc1.Init.DiscontinuousConvMode    = DISABLE;
//#if (CAP_MODE == CAP_MODE_FREERUN)
//  hadc1.Init.ContinuousConvMode       = ENABLE;
//  hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
//  hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
//#else
//  hadc1.Init.ContinuousConvMode       = DISABLE;
//  hadc1.Init.ExternalTrigConv         = ADC_EXTERNALTRIG_T8_TRGO;
//  hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
//#endif
//  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
//  hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
//  hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
//  hadc1.Init.OversamplingMode         = DISABLE;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Independent mode, set explicitly. The DUAL field lives in the ADC12
//     COMMON register, which HAL_ADC_Init() does not touch - so if a previous
//     image left dual interleaved mode set, it would still be set here and the
//     DMA would be fed from CDR instead of DR. */
//  mm.Mode             = ADC_MODE_INDEPENDENT;
//  mm.DualModeData     = ADC_DUALMODEDATAFORMAT_DISABLED;
//  mm.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mm) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Calibration has to run with the ADC disabled, so before any start. */
//  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
//                                  ADC_SINGLE_ENDED) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  ch.Channel      = ADC_CHANNEL_10;
//  ch.Rank         = ADC_REGULAR_RANK_1;
//  ch.SamplingTime = ADC_SAMPLETIME;
//  ch.SingleDiff   = ADC_SINGLE_ENDED;
//  ch.OffsetNumber = ADC_OFFSET_NONE;
//  ch.Offset       = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief TIM2: free-running 32-bit counter at 240 MHz, the measurement clock.
//  *
//  * Nothing drives it and nothing is driven by it. 4.166 ns per tick and a
//  * 32-bit count, so it wraps every 17.9 seconds - far longer than any capture,
//  * and the subtraction is done in unsigned arithmetic so a wrap inside the
//  * window would still give the right elapsed count.
//  */
//static void Timebase_Config(void)
//{
//  __HAL_RCC_TIM2_CLK_ENABLE();
//
//  htim2.Instance               = TIM2;
//  htim2.Init.Prescaler         = 0;
//  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
//  htim2.Init.Period            = 0xFFFFFFFFUL;
//  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
///**
//  * @brief TIM8: the sample trigger. TRGO on update, no output pin.
//  *
//  * The counter runs at 240 MHz, ARR is 240 MHz / SAMPLE_RATE_HZ - 1, and every
//  * counter overflow raises an update event. MMS = 010 (TIM_TRGO_UPDATE) routes
//  * that update event onto the internal tim8_trgo line, which the ADC's EXTSEL
//  * has selected. RepetitionCounter is 0 - on an advanced timer a non-zero RCR
//  * would only pass every (RCR+1)th update through, and that alone would halve
//  * or quarter the sample rate with everything else looking correct.
//  */
//static void TrigTimer_Config(void)
//{
//  TIM_MasterConfigTypeDef m = {0};
//
//  __HAL_RCC_TIM8_CLK_ENABLE();
//
//  htim8.Instance               = TIM8;
//  htim8.Init.Prescaler         = 0;
//  htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
//  htim8.Init.Period            = (TIMER_CLK_HZ / SAMPLE_RATE_HZ) - 1U;
//  htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htim8.Init.RepetitionCounter = 0;
//  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  m.MasterOutputTrigger  = TIM_TRGO_UPDATE;
//  m.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  m.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &m) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//#endif /* CAP_MODE_TRIGGERED */
//
//void DMA2_Stream0_IRQHandler(void)
//{
//  HAL_DMA_IRQHandler(&hdma_adc);
//}
//
///* ==========================================================================
// * main
// * ========================================================================== */
//
//int main(void)
//{
//  /* Before the D-cache, so the capture buffer is never cached. */
//  Mpu_Config();
//
//  SCB_EnableICache();
//  SCB_EnableDCache();
//
//  HAL_Init();
//  SystemClock_Config();
//
//  /* Keep the debug unit alive even if the CPU ever does sleep, so a hot-plug
//     connect can always read the buffer. A sleeping H7 gates its own debug
//     clock and the programmer then reports "Unable to read device id from ROM
//     table" - a connection failure that looks like a capture failure. */
//  DBGMCU->CR |= DBGMCU_CR_DBG_SLEEPD1 | DBGMCU_CR_DBG_STOPD1
//              | DBGMCU_CR_DBG_STANDBYD1;
//
//  AdcClock_Config();
//  Gpio_Config();
//  Dma_Config();
//  Adc_Config();
//  Timebase_Config();
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//  TrigTimer_Config();
//#endif
//
//  /* ---- describe the capture, so the host does not have to be told -------
//     Written before the samples, so a reader that sees CAP_MAGIC can trust the
//     fields; status says whether the samples are finished. */
//  cap_hdr->magic           = CAP_MAGIC;
//  cap_hdr->version         = CAP_VERSION;
//#if (CAP_MODE == CAP_MODE_FREERUN)
//  cap_hdr->sample_rate_hz  = 0U;          /* unknown by definition */
//#else
//  cap_hdr->sample_rate_hz  = SAMPLE_RATE_HZ;
//#endif
//  cap_hdr->sample_count    = CAP_SAMPLES;
//  cap_hdr->pulse_us        = CAPTURE_US;
//  cap_hdr->adc_bits        = ADC_RES_BITS;
//  cap_hdr->vref_mv         = 3234U;
//  cap_hdr->status          = CAP_STATUS_RUNNING;
//  cap_hdr->timer_clk_hz    = TIMER_CLK_HZ;
//  cap_hdr->elapsed_ticks   = 0U;
//  cap_hdr->adc_clk_hz      = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
//  cap_hdr->mode            = CAP_MODE;
//  cap_hdr->sample_bytes    = 2U;
//  cap_hdr->rev_id          = HAL_GetREVID();
//  cap_hdr->adc_isr         = 0U;
//  cap_hdr->conv_halfcycles = ADC_CONV_HALFCYCLES;
//
//  /* Mid-scale, so a half-finished or failed capture is obvious rather than
//     looking like real data sitting at zero. */
//  for (uint32_t i = 0U; i < CAP_SAMPLES; i++)
//  {
//    cap_buf[i] = (uint16_t)(1U << (ADC_RES_BITS - 1U));
//  }
//
//  /* ---- arm, open the window, and time it -------------------------------- */
//  GATE_PORT->BSRR = (uint32_t)GATE_PIN;              /* PF3 HIGH */
//
//#if (CAP_MODE == CAP_MODE_FREERUN)
//  /* Start_DMA enables the ADC, waits for ADRDY and sets ADSTART, so the
//     conversions begin inside the call - latch the counter immediately after
//     it returns. The residual offset is the handful of CPU cycles between
//     ADSTART and the latch, tens of nanoseconds, spread over CAP_SAMPLES. */
//  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)CAP_DATA_ADDR,
//                        CAP_SAMPLES) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  uint32_t tick0 = TIM2->CNT;
//#else
//  /* The stream is armed first so it is waiting when the first trigger
//     arrives, then the counter is latched, then TIM8 is released - so the
//     measured interval starts at the same moment the trigger train does.
//     Note the first trigger comes one full TIM8 period after the start, which
//     biases the measured rate low by one sample in CAP_SAMPLES. */
//  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)CAP_DATA_ADDR,
//                        CAP_SAMPLES) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  uint32_t tick0 = TIM2->CNT;
//  if (HAL_TIM_Base_Start(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//#endif
//
//  /* ---- wait for the one-shot to fill -----------------------------------
//     NDTR counts down to zero as the stream completes; in NORMAL mode it stops
//     there rather than reloading. The timeout exists only so a wiring or clock
//     fault cannot wedge the program. */
//  uint32_t t0 = HAL_GetTick();
//  while ((DMA2_Stream0->NDTR != 0U) && ((HAL_GetTick() - t0) < 500U))
//  {
//    /* spin */
//  }
//  uint32_t tick1 = TIM2->CNT;
//
//  GATE_PORT->BSRR = (uint32_t)GATE_PIN << 16;        /* PF3 LOW */
//
//  bool complete = (DMA2_Stream0->NDTR == 0U);
//
//  cap_hdr->adc_isr = ADC1->ISR;
//  (void)HAL_ADC_Stop_DMA(&hadc1);
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//  (void)HAL_TIM_Base_Stop(&htim8);
//#endif
//
//  /* Unsigned subtraction, so a counter wrap inside the window is still the
//     correct elapsed count. */
//  cap_hdr->elapsed_ticks = (uint32_t)(tick1 - tick0);
//  cap_hdr->status        = complete ? CAP_STATUS_COMPLETE : CAP_STATUS_TIMEOUT;
//
//  /* Done. The samples are at CAP_DATA_ADDR, the descriptor at CAP_HDR_ADDR,
//     and the measured rate is timer_clk_hz * sample_count / elapsed_ticks.
//     Reset the target for another capture - adc_capture.py does that.
//
//     A busy loop, deliberately not __WFI(): the debug unit must stay reachable
//     for the host to read the buffer out of a running target. */
//  while (1)
//  {
//    __NOP();
//  }
//}
//
//void Error_Handler(void)
//{
//  __disable_irq();
//  while (1)
//  {
//  }
//}
//
//#ifdef USE_FULL_ASSERT
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  (void)file;
//  (void)line;
//}
//#endif /* USE_FULL_ASSERT */
