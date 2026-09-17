///**
//  ******************************************************************************
//  * @file    adc_sine_comp.c
//  * @brief   ADC1 in continuous conversion at its measured maximum, 10-bit,
//  *          against a DDS sine re-evaluated at 16 MHz, driving PB5.
//  *          NUCLEO-H753ZI.
//  *
//  * Standalone: has its own main(). Exclude adc_rate_probe.c, adc_dac_single.c,
//  * adc_sine_pwm.c, adc_dac_echo.c and adc_dac_passthru.c from the build.
//  *
//  *   PC0    input,  ADC1_INP10 (Arduino A1), 0..3 V
//  *   PB5    output, the hysteretic decision. REMOVE PB5 from the CubeMX GPIO
//  *          init - this file owns it.
//  *
//  * THE LAW
//  * -------
//  *     ADC in  >  sine + threshold   ->  PB5 LOW
//  *     ADC in  <  sine - threshold   ->  PB5 HIGH
//  *     otherwise                     ->  PB5 holds
//  *
//  * Evaluated every DECISION_RATE_HZ (16 MHz nominal, 62.5 ns), against the
//  * newest value the ADC has in its data register.
//  *
//  * WHAT 16 MHz BUYS, AND WHAT IT DOES NOT
//  * --------------------------------------
//  * The ADC in continuous 10-bit mode produces a new sample roughly every
//  * 280 ns (measured at boot - see below), so about four consecutive decisions
//  * see the SAME input value. The 16 MHz rate therefore buys two things:
//  *
//  *   - the sine is resolved to 62.5 ns, so a crossing is acted on at the
//  *     moment the SINE reaches the input, not up to one ADC period later;
//  *   - the decision latency after a fresh conversion is at most 62.5 ns.
//  *
//  * It does not buy input bandwidth. Total worst-case latency from a real input
//  * crossing to a PB5 edge is one conversion (~280 ns) plus one decision period
//  * (62.5 ns) plus the GPIO delay - about 350 ns. Maximum PB5 switching rate is
//  * one edge per decision, so 8 MHz square at the absolute limit.
//  *
//  * EVERY RATE HERE IS MEASURED, NOT DERIVED
//  * ----------------------------------------
//  * Two numbers that a derivation gets wrong on this part:
//  *
//  *   - the ADC conversion rate. Measured on this board, a nominal 6-cycle
//  *     8-bit conversion at a 50 MHz kernel clock took 240.4 ns, i.e. 12.0
//  *     cycles - exactly twice the datasheet cycle count. So a 10-bit
//  *     conversion (7 nominal cycles) is expected around 280 ns / 3.6 MSPS,
//  *     and the firmware measures it at boot rather than assuming it.
//  *   - the decision loop rate. 16 MHz means 30 CPU cycles per decision at
//  *     480 MHz, and the loop body includes a peripheral read of ADC1->DR
//  *     whose latency is not knowable from the source. The loop counts its own
//  *     iterations against TIM2 and publishes the achieved rate.
//  *
//  * Both land in the status block at 0x30000000, readable while the target
//  * runs. ctrl_status.py prints them.
//  *
//  * THE SINE CANNOT DRIFT
//  * ---------------------
//  * The phase accumulator is advanced by PHASE_INC once per decision, which is
//  * cheap but only as accurate as the loop rate. So the phase is re-derived
//  * from TIM2 - a free-running 32-bit counter on the 240 MHz crystal-derived
//  * kernel clock - every RESYNC_DECISIONS decisions, and the reference point
//  * for that is itself recomputed from the absolute 64-bit tick count at every
//  * publish. The sine frequency is therefore exact to the crystal no matter
//  * what rate the loop actually achieves; the accumulator only interpolates
//  * between resyncs.
//  *
//  * THE HOT LOOP IS THREE INSTRUCTIONS OF WORK PLUS A LOAD
//  * ------------------------------------------------------
//  * The threshold arithmetic is not done in the loop. A window LUT is built at
//  * startup holding, for each phase step, BOTH comparison limits already in ADC
//  * code units and already clamped, packed into one 32-bit word - the high
//  * limit in bits 15:0, the low limit in bits 31:16. So a decision is: advance
//  * the accumulator, one load, one mask, two compares, at most one store to
//  * BSRR. No multiply, no add of offset or amplitude, no clamp, no branchy
//  * scaling. The LUT lives in .bss, which this project's linker script places
//  * in DTCM - zero-wait-state memory that no DMA touches.
//  *
//  * PARAMETERS
//  * ----------
//  * Sine offset, amplitude, frequency and threshold are all in millivolts /
//  * hertz below and are converted to ADC codes when the LUT is built, so any
//  * of them can change without touching the loop. The status block reports the
//  * values in force, so the host never has to be told them.
//  *
//  * If offset +/- amplitude +/- threshold leaves the 0..full-scale range the
//  * limits saturate rather than wrap, and the status block flags it: a
//  * saturated high limit means the LOW branch can never fire in that part of
//  * the cycle, which is a real behaviour change and not something to discover
//  * from a scope trace.
//  *
//  * BUILD
//  * -----
//  * stm32h7xx_hal_conf.h needs HAL_ADC_MODULE_ENABLED. stm32h7xx_hal_adc.c and
//  * stm32h7xx_hal_adc_ex.c must be in the project. No math library - the sine
//  * is a polynomial, evaluated only at startup.
//  *
//  * Resources: ADC1, DMA2_Stream0 (boot measurement only), TIM2, PLL2P, PB5,
//  *            MPU region 0, the bottom 32 KB of SRAM1, 4 * LUT_N bytes of
//  *            DTCM for the window LUT.
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
///* The sine. Offset and amplitude are about the ADC's own scale: offset is the
//   centre, amplitude is the peak deviation, so the sine spans
//   offset +/- amplitude. */
//#define SINE_HZ                 1000UL
//#define SINE_OFFSET_MV          1500
//#define SINE_AMP_MV             200
//#define THRESHOLD_MV            200
//
///* Decisions per second. 16 MHz = 62.5 ns = 30 CPU cycles at 480 MHz. This is
//   the REQUESTED rate; the achieved rate is measured and published. */
//#define DECISION_RATE_HZ        16000000UL
//
///* ADC. 10 bits, so codes are 0..1023 and one LSB is 3300/1024 = 3.22 mV.
//   50 MHz is the rev V kernel-clock ceiling; the status block reports REV_ID
//   so an incorrect choice is visible. */
//#define ADC_RES_BITS            10U
//#define ADC_CLK_HZ              50000000UL
//#define ADC_SAMPLETIME          ADC_SAMPLETIME_1CYCLE_5
//#define ADC_SAMPLE_HALFCYCLES   3U      /* keep in step: 1.5 cyc = 3 halves */
//#define VREF_MV                 3300
//
///* Window LUT length. The value step between adjacent entries is
//   amplitude * 2*pi / LUT_N, so 4096 entries keeps the step below one ADC LSB
//   for amplitudes up to about 1.3 V - the sine is smooth in code units and
//   there is nothing to gain from interpolating. 4 bytes per entry. */
//#define LUT_N                   4096U
//#define LUT_SHIFT               20U     /* 32 - log2(LUT_N) */
//
///* How often the phase is re-derived from TIM2, in decisions. Small enough
//   that the accumulator never has time to matter, large enough that the 64-bit
//   multiply costs well under a cycle per decision. Must be a multiple of 8 -
//   the inner loop is unrolled by 8. */
//#define RESYNC_DECISIONS        8U
//
///* How many resync blocks between publishes to the status block. SRAM1 is
//   non-cacheable so a write there costs bus cycles; at the default this is
//   once per 65536 decisions, about 4 ms. */
//#define PUBLISH_BLOCKS          8192U
//
///* Count PB5 edges. Costs a register compare and a branch in the hot loop, so
//   it is off by default - turn it on when the switching rate is the question
//   rather than the timing. */
//#define ASP_COUNT_EDGES         0
//
///* Share one ADC1->DR read between two consecutive decisions.
//   The generated inner loop is eight instructions per decision, and the one
//   that can stall is the read of ADC1->DR - it leaves the core's own buses
//   and the compare needs the result immediately. If the measured decision
//   rate comes in below DECISION_RATE_HZ, this is the first thing to try: the
//   ADC only produces a new sample every ~280 ns, so two decisions 62.5 ns
//   apart are looking at the same conversion anyway and nothing is lost by
//   reading it once. Off by default, so the first measurement is of the plain
//   loop. */
//#define ASP_SHARE_DR            0
//
///* Samples taken during the boot measurement pass, which doubles as a capture
//   window readable by adc_capture.py --addr 0x30000100. */
//#define BOOT_SAMPLES            4096U
//
///* 240 MHz timer kernel clock: 480 MHz SYSCLK, HCLK /2, APB /2, timer x2. */
//#define TIMER_CLK_HZ            240000000UL
//
///* PB5, the output. */
//#define PWM_PORT                GPIOB
//#define PWM_PIN                 GPIO_PIN_5
//
///* ==========================================================================
// * Derived, checked at compile time
// * ========================================================================== */
//
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
//#define ADC_FULL_SCALE          ((1U << ADC_RES_BITS) - 1U)
//
//#define PLL2_VCO_HZ             400000000UL
//#define PLL2P_DIV               (PLL2_VCO_HZ / ADC_CLK_HZ)
//
//_Static_assert(PLL2_VCO_HZ % ADC_CLK_HZ == 0U,
//               "ADC_CLK_HZ must divide the 400 MHz PLL2 VCO exactly");
//_Static_assert(ADC_CLK_HZ <= 50000000UL,
//               "adc_ker_ck above the rev V 50 MHz maximum");
//_Static_assert((1UL << (32U - LUT_SHIFT)) == LUT_N,
//               "LUT_SHIFT must be 32 - log2(LUT_N)");
//_Static_assert((RESYNC_DECISIONS % 8U) == 0U,
//               "RESYNC_DECISIONS must be a multiple of 8 - the inner loop is "
//               "unrolled by 8");
//
///* Phase advance per decision, and per TIM2 tick in Q8. The Q8 fraction is
//   what keeps the resync exact: at Q0 the per-tick advance would be rounded to
//   a whole phase step and the frequency error would be parts in 10^4. */
//#define PHASE_INC               ((uint32_t)(((uint64_t)SINE_HZ << 32) \
//                                            / DECISION_RATE_HZ))
//#define PHASE_PER_TICK_Q8       ((uint32_t)(((uint64_t)SINE_HZ << 40) \
//                                            / TIMER_CLK_HZ))
//
//_Static_assert(PHASE_PER_TICK_Q8 > 0U,
//               "SINE_HZ too low for the Q8 tick scaling - raise the Q or the "
//               "frequency");
//_Static_assert(((uint64_t)SINE_HZ << 40) / TIMER_CLK_HZ <= 0xFFFFFFFFULL,
//               "SINE_HZ too high for the Q8 tick scaling (limit ~937 kHz)");
//_Static_assert(SINE_HZ * 16UL <= DECISION_RATE_HZ,
//               "fewer than 16 decisions per sine cycle - the sine would be "
//               "resolved too coarsely to compare against");
//
///* ==========================================================================
// * Shared memory: the status block, and the boot capture
// * ==========================================================================
// * SRAM1 (D2 domain) at the base, because the boot measurement pass DMAs into
// * it and no DMA controller can reach DTCM. The MPU marks the whole 32 KB
// * non-cacheable so the host reads what the DMA wrote and the CPU's own
// * publishes are not sitting in a dirty cache line.
// *
// *   0x30000000  controller status, 128 bytes, magic 0xC0DECAFE
// *   0x30000100  capture descriptor, 64 bytes, magic 0xADC0CAFE (v3)
// *   0x30000140  BOOT_SAMPLES halfwords from the measurement pass
// */
//#define SHARED_BASE             0x30000000UL
//#define CTRL_ADDR               (SHARED_BASE)
//#define CAP_HDR_ADDR            (SHARED_BASE + 0x100UL)
//#define CAP_DATA_ADDR           (SHARED_BASE + 0x140UL)
//
//#define CTRL_MAGIC              0xC0DECAFEUL
//#define CTRL_VERSION            1UL
//
//#define CTRL_STATUS_INIT        0UL
//#define CTRL_STATUS_MEASURING   1UL
//#define CTRL_STATUS_RUNNING     2UL
//
//typedef struct
//{
//  uint32_t magic;               /* CTRL_MAGIC                               */
//  uint32_t version;             /* 1                                        */
//  uint32_t status;              /* CTRL_STATUS_*                            */
//  uint32_t rev_id;              /* 0x2003 rev V, 0x1003 rev Y               */
//  uint32_t timer_clk_hz;
//  uint32_t adc_clk_hz;
//  uint32_t adc_bits;
//  uint32_t vref_mv;
//  uint32_t adc_conv_halfcycles; /* nominal, for comparison with the measured */
//  uint32_t adc_rate_meas_hz;    /* MEASURED at boot                         */
//  uint32_t dec_rate_nom_hz;     /* DECISION_RATE_HZ                         */
//  uint32_t dec_rate_meas_hz;    /* MEASURED, updated every publish          */
//  uint32_t decisions_lo;        /* 64-bit total decisions                   */
//  uint32_t decisions_hi;
//  uint32_t ticks_lo;            /* 64-bit total TIM2 ticks                  */
//  uint32_t ticks_hi;
//  uint32_t sine_hz;
//  uint32_t sine_offset_mv;
//  uint32_t sine_amp_mv;
//  uint32_t threshold_mv;
//  uint32_t lut_n;
//  uint32_t edges_lo;            /* 0 unless ASP_COUNT_EDGES                 */
//  uint32_t edges_hi;
//  uint32_t clamp_hi;            /* entries whose high limit saturated       */
//  uint32_t clamp_lo;            /* entries whose low limit saturated        */
//  uint32_t publishes;
//  uint32_t reserved[6];
//} ctrl_blk_t;
//
//_Static_assert(sizeof(ctrl_blk_t) == 128U, "status block must stay 128 bytes");
//
///* The capture descriptor is the v3 layout adc_capture.py already reads, so
//   the boot measurement pass is directly plottable with
//   adc_capture.py --addr 0x30000100 --no-reset. */
//#define CAP_MAGIC               0xADC0CAFEUL
//
//typedef struct
//{
//  uint32_t magic, version, sample_rate_hz, sample_count;
//  uint32_t pulse_us, adc_bits, vref_mv, status;
//  uint32_t timer_clk_hz, elapsed_ticks, adc_clk_hz, mode;
//  uint32_t sample_bytes, rev_id, adc_isr, conv_halfcycles;
//} cap_hdr_t;
//
//_Static_assert(sizeof(cap_hdr_t) == 64U, "descriptor must stay 64 bytes");
//
//_Static_assert(0x140UL + (BOOT_SAMPLES * 2U) <= 32768UL,
//               "boot capture exceeds the 32 KB non-cacheable MPU region");
//
//static volatile ctrl_blk_t * const ctrl = (volatile ctrl_blk_t *)CTRL_ADDR;
//static volatile cap_hdr_t  * const cap  = (volatile cap_hdr_t *)CAP_HDR_ADDR;
//
///* The window LUT. .bss, so this project's linker script puts it in DTCM: the
//   hot loop's only load, and it costs no bus cycles. */
//static uint32_t window_lut[LUT_N];
//
//static ADC_HandleTypeDef hadc1;
//static DMA_HandleTypeDef hdma_adc;
//static TIM_HandleTypeDef htim2;
//
//void Error_Handler(void);
//
///* ==========================================================================
// * MPU, clocks
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
//  mpu.BaseAddress      = SHARED_BASE;
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
///** PLL2P at ADC_CLK_HZ, routed to the ADC. Must run before HAL_ADC_Init(),
//    which reads this clock back to set ADC_CR BOOST - mandatory above 25 MHz. */
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
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//
//  /* PC0 = ADC1_INP10. */
//  g.Pin  = GPIO_PIN_0;
//  g.Mode = GPIO_MODE_ANALOG;
//  g.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &g);
//
//  /* PB5 = the decision output. Driven low before being made an output so it
//     cannot glitch during configuration. MEDIUM speed: enough slew for edges
//     this fast and far less ringing into the analog front end than VERY_HIGH,
//     which matters because the input this loop is measuring is on the same
//     board. */
//  HAL_GPIO_WritePin(PWM_PORT, PWM_PIN, GPIO_PIN_RESET);
//  g.Pin       = PWM_PIN;
//  g.Mode      = GPIO_MODE_OUTPUT_PP;
//  g.Pull      = GPIO_NOPULL;
//  g.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//  g.Alternate = 0;
//  HAL_GPIO_Init(PWM_PORT, &g);
//}
//
///** TIM2: free-running 32-bit counter at 240 MHz. The phase reference and the
//    measurement clock. 4.166 ns per tick, wraps every 17.9 s, and every
//    subtraction of it is unsigned so a wrap costs nothing. */
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
///** DMA2_Stream0, used only for the boot measurement pass. */
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
//  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
//}
//
///**
//  * @brief ADC1 on PC0, continuous conversion, software start.
//  *
//  * Continuous means one ADSTART and the converter never stops: DR always holds
//  * the most recent result and the hot loop just reads it. There is no trigger
//  * to be dropped and no timer in the sample path at all, which is the whole
//  * point of running it this way - the ADC goes as fast as it can and the
//  * control loop samples whatever is newest.
//  *
//  * OVR is left in overwrite mode and then ignored. In continuous mode with no
//  * DMA the overrun flag sets almost immediately, because the CPU is not
//  * obliged to read every conversion - and it should not be. Overwrite means DR
//  * keeps advancing regardless, which is exactly the wanted behaviour.
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
//  hadc1.Init.ContinuousConvMode       = ENABLE;
//  hadc1.Init.NbrOfConversion          = 1;
//  hadc1.Init.DiscontinuousConvMode    = DISABLE;
//  hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
//  hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
//  hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
//  hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
//  hadc1.Init.OversamplingMode         = DISABLE;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Independent mode, set explicitly: the DUAL field lives in the ADC12
//     COMMON register, which HAL_ADC_Init() does not touch, so a previous image
//     that left dual interleaved mode set would still have it set here and the
//     data would come from CDR instead of DR. */
//  mm.Mode             = ADC_MODE_INDEPENDENT;
//  mm.DualModeData     = ADC_DUALMODEDATAFORMAT_DISABLED;
//  mm.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mm) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
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
//void DMA2_Stream0_IRQHandler(void)
//{
//  HAL_DMA_IRQHandler(&hdma_adc);
//}
//
///* ==========================================================================
// * The window LUT
// * ========================================================================== */
//
///**
//  * @brief sin(2*pi*x) for x in [0,1), as a float, without libm.
//  *
//  * Quadrant folding onto [0, pi/2] and then a 7th-order odd polynomial, which
//  * is good to about 1e-6 - four orders of magnitude finer than a 10-bit code.
//  * Called LUT_N times at startup and never again, so its speed is irrelevant.
//  */
//static float sine_turn(float x)
//{
//  float sign = 1.0f;
//
//  if (x >= 0.5f)                       /* second half of the cycle */
//  {
//    x -= 0.5f;
//    sign = -1.0f;
//  }
//  if (x > 0.25f)                       /* fold about the quarter point */
//  {
//    x = 0.5f - x;
//  }
//
//  const float t  = x * 6.28318530718f; /* now in [0, pi/2] */
//  const float t2 = t * t;
//
//  /* t - t^3/6 + t^5/120 - t^7/5040 */
//  const float s = t * (1.0f + t2 * (-1.0f / 6.0f
//                            + t2 * (1.0f / 120.0f
//                            + t2 * (-1.0f / 5040.0f))));
//  return sign * s;
//}
//
///**
//  * @brief Build the packed comparison limits, in ADC codes.
//  *
//  * Entry i, for phase i/LUT_N of a cycle, holds
//  *     bits 15:0   sine + threshold, the limit above which PB5 goes LOW
//  *     bits 31:16  sine - threshold, the limit below which PB5 goes HIGH
//  *
//  * both already clamped into 0..full scale, so the hot loop needs no
//  * arithmetic beyond a mask and a shift. Saturation is counted rather than
//  * hidden: a clamped high limit means the LOW branch cannot fire at that
//  * phase, and the host is told how many entries are affected.
//  */
//static void Lut_Build(void)
//{
//  const float scale = (float)(1U << ADC_RES_BITS) / (float)VREF_MV;
//  const float off_c = (float)SINE_OFFSET_MV * scale;
//  const float amp_c = (float)SINE_AMP_MV * scale;
//  const float thr_c = (float)THRESHOLD_MV * scale;
//
//  uint32_t nhi = 0U, nlo = 0U;
//
//  for (uint32_t i = 0U; i < LUT_N; i++)
//  {
//    const float s  = off_c + amp_c * sine_turn((float)i / (float)LUT_N);
//    float       hi = s + thr_c;
//    float       lo = s - thr_c;
//
//    if (hi > (float)ADC_FULL_SCALE) { hi = (float)ADC_FULL_SCALE; nhi++; }
//    if (hi < 0.0f)                  { hi = 0.0f;                  nhi++; }
//    if (lo > (float)ADC_FULL_SCALE) { lo = (float)ADC_FULL_SCALE; nlo++; }
//    if (lo < 0.0f)                  { lo = 0.0f;                  nlo++; }
//
//    window_lut[i] = ((uint32_t)(lo + 0.5f) << 16) | (uint32_t)(hi + 0.5f);
//  }
//
//  ctrl->clamp_hi = nhi;
//  ctrl->clamp_lo = nlo;
//}
//
///* ==========================================================================
// * The boot measurement pass
// * ========================================================================== */
//
///**
//  * @brief Time BOOT_SAMPLES continuous conversions to get the real ADC rate.
//  *
//  * The ADC is already in continuous mode, so this is one DMA transfer of
//  * BOOT_SAMPLES bracketed by TIM2 reads. It leaves the samples where
//  * adc_capture.py can plot them and the elapsed ticks in both the capture
//  * descriptor and the status block.
//  *
//  * Takes about 1.1 ms at the expected rate. Nothing else has started yet, so
//  * it cannot disturb the control loop.
//  */
//static uint32_t Adc_MeasureRate(void)
//{
//  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)CAP_DATA_ADDR,
//                        BOOT_SAMPLES) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  const uint32_t t0 = TIM2->CNT;
//
//  const uint32_t guard = HAL_GetTick();
//  while ((DMA2_Stream0->NDTR != 0U) && ((HAL_GetTick() - guard) < 500U))
//  {
//    /* spin */
//  }
//  const uint32_t t1 = TIM2->CNT;
//
//  const bool     ok      = (DMA2_Stream0->NDTR == 0U);
//  const uint32_t elapsed = t1 - t0;
//  const uint32_t isr     = ADC1->ISR;
//
//  (void)HAL_ADC_Stop_DMA(&hadc1);
//
//  const uint32_t rate = (ok && elapsed)
//      ? (uint32_t)(((uint64_t)TIMER_CLK_HZ * BOOT_SAMPLES) / elapsed)
//      : 0U;
//
//  cap->magic           = CAP_MAGIC;
//  cap->version         = 3U;
//  cap->sample_rate_hz  = 0U;              /* free running: no requested rate */
//  cap->sample_count    = BOOT_SAMPLES;
//  cap->pulse_us        = elapsed / (TIMER_CLK_HZ / 1000000UL);
//  cap->adc_bits        = ADC_RES_BITS;
//  cap->vref_mv         = VREF_MV;
//  cap->status          = ok ? 2U : 3U;
//  cap->timer_clk_hz    = TIMER_CLK_HZ;
//  cap->elapsed_ticks   = elapsed;
//  cap->adc_clk_hz      = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
//  cap->mode            = 1U;              /* free running */
//  cap->sample_bytes    = 2U;
//  cap->rev_id          = HAL_GetREVID();
//  cap->adc_isr         = isr;
//  cap->conv_halfcycles = ADC_CONV_HALFCYCLES;
//
//  return rate;
//}
//
///* ==========================================================================
// * The control loop
// * ==========================================================================
// * One decision. Deliberately free of arithmetic: the accumulator advance, one
// * DTCM load, a mask, two compares, at most one BSRR store.
// *
// * ADC1->DR is read first so the load is in flight while the accumulator and
// * the LUT index are computed - the Cortex-M7 issues in order but only stalls
// * at the USE of a loaded value, and DR is the one access here that goes off
// * the core's own buses.
// *
// * Both branches write BSRR, which is write-only and atomic per bit, so there
// * is no read-modify-write of ODR and no need to track the current state.
// * Writing the same level twice is harmless, so nothing is conditional on what
// * the pin is already doing.
// */
//#if ASP_COUNT_EDGES
//  #define ASP_EDGE_LOW()   do { if (state != 0U) { edges++; state = 0U; } } while (0)
//  #define ASP_EDGE_HIGH()  do { if (state == 0U) { edges++; state = 1U; } } while (0)
//#else
//  #define ASP_EDGE_LOW()   do { } while (0)
//  #define ASP_EDGE_HIGH()  do { } while (0)
//#endif
//
//#define ASP_DECIDE()                                                        \
//  do {                                                                      \
//    const uint32_t a = ADC1->DR;                                            \
//    phase += PHASE_INC;                                                     \
//    const uint32_t w = window_lut[phase >> LUT_SHIFT];                      \
//    if (a > (w & 0xFFFFU))                                                  \
//    {                                                                       \
//      PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;   /* above sine+thr: LOW  */ \
//      ASP_EDGE_LOW();                                                       \
//    }                                                                       \
//    else if (a < (w >> 16))                                                 \
//    {                                                                       \
//      PWM_PORT->BSRR = (uint32_t)PWM_PIN;         /* below sine-thr: HIGH */ \
//      ASP_EDGE_HIGH();                                                      \
//    }                                                                       \
//    /* inside the window: hold */                                           \
//  } while (0)
//
///* Two decisions on one DR read. The phase still advances per decision, so the
//   sine is evaluated at the full rate; only the input is shared. */
//#define ASP_DECIDE_ON(a)                                                    \
//  do {                                                                      \
//    phase += PHASE_INC;                                                     \
//    const uint32_t w = window_lut[phase >> LUT_SHIFT];                      \
//    if ((a) > (w & 0xFFFFU))                                                \
//    {                                                                       \
//      PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;                             \
//      ASP_EDGE_LOW();                                                       \
//    }                                                                       \
//    else if ((a) < (w >> 16))                                               \
//    {                                                                       \
//      PWM_PORT->BSRR = (uint32_t)PWM_PIN;                                   \
//      ASP_EDGE_HIGH();                                                      \
//    }                                                                       \
//  } while (0)
//
//#define ASP_DECIDE2()                                                       \
//  do {                                                                      \
//    const uint32_t a2 = ADC1->DR;                                           \
//    ASP_DECIDE_ON(a2);                                                      \
//    ASP_DECIDE_ON(a2);                                                      \
//  } while (0)
//
//static void Control_Loop(uint32_t adc_rate_hz)
//{
//  uint32_t phase     = 0U;
//  uint64_t decisions = 0U;
//  uint64_t ticks     = 0U;
//#if ASP_COUNT_EDGES
//  uint64_t edges     = 0U;
//  uint32_t state     = 0U;
//#endif
//  uint32_t publishes = 0U;
//
//  /* The phase reference. tick_ref is a TIM2 snapshot; phase_ref is the exact
//     phase at that snapshot, recomputed from the absolute tick count at every
//     publish so it can never accumulate error. */
//  uint32_t tick_ref  = TIM2->CNT;
//  uint32_t tick_last = tick_ref;
//  uint32_t phase_ref = 0U;
//
//  ctrl->status           = CTRL_STATUS_RUNNING;
//  ctrl->adc_rate_meas_hz = adc_rate_hz;
//  ctrl->dec_rate_nom_hz  = DECISION_RATE_HZ;
//
//  for (;;)
//  {
//    for (uint32_t blk = 0U; blk < PUBLISH_BLOCKS; blk++)
//    {
//      for (uint32_t k = 0U; k < (RESYNC_DECISIONS / 8U); k++)
//      {
//#if ASP_SHARE_DR
//        ASP_DECIDE2(); ASP_DECIDE2(); ASP_DECIDE2(); ASP_DECIDE2();
//#else
//        ASP_DECIDE(); ASP_DECIDE(); ASP_DECIDE(); ASP_DECIDE();
//        ASP_DECIDE(); ASP_DECIDE(); ASP_DECIDE(); ASP_DECIDE();
//#endif
//      }
//
//      /* Snap the accumulator back onto the crystal. dt is small - one block
//         of decisions - so this is a single 32x32 multiply. */
//      const uint32_t dt = TIM2->CNT - tick_ref;
//      phase = phase_ref
//            + (uint32_t)(((uint64_t)dt * PHASE_PER_TICK_Q8) >> 8);
//    }
//
//    /* ---- publish, roughly every RESYNC_DECISIONS * PUBLISH_BLOCKS ------- */
//    const uint32_t now = TIM2->CNT;
//    ticks     += (uint64_t)(uint32_t)(now - tick_last);
//    tick_last  = now;
//    decisions += (uint64_t)RESYNC_DECISIONS * PUBLISH_BLOCKS;
//    publishes++;
//
//    /* Recompute the reference from the absolute tick count: the modulo-2^32
//       truncation is the phase wrap, which is what is wanted. */
//    tick_ref  = now;
//    phase_ref = (uint32_t)((ticks * (uint64_t)PHASE_PER_TICK_Q8) >> 8);
//    phase     = phase_ref;
//
//    if (ticks)
//    {
//      ctrl->dec_rate_meas_hz =
//          (uint32_t)((decisions * (uint64_t)TIMER_CLK_HZ) / ticks);
//    }
//    ctrl->decisions_lo = (uint32_t)decisions;
//    ctrl->decisions_hi = (uint32_t)(decisions >> 32);
//    ctrl->ticks_lo     = (uint32_t)ticks;
//    ctrl->ticks_hi     = (uint32_t)(ticks >> 32);
//    ctrl->publishes    = publishes;
//#if ASP_COUNT_EDGES
//    ctrl->edges_lo     = (uint32_t)edges;
//    ctrl->edges_hi     = (uint32_t)(edges >> 32);
//#endif
//  }
//}
//
///* ==========================================================================
// * main
// * ========================================================================== */
//
//int main(void)
//{
//  Mpu_Config();                 /* before the D-cache: the shared block must
//                                   never be cached */
//  SCB_EnableICache();
//  SCB_EnableDCache();
//
//  HAL_Init();
//  SystemClock_Config();
//
//  /* Keep the debug unit reachable so the status block can be read out of a
//     running target. */
//  DBGMCU->CR |= DBGMCU_CR_DBG_SLEEPD1 | DBGMCU_CR_DBG_STOPD1
//              | DBGMCU_CR_DBG_STANDBYD1;
//
//  AdcClock_Config();
//  Gpio_Config();
//  Timebase_Config();
//  Dma_Config();
//  Adc_Config();
//
//  ctrl->magic               = CTRL_MAGIC;
//  ctrl->version             = CTRL_VERSION;
//  ctrl->status              = CTRL_STATUS_MEASURING;
//  ctrl->rev_id              = HAL_GetREVID();
//  ctrl->timer_clk_hz        = TIMER_CLK_HZ;
//  ctrl->adc_clk_hz          = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
//  ctrl->adc_bits            = ADC_RES_BITS;
//  ctrl->vref_mv             = VREF_MV;
//  ctrl->adc_conv_halfcycles = ADC_CONV_HALFCYCLES;
//  ctrl->adc_rate_meas_hz    = 0U;
//  ctrl->dec_rate_nom_hz     = DECISION_RATE_HZ;
//  ctrl->dec_rate_meas_hz    = 0U;
//  ctrl->decisions_lo        = 0U;
//  ctrl->decisions_hi        = 0U;
//  ctrl->ticks_lo            = 0U;
//  ctrl->ticks_hi            = 0U;
//  ctrl->sine_hz             = SINE_HZ;
//  ctrl->sine_offset_mv      = SINE_OFFSET_MV;
//  ctrl->sine_amp_mv         = SINE_AMP_MV;
//  ctrl->threshold_mv        = THRESHOLD_MV;
//  ctrl->lut_n               = LUT_N;
//  ctrl->edges_lo            = 0U;
//  ctrl->edges_hi            = 0U;
//  ctrl->publishes           = 0U;
//
//  Lut_Build();
//
//  /* Measure what the ADC actually does before relying on it. */
//  const uint32_t adc_rate = Adc_MeasureRate();
//
//  /* Switch the ADC off DMA and onto plain DR: the control loop reads DR
//     directly, so there is no reason to move every conversion into memory.
//     DMNGT = 00 is "regular conversion data stored in DR only", and it can
//     only be changed with ADSTART clear - which it is, after Stop_DMA. */
//  MODIFY_REG(ADC1->CFGR, ADC_CFGR_DMNGT, 0U);
//
//  if (HAL_ADC_Start(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  Control_Loop(adc_rate);       /* never returns */
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
