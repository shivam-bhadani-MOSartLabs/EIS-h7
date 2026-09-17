///**
//  ******************************************************************************
//  * @file    adc_sine_pwm.c
//  * @brief   Free-running hysteretic controller on the NUCLEO-H753ZI.
//  *          See adc_sine_pwm.h for the control law and the pin map.
//  *
//  * How this differs from the PWM version
//  * -------------------------------------
//  * TIM3 and TIM4 are gone. There is no carrier and no compare register, so
//  * PB5 and PD14 are plain GPIOs written directly from the sample loop, and the
//  * loop runs in main() rather than in an interrupt - there is nothing else for
//  * the CPU to do, and skipping the interrupt entry saves latency, which in a
//  * hysteretic loop is the thing that sets the switching frequency.
//  *
//  * TIM2 (32-bit) is new: it is the time anchor for the DDS. In the PWM version
//  * the sine phase advanced once per ADC sample processed, which meant that if
//  * the ADC delivered fewer samples than assumed, the reference sine slowed
//  * down in proportion while the gate stayed the right length. Anchoring the
//  * phase to a hardware counter removes that failure mode entirely: the sine is
//  * now the frequency you asked for whatever the ADC does.
//  *
//  * Loop timing
//  * -----------
//  * There is no filter. Each pass reduces the conversions that have arrived to
//  * their maximum - two compares per sample, nothing else - and then makes one
//  * comparison and at most one GPIO store for the whole chunk. The sine
//  * reference is evaluated once per chunk rather than once per sample.
//  *
//  * End-to-end delay from the input crossing the band to the pin moving is
//  * about 250 ns (one conversion) plus one pass of the loop, so roughly 400 ns
//  * with nothing else in the way. That is the floor on the switching period, so
//  * the output could in principle switch above 1 MHz. Nothing in this firmware
//  * bounds it - check the gate driver tolerates whatever the loop settles at,
//  * and raise the threshold if it is too fast.
//  *
//  * BUILD AT -O2. At -O0 this loop runs three to five times slower, which on
//  * its own is enough to stop it keeping up with a 4 MSPS stream.
//  *
//  * BEFORE THIS BUILDS
//  * ------------------
//  * 1. Core/Inc/stm32h7xx_hal_conf.h: uncomment  #define HAL_ADC_MODULE_ENABLED
//  * 2. Add stm32h7xx_hal_adc.c and stm32h7xx_hal_adc_ex.c to the project.
//  * 3. stm32h7xx_it.c: delete DMA1_Stream0..3_IRQHandler, COMP1_IRQHandler and
//  *    their extern handle declarations.
//  * 4. stm32h7xx_hal_msp.c: delete the MspInit/MspDeInit functions for DAC1,
//  *    COMP1, COMP2, HRTIM1, TIM1, TIM2, TIM6, TIM7 and HAL_TIM_MspPostInit.
//  *    Keep HAL_MspInit() and HAL_ADC_MspInit().
//  *
//  * No math library is used.
//  *
//  * Resources claimed
//  * -----------------
//  *   ADC1, ADC2        dual interleaved, channel INP10 on PC0
//  *   DMA2_Stream0      ADC1 request, circular, 32-bit
//  *   TIM8              ADC sample trigger, TRGO on update, no pin
//  *   TIM2              32-bit free-running time base for the DDS and the gate
//  *   PB5               the hysteretic output, plain GPIO
//  *   PD14              PB5 replayed a constant 1 us later, plain GPIO
//  *   PF3               one-shot gate
//  *   PLL2P             32 MHz ADC kernel clock
//  *   MPU region 0      0x30000000, 32 KB, non-cacheable (the DMA buffer)
//  *   SRAM1 4 KB        at 0x30000000, the DMA buffer itself
//  ******************************************************************************
//  */
//
//#include "adc_sine_pwm.h"
//
///* ==========================================================================
// * Interleave timing - these three move together
// * ==========================================================================
// * At 12-bit resolution the stagger between the two converters is limited to
// * 8 ADC clock cycles (the HAL header: 1..9 cycles for 16 and 14 bits, 1..8 for
// * 12 bits). For a uniformly spaced interleaved stream the stagger has to be
// * exactly half the pair period, so a pair is 16 cycles and the conversion has
// * to fit inside 8. A 12-bit conversion is sampling + 6.5 cycles, hence 1.5
// * cycles of sampling and an 8-cycle conversion - exactly the stagger.
// *
// * At 32 MHz that is a 500 ns pair period: 2.0 MHz pairs, 4.0 MSPS, 46.9 ns
// * aperture. See adc_sine_pwm.h for the slower variants.
// */
//#define ASP_ADC_SAMPLETIME          ADC_SAMPLETIME_1CYCLE_5
//#define ASP_TWO_SAMPLING_DELAY      ADC_TWOSAMPLINGDELAY_8CYCLES
//
///* ==========================================================================
// * DMA buffer placement
// * ==========================================================================
// * The CubeIDE default linker script puts .bss in DTCMRAM at 0x20000000, which
// * the DMA controllers cannot reach at all, so the buffer is placed by hand in
// * SRAM1 (D2 domain). Nothing else may use the first 4 KB of SRAM1.
// */
//#define ASP_BUF_BASE_ADDR           0x30000000UL
//#define ASP_BUF_MASK                (ASP_BUF_WORDS - 1U)
//
//static volatile uint32_t * const asp_buf = (volatile uint32_t *)ASP_BUF_BASE_ADDR;
//
///* Output pin shorthands, used as raw BSRR writes in the sample loop. */
//#define ASP_OUT_A_PORT              GPIOB
//#define ASP_OUT_A_PIN               GPIO_PIN_5
//#define ASP_OUT_B_PORT              GPIOD
//#define ASP_OUT_B_PIN               GPIO_PIN_14
//#define ASP_GATE_PORT               GPIOF
//#define ASP_GATE_PIN                GPIO_PIN_3
//
///* Instrumentation pins. */
//#define ASP_CLK_PORT                GPIOD
//#define ASP_CLK_PIN                 GPIO_PIN_15
//
///* ==========================================================================
// * Module state
// * ========================================================================== */
//
//ASP_Ctx_t ASP_Ctx;
//
//static int16_t asp_sin_lut[ASP_LUT_SIZE];
//
//
///* ==========================================================================
// * PD14 delay line
// * ==========================================================================
// * PB5 is aperiodic, so the second output cannot be made by shifting a phase.
// * Instead every transition of PB5 is stamped with the TIM2 tick at which it
// * should appear on PD14 - the tick the sample itself belongs to, plus the
// * delay - and pushed into this queue. The queue is drained against the live
// * counter, so the delay is measured in real time and does not care that the
// * sample loop works in bursts of a few samples at a time.
// *
// * 64 slots is far more than needed: the output can change at most once per
// * pass of the loop, so a 1 us delay holds only a handful of edges even at the
// * fastest the loop can switch.
// */
//#define ASP_DQ_SIZE                 64U
//#define ASP_DQ_MASK                 (ASP_DQ_SIZE - 1U)
//
//typedef struct
//{
//  uint32_t due;      /* TIM2 tick at which this edge is to appear */
//  uint8_t  level;    /* the PB5 level it came from                */
//} asp_edge_t;
//
//static asp_edge_t asp_dq[ASP_DQ_SIZE];
//static uint32_t   asp_dq_head;      /* next slot to write            */
//static uint32_t   asp_dq_tail;      /* next slot to play             */
//static uint32_t   asp_dq_last_due;  /* due time of the last push     */
//
///** Drive PD14 from a PB5 level, applying the complement option. */
//static inline void ASP_WriteOutB(bool level)
//{
//#if ASP_PD14_COMPLEMENT
//  level = !level;
//#endif
//  ASP_OUT_B_PORT->BSRR = level ? (uint32_t)ASP_OUT_B_PIN
//                               : ((uint32_t)ASP_OUT_B_PIN << 16);
//}
//
//#if ASP_PD14_HARDWARE
///* ==========================================================================
// * Hardware edge placement on TIM4_CH3
// * ==========================================================================
// * TIM4 free-runs 16-bit at 240 MHz, wrapping every 273 us. Channel 3 sits in
// * TOGGLE mode with its output permanently enabled, and CCR3 always holds
// * either the next real due time or a parking value.
// *
// * Why parking rather than switching the channel off between edges: clearing
// * CC3E disconnects the timer from the pin, which makes the pin glitch. So the
// * channel stays on, and when there is nothing pending CCR3 is held a few ticks
// * BEHIND the counter - a match is then a full wrap away, and the value is
// * refreshed on every pass of the loop so the counter never catches up.
// *
// * The queue holds only due times. Toggle mode does not need levels: each match
// * flips the pin, and the pin's parity is set once at the start of the burst
// * with a forced-level write before the mode is switched to toggle.
// */
//
///* Idle level of PD14 between bursts, expressed as an output-compare forced
//   mode. Non-complement idles low; complement idles high. */
//#if ASP_PD14_COMPLEMENT
//#define ASP_PD14_IDLE_MODE          TIM_OCMODE_FORCED_ACTIVE
//#else
//#define ASP_PD14_IDLE_MODE          TIM_OCMODE_FORCED_INACTIVE
//#endif
//
///** Set TIM4 channel 3's output-compare mode. OC3M sits at the same bit
//    positions inside CCMR2 as OC1M does inside CCMR1, so the HAL's
//    TIM_OCMODE_* values can be used directly. */
//static inline void ASP_Pd14SetMode(uint32_t mode)
//{
//  MODIFY_REG(TIM4->CCMR2, TIM_CCMR2_OC3M, mode);
//}
//
///** Hold CCR3 just behind the counter, so the next match is a wrap away. */
//static inline void ASP_Pd14Park(void)
//{
//  TIM4->CCR3 = (uint16_t)(TIM4->CNT - 8U);
//}
//
///**
//  * @brief Retire a compare that has fired and arm the next one.
//  *
//  * CC3IF is the evidence that the hardware has toggled the pin. One flag means
//  * exactly one toggle, so popping exactly one queue entry per flag keeps the
//  * pin's parity locked to the queue.
//  */
//static inline void ASP_ServiceDelayLine(uint32_t now)
//{
//  (void)now;
//
//  if ((TIM4->SR & TIM_SR_CC3IF) != 0U)
//  {
//    TIM4->SR    = ~TIM_SR_CC3IF;
//    asp_dq_tail = (asp_dq_tail + 1U) & ASP_DQ_MASK;
//  }
//
//  if (asp_dq_tail != asp_dq_head)
//  {
//    /* Always the OLDEST pending entry - a newer push must never overtake one
//       already waiting in CCR3. */
//    TIM4->CCR3 = (uint16_t)asp_dq[asp_dq_tail].due;
//  }
//  else
//  {
//    ASP_Pd14Park();
//  }
//}
//#else
///**
//  * @brief Play out every queued edge whose time has come.
//  * @note  The subtraction is done in signed 32-bit so that it stays correct
//  *        across a counter wrap.
//  */
//static inline void ASP_ServiceDelayLine(uint32_t now)
//{
//  while (asp_dq_tail != asp_dq_head)
//  {
//    if ((int32_t)(now - asp_dq[asp_dq_tail].due) < 0)
//    {
//      break;                       /* the head of the queue is still in the future */
//    }
//    ASP_WriteOutB(asp_dq[asp_dq_tail].level != 0U);
//    asp_dq_tail = (asp_dq_tail + 1U) & ASP_DQ_MASK;
//  }
//}
//#endif
//
///**
//  * @brief Queue one PB5 transition for replay on PD14 at `due`.
//  *
//  * The queue is a FIFO played strictly in order, so PB5 switching several
//  * times inside one delay window is fine - every one of those edges is held
//  * and replayed at its own offset, and PD14 ends up a faithful copy of PB5
//  * shifted in time. Nothing is merged and nothing is lost.
//  *
//  * The one way that could break is a shrinking delay: an edge stamped with a
//  * shorter delay could fall due before one already waiting, which would play
//  * the two out of order and leave PD14 at the wrong level. So a new due time
//  * is never allowed to precede the previous one.
//  */
//static inline void ASP_QueueEdge(uint32_t due, bool level)
//{
//#if ASP_PD14_HARDWARE
//  /* Due times live in TIM4's 16-bit counter, so ordering is compared in
//     signed 16-bit. */
//  if ((int16_t)((uint16_t)due - (uint16_t)asp_dq_last_due) < 0)
//  {
//    due = asp_dq_last_due;
//    ASP_Ctx.pd14_reorder_count++;
//  }
//#else
//  if ((int32_t)(due - asp_dq_last_due) < 0)
//  {
//    due = asp_dq_last_due;
//    ASP_Ctx.pd14_reorder_count++;
//  }
//#endif
//  asp_dq_last_due = due;
//
//  uint32_t next = (asp_dq_head + 1U) & ASP_DQ_MASK;
//
//  if (next == asp_dq_tail)
//  {
//    /* Queue full. Retire the oldest rather than lose an edge and leave PD14
//       stuck at the wrong level for good. With 64 slots and at most one edge
//       per pass of the loop this should never happen. */
//#if ASP_PD14_HARDWARE
//    ASP_Ctx.pd14_parity_risk++;
//#else
//    ASP_WriteOutB(asp_dq[asp_dq_tail].level != 0U);
//#endif
//    asp_dq_tail = (asp_dq_tail + 1U) & ASP_DQ_MASK;
//    ASP_Ctx.pd14_drop_count++;
//  }
//
//  asp_dq[asp_dq_head].due   = due;
//  asp_dq[asp_dq_head].level = level ? 1U : 0U;
//  asp_dq_head = next;
//
//  uint32_t depth = (asp_dq_head - asp_dq_tail) & ASP_DQ_MASK;
//  if (depth > ASP_Ctx.pd14_depth_max) { ASP_Ctx.pd14_depth_max = depth; }
//}
//
//static ADC_HandleTypeDef hadc_master;   /* ADC1 */
//static ADC_HandleTypeDef hadc_slave;    /* ADC2 */
//static DMA_HandleTypeDef hdma_adc;
//static TIM_HandleTypeDef htim_trig;     /* TIM8, ADC sample clock */
//static TIM_HandleTypeDef htim_time;     /* TIM2, 32-bit time base */
//
///* ==========================================================================
// * Small helpers - no libm
// * ========================================================================== */
//
//static int32_t ASP_MvToCode(uint32_t mv)
//{
//  return (int32_t)((mv * (ASP_ADC_FULL_SCALE + 1U)) / ASP_VREF_MV);
//}
//
///**
//  * @brief Build the DDS sine table without calling into the math library.
//  *
//  * A unit vector is rotated by one table step at a time:
//  *   s' = s*cos(d) + c*sin(d),  c' = c*cos(d) - s*sin(d),  d = 2*pi/1024.
//  * In double precision the error after a full turn is far below one LSB of the
//  * 16-bit table.
//  */
//static void ASP_BuildLut(void)
//{
//  const double sd = 0.0061358846491544753;   /* sin(2*pi/1024) */
//  const double cd = 0.9999811752826011;      /* cos(2*pi/1024) */
//  double s = 0.0;
//  double c = 1.0;
//
//  for (uint32_t i = 0U; i < ASP_LUT_SIZE; i++)
//  {
//    double v = 32767.0 * s;
//    asp_sin_lut[i] = (int16_t)((v >= 0.0) ? (v + 0.5) : (v - 0.5));
//
//    double ns = (s * cd) + (c * sd);
//    double nc = (c * cd) - (s * sd);
//    s = ns;
//    c = nc;
//  }
//}
//
///** Nanoseconds to TIM2 ticks. 240 ticks per microsecond at 240 MHz. */
//static uint32_t ASP_NsToTicks(uint32_t ns)
//{
//  return (uint32_t)(((uint64_t)ns * ASP_TIMER_KERNEL_CLK_HZ) / 1000000000ULL);
//}
//
///** Recompute the two DDS increments and the gate length. */
//static void ASP_RecalcTiming(void)
//{
//  ASP_Ctx_t *c = &ASP_Ctx;
//
//  /* Per ADC sample, used to walk the phase forward inside one chunk. */
//  c->phase_inc = (uint32_t)((double)c->sine_freq_hz * 4294967296.0 /
//                            (double)c->sample_rate_hz);
//
//  /* Per TIM2 tick, in Q8. This is the anchor: the phase of any instant is
//     (tick * inc_tick_q8) >> 8, so it can never drift no matter how many or
//     how few ADC samples turn up. Q8 rather than Q16 so that sine frequencies
//     up to ~937 kHz still fit in 32 bits. */
//  c->inc_tick_q8 = (uint32_t)((double)c->sine_freq_hz * 4294967296.0 * 256.0 /
//                              (double)ASP_TIMER_KERNEL_CLK_HZ);
//
//  /* Burst length in TIM2 ticks. 3 periods at 1 kHz = 720000 ticks = 3.000 ms. */
//  c->gate_ticks = (uint32_t)(((double)c->gate_sine_periods *
//                              (double)ASP_TIMER_KERNEL_CLK_HZ) /
//                             (double)c->sine_freq_hz);
//
//  /* TIM2 ticks between consecutive ADC samples: 240 MHz / 4.0 MSPS = 60.
//     Used to give every sample in a chunk its own timestamp. */
//  c->ticks_per_sample = ASP_TIMER_KERNEL_CLK_HZ / c->sample_rate_hz;
//  if (c->ticks_per_sample == 0U) { c->ticks_per_sample = 1U; }
//
//  /* Minimum hold times, ticks. 750 ns at 240 MHz = 180 ticks. */
//  c->min_on_ticks  = ASP_NsToTicks(c->min_on_ns);
//  c->max_on_ticks  = ASP_NsToTicks(c->max_on_ns);
//  c->max_off_ticks = ASP_NsToTicks(c->max_off_ns);
//  c->min_off_ticks = ASP_NsToTicks(c->min_off_ns);
//
//  /* PD14 delay, ticks. 1 us at 240 MHz = 240 ticks. */
//  c->pd14_delay_ticks = ASP_NsToTicks(c->pd14_delay_ns);
//
//  c->pd14_delay_min_ticks = ASP_NsToTicks(ASP_PD14_DELAY_MIN_NS);
//  c->pd14_delay_max_ticks = ASP_NsToTicks(ASP_PD14_DELAY_MAX_NS);
//  if (c->pd14_delay_min_ticks < 1U) { c->pd14_delay_min_ticks = 1U; }
//}
//
///* ==========================================================================
// * MPU
// * ========================================================================== */
//
//void ASP_MPU_Config(void)
//{
//  MPU_Region_InitTypeDef mpu = {0};
//
//  HAL_MPU_Disable();
//
//  mpu.Enable           = MPU_REGION_ENABLE;
//  mpu.Number           = MPU_REGION_NUMBER0;
//  mpu.BaseAddress      = ASP_BUF_BASE_ADDR;
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
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
//  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
//  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM       = 4;
//  RCC_OscInitStruct.PLL.PLLN       = 480;
//  RCC_OscInitStruct.PLL.PLLP       = 2;
//  RCC_OscInitStruct.PLL.PLLQ       = 4;
//  RCC_OscInitStruct.PLL.PLLR       = 2;
//  RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_1;
//  RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN   = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
//                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
//                                   | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief Run PLL2 at 32 MHz on the P output and route it to the ADCs.
//  *
//  * HSE 8 MHz / M 4 = 2 MHz reference, N 192 -> 384 MHz VCO, P 12 -> 32 MHz.
//  * One ADC cycle is 31.25 ns; a 12-bit conversion with 1.5 cycles of sampling
//  * is 8 cycles (250 ns) and the 8-cycle interleave delay is also 250 ns,
//  * exactly half of the 500 ns pair period.
//  *
//  * PLL2P is the one knob for the whole interleave; the VCO stays at 384 MHz
//  * for every entry in the table in adc_sine_pwm.h. PLL1 is untouched.
//  */
//static void ASP_AdcClockConfig(void)
//{
//  RCC_PeriphCLKInitTypeDef p = {0};
//
//  p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
//  p.PLL2.PLL2M      = 4;
//  p.PLL2.PLL2N      = 192;
//  p.PLL2.PLL2P      = 12;
//  p.PLL2.PLL2Q      = 2;
//  p.PLL2.PLL2R      = 2;
//  p.PLL2.PLL2RGE    = RCC_PLL2VCIRANGE_1;   /* 2 MHz reference */
//  p.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;      /* 384 MHz VCO     */
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
// * GPIO, DMA, ADC, timers
// * ========================================================================== */
//
//static void ASP_GpioConfig(void)
//{
//  GPIO_InitTypeDef g = {0};
//
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//
//  /* PC0 = ADC1_INP10 = ADC2_INP10, the shared interleaved input (A1). */
//  g.Pin  = GPIO_PIN_0;
//  g.Mode = GPIO_MODE_ANALOG;
//  g.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &g);
//
//  /* All three digital pins are driven low before being made outputs, so none
//     of them glitches high as it is configured. */
//  HAL_GPIO_WritePin(ASP_OUT_A_PORT, ASP_OUT_A_PIN, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(ASP_OUT_B_PORT, ASP_OUT_B_PIN, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(ASP_GATE_PORT,  ASP_GATE_PIN,  GPIO_PIN_RESET);
//
//  /* MEDIUM rather than VERY_HIGH deliberately. VERY_HIGH gives roughly 2 ns
//     edges on 3.3 V CMOS, which rings hard into any probe with a ground lead
//     and into neighbouring pins - and it buys nothing here, because the
//     shortest interval this design produces is the 750 ns hold time. MEDIUM is
//     still far faster than anything the loop can ask for. Drop it to LOW if the
//     edges still ring. */
//  g.Mode      = GPIO_MODE_OUTPUT_PP;
//  g.Pull      = GPIO_NOPULL;
//  g.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//  g.Alternate = 0;
//
//  g.Pin = ASP_OUT_A_PIN;   HAL_GPIO_Init(ASP_OUT_A_PORT, &g);   /* PB5  */
//  g.Pin = ASP_GATE_PIN;    HAL_GPIO_Init(ASP_GATE_PORT,  &g);   /* PF3  */
//
//#if ASP_PD14_HARDWARE
//  /* PD14 belongs to TIM4_CH3, not to the GPIO block. */
//  g.Pin       = ASP_OUT_B_PIN;
//  g.Mode      = GPIO_MODE_AF_PP;
//  g.Alternate = GPIO_AF2_TIM4;
//  HAL_GPIO_Init(ASP_OUT_B_PORT, &g);
//  g.Alternate = 0;
//  g.Mode      = GPIO_MODE_OUTPUT_PP;
//#else
//  g.Pin = ASP_OUT_B_PIN;   HAL_GPIO_Init(ASP_OUT_B_PORT, &g);   /* PD14 */
//#endif
//
//#if ASP_SAMPLE_CLK_ENABLE
//  HAL_GPIO_WritePin(ASP_CLK_PORT, ASP_CLK_PIN, GPIO_PIN_RESET);
//  g.Pin = ASP_CLK_PIN;     HAL_GPIO_Init(ASP_CLK_PORT,   &g);   /* PD15 */
//#endif
//
//#if ASP_DAC_ECHO_ENABLE
//  /* PA4 = DAC1_OUT1. Analog mode, no pull. */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  g.Pin       = GPIO_PIN_4;
//  g.Mode      = GPIO_MODE_ANALOG;
//  g.Pull      = GPIO_NOPULL;
//  g.Alternate = 0;
//  HAL_GPIO_Init(GPIOA, &g);
//#endif
//}
//
//#if ASP_DAC_ECHO_ENABLE
///**
//  * @brief DAC1 channel 1 on PA4, no trigger, unbuffered.
//  *
//  * With DAC_TRIGGER_NONE a write to DHR12R1 reaches the output register on the
//  * next APB cycle, so the sample loop can just store to it - no DMA, no timer.
//  * Unbuffered because it settles faster; probe it with a scope and do not load
//  * the pin.
//  */
//static DAC_HandleTypeDef hdac_echo;
//
//static void ASP_DacEchoConfig(void)
//{
//  DAC_ChannelConfTypeDef ch = {0};
//
//  __HAL_RCC_DAC12_CLK_ENABLE();
//
//  hdac_echo.Instance = DAC1;
//  if (HAL_DAC_Init(&hdac_echo) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  ch.DAC_SampleAndHold            = DAC_SAMPLEANDHOLD_DISABLE;
//  ch.DAC_Trigger                  = DAC_TRIGGER_NONE;
//  ch.DAC_OutputBuffer             = DAC_OUTPUTBUFFER_DISABLE;
//  ch.DAC_ConnectOnChipPeripheral  = DAC_CHIPCONNECT_DISABLE;
//  ch.DAC_UserTrimming             = DAC_TRIMMING_FACTORY;
//  if (HAL_DAC_ConfigChannel(&hdac_echo, &ch, DAC_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_DAC_Start(&hdac_echo, DAC_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//#endif
//
//static void ASP_DmaConfig(void)
//{
//  __HAL_RCC_DMA2_CLK_ENABLE();
//
//  hdma_adc.Instance                 = DMA2_Stream0;
//  hdma_adc.Init.Request             = DMA_REQUEST_ADC1;
//  hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
//  hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
//  hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
//  /* In dual interleaved mode the source is the common data register, which
//     packs both results into one 32-bit word, so both sides are WORD. */
//  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//  hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
//  hdma_adc.Init.Mode                = DMA_CIRCULAR;
//  hdma_adc.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
//  hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
//
//  if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* HAL_ADCEx_MultiModeStart_DMA() arms the transfer-complete interrupt, so
//     the vector has to be serviced even though the loop polls NDTR. Priority is
//     irrelevant - nothing else runs. */
//  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
//}
//
//static void ASP_AdcConfig(void)
//{
//  ADC_ChannelConfTypeDef ch = {0};
//  ADC_MultiModeTypeDef   mm = {0};
//
//  __HAL_RCC_ADC12_CLK_ENABLE();
//
//  /* ---- ADC1: the master, it owns the trigger and the DMA --------------- */
//  hadc_master.Instance                      = ADC1;
//  hadc_master.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
//  hadc_master.Init.Resolution               = ADC_RESOLUTION_12B;
//  hadc_master.Init.ScanConvMode             = ADC_SCAN_DISABLE;
//  hadc_master.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
//  hadc_master.Init.LowPowerAutoWait         = DISABLE;
//  hadc_master.Init.ContinuousConvMode       = DISABLE;
//  hadc_master.Init.NbrOfConversion          = 1;
//  hadc_master.Init.DiscontinuousConvMode    = DISABLE;
//  hadc_master.Init.ExternalTrigConv         = ADC_EXTERNALTRIG_T8_TRGO;
//  hadc_master.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
//  hadc_master.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
//  hadc_master.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
//  hadc_master.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
//  hadc_master.Init.OversamplingMode         = DISABLE;
//  if (HAL_ADC_Init(&hadc_master) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  __HAL_LINKDMA(&hadc_master, DMA_Handle, hdma_adc);
//
//  /* ---- ADC2: the slave. Its trigger field is ignored in dual mode and it
//         never drives the DMA; its result is merged into the common data
//         register by the master. ------------------------------------------ */
//  hadc_slave = hadc_master;
//  hadc_slave.Instance                      = ADC2;
//  hadc_slave.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
//  hadc_slave.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc_slave.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  hadc_slave.DMA_Handle                    = NULL;
//  hadc_slave.State                         = 0U;
//  hadc_slave.Lock                          = HAL_UNLOCKED;
//  if (HAL_ADC_Init(&hadc_slave) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Calibration has to run with the ADC disabled, i.e. before any start. */
//  if (HAL_ADCEx_Calibration_Start(&hadc_master, ADC_CALIB_OFFSET_LINEARITY,
//                                  ADC_SINGLE_ENDED) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_ADCEx_Calibration_Start(&hadc_slave, ADC_CALIB_OFFSET_LINEARITY,
//                                  ADC_SINGLE_ENDED) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Both ADCs must sample the same pin, otherwise this is not interleaving. */
//  ch.Channel      = ADC_CHANNEL_10;
//  ch.Rank         = ADC_REGULAR_RANK_1;
//  ch.SamplingTime = ASP_ADC_SAMPLETIME;
//  ch.SingleDiff   = ADC_SINGLE_ENDED;
//  ch.OffsetNumber = ADC_OFFSET_NONE;
//  ch.Offset       = 0;
//  if (HAL_ADC_ConfigChannel(&hadc_master, &ch) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_ADC_ConfigChannel(&hadc_slave, &ch) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* ---- dual interleaved ------------------------------------------------
//     ADC_DUALMODEDATAFORMAT_32_10_BITS makes the common data register hold both
//     results in one 32-bit word: master in bits 15:0, slave in bits 31:16.
//     HAL_ADCEx_MultiModeConfigChannel() writes it into ADC12_COMMON->CCR.DAMDF,
//     and HAL_ADCEx_MultiModeStart_DMA() then points the DMA at CDR rather than
//     the master's own DR. */
//  mm.Mode             = ADC_DUALMODE_INTERL;
//  mm.DualModeData     = ADC_DUALMODEDATAFORMAT_32_10_BITS;
//  mm.TwoSamplingDelay = ASP_TWO_SAMPLING_DELAY;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc_master, &mm) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///** TIM8: the ADC sample trigger. TRGO on update, no output pin. */
//static void ASP_TrigTimerConfig(void)
//{
//  TIM_MasterConfigTypeDef m = {0};
//
//  __HAL_RCC_TIM8_CLK_ENABLE();
//
//  uint32_t arr = (ASP_TIMER_KERNEL_CLK_HZ / ASP_PAIR_RATE_HZ_TARGET) - 1U;
//
//  htim_trig.Instance               = TIM8;
//  htim_trig.Init.Prescaler         = 0;
//  htim_trig.Init.CounterMode       = TIM_COUNTERMODE_UP;
//  htim_trig.Init.Period            = arr;
//  htim_trig.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htim_trig.Init.RepetitionCounter = 0;
//  htim_trig.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim_trig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  m.MasterOutputTrigger  = TIM_TRGO_UPDATE;
//  m.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  m.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim_trig, &m) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  ASP_Ctx.pair_rate_hz   = ASP_TIMER_KERNEL_CLK_HZ / (arr + 1U);
//  ASP_Ctx.sample_rate_hz = ASP_Ctx.pair_rate_hz * 2U;
//}
//
//#if ASP_PD14_HARDWARE
///**
//  * @brief TIM4: places the PD14 edge in hardware.
//  *
//  * Free-running 16-bit at 240 MHz, wrapping every 273 us. Channel 3 is on PD14
//  * (AF2) in output-compare TOGGLE mode with CC3E permanently set; software
//  * only ever writes CCR3. CCR3 preload is left DISABLED so that a write takes
//  * effect immediately rather than at the next update event.
//  */
//static void ASP_Pd14TimerConfig(void)
//{
//  __HAL_RCC_TIM4_CLK_ENABLE();
//
//  TIM4->CR1  = 0U;
//  TIM4->PSC  = 0U;
//  TIM4->ARR  = 0xFFFFU;
//  TIM4->CNT  = 0U;
//  TIM4->SMCR = 0U;
//
//  /* Channel 3: no preload, start in the idle forced level so the pin's parity
//     is defined before toggling begins. */
//  TIM4->CCMR2 = 0U;
//  ASP_Pd14SetMode(ASP_PD14_IDLE_MODE);
//
//  TIM4->CCER = 0U;
//  TIM4->CCER |= TIM_CCER_CC3E;          /* stays enabled for good */
//  TIM4->CCR3 = 0U;
//  TIM4->EGR  = TIM_EGR_UG;
//  TIM4->SR   = 0U;
//  TIM4->CR1 |= TIM_CR1_CEN;
//}
//#endif
//
///**
//  * @brief TIM2: a free-running 32-bit counter at 240 MHz, nothing attached.
//  *
//  * This is the time reference for both the DDS phase and the gate. 32 bits at
//  * 240 MHz wraps after 17.9 s; the counter is zeroed at the start of every
//  * burst, so a burst would have to be longer than that to see a wrap.
//  */
//static void ASP_TimeBaseConfig(void)
//{
//  __HAL_RCC_TIM2_CLK_ENABLE();
//
//  htim_time.Instance               = TIM2;
//  htim_time.Init.Prescaler         = 0;
//  htim_time.Init.CounterMode       = TIM_COUNTERMODE_UP;
//  htim_time.Init.Period            = 0xFFFFFFFFUL;
//  htim_time.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htim_time.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim_time) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///* ==========================================================================
// * Interrupt handlers
// * ========================================================================== */
//
//void DMA2_Stream0_IRQHandler(void)
//{
//  HAL_DMA_IRQHandler(&hdma_adc);
//}
//
///* ==========================================================================
// * The hysteretic loop
// * ========================================================================== */
//
///**
//  * @brief Run one gated burst, start to finish. Blocking.
//  *
//  * Structure of the inner loop:
//  *   - poll the DMA's NDTR to see how many new interleaved pairs have landed
//  *   - re-anchor the DDS phase to TIM2 for the oldest sample of that chunk,
//  *     so the reference can never drift away from real time
//  *   - for each sample: boxcar filter, build the reference, compare against
//  *     the +/- threshold band, drive the pin
//  */
//void ASP_RunBurst(void)
//{
//  ASP_Ctx_t *c = &ASP_Ctx;
//
//  ASP_RecalcTiming();
//
//  /* --- reset all per-burst state ---------------------------------------- */
//  c->pairs_seen      = 0U;
//  c->pairs_total     = 0U;
//  c->samples_total   = 0U;
//  c->dropped_pairs   = 0U;
//  c->switch_count    = 0U;
//  c->min_time_blocks   = 0U;
//  c->pending_applied   = 0U;
//  c->forced_flip_count = 0U;
//  c->starve_count    = 0U;
//  c->overrun_count   = 0U;
//  c->pd14_late_count    = 0U;
//  c->pd14_drop_count    = 0U;
//  c->pd14_depth_max     = 0U;
//  c->pd14_reorder_count = 0U;
//  c->pd14_parity_risk   = 0U;
//
//  asp_dq_head     = 0U;
//  asp_dq_tail     = 0U;
//
//#if ASP_PD14_HARDWARE
//  /* Establish PD14's level with a forced write, then hand the channel over to
//     toggle mode. From here every match flips the pin, so its parity follows
//     the queue exactly. */
//  ASP_Pd14SetMode(ASP_PD14_IDLE_MODE);
//  TIM4->SR = ~TIM_SR_CC3IF;
//  ASP_Pd14SetMode(TIM_OCMODE_TOGGLE);
//  ASP_Pd14Park();
//  asp_dq_last_due = TIM4->CNT;
//#else
//  asp_dq_last_due = 0U;
//#endif
//
//  uint64_t master_sum = 0U;
//  uint64_t slave_sum  = 0U;
//  int32_t  ma_min = 0x7FFFFFFF, ma_max = -0x7FFFFFFF;
//  int32_t  sl_min = 0x7FFFFFFF, sl_max = -0x7FFFFFFF;
//
//#if ASP_SAMPLE_CLK_ENABLE
//  uint32_t clk_hi = 0U;
//#endif
//#if ASP_DAC_ECHO_ENABLE
//  uint32_t echo_ctr = 0U;
//#endif
//
//  c->decisions              = 0U;
//  c->decision_gap_max_ticks = 0U;
//  c->straddle_count         = 0U;
//
//  const int32_t  centre = c->center_code;
//  const int32_t  ampl   = c->ampl_code;
//  const int32_t  thr    = c->thr_code;
//  const uint32_t inc_t  = c->inc_tick_q8;
//  const uint32_t gate   = c->gate_ticks;
//  const uint32_t tps    = c->ticks_per_sample;
//  const uint32_t min_on  = c->min_on_ticks;
//  const uint32_t min_off = c->min_off_ticks;
//  const uint32_t max_on  = c->max_on_ticks;
//  const uint32_t max_off = c->max_off_ticks;
//  bool     pending_flip  = false;   /* an edge refused by the min-time guard */
//
//  /* The physical pin level, which is what the minimum on and off times are
//     about - out_state is the logical decision, and ASP_OUTPUT_INVERT can put
//     the two out of step. */
//  bool pin_hi = false;
//
//  /* PD14 delay line state, held in locals through the hot loop. */
//  const bool     adaptive = c->pd14_adaptive;
//  const uint32_t dly_min  = c->pd14_delay_min_ticks;
//  const uint32_t dly_max  = c->pd14_delay_max_ticks;
//  const uint32_t slew     = c->pd14_slew_ticks;
//  uint32_t dly_live       = c->pd14_delay_ticks;   /* starts at the fixed value */
//  uint32_t half_period    = c->pd14_delay_ticks;   /* seeded, not measured yet  */
//  uint32_t last_edge_t    = 0U;
//  uint32_t last_decision_t = 0U;
//
//  /* --- start the hardware ----------------------------------------------- */
//  if (HAL_ADCEx_MultiModeStart_DMA(&hadc_master, (uint32_t *)ASP_BUF_BASE_ADDR,
//                                   ASP_BUF_WORDS) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_Base_Start(&htim_trig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Let the pipeline fill so the first chunk is real data, not the pattern
//     the buffer was pre-loaded with. */
//  for (volatile uint32_t d = 0U; d < 10000U; d++) { __NOP(); }
//
//  uint32_t rd = (ASP_BUF_WORDS - (uint32_t)(DMA2_Stream0->NDTR)) & ASP_BUF_MASK;
//
//  /* Zero the time base and open the gate in the same breath: t = 0 is the
//     rising edge of PF3, and the sine starts from phase 0 there. */
//  bool out_state = false;
//  ASP_GATE_PORT->BSRR = (uint32_t)ASP_GATE_PIN;
//  TIM2->CNT = 0U;
//  if (HAL_TIM_Base_Start(&htim_time) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* --- the loop ---------------------------------------------------------- */
//  uint32_t now = 0U;
//
//  while ((now = TIM2->CNT) < gate)
//  {
//    uint32_t widx  = (ASP_BUF_WORDS - (uint32_t)(DMA2_Stream0->NDTR)) & ASP_BUF_MASK;
//    uint32_t avail = (widx - rd) & ASP_BUF_MASK;
//
//    /* Drain the PD14 queue on every pass, including passes with no new ADC
//       data - otherwise a quiet stretch would hold a due edge back. */
//    ASP_ServiceDelayLine(now);
//
//    if (avail == 0U)
//    {
//      c->starve_count++;
//      continue;
//    }
//
//    if (avail > (ASP_BUF_WORDS / 2U))
//    {
//      /* Fell far behind - samples were overwritten. Drop to the newest few and
//         carry on; the reference is re-derived from TIM2 below, so skipping
//         samples cannot shift the sine. */
//      rd = (widx - 8U) & ASP_BUF_MASK;
//      avail = 8U;
//      c->overrun_count++;
//    }
//
//    c->pairs_seen += avail;
//
//    /* Cap how much time one decision covers. The maximum of a chunk is only
//       meaningful while the chunk is short next to a half period of the input;
//       let it span a whole ramp and the maximum is nearly full scale, always
//       above the band, and the output latches. So keep only the newest pairs
//       and throw the backlog away - a comparator has no state to lose, and
//       discarding also stops the loop delay growing. */
//    if (avail > ASP_MAX_PAIRS_PER_DECISION)
//    {
//      uint32_t drop = avail - ASP_MAX_PAIRS_PER_DECISION;
//      rd = (rd + drop) & ASP_BUF_MASK;
//      c->dropped_pairs += drop;
//      avail = ASP_MAX_PAIRS_PER_DECISION;
//    }
//
//    uint32_t nsamp = avail * 2U;
//
//    /* The newest sample of this chunk was converted about one sample period
//       ago; everything is referenced to that instant, in both the phase and the
//       time domain. Re-derived from TIM2 every chunk, so neither can drift. */
//    uint32_t t_newest = now - tps;
//    uint32_t phase    = (uint32_t)(((uint64_t)t_newest * inc_t) >> 8);
//
//    int32_t peak   = -1;
//    int32_t newest = 0;             /* last sample of the chunk, in time */
//#if ASP_LOWER_USES_MIN
//    int32_t trough = 0x7FFFFFFF;
//#endif
//
//    for (uint32_t i = 0U; i < avail; i++)
//    {
//      uint32_t w = asp_buf[rd];
//      rd = (rd + 1U) & ASP_BUF_MASK;
//
//      /* Master result first in time, slave half a sample period later. */
//      int32_t s0 = (int32_t)(w & 0xFFFFU);
//      int32_t s1 = (int32_t)((w >> 16) & 0xFFFFU);
//
//#if ASP_STATS_ENABLE
//      /* Per-converter statistics, so the interleaving can be checked after the
//         fact. Held in locals, not in the context struct, so that even at low
//         optimisation levels this is registers rather than memory. */
//      master_sum += (uint32_t)s0;
//      slave_sum  += (uint32_t)s1;
//      if (s0 < ma_min) { ma_min = s0; }
//      if (s0 > ma_max) { ma_max = s0; }
//      if (s1 < sl_min) { sl_min = s1; }
//      if (s1 > sl_max) { sl_max = s1; }
//#endif
//
//      /* --- reduce the chunk to its peak -------------------------------- */
//      if (s0 > peak) { peak = s0; }
//      if (s1 > peak) { peak = s1; }
//      newest = s1;                  /* slave converts last */
//#if ASP_LOWER_USES_MIN
//      if (s0 < trough) { trough = s0; }
//      if (s1 < trough) { trough = s1; }
//#endif
//
//#if ASP_SAMPLE_CLK_ENABLE
//      /* Two toggles per pair, i.e. one per sample, so PD15 carries half the
//         sample rate: 2.000 MHz for a genuine 4.0 MSPS interleaved stream. */
//      ASP_CLK_PORT->BSRR = clk_hi ? (uint32_t)ASP_CLK_PIN
//                                  : ((uint32_t)ASP_CLK_PIN << 16);
//      clk_hi ^= 1U;
//      ASP_CLK_PORT->BSRR = clk_hi ? (uint32_t)ASP_CLK_PIN
//                                  : ((uint32_t)ASP_CLK_PIN << 16);
//      clk_hi ^= 1U;
//#endif
//#if ASP_DAC_ECHO_ENABLE
//      /* Decimated echo of the raw conversions, so the digitised waveform can
//         be seen on a scope without asking the DAC to settle faster than it
//         can. What to read off it is the period, not the shape. */
//      if ((echo_ctr++ & (ASP_DAC_ECHO_DECIM - 1U)) == 0U)
//      {
//        DAC1->DHR12R1 = (uint32_t)s0 & 0x0FFFU;
//      }
//      if ((echo_ctr++ & (ASP_DAC_ECHO_DECIM - 1U)) == 0U)
//      {
//        DAC1->DHR12R1 = (uint32_t)s1 & 0x0FFFU;
//      }
//#endif
//    }
//
//    /* --- one reference, one comparison, at most one edge ---------------- */
//
//    /* The reference is evaluated once for the whole chunk, at the instant the
//       newest sample was converted. A chunk is a few hundred nanoseconds, which
//       at a 1 kHz sine is a fraction of a degree, so nothing is lost by not
//       walking the phase sample by sample. */
//    int32_t ref = centre +
//                  (((int32_t)asp_sin_lut[phase >> ASP_LUT_SHIFT] * ampl) >> 15);
//
//#if ASP_LOWER_USES_MIN
//    int32_t lo_stat = trough;
//#else
//    int32_t lo_stat = peak;
//#endif
//
//    /* The comparator's opinion, as a tri-state: -1 wants the output low,
//       +1 wants it high, 0 means the input is inside the band and it has no
//       opinion at all. Keeping "no opinion" distinct from "wants what it has
//       already got" is what lets a refused edge be remembered safely below. */
//    int32_t cmp   = 0;
//    bool    want  = out_state;
//    bool    above = (peak    > (ref + thr));
//    bool    below = (lo_stat < (ref - thr));
//
//    if (above && below)
//    {
//      /* Both tests true at once. That happens because the two samples of a
//         pair are 250 ns apart, and on a fast input that is a long way in
//         volts - 750 mV on a 3 V triangle at 500 kHz, wider than the 400 mV
//         band - so the pair can span the whole band at once. Taking the upper
//         test first would then always force LOW and swallow the rising edge.
//         Decide from the NEWEST sample instead: it is the one that says which
//         side the signal is actually heading for. */
//      want = (newest > ref) ? false : true;
//      cmp  = want ? 1 : -1;
//      c->straddle_count++;
//    }
//    else if (above)
//    {
//      want = false;                 /* above the band -> output low  */
//      cmp  = -1;
//    }
//    else if (below)
//    {
//      want = true;                  /* below the band -> output high */
//      cmp  = 1;
//    }
//    else
//    {
//      /* inside the band: no opinion, hold */
//      cmp = 0;
//    }
//
//    /* How long since the previous comparison. This is the number that decides
//       whether a crossing can be missed - see decision_gap_max_ticks. */
//    uint32_t gap_dec = now - last_decision_t;
//    if ((c->decisions != 0U) && (gap_dec > c->decision_gap_max_ticks))
//    {
//      c->decision_gap_max_ticks = gap_dec;
//    }
//    last_decision_t = now;
//
//    c->decisions++;
//    c->last_input = peak;
//    c->last_ref   = ref;
//
//    /* --- minimum on and off time ---------------------------------------
//       The pin has to have been at its present level for at least the relevant
//       hold time before it is allowed to leave it. Nothing is latched: the
//       comparison is remade from fresh samples next pass, so the edge lands as
//       soon as the hold expires if the input still wants it. */
//    /* Both ends measured from `now`. last_edge_t is stamped with `now` when an
//       edge is taken, so measuring from t_newest (one sample period earlier)
//       understated the hold by 250 ns and made the guard hold 250 ns longer
//       than asked - an effective 1000 ns floor instead of 750 ns, enough on its
//       own to start skipping cycles above about 430 kHz. */
//    uint32_t held = now - last_edge_t;
//    uint32_t need = pin_hi ? min_on : min_off;
//    uint32_t cap  = pin_hi ? max_on : max_off;
//
//    bool flip = (want != out_state);
//
//    if (flip && (held < need))
//    {
//      /* Too soon. REMEMBER it rather than throw it away. Discarding a refused
//         edge is what latches the output: the input goes back through the band,
//         comes out the other side asking for the state the pin is already in,
//         nothing happens, and it stays there indefinitely. */
//      pending_flip = true;
//      c->min_time_blocks++;
//      flip = false;
//    }
//    else if (flip)
//    {
//      pending_flip = false;            /* honoured now, nothing outstanding */
//    }
//    else if (pending_flip && (held >= need))
//    {
//      /* The hold has expired, so honour the edge refused earlier. Only reached
//         while the comparator has no opinion - if it had actively asked for the
//         current state, the branch below would have cleared the request. */
//      flip         = true;
//      pending_flip = false;
//      c->pending_applied++;
//    }
//    else if (pending_flip && (cmp != 0))
//    {
//      /* The comparator has changed its mind and now wants what the pin already
//         has, so the outstanding request is stale. */
//      pending_flip = false;
//    }
//
//    /* --- hard dwell ceiling ---------------------------------------------
//       Whatever the comparator and the queue say, the pin may not sit in one
//       state longer than this. It is the backstop against any remaining way of
//       wedging the loop, and it puts a floor on switching frequency. */
//    if (!flip && (cap != 0U) && (held >= cap))
//    {
//      flip = true;
//      c->forced_flip_count++;
//    }
//
//    if (flip)
//    {
//      want = !out_state;
//      out_state = want;
//      c->switch_count++;
//
//      /* PB5 moves immediately. */
//#if ASP_OUTPUT_INVERT
//      bool drive = !out_state;
//#else
//      bool drive = out_state;
//#endif
//      pin_hi = drive;
//
//      /* PB5 moves here, and PD14's due time is taken from the very same
//         instant - one counter read either side of the store. Both edges then
//         share a single reference, so their spacing is the delay and nothing
//         else. Scheduling PD14 from the sample's conversion time instead used
//         to make the spacing short by one sample period. */
//#if ASP_PD14_HARDWARE
//      uint32_t t_ref = TIM4->CNT;
//      ASP_OUT_A_PORT->BSRR = drive ? (uint32_t)ASP_OUT_A_PIN
//                                   : ((uint32_t)ASP_OUT_A_PIN << 16);
//#else
//      ASP_OUT_A_PORT->BSRR = drive ? (uint32_t)ASP_OUT_A_PIN
//                                   : ((uint32_t)ASP_OUT_A_PIN << 16);
//      uint32_t t_ref = TIM2->CNT;
//#endif
//
//      /* Interval since the previous edge. Because the output alternates, this
//         interval IS the switching half period, so a smoothed version of it is
//         exactly what the PD14 delay should be for interleaving. */
//      uint32_t gap = now - last_edge_t;
//      last_edge_t = now;
//
//      if ((gap > 0U) && (gap < (dly_max * 4U)))
//      {
//        half_period += ((int32_t)gap - (int32_t)half_period) >>
//                       ASP_PD14_EMA_SHIFT;
//      }
//
//      if (adaptive)
//      {
//        /* Walk the delay towards that estimate, a bounded step per edge, so it
//           never jumps. */
//        uint32_t target = half_period;
//        if (target < dly_min) { target = dly_min; }
//        if (target > dly_max) { target = dly_max; }
//
//        if (dly_live < target)
//        {
//          uint32_t d = target - dly_live;
//          dly_live += (d > slew) ? slew : d;
//        }
//        else
//        {
//          uint32_t d = dly_live - target;
//          dly_live -= (d > slew) ? slew : d;
//        }
//      }
//
//      /* PD14 gets the same edge, one delay after the PB5 edge. */
//#if ASP_PD14_HARDWARE
//      uint32_t due = (t_ref + dly_live) & 0xFFFFU;
//#else
//      uint32_t due = t_ref + dly_live;
//      if ((int32_t)(now - due) > 0) { c->pd14_late_count++; }
//#endif
//      ASP_QueueEdge(due, drive);
//    }
//
//    /* The queue is checked for emptiness before reading TIM2: it sits on APB
//       and the read stalls the core for several cycles, not worth paying when
//       nothing is pending. */
//    if (asp_dq_head != asp_dq_tail)
//    {
//      ASP_ServiceDelayLine(TIM2->CNT);
//    }
//
//    c->pairs_total   += avail;
//    c->samples_total += nsamp;
//  }
//
//  /* --- shut down -------------------------------------------------------- */
//  /* PB5 stops at the gate, but PD14 is one delay behind it, so keep draining
//     the queue for that long before pulling everything down. Otherwise the
//     last delay's worth of PD14 edges would simply be lost. */
//  c->elapsed_ticks = TIM2->CNT;
//
//  uint32_t drain_until = c->elapsed_ticks + dly_live;
//  ASP_OUT_A_PORT->BSRR = (uint32_t)ASP_OUT_A_PIN << 16;
//
//  while ((int32_t)(TIM2->CNT - drain_until) < 0)
//  {
//    ASP_ServiceDelayLine(TIM2->CNT);
//  }
//  ASP_ServiceDelayLine(TIM2->CNT);
//
//#if ASP_PD14_HARDWARE
//  /* Back to a forced level, so PD14 parks where it started and no stale
//     compare value can toggle it between bursts. */
//  ASP_Pd14SetMode(ASP_PD14_IDLE_MODE);
//#else
//  ASP_OUT_B_PORT->BSRR = (uint32_t)ASP_OUT_B_PIN << 16;
//#endif
//  ASP_GATE_PORT->BSRR  = (uint32_t)ASP_GATE_PIN << 16;
//
//  c->last_output       = out_state;
//  c->pd14_delay_live   = dly_live;
//  c->half_period_ticks = half_period;
//
//  HAL_TIM_Base_Stop(&htim_time);
//  HAL_TIM_Base_Stop(&htim_trig);
//  HAL_ADCEx_MultiModeStop_DMA(&hadc_master);
//
//  /* --- work out what actually happened ---------------------------------- */
//  float secs = (float)c->elapsed_ticks / (float)ASP_TIMER_KERNEL_CLK_HZ;
//
//  /* From pairs_seen, not samples_total: pairs_seen counts everything the DMA
//     delivered, including any backlog that was discarded, so it is the real
//     converter throughput rather than what the loop chose to look at. */
//  c->measured_sample_rate_hz = (secs > 0.0f)
//                             ? (((float)c->pairs_seen * 2.0f) / secs)
//                             : 0.0f;
//  /* Two transitions make one switching cycle. */
//  c->measured_switch_freq_hz = (secs > 0.0f)
//                             ? (((float)c->switch_count * 0.5f) / secs)
//                             : 0.0f;
//
//  c->master_min = ma_min;
//  c->master_max = ma_max;
//  c->slave_min  = sl_min;
//  c->slave_max  = sl_max;
//
//  if (c->pairs_total != 0U)
//  {
//    c->master_avg = (int32_t)(master_sum / c->pairs_total);
//    c->slave_avg  = (int32_t)(slave_sum  / c->pairs_total);
//  }
//}
//
///* ==========================================================================
// * Public API
// * ========================================================================== */
//
//void ASP_SetSineFreqHz(float hz)
//{
//  if (hz > 0.0f) { ASP_Ctx.sine_freq_hz = hz; }
//}
//
//void ASP_SetSineCenterMv(uint32_t mv)
//{
//  ASP_Ctx.center_code = ASP_MvToCode(mv);
//}
//
//void ASP_SetSineAmplitudeMv(uint32_t mv)
//{
//  ASP_Ctx.ampl_code = ASP_MvToCode(mv);
//}
//
//void ASP_SetThresholdMv(uint32_t mv)
//{
//  int32_t t = ASP_MvToCode(mv);
//  ASP_Ctx.thr_code = (t < 1) ? 1 : t;
//}
//
//void ASP_SetGateSinePeriods(float periods)
//{
//  if (periods > 0.0f) { ASP_Ctx.gate_sine_periods = periods; }
//}
//
//void ASP_SetMaxTimesNs(uint32_t on_ns, uint32_t off_ns)
//{
//  /* A ceiling at or below the corresponding floor would force an edge that the
//     minimum-time guard then has to refuse, so raise it clear of the floor. */
//  if ((on_ns != 0U) && (on_ns <= ASP_Ctx.min_on_ns))
//  {
//    on_ns = ASP_Ctx.min_on_ns * 2U;
//  }
//  if ((off_ns != 0U) && (off_ns <= ASP_Ctx.min_off_ns))
//  {
//    off_ns = ASP_Ctx.min_off_ns * 2U;
//  }
//
//  ASP_Ctx.max_on_ns  = on_ns;
//  ASP_Ctx.max_off_ns = off_ns;
//  ASP_RecalcTiming();
//}
//
//void ASP_SetMinTimesNs(uint32_t on_ns, uint32_t off_ns)
//{
//  ASP_Ctx.min_on_ns  = on_ns;
//  ASP_Ctx.min_off_ns = off_ns;
//  ASP_RecalcTiming();
//}
//
//void ASP_SetPd14DelayNs(uint32_t ns)
//{
//  ASP_Ctx.pd14_delay_ns = ns;
//  ASP_RecalcTiming();
//}
//
//void ASP_SetPd14Adaptive(bool enable, uint32_t slew_ns_per_edge)
//{
//  ASP_Ctx.pd14_adaptive  = enable;
//  ASP_Ctx.pd14_slew_ticks = ASP_NsToTicks(slew_ns_per_edge);
//  if (ASP_Ctx.pd14_slew_ticks < 1U) { ASP_Ctx.pd14_slew_ticks = 1U; }
//}
//
//void ASP_Init(void)
//{
//  ASP_Ctx_t *c = &ASP_Ctx;
//
//  c->center_code       = ASP_MvToCode(ASP_SINE_CENTER_MV_DEFAULT);
//  c->ampl_code         = ASP_MvToCode(ASP_SINE_AMPL_MV_DEFAULT);
//  c->thr_code          = ASP_MvToCode(ASP_THRESHOLD_MV_DEFAULT);
//  if (c->thr_code < 1) { c->thr_code = 1; }
//
//  c->sine_freq_hz      = ASP_SINE_FREQ_HZ_DEFAULT;
//  c->gate_sine_periods = ASP_GATE_SINE_PERIODS_DEFAULT;
//  c->min_on_ns         = ASP_MIN_ON_NS_DEFAULT;
//  c->min_off_ns        = ASP_MIN_OFF_NS_DEFAULT;
//  c->max_on_ns         = ASP_MAX_ON_NS_DEFAULT;
//  c->max_off_ns        = ASP_MAX_OFF_NS_DEFAULT;
//  c->pd14_delay_ns     = ASP_PD14_DELAY_NS_DEFAULT;
//  c->pd14_adaptive     = ASP_PD14_ADAPTIVE;
//  c->pd14_slew_ticks   = ASP_NsToTicks(ASP_PD14_SLEW_NS_PER_EDGE);
//  if (c->pd14_slew_ticks < 1U) { c->pd14_slew_ticks = 1U; }
//
//  ASP_BuildLut();
//
//  ASP_AdcClockConfig();
//  ASP_GpioConfig();
//  ASP_DmaConfig();
//  ASP_AdcConfig();
//  ASP_TrigTimerConfig();   /* fills in pair_rate_hz / sample_rate_hz */
//  ASP_TimeBaseConfig();
//#if ASP_PD14_HARDWARE
//  ASP_Pd14TimerConfig();
//#endif
//#if ASP_DAC_ECHO_ENABLE
//  ASP_DacEchoConfig();
//#endif
//
//  ASP_RecalcTiming();
//
//  /* Park the buffer at the sine centre so a stale SRAM pattern cannot be
//     mistaken for data. */
//  for (uint32_t i = 0U; i < ASP_BUF_WORDS; i++)
//  {
//    asp_buf[i] = ((uint32_t)c->center_code) | ((uint32_t)c->center_code << 16);
//  }
//}
//
///* ==========================================================================
// * main
// * ========================================================================== */
//
//int main(void)
//{
//  /* The MPU has to be set up before the D-cache is turned on, so the DMA
//     buffer region is never cached in the first place. */
//  ASP_MPU_Config();
//
//  SCB_EnableICache();
//  SCB_EnableDCache();
//
//  HAL_Init();
//  SystemClock_Config();
//
//  ASP_Init();
//
//  /* These ARE the defaults from adc_sine_pwm.h; change them here or there. */
//  ASP_SetSineFreqHz(ASP_SINE_FREQ_HZ_DEFAULT);
//  ASP_SetSineCenterMv(ASP_SINE_CENTER_MV_DEFAULT);
//  ASP_SetSineAmplitudeMv(ASP_SINE_AMPL_MV_DEFAULT);
//  ASP_SetThresholdMv(ASP_THRESHOLD_MV_DEFAULT);
//  ASP_SetGateSinePeriods(ASP_GATE_SINE_PERIODS_DEFAULT);
//  ASP_SetMinTimesNs(ASP_MIN_ON_NS_DEFAULT, ASP_MIN_OFF_NS_DEFAULT);
//  ASP_SetMaxTimesNs(ASP_MAX_ON_NS_DEFAULT, ASP_MAX_OFF_NS_DEFAULT);
//  ASP_SetPd14DelayNs(ASP_PD14_DELAY_NS_DEFAULT);
//  ASP_SetPd14Adaptive(ASP_PD14_ADAPTIVE, ASP_PD14_SLEW_NS_PER_EDGE);
//
//  /* One burst: PF3 up, hysteretic loop for 3 sine periods, PF3 down. */
//  ASP_RunBurst();
//
//  /* Everything worth knowing is now in ASP_Ctx. Put a breakpoint here and
//     read, in this order:
//       measured_sample_rate_hz  expect 4.0e6. 2.0e6 means only one conversion
//                                per trigger, i.e. the interleaving is not
//                                working. Anything else is a real rate, and the
//                                sine is still correct because it is timed off
//                                TIM2 rather than off the sample count.
//       master_avg / slave_avg   must agree within a few LSB.
//       slave_min / slave_max    a stuck 0 or 4095 means no slave data.
//       measured_switch_freq_hz  the frequency the loop settled at.
//       overrun_count            should be 0.
//       elapsed_ticks            720000 for 3 periods at 1 kHz.
//       pd14_late_count          edges the loop played out after they were due.
//                                A few is normal; a large fraction of
//                                switch_count means the loop delay is
//                                comparable to the PD14 delay itself.
//       pd14_drop_count          must be 0.
//       pd14_depth_max           how many edges were ever in flight at once;
//                                1 or 2 at a 1 us delay and a few hundred kHz.
//       half_period_ticks        smoothed interval between PB5 edges. Divide
//                                by 240 for microseconds - this is the delay
//                                that would make the two outputs interleaved,
//                                whether or not adaptive mode is on.
//       pd14_delay_live          the delay in force when the burst ended.
//       pd14_reorder_count       edges held back to keep the queue in time
//                                order; only ever nonzero in adaptive mode.
//       decisions                comparisons made.
//       dropped_pairs            backlog thrown away to keep each decision's
//                                window short. A few is fine. A large fraction
//                                of pairs_seen means the loop cannot keep up
//                                with the converters - build at -O2, and check
//                                the instrumentation is switched off.
//       pending_applied          refused edges later honoured. Should roughly
//                                track min_time_blocks - if min_time_blocks
//                                climbs and this does not, edges are being lost
//                                and the output will latch.
//       forced_flip_count        commutations forced by the dwell ceiling. Near
//                                0 in normal running. Climbing steadily means
//                                the ceiling is doing the switching rather than
//                                the comparator, so look upstream.
//       min_time_blocks          edges refused by the 750 ns hold. Zero below
//                                666 kHz switching; climbing means the input is
//                                asking for edges faster than the guard allows
//                                and cycles are being skipped by design.      */
//  while (1)
//  {
//    /* Call ASP_RunBurst() again from here for another burst. */
//  }
//}
//
///* ==========================================================================
// * Error handling
// * ========================================================================== */
//
//void Error_Handler(void)
//{
//  __disable_irq();
//
//  ASP_OUT_A_PORT->BSRR = (uint32_t)ASP_OUT_A_PIN << 16;
//  ASP_OUT_B_PORT->BSRR = (uint32_t)ASP_OUT_B_PIN << 16;
//  ASP_GATE_PORT->BSRR  = (uint32_t)ASP_GATE_PIN << 16;
//
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
