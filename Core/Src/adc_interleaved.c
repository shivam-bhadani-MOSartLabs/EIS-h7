///*
// * adc_interleaved.c
// *
// *  Created on: Sep 8, 2026
// *      Author: shivam
// */
//
//
///**
//  ******************************************************************************
//  * @file    adc_interleaved_probe.c
//  * @brief   ADC1 + ADC2 in DUAL INTERLEAVED mode, 12-bit, PF3-gated one-shot
//  *          capture that MEASURES its own sample rate. NUCLEO-H753ZI.
//  *
//  * Standalone: has its own main(). Exclude adc_rate_probe.c, adc_sine_comp.c,
//  * adc_dac_single.c, adc_sine_pwm.c, adc_dac_echo.c and adc_dac_passthru.c
//  * from the build while using this one.
//  *
//  *   PC0    input,  ADC1_INP10 AND ADC2_INP10 - the same pin feeds both
//  *   PF3    output, HIGH for exactly the capture window - the scope marker
//  *
//  * WHY INTERLEAVE
//  * --------------
//  * One H7 converter is finished at about 3.1 MSPS at 12 bits: 8 nominal cycles
//  * at a 50 MHz kernel clock is 160 ns, and measured on this board a conversion
//  * takes twice its nominal cycle count, so 320 ns. Interleaving two converters
//  * on the same pin doubles that to about 6.25 MSPS without touching the
//  * conversion time of either one. It is the only route past ~3.1 MSPS at 12
//  * bits, and it is what the original 5 MSPS target needs.
//  *
//  * WHERE THE DATA GOES - ONE REGISTER, TWO SAMPLES
//  * -----------------------------------------------
//  * In dual mode the results do NOT come out of ADC1_DR and ADC2_DR separately.
//  * They are packed into the common data register, ADC12_COMMON->CDR:
//  *
//  *     bits 15:0    master result (ADC1) - the EARLIER sample
//  *     bits 31:16   slave result  (ADC2) - TwoSamplingDelay later
//  *
//  * selected by DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS, which is the
//  * setting for any resolution from 32 down to 10 bits. One DMA request is
//  * raised per PAIR, and the DMA moves one 32-bit word, so the buffer address
//  * advances four bytes per two samples.
//  *
//  * Which means the buffer needs no unpacking. The word reaches memory
//  * little-endian as
//  *
//  *     master_lo  master_hi  slave_lo  slave_hi
//  *
//  * so reading the buffer as a plain array of uint16 yields the samples ALREADY
//  * IN TIME ORDER - master, slave, master, slave. adc_capture.py reads exactly
//  * that and needs no change; the only thing that has to be right is the
//  * descriptor's sample_count, which is 2 * pairs, and it is set below.
//  *
//  * Even indices are ADC1, odd indices are ADC2. That is what makes the
//  * characterisation below possible.
//  *
//  * THE TWO THINGS THAT GO WRONG WITH INTERLEAVING
//  * ----------------------------------------------
//  * They are separable, because they show up in different places. Do NOT try
//  * to tune them against one number.
//  *
//  * 1. UNEVEN SPACING. The pair rate is set by how fast one converter runs; the
//  *    delay only decides where in that period the slave sample sits.
//  *    TwoSamplingDelay should be HALF the conversion period, otherwise samples
//  *    arrive in bunched pairs and the record is not uniformly sampled even
//  *    though the average rate is right. A conversion measured at 320 ns wants
//  *    160 ns of delay, which at 50 MHz is 8 kernel cycles - and 8 is exactly
//  *    the 12-bit maximum, so it fits with nothing to spare.
//  *
//  *    Whether that field counts kernel cycles or the halved internal cycles is
//  *    not something the reference manual settles for this part, so sweep it:
//  *    TWO_SAMPLING_DELAY from 1 to 8. A stagger error puts an IMAGE of the
//  *    input mirrored about Fs/2 into the spectrum - at Fs/2 minus the input
//  *    frequency, NOT at Fs/2 - and adc_capture.py reports that bin. It also
//  *    prints a step-symmetry ratio straight off the samples, which carries the
//  *    same information at any input frequency: 1.00 is evenly spaced, 0.34
//  *    means the slave is sampling a third of the way into the pair instead of
//  *    halfway. Drive a mid-band tone while sweeping - the image is invisible
//  *    near DC because a slow input barely moves between the two samples.
//  *
//  * 2. CHANNEL MISMATCH. The two converters have their own offset and gain, so
//  *    the interleaved stream carries a small alternating pattern - and THAT is
//  *    the tone at exactly Fs/2, present whatever the input does.
//  *    adc_capture.py reports the mean of the even samples and the mean of the
//  *    odd samples separately; the difference is the offset mismatch in codes.
//  *    Feed it back through ADC_INTERLEAVE_TRIM below and both the alternation
//  *    and the Fs/2 spur go away.
//  *
//  *    Order matters: fix the stagger first against the image, then null the
//  *    offset with the trim. The Fs/2 spur does not move when the delay
//  *    changes, and the image does not move when the trim changes.
//  *
//  * RUNNING A CONTROL LOOP ON THIS
//  * ------------------------------
//  * The interleave costs a control loop nothing and gains it a factor of two:
//  *
//  *   - ONE 32-bit read of CDR returns BOTH samples. That is a single bus
//  *     access, the same cost as reading one DR, so a decision loop gets two
//  *     input samples for the price of one peripheral read - exactly what the
//  *     ASP_SHARE_DR path in adc_sine_comp.c already assumes, except that there
//  *     the two decisions share one sample and here they get two real ones.
//  *   - For minimum latency take the upper half (slave): it is the newer of the
//  *     two. For the finest crossing detection use both, in order - lower half
//  *     first - which is what a hysteretic comparator wants, because a crossing
//  *     between the two samples is then caught half a conversion earlier.
//  *   - Fresh input every ~160 ns instead of ~320 ns, so worst-case latency
//  *     from a real input crossing to an output edge roughly halves.
//  *
//  * The catch to design around: with a mismatch trim of zero, the effective
//  * comparison threshold alternates by the offset difference between the two
//  * converters, sample to sample. On a hysteretic law that shows up as jitter
//  * on the switching instant of about (mismatch / input slew). Characterise the
//  * mismatch with this program first, put it in ADC_INTERLEAVE_TRIM, and the
//  * loop sees one converter's worth of consistency at twice the rate.
//  *
//  * TWO MODES
//  * ---------
//  *   CAP_MODE_FREERUN     ContinuousConvMode, pairs back to back at whatever
//  *                        the hardware manages. The measured rate IS the
//  *                        interleaved ceiling.
//  *   CAP_MODE_TRIGGERED   TIM8 TRGO paces the MASTER, so one trigger produces
//  *                        one pair and the sample rate is twice the trigger
//  *                        rate. SAMPLE_RATE_HZ below is the SAMPLE rate; the
//  *                        timer is set to half of it.
//  *
//  * READING THE BUFFER
//  * ------------------
//  * Descriptor at 0x30000000 (64 bytes), samples from 0x30000040, 16-bit, in
//  * time order. adc_capture.py resets the target, reads the descriptor and
//  * prints the measured rate; with mode 2 or 3 in the descriptor it also splits
//  * the record by converter and reports the mismatch.
//  *
//  * BUILD
//  * -----
//  * stm32h7xx_hal_conf.h needs HAL_ADC_MODULE_ENABLED. stm32h7xx_hal_adc.c and
//  * stm32h7xx_hal_adc_ex.c must be in the project.
//  *
//  * Resources: ADC1 (master), ADC2 (slave), DMA2_Stream0, TIM2 (timebase),
//  *            TIM8 (trigger, TRIGGERED mode only), PLL2P, PF3, MPU region 0,
//  *            64 + 2*CAP_SAMPLES bytes at the base of SRAM1.
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
///* FREERUN first, to find the interleaved ceiling. */
//#define CAP_MODE                CAP_MODE_FREERUN
//
///* 8, 10, 12, 14 or 16. The packed CDR format used here covers 32 down to 10
//   bits, so 8-bit is not available in this program. */
//#define ADC_RES_BITS            12U
//
///* adc_ker_ck. 50 MHz is the rev V ceiling; use 33333333 on rev Y.
//   Must divide the 400 MHz PLL2 VCO exactly. */
//#define ADC_CLK_HZ              50000000UL
//
//#define ADC_SAMPLETIME          ADC_SAMPLETIME_1CYCLE_5
//#define ADC_SAMPLE_HALFCYCLES   3U      /* keep in step: 1.5 cyc = 3 halves */
//
///* Stagger between master and slave, in ADC clock cycles. 1..8 at 12 bits and
//   below; 14- and 16-bit allow 9. Want half the conversion period - see the
//   sweep procedure in the header. */
//#define TWO_SAMPLING_DELAY      8U
//
///* Offset mismatch trim, SIGNED, in ADC codes:
//       ADC_INTERLEAVE_TRIM = (mean of odd samples) - (mean of even samples)
//   which is exactly what adc_capture.py prints as the mismatch. Positive means
//   ADC2 reads high, so the trim is subtracted from ADC2; negative means ADC1
//   reads high and it is subtracted from ADC1 instead. The H7 offset register on
//   ADC1/ADC2 only subtracts, which is why the sign picks the converter rather
//   than the direction.
//
//   Leave at 0 for the first characterisation run, then put the measured number
//   here and re-run - the Fs/2 spur should collapse. */
//#define ADC_INTERLEAVE_TRIM     0
//
//
///* TRIGGERED mode only: the SAMPLE rate. One trigger yields a pair, so TIM8
//   runs at half this. Half of it must divide 240 MHz exactly:
//   6.0 MSPS -> 3.0 MHz (ARR 79), 5.0 -> 2.5 MHz (ARR 95),
//   4.0 -> 2.0 MHz (ARR 119). */
//#define SAMPLE_RATE_HZ          6000000UL
//
///* Window length. Longer measures the rate more precisely - the error is about
//   one sample in sample_count - and gives the spectrum finer bins, which is
//   what the Fs/2 spur is read from. */
//#define CAPTURE_US              1000U
//
///* 240 MHz timer kernel clock: 480 MHz SYSCLK, HCLK /2, APB /2, timer x2. */
//#define TIMER_CLK_HZ            240000000UL
//
//#define CAP_SAMPLES             ((uint32_t)(((uint64_t)CAPTURE_US * \
//                                             SAMPLE_RATE_HZ) / 1000000ULL))
//#define CAP_PAIRS               (CAP_SAMPLES / 2U)
//
///* ==========================================================================
// * Derived, checked at compile time
// * ========================================================================== */
//
//#if   ADC_RES_BITS == 16U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_16B
//  #define ADC_TSAR_HALFCYCLES   17U
//  #define ADC_DELAY_MAX         9U
//#elif ADC_RES_BITS == 14U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_14B
//  #define ADC_TSAR_HALFCYCLES   15U
//  #define ADC_DELAY_MAX         9U
//#elif ADC_RES_BITS == 12U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_12B
//  #define ADC_TSAR_HALFCYCLES   13U
//  #define ADC_DELAY_MAX         8U
//#elif ADC_RES_BITS == 10U
//  #define ADC_RESOLUTION        ADC_RESOLUTION_10B
//  #define ADC_TSAR_HALFCYCLES   11U
//  #define ADC_DELAY_MAX         8U
//#else
//  #error "ADC_RES_BITS must be 10, 12, 14 or 16 - the packed CDR format used \
//here covers 32 down to 10 bits only"
//#endif
//
//#define ADC_CONV_HALFCYCLES     (ADC_SAMPLE_HALFCYCLES + ADC_TSAR_HALFCYCLES)
//
///* The HAL constants are not a contiguous enum, so the delay is selected here
//   rather than computed. */
//#if   TWO_SAMPLING_DELAY == 1U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_1CYCLE
//#elif TWO_SAMPLING_DELAY == 2U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_2CYCLES
//#elif TWO_SAMPLING_DELAY == 3U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_3CYCLES
//#elif TWO_SAMPLING_DELAY == 4U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_4CYCLES
//#elif TWO_SAMPLING_DELAY == 5U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_5CYCLES
//#elif TWO_SAMPLING_DELAY == 6U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_6CYCLES
//#elif TWO_SAMPLING_DELAY == 7U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_7CYCLES
//#elif TWO_SAMPLING_DELAY == 8U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_8CYCLES
//#elif TWO_SAMPLING_DELAY == 9U
//  #define ADC_TWOSAMPLINGDELAY  ADC_TWOSAMPLINGDELAY_9CYCLES
//#else
//  #error "TWO_SAMPLING_DELAY must be 1..8 (1..9 at 14 or 16 bits)"
//#endif
//
//_Static_assert(TWO_SAMPLING_DELAY <= ADC_DELAY_MAX,
//               "TwoSamplingDelay above the maximum for this resolution - 8 "
//               "cycles at 12 bits and below, 9 only at 14 and 16 bits. A "
//               "value the hardware rejects would silently mis-stagger the "
//               "interleave");
//
//#define PLL2_VCO_HZ             400000000UL
//#define PLL2P_DIV               (PLL2_VCO_HZ / ADC_CLK_HZ)
//
//_Static_assert(PLL2_VCO_HZ % ADC_CLK_HZ == 0U,
//               "ADC_CLK_HZ must divide the 400 MHz PLL2 VCO exactly");
//_Static_assert(ADC_CLK_HZ <= 50000000UL,
//               "adc_ker_ck above the rev V 50 MHz maximum");
//
//_Static_assert((CAP_SAMPLES % 2U) == 0U,
//               "CAP_SAMPLES must be even - the DMA moves whole pairs");
//_Static_assert(TIMER_CLK_HZ % (SAMPLE_RATE_HZ / 2U) == 0U,
//               "half of SAMPLE_RATE_HZ must divide 240 MHz exactly or TIM8 "
//               "cannot produce the pair rate");
//_Static_assert((CAP_SAMPLES * 2U) + 64U <= 32768U,
//               "capture window exceeds the 32 KB non-cacheable MPU region");
//_Static_assert(CAP_SAMPLES >= 32U, "capture window too short to be useful");
//
///* ==========================================================================
// * Buffer layout - identical to adc_rate_probe.c, so the same host script
// * reads it. SRAM1 (D2), because .bss lands in DTCM which no DMA can reach.
// * ========================================================================== */
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
///* mode: 0 single triggered, 1 single freerun,
//         2 interleaved freerun, 3 interleaved triggered.
//   The host uses it to know the record alternates between two converters. */
//#if (CAP_MODE == CAP_MODE_FREERUN)
//  #define CAP_MODE_FIELD        2UL
//#else
//  #define CAP_MODE_FIELD        3UL
//#endif
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
//static volatile cap_hdr_t * const cap_hdr = (volatile cap_hdr_t *)CAP_HDR_ADDR;
//static volatile uint16_t  * const cap_buf = (volatile uint16_t *)CAP_DATA_ADDR;
//
//#define GATE_PORT               GPIOF
//#define GATE_PIN                GPIO_PIN_3
//
//static ADC_HandleTypeDef hadc1;      /* master */
//static ADC_HandleTypeDef hadc2;      /* slave  */
//static DMA_HandleTypeDef hdma_adc;   /* DMA2_Stream0: CDR -> cap_buf, WORDS */
//static TIM_HandleTypeDef htim2;
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//static TIM_HandleTypeDef htim8;
//#endif
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
//  /* PC0 is ADC1_INP10 and ADC2_INP10 both - one pin, two converters, which is
//     what makes interleaving on a single input possible without any external
//     wiring. */
//  g.Pin  = GPIO_PIN_0;
//  g.Mode = GPIO_MODE_ANALOG;
//  g.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &g);
//
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
//  * @brief DMA2_Stream0: the common data register into the buffer, in WORDS.
//  *
//  * WORD at both ends, not halfword. One DMA request per pair carries one
//  * 32-bit CDR value holding two samples, so NDTR counts PAIRS and the memory
//  * pointer advances four bytes per transfer. Getting this wrong - halfword
//  * peripheral size against a 32-bit register - would drop every slave result
//  * and quietly halve the record.
//  *
//  * NORMAL, not circular: the window fills once and stops, which is what makes
//  * NDTR the exact end-of-capture marker for the timing measurement.
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
//  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//  hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
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
//  * @brief One converter's Init block. Master and slave get the same one.
//  *
//  * In interleaved mode the slave's own trigger settings are ignored - the
//  * master's trigger starts the pair - but the HAL still requires the slave to
//  * be initialised and calibrated, and the resolution and sampling time must
//  * match or the two halves of the record would not be comparable.
//  */
//static void Adc_InitOne(ADC_HandleTypeDef *h, ADC_TypeDef *inst, bool master)
//{
//  ADC_ChannelConfTypeDef ch = {0};
//
//  h->Instance                      = inst;
//  h->Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
//  h->Init.Resolution               = ADC_RESOLUTION;
//  h->Init.ScanConvMode             = ADC_SCAN_DISABLE;
//  h->Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
//  h->Init.LowPowerAutoWait         = DISABLE;
//  h->Init.NbrOfConversion          = 1;
//  h->Init.DiscontinuousConvMode    = DISABLE;
//#if (CAP_MODE == CAP_MODE_FREERUN)
//  h->Init.ContinuousConvMode       = ENABLE;
//  h->Init.ExternalTrigConv         = ADC_SOFTWARE_START;
//  h->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
//#else
//  h->Init.ContinuousConvMode       = DISABLE;
//  h->Init.ExternalTrigConv         = ADC_EXTERNALTRIG_T8_TRGO;
//  h->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
//#endif
//  /* Only the master drives the DMA in dual mode; the slave's result reaches
//     memory through CDR, so its own data management stays at DR. */
//  h->Init.ConversionDataManagement = master ? ADC_CONVERSIONDATA_DMA_ONESHOT
//                                            : ADC_CONVERSIONDATA_DR;
//  h->Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
//  h->Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
//  h->Init.OversamplingMode         = DISABLE;
//  if (HAL_ADC_Init(h) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  if (HAL_ADCEx_Calibration_Start(h, ADC_CALIB_OFFSET_LINEARITY,
//                                  ADC_SINGLE_ENDED) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  ch.Channel              = ADC_CHANNEL_10;
//  ch.Rank                 = ADC_REGULAR_RANK_1;
//  ch.SamplingTime         = ADC_SAMPLETIME;
//  ch.SingleDiff           = ADC_SINGLE_ENDED;
//  ch.OffsetNumber         = ADC_OFFSET_NONE;
//  ch.Offset               = 0;
//  ch.OffsetRightShift     = DISABLE;
//  ch.OffsetSignedSaturation = DISABLE;
//
//  /* The mismatch trim. The offset register on ADC1/ADC2 can only SUBTRACT, so
//     the sign of the trim decides which converter it is applied to: a positive
//     trim means ADC2 reads high and is brought down, a negative one means ADC1
//     does. */
//#if (ADC_INTERLEAVE_TRIM > 0)
//  if (!master)
//  {
//    ch.OffsetNumber = ADC_OFFSET_1;
//    ch.Offset       = (uint32_t)(ADC_INTERLEAVE_TRIM);
//  }
//#elif (ADC_INTERLEAVE_TRIM < 0)
//  if (master)
//  {
//    ch.OffsetNumber = ADC_OFFSET_1;
//    ch.Offset       = (uint32_t)(-(ADC_INTERLEAVE_TRIM));
//  }
//#endif
//
//  if (HAL_ADC_ConfigChannel(h, &ch) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief Both converters, then the dual-mode setting that ties them together.
//  *
//  * DualModeData selects the packed CDR format; without it the DMA would be fed
//  * from the master's DR alone and the slave's conversions would never reach
//  * memory. The DUAL field lives in the ADC12 COMMON register, which
//  * HAL_ADC_Init() does not touch - so it is always set explicitly here, in
//  * both this program and the single-ADC ones, because a stale value from a
//  * previous image survives a reset of the individual ADCs.
//  */
//static void Adc_Config(void)
//{
//  ADC_MultiModeTypeDef mm = {0};
//
//  __HAL_RCC_ADC12_CLK_ENABLE();
//
//  Adc_InitOne(&hadc1, ADC1, true);
//  Adc_InitOne(&hadc2, ADC2, false);
//
//  mm.Mode             = ADC_DUALMODE_INTERL;
//  mm.DualModeData     = ADC_DUALMODEDATAFORMAT_32_10_BITS;
//  mm.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mm) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
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
///** TIM8 at HALF the sample rate: one trigger produces one master+slave pair. */
//static void TrigTimer_Config(void)
//{
//  TIM_MasterConfigTypeDef m = {0};
//
//  __HAL_RCC_TIM8_CLK_ENABLE();
//
//  htim8.Instance               = TIM8;
//  htim8.Init.Prescaler         = 0;
//  htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
//  htim8.Init.Period            = (TIMER_CLK_HZ / (SAMPLE_RATE_HZ / 2U)) - 1U;
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
//#endif
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
//  Mpu_Config();
//  SCB_EnableICache();
//  SCB_EnableDCache();
//
//  HAL_Init();
//  SystemClock_Config();
//
//  /* Keep the debug unit reachable: a sleeping H7 gates its own debug clock and
//     a hot-plug connect then fails with "Unable to read device id from ROM
//     table", which reads like a capture failure but is a connection one. */
//  DBGMCU->CR |= DBGMCU_CR_DBG_SLEEPD1 | DBGMCU_CR_DBG_STOPD1
//              | DBGMCU_CR_DBG_STANDBYD1;
//
//  AdcClock_Config();
//  Gpio_Config();
//  Timebase_Config();
//  Dma_Config();
//  Adc_Config();
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//  TrigTimer_Config();
//#endif
//
//  cap_hdr->magic           = CAP_MAGIC;
//  cap_hdr->version         = CAP_VERSION;
//#if (CAP_MODE == CAP_MODE_FREERUN)
//  cap_hdr->sample_rate_hz  = 0U;          /* unknown by definition */
//#else
//  cap_hdr->sample_rate_hz  = SAMPLE_RATE_HZ;
//#endif
//  cap_hdr->sample_count    = CAP_SAMPLES; /* SAMPLES, i.e. 2 * pairs */
//  cap_hdr->pulse_us        = CAPTURE_US;
//  cap_hdr->adc_bits        = ADC_RES_BITS;
//  cap_hdr->vref_mv         = 3300U;
//  cap_hdr->status          = CAP_STATUS_RUNNING;
//  cap_hdr->timer_clk_hz    = TIMER_CLK_HZ;
//  cap_hdr->elapsed_ticks   = 0U;
//  cap_hdr->adc_clk_hz      = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
//  cap_hdr->mode            = CAP_MODE_FIELD;
//  cap_hdr->sample_bytes    = 2U;
//  cap_hdr->rev_id          = HAL_GetREVID();
//  cap_hdr->adc_isr         = 0U;
//  cap_hdr->conv_halfcycles = ADC_CONV_HALFCYCLES;
//
//  /* Mid-scale, so a half-finished capture looks obviously wrong rather than
//     like real data resting at zero. */
//  for (uint32_t i = 0U; i < CAP_SAMPLES; i++)
//  {
//    cap_buf[i] = (uint16_t)(1U << (ADC_RES_BITS - 1U));
//  }
//
//  /* ---- arm, open the window, time it -----------------------------------
//     Length is in CDR words, so CAP_PAIRS, not CAP_SAMPLES. MultiModeStart
//     enables and starts both converters. */
//  GATE_PORT->BSRR = (uint32_t)GATE_PIN;              /* PF3 HIGH */
//
//  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)CAP_DATA_ADDR,
//                                   CAP_PAIRS) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  uint32_t tick0 = TIM2->CNT;
//
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//  if (HAL_TIM_Base_Start(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//#endif
//
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
//  (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
//#if (CAP_MODE == CAP_MODE_TRIGGERED)
//  (void)HAL_TIM_Base_Stop(&htim8);
//#endif
//
//  /* Unsigned, so a counter wrap inside the window is still correct. The host
//     divides sample_count by this, so the rate it reports is the SAMPLE rate -
//     both converters together - which is the number that matters. */
//  cap_hdr->elapsed_ticks = (uint32_t)(tick1 - tick0);
//  cap_hdr->status        = complete ? CAP_STATUS_COMPLETE : CAP_STATUS_TIMEOUT;
//
//  /* A busy loop, deliberately not __WFI(), so the buffer stays readable out
//     of a running target. */
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
