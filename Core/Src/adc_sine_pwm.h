///**
//  ******************************************************************************
//  * @file    adc_sine_pwm.h
//  * @brief   Free-running hysteretic controller: interleaved dual ADC vs a DDS
//  *          sine, driving a bare GPIO. NUCLEO-H753ZI.
//  *
//  * The control law is the whole law, where "input" is the maximum of the
//  * conversions that arrived since the last decision:
//  *
//  *      input > sine + threshold   ->  output LOW
//  *      input < sine - threshold   ->  output HIGH
//  *      inside the band            ->  hold
//  *
//  * There is no filtering. One decision per pass of the loop, on the peak of
//  * that pass's samples - see ASP_LOWER_USES_MIN.
//  *
//  * There is no carrier, no duty cycle, no minimum or maximum on-time and no
//  * fixed switching frequency. The output switches whenever a sample leaves the
//  * band, and the loop finds its own equilibrium. Switching frequency is an
//  * emergent property of the plant, the threshold and the loop delay - measure
//  * it, do not set it (ASP_Ctx.switch_count tells you what it came out as).
//  *
//  * Pin map - unchanged from the PWM version
//  * ----------------------------------------
//  *   PC0    analog input, ADC1_INP10 = ADC2_INP10 (Arduino A1)
//  *   PB5    the hysteretic output, now a plain GPIO
//  *   PD14   the same waveform delayed by a constant 1 us
//  *   PF3    one-shot gate, high for the length of the burst
//  *   PD15   sample clock, toggles once per ADC sample (instrumentation)
//  *   PA4    DAC echo of the converted samples (instrumentation, off)
//  ******************************************************************************
//  */
//
//#ifndef ADC_SINE_PWM_H
//#define ADC_SINE_PWM_H
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#include "stm32h7xx_hal.h"
//#include <stdint.h>
//#include <stdbool.h>
//
///* ==========================================================================
// * Board constants
// * ========================================================================== */
//
//#define ASP_VREF_MV                     3300U
//#define ASP_ADC_FULL_SCALE              4095U
//
///* Kernel clock of TIM2 and TIM8: 480 MHz SYSCLK, HCLK /2, APB /2, timer x2. */
//#define ASP_TIMER_KERNEL_CLK_HZ         240000000UL
//
///* ==========================================================================
// * Operating parameters
// * ========================================================================== */
//
///* --- Sine reference ---------------------------------------------------- */
//#define ASP_SINE_FREQ_HZ_DEFAULT        1000.0f   /* parameter               */
//#define ASP_SINE_CENTER_MV_DEFAULT      1500U     /* 1.5 V centre            */
//#define ASP_SINE_AMPL_MV_DEFAULT        200U      /* peak, so 1.3 V .. 1.7 V */
//
///* --- Hysteresis band ---------------------------------------------------- */
//#define ASP_THRESHOLD_MV_DEFAULT        200U      /* parameter, 200 mV       */
//
///* --- One-shot gate on PF3 -----------------------------------------------
// * The burst lasts this many sine periods, timed off the free-running 32-bit
// * TIM2 rather than counted in software, so it is exact regardless of what the
// * ADC is doing. 3 periods at 1 kHz = 3.000 ms. */
//#define ASP_GATE_SINE_PERIODS_DEFAULT   0.50f      /* parameter               */
//
///* --- Output polarity -----------------------------------------------------
// * 0 : above the band -> LOW, below the band -> HIGH  (as specified)
// * 1 : the opposite, for a plant where more on-time lowers the measured signal.
// * If the loop runs away to one rail and stays there, this is the first thing
// * to flip. */
//#define ASP_OUTPUT_INVERT               0         /* parameter               */
//
///* --- PD14: PB5 delayed by a constant time --------------------------------
// * The output is aperiodic now, so this cannot be a phase shift - it is a real
// * time delay applied to the edge stream. Every transition of PB5 is stamped
// * with the TIM2 tick at which it should be reproduced on PD14, queued, and
// * played out when the counter gets there.
// *
// * That means the delay is referenced to hardware time, not to how many samples
// * the loop happened to process in a burst, so it does not stretch or collapse
// * when the loop runs in chunks. Residual jitter is one pass of the sample loop,
// * roughly 100 ns; ASP_Ctx.pd14_late_count reports how often the queue was
// * serviced later than the edge was due.
// *
// * 1000 ns at a switching frequency around 500 kHz is half a period, which is
// * the interleaved case. There is nothing forcing that relationship any more -
// * the loop picks its own frequency - so read ASP_Ctx.measured_switch_freq_hz
// * and set this to half the period you actually get. */
//#define ASP_PD14_DELAY_NS_DEFAULT       1000U     /* parameter               */
//
///* --- How the PD14 edge is placed -----------------------------------------
// * 1 : hardware. PD14 is TIM4_CH3, and TIM4 channel 3 runs in output-compare
// *     TOGGLE mode. The due time goes into CCR3 and the timer places the edge
// *     itself, so its accuracy is one 240 MHz tick - 4.2 ns - no matter what
// *     the sample loop is doing. Toggle mode is the right fit because PD14's
// *     edges strictly alternate: it is a copy of a two-state signal.
// *
// * 0 : software. The queue is drained against the counter and the pin driven
// *     with a GPIO store, so the edge lands within one pass of the sample loop.
// *     That is 100 to 150 ns at -O2 and 400 to 600 ns at -O0 - which shows up
// *     directly as PB5-to-PD14 jitter. Kept only as a fallback.
// *
// * Both paths take their reference from the instant PB5 itself is written, so
// * the spacing between the two pins is the delay and nothing else. An earlier
// * version scheduled PD14 from the estimated conversion time of the newest
// * sample instead, which sat one sample period before the PB5 edge and made
// * the measured spacing 750 ns rather than 1000 ns. */
//#define ASP_PD14_HARDWARE               1         /* parameter               */
//
///* 0 : PD14 is PB5 delayed.  1 : PD14 is the complement of PB5, delayed.
// *
// * WARNING on complement mode: there is NO dead time anywhere in this design.
// * Driving both halves of a half-bridge from PB5 and PD14 will cause
// * shoot-through unless the gate driver inserts its own. */
//#define ASP_PD14_COMPLEMENT             0         /* parameter               */
//
///* --- Adaptive delay ------------------------------------------------------
// * 0 : the delay stays at ASP_PD14_DELAY_NS_DEFAULT.
// * 1 : the delay tracks half the switching period, which is what interleaving
// *     actually wants. Because the two outputs alternate, the time between
// *     consecutive PB5 edges IS the half period, so an exponential average of
// *     that interval is the target delay directly - no division, no period
// *     measurement.
// *
// * The delay moves towards the target by at most ASP_PD14_SLEW_NS_PER_EDGE per
// * edge, so it walks there over many cycles instead of jumping. At 20 ns/edge
// * and a few hundred kHz it takes well under a millisecond to cross a
// * microsecond, which is gentle compared with the plant.
// *
// * ASP_PD14_EMA_SHIFT sets how heavily the half-period estimate is smoothed:
// * 4 means each new interval moves the estimate by a sixteenth. Raise it if the
// * switching interval is ragged.
// *
// * Two guards keep this safe. The delay is clamped to
// * [ASP_PD14_DELAY_MIN_NS, ASP_PD14_DELAY_MAX_NS], and queued edges are forced
// * to stay in time order - shortening the delay can otherwise make a new edge
// * due before one already waiting, which would reorder the output. */
//#define ASP_PD14_ADAPTIVE               0         /* parameter               */
//#define ASP_PD14_DELAY_MIN_NS           200U      /* parameter               */
//#define ASP_PD14_DELAY_MAX_NS           20000U    /* parameter               */
//#define ASP_PD14_SLEW_NS_PER_EDGE       20U       /* parameter               */
//#define ASP_PD14_EMA_SHIFT              4U        /* parameter               */
//
///* --- The decision statistic: max of the chunk ----------------------------
// * There is no filter. Every pass of the loop takes whatever conversions have
// * arrived since the last pass, reduces them to their MAXIMUM, and makes one
// * comparison against the band. One decision per pass, so at most one edge.
// *
// * Why max and not an average: an average attenuates the peaks of the input,
// * which is exactly what the comparison needs. A 2 us moving average on a
// * 200 kHz triangle clipped its peaks to 60 % of full swing, which caused the
// * output to stop switching near the extremes of the sine. Max keeps the peaks
// * intact and adds no group delay at all - the only lag left in the loop is one
// * conversion plus one poll, roughly 400 ns.
// *
// * It also simplifies the switching, which was the point: the output can change
// * at most once per pass rather than once per sample, so single noisy samples
// * cannot produce a burst of edges.
// *
// * Max is not symmetric - it biases the statistic upward by roughly half a
// * sample step. For the "input above the band" test that is the correct and
// * conservative choice. For the "input below the band" test the strictly
// * symmetric statistic would be the minimum of the chunk; set this to 1 to use
// * it there and get an envelope detector instead. Default 0 uses the maximum
// * for both tests, which is what the control law literally says. */
//#define ASP_LOWER_USES_MIN              1         /* parameter               */
//
///* --- How many pairs may fold into one decision ---------------------------
// * The maximum is only a useful statistic while the samples it covers span a
// * short time compared with a half period of the input. Let a pass pick up two
// * or more pairs at a 500 kHz carrier and the chunk covers a whole ramp - and
// * the maximum of a whole ramp is nearly full scale, which is always above the
// * band, so the output latches and that switching cycle is skipped.
// *
// * So a decision folds at most this many pairs, and any backlog beyond that is
// * DISCARDED, newest kept. A comparator has no state to lose: stale samples are
// * worthless, and throwing them away also keeps the loop delay at its floor
// * rather than letting it grow. ASP_Ctx.dropped_pairs counts them.
// *
// * 1 pair = a 250 ns window, which stays small next to a half period up to
// * about 1 MHz. Raise it only if the input is noisy and you want more samples
// * behind each decision. */
//#define ASP_MAX_PAIRS_PER_DECISION      1U        /* parameter               */
//
///* --- Minimum on and off time ---------------------------------------------
// * Once the pin changes it is held for at least this long before it is allowed
// * to change back. These are TIMES, not a frequency: together they put a
// * ceiling on the switching frequency without setting it, which is
// *
// *      f_max = 1 / (min_on + min_off) = 666.7 kHz at 750 ns each.
// *
// * "On" is the time the pin is HIGH, after ASP_OUTPUT_INVERT has been applied.
// *
// * Note what this becomes in duty terms at a given switching frequency: at
// * 500 kHz a 2 us period with a 750 ns floor confines the duty to 37.5 % ..
// * 62.5 %. The faster the loop switches, the tighter that gets, and above
// * 666.7 kHz it starts refusing edges outright and cycles will be skipped -
// * ASP_Ctx.min_time_blocks counts every edge held back.
// *
// * A blocked edge is not queued or remembered. The comparison is re-made from
// * fresh samples on the very next pass, so the edge happens as soon as the hold
// * time expires, provided the input still wants it. */
//#define ASP_MIN_ON_NS_DEFAULT           750U      /* parameter               */
//#define ASP_MIN_OFF_NS_DEFAULT          750U      /* parameter               */
//
///* --- Maximum on and off time: a hard ceiling on dwell --------------------
// * The output may not sit in one state longer than this. When it has, the pin
// * is flipped whatever the comparator says. Where min_on and min_off put a
// * ceiling on switching frequency, these put a FLOOR on it:
// *
// *      f_min = 1 / (max_on + max_off) = 100 kHz at 5 us each.
// *
// * This exists because a hysteretic loop can otherwise latch. The mechanism is
// * worth understanding: if an edge is ever refused by the minimum-time guard
// * and then forgotten, the input goes back through the band and comes out the
// * other side asking for the state the output is ALREADY in - so nothing
// * happens, and the output stays put indefinitely. One refusal desynchronises
// * the loop for good.
// *
// * Two independent defences are in place. A refused edge is now REMEMBERED and
// * applied the moment its hold expires (ASP_Ctx.pending_applied counts those),
// * which removes the mechanism. And this ceiling forces a commutation
// * regardless (ASP_Ctx.forced_flip_count counts those), which bounds the damage
// * if anything else ever wedges the loop.
// *
// * Choosing the value: it must be comfortably longer than the natural dwell,
// * which for a 0 to 3 V triangle against a band near 1.3 V .. 1.7 V is
// * 0.433 x T_excitation. At 5 us that means excitation above about 87 kHz is
// * untouched. Go below that and the ceiling starts forcing edges of its own -
// * which shows up as forced_flip_count climbing steadily rather than staying
// * near zero.
// *
// * 0 disables the ceiling on that side. */
//#define ASP_MAX_ON_NS_DEFAULT           5000U     /* parameter, 0 = off      */
//#define ASP_MAX_OFF_NS_DEFAULT          5000U     /* parameter, 0 = off      */
//
///* --- Sample rate ---------------------------------------------------------
// * One TIM8 trigger produces one interleaved PAIR of conversions, so the
// * aggregate rate is 2 x the pair rate. 2.0 MHz pairs = 4.0 MSPS.
// *
// * At 12-bit the interleave stagger is limited to 8 ADC clock cycles, and for a
// * uniformly spaced stream it must be exactly half the pair period - so a pair
// * is 16 cycles. At a 32 MHz ADC kernel clock that is a 500 ns pair period with
// * a 46.9 ns aperture, good for roughly 600 ohm of source impedance.
// *
// * For a weaker source, slow the whole structure down: change PLL2P in
// * ASP_AdcClockConfig() and this target together.
// *   32.0 MHz -> 2000000 here, 4.0 MSPS, 46.9 ns aperture, ~600 ohm
// *   24.0 MHz -> 1500000 here, 3.0 MSPS, 62.5 ns aperture, ~820 ohm
// *   19.2 MHz -> 1200000 here, 2.4 MSPS, 78.1 ns aperture, ~1 kohm           */
//#define ASP_PAIR_RATE_HZ_TARGET         2000000UL
//
///* ==========================================================================
// * Instrumentation - all of it costs loop time, so turn it off for real runs
// * ==========================================================================
// *
// * BUILD AT -O2. At -O0 the sample loop runs roughly three to five times
// * slower - every struct member is reloaded and every local spills to the
// * stack - which is enough on its own to stop it keeping up at 4 MSPS.
// */
//
///* --- Sample clock on PD15 ------------------------------------------------
// * Toggles once per ADC sample, so the pin carries a square wave at half the
// * sample rate. At 4.0 MSPS that is exactly 2.000 MHz. Read 1.000 MHz and only
// * one conversion per trigger is happening, i.e. the interleaving is not
// * working. This is the cheapest and least ambiguous rate check there is - one
// * instruction per sample. */
//#define ASP_SAMPLE_CLK_ENABLE           0         /* parameter               */
//
///* --- DAC echo on PA4 (DAC1_OUT1) ----------------------------------------
// * Replays converted samples so the digitised waveform can be seen on a scope.
// *
// * The H7 DAC settles in 1 to 3 us, so it cannot follow 4 MSPS - replaying
// * every sample would measure the DAC, not the ADC. Every ASP_DAC_ECHO_DECIM-th
// * sample is written instead, which keeps the update rate inside the DAC's
// * ability and still makes the test valid: what matters is the *period* of the
// * replayed waveform. Echo every 4th sample of a genuine 4 MSPS stream and a
// * 200 kHz triangle comes back out at 200 kHz. If the real rate were 2 MSPS the
// * same decimation would replay it at 100 kHz.
// *
// * Unbuffered output, so probe it with a scope (1 Mohm) and do not load it. */
//#define ASP_DAC_ECHO_ENABLE             0         /* parameter               */
//#define ASP_DAC_ECHO_DECIM              4U        /* parameter, power of two */
//
///* --- Per-converter statistics -------------------------------------------
// * The master/slave min, max and average in ASP_Ctx. Two adds and four compares
// * per pair. Worth having while bringing the interleaving up, worth removing
// * once it is trusted. */
//#define ASP_STATS_ENABLE                0         /* parameter               */
//
///* ==========================================================================
// * Fixed sizes
// * ========================================================================== */
//
//#define ASP_LUT_SIZE                    1024U
//#define ASP_LUT_SHIFT                   22U
//
///* Circular ADC buffer, in 32-bit words (one word = one interleaved pair).
//   Power of two. 1024 words = 2048 samples = 512 us of history at 4.0 MSPS. */
//#define ASP_BUF_WORDS                   1024U
//
///* ==========================================================================
// * Diagnostics and state
// * ========================================================================== */
//
//typedef struct
//{
//  /* --- live configuration ---------------------------------------------- */
//  int32_t  center_code;      /* sine centre, ADC codes                     */
//  int32_t  ampl_code;        /* sine peak amplitude, ADC codes             */
//  int32_t  thr_code;         /* hysteresis threshold, ADC codes            */
//  float    sine_freq_hz;
//  float    gate_sine_periods;
//  uint32_t min_on_ns;
//  uint32_t min_off_ns;
//  uint32_t min_on_ticks;
//  uint32_t min_off_ticks;
//  uint32_t max_on_ns;
//  uint32_t max_off_ns;
//  uint32_t max_on_ticks;      /* 0 = no ceiling on that side               */
//  uint32_t max_off_ticks;
//
//  /* --- DDS -------------------------------------------------------------- */
//  uint32_t phase_inc;        /* per ADC sample                             */
//  uint32_t inc_tick_q8;      /* per TIM2 tick, Q8 - the time anchor        */
//  uint32_t gate_ticks;       /* burst length in TIM2 ticks                 */
//  uint32_t ticks_per_sample; /* TIM2 ticks between two ADC samples         */
//
//  /* --- PD14 delay line -------------------------------------------------- */
//  uint32_t pd14_delay_ns;    /* requested fixed delay                      */
//  uint32_t pd14_delay_ticks; /* the same in TIM2 ticks, the starting point */
//  uint32_t pd14_late_count;  /* edges played out after they were due       */
//  uint32_t pd14_drop_count;  /* queue was full - should always be 0        */
//  uint32_t pd14_depth_max;   /* deepest the queue got                      */
//
//  /* --- adaptive delay --------------------------------------------------- */
//  bool     pd14_adaptive;
//  uint32_t pd14_delay_min_ticks;
//  uint32_t pd14_delay_max_ticks;
//  uint32_t pd14_slew_ticks;      /* max change per edge                    */
//  uint32_t pd14_delay_live;      /* the delay actually in use right now    */
//  uint32_t half_period_ticks;    /* smoothed interval between PB5 edges    */
//  uint32_t pd14_reorder_count;   /* edges held back to keep time order     */
//  uint32_t pd14_parity_risk;     /* queue overflow in hardware mode - would
//                                    invert PD14. Must stay 0.              */
//
//  /* --- derived rates ---------------------------------------------------- */
//  uint32_t pair_rate_hz;     /* TIM8 trigger rate, as configured           */
//  uint32_t sample_rate_hz;   /* 2 x pair rate, as configured               */
//
//  /* --- results of the last burst ---------------------------------------- */
//  uint32_t elapsed_ticks;    /* TIM2 ticks the burst actually took         */
//  uint32_t pairs_seen;       /* pairs the DMA delivered - use this for the
//                                real sample rate                           */
//  uint32_t pairs_total;      /* pairs actually folded into a decision      */
//  uint32_t samples_total;    /* 2 x pairs_total                            */
//  uint32_t dropped_pairs;    /* backlog discarded to keep the chunk short  */
//  uint32_t switch_count;     /* output transitions                         */
//  uint32_t min_time_blocks;  /* edges refused by the min on/off guard      */
//  uint32_t pending_applied;  /* refused edges later honoured - the anti-latch
//                                path. Should roughly track min_time_blocks. */
//  uint32_t forced_flip_count;/* commutations forced by the dwell ceiling.
//                                Near 0 in normal running; climbing steadily
//                                means the ceiling is doing the switching, not
//                                the comparator.                            */
//  uint32_t starve_count;     /* polls that found no new data               */
//  uint32_t overrun_count;    /* loop fell behind the DMA by half a buffer  */
//
//  /* Measured, not assumed. This is the number that says whether the
//     interleaving is really working. */
//  float    measured_sample_rate_hz;
//  float    measured_switch_freq_hz;
//
//  /* Per-converter statistics, for the same reason: on a slowly varying input
//     the two averages must agree to within a few LSB. */
//  int32_t  master_avg;
//  int32_t  slave_avg;
//  int32_t  master_min;
//  int32_t  master_max;
//  int32_t  slave_min;
//  int32_t  slave_max;
//
//  /* --- last decision, for a quick look in the debugger ------------------ */
//  int32_t  last_input;       /* the chunk maximum that was compared        */
//  uint32_t decisions;        /* comparisons made = passes with new data    */
//
//  /* --- is the loop fast enough? ------------------------------------------
//   * The input is outside the band for only part of each excitation cycle, and
//   * the loop has to make at least one comparison inside that window or the
//   * crossing is never seen and a cycle is skipped. For a 0 to 3 V triangle
//   * against a band near 1.3 V .. 1.7 V the window is 0.433 x T_excitation:
//   *
//   *     400 kHz -> 1083 ns      500 kHz -> 867 ns      600 kHz -> 722 ns
//   *
//   * decision_gap_max_ticks is the longest the loop ever went between two
//   * comparisons, in 240 MHz ticks - divide by 240 for microseconds. It MUST
//   * come out below that window, with margin. If it does not, the loop is the
//   * reason cycles are being skipped: build at -O2 and switch the
//   * instrumentation off. */
//  uint32_t decision_gap_max_ticks;
//  uint32_t straddle_count;   /* passes where the sample pair spanned the
//                                whole band - see the tie-break in the loop  */
//  int32_t  last_ref;         /* sine reference, ADC codes                  */
//  bool     last_output;
//} ASP_Ctx_t;
//
//extern ASP_Ctx_t ASP_Ctx;
//
///* ==========================================================================
// * API
// * ========================================================================== */
//
//void ASP_MPU_Config(void);
//void ASP_Init(void);
//
///**
//  * @brief  Run one gated burst, start to finish. Blocking.
//  * @note   Drives PF3 high, runs the hysteretic loop for
//  *         ASP_GATE_SINE_PERIODS sine periods, then drives both outputs and
//  *         PF3 low and fills in the measurement fields of ASP_Ctx.
//  */
//void ASP_RunBurst(void);
//
//void ASP_SetSineFreqHz(float hz);
//void ASP_SetSineCenterMv(uint32_t mv);
//void ASP_SetSineAmplitudeMv(uint32_t mv);
//void ASP_SetThresholdMv(uint32_t mv);
//void ASP_SetGateSinePeriods(float periods);
//
///** @brief Set the minimum on and off times in nanoseconds. Together these cap
//  *        the switching frequency at 1/(on+off). */
//void ASP_SetMinTimesNs(uint32_t on_ns, uint32_t off_ns);
//
///**
//  * @brief  Set the maximum on and off times in nanoseconds - the hard dwell
//  *         ceiling. Together these put a floor on switching frequency at
//  *         1/(on+off). 0 on either side disables the ceiling there.
//  * @note   A value at or below the corresponding minimum is rejected and
//  *         raised to twice it, since a ceiling under the floor would force an
//  *         edge the minimum-time guard then has to refuse.
//  */
//void ASP_SetMaxTimesNs(uint32_t on_ns, uint32_t off_ns);
//
///** @brief Set the constant PD14 delay, in nanoseconds. Quantised to TIM2
//  *        ticks (4.17 ns at 240 MHz), so effectively continuous. */
//void ASP_SetPd14DelayNs(uint32_t ns);
//
///**
//  * @brief  Let the PD14 delay track half the switching period.
//  * @param  enable          false pins it at the fixed delay.
//  * @param  slew_ns_per_edge how fast it is allowed to walk there.
//  */
//void ASP_SetPd14Adaptive(bool enable, uint32_t slew_ns_per_edge);
//
//void Error_Handler(void);
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif /* ADC_SINE_PWM_H */
