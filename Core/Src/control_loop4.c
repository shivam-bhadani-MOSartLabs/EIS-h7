///**
//  ******************************************************************************
//  * @file    adc_sine_pwm_eoc.c
//  * @brief   Single ADC1, 12-bit, continuous conversion at its maximum rate.
//  *          One decision per END-OF-CONVERSION INTERRUPT against a
//  *          timer-derived DDS sine, with minimum and maximum on/off times, a
//  *          constant-delay copy on PD14, and PD15/PF3 enables.
//  *          NUCLEO-H753ZI.
//  *
//  * Standalone: has its own main(). Exclude adc_rate_probe.c, adc_sine_comp.c,
//  * adc_interleaved_probe.c, adc_dac_single.c, adc_sine_pwm.c, adc_dac_echo.c
//  * and adc_dac_passthru.c from the build.
//  *
//  *   PC0    input,  ADC1_INP10 (Arduino A1), 0..3 V, also traced to DTCM
//  *   PB5    output, the hysteretic decision
//  *   PD14   output, the same waveform delayed PD14_DELAY_NS, driven by
//  *          TIM4_CH3 in hardware compare - AF2, not GPIO
//  *   PD15   output, startup-circuit enable. First up, last down.
//  *   PF3    output, gate-driver enable. Rises PF3_AFTER_PD15_US after PD15,
//  *          falls GATE_SINE_PERIODS sine periods later, and PD15 follows it
//  *          down PD15_AFTER_PF3_US after that. Also falls, and LATCHES down
//  *          until reset, if the input leaves SAFE_LO_MV..SAFE_HI_MV.
//  *
//  * REMOVE PB5, PD14, PD15 and PF3 from the CubeMX GPIO init - this file owns
//  * all four.
//  *
//  * THE LAW
//  * -------
//  *     ADC in  >  sine + threshold   ->  PB5 OFF (LOW)
//  *     ADC in  <  sine - threshold   ->  PB5 ON  (HIGH)
//  *     in between                    ->  PB5 holds
//  *
//  * With the dwell windows folded in, in full: above the upper limit PB5 goes
//  * off once min Ton has elapsed; while off, PB5 comes back on when the input
//  * drops below the lower limit AND min Toff has elapsed, or unconditionally
//  * once max Toff is crossed.
//  *
//  * ASP_INVERT swaps the two comparisons - one XOR in the decision, not a
//  * rewrite.
//  *
//  * On top of the law sit four dwell constraints, all parametric in
//  * nanoseconds, all measured from the last PB5 edge:
//  *
//  *     MIN_TON_NS    once HIGH, cannot go LOW  before this has elapsed
//  *     MAX_TON_NS    once HIGH, is FORCED LOW  when this has elapsed
//  *     MIN_TOFF_NS   once LOW,  cannot go HIGH before this has elapsed
//  *     MAX_TOFF_NS   once LOW,  is FORCED HIGH when this has elapsed
//  *
//  * So the loop cannot turn off inside the minimum on window and cannot turn on
//  * inside the minimum off window, exactly as specified. A decision the law
//  * wants but a window forbids is NOT queued - it is simply refused, and the
//  * next conversion re-evaluates it. That is the right structure here because
//  * conversions arrive every ~320 ns while the windows are microseconds: the
//  * refused edge is re-proposed several times before the window even expires,
//  * so it lands within one conversion period of the window opening. An earlier
//  * design queued refused edges and could apply a stale one; this cannot.
//  *
//  * MAX_TOFF_NS forces the output ON, not off - it is a ceiling on how long the
//  * output may stay off, the mirror of MAX_TON_NS. Set it to 0 to disable that
//  * ceiling (MAX_TON_NS = 0 likewise).
//  *
//  * EVERY TIMING DECISION HAS ONE-CONVERSION GRANULARITY
//  * ----------------------------------------------------
//  * All four dwell limits are checked in the EOC interrupt, so an edge lands
//  * within one conversion period - about 320 ns - of the instant it becomes
//  * legal. That is inherent to an EOC-driven design and not a bug: the limits
//  * are microseconds, so the error is under 10%, and it is always LATE, never
//  * early, so a minimum time is never violated. If a forced flip ever needs
//  * sub-100 ns placement it has to come off a timer compare instead, the way
//  * PD14 does below.
//  *
//  * PB8 - THE EXTERNAL START
//  * ------------------------
//  * A rising edge on PB8 starts the run. Everything downstream is unchanged:
//  * the edge is simply what PD15's rise now waits for, and PD15 -> PF3 -> the
//  * gate -> PD15 low then proceed exactly as they did when main raised PD15 the
//  * moment the converter was going.
//  *
//  * The point is that the start instant becomes knowable to something other
//  * than this firmware. Whatever generates the edge can trigger a scope from
//  * the same signal and know exactly where the excitation begins, instead of
//  * arming a capture and hoping it overlaps.
//  *
//  * The pin is configured with a PULL-DOWN and rising-edge detection, so a
//  * disconnected input rests low and cannot start the power stage. REMOVE PB8
//  * FROM THE CubeMX GPIO INIT - this file owns it, as it does PB5, PD14, PD15
//  * and PF3.
//  *
//  * THE START CONDITION IS A HIGH LEVEL, NOT ONLY AN EDGE. The run begins when
//  * PB8 is high - whether it rose a moment ago or has been high since before
//  * the board came out of reset. That distinction is the whole usability of the
//  * thing: a bench supply left sitting at 3.3 V produces exactly one edge, when
//  * it is first connected, and none at all on any reset after that. Waiting for
//  * an edge would mean the board started once and then never again, which is
//  * precisely what happened the first time this was tried.
//  *
//  * The EXTI interrupt is still there, and is what catches a brief pulse that
//  * would be gone before the polling loop next looked. So a pulse works, a
//  * level works, and a level applied before reset works.
//  *
//  * EXT_TRIG_ENABLE 0 restores the old behaviour exactly. EXT_TRIG_TIMEOUT_MS
//  * bounds the wait, because a board that blocks forever on a start that never
//  * arrives is indistinguishable from one that has crashed; on timeout the run
//  * is abandoned and the status block says which happened.
//  *
//  * THE FREQUENCY IS IN MILLIHERTZ, AND THE TIMEBASE IS 64 BITS
//  * ----------------------------------------------------------
//  * SINE_MHZ sets the frequency in whole MILLIHERTZ, so 1234.567 Hz and
//  * 0.005 Hz are both expressible. Two things had to change to make the low end
//  * real rather than nominal.
//  *
//  * TIM2 is 32 bits at 240 MHz, so it WRAPS EVERY 17.9 SECONDS. One period at
//  * 1 mHz is a thousand seconds - fifty-five wraps - so a phase computed from a
//  * bare 32-bit tick difference is meaningless below about 0.06 Hz. The handler
//  * now carries a wrap counter: it reads the timer every few hundred
//  * nanoseconds so it cannot miss an overflow, and one compare per interrupt
//  * extends the timebase to 64 bits.
//  *
//  * The phase step also had to change scale. It used to be a Q8 fraction of a
//  * phase unit per tick; at 1 mHz the true value is 0.0179, which Q8 rounds to
//  * 4/256 - a 13% frequency error. The step is now (mul >> shift) with BOTH
//  * chosen from the frequency, so the mantissa always lands in the top bits of
//  * a 32-bit word whatever the frequency. Checked against exact arithmetic from
//  * 1 mHz to 900 kHz: worst frequency error 4 parts in 10^9, worst 64-bit
//  * product 8.3e18 against a limit of 1.8e19.
//  *
//  * The phase reference is RE-ANCHORED whenever the tick difference reaches
//  * 2^32, which is what keeps that product in range. The branch is taken once
//  * every 17.9 seconds.
//  *
//  * The gate deadline is 64 bits for the same reason: five periods at 1 mHz is
//  * five thousand seconds, and the old 32-bit deadline covered eighteen.
//  *
//  * THE SINE IS DERIVED FROM THE TIMER, NOT FROM A PER-SAMPLE INCREMENT
//  * -------------------------------------------------------------------
//  * The phase is computed from TIM2 - free-running, 32-bit, 240 MHz off the
//  * crystal - on every interrupt:
//  *
//  *     phase = (now - phase_ref) * PHASE_PER_TICK_Q8 >> 8
//  *
//  * one 32x32 multiply, and TIM2 has to be read anyway for the dwell logic, so
//  * it is nearly free. This matters more than it looks: a phase accumulator
//  * advanced once per conversion would need the conversion rate to be known in
//  * advance, and it is not: the converter free-runs at whatever it manages, and
//  * an accumulator tuned for the wrong guess would produce a sine at the wrong
//  * frequency with nothing in the code looking wrong. Deriving the phase from
//  * the timer makes the sine frequency exact regardless of the rate.
//  *
//  * THE CONVERSION RATE IS AN OUTPUT, NOT A SETTING
//  * -----------------------------------------------
//  * ONE ADC, independent mode, 12 bits, continuous conversion. There is no
//  * target rate anywhere in this file and nothing is compared against one. The
//  * converter is started once and never stopped, it runs as fast as it runs,
//  * and every conversion raises an interrupt that makes one decision. The rate
//  * that comes out is measured over the gate and published, and that is the
//  * answer - whatever it is.
//  *
//  * PD14 - A CONSTANT DELAY, IN HARDWARE
//  * ------------------------------------
//  * PD14 is TIM4_CH3 in toggle-on-compare mode, not a GPIO. When PB5 flips, the
//  * interrupt pushes a compare value of (TIM4 now + PD14_DELAY_NS) and the
//  * hardware toggles PD14 at exactly that count. The delay is therefore exact
//  * to the timer, with none of the interrupt jitter that a software-timed copy
//  * would carry - which was the whole problem with the earlier attempt at this.
//  *
//  * One compare register can only hold one pending edge, and PB5 can flip again
//  * before the previous copy has come out: with MIN_TON and MIN_TOFF at 750 ns
//  * and a 1.6 us delay, up to three edges can be in flight. So pending edges sit
//  * in a small queue, and the queue is advanced from the SAME interrupt that
//  * fills it - the EOC handler checks TIM4's CC3 flag and re-arms the compare
//  * when it finds it set. One writer, no second interrupt, no race. It works
//  * because the EOC period (~320 ns) is shorter than the minimum spacing
//  * between two PB5 edges (750 ns), so a fired compare is always re-armed long
//  * before the next one is due.
//  *
//  * If a due time has already passed when it reaches the compare register - only
//  * possible under a pathological overload - it is clamped a few ticks into the
//  * future and counted in pd14_late, rather than being missed and coming out
//  * 273 us later when the 16-bit counter wraps.
//  *
//  * THE ENABLES - TWO SEQUENCED SIGNALS, NEITHER OF THEM THE PWM
//  * ------------------------------------------------------------
//  * PD15 enables the startup circuit. PF3 enables the GATE DRIVER. Neither is
//  * an output enable for PB5, and neither is derived from the decision logic -
//  * both are placed purely by time:
//  *
//  *     PD15 high
//  *       -> +PF3_AFTER_PD15_US               PF3 high, and the sine starts
//  *            -> +GATE_SINE_PERIODS periods  PF3 low
//  *                 -> +PD15_AFTER_PF3_US     PD15 low
//  *
//  * The DDS phase reference is the PF3 rising edge, so the PF3 window holds
//  * that many WHOLE sine cycles starting from phase zero.
//  *
//  * ALL FOUR EDGES ARE PLACED BY THE INTERRUPT, NOT BY MAIN. That is not an
//  * optimisation, it is the only way they can be trusted. An earlier version
//  * spun on TIM2 in main:
//  *
//  *     while ((TIM2->CNT - t_gate) < gate_ticks) { }
//  *     EN2_PORT->BSRR = EN2_PIN << 16;
//  *
//  * and that loop can only notice its deadline when main gets the CPU. With an
//  * interrupt per conversion, main's share is 1 - (handler / conversion
//  * period), which collapses towards zero as the handler approaches the
//  * period - and the handler's length is DATA DEPENDENT, because every PB5
//  * edge adds the TIM4 accesses that queue the PD14 copy. So the gate came out
//  * both far too long and different on every run: a requested 50 us window
//  * measured about 1.05 ms and moved from run to run with the input.
//  *
//  * The handler, by contrast, always runs. It carries one deadline and one
//  * compare - about four cycles when nothing is due - and advances a small
//  * state machine when the deadline passes. Every edge therefore lands within
//  * ONE CONVERSION of its deadline, roughly 320 ns, whatever the CPU load, and
//  * the jitter is bounded and known instead of unbounded and mysterious.
//  *
//  * main's only remaining job is to raise PD15, hand the sequence over, and
//  * wait for it to finish.
//  *
//  * PWM_ACTIVE_IN_GATE decides only whether the modulator runs while the gate
//  * driver is disabled. At 1 - the default - PB5 is idle low until PF3 rises
//  * and returns low when PF3 falls, which keeps the record clean; that drop is
//  * immediate and does NOT wait for MIN_TON, the one place a minimum on time is
//  * deliberately not honoured. At 0, PB5 modulates for the whole PD15 window
//  * and the gate driver simply ignores it until PF3 rises.
//  *
//  * THE SAFETY TRIP
//  * ---------------
//  * If the input leaves SAFE_LO_MV..SAFE_HI_MV, PF3 goes low inside the same
//  * interrupt that saw the sample, PB5 drops with it, and both stay down until
//  * the target is RESET. There is no path back: the latch is checked before
//  * anything else in the handler and main never raises PF3 again.
//  *
//  * The trip is armed by SAFE_ARM_IN_GATE:
//  *
//  *   1 (default)  armed when PF3 rises, so the input is only policed while
//  *                the gate driver is enabled.
//  *   0            armed when PD15 rises, so the whole run is policed.
//  *
//  * That default is deliberate. An UNDRIVEN PC0 sits wherever leakage and
//  * sampling charge injection leave it - measured on this board, about 0.2 V -
//  * which is below SAFE_LO_MV. Armed from PD15 with nothing connected, the trip
//  * would fire before PF3 ever rose and the board would look dead. Armed at
//  * PF3, the pin has a live circuit on it by the time anyone is watching.
//  *
//  * SAFE_BLANK_US ignores the first stretch after arming, for a startup
//  * transient that legitimately swings outside the window.
//  * SAFE_TRIP_COUNT requires that many CONSECUTIVE out-of-range conversions,
//  * so a single noise sample cannot end the run; at 1 it trips on the first.
//  *
//  * A static assert rejects a safety window that does not contain the
//  * regulation band itself - if offset +/- amplitude +/- threshold pokes
//  * outside SAFE_LO_MV..SAFE_HI_MV, normal operation would trip and the run
//  * could never complete.
//  *
//  * THE TRACE
//  * ---------
//  * Every conversion inside the gate is recorded as one 32-bit word:
//  *
//  *     bits 11:0    the ADC code
//  *     bit  15      the PB5 level AFTER this conversion's decision
//  *     bits 27:16   the upper comparison limit in force, i.e. sine + threshold
//  *
//  * so one buffer carries the input, the output and the reference together,
//  * sample-aligned, with no way for them to drift apart. The sine is recovered
//  * on the host as (upper limit - threshold), which is exact rather than
//  * reconstructed from a phase guess.
//  *
//  * The buffer is a plain static array, so the linker puts it in DTCM - zero
//  * wait states, and the store costs the handler a couple of cycles. It is NOT
//  * in SRAM1 with the status block, because that region is mapped
//  * non-bufferable for DMA coherence and a non-bufferable store stalls the core
//  * until it retires. Nothing DMAs the trace, so DTCM is free.
//  *
//  * Its address is published in the status block rather than fixed, so there is
//  * no linker script to edit and nothing to keep in sync: pwm_capture.py reads
//  * the address out of the target and then reads the buffer.
//  *
//  * Recording starts when PF3 rises and stops when the buffer is full, when the
//  * gate closes, or the instant the safety trip fires - so after a trip the
//  * buffer holds the run right up to the offending sample, which is exactly
//  * what a trip needs to be diagnosed. TRACE_DECIMATE records every Nth
//  * conversion instead, to cover a long gate at coarser resolution.
//  *
//  * WHAT "MEASURED" MEANS HERE, EXACTLY
//  * -----------------------------------
//  * The conversion rate is interrupts counted INSIDE THE GATE divided by the
//  * MEASURED length of the gate. Both halves matter:
//  *
//  *   - the handler also runs before the gate opens and after it closes, so a
//  *     lifetime interrupt count divided by the gate length overstates the rate
//  *     by however many times longer the run is than the gate. With a 100 us
//  *     gate inside a 2 ms run that is a factor of twenty, and it produced a
//  *     reported 148 MSPS from a converter doing about three.
//  *   - the gate length is taken from TIM2 at the two edges, not from the
//  *     configured value, so the spin loop's overshoot is included rather than
//  *     assumed away.
//  *
//  * The worst-case handler duration is likewise only sampled inside the gate.
//  * The very first interrupt of a run executes with a cold instruction cache
//  * out of flash and takes several times longer than the steady state; before
//  * the gate opens there are thousands of interrupts to warm it, so an in-gate
//  * maximum is the number that describes the hot path.
//  *
//  * ADC OVR is cleared when the gate opens for the same reason. It is a sticky
//  * flag, so one cold-start overrun would otherwise mark the whole run as
//  * having fallen behind.
//  *
//  * WHAT IS MEASURED AND WHERE TO READ IT
//  * -------------------------------------
//  * A 256-byte status block at 0x30000000, readable with ctrl_status.py while
//  * the target runs. It carries the measured conversion rate, the WORST-CASE
//  * interrupt duration in nanoseconds, whether the ADC ever overran (the one
//  * unambiguous sign the handler could not keep up), the PB5 edge count, how
//  * many edges were forced by the maximum times, how many were refused by the
//  * minimum times, and the PD14 queue's high-water mark.
//  *
//  * The worst-case interrupt duration against the conversion period is the
//  * number that says whether this architecture fits at all. If it approaches
//  * the period, the polling loop in adc_sine_comp.c is the alternative - no
//  * interrupt entry cost at all.
//  *
//  * It is measured with DWT->CYCCNT, the core's own cycle counter, NOT with a
//  * timer. That matters: TIM2 sits on APB1 at 120 MHz in another power domain,
//  * so reading it stalls the core for tens of cycles, and a diagnostic that
//  * costs a quarter of the handler's bus traffic is measuring itself. CYCCNT
//  * is on the core's private bus - two cycles to read, 2.08 ns resolution at
//  * 480 MHz - so the measurement is now free enough to leave switched on.
//  *
//  * If a debugger has taken DWT away, isr_max_cycles comes back 0 and the host
//  * scripts say the counter was not running rather than reporting a zero
//  * duration as fact.
//  *
//  * ---------------------------------------------------------------------
//  * THIS FILE IS control_loop2.c WITH THE HOT-PATH CONTROLS MADE OPTIONAL.
//  *
//  * What changed, and nothing else did:
//  *   - MIN_TON / MIN_TOFF are behind DWELL_MIN_ENABLE and are OFF by default.
//  *     The modulator is then limited by the conversion rate and by
//  *     THRESHOLD_MV, which becomes the switching-frequency control.
//  *   - MAX_TON / MAX_TOFF are behind DWELL_MAX_ENABLE and stay ON. MAX_TON is
//  *     the only thing that ends a stuck-ON when the sense path fails low.
//  *   - The PD14 delayed copy, the DWT profiling and the edge statistics are
//  *     each behind their own switch, because each costs work on every
//  *     conversion. PD14 costs an APB read whether or not an edge is pending.
//  *   - The safety window is 500 mV .. 2500 mV and, crucially, does not go
//  *     live until it has seen SAFE_HEALTHY_COUNT consecutive conversions
//  *     inside itself. A cold sense amplifier reads near zero; arming against
//  *     that latched the run shut before it delivered an edge. If the window
//  *     never settles within SAFE_HEALTHY_TIMEOUT_US the run is stopped and
//  *     said so - status 8, trip_code 0xFFFFFFFF - rather than continuing
//  *     unprotected.
//  *   - relax_flags in the status block records which of these were compiled
//  *     in, so a capture can be matched to the build that produced it.
//  *
//  * BUILD
//  * -----
//  * stm32h7xx_hal_conf.h needs HAL_ADC_MODULE_ENABLED and HAL_TIM_MODULE_ENABLED.
//  * stm32h7xx_hal_adc.c, stm32h7xx_hal_adc_ex.c, stm32h7xx_hal_tim.c and
//  * stm32h7xx_hal_tim_ex.c must be in the project. No math library - the sine is
//  * a polynomial, evaluated only at startup.
//  *
//  * This file defines ADC_IRQHandler. If stm32h7xx_it.c also defines it, the
//  * build will fail with a duplicate symbol - delete it there.
//  *
//  * Resources: ADC1, TIM2 (timebase and phase reference), TIM4 CH3 (PD14),
//  *            PLL2P, PB5, PD14, PD15, PF3, MPU region 0, 256 bytes at the base
//  *            of SRAM1, 4 * LUT_N bytes of DTCM.
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
///* The sine, in millivolts about the ADC's own scale. The sine spans
//   SINE_OFFSET_MV +/- SINE_AMP_MV; the comparison limits are that plus and
//   minus THRESHOLD_MV. */
///* The sine frequency, in MILLIHERTZ. 1000 is 1 Hz, 1234567 is 1234.567 Hz,
//   5 is 5 mHz. The range the arithmetic below is verified over is 1 mHz to
//   900 kHz.
//
//   A long gate at a low frequency is a long RUN: five periods at 10 mHz is
//   five hundred seconds of continuous excitation. Check GATE_SINE_PERIODS
//   against the frequency before starting something the hardware has to sit
//   through. */
//#define SINE_MHZ                200UL
///* MEASURED, not assumed: the sense amplifier's own output with no current
//   flowing. Taken on 2026-09-18 from the pre-trigger part of six reset
//   captures on CH1 - the same node this ADC reads - which agreed to 0.18 mV
//   between resets. 1500 was a mid-rail guess and sat 24 mV low, which is a
//   standing DC error in the reference the loop could never correct.
//
//   The idle level is the right number here rather than the 1527.6 mV mean
//   measured DURING excitation: that one contains the excitation itself, so
//   centring the reference on it would fold the loop's existing DC error back
//   into the reference. Re-measure with reset_capture.ipynb if the sense
//   amplifier or its supply is ever changed. */
//#define SINE_OFFSET_MV          1476
//#define SINE_AMP_MV             130
//#define THRESHOLD_MV            20
//
///* ==========================================================================
// * RELAXABLE CONTROLS
// *
// * Everything in this block is compiled in or out. An option set to 0 is not a
// * branch that predicts well - it is absent, and the instructions it would have
// * needed are not fetched. That matters because the handler runs once per
// * conversion with nothing but leftovers going to main: measured, the 64-bit
// * build takes 1.74 MSPS in-gate and leaves main no slice at all, so the run
// * never reaches its own publish. Each option below buys some of that back.
// *
// * Relaxing a control moves the duty of stopping a runaway onto whatever is
// * left. Read the note under each one before turning it off.
// * ========================================================================== */
//
///* ---- dwell floors: MIN_TON / MIN_TOFF ---------------------------------
//   0 removes them. The modulator may then toggle on consecutive conversions,
//   so PB5's switching frequency is bounded by nothing but the conversion rate
//   and the hysteresis band: roughly 1/(2 * conversion period) in the worst
//   case, which at 1.74 MSPS is order 800 kHz. What keeps it far below that in
//   practice is THRESHOLD_MV - the current has to cross the whole band before
//   the law changes its mind - so THRESHOLD_MV becomes the switching-frequency
//   control once this is 0. Check the gate driver and its dead time carry it
//   before relaxing this. */
//#define DWELL_MIN_ENABLE        0
//#define MIN_TON_NS              300UL
//#define MIN_TOFF_NS             300UL
//
///* ---- dwell ceilings: MAX_TON / MAX_TOFF -------------------------------
//   These two are NOT symmetric, on purpose. MAX_TON forces OFF whatever the
//   law wants, because that direction only removes energy. MAX_TOFF forces ON
//   only when the law is holding - never when it has asked for OFF.
//   MAX_TON is a runaway guard, not a shaping control. If the sense path dies -
//   amplifier unpowered, connector out, ADC stuck - the law sees a value that
//   never crosses the upper limit and PB5 stays ON indefinitely. MAX_TON is
//   what ends that; the safety window only helps if the reading is wrong in the
//   right direction. Leave this at 1 unless the window has been proven to cover
//   every failure that can hold the reading low.
//   Either NS value may still be set to 0 individually to drop just that one. */
//#define DWELL_MAX_ENABLE        1
//#define MAX_TON_NS              4000UL
//#define MAX_TOFF_NS             4000UL
//
///* ---- the delayed copy on PD14 -----------------------------------------
//   0 removes TIM4 and the pending-edge queue. This is the single biggest
//   saving on the hot path, because servicing it costs an APB read of TIM4->SR
//   on EVERY conversion whether or not an edge is pending, and APB is slow
//   relative to everything else the handler touches. Turn it off whenever the
//   delayed copy is not being used. */
//#define PD14_ENABLE             1
//
///* ---- worst-case handler timing ----------------------------------------
//   0 removes the two DWT reads and the compare. Cheap, but it is the only
//   thing that reports isr_max_cycles, which is how a too-long handler is
//   noticed at all. Keep it on while tuning the options above; turn it off
//   once the configuration is settled. */
//#define ISR_PROFILE_ENABLE      0
//
///* ---- edge and dwell statistics ----------------------------------------
//   0 removes edges, on_ticks, forced_off/on and blocked_off/on. They are all
//   read-modify-writes to DTCM on the hot path. Losing them means losing the
//   evidence of what the modulator did, so turn this off last. */
//#define STATS_ENABLE            0
//
///* ==========================================================================
// * Safety window
// *
// * With the dwell floors gone this is the control that matters, so it is worth
// * being exact about when it is live.
// *
// * The hazard being avoided: PC0 sits near zero until the sense amplifier is
// * powered and settled at its mid-rail bias. Arming a lower limit of 500 mV
// * against that reading latches the trip on the first conversion and the run
// * dies before it starts - the board looks broken and nothing says why.
// *
// * Three things prevent it, in order:
// *   SAFE_ARM_IN_GATE    - do not look at all until PF3 is up, which is after
// *                         PB8 and after PD15 has powered the front end.
// *   SAFE_ARM_AFTER_TRIG - refuse to arm before the external start, belt and
// *                         braces for a configuration that opens the gate some
// *                         other way.
// *   SAFE_HEALTHY_COUNT  - after arming, the trip stays dormant until this
// *                         many CONSECUTIVE conversions land inside the
// *                         window. That is the real fix: it does not guess how
// *                         long settling takes, it waits for evidence of it.
// *
// * A window that never goes healthy is a fault, not a licence to run
// * unprotected, so SAFE_HEALTHY_TIMEOUT_US bounds the wait and drops PF3 if it
// * expires. Set it to 0 only if you would rather wait forever than trip.
// * ========================================================================== */
//
///* Outside this window, PF3 drops and latches until reset. */
//#define SAFE_HI_MV              2500
//#define SAFE_LO_MV              500
//
///* 1 = police the input only while PF3 is high.
//   0 = police from PD15, i.e. the whole run. */
//#define SAFE_ARM_IN_GATE        1
//
///* 1 = never arm the window before the PB8 start has been seen. Requires
//   EXT_TRIG_ENABLE. */
//#define SAFE_ARM_AFTER_TRIG     1
//
///* Consecutive IN-window conversions required before the trip goes live.
//   0 disables the qualification and arms immediately - the old behaviour, and
//   the one that latches shut on a cold front end. At 1.7 MSPS, 64 samples is
//   about 37 us. */
//#define SAFE_HEALTHY_COUNT      64U
//
///* How long to allow for that, microseconds, counted from arming. 0 waits
//   forever, which means the trip may never become live - say so out loud
//   rather than discovering it after a run. */
//#define SAFE_HEALTHY_TIMEOUT_US 2000UL
//
///* Ignore the input this long after arming, for a startup transient. Largely
//   superseded by SAFE_HEALTHY_COUNT, which waits for the signal to be right
//   rather than for a fixed time to pass. */
//#define SAFE_BLANK_US           0UL
//
///* Consecutive out-of-range conversions needed to trip. 1 trips on the first;
//   raising it rejects a single noisy conversion at the cost of that many
//   conversions of exposure. */
//#define SAFE_TRIP_COUNT         1U
//
///* PD14: the same waveform, this far behind PB5. */
//#define PD14_DELAY_NS           1600UL
//
///* The enable sequence, in order:
//       PD15 high  (startup circuit)
//         -> PF3_AFTER_PD15_US later, PF3 high  (gate driver)
//              -> GATE_SINE_PERIODS sine periods later, PF3 low
//                   -> PD15_AFTER_PF3_US later, PD15 low
//   The period count is a float, so 3.5 periods is expressible. */
//#define PF3_AFTER_PD15_US       1000UL
//#define GATE_SINE_PERIODS       5.0f
//#define PD15_AFTER_PF3_US       1000UL
//
///* 1 = the run waits for a rising edge on PB8 before PD15 goes high.
//   0 = PD15 rises as soon as the converter is running, which is the older
//       behaviour, unchanged in every other respect. */
//#define EXT_TRIG_ENABLE         1
//
///* How long to wait for that edge before giving up, milliseconds. 0 waits
//   forever. */
//#define EXT_TRIG_TIMEOUT_MS     30000UL
//
///* PB5 modulates only inside the PF3 window, and is idle low outside it.
//   This does NOT make PF3 an output enable. PF3 is the GATE DRIVER's own
//   enable, sequenced purely by time below: nothing in the decision logic ever
//   drives it and it is not derived from the PB5 state. This flag decides only
//   whether the modulator runs while the gate driver is disabled. Set it to 0
//   to have PB5 modulate for the whole PD15 window instead. */
//#define PWM_ACTIVE_IN_GATE      1
//
///* 1 swaps the law: above the upper limit drives PB5 ON instead of off. */
//#define ASP_INVERT              0
//
///* ADC. 12 bits: codes 0..4095, one LSB is 3300/4096 = 0.806 mV.
//   50 MHz is the rev V kernel-clock ceiling; the status block reports REV_ID so
//   a wrong choice for the silicon is visible. */
//#define ADC_RES_BITS            12U
//#define ADC_CLK_HZ              50000000UL
//#define ADC_SAMPLETIME          ADC_SAMPLETIME_1CYCLE_5
//#define ADC_SAMPLE_HALFCYCLES   3U      /* keep in step: 1.5 cyc = 3 halves */
//#define VREF_MV                 3234
//
///* Window LUT length. The value step between adjacent entries is
//   amplitude * 2*pi / LUT_N: 8192 entries holds it to 1.4 codes at full-scale
//   amplitude and well under one code at the 200 mV default. 4 bytes each, in
//   DTCM. */
//#define LUT_N                   8192U
//#define LUT_SHIFT               19U     /* 32 - log2(LUT_N) */
//
///* PD14 pending-edge queue. Three can be in flight at the defaults; eight
//   leaves room for shorter minimum times or a longer delay. Power of two. */
//#define PD14_QUEUE_N            16U
//
///* Ticks a compare must be in the future to be safe to arm. */
//#define PD14_MIN_AHEAD          24U
//
///* The trace. 4 bytes per recorded conversion, in DTCM.
//   8192 samples at the measured ~3.1 MSPS is about 2.6 ms of the gate; raise
//   TRACE_DECIMATE to cover a longer window at coarser resolution. */
//#define TRACE_ENABLE            1
//#define TRACE_SAMPLES           8192U
//#define TRACE_DECIMATE          1U
//
///* 240 MHz timer kernel clock: 480 MHz SYSCLK, HCLK /2, APB /2, timer x2. */
//#define TIMER_CLK_HZ            240000000UL
//
///* ==========================================================================
// * Pins
// * ========================================================================== */
//
//#define PWM_PORT                GPIOB
//#define PWM_PIN                 GPIO_PIN_5
//
//#define EN1_PORT                GPIOD           /* PD15 */
//#define EN1_PIN                 GPIO_PIN_15
//#define EN2_PORT                GPIOF           /* PF3  */
//#define EN2_PIN                 GPIO_PIN_3
//
//#define DLY_PORT                GPIOD           /* PD14 = TIM4_CH3, AF2 */
//#define DLY_PIN                 GPIO_PIN_14
//
//#define TRIG_PORT               GPIOB           /* PB8, external start  */
//#define TRIG_PIN                GPIO_PIN_8
//#define TRIG_IRQn               EXTI9_5_IRQn
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
///* Nanoseconds to TIM2 ticks. 240 ticks per microsecond, so the arithmetic is
//   exact for any multiple of 25/6 ns and never off by more than one tick. */
//#define NS_TO_TICKS(ns)         ((uint32_t)(((uint64_t)(ns) * TIMER_CLK_HZ) \
//                                            / 1000000000ULL))
//
//#define MIN_TON_TICKS           NS_TO_TICKS(MIN_TON_NS)
//#define MAX_TON_TICKS           NS_TO_TICKS(MAX_TON_NS)
//#define MIN_TOFF_TICKS          NS_TO_TICKS(MIN_TOFF_NS)
//#define MAX_TOFF_TICKS          NS_TO_TICKS(MAX_TOFF_NS)
//#define PD14_DELAY_TICKS        NS_TO_TICKS(PD14_DELAY_NS)
//#define SAFE_BLANK_TICKS        NS_TO_TICKS((uint64_t)SAFE_BLANK_US * 1000ULL)
//#define SAFE_HEALTHY_TICKS      NS_TO_TICKS((uint64_t)SAFE_HEALTHY_TIMEOUT_US \
//                                            * 1000ULL)
//#define PF3_DELAY_TICKS         NS_TO_TICKS((uint64_t)PF3_AFTER_PD15_US * 1000ULL)
//#define PD15_HOLD_TICKS         NS_TO_TICKS((uint64_t)PD15_AFTER_PF3_US * 1000ULL)
//
///* The gate, in 64-bit ticks. At 1 mHz and five periods this is 1.2e12 - five
//   thousand seconds - which is why it can no longer be a 32-bit count. */
//#define GATE_TICKS \
//    ((uint64_t)((double)GATE_SINE_PERIODS * 1000.0 \
//                * (double)TIMER_CLK_HZ / (double)SINE_MHZ))
//
///* WHETHER THE 64-BIT TIMEBASE IS NEEDED AT ALL.
//   TIM2 is 32 bits at 240 MHz and wraps every 17.9 s. A gate shorter than that
//   needs nothing more than the timer itself, and on this silicon the handler
//   and the conversion period are within a few nanoseconds of each other - so
//   the wrap counter, the 64-bit deadline compare and the phase re-anchor are
//   not small additions, they are the difference between main getting a slice
//   of the CPU and getting none. Measured: with them always on, main stopped
//   advancing entirely and never raised PD15.
//
//   So they are compiled in only when the gate is long enough to need them.
//   The threshold is 2^30 ticks - 4.5 s, a quarter of the wrap - which leaves
//   room for the signed compare and for the run's own preamble. Above about
//   0.2 Hz with five periods, this is 0 and the handler is byte-for-byte what
//   it was before sub-hertz support existed. */
///* The test is on the FREQUENCY, not on GATE_TICKS, because GATE_TICKS
//   contains a cast and a float and the preprocessor can evaluate neither - it
//   would silently fail to parse rather than give the wrong answer, but it is
//   still no basis for a #if. Two hertz with five periods is a 2.5 s gate,
//   comfortably inside the 4.5 s the 32-bit path can carry.
//
//   A long gate at a HIGH frequency - say two hundred periods at 100 Hz - would
//   slip through that test, so the static assert below checks the real gate and
//   fails the build with instructions rather than letting it run wrong. */
//#define FORCE_64BIT_TIME        0
//#define NEED_64BIT_TIME         (FORCE_64BIT_TIME || (SINE_MHZ < 2000UL))
//
//#if NEED_64BIT_TIME
//typedef uint64_t tick_t;
//typedef int64_t  stick_t;
//#else
//typedef uint32_t tick_t;
//typedef int32_t  stick_t;
//#endif
//
///* The frequency range the DDS scaling below is verified over. */
//#define SINE_MHZ_MIN            1UL
//#define SINE_MHZ_MAX            900000000UL
//
//
///* Safety window in ADC codes, so the interrupt compares integers. */
//#define SAFE_HI_CODES           ((uint32_t)(((uint64_t)SAFE_HI_MV \
//                                             << ADC_RES_BITS) / VREF_MV))
//#define SAFE_LO_CODES           ((uint32_t)(((uint64_t)SAFE_LO_MV \
//                                             << ADC_RES_BITS) / VREF_MV))
//
//#if TRACE_ENABLE
//_Static_assert(ADC_RES_BITS <= 12U,
//               "the trace word packs the code into bits 11:0 and the PB5 level "
//               "into bit 15, so it only holds 12 bits of resolution - widen "
//               "the word or drop TRACE_ENABLE for 14 or 16 bits");
//_Static_assert(TRACE_DECIMATE >= 1U, "TRACE_DECIMATE must be at least 1");
//_Static_assert((TRACE_SAMPLES * 4UL) + (LUT_N * 4UL) <= 96UL * 1024UL,
//               "the trace and the window LUT together do not leave enough of "
//               "the 128 KB DTCM for the stack - shrink TRACE_SAMPLES or LUT_N");
//#endif
//
//_Static_assert(SAFE_LO_MV < SAFE_HI_MV,
//               "the safety window is inverted - SAFE_LO_MV must be below "
//               "SAFE_HI_MV");
//_Static_assert(SAFE_HI_MV <= VREF_MV,
//               "SAFE_HI_MV above VREF - the upper trip could never fire");
//_Static_assert(SAFE_TRIP_COUNT >= 1U, "SAFE_TRIP_COUNT must be at least 1");
//_Static_assert(SINE_OFFSET_MV + SINE_AMP_MV + THRESHOLD_MV <= SAFE_HI_MV,
//               "the regulation band's upper limit is ABOVE the safety window, "
//               "so normal operation would trip PF3 and the run could never "
//               "complete - raise SAFE_HI_MV or lower the offset, amplitude or "
//               "threshold");
//_Static_assert(SINE_OFFSET_MV - SINE_AMP_MV - THRESHOLD_MV >= SAFE_LO_MV,
//               "the regulation band's lower limit is BELOW the safety window, "
//               "so normal operation would trip PF3 - lower SAFE_LO_MV or "
//               "raise the offset");
//
//_Static_assert(PLL2_VCO_HZ % ADC_CLK_HZ == 0U,
//               "ADC_CLK_HZ must divide the 400 MHz PLL2 VCO exactly");
//_Static_assert(ADC_CLK_HZ <= 50000000UL,
//               "adc_ker_ck above the rev V 50 MHz maximum");
//_Static_assert((1UL << (32U - LUT_SHIFT)) == LUT_N,
//               "LUT_SHIFT must be 32 - log2(LUT_N)");
//_Static_assert((PD14_QUEUE_N & (PD14_QUEUE_N - 1U)) == 0U,
//               "PD14_QUEUE_N must be a power of two");
//_Static_assert(SINE_MHZ >= SINE_MHZ_MIN && SINE_MHZ <= SINE_MHZ_MAX,
//               "SINE_MHZ is outside the verified 1 mHz .. 900 kHz range");
//
//#if SAFE_ARM_AFTER_TRIG && !EXT_TRIG_ENABLE
//#error "SAFE_ARM_AFTER_TRIG needs EXT_TRIG_ENABLE: there is no trigger to wait for"
//#endif
//
//#if !DWELL_MAX_ENABLE && (SAFE_LO_MV == 0)
//#error "MAX_TON and the lower safety limit are both off: nothing ends a stuck-ON"
//#endif
//
//_Static_assert(SAFE_HI_MV > SAFE_LO_MV,
//               "the safety window is inverted or empty");
//
///* THE HYSTERESIS BAND MUST BE NARROW COMPARED WITH THE SIGNAL.
//   The law only acts when the measurement leaves the band; inside it the law
//   holds and the stage is driven by the dwell ceilings alone, open loop. If
//   the band is as wide as the reference swing then the band at one phase
//   overlaps the band at every other phase, the comparator almost never fires,
//   and the output sits at whatever current the ceilings' duty produces - flat
//   through the middle of the waveform. That is what a saddle at mid-amplitude
//   is. Measured with THRESHOLD_MV 140 against SINE_AMP_MV 130: 173378 edges in
//   one gate, of which 86534 were forced_on and 1 was forced_off, so all but a
//   handful of the switching came from the ceiling rather than from the law. */
//_Static_assert(SINE_AMP_MV == 0 || 2 * THRESHOLD_MV <= SINE_AMP_MV,
//               "THRESHOLD_MV is too wide for SINE_AMP_MV: the loop will spend "
//               "the middle of every cycle in its hold region and free-run on "
//               "the dwell ceilings. Lower THRESHOLD_MV to a fraction of the "
//               "amplitude, or raise the amplitude.");
//_Static_assert(SAFE_LO_MV < SINE_OFFSET_MV - SINE_AMP_MV - THRESHOLD_MV,
//               "the lower safety limit cuts into the modulation band");
//_Static_assert(SAFE_HI_MV > SINE_OFFSET_MV + SINE_AMP_MV + THRESHOLD_MV,
//               "the upper safety limit cuts into the modulation band");
//_Static_assert(GATE_SINE_PERIODS > 0.0f, "GATE_SINE_PERIODS must be positive");
///* GATE_TICKS cannot be checked by a _Static_assert: it is a cast of a double
//   expression, so it is not an integer constant expression and the assert is
//   rejected before it can ever fail usefully. It is checked in main instead,
//   where the value is an ordinary number - see CTRL_STATUS_BAD_GATE. */
//
//_Static_assert(MAX_TON_NS == 0UL || MAX_TON_NS > MIN_TON_NS,
//               "MAX_TON_NS must exceed MIN_TON_NS, or be 0 to disable the "
//               "ceiling - otherwise every on time would be forced to end "
//               "before it was allowed to");
//_Static_assert(MAX_TOFF_NS == 0UL || MAX_TOFF_NS > MIN_TOFF_NS,
//               "MAX_TOFF_NS must exceed MIN_TOFF_NS, or be 0 to disable the "
//               "ceiling");
//
///* The PD14 queue has to be able to hold every edge that can be in flight
//   during one delay. Worst case is an edge every MIN(MIN_TON, MIN_TOFF). */
//#define PD14_MIN_SPACING_NS     ((MIN_TON_NS < MIN_TOFF_NS) ? MIN_TON_NS \
//                                                            : MIN_TOFF_NS)
//_Static_assert(PD14_MIN_SPACING_NS > 0UL,
//               "with no minimum on or off time, PB5 can flip every conversion "
//               "and the PD14 queue cannot be sized - set MIN_TON_NS and "
//               "MIN_TOFF_NS, or drive PD14 from a second comparison instead");
//_Static_assert((PD14_DELAY_NS / PD14_MIN_SPACING_NS) + 2UL <= PD14_QUEUE_N,
//               "PD14_QUEUE_N too small for this delay and minimum dwell - "
//               "raise it to (PD14_DELAY_NS / min dwell) + 2");
//#if PD14_ENABLE
//_Static_assert(PD14_DELAY_TICKS + PD14_MIN_AHEAD < 65536UL,
//               "PD14_DELAY_NS exceeds the 16-bit TIM4 range at 240 MHz "
//               "(273 us) - use TIM2 or TIM5 for a longer delay");
//#endif
//
///* ==========================================================================
// * Status block
// * ==========================================================================
// * SRAM1 (D2) at the base, non-cacheable through the MPU so the host reads what
// * the CPU wrote. .bss goes to DTCM, which the debugger can read but which is
// * awkward to point a script at; this address is the same one every other
// * program in this set publishes to.
// */
//#define CTRL_ADDR               0x30000000UL
///* Bits of relax_flags. */
//#define RELAX_DWELL_MIN         (1UL << 0)
//#define RELAX_DWELL_MAX         (1UL << 1)
//#define RELAX_PD14              (1UL << 2)
//#define RELAX_TRACE             (1UL << 3)
//#define RELAX_PROFILE           (1UL << 4)
//#define RELAX_STATS             (1UL << 5)
//#define RELAX_HEALTHY           (1UL << 6)
//
//#define RELAX_FLAGS \
//    ((DWELL_MIN_ENABLE    ? RELAX_DWELL_MIN : 0UL) \
//   | (DWELL_MAX_ENABLE    ? RELAX_DWELL_MAX : 0UL) \
//   | (PD14_ENABLE         ? RELAX_PD14      : 0UL) \
//   | (TRACE_ENABLE        ? RELAX_TRACE     : 0UL) \
//   | (ISR_PROFILE_ENABLE  ? RELAX_PROFILE   : 0UL) \
//   | (STATS_ENABLE        ? RELAX_STATS     : 0UL) \
//   | (SAFE_HEALTHY_COUNT  ? RELAX_HEALTHY   : 0UL))
//
//#define CTRL_MAGIC              0xC0DECAFEUL
//#define CTRL_VERSION            2UL
//
//#define CTRL_STATUS_INIT        0UL
//#define CTRL_STATUS_WAIT_GATE   1UL
//#define CTRL_STATUS_RUNNING     2UL
//#define CTRL_STATUS_DONE        3UL
//#define CTRL_STATUS_TRIPPED     4UL
//#define CTRL_STATUS_WAIT_TRIG   5UL   /* armed, waiting for a PB8 edge  */
//#define CTRL_STATUS_NO_TRIG     6UL   /* gave up waiting for one        */
//#define CTRL_STATUS_UNHEALTHY    8UL  /* window never settled in range  */
//#define CTRL_STATUS_BAD_GATE    7UL   /* gate too long for the timebase */
//
//typedef struct
//{
//  uint32_t magic;               /* CTRL_MAGIC                              */
//  uint32_t version;             /* 2                                       */
//  uint32_t status;              /* CTRL_STATUS_*                           */
//  uint32_t rev_id;
//  uint32_t timer_clk_hz;
//  uint32_t adc_clk_hz;
//  uint32_t adc_bits;
//  uint32_t vref_mv;
//  uint32_t adc_conv_halfcycles;
//  uint32_t adc_rate_nom_hz;      /* always 0 - there is no target rate      */
//  uint32_t adc_rate_meas_hz;    /* MEASURED: interrupts / gate duration    */
//  uint32_t isr_count;           /* interrupts INSIDE the gate              */
//  uint32_t gate_ticks;          /* MEASURED length of the PF3 window       */
//  uint32_t ext_trig_count;      /* PB8 edges seen since reset. This word
//                                   used to be isr_max_ticks, which has been
//                                   published as 0 since the duration moved to
//                                   the cycle counter - so the index is reused
//                                   rather than moving every field after it. */
//  uint32_t adc_ovr;             /* 1 if the ADC ever overran               */
//  uint32_t sine_hz;
//  uint32_t sine_offset_mv;
//  uint32_t sine_amp_mv;
//  uint32_t threshold_mv;
//  uint32_t min_ton_ns;
//  uint32_t max_ton_ns;
//  uint32_t min_toff_ns;
//  uint32_t max_toff_ns;
//  uint32_t pd14_delay_ns;
//  uint32_t pf3_after_pd15_us;
//  uint32_t pd15_after_pf3_us;
//  uint32_t edges;               /* PB5 transitions                         */
//  uint32_t forced_off;          /* MAX_TON hits                            */
//  uint32_t forced_on;           /* MAX_TOFF hits                           */
//  uint32_t blocked_off;         /* law wanted LOW inside MIN_TON           */
//  uint32_t blocked_on;          /* law wanted HIGH inside MIN_TOFF         */
//  uint32_t pd14_queue_max;      /* high-water mark                         */
//  uint32_t pd14_late;           /* compares clamped into the future        */
//  uint32_t pd14_dropped;        /* toggles LOST - PD14 parity now inverted */
//  uint32_t clamp_hi;            /* LUT entries whose upper limit saturated */
//  uint32_t clamp_lo;
//  uint32_t lut_n;
//  uint32_t gate_periods_milli;  /* GATE_SINE_PERIODS * 1000                */
//  uint32_t on_ticks;            /* total time PB5 spent HIGH               */
//  uint32_t safe_hi_mv;
//  uint32_t safe_lo_mv;
//  uint32_t tripped;             /* 1 = the safety trip fired, latched      */
//  uint32_t trip_code;           /* the conversion that did it              */
//  uint32_t trip_ticks;          /* ticks from arming to the trip           */
//  uint32_t trace_addr;          /* where the trace lives, 0 if disabled    */
//  uint32_t trace_count;         /* words actually recorded                 */
//  uint32_t trace_stride;        /* bytes per word, 4                       */
//  uint32_t trace_decimate;      /* conversions per recorded word           */
//  uint32_t isr_total;           /* interrupts over the WHOLE run           */
//  uint32_t isr_max_cycles;      /* WORST-CASE handler, in CPU cycles       */
//  uint32_t cpu_hz;              /* so the host can turn those into ns      */
//  uint32_t sine_mhz;            /* the frequency actually running, mHz     */
//  uint32_t gate_us;             /* MEASURED gate length, microseconds. The
//                                   tick count is 32 bits and a millihertz
//                                   gate is minutes long, so trust this one. */
//  uint32_t ext_trig_enabled;    /* 1 = the run waited for a PB8 edge        */
//  uint32_t phase_mul;           /* the DDS scaling in force, for diagnosis  */
//  uint32_t phase_shift;
//  uint32_t tim2_wraps;          /* 32-bit timer overflows - the high half of
//                                   the 64-bit timebase                      */
//  uint32_t relax_flags;         /* which controls this build kept - see
//                                   RELAX_* below. Without it a capture cannot
//                                   be told apart from one taken with a
//                                   different set of options.              */
//  uint32_t safe_healthy;        /* 1 = the safety window went live         */
//  uint32_t safe_hi_codes;       /* the window as the handler compares it   */
//  uint32_t safe_lo_codes;
//  uint32_t reserved[3];
//} ctrl_blk_t;
//
//_Static_assert(sizeof(ctrl_blk_t) == 256U, "status block must stay 256 bytes");
//
//static volatile ctrl_blk_t * const ctrl = (volatile ctrl_blk_t *)CTRL_ADDR;
//
///* ==========================================================================
// * State
// * ========================================================================== */
//
///* Packed comparison limits per phase step: upper in bits 15:0, lower in
//   31:16, both in ADC codes, both clamped. .bss, so DTCM. */
//static uint32_t window_lut[LUT_N];
//
//static ADC_HandleTypeDef hadc1;
//static TIM_HandleTypeDef htim2;
//#if PD14_ENABLE
//static TIM_HandleTypeDef htim4;
//#endif
//
///* Written by main before the gate opens, read by the interrupt. */
//static volatile tick_t   phase_ref;   /* ticks at the gate's opening        */
//static volatile uint32_t phase_base;  /* phase already accumulated at it    */
//static volatile uint32_t gate_open;
//
///* The DDS step, as (mul >> shift) phase units per tick. Both are computed once
//   at startup by Dds_Scale(); see the header for why one fixed scale cannot
//   cover millihertz and kilohertz at the same time. */
//static uint32_t phase_mul;
//static uint32_t phase_shift;
//
//#if NEED_64BIT_TIME
///* The 64-bit timebase. tim2_hi counts TIM2 overflows; the handler reads the
//   timer far more often than it wraps, so a single compare cannot miss one. */
//static volatile uint32_t tim2_hi;
//static uint32_t          tim2_last;
//#endif
//
///* The external start. ext_trig_fired is set by the EXTI handler and cleared by
//   main just before it arms, so an edge that arrived while nobody was waiting
//   cannot fire the run. ext_trig_count keeps counting regardless, which is what
//   tells you whether a pulse is reaching the pin at all. */
//static volatile uint32_t ext_trig_fired;
//static volatile uint32_t ext_trig_count;
//
///* The enable sequence, advanced by the interrupt. seq_deadline is always a
//   valid TIM2 tick: while there is nothing to do it sits far in the future, so
//   the handler's hot path is one load, one signed subtract and a branch that is
//   not taken. */
//#define SEQ_IDLE                0U   /* nothing armed yet                   */
//#define SEQ_WAIT_PF3            1U   /* PD15 up, waiting to raise PF3       */
//#define SEQ_GATE                2U   /* PF3 up, waiting to drop it          */
//#define SEQ_POST                3U   /* PF3 down, waiting to drop PD15      */
//#define SEQ_DONE                4U   /* sequence complete                   */
//
///* trip_code when the trip fired because the window never went healthy, rather
//   than because a conversion left it. No conversion can produce this value. */
//#define TRIP_CODE_UNHEALTHY     0xFFFFFFFFUL
//
///* Parked this far ahead when no edge is pending - about 4.5 s at 240 MHz,
//   which the signed compare treats as "not yet" without any special case. */
///* Parked this far ahead when nothing is due.
//   IT MUST FIT IN tick_t. Written as (1ULL << 44) unconditionally it truncated
//   to EXACTLY ZERO in the 32-bit build, so "park forever" became "fire
//   immediately": Seq_Advance() was then called on every single conversion for
//   the whole life of the program, which is what stopped main from ever
//   finishing the run and publishing it.
//   In the 64-bit build it has to exceed the longest legal gate - thousands of
//   seconds at 1 mHz - so the two cases need different values, and the runtime
//   gate check in main keeps the 32-bit case inside 0x40000000. */
//#if NEED_64BIT_TIME
//#define SEQ_NEVER_TICKS         (1ULL << 44)   /* ~20 hours */
//#else
//#define SEQ_NEVER_TICKS         0x40000000UL   /* 4.5 s, the most 32 bits
//                                                  can carry with a signed
//                                                  compare                  */
//#endif
//_Static_assert((tick_t)SEQ_NEVER_TICKS == SEQ_NEVER_TICKS,
//               "SEQ_NEVER_TICKS does not fit in tick_t - it would truncate and "
//               "the sequence deadline would fire on every conversion");
//
//
//static volatile uint32_t seq_state;
//static volatile tick_t   seq_deadline;
//
//static volatile tick_t   gate_meas_ticks;   /* how long PF3 was really up */
//static tick_t            gate_ticks_cfg;    /* set once, by main          */
//
///* The safety trip. safe_arm_tick is written before safe_arm, so the interrupt
//   can never see an armed trip with a stale reference. tripped is the latch -
//   nothing anywhere clears it, which is what "until reset" means. */
//static volatile uint32_t safe_arm;
//static volatile tick_t   safe_arm_tick;
//static volatile uint32_t tripped;
//static volatile uint32_t trip_code;
//static volatile tick_t   trip_ticks;
//
///* Owned by the interrupt alone. */
//static uint32_t pb5_high;          /* current PB5 level                    */
//static uint32_t last_edge_tick;
///* volatile: written by the handler, read by main when it publishes. */
//static volatile uint32_t isr_total;      /* every interrupt              */
//static volatile uint32_t isr_count;      /* only those inside the gate   */
//static volatile uint32_t isr_max_cycles; /* only those inside the gate   */
//static uint32_t edges;
//static uint32_t forced_off, forced_on, blocked_off, blocked_on;
//static uint32_t on_ticks;          /* accumulated HIGH time                */
//static uint32_t safe_run;          /* consecutive out-of-range conversions */
//#if SAFE_HEALTHY_COUNT
//static volatile uint32_t safe_healthy;  /* 1 = the trip is live            */
//static uint32_t          safe_good;     /* consecutive in-range so far     */
//#endif
//
//#if TRACE_ENABLE
///* .bss, so DTCM. Written by the handler, read by the host through the address
//   published in the status block. */
//static uint32_t trace_buf[TRACE_SAMPLES];
//static uint32_t trace_n;           /* words recorded so far                */
//static uint32_t trace_skip;        /* decimation counter                   */
//#endif
//
//static uint16_t pd14_q[PD14_QUEUE_N];
//static uint32_t pd14_head, pd14_tail, pd14_busy;
//static uint32_t pd14_queue_max, pd14_late, pd14_dropped;
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
//  mpu.BaseAddress      = CTRL_ADDR;
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
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//
//  /* PC0 = ADC1_INP10. */
//  g.Pin  = GPIO_PIN_0;
//  g.Mode = GPIO_MODE_ANALOG;
//  g.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &g);
//
//  /* Every output is driven to its idle level BEFORE the pin becomes an
//     output, so none of them can glitch while being configured. MEDIUM speed:
//     enough slew for edges this fast, and far less ringing into the analog
//     front end than VERY_HIGH - the input being measured is on the same
//     board. */
//  HAL_GPIO_WritePin(PWM_PORT, PWM_PIN, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, GPIO_PIN_RESET);
//
//  g.Mode      = GPIO_MODE_OUTPUT_PP;
//  g.Pull      = GPIO_NOPULL;
//  g.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//  g.Alternate = 0;
//
//  g.Pin = PWM_PIN;  HAL_GPIO_Init(PWM_PORT, &g);
//  g.Pin = EN1_PIN;  HAL_GPIO_Init(EN1_PORT, &g);
//  g.Pin = EN2_PIN;  HAL_GPIO_Init(EN2_PORT, &g);
//
//  /* PB8: the external start. Pull-down and rising-edge detection, so a
//     disconnected pin rests low and only a real drive can begin a run - the
//     opposite arrangement would let a floating input start the power stage.
//
//     Priority 1, below the conversion interrupt at 0. The edge only sets a
//     flag that main polls, so nothing is lost by letting a conversion finish
//     first, and the conversion path must not be delayed by anything. */
//  g.Pin  = TRIG_PIN;
//  g.Mode = GPIO_MODE_IT_RISING;
//  g.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(TRIG_PORT, &g);
//  HAL_NVIC_SetPriority(TRIG_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(TRIG_IRQn);
//
//#if PD14_ENABLE
//  /* PD14 belongs to TIM4_CH3, AF2 - the timer drives the pin, not the CPU, so
//     writing PD14's BSRR would do nothing. */
//  g.Pin       = DLY_PIN;
//  g.Mode      = GPIO_MODE_AF_PP;
//  g.Pull      = GPIO_NOPULL;
//  g.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//  g.Alternate = GPIO_AF2_TIM4;
//  HAL_GPIO_Init(DLY_PORT, &g);
//#else
//  /* The delayed copy is compiled out, so TIM4 is never clocked. Leaving the
//     pin on AF2 would hand it to a dead timer and its level would be whatever
//     the output flip-flop powered up holding. A plain output, driven low, is a
//     state that can be reasoned about. */
//  DLY_PORT->BSRR = (uint32_t)DLY_PIN << 16;
//  g.Pin   = DLY_PIN;
//  g.Mode  = GPIO_MODE_OUTPUT_PP;
//  g.Pull  = GPIO_NOPULL;
//  g.Speed = GPIO_SPEED_FREQ_MEDIUM;
//  HAL_GPIO_Init(DLY_PORT, &g);
//#endif
//}
//
///** TIM2: free-running 32-bit counter at 240 MHz. Phase reference, dwell clock
//    and measurement clock. 4.166 ns per tick, wraps every 17.9 s, and every
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
//#if PD14_ENABLE
///**
//  * @brief TIM4 CH3 on PD14: free-running 16-bit at 240 MHz, toggle on compare.
//  *
//  * The channel starts in FORCED_INACTIVE so that enabling the output drives
//  * PD14 low with certainty - a toggle-mode channel's initial level would
//  * otherwise depend on whatever the output flip-flop happened to hold. Pd14_Arm()
//  * switches the mode to TOGGLE once the pin is known to be low and PB5 is
//  * known to be low, which is what keeps the two pins' levels in step: from
//  * then on every PB5 edge produces exactly one PD14 toggle.
//  *
//  * No interrupt is enabled on TIM4. The EOC handler polls CC3IF instead, which
//  * is safe because it is then the only writer of CCR3 - see the header.
//  */
//static void Delay_Timer_Config(void)
//{
//  TIM_OC_InitTypeDef oc = {0};
//
//  __HAL_RCC_TIM4_CLK_ENABLE();
//
//  htim4.Instance               = TIM4;
//  htim4.Init.Prescaler         = 0;
//  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
//  htim4.Init.Period            = 0xFFFFU;
//  htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  oc.OCMode     = TIM_OCMODE_FORCED_INACTIVE;
//  oc.Pulse      = 0;
//  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
//  oc.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_OC_ConfigChannel(&htim4, &oc, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///** Switch CH3 from forced-low to toggle-on-compare. PD14 is low and PB5 is
//    low at this point, so the two stay in step from here. */
//static void Pd14_Arm(void)
//{
//  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC3);
//  MODIFY_REG(TIM4->CCMR2, TIM_CCMR2_OC3M, TIM_OCMODE_TOGGLE);
//}
//#endif  /* PD14_ENABLE */
//
///**
//  * @brief ADC1 on PC0, continuous, software start, EOC interrupt per
//  *        conversion, no DMA.
//  *
//  * ConversionDataManagement stays at DR: the handler reads DR itself, which is
//  * also what clears EOC. Continuous mode means one ADSTART and the converter
//  * never stops, so there is no trigger to be dropped and no timer in the
//  * sample path - the rate is whatever the hardware manages, which is the point.
//  *
//  * Overrun is left in overwrite mode. It should never occur here because the
//  * handler reads every conversion, so the OVR flag is a genuine signal that
//  * the handler missed one - main reads it at the end of the run and publishes
//  * it. That is the one unambiguous test of whether this architecture keeps up.
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
//  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
//  hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
//  hadc1.Init.OversamplingMode         = DISABLE;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Independent mode, explicitly: the DUAL field lives in the ADC12 COMMON
//     register, which HAL_ADC_Init() does not touch, so dual interleaved mode
//     left set by adc_interleaved_probe.c would still be set here and the data
//     would arrive through CDR instead of DR. */
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
//
//  /* Top priority: this handler has a few hundred nanoseconds to run and
//     nothing else in the system is time-critical. */
//  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(ADC_IRQn);
//}
//
///**
//  * @brief PB8 rose. Record it and get out.
//  *
//  * Deliberately does nothing but stamp a flag and a count. Raising PD15 from
//  * here would put the start of a power sequence inside an interrupt; main is
//  * already waiting a few instructions away, and the cost of letting it do the
//  * work is one poll of a volatile word.
//  */
//void EXTI9_5_IRQHandler(void)
//{
//  if (__HAL_GPIO_EXTI_GET_IT(TRIG_PIN) != 0x00U)
//  {
//    __HAL_GPIO_EXTI_CLEAR_IT(TRIG_PIN);
//    ext_trig_count++;
//    ext_trig_fired = 1U;
//  }
//}
//
///**
//  * @brief Split the frequency into (mul, shift) so the DDS step keeps its bits.
//  *
//  * The step wanted is f * 2^32 / TIMER_CLK_HZ phase units per tick - 0.0179 at
//  * 1 mHz, 1.6e7 at 900 kHz. No single fixed-point format holds both: a scale
//  * coarse enough for the top of the range rounds the bottom to nothing, which
//  * is what the old Q8 step did, turning 1 mHz into a 13% error.
//  *
//  * So the binary point moves. This is ordinary long division carried on past
//  * the point, stopping as soon as the quotient fills the top of a 32-bit word,
//  * with the shift recording how far it went. The mantissa therefore always
//  * carries about 30 significant bits whatever the frequency, and the handler's
//  * multiply stays a single 64-bit operation.
//  *
//  * Startup only - the frequency is fixed for the run.
//  */
//static void Dds_Scale(uint32_t sine_mhz, uint32_t *mul, uint32_t *shift)
//{
//  /* Ticks per second x 1000, because the frequency is x 1000. */
//  const uint64_t den = 1000ULL * (uint64_t)TIMER_CLK_HZ;
//  uint64_t n = (uint64_t)sine_mhz << 32;      /* <= 3.9e18, no overflow */
//  uint64_t q = n / den;
//  uint64_t r = n % den;
//  uint32_t sh = 0U;
//
//  while ((q < (1ULL << 30)) && (sh < 40U))
//  {
//    r <<= 1;
//    q <<= 1;
//    if (r >= den)
//    {
//      r -= den;
//      q += 1U;
//    }
//    sh++;
//  }
//  *mul   = (uint32_t)q;
//  *shift = sh;
//}
//
///* ==========================================================================
// * The window LUT
// * ========================================================================== */
//
///** sin(2*pi*x) for x in [0,1), without libm: quadrant folding then a
//    7th-order odd polynomial, good to about 1e-6. Startup only. */
//static float sine_turn(float x)
//{
//  float sign = 1.0f;
//
//  if (x >= 0.5f)
//  {
//    x -= 0.5f;
//    sign = -1.0f;
//  }
//  if (x > 0.25f)
//  {
//    x = 0.5f - x;
//  }
//
//  const float t  = x * 6.28318530718f;
//  const float t2 = t * t;
//  const float s  = t * (1.0f + t2 * (-1.0f / 6.0f
//                            + t2 * (1.0f / 120.0f
//                            + t2 * (-1.0f / 5040.0f))));
//  return sign * s;
//}
//
///**
//  * @brief Build the packed limits so the interrupt needs no arithmetic.
//  *
//  * Entry i, for phase i/LUT_N of a cycle:
//  *     bits 15:0   sine + threshold, the upper limit
//  *     bits 31:16  sine - threshold, the lower limit
//  * both clamped into 0..full scale. Saturation is counted, not hidden: a
//  * clamped upper limit means the input can never exceed it, so that branch
//  * cannot fire at that phase, and the status block says how many entries are
//  * affected.
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
// * PD14: push one delayed edge
// * ========================================================================== */
//
//#if PD14_ENABLE
///** Arm CCR3, clamping a due time that has already slipped into the past. */
//static inline void pd14_load(uint16_t due)
//{
//  const uint16_t ahead = (uint16_t)(due - (uint16_t)TIM4->CNT);
//
//  if ((ahead == 0U) || (ahead > (uint16_t)0x8000U)
//                    || (ahead < (uint16_t)PD14_MIN_AHEAD))
//  {
//    due = (uint16_t)(TIM4->CNT + PD14_MIN_AHEAD);
//    pd14_late++;
//  }
//  TIM4->CCR3 = due;
//}
//
///**
//  * @brief Queue a PD14 toggle PD14_DELAY_TICKS after now.
//  *
//  * Called from the EOC handler at the same moment PB5 is written, so the delay
//  * is measured from the real PB5 edge. If no compare is pending the value goes
//  * straight into CCR3; otherwise it waits in the queue and the handler arms it
//  * when it sees the previous compare fire.
//  */
//static inline void pd14_push(void)
//{
//  const uint16_t due = (uint16_t)(TIM4->CNT + PD14_DELAY_TICKS);
//
//  if (!pd14_busy)
//  {
//    pd14_load(due);
//    pd14_busy = 1U;
//    return;
//  }
//
//  const uint32_t next = (pd14_head + 1U) & (PD14_QUEUE_N - 1U);
//  if (next != pd14_tail)          /* full queue drops, and says so */
//  {
//    pd14_q[pd14_head] = due;
//    pd14_head         = next;
//
//    const uint32_t used = (pd14_head - pd14_tail) & (PD14_QUEUE_N - 1U);
//    if (used > pd14_queue_max)
//    {
//      pd14_queue_max = used;
//    }
//  }
//  else
//  {
//    /* A dropped toggle is not a timing error, it is a correctness one: PD14's
//       level is the parity of the toggles it has received, so losing one leaves
//       PD14 inverted with respect to PB5 for the rest of the run. The static
//       assert on PD14_QUEUE_N is meant to make this unreachable; the counter
//       exists so that if it ever happens it is visible rather than looking
//       like a wiring mistake on the scope. */
//    pd14_dropped++;
//  }
//}
//#endif  /* PD14_ENABLE */
//
///**
//  * @brief Start DWT->CYCCNT, the core's cycle counter.
//  *
//  * TRCENA gates the whole trace/debug block; CYCCNT itself then has to be
//  * enabled. A debugger can own DWT and undo this, so the caller checks that
//  * the counter actually moves rather than trusting it - a diagnostic that
//  * silently reports zero is worse than no diagnostic.
//  *
//  * 32 bits at 480 MHz wraps every 8.95 s, which no single interrupt can span,
//  * and the subtraction is unsigned so a wrap inside one costs nothing.
//  */
//static bool Dwt_Start(void)
//{
//  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//  DWT->CYCCNT       = 0U;
//  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
//
//  const uint32_t a = DWT->CYCCNT;
//  __NOP(); __NOP(); __NOP(); __NOP();
//  return (DWT->CYCCNT != a);
//}
//
///* ==========================================================================
// * The enable sequence
// * ==========================================================================
// * Called from the interrupt only when the deadline has passed, so its cost is
// * off the hot path entirely. Deadlines are compared as SIGNED differences, so
// * a TIM2 wrap between arming and firing costs nothing.
// */
//static void Seq_Advance(uint64_t now)
//{
//  switch (seq_state)
//  {
//    case SEQ_WAIT_PF3:
//      /* Raise the gate-driver enable and start the sine on the same tick. */
//      phase_ref      = now;
//      phase_base     = 0U;
//      last_edge_tick = (uint32_t)now;
//#if SAFE_ARM_IN_GATE
//#if SAFE_HEALTHY_COUNT
//      safe_healthy   = 0U;       /* prove the input before trusting it */
//      safe_good      = 0U;
//#endif
//      safe_arm_tick  = now;      /* reference first, then arm */
//      safe_arm       = 1U;
//#endif
//      /* OVR is sticky and the cold first interrupt of the run can set it.
//         Clear it here so it means "the handler fell behind inside the gate".
//         Write-1-to-clear, so this touches nothing else. */
//      ADC1->ISR      = ADC_ISR_OVR;
//
//      gate_open      = 1U;
//      EN2_PORT->BSRR = (uint32_t)EN2_PIN;
//      seq_deadline   = now + gate_ticks_cfg;
//      seq_state      = SEQ_GATE;
//      break;
//
//    case SEQ_GATE:
//      gate_open       = 0U;
//      EN2_PORT->BSRR  = (uint32_t)EN2_PIN << 16;
//      gate_meas_ticks = now - phase_ref;
//      seq_deadline    = now + PD15_HOLD_TICKS;
//      seq_state       = SEQ_POST;
//      break;
//
//    case SEQ_POST:
//      EN1_PORT->BSRR = (uint32_t)EN1_PIN << 16;
//      seq_deadline   = now + SEQ_NEVER_TICKS;
//      seq_state      = SEQ_DONE;
//      break;
//
//    default:
//      seq_deadline = now + SEQ_NEVER_TICKS;
//      break;
//  }
//}
//
///* ==========================================================================
// * The decision, once per conversion
// * ==========================================================================
// * Order inside the handler:
// *   1. latch TIM2 once - the same value times the dwell test AND stamps the
// *      new edge, so a minimum time can never be understated by the few
// *      nanoseconds between two reads. Getting that wrong once before cost an
// *      effective 250 ns of minimum on time.
// *   2. read DR, which clears EOC.
// *   3. service any PD14 compare that has fired.
// *   4. phase from TIM2, limits from the LUT, decide, apply subject to dwell.
// */
//void ADC_IRQHandler(void)
//{
//#if ISR_PROFILE_ENABLE
//  const uint32_t c0  = DWT->CYCCNT;      /* core-private, ~2 cycles */
//#endif
//  const uint32_t now = TIM2->CNT;
//  const uint32_t a   = ADC1->DR;         /* clears EOC */
//
//#if NEED_64BIT_TIME
//  /* Extend the timebase to 64 bits. TIM2 wraps every 17.9 s, which is shorter
//     than this gate. The handler reads the counter every few hundred
//     nanoseconds so one compare catches every wrap. Compiled in only when the
//     gate is long enough to need it - see NEED_64BIT_TIME. */
//  if (now < tim2_last)
//  {
//    tim2_hi++;
//  }
//  tim2_last = now;
//  const tick_t now64 = ((uint64_t)tim2_hi << 32) | (uint64_t)now;
//#else
//  /* The gate fits inside one wrap of TIM2, so the timer IS the timebase and
//     every line below costs exactly what it did before sub-hertz support. */
//  const tick_t now64 = now;
//#endif
//
//  isr_total++;
//
//#if PD14_ENABLE
//  /* PD14: re-arm from the queue if the pending compare has come out.
//     This is an APB read on every conversion, pending or not, and APB is the
//     slowest thing the handler touches - which is why PD14_ENABLE exists. */
//  if (TIM4->SR & TIM_SR_CC3IF)
//  {
//    TIM4->SR = (uint16_t)~TIM_SR_CC3IF;
//    if (pd14_head != pd14_tail)
//    {
//      pd14_load(pd14_q[pd14_tail]);
//      pd14_tail = (pd14_tail + 1U) & (PD14_QUEUE_N - 1U);
//    }
//    else
//    {
//      pd14_busy = 0U;
//    }
//  }
//#endif
//
//  /* ---- the enable sequence ---------------------------------------------
//     One signed compare. The branch is not taken on all but four interrupts of
//     the entire run, and taking it is what places PF3 to within one conversion
//     of its deadline regardless of what else the CPU is doing. */
//  /* The deadline test, on the path taken by EVERY conversion.
//     A full 64-bit compare here cost what it only needs to cost four times in a
//     run, and on this silicon the handler and the conversion period are within
//     a few nanoseconds of each other - so those cycles came straight out of
//     main's share, which is what is left over. Measured: main stopped
//     advancing at all.
//     The high halves are compared first. While the deadline is parked at
//     SEQ_NEVER its high half is huge, so this is one compare that falls
//     through. Only when the run is armed do the halves match and the 32-bit
//     compare below decide anything. */
//  if ((stick_t)(now64 - seq_deadline) >= 0)
//  {
//    Seq_Advance(now64);
//  }
//
//  /* ---- the safety trip, checked before anything decides an output ------
//     PF3 is dropped here, in the same interrupt as the offending sample, not
//     handed to main - main may be inside a spin loop microseconds long. The
//     latch is never cleared. */
//  if (tripped)
//  {
//    goto out;
//  }
//#if SAFE_BLANK_US
//  if (safe_arm && ((now64 - safe_arm_tick) >= SAFE_BLANK_TICKS))
//#else
//  if (safe_arm)                  /* no blanking: armed is armed */
//#endif
//  {
//    const uint32_t out_of_range = (a > SAFE_HI_CODES) || (a < SAFE_LO_CODES);
//
//#if SAFE_HEALTHY_COUNT
//    /* Until the input has been demonstrably inside the window for a while,
//       the trip is not live. A cold sense amplifier reads near zero, which is
//       below any useful lower limit, and arming against that would latch the
//       run shut before it delivered a single edge. */
//    if (!safe_healthy)
//    {
//      if (out_of_range)
//      {
//        safe_good = 0U;
//#if SAFE_HEALTHY_TIMEOUT_US
//        /* Never settling is a fault in the sense path, and running on with
//           no protection at all is the one outcome worse than stopping. */
//        if ((now64 - safe_arm_tick) >= SAFE_HEALTHY_TICKS)
//        {
//          EN2_PORT->BSRR  = (uint32_t)EN2_PIN << 16;   /* PF3 LOW, at once */
//          gate_open       = 0U;
//          trip_code       = TRIP_CODE_UNHEALTHY;
//          trip_ticks      = now64 - safe_arm_tick;
//          tripped         = 1U;
//          gate_meas_ticks = 0U;
//          seq_deadline    = now64 + PD15_HOLD_TICKS;
//          seq_state       = SEQ_POST;
//          if (pb5_high)
//          {
//            PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//            pb5_high       = 0U;
//#if STATS_ENABLE
//            on_ticks      += now - last_edge_tick;
//            edges++;
//#endif
//            last_edge_tick = now;
//#if PD14_ENABLE
//            pd14_push();
//#endif
//          }
//          goto out;
//        }
//#endif
//      }
//      else if (++safe_good >= SAFE_HEALTHY_COUNT)
//      {
//        safe_healthy = 1U;
//      }
//    }
//    else
//#endif
//    if (out_of_range)
//    {
//      safe_run++;
//      if (safe_run >= SAFE_TRIP_COUNT)
//      {
//        EN2_PORT->BSRR = (uint32_t)EN2_PIN << 16;   /* PF3 LOW, at once */
//        gate_open  = 0U;
//        trip_code  = a;
//        trip_ticks = now64 - safe_arm_tick;
//        tripped    = 1U;
//
//        /* PF3 is down for good, but the rest of the sequence still has to
//           run: PD15 comes down PD15_AFTER_PF3_US later exactly as it would
//           have at a normal end of gate. gate_meas_ticks records how much of
//           the window was actually delivered - zero if the trip beat PF3 up. */
//        gate_meas_ticks = (seq_state == SEQ_GATE) ? (now64 - phase_ref) : 0U;
//        seq_deadline    = now64 + PD15_HOLD_TICKS;
//        seq_state       = SEQ_POST;
//
//        if (pb5_high)
//        {
//          PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//          pb5_high       = 0U;
//#if STATS_ENABLE
//          on_ticks      += now - last_edge_tick;
//          edges++;
//#endif
//          last_edge_tick = now;
//#if PD14_ENABLE
//          pd14_push();
//#endif
//        }
//        goto out;
//      }
//    }
//    else
//    {
//      safe_run = 0U;             /* the run has to be consecutive */
//    }
//  }
//
//#if PWM_ACTIVE_IN_GATE
//  if (!gate_open)
//  {
//    /* Outside the gate-driver window: hold the modulator off. The drop is
//       immediate, without waiting for MIN_TON. PD14 follows one delay later so
//       the pair keeps its shape. */
//    if (pb5_high)
//    {
//      PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//      pb5_high       = 0U;
//#if STATS_ENABLE
//      on_ticks      += now - last_edge_tick;
//      edges++;
//#endif
//      last_edge_tick = now;
//#if PD14_ENABLE
//      pd14_push();
//#endif
//    }
//    goto out;
//  }
//#endif
//
//  {
//    /* The DDS. The tick difference is 64 bit and the step is (mul >> shift)
//       with both picked from the frequency, so one expression covers
//       millihertz and kilohertz alike. Re-anchoring keeps the product inside
//       64 bits: a difference of 2^32 against a mantissa of 2^31 is 2^63, and
//       without it a long gate would run past that. The branch is taken once
//       every 17.9 seconds. */
//#if NEED_64BIT_TIME
//    uint64_t dt = now64 - phase_ref;
//    if (dt >= (1ULL << 32))
//    {
//      phase_base += (uint32_t)((dt * phase_mul) >> phase_shift);
//      phase_ref   = now64;
//      dt          = 0U;
//    }
//    const uint32_t phase = phase_base
//                         + (uint32_t)((dt * phase_mul) >> phase_shift);
//#else
//    /* One multiply. The difference cannot exceed 2^32 and the mantissa cannot
//       exceed 2^31, so the product fits without any re-anchoring. */
//    const uint32_t phase = (uint32_t)(((uint64_t)(now64 - phase_ref)
//                                       * phase_mul) >> phase_shift);
//#endif
//    const uint32_t w     = window_lut[phase >> LUT_SHIFT];
//    const uint32_t hi    = w & 0xFFFFU;
//    const uint32_t lo    = w >> 16;
//#if DWELL_MIN_ENABLE || DWELL_MAX_ENABLE || STATS_ENABLE
//    const uint32_t held  = now - last_edge_tick;
//#endif
//
//    /* Tri-state: above the upper limit, below the lower, or hold. */
//    int32_t want = 0;                        /* 0 = hold */
//    if (a > hi)      { want = -1; }          /* above sine+thr -> OFF */
//    else if (a < lo) { want =  1; }          /* below sine-thr -> ON  */
//#if ASP_INVERT
//    want = -want;
//#endif
//
//    if (pb5_high)
//    {
//#if DWELL_MAX_ENABLE
//      if ((MAX_TON_TICKS != 0U) && (held >= MAX_TON_TICKS))
//      {
//        PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//        pb5_high       = 0U;
//#if STATS_ENABLE
//        on_ticks      += held;
//        edges++;
//        forced_off++;
//#endif
//        last_edge_tick = now;
//#if PD14_ENABLE
//        pd14_push();
//#endif
//      }
//      else
//#endif
//      if (want < 0)
//      {
//#if DWELL_MIN_ENABLE
//        if (held < MIN_TON_TICKS)
//        {
//#if STATS_ENABLE
//          blocked_off++;               /* inside the minimum on window */
//#endif
//        }
//        else
//#endif
//        {
//          PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//          pb5_high       = 0U;
//#if STATS_ENABLE
//          on_ticks      += held;
//          edges++;
//#endif
//          last_edge_tick = now;
//#if PD14_ENABLE
//          pd14_push();
//#endif
//        }
//      }
//    }
//    else
//    {
//#if DWELL_MAX_ENABLE
//      /* Only out of a HOLD. want < 0 means the input is above the band, and
//         the answer to that is to stay off - forcing on there measured as 127
//         forced edges in one gate while the sensed value ran away past 2.6 V.
//         want > 0 is handled by the branch below, which turns it on anyway. */
//      if ((MAX_TOFF_TICKS != 0U) && (held >= MAX_TOFF_TICKS) && (want == 0))
//      {
//        PWM_PORT->BSRR = (uint32_t)PWM_PIN;
//        pb5_high       = 1U;
//        last_edge_tick = now;
//#if STATS_ENABLE
//        edges++;
//        forced_on++;
//#endif
//#if PD14_ENABLE
//        pd14_push();
//#endif
//      }
//      else
//#endif
//      if (want > 0)
//      {
//#if DWELL_MIN_ENABLE
//        if (held < MIN_TOFF_TICKS)
//        {
//#if STATS_ENABLE
//          blocked_on++;                /* inside the minimum off window */
//#endif
//        }
//        else
//#endif
//        {
//          PWM_PORT->BSRR = (uint32_t)PWM_PIN;
//          pb5_high       = 1U;
//          last_edge_tick = now;
//#if STATS_ENABLE
//          edges++;
//#endif
//#if PD14_ENABLE
//          pd14_push();
//#endif
//        }
//      }
//    }
//
//#if TRACE_ENABLE
//    /* One store, after the decision, so bit 15 is the level this conversion
//       actually produced rather than the one before it. Gated even when the
//       modulator is not (PWM_ACTIVE_IN_GATE 0), so sample 0 of the trace is
//       always the first conversion after PF3 rose and the host's time axis
//       needs no offset. */
//    if (gate_open && (trace_n < TRACE_SAMPLES))
//    {
//      if (++trace_skip >= TRACE_DECIMATE)
//      {
//        trace_skip = 0U;
//        trace_buf[trace_n++] = (hi << 16)
//                             | (pb5_high << 15)
//                             | (a & 0x0FFFU);
//      }
//    }
//#endif
//  }
//
//out:
//  if (gate_open)
//  {
//    /* Counted and timed only inside the gate. Outside it this handler is doing
//       different work - and the first interrupt of the run runs from flash with
//       a cold instruction cache, which would set a worst case several times the
//       steady state and describe nothing. */
//    isr_count++;
//
//#if ISR_PROFILE_ENABLE
//    /* No second TIM2 read: that one APB round trip was a quarter of the
//       handler's bus traffic, spent measuring the handler. */
//    const uint32_t dur = DWT->CYCCNT - c0;
//    if (dur > isr_max_cycles)
//    {
//      isr_max_cycles = dur;
//    }
//#endif
//  }
//}
//
///* ==========================================================================
// * main
// * ========================================================================== */
//
//static void Ctrl_Init(void)
//{
//  ctrl->magic               = CTRL_MAGIC;
//  ctrl->version             = CTRL_VERSION;
//  ctrl->status              = CTRL_STATUS_INIT;
//  ctrl->rev_id              = HAL_GetREVID();
//  ctrl->timer_clk_hz        = TIMER_CLK_HZ;
//  ctrl->adc_clk_hz          = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
//  ctrl->adc_bits            = ADC_RES_BITS;
//  ctrl->vref_mv             = VREF_MV;
//  ctrl->adc_conv_halfcycles = ADC_CONV_HALFCYCLES;
//  ctrl->adc_rate_nom_hz     = 0U;     /* no target rate to compare against */
//  ctrl->adc_rate_meas_hz    = 0U;
//  ctrl->isr_count           = 0U;
//  ctrl->isr_total           = 0U;
//  ctrl->gate_ticks          = 0U;
//  ctrl->isr_max_cycles      = 0U;
//  ctrl->cpu_hz              = HAL_RCC_GetSysClockFreq();
//  ctrl->adc_ovr             = 0U;
//  ctrl->sine_hz             = (SINE_MHZ + 500UL) / 1000UL;
//  ctrl->sine_mhz            = SINE_MHZ;
//  ctrl->ext_trig_enabled    = EXT_TRIG_ENABLE ? 1U : 0U;
//  ctrl->ext_trig_count      = 0U;
//  ctrl->gate_us             = 0U;
//  ctrl->phase_mul           = 0U;
//  ctrl->phase_shift         = 0U;
//  ctrl->tim2_wraps          = 0U;
//  ctrl->relax_flags         = RELAX_FLAGS;
//  ctrl->safe_healthy        = 0U;
//  ctrl->safe_hi_codes       = SAFE_HI_CODES;
//  ctrl->safe_lo_codes       = SAFE_LO_CODES;
//  ctrl->sine_offset_mv      = SINE_OFFSET_MV;
//  ctrl->sine_amp_mv         = SINE_AMP_MV;
//  ctrl->threshold_mv        = THRESHOLD_MV;
//  ctrl->min_ton_ns          = MIN_TON_NS;
//  ctrl->max_ton_ns          = MAX_TON_NS;
//  ctrl->min_toff_ns         = MIN_TOFF_NS;
//  ctrl->max_toff_ns         = MAX_TOFF_NS;
//  ctrl->pd14_delay_ns       = PD14_DELAY_NS;
//  ctrl->pf3_after_pd15_us   = PF3_AFTER_PD15_US;
//  ctrl->pd15_after_pf3_us   = PD15_AFTER_PF3_US;
//  ctrl->edges               = 0U;
//  ctrl->forced_off          = 0U;
//  ctrl->forced_on           = 0U;
//  ctrl->blocked_off         = 0U;
//  ctrl->blocked_on          = 0U;
//  ctrl->pd14_queue_max      = 0U;
//  ctrl->pd14_late           = 0U;
//  ctrl->pd14_dropped        = 0U;
//  ctrl->lut_n               = LUT_N;
//  ctrl->gate_periods_milli  = (uint32_t)(GATE_SINE_PERIODS * 1000.0f);
//  ctrl->on_ticks            = 0U;
//  ctrl->safe_hi_mv          = SAFE_HI_MV;
//  ctrl->safe_lo_mv          = SAFE_LO_MV;
//  ctrl->tripped             = 0U;
//  ctrl->trip_code           = 0U;
//  ctrl->trip_ticks          = 0U;
//#if TRACE_ENABLE
//  ctrl->trace_addr          = (uint32_t)(uintptr_t)trace_buf;
//  ctrl->trace_stride        = 4U;
//  ctrl->trace_decimate      = TRACE_DECIMATE;
//#else
//  ctrl->trace_addr          = 0U;
//  ctrl->trace_stride        = 0U;
//  ctrl->trace_decimate      = 0U;
//#endif
//  ctrl->trace_count         = 0U;
//}
//
///** @param gate_ticks the MEASURED length of the PF3 window, in TIM2 ticks. */
//static void Ctrl_Publish(uint64_t gate_ticks)
//{
//  ctrl->isr_count        = isr_count;
//  ctrl->isr_total        = isr_total;
//  ctrl->isr_max_cycles   = isr_max_cycles;
//  ctrl->ext_trig_count   = ext_trig_count;
//#if NEED_64BIT_TIME
//  ctrl->tim2_wraps       = tim2_hi;
//#endif
//  ctrl->phase_mul        = phase_mul;
//  ctrl->phase_shift      = phase_shift;
//  ctrl->gate_us          = (uint32_t)(gate_ticks
//                                      / (TIMER_CLK_HZ / 1000000UL));
//  ctrl->edges            = edges;
//  ctrl->forced_off       = forced_off;
//  ctrl->forced_on        = forced_on;
//  ctrl->blocked_off      = blocked_off;
//  ctrl->blocked_on       = blocked_on;
//  ctrl->pd14_queue_max   = pd14_queue_max;
//  ctrl->pd14_late        = pd14_late;
//  ctrl->pd14_dropped     = pd14_dropped;
//  ctrl->on_ticks         = on_ticks;
//  ctrl->adc_ovr          = (ADC1->ISR & ADC_ISR_OVR) ? 1U : 0U;
//  ctrl->tripped          = tripped;
//  ctrl->trip_code        = trip_code;
//#if SAFE_HEALTHY_COUNT
//  ctrl->safe_healthy     = safe_healthy;
//#else
//  ctrl->safe_healthy     = safe_arm;     /* armed is live in this build */
//#endif
//  ctrl->trip_ticks       = (uint32_t)trip_ticks;
//#if TRACE_ENABLE
//  ctrl->trace_count      = trace_n;
//#endif
//
//  if (gate_ticks)
//  {
//    ctrl->gate_ticks       = (uint32_t)gate_ticks;   /* see gate_us */
//    ctrl->adc_rate_meas_hz =
//        (uint32_t)(((uint64_t)isr_count * (uint64_t)TIMER_CLK_HZ) / gate_ticks);
//  }
//}
//
//int main(void)
//{
//  Mpu_Config();                 /* before the D-cache: the status block must
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
//  /* Before anything that wants to time itself. */
//  (void)Dwt_Start();
//
//  AdcClock_Config();
//  Gpio_Config();
//  Timebase_Config();
//#if PD14_ENABLE
//  Delay_Timer_Config();
//#endif
//  Ctrl_Init();
//  Lut_Build();
//  Adc_Config();
//
//  /* The DDS scaling, once, from the configured frequency. */
//  Dds_Scale(SINE_MHZ, &phase_mul, &phase_shift);
//
//  /* PB5 low, PD14 low. Tie them together before anything can move. */
//  last_edge_tick = TIM2->CNT;
//  pb5_high       = 0U;
//#if PD14_ENABLE
//  Pd14_Arm();
//#endif
//
//  /* Park the sequence before the first interrupt can see it. */
//  seq_state    = SEQ_IDLE;
//  seq_deadline = (tick_t)TIM2->CNT + SEQ_NEVER_TICKS;
//
//  /* The requested window, computed once here rather than in the handler. */
//  gate_ticks_cfg = GATE_TICKS;
//
//#if !NEED_64BIT_TIME
//  /* A long gate at a high frequency - two hundred periods at 100 Hz, say -
//     slips past the frequency test that chose the 32-bit timebase. Refuse to
//     run rather than wrap the deadline mid-gate, and say why: a silent wrap
//     would look like the sequence simply never finishing. */
//  if (gate_ticks_cfg > 0x40000000ULL)
//  {
//    ctrl->status = CTRL_STATUS_BAD_GATE;
//    Error_Handler();
//  }
//#endif
//
//  /* The converter runs from here; the handler starts firing immediately and
//     does nothing but service PD14 until the gate opens. */
//  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
//  if (HAL_ADC_Start(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* ---- wait for the external start ------------------------------------
//     Everything above is preparation and none of it drives anything; the run
//     properly begins on the line that raises PD15. So this is where the edge
//     belongs: the board is fully armed, and the instant PB8 rises PD15 follows
//     within one turn of this loop. */
//#if EXT_TRIG_ENABLE
//  ctrl->status   = CTRL_STATUS_WAIT_TRIG;
//  ext_trig_fired = 0U;            /* discard anything that arrived earlier */
//  {
//#if EXT_TRIG_TIMEOUT_MS
//    /* Timed in MILLISECONDS off SysTick, not in timer ticks. At 240 MHz a
//       32-bit tick count tops out at 17.9 s, so the obvious
//       NS_TO_TICKS(30000 ms) silently truncated to about 6.8 s - a timeout
//       that fired early and looked like a missed trigger. */
//    const uint32_t t_wait = HAL_GetTick();
//#endif
//    /* EITHER the pin is already high, OR an edge was caught while we looked
//       away. The level test is what makes a supply left at 3.3 V work on every
//       reset; the edge flag is what catches a pulse too short to poll. */
//    while (!ext_trig_fired
//           && ((TRIG_PORT->IDR & (uint32_t)TRIG_PIN) == 0U))
//    {
//      ctrl->ext_trig_count = ext_trig_count;
//#if EXT_TRIG_TIMEOUT_MS
//      if ((HAL_GetTick() - t_wait) >= EXT_TRIG_TIMEOUT_MS)
//      {
//        /* Say so rather than sit silent. A board that blocks forever on an
//           edge that never comes looks exactly like a crashed one from the
//           outside, and the two want completely different responses. */
//        ctrl->ext_trig_count = ext_trig_count;
//        ctrl->status         = CTRL_STATUS_NO_TRIG;
//        while (1)
//        {
//          __NOP();
//        }
//      }
//#endif
//    }
//  }
//#endif
//
//  /* ---- startup-circuit enable: PD15, then hand over to the interrupt ----
//     PD15's own rising edge is the one thing main still places, and nothing is
//     measured from it by anything except the PF3 deadline below - so a few
//     hundred nanoseconds of imprecision here costs nothing. Every edge after
//     it is the handler's. */
//  ctrl->status = CTRL_STATUS_WAIT_GATE;
//
//#if NEED_64BIT_TIME
//  const tick_t t_en1 = ((uint64_t)tim2_hi << 32) | (uint64_t)TIM2->CNT;
//#else
//  const tick_t t_en1 = TIM2->CNT;
//#endif
//  EN1_PORT->BSRR = (uint32_t)EN1_PIN;
//
//#if !SAFE_ARM_IN_GATE
//#if SAFE_HEALTHY_COUNT
//  safe_healthy  = 0U;           /* prove the input before trusting it */
//  safe_good     = 0U;
//#endif
//  safe_arm_tick = t_en1;        /* reference first, then arm */
//  safe_arm      = 1U;
//#endif
//
//  seq_deadline = t_en1 + PF3_DELAY_TICKS;   /* deadline first, then state */
//  seq_state    = SEQ_WAIT_PF3;
//
//  /* ---- wait for the handler to finish the sequence -----------------------
//     main is now irrelevant to the timing: however long these iterations take,
//     PF3 and PD15 have already been placed by the interrupt. */
//  while (seq_state != SEQ_DONE)
//  {
//    if (gate_open)
//    {
//      ctrl->status = CTRL_STATUS_RUNNING;
//    }
//  }
//
//  /* PD14's last edge is still in flight in TIM4, one delay behind PB5. */
//  const uint32_t settle = TIM2->CNT;
//  while ((TIM2->CNT - settle) < NS_TO_TICKS(4UL * PD14_DELAY_NS))
//  {
//    /* spin */
//  }
//
//  Ctrl_Publish(gate_meas_ticks);
//  if (!tripped)
//  {
//    ctrl->status = CTRL_STATUS_DONE;
//  }
//  else if (trip_code == TRIP_CODE_UNHEALTHY)
//  {
//    /* Distinct from an ordinary trip: nothing went out of range, the input
//       never came INTO range, so the fault is in the sense path and not in the
//       power stage. */
//    ctrl->status = CTRL_STATUS_UNHEALTHY;
//  }
//  else
//  {
//    ctrl->status = CTRL_STATUS_TRIPPED;
//  }
//
//  /* Both enables are low and the run is over. The converter and its handler
//     stay running so the PD14 queue is always serviced and the debug unit
//     always reachable; a busy loop rather than __WFI() for the same reason. */
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
