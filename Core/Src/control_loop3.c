//
///*
// * control_loop3..c
// *
// *  Created on: Sep 12, 2026
// *      Author: shivam
// */
//
//
///**
//  ******************************************************************************
//  * @file    adc_sine_pwm_eoc.c
//  * @brief   Single ADC1, 12-bit, continuous conversion at its maximum rate.
//  *          One decision per END-OF-CONVERSION INTERRUPT against a
//  *          timer-derived DDS sine, with minimum and maximum on/off times, a
//  *          constant-delay copy on PD14, and PD15/PF3 enables.
//  *          NUCLEO-H753ZI.
//  *
//  * THE PARAMETERS ARE RUNTIME, AND THE RUN IS A PLAYLIST
//  * ------------------------------------------------------
//  * The sine frequency, amplitude, offset, threshold, the four dwell limits,
//  * the safety window and the PD14 delay are no longer compile-time constants
//  * in the decision path. They live in variables that main loads before each
//  * SEGMENT, and a segment is one PF3 window run at one operating point.
//  *
//  * A RUN is a list of segments executed back to back inside a single PD15
//  * window:
//  *
//  *     PD15 high
//  *       -> +PF3_AFTER_PD15_US
//  *       for each segment:
//  *            load its parameters, rebuild the window LUT
//  *            PF3 high  -> its own periods at its own frequency -> PF3 low
//  *            -> +its own gap_us
//  *       -> +PD15_AFTER_PF3_US
//  *     PD15 low
//  *
//  * PD15 stays high for the whole playlist because it enables the startup
//  * circuit, which has no reason to be cycled between operating points. PF3 is
//  * cycled per segment because it enables the GATE DRIVER, and each segment is
//  * a separate operating point that deserves a clean start: the DDS phase
//  * reference is the PF3 rising edge, so every segment holds whole sine cycles
//  * counted from phase zero, exactly as a single-segment run did.
//  *
//  * THE PARAMETERS CAN BE CHANGED WITHOUT RECOMPILING
//  * -------------------------------------------------
//  * A COMMAND BLOCK sits at 0x30000100, in the same non-cacheable SRAM1 window
//  * as the status block. A host writes a playlist into it over SWD - the
//  * ST-LINK can write the memory of a RUNNING target in hotplug mode, with no
//  * reset and no reflash - bumps cmd->seq, and main picks it up and runs it.
//  * pwm_live.py and pwm_sweep.ipynb do exactly that, which is what makes a
//  * parameter sweep a loop in a notebook instead of a rebuild per point.
//  *
//  * A run is triggered by TWO conditions together - cmd->magic present AND
//  * cmd->seq changed - and the host writes SEQ LAST. That ordering is what
//  * makes a half-written block harmless: every parameter is already in place
//  * before the one word that arms the run is touched, so there is no moment at
//  * which main can start a playlist that is half this request and half the
//  * last one. The magic alone would not do it, because after the first command
//  * the magic is already there and seq is the only thing left that changes.
//  *
//  * A compiled-in DEFAULT PLAYLIST runs once at power-on, so the board sweeps
//  * on its own with no host attached - which is what a video demonstration
//  * needs. After it finishes, main idles waiting for a command.
//  *
//  * WHY THE LUT IS REBUILT WITH THE EOC INTERRUPT MASKED
//  * ----------------------------------------------------
//  * Changing the amplitude or the threshold means rebuilding all LUT_N window
//  * entries, about 330k cycles of float work. main cannot do that at a
//  * predictable speed while an interrupt fires every ~320 ns and takes most of
//  * the CPU - main's share is 1 - (handler / conversion period), and it is
//  * small. So the rebuild masks ADC_IT_EOC, runs at full speed, clears the
//  * overrun the mask caused, and unmasks. It is only ever done BETWEEN
//  * segments, with PF3 low and PB5 already idle low, and only after the PD14
//  * queue has been given time to drain - so nothing is in flight while the
//  * handler is off, and the gap between segments comes out the same every time
//  * instead of depending on how starved main happens to be.
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
//  * THE FREQUENCY IS IN MILLIHERTZ, AND THE TIMEBASE IS 64 BITS
//  * ------------------------------------------------------------
//  * Frequency is carried as an integer number of MILLIHERTZ, so 1234.567 Hz
//  * and 0.005 Hz are both expressible and the resolution is the same fraction
//  * of a hertz everywhere. Two things had to change to make the low end real
//  * rather than nominal.
//  *
//  * TIM2 is 32 bits at 240 MHz, so it WRAPS EVERY 17.9 SECONDS. One period at
//  * 1 mHz is a thousand seconds - fifty-five wraps. Any phase computed from a
//  * bare 32-bit tick difference is therefore meaningless below about 0.06 Hz.
//  * The handler now carries a wrap counter: it reads the timer every few
//  * hundred nanoseconds so it cannot miss an overflow, and one compare per
//  * interrupt extends the timebase to 64 bits. That is the whole cost, and it
//  * buys five decades.
//  *
//  * The phase step also had to change scale. It used to be a Q8 fraction of a
//  * phase unit per tick; at 1 mHz the true value is 0.0179, which Q8 rounds to
//  * 4/256 - a 13% frequency error. The step is now (mul >> shift) with BOTH
//  * chosen per segment, so the mantissa always lands in the top bits of a
//  * 32-bit word whatever the frequency. Measured against exact arithmetic
//  * across nine decades, from 1 mHz to 900 kHz, the worst frequency error is
//  * 8 parts in 10^10 and the 64-bit product never exceeds 8.9e18 against a
//  * limit of 1.8e19.
//  *
//  * The phase reference is RE-ANCHORED whenever the tick difference reaches
//  * 2^32, which is what keeps that product in range: without it a thousand-
//  * second gate at a kilohertz would overflow. Re-anchoring costs one branch
//  * that is taken once every 17.9 seconds.
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
//  * PB8 - THE EXTERNAL START
//  * ------------------------
//  * A rising edge on PB8 starts a run. Everything downstream is unchanged: the
//  * edge is simply what PD15's rise waits for, and PD15 -> PF3 -> the segments
//  * -> PD15 low then proceed exactly as they did when main raised PD15 itself.
//  *
//  * A run is ARMED by a host command and FIRED by the edge, and the two are
//  * deliberately separate. Arming is slow - several seconds of programmer
//  * traffic to write the command block - while the run itself is milliseconds,
//  * and nothing downstream should have to care about that asymmetry. Separating
//  * them also makes the start instant knowable to something other than the
//  * firmware, which is what an instrument wants: whatever generates the edge
//  * can trigger a scope from the same signal and know exactly where the
//  * excitation begins, instead of arming a capture and hoping.
//  *
//  * The pin is configured here, with a PULL-DOWN and rising-edge detection, so
//  * a disconnected input sits low and cannot fire. Remove PB8 from the CubeMX
//  * GPIO init - this file owns it.
//  *
//  * EXT_TRIG_ENABLE is the compiled-in default and CMD_FLAG_EXT_TRIG is the
//  * per-command override, so a host can choose per run whether to wait. With
//  * waiting off the behaviour is exactly as before: main raises PD15 as soon as
//  * the command is adopted.
//  *
//  * The wait has a timeout - EXT_TRIG_TIMEOUT_MS - because a run that blocks
//  * forever on an edge that never comes is indistinguishable, from the host,
//  * from a target that has crashed. On timeout the run is abandoned and the
//  * status block says so rather than the board sitting silent.
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
//  *            PLL2P, PB5, PD14, PD15, PF3, MPU region 0, 2 KB at the base of
//  *            SRAM1, 4 * LUT_N bytes of DTCM.
//  *
//  * THE SRAM1 MAP
//  * -------------
//  *     0x30000000   512 B    status block    (ctrl_blk_t)    target -> host
//  *     0x30000200   576 B    command block   (cmd_blk_t)     host   -> target
//  *     0x30000600  1280 B    segment results (seg_res_t[8])  target -> host
//  *
//  * The two upper blocks moved once, when the status block outgrew 256 bytes.
//  * Nothing should ever hard-code their addresses: the status block publishes
//  * cmd_addr and res_addr, and the host tools read them from there, so a future
//  * move costs a recompile and nothing else.
//  *
//  * All three are inside MPU region 0, so none of them is cached and the host
//  * reads what the CPU actually wrote.
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
///* EVERY NUMBER IN THIS SECTION IS A DEFAULT, NOT A FIXED SETTING. It seeds
//   the built-in playlist below and is what a segment falls back to when the
//   host leaves a field at zero. The decision path reads the rt_* variables
//   that main loads per segment, never these. */
//
///* The sine, in millivolts about the ADC's own scale. The sine spans
//   SINE_OFFSET_MV +/- SINE_AMP_MV; the comparison limits are that plus and
//   minus THRESHOLD_MV. */
//#define SINE_HZ                 2000UL
//#define SINE_OFFSET_MV          1500
//#define SINE_AMP_MV             200
//#define THRESHOLD_MV            160
//
///* Dwell limits, nanoseconds, measured from the last PB5 edge. Set either
//   maximum to 0 to remove that ceiling. */
//#define MIN_TON_NS              300UL
//#define MAX_TON_NS              4000UL
//#define MIN_TOFF_NS             500UL
//#define MAX_TOFF_NS             4000UL
//
///* Safety trip. Outside this window, PF3 drops and latches until reset. */
//#define SAFE_HI_MV              2600
//#define SAFE_LO_MV              0
//
///* 1 = police the input only while PF3 is high (see the header on why an
//   undriven PC0 would otherwise trip before PF3 ever rose).
//   0 = police from PD15, i.e. the whole run. */
//#define SAFE_ARM_IN_GATE        1
//
///* Ignore the input this long after arming, for a startup transient. */
//#define SAFE_BLANK_US           0UL
//
///* Consecutive out-of-range conversions needed to trip. 1 trips on the first. */
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
///* 1 = a run waits for a rising edge on PB8 before PD15 goes high.
//   0 = PD15 rises as soon as the run is armed, which is the older behaviour. */
//#define EXT_TRIG_ENABLE         1
//
///* How long a run will wait for that edge before giving up, milliseconds.
//   0 waits forever. */
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
///* ==========================================================================
// * The built-in playlist
// * ==========================================================================
// * One row per segment: sine frequency in Hz, amplitude in mV, threshold in mV,
// * sine periods, and the dead time after the segment in microseconds.
// *
// * The default is the frequency/amplitude sweep: the amplitude is raised with
// * the frequency because at a low frequency the current has far longer to run
// * away between the sine's own turning points, so the same amplitude that is
// * comfortable at 5 kHz will hit the safety trip at 100 Hz.
// *
// * THE THRESHOLD IS NOT SCALED WITH THE AMPLITUDE HERE. At 50 mV amplitude and
// * a 160 mV threshold the hysteresis band is more than three times the sine, so
// * the reference is nearly irrelevant and the output is a free-running ripple
// * oscillator. That may be exactly what you want to show, or it may not - the
// * per-segment threshold column is there so it can be changed without touching
// * anything else, and pwm_live.py can change it between runs with no rebuild.
// *
// * PLAYLIST_LOOP 1 repeats the playlist forever, which is what a continuously
// * running video demonstration wants. At 0 it runs once and then idles waiting
// * for a host command.
// */
//#define PLAYLIST_LOOP           0
//
///*                  Hz      amp   thr   periods   gap_us */
//#define PLAYLIST_ROWS \
//    X(  100UL,       50,   160,     5.0f,   2000UL ) \
//    X(  500UL,      100,   160,     5.0f,   2000UL ) \
//    X( 1000UL,      300,   160,     5.0f,   2000UL ) \
//    X( 5000UL,      500,   160,    25.0f,   2000UL )
//
///* The most segments a playlist can hold, built-in or host-supplied. Sets the
//   size of the command block and the result table. */
//#define MAX_SEG                 8U
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
///* The trace. 4 bytes per recorded conversion, in DTCM. The buffer is SPLIT
//   EVENLY between the segments of a run, so a four-segment playlist gives each
//   segment 2048 words.
//
//   TRACE_DECIMATE is the fallback when a segment does not ask for one. At 0 -
//   the default for every built-in row - the decimation is CHOSEN PER SEGMENT so
//   that the segment's slice spans its whole gate: the gate length is known, the
//   conversion rate is MEASURED once at startup by Rate_Probe(), and
//
//       decimate = ceil(gate_ticks * rate / (TIMER_CLK_HZ * slice))
//
//   is how many conversions have to be skipped to make the two meet. Without it
//   a 100 Hz segment - a 50 ms gate - would fill its slice in the first 0.7 ms
//   and the capture would show a fortieth of one sine period. */
//#define TRACE_ENABLE            1
//#define TRACE_SAMPLES           8192U
//#define TRACE_DECIMATE          0U
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
///* These are the DEFAULT tick counts. main converts each segment's own
//   nanoseconds at load time with ns_to_ticks(); the handler reads the rt_*
//   variables, never these. They are kept because the static asserts below
//   still check that the defaults themselves are sane. */
//#define MIN_TON_TICKS           NS_TO_TICKS(MIN_TON_NS)
//#define MAX_TON_TICKS           NS_TO_TICKS(MAX_TON_NS)
//#define MIN_TOFF_TICKS          NS_TO_TICKS(MIN_TOFF_NS)
//#define MAX_TOFF_TICKS          NS_TO_TICKS(MAX_TOFF_NS)
//#define PD14_DELAY_TICKS        NS_TO_TICKS(PD14_DELAY_NS)
//#define SAFE_BLANK_TICKS        NS_TO_TICKS((uint64_t)SAFE_BLANK_US * 1000ULL)
//#define PF3_DELAY_TICKS         NS_TO_TICKS((uint64_t)PF3_AFTER_PD15_US * 1000ULL)
//#define PD15_HOLD_TICKS         NS_TO_TICKS((uint64_t)PD15_AFTER_PF3_US * 1000ULL)
//
///* The highest sine frequency the Q8 tick scaling can express, and the longest
//   gate a signed deadline compare can carry. Both are checked per segment at
//   load time, because a host can write any number it likes into the command
//   block and a rejected segment has to say why rather than misbehave. */
//#define SINE_MHZ_MAX            900000000UL   /* 900 kHz, in millihertz   */
//#define SINE_MHZ_MIN            1UL           /* 1 mHz                    */
//
///* The longest gate a 64-bit tick deadline will carry. 2^42 ticks at 240 MHz
//   is about five hours - five periods at the 1 mHz floor is 5000 seconds, so
//   there is an order of magnitude in hand - and it is deliberately well below
//   SEQ_NEVER_TICKS, the value the deadline is parked at when nothing is due,
//   so a legal gate can never be confused with an idle one. */
//#define GATE_TICKS_MAX          (SEQ_NEVER_TICKS / 4U)
//
///* Phase per TIM2 tick, Q8. The Q8 fraction is what makes the frequency exact:
//   at Q0 the per-tick advance would round to a whole phase step and the
//   frequency error would be parts in 10^4. */
///* The default frequency in millihertz. SINE_HZ stays a whole number because
//   it is the obvious thing to edit; a fractional default is expressed here. */
//#define SINE_MHZ_DEFAULT        (SINE_HZ * 1000UL)
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
///* TRACE_DECIMATE 0 is legal now and means "choose it from the measured rate",
//   so there is nothing left to assert about it here. seg_validate() checks the
//   per-segment value at load time instead, because a host can write any number
//   into the command block and the answer has to be a rejection with a reason
//   rather than a compile that never happens. */
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
//_Static_assert(SINE_MHZ_DEFAULT >= SINE_MHZ_MIN
//               && SINE_MHZ_DEFAULT <= SINE_MHZ_MAX,
//               "the default SINE_HZ is outside the 1 mHz .. 900 kHz range");
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
//_Static_assert(PD14_DELAY_TICKS + PD14_MIN_AHEAD < 65536UL,
//               "the DEFAULT PD14_DELAY_NS exceeds the 16-bit TIM4 range at "
//               "240 MHz (273 us) - use TIM2 or TIM5 for a longer delay. A "
//               "host-supplied delay is checked at load time and rejected with "
//               "REJ_PD14_RANGE.");
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
//  uint32_t cmd_bad;             /* run-wide command fields out of range:
//                                   bit0 PD14 delay, bit1 PF3 delay, bit2
//                                   PD15 hold. Each named bit means the host
//                                   sent something impossible and the compiled
//                                   default was used instead. This word used to
//                                   be adc_rate_nom_hz, which was always 0 -
//                                   there is no target conversion rate in this
//                                   design - so the index is reused rather than
//                                   moving every field after it.            */
//  uint32_t adc_rate_meas_hz;    /* MEASURED: interrupts / gate duration    */
//  uint32_t isr_count;           /* interrupts INSIDE the gate              */
//  uint32_t gate_ticks;          /* MEASURED length of the PF3 window       */
//  uint32_t isr_max_ticks;       /* superseded by isr_max_cycles, kept 0    */
//  uint32_t adc_ovr;             /* 1 if the ADC ever overran               */
//  /* From here to on_ticks, these describe the LAST SEGMENT THAT RAN. The
//     per-segment table at res_addr carries all of them for every segment; these
//     are kept so that a one-segment playlist reads exactly as it did before and
//     ctrl_status.py still works unchanged. */
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
//  /* ---- the playlist ---------------------------------------------------- */
//  uint32_t cmd_addr;            /* where the host writes a playlist        */
//  uint32_t res_addr;            /* where the per-segment results land      */
//  uint32_t max_seg;             /* how many segments either block holds    */
//  uint32_t seg_count;           /* segments in the playlist now running    */
//  uint32_t seg_index;           /* the segment running, or the last one    */
//  uint32_t seg_done;            /* segments that actually completed        */
//  uint32_t run_seq;             /* the cmd->seq this run came from, 0 =
//                                   the compiled-in default playlist        */
//  uint32_t rate_probe_hz;       /* MEASURED before the run, sizes the trace*/
//  uint32_t trace_samples;       /* words in the whole trace buffer         */
//  /* ---- live, republished while main waits ------------------------------
//     Everything above is written at the END of a run or a segment. These four
//     are written while main is spinning, so a target that is stuck says WHERE:
//     which state the sequence is in, whether the gate is open, how long the
//     gate it is waiting on is meant to be, and - the decisive one - whether the
//     conversion interrupt is running at all. A hang with seq_state at
//     SEQ_WAIT_PF3 and isr_live frozen is a dead handler; the same hang with
//     isr_live climbing is a deadline that was never armed. */
//  uint32_t seq_state;
//  uint32_t gate_open;
//  uint32_t gate_ticks_cfg;      /* the gate main is waiting for, in ticks  */
//  uint32_t isr_live;            /* interrupts since reset, RIGHT NOW       */
//  /* ---- added with fractional frequency and the external trigger --------- */
//  uint32_t sine_mhz;            /* the last segment's frequency, MILLIHERTZ
//                                   - sine_hz above is it rounded to whole Hz
//                                   and is kept only so older host scripts
//                                   still read something sensible            */
//  uint32_t gate_us;             /* MEASURED gate length in microseconds. The
//                                   tick count above is 32 bits and a
//                                   millihertz gate is minutes long, so this
//                                   is the field to trust for anything slower
//                                   than about ten seconds                   */
//  uint32_t ext_trig_enabled;    /* 1 = a run waits for a PB8 edge           */
//  uint32_t ext_trig_count;      /* PB8 edges seen since reset               */
//  uint32_t ext_trig_waits;      /* runs that actually waited for one        */
//  uint32_t tim2_wraps;          /* 32-bit timer overflows since reset - the
//                                   high half of the 64-bit timebase         */
//  uint32_t phase_mul;           /* the DDS scaling in force, for diagnosis  */
//  uint32_t phase_shift;
//  uint32_t reserved[56];
//} ctrl_blk_t;
//
//_Static_assert(sizeof(ctrl_blk_t) == 512U, "status block must stay 512 bytes");
//
//static volatile ctrl_blk_t * const ctrl = (volatile ctrl_blk_t *)CTRL_ADDR;
//
///* ==========================================================================
// * The command block - host to target
// * ==========================================================================
// * One playlist, written by the host over SWD while the target runs. Every
// * field is a plain uint32 so the host can build the block with struct.pack and
// * push it with a single STM32_Programmer_CLI write; nothing here is a float,
// * because a float would have to agree bit for bit between two toolchains for
// * no benefit - periods are carried as thousandths and microvolts are not
// * needed anywhere.
// *
// * A ZERO FIELD MEANS "USE THE COMPILED-IN DEFAULT". That is what makes the
// * block usable from a notebook: setting the frequency and the amplitude is two
// * assignments, and the eleven fields nobody wants to think about stay at the
// * values that are known to work. It also means a host cannot accidentally ask
// * for a zero minimum off time by forgetting a field.
// *
// * The host writes every other word first and SEQ LAST - see the file header.
// */
//#define CMD_ADDR                0x30000200UL
//#define CMD_MAGIC               0xC0DEFEEDUL
//
//#define CMD_FLAG_CLEAR_TRIP     0x00000001UL  /* re-arm after a safety trip */
//#define CMD_FLAG_LOOP           0x00000002UL  /* repeat until the next cmd  */
//#define CMD_FLAG_EXT_TRIG       0x00000004UL  /* wait for a PB8 edge        */
//#define CMD_FLAG_NO_EXT_TRIG    0x00000008UL  /* do not, whatever the
//                                                 compiled default says      */
//
//typedef struct
//{
//  uint32_t sine_hz;
//  uint32_t sine_offset_mv;
//  uint32_t sine_amp_mv;
//  uint32_t threshold_mv;
//  uint32_t min_ton_ns;
//  uint32_t max_ton_ns;        /* 0xFFFFFFFF disables the ceiling, not 0 -
//                                 0 means "use the default" everywhere here  */
//  uint32_t min_toff_ns;
//  uint32_t max_toff_ns;       /* likewise                                   */
//  uint32_t periods_milli;     /* sine periods x 1000                        */
//  uint32_t gap_us;            /* dead time after this segment               */
//  uint32_t safe_hi_mv;
//  uint32_t safe_lo_mv;        /* 0xFFFFFFFF means a floor of 0 mV           */
//  uint32_t safe_blank_us;     /* 0xFFFFFFFF means no blanking               */
//  uint32_t safe_trip_count;
//  uint32_t trace_decimate;    /* 0 = choose it from the measured rate       */
//  uint32_t sine_mhz;          /* frequency in MILLIHERTZ. When non-zero this
//                                 is authoritative and sine_hz above is
//                                 ignored; when zero the whole-hertz field is
//                                 used instead. Two fields rather than a
//                                 changed meaning for one, so a host written
//                                 against the older block still works and a
//                                 fractional frequency is impossible to
//                                 request by accident.                       */
//} seg_cfg_t;
//
//_Static_assert(sizeof(seg_cfg_t) == 64U, "seg_cfg_t must stay 64 bytes");
//
//typedef struct
//{
//  uint32_t magic;             /* CMD_MAGIC - written LAST by the host       */
//  uint32_t seq;               /* bumped to request a run                    */
//  uint32_t n_seg;             /* 1..MAX_SEG                                 */
//  uint32_t flags;             /* CMD_FLAG_*                                 */
//  uint32_t pd14_delay_ns;
//  uint32_t pf3_after_pd15_us;
//  uint32_t pd15_after_pf3_us;
//  uint32_t reserved[9];
//  seg_cfg_t seg[MAX_SEG];
//} cmd_blk_t;
//
//_Static_assert(sizeof(cmd_blk_t) == 64U + 64U * MAX_SEG,
//               "cmd_blk_t layout must stay flat and packed - the host builds "
//               "it with struct.pack");
//
//static volatile cmd_blk_t * const cmd = (volatile cmd_blk_t *)CMD_ADDR;
//
///* ==========================================================================
// * The per-segment result table - target to host
// * ==========================================================================
// * The status block still carries the last segment's numbers so that a
// * one-segment playlist reads exactly as it always did. This table carries all
// * of them for every segment, including the slice of the trace buffer each
// * segment occupies, so the host can cut the single trace buffer into per
// * segment captures without guessing where the boundaries are.
// *
// * reject_reason is the whole point of running a host-supplied playlist: a
// * segment that asks for something impossible is SKIPPED and says which rule it
// * broke, rather than being silently clamped into something that looks like it
// * worked.
// */
//#define RES_ADDR                0x30000600UL
//
//#define SEG_NOT_RUN             0U
//#define SEG_RAN                 1U
//#define SEG_REJECTED            2U
//
//#define REJ_NONE                0U
//#define REJ_FREQ                1U   /* 0, or above SINE_HZ_MAX             */
//#define REJ_BAND_HI             2U   /* band's top is outside the safety
//                                        window, so it would trip at once    */
//#define REJ_BAND_LO             3U
//#define REJ_DWELL               4U   /* a maximum at or below its minimum   */
//#define REJ_PERIODS             5U
//#define REJ_GATE_LONG           6U   /* gate longer than a deadline can hold*/
//#define REJ_PD14_QUEUE          7U   /* delay/dwell needs a longer queue    */
//#define REJ_PD14_RANGE          8U   /* delay beyond TIM4's 16-bit range    */
//
//typedef struct
//{
//  uint32_t valid;             /* SEG_NOT_RUN / SEG_RAN / SEG_REJECTED       */
//  uint32_t reject_reason;     /* REJ_*                                      */
//  uint32_t sine_hz;
//  uint32_t sine_amp_mv;
//  uint32_t sine_offset_mv;
//  uint32_t threshold_mv;
//  uint32_t min_ton_ns;
//  uint32_t max_ton_ns;
//  uint32_t min_toff_ns;
//  uint32_t max_toff_ns;
//  uint32_t periods_milli;
//  uint32_t gate_ticks;        /* MEASURED length of this PF3 window         */
//  uint32_t isr_count;         /* conversions inside it                      */
//  uint32_t rate_meas_hz;      /* MEASURED, this segment alone               */
//  uint32_t isr_max_cycles;
//  uint32_t adc_ovr;
//  uint32_t edges;
//  uint32_t forced_off;
//  uint32_t forced_on;
//  uint32_t blocked_off;
//  uint32_t blocked_on;
//  uint32_t on_ticks;
//  uint32_t tripped;
//  uint32_t trip_code;
//  uint32_t trip_ticks;
//  uint32_t trace_off;         /* first word of this segment's slice         */
//  uint32_t trace_count;       /* words it actually recorded                 */
//  uint32_t trace_decimate;    /* conversions per recorded word              */
//  uint32_t clamp_hi;
//  uint32_t clamp_lo;
//  uint32_t pd14_late;
//  uint32_t pd14_dropped;
//  /* ---- added with fractional frequency --------------------------------- */
//  uint32_t sine_mhz;            /* what was actually asked for, millihertz  */
//  uint32_t gate_us;             /* MEASURED, microseconds - see ctrl->gate_us*/
//  uint32_t phase_mul;           /* the DDS scaling this segment ran at      */
//  uint32_t phase_shift;
//  uint32_t reserved[4];
//} seg_res_t;
//
//_Static_assert(sizeof(seg_res_t) == 160U, "seg_res_t must stay 160 bytes");
//_Static_assert(RES_ADDR + sizeof(seg_res_t) * MAX_SEG
//               <= CTRL_ADDR + 32U * 1024U,
//               "the result table runs past MPU region 0, so the host would "
//               "read cached stale data - shrink MAX_SEG or enlarge region 0");
//_Static_assert(CMD_ADDR + sizeof(cmd_blk_t) <= RES_ADDR,
//               "the command block overlaps the result table");
//_Static_assert(CTRL_ADDR + sizeof(ctrl_blk_t) <= CMD_ADDR,
//               "the status block overlaps the command block");
//
//static volatile seg_res_t * const res = (volatile seg_res_t *)RES_ADDR;
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
//static TIM_HandleTypeDef htim4;
//
///* Written by main before the gate opens, read by the interrupt. */
//static volatile uint64_t phase_ref;   /* 64-bit ticks at the gate's opening */
//static volatile uint32_t phase_base;  /* phase already accumulated at it    */
//static volatile uint32_t gate_open;
//
///* The 64-bit timebase. tim2_hi counts TIM2 overflows; the handler reads the
//   timer far more often than it wraps, so a single compare cannot miss one.
//   Both are owned by the handler and only read by main. */
//static volatile uint32_t tim2_hi;
//static uint32_t          tim2_last;
//
///* ---- THE SEGMENT'S PARAMETERS, AS THE HANDLER SEES THEM ----------------
//   These are the only form of the parameters the decision path ever touches.
//   main writes them with the gate closed and the EOC interrupt masked, so the
//   handler can never see half of one segment's numbers and half of another's.
//   They are in .bss, so DTCM: a load costs the handler the same as the
//   immediate it replaces did, plus the load itself - zero wait states, and it
//   is what buys a parameter sweep with no rebuild. */
//static uint32_t rt_min_ton, rt_max_ton, rt_min_toff, rt_max_toff;
///* The DDS step, as (mul >> shift) phase units per tick. Both are chosen per
//   segment by Dds_Scale() so that mul always fills the top of a 32-bit word,
//   which is what lets one representation cover millihertz and kilohertz. */
//static uint32_t rt_phase_mul;
//static uint32_t rt_phase_shift;
//static uint32_t rt_safe_hi, rt_safe_lo;      /* ADC codes                  */
//static uint32_t rt_safe_blank;        /* ticks                             */
//static uint32_t rt_trip_count;
//static uint32_t rt_pd14_delay;        /* ticks                             */
//static uint32_t rt_trace_base;        /* first word of this segment's slice*/
//static uint32_t rt_trace_cap;         /* words it may use                  */
//static uint32_t rt_trace_dec;         /* conversions per recorded word     */
//
///* The external start. ext_trig_fired is set by the EXTI handler and cleared
//   by main just before it arms, so an edge that arrived while nobody was
//   waiting cannot fire the next run. ext_trig_count keeps counting regardless,
//   which is what tells a host whether its pulse is reaching the pin at all. */
//static volatile uint32_t ext_trig_fired;
//static volatile uint32_t ext_trig_count;
//static volatile uint64_t ext_trig_tick;
//static uint32_t          ext_trig_waits;
//static uint32_t          run_ext_trig;      /* is THIS run waiting?         */
//
///* The playlist main is working through. */
//static seg_cfg_t playlist[MAX_SEG];
//static uint32_t  playlist_n;
//static uint32_t  playlist_flags;
//static uint32_t  run_seq;             /* the cmd->seq this run came from   */
//static uint32_t  rt_pf3_delay, rt_pd15_hold;   /* ticks                    */
//static uint32_t  rate_probe_hz;       /* MEASURED once, sizes the trace    */
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
///* The gate has closed and the playlist has not finished. The handler parks
//   here and does nothing further; main notices, records the segment, loads the
//   next one and arms SEQ_WAIT_PF3 again. PD15 is NOT dropped - it enables the
//   startup circuit and stays up for the whole playlist. */
//#define SEQ_SEG_DONE            5U
//
///* Parked this far ahead when no edge is pending - about 4.5 s at 240 MHz,
//   which the signed compare treats as "not yet" without any special case. */
//#define SEQ_NEVER_TICKS         (1ULL << 44)   /* ~20 hours, never reached */
//
//static volatile uint32_t seq_state;
//static volatile uint64_t seq_deadline;
//static volatile uint64_t gate_meas_ticks;   /* how long PF3 was really up */
//static uint64_t          gate_ticks_cfg;    /* per segment, by main       */
//
///* The safety trip. safe_arm_tick is written before safe_arm, so the interrupt
//   can never see an armed trip with a stale reference. tripped is the latch.
//   Nothing inside a run clears it: once it fires, that segment ends, the rest
//   of the playlist is abandoned and PD15 comes down. The ONLY thing that clears
//   it is an explicit host command carrying CMD_FLAG_CLEAR_TRIP - which is a
//   deliberate act by a person, the same standard a reset was. */
//static volatile uint32_t safe_arm;
//static volatile uint64_t safe_arm_tick;
//static volatile uint32_t tripped;
//static volatile uint32_t trip_code;
//static volatile uint64_t trip_ticks;
//
///* Owned by the interrupt alone. */
//static uint32_t pb5_high;          /* current PB5 level                    */
//static uint32_t last_edge_tick;
///* volatile: written by the handler, read by main when it publishes. */
//static volatile uint32_t isr_total;      /* every interrupt              */
//static volatile uint32_t isr_count;      /* only those inside the gate   */
//static volatile uint32_t isr_max_cycles; /* only those inside the gate   */
///* EVERY COUNTER THE HANDLER WRITES AND main READS BACK MUST BE volatile.
//
//   Not for ordering - for existence. main zeroes these in Seg_Load(), spins
//   waiting for the segment, then reads them in Seg_Record(). The spin loop
//   contains no call the compiler cannot see through, so with a plain static
//   GCC is entitled to conclude nothing changed the variable and fold the read
//   into the zero it just stored. It does exactly that at -O2: the reload
//   disappears and becomes movs r3, #0. Every per-segment figure then comes back
//   zero, including trace_n - which makes the capture look empty rather than
//   wrong, so it does not even present as a compiler problem.
//
//   The single-run version this grew out of never stored a zero for GCC to
//   propagate, which is why the hazard only appeared once runs became repeatable.
//
//   The cost is nothing: the handler loads and stores each of these once per
//   entry either way. */
//static volatile uint32_t edges;
//static volatile uint32_t forced_off, forced_on, blocked_off, blocked_on;
//static volatile uint32_t on_ticks; /* accumulated HIGH time                */
//static uint32_t safe_run;          /* consecutive out-of-range conversions */
//
//#if TRACE_ENABLE
///* .bss, so DTCM. Written by the handler, read by the host through the address
//   published in the status block. */
//static uint32_t trace_buf[TRACE_SAMPLES];
//static volatile uint32_t trace_n;  /* words recorded so far - see above    */
//static uint32_t trace_skip;        /* decimation counter, handler only     */
//#endif
//
///* Whether the trace slice this segment has been given ran out before the gate
//   closed - a decimation that was too optimistic, which the host should see
//   rather than infer from a short capture. */
//static volatile uint32_t trace_full;
//
//static uint16_t pd14_q[PD14_QUEUE_N];
//static uint32_t pd14_head, pd14_tail, pd14_busy;   /* handler only */
//static volatile uint32_t pd14_queue_max, pd14_late, pd14_dropped;
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
//     disconnected pin rests low and only a real drive can fire a run - the
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
//  /* PD14 belongs to TIM4_CH3, AF2 - the timer drives the pin, not the CPU, so
//     writing PD14's BSRR would do nothing. */
//  g.Pin       = DLY_PIN;
//  g.Mode      = GPIO_MODE_AF_PP;
//  g.Pull      = GPIO_NOPULL;
//  g.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//  g.Alternate = GPIO_AF2_TIM4;
//  HAL_GPIO_Init(DLY_PORT, &g);
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
//  * Deliberately does nothing but stamp a flag and a time. Raising PD15 from
//  * here would put the start of a power sequence inside an interrupt that can
//  * preempt nothing useful and would have to be made safe against arriving
//  * mid-run; main is already waiting a few instructions away, and the cost of
//  * letting it do the work is one poll of a volatile word.
//  */
//void EXTI9_5_IRQHandler(void)
//{
//  if (__HAL_GPIO_EXTI_GET_IT(TRIG_PIN) != 0x00U)
//  {
//    __HAL_GPIO_EXTI_CLEAR_IT(TRIG_PIN);
//    ext_trig_tick  = ((uint64_t)tim2_hi << 32) | (uint64_t)TIM2->CNT;
//    ext_trig_count++;
//    ext_trig_fired = 1U;
//  }
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
//static uint32_t lut_clamp_hi, lut_clamp_lo;
//
//static void Lut_Build(uint32_t off_mv, uint32_t amp_mv, uint32_t thr_mv)
//{
//  const float scale = (float)(1U << ADC_RES_BITS) / (float)VREF_MV;
//  const float off_c = (float)off_mv * scale;
//  const float amp_c = (float)amp_mv * scale;
//  const float thr_c = (float)thr_mv * scale;
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
//  /* Kept for the segment record rather than written straight through to the
//     status block: with a playlist these belong to one segment, and the status
//     block's copy is published at the end of the run like everything else. */
//  lut_clamp_hi = nhi;
//  lut_clamp_lo = nlo;
//}
//
///* ==========================================================================
// * PD14: push one delayed edge
// * ========================================================================== */
//
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
//  * @brief Queue a PD14 toggle rt_pd14_delay ticks after now.
//  *
//  * Called from the EOC handler at the same moment PB5 is written, so the delay
//  * is measured from the real PB5 edge. If no compare is pending the value goes
//  * straight into CCR3; otherwise it waits in the queue and the handler arms it
//  * when it sees the previous compare fire.
//  */
//static inline void pd14_push(void)
//{
//  const uint16_t due = (uint16_t)(TIM4->CNT + rt_pd14_delay);
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
//      trace_full     = 0U;
//      break;
//
//    case SEQ_GATE:
//      /* Drop the gate-driver enable and stop there. Whether this was the last
//         segment is main's business, not the handler's: the handler has no
//         idea how long loading the next segment takes, and PD15 must not come
//         down in between. */
//      gate_open       = 0U;
//      EN2_PORT->BSRR  = (uint32_t)EN2_PIN << 16;
//      gate_meas_ticks = now - phase_ref;
//#if SAFE_ARM_IN_GATE
//      /* Armed per gate, so disarmed per gate. NOT disarmed when the trip is
//         armed from PD15 instead: that mode polices the whole run, and the gaps
//         between segments are part of the run. */
//      safe_arm        = 0U;
//#endif
//      seq_deadline    = now + SEQ_NEVER_TICKS;
//      seq_state       = SEQ_SEG_DONE;
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
//  const uint32_t c0  = DWT->CYCCNT;      /* core-private, ~2 cycles */
//  const uint32_t now = TIM2->CNT;
//  const uint32_t a   = ADC1->DR;         /* clears EOC */
//
//  /* Extend the timebase to 64 bits. TIM2 is 32 bits at 240 MHz and wraps
//     every 17.9 s, which is shorter than a single period below 0.056 Hz. This
//     handler reads the counter every few hundred nanoseconds, so one compare
//     is enough to catch every wrap - and it is all that stands between a
//     frequency range of two decades and one of nine. */
//  if (now < tim2_last)
//  {
//    tim2_hi++;
//  }
//  tim2_last = now;
//  const uint64_t now64 = ((uint64_t)tim2_hi << 32) | (uint64_t)now;
//
//  isr_total++;
//
//  /* PD14: re-arm from the queue if the pending compare has come out. */
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
//
//  /* ---- the enable sequence ---------------------------------------------
//     One signed compare. The branch is not taken on all but four interrupts of
//     the entire run, and taking it is what places PF3 to within one conversion
//     of its deadline regardless of what else the CPU is doing. */
//  if ((int64_t)(now64 - seq_deadline) >= 0)
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
//  /* rt_safe_blank is 0 when the segment asked for no blanking, and the
//     compare is then trivially true - so the compile-time #if that used to
//     switch this line away has nothing left to do. */
//  if (safe_arm && ((now64 - safe_arm_tick) >= rt_safe_blank))
//  {
//    if ((a > rt_safe_hi) || (a < rt_safe_lo))
//    {
//      safe_run++;
//      if (safe_run >= rt_trip_count)
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
//        seq_deadline    = now64 + rt_pd15_hold;
//        seq_state       = SEQ_POST;
//        safe_arm        = 0U;
//
//        if (pb5_high)
//        {
//          PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//          on_ticks      += now - last_edge_tick;
//          pb5_high       = 0U;
//          last_edge_tick = now;
//          edges++;
//          pd14_push();
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
//      on_ticks      += now - last_edge_tick;
//      pb5_high       = 0U;
//      last_edge_tick = now;
//      edges++;
//      pd14_push();
//    }
//    goto out;
//  }
//#endif
//
//  {
//    /* The DDS. The tick difference is 64 bit and the step is (mul >> shift)
//       with both picked for this segment, so one expression covers millihertz
//       and kilohertz alike. Re-anchoring keeps the product inside 64 bits: a
//       difference of 2^32 against a mantissa of 2^31 is 2^63, and without it a
//       long gate at a high frequency would run past that. The branch is taken
//       once every 17.9 seconds. */
//    uint64_t dt = now64 - phase_ref;
//    if (dt >= (1ULL << 32))
//    {
//      phase_base += (uint32_t)((dt * rt_phase_mul) >> rt_phase_shift);
//      phase_ref   = now64;
//      dt          = 0U;
//    }
//    const uint32_t phase = phase_base
//                         + (uint32_t)((dt * rt_phase_mul) >> rt_phase_shift);
//    const uint32_t w     = window_lut[phase >> LUT_SHIFT];
//    const uint32_t hi    = w & 0xFFFFU;
//    const uint32_t lo    = w >> 16;
//    const uint32_t held  = now - last_edge_tick;
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
//      if ((rt_max_ton != 0U) && (held >= rt_max_ton))
//      {
//        PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//        on_ticks      += held;
//        pb5_high       = 0U;
//        last_edge_tick = now;
//        edges++;
//        forced_off++;
//        pd14_push();
//      }
//      else if (want < 0)
//      {
//        if (held >= rt_min_ton)
//        {
//          PWM_PORT->BSRR = (uint32_t)PWM_PIN << 16;
//          on_ticks      += held;
//          pb5_high       = 0U;
//          last_edge_tick = now;
//          edges++;
//          pd14_push();
//        }
//        else
//        {
//          blocked_off++;               /* inside the minimum on window */
//        }
//      }
//    }
//    else
//    {
//      if ((rt_max_toff != 0U) && (held >= rt_max_toff))
//      {
//        PWM_PORT->BSRR = (uint32_t)PWM_PIN;
//        pb5_high       = 1U;
//        last_edge_tick = now;
//        edges++;
//        forced_on++;
//        pd14_push();
//      }
//      else if (want > 0)
//      {
//        if (held >= rt_min_toff)
//        {
//          PWM_PORT->BSRR = (uint32_t)PWM_PIN;
//          pb5_high       = 1U;
//          last_edge_tick = now;
//          edges++;
//          pd14_push();
//        }
//        else
//        {
//          blocked_on++;                /* inside the minimum off window */
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
//    if (gate_open)
//    {
//      if (trace_n < rt_trace_cap)
//      {
//        if (++trace_skip >= rt_trace_dec)
//        {
//          trace_skip = 0U;
//          trace_buf[rt_trace_base + trace_n] = (hi << 16)
//                                             | (pb5_high << 15)
//                                             | (a & 0x0FFFU);
//          trace_n++;
//        }
//      }
//      else
//      {
//        /* The slice ran out before the gate closed, so the capture covers only
//           the front of this segment. Said out loud rather than left for the
//           host to infer from a short array, because the decimation that caused
//           it was computed from a measured rate and a measured rate can be
//           wrong about a segment that trips early or runs hot. */
//        trace_full = 1U;
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
//    /* No second TIM2 read: that one APB round trip was a quarter of the
//       handler's bus traffic, spent measuring the handler. */
//    const uint32_t dur = DWT->CYCCNT - c0;
//    if (dur > isr_max_cycles)
//    {
//      isr_max_cycles = dur;
//    }
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
//  ctrl->cmd_bad             = 0U;
//  ctrl->adc_rate_meas_hz    = 0U;
//  ctrl->isr_count           = 0U;
//  ctrl->isr_total           = 0U;
//  ctrl->gate_ticks          = 0U;
//  ctrl->isr_max_ticks       = 0U;
//  ctrl->isr_max_cycles      = 0U;
//  ctrl->cpu_hz              = HAL_RCC_GetSysClockFreq();
//  ctrl->cmd_addr            = CMD_ADDR;
//  ctrl->res_addr            = RES_ADDR;
//  ctrl->max_seg             = MAX_SEG;
//  ctrl->seg_count           = 0U;
//  ctrl->seg_index           = 0U;
//  ctrl->seg_done            = 0U;
//  ctrl->run_seq             = 0U;
//  ctrl->rate_probe_hz       = 0U;
//  ctrl->ext_trig_enabled    = EXT_TRIG_ENABLE ? 1U : 0U;
//  ctrl->ext_trig_count      = 0U;
//  ctrl->ext_trig_waits      = 0U;
//  ctrl->seq_state           = 0U;
//  ctrl->gate_open           = 0U;
//  ctrl->gate_ticks_cfg      = 0U;
//  ctrl->isr_live            = 0U;
//#if TRACE_ENABLE
//  ctrl->trace_samples       = TRACE_SAMPLES;
//#else
//  ctrl->trace_samples       = 0U;
//#endif
//  ctrl->adc_ovr             = 0U;
//  ctrl->sine_hz             = SINE_HZ;
//  ctrl->sine_mhz            = SINE_MHZ_DEFAULT;
//  ctrl->gate_us             = 0U;
//  ctrl->phase_mul           = 0U;
//  ctrl->phase_shift         = 0U;
//  ctrl->tim2_wraps          = 0U;
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
//  ctrl->gate_periods_milli  = 0U;   /* per segment now; see the table */
//  ctrl->on_ticks            = 0U;
//  ctrl->safe_hi_mv          = SAFE_HI_MV;
//  ctrl->safe_lo_mv          = SAFE_LO_MV;
//  ctrl->tripped             = 0U;
//  ctrl->trip_code           = 0U;
//  ctrl->trip_ticks          = 0U;
//#if TRACE_ENABLE
//  ctrl->trace_addr          = (uint32_t)(uintptr_t)trace_buf;
//  ctrl->trace_stride        = 4U;
//  ctrl->trace_decimate      = 0U;   /* per segment now; see the table */
//#else
//  ctrl->trace_addr          = 0U;
//  ctrl->trace_stride        = 0U;
//  ctrl->trace_decimate      = 0U;
//#endif
//  ctrl->trace_count         = 0U;
//}
//
///** Republish the handful of fields that describe where the run is now.
//
//    Called from main's spin loops. It is four stores to non-cacheable SRAM1,
//    which is not free - but it is in a loop that exists only to wait, and a run
//    that hangs is otherwise completely opaque from the host. */
//static void Ctrl_Live(void)
//{
//  ctrl->seq_state      = seq_state;
//  ctrl->gate_open      = gate_open;
//  ctrl->gate_ticks_cfg = (uint32_t)gate_ticks_cfg;
//  ctrl->tim2_wraps     = tim2_hi;
//  ctrl->isr_live       = isr_total;
//  ctrl->ext_trig_count = ext_trig_count;
//  ctrl->tim2_wraps     = tim2_hi;
//}
//
///** @param gate_ticks the MEASURED length of the PF3 window, in TIM2 ticks. */
//static void Ctrl_Publish(uint32_t gate_ticks)
//{
//  ctrl->isr_count        = isr_count;
//  ctrl->isr_total        = isr_total;
//  ctrl->gate_ticks       = gate_ticks;
//  ctrl->isr_max_ticks    = 0U;             /* superseded */
//  ctrl->isr_max_cycles   = isr_max_cycles;
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
//  ctrl->trip_ticks       = (uint32_t)trip_ticks;
//#if TRACE_ENABLE
//  ctrl->trace_count      = trace_n;
//#endif
//
//  if (gate_ticks)
//  {
//    ctrl->adc_rate_meas_hz =
//        (uint32_t)(((uint64_t)isr_count * TIMER_CLK_HZ) / gate_ticks);
//  }
//}
//
///* ==========================================================================
// * The playlist engine
// * ==========================================================================
// * Everything here runs in main, between segments, with the gate closed. None
// * of it is on the conversion path.
// */
//
///** Nanoseconds to TIM2 ticks, at runtime. The macro version is still used for
//    the compile-time defaults; this is the same arithmetic for a number that is
//    not known until a host sends it. */
//static uint32_t ns_to_ticks(uint64_t ns)
//{
//  return (uint32_t)((ns * TIMER_CLK_HZ) / 1000000000ULL);
//}
//
///**
//  * @brief Resolve one field of a host-supplied segment.
//  *
//  * 0 means "use the compiled-in default", which is what lets a notebook set
//  * two fields and leave the rest alone. That leaves no way to ask for a
//  * literal zero, and two fields genuinely want one - a disabled maximum on
//  * time, and a safety floor of 0 mV - so 0xFFFFFFFF is the explicit zero.
//  * Encoding it that way round is deliberate: forgetting a field gives you the
//  * working default, never a disabled dwell ceiling or an unpoliced floor.
//  */
//static uint32_t opt_u32(uint32_t v, uint32_t dflt)
//{
//  if (v == 0U)          { return dflt; }
//  if (v == 0xFFFFFFFFU) { return 0U; }
//  return v;
//}
//
///* The widest a host may ask for. PD14's delay is bounded by TIM4's 16-bit
//   counter at 240 MHz; the two sequence delays are bounded by what a 32-bit
//   tick deadline can carry, with room to spare. */
//#define PD14_DELAY_MAX_NS       250000UL      /* 250 us, inside TIM4's 273 */
//#define SEQ_DELAY_MAX_US        2000000UL     /* 2 s                       */
//
//static uint32_t cmd_bad;      /* bitmask of run-wide fields that were bad */
//
///** Take a host value if it is inside range, otherwise the default, and record
//    which it was. 0 means "use the default" as everywhere else. */
//static uint32_t Cmd_Range(uint32_t v, uint32_t dflt, uint32_t hi, uint32_t bit)
//{
//  if (v == 0U)
//  {
//    return dflt;
//  }
//  if (v > hi)
//  {
//    cmd_bad |= bit;
//    return dflt;
//  }
//  return v;
//}
//
//static uint32_t run_pd14_delay_ns  = PD14_DELAY_NS;
//static uint32_t run_pf3_after_us   = PF3_AFTER_PD15_US;
//static uint32_t run_pd15_after_us  = PD15_AFTER_PF3_US;
//
///** Fill in every field a segment left at zero. Works on a private copy, so the
//    playlist keeps the host's raw request and the result table can report what
//    was actually used. */
//static void seg_resolve(const seg_cfg_t *in, seg_cfg_t *o)
//{
//  /* Millihertz wins when it is given; whole hertz is the fallback, so a host
//     written before fractional frequencies existed still works unchanged. */
//  o->sine_mhz        = in->sine_mhz ? in->sine_mhz
//                       : (in->sine_hz ? in->sine_hz * 1000UL
//                                      : SINE_MHZ_DEFAULT);
//  o->sine_hz         = (o->sine_mhz + 500UL) / 1000UL;
//  o->sine_offset_mv  = in->sine_offset_mv ? in->sine_offset_mv
//                                          : (uint32_t)SINE_OFFSET_MV;
//  o->sine_amp_mv     = in->sine_amp_mv ? in->sine_amp_mv
//                                        : (uint32_t)SINE_AMP_MV;
//  o->threshold_mv    = in->threshold_mv ? in->threshold_mv
//                                         : (uint32_t)THRESHOLD_MV;
//  o->min_ton_ns      = in->min_ton_ns  ? in->min_ton_ns  : MIN_TON_NS;
//  o->min_toff_ns     = in->min_toff_ns ? in->min_toff_ns : MIN_TOFF_NS;
//  o->max_ton_ns      = opt_u32(in->max_ton_ns,  MAX_TON_NS);
//  o->max_toff_ns     = opt_u32(in->max_toff_ns, MAX_TOFF_NS);
//  o->periods_milli   = in->periods_milli ? in->periods_milli
//                       : (uint32_t)(GATE_SINE_PERIODS * 1000.0f);
//  o->gap_us          = in->gap_us;            /* 0 gap is a legal request */
//  o->safe_hi_mv      = in->safe_hi_mv ? in->safe_hi_mv : (uint32_t)SAFE_HI_MV;
//  o->safe_lo_mv      = opt_u32(in->safe_lo_mv,    (uint32_t)SAFE_LO_MV);
//  o->safe_blank_us   = opt_u32(in->safe_blank_us, SAFE_BLANK_US);
//  o->safe_trip_count = in->safe_trip_count ? in->safe_trip_count
//                                            : SAFE_TRIP_COUNT;
//  o->trace_decimate  = in->trace_decimate ? in->trace_decimate
//                                           : TRACE_DECIMATE;
//}
//
///**
//  * @brief Can this segment run at all?
//  *
//  * Every rule here is one that would otherwise produce a run that LOOKS like
//  * it worked. A band that pokes outside the safety window trips on the first
//  * conversion and yields a two-sample capture; a maximum on time at or below
//  * its minimum forces an edge before the minimum allows one, so nothing ever
//  * turns on; a PD14 delay longer than TIM4's 16-bit range comes out 273 us
//  * late. Each is refused with a reason the host can print, rather than
//  * clamped into something plausible.
//  */
///**
//  * @brief Split a frequency into (mul, shift) so the DDS step keeps its bits.
//  *
//  * The step wanted is f * 2^32 / TIMER_CLK_HZ phase units per tick - 0.0179 at
//  * 1 mHz, 1.6e7 at 900 kHz. No single fixed-point format holds both: a scale
//  * coarse enough for the top of the range rounds the bottom to nothing, which
//  * is exactly what the old Q8 step did, turning 1 mHz into a 13% error.
//  *
//  * So the binary point moves. This is ordinary long division carried on past
//  * the point, stopping as soon as the quotient fills the top of a 32-bit word,
//  * and the shift records how far it went. The mantissa therefore always has
//  * about 30 significant bits whatever the frequency, and the handler's
//  * multiply stays a single 64-bit operation.
//  *
//  * Checked against exact arithmetic from 1 mHz to 900 kHz: worst frequency
//  * error 8 parts in 10^10, worst 64-bit product 8.9e18 against a 1.8e19 limit.
//  */
//static void Dds_Scale(uint32_t sine_mhz, uint32_t *mul, uint32_t *shift)
//{
//  /* Denominator is ticks per second x 1000, because the frequency is x 1000. */
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
//static uint32_t seg_check(const seg_cfg_t *c, uint64_t *gate_out)
//{
//  if ((c->sine_mhz < SINE_MHZ_MIN) || (c->sine_mhz > SINE_MHZ_MAX))
//  {
//    return REJ_FREQ;
//  }
//  if (c->periods_milli == 0U)
//  {
//    return REJ_PERIODS;
//  }
//
//  const int32_t band_hi = (int32_t)c->sine_offset_mv + (int32_t)c->sine_amp_mv
//                        + (int32_t)c->threshold_mv;
//  const int32_t band_lo = (int32_t)c->sine_offset_mv - (int32_t)c->sine_amp_mv
//                        - (int32_t)c->threshold_mv;
//  if (band_hi > (int32_t)c->safe_hi_mv)
//  {
//    return REJ_BAND_HI;
//  }
//  if (band_lo < (int32_t)c->safe_lo_mv)
//  {
//    return REJ_BAND_LO;
//  }
//
//  if ((c->max_ton_ns != 0U) && (c->max_ton_ns <= c->min_ton_ns))
//  {
//    return REJ_DWELL;
//  }
//  if ((c->max_toff_ns != 0U) && (c->max_toff_ns <= c->min_toff_ns))
//  {
//    return REJ_DWELL;
//  }
//
//  /* The PD14 queue has to hold every edge in flight during one delay, and the
//     worst case is an edge every MIN(min Ton, min Toff) - the same rule the
//     static assert enforces for the defaults, applied to what was asked for. */
//  const uint32_t spacing = (c->min_ton_ns < c->min_toff_ns) ? c->min_ton_ns
//                                                            : c->min_toff_ns;
//  if (spacing == 0U)
//  {
//    return REJ_PD14_QUEUE;
//  }
//  if (((run_pd14_delay_ns / spacing) + 2U) > PD14_QUEUE_N)
//  {
//    return REJ_PD14_QUEUE;
//  }
//  if ((ns_to_ticks(run_pd14_delay_ns) + PD14_MIN_AHEAD) >= 65536U)
//  {
//    return REJ_PD14_RANGE;
//  }
//
//  /* periods_milli / 1000 periods, each 1000/sine_mhz seconds long:
//         ticks = periods_milli * 1000 * TIMER_CLK_HZ / (1000 * sine_mhz)
//     which is the expression below with the thousands cancelled. At 1 mHz and
//     five periods this is 1.2e12 ticks - five thousand seconds - which is why
//     the gate is carried in 64 bits and why GATE_TICKS_MAX is what it is. */
//  const uint64_t g = ((uint64_t)c->periods_milli * (uint64_t)TIMER_CLK_HZ)
//                   / (uint64_t)c->sine_mhz;
//  if ((g == 0ULL) || (g > GATE_TICKS_MAX))
//  {
//    return REJ_GATE_LONG;
//  }
//  *gate_out = g;
//  return REJ_NONE;
//}
//
///**
//  * @brief Hand one segment's numbers to the handler.
//  *
//  * THE CALLER MUST HAVE MASKED ADC_IT_EOC. Half of these are read by the
//  * decision path and half by the safety trip, and there is no ordering of the
//  * writes that would let the handler see a consistent mixture of two segments
//  * - so the handler is simply not running while they change. That is safe
//  * here and nowhere else: PF3 is low, PB5 is idle low, and the PD14 queue has
//  * already been given four delays to drain.
//  */
//static void Seg_Load(const seg_cfg_t *c, uint64_t gate_ticks,
//                     uint32_t slice_base, uint32_t slice_cap)
//{
//  rt_min_ton    = ns_to_ticks(c->min_ton_ns);
//  rt_max_ton    = ns_to_ticks(c->max_ton_ns);     /* 0 disables the ceiling */
//  rt_min_toff   = ns_to_ticks(c->min_toff_ns);
//  rt_max_toff   = ns_to_ticks(c->max_toff_ns);
//  Dds_Scale(c->sine_mhz, &rt_phase_mul, &rt_phase_shift);
//  rt_safe_hi    = (uint32_t)(((uint64_t)c->safe_hi_mv << ADC_RES_BITS)
//                             / VREF_MV);
//  rt_safe_lo    = (uint32_t)(((uint64_t)c->safe_lo_mv << ADC_RES_BITS)
//                             / VREF_MV);
//  rt_safe_blank = ns_to_ticks((uint64_t)c->safe_blank_us * 1000ULL);
//  rt_trip_count = c->safe_trip_count;
//
//  gate_ticks_cfg = gate_ticks;
//
//  /* The trace slice. A decimation of 0 means "make the slice span the gate",
//     which needs the conversion rate - so it uses the one Rate_Probe()
//     MEASURED at startup, not a figure derived from the ADC configuration.
//     The configuration said 4 MSPS once on this silicon and the hardware
//     delivered 2. */
//  uint32_t dec = c->trace_decimate;
//  if ((dec == 0U) && slice_cap)
//  {
//    const uint64_t expect = (gate_ticks / TIMER_CLK_HZ) * rate_probe_hz
//                          + ((gate_ticks % TIMER_CLK_HZ) * rate_probe_hz)
//                            / TIMER_CLK_HZ;
//    dec = (uint32_t)((expect + slice_cap - 1U) / slice_cap);
//  }
//  if (dec == 0U)
//  {
//    dec = 1U;
//  }
//  rt_trace_dec  = dec;
//  rt_trace_base = slice_base;
//  rt_trace_cap  = slice_cap;
//
//  Lut_Build(c->sine_offset_mv, c->sine_amp_mv, c->threshold_mv);
//
//  /* Every counter below describes ONE segment, so every one of them is zeroed
//     here rather than accumulated across the playlist. */
//  isr_count      = 0U;
//  isr_max_cycles = 0U;
//  edges          = 0U;
//  forced_off     = 0U;
//  forced_on      = 0U;
//  blocked_off    = 0U;
//  blocked_on     = 0U;
//  on_ticks       = 0U;
//  safe_run       = 0U;
//  pd14_late      = 0U;
//  pd14_dropped   = 0U;
//  pd14_queue_max = 0U;
//  gate_meas_ticks = 0U;
//  trip_code      = 0U;
//  trip_ticks     = 0U;
//#if TRACE_ENABLE
//  trace_n        = 0U;
//  trace_skip     = 0U;
//  trace_full     = 0U;
//#endif
//}
//
///** The parameters that were asked for - written for a rejected segment too, so
//    the host can see what it sent alongside the reason it was refused. */
//static void Seg_Params(uint32_t i, const seg_cfg_t *c)
//{
//  res[i].sine_hz        = c->sine_hz;
//  res[i].sine_mhz       = c->sine_mhz;
//  res[i].phase_mul      = rt_phase_mul;
//  res[i].phase_shift    = rt_phase_shift;
//  res[i].gate_us        = (uint32_t)(gate_meas_ticks / (TIMER_CLK_HZ / 1000000UL));
//  res[i].sine_amp_mv    = c->sine_amp_mv;
//  res[i].sine_offset_mv = c->sine_offset_mv;
//  res[i].threshold_mv   = c->threshold_mv;
//  res[i].min_ton_ns     = c->min_ton_ns;
//  res[i].max_ton_ns     = c->max_ton_ns;
//  res[i].min_toff_ns    = c->min_toff_ns;
//  res[i].max_toff_ns    = c->max_toff_ns;
//  res[i].periods_milli  = c->periods_milli;
//}
//
//static void Seg_Record(uint32_t i, const seg_cfg_t *c)
//{
//  Seg_Params(i, c);
//  res[i].valid          = SEG_RAN;
//  res[i].reject_reason  = REJ_NONE;
//  res[i].gate_ticks     = (uint32_t)gate_meas_ticks;   /* see gate_us */
//  res[i].isr_count      = isr_count;
//  res[i].isr_max_cycles = isr_max_cycles;
//  res[i].adc_ovr        = (ADC1->ISR & ADC_ISR_OVR) ? 1U : 0U;
//  res[i].edges          = edges;
//  res[i].forced_off     = forced_off;
//  res[i].forced_on      = forced_on;
//  res[i].blocked_off    = blocked_off;
//  res[i].blocked_on     = blocked_on;
//  res[i].on_ticks       = on_ticks;
//  res[i].tripped        = tripped;
//  res[i].trip_code      = trip_code;
//  res[i].trip_ticks     = (uint32_t)trip_ticks;
//#if TRACE_ENABLE
//  /* Bit 31 of trace_decimate is the "this slice filled before the gate
//     closed" flag, so the host knows the capture is the front of the segment
//     and not the whole of it. */
//  res[i].trace_off      = rt_trace_base;
//  res[i].trace_count    = trace_n;
//  res[i].trace_decimate = trace_full ? (rt_trace_dec | 0x80000000U)
//                                     : rt_trace_dec;
//#endif
//  res[i].clamp_hi       = lut_clamp_hi;
//  res[i].clamp_lo       = lut_clamp_lo;
//  res[i].pd14_late      = pd14_late;
//  res[i].pd14_dropped   = pd14_dropped;
//
//  res[i].rate_meas_hz   = gate_meas_ticks
//      ? (uint32_t)(((uint64_t)isr_count * (uint64_t)TIMER_CLK_HZ)
//                   / gate_meas_ticks)
//      : 0U;
//
//  /* Feed the measured IN-GATE rate back into the trace sizing. Rate_Probe()
//     runs before any gate is open, where the handler takes its short path and
//     conversions therefore arrive faster than they will once the decision logic
//     is running - so its figure is optimistic and the first segment's slice
//     comes out sparser than it could be. From the second segment on, the
//     decimation is sized from a rate measured under the real load. */
//  if (res[i].rate_meas_hz)
//  {
//    rate_probe_hz = res[i].rate_meas_hz;
//  }
//
//  /* The status block's scalars describe the last segment that ran, so a
//     one-segment playlist reads exactly as a single run always did. */
//  ctrl->sine_hz            = c->sine_hz;
//  ctrl->sine_mhz           = c->sine_mhz;
//  ctrl->phase_mul          = rt_phase_mul;
//  ctrl->phase_shift        = rt_phase_shift;
//  ctrl->gate_us            = res[i].gate_us;
//  ctrl->sine_offset_mv     = c->sine_offset_mv;
//  ctrl->sine_amp_mv        = c->sine_amp_mv;
//  ctrl->threshold_mv       = c->threshold_mv;
//  ctrl->min_ton_ns         = c->min_ton_ns;
//  ctrl->max_ton_ns         = c->max_ton_ns;
//  ctrl->min_toff_ns        = c->min_toff_ns;
//  ctrl->max_toff_ns        = c->max_toff_ns;
//  ctrl->gate_periods_milli = c->periods_milli;
//#if TRACE_ENABLE
//  ctrl->trace_decimate     = rt_trace_dec;
//#endif
//  ctrl->clamp_hi           = lut_clamp_hi;
//  ctrl->clamp_lo           = lut_clamp_lo;
//  ctrl->pd14_delay_ns      = run_pd14_delay_ns;
//  ctrl->pf3_after_pd15_us  = run_pf3_after_us;
//  ctrl->pd15_after_pf3_us  = run_pd15_after_us;
//}
//
//static void Seg_Reject(uint32_t i, const seg_cfg_t *c, uint32_t reason)
//{
//  Seg_Params(i, c);
//  res[i].valid         = SEG_REJECTED;
//  res[i].reject_reason = reason;
//  res[i].trace_count   = 0U;
//  res[i].gate_ticks    = 0U;
//  res[i].isr_count     = 0U;
//}
//
///**
//  * @brief MEASURE the conversion rate, once, before the run.
//  *
//  * Counts interrupts over a one-millisecond TIM2 window. It is used only to
//  * size the trace decimation, so a few percent either way is harmless - but it
//  * has to be measured rather than computed from the ADC dividers, because on
//  * this silicon a conversion takes twice its datasheet cycle count and a
//  * derived figure would be out by two.
//  *
//  * This runs before PD15 rises, by which point the handler has already been
//  * firing for the whole of Adc_Config()'s calibration, so the instruction
//  * cache is warm and the figure describes the steady state.
//  */
//static void Rate_Probe(void)
//{
//  const uint32_t t0 = TIM2->CNT;
//  const uint32_t n0 = isr_total;
//
//  while ((TIM2->CNT - t0) < (TIMER_CLK_HZ / 1000U))
//  {
//    /* spin */
//  }
//
//  const uint32_t dt = TIM2->CNT - t0;
//  const uint32_t dn = isr_total - n0;
//
//  rate_probe_hz = dt ? (uint32_t)(((uint64_t)dn * TIMER_CLK_HZ) / dt) : 0U;
//  if (rate_probe_hz == 0U)
//  {
//    rate_probe_hz = 1U;       /* never divide the trace sizing by zero */
//  }
//  ctrl->rate_probe_hz = rate_probe_hz;
//}
//
///** The compiled-in sweep. Every field the rows do not name stays 0, which
//    seg_resolve() turns into the default - so the built-in playlist and a
//    host-supplied one go through exactly the same path. */
//static void Playlist_Default(void)
//{
//  for (uint32_t i = 0U; i < MAX_SEG; i++)
//  {
//    uint32_t *w = (uint32_t *)&playlist[i];
//    for (uint32_t k = 0U; k < sizeof(seg_cfg_t) / 4U; k++)
//    {
//      w[k] = 0U;
//    }
//  }
//
//  uint32_t n = 0U;
//#define X(hz, amp, thr, per, gap)                                   \
//  playlist[n].sine_mhz      = (uint32_t)((hz) * 1000UL);            \
//  playlist[n].sine_amp_mv   = (amp);                                \
//  playlist[n].threshold_mv  = (thr);                                \
//  playlist[n].periods_milli = (uint32_t)((per) * 1000.0f);           \
//  playlist[n].gap_us        = (gap);                                \
//  n++;
//  PLAYLIST_ROWS
//#undef X
//
//  playlist_n        = n;
//  playlist_flags    = PLAYLIST_LOOP ? CMD_FLAG_LOOP : 0U;
//  run_ext_trig      = EXT_TRIG_ENABLE ? 1U : 0U;
//  run_seq           = 0U;
//  run_pd14_delay_ns = PD14_DELAY_NS;
//  run_pf3_after_us  = PF3_AFTER_PD15_US;
//  run_pd15_after_us = PD15_AFTER_PF3_US;
//}
//
//#define X(a, b, c, d, e) + 1U
//_Static_assert((0U PLAYLIST_ROWS) <= MAX_SEG,
//               "the built-in playlist has more rows than MAX_SEG");
//_Static_assert((0U PLAYLIST_ROWS) >= 1U, "the built-in playlist is empty");
//#undef X
//
///** Clear the command block, so a request left over from a previous image
//    cannot launch a run the moment this one boots. */
//static void Cmd_Init(void)
//{
//  uint32_t *w = (uint32_t *)CMD_ADDR;
//  for (uint32_t k = 0U; k < sizeof(cmd_blk_t) / 4U; k++)
//  {
//    w[k] = 0U;
//  }
//}
//
///**
//  * @brief Adopt a host-supplied playlist, if there is a new one.
//  * @return 1 if the playlist changed.
//  *
//  * The magic AND a changed seq are both required, and the host writes seq
//  * last - so a block caught half written still carries the previous seq and is
//  * ignored. A host that writes the same seq twice gets one run, not two.
//  */
//static uint32_t Cmd_Poll(void)
//{
//  if (cmd->magic != CMD_MAGIC)
//  {
//    return 0U;
//  }
//
//  const uint32_t seq = cmd->seq;
//  if (seq == run_seq)
//  {
//    return 0U;
//  }
//
//  uint32_t n = cmd->n_seg;
//  if (n == 0U)       { n = 1U; }
//  if (n > MAX_SEG)   { n = MAX_SEG; }
//
//  for (uint32_t i = 0U; i < MAX_SEG; i++)
//  {
//    const volatile uint32_t *src = (const volatile uint32_t *)&cmd->seg[i];
//    uint32_t *dst = (uint32_t *)&playlist[i];
//    for (uint32_t k = 0U; k < sizeof(seg_cfg_t) / 4U; k++)
//    {
//      dst[k] = src[k];
//    }
//  }
//
//  playlist_n     = n;
//  playlist_flags = cmd->flags;
//
//  /* The three run-wide times are RANGE CHECKED, not just defaulted.
//     seg_check() polices the per-segment fields, but these three are used
//     before any segment is looked at - and an out-of-range one does not fail
//     loudly, it turns into a wait. ns_to_ticks(0xFFFFFFFF * 1000) is a 32-bit
//     truncation of 10^12 ticks, so a corrupt word here becomes a delay of
//     anything up to eighteen seconds, per segment, and the run just looks
//     slow. Anything outside a sane range falls back to the compiled default
//     and cmd_bad says so, which is the difference between a visible fault and
//     a mysterious one.
//
//     They are checked rather than trusted because this block is written over
//     SWD by a tool that pads and aligns as it sees fit: an all-ones word is
//     exactly what a short or misaligned write leaves behind. */
//  cmd_bad = 0U;
//  run_pd14_delay_ns = Cmd_Range(cmd->pd14_delay_ns, PD14_DELAY_NS,
//                                PD14_DELAY_MAX_NS, 0x1U);
//  run_pf3_after_us  = Cmd_Range(cmd->pf3_after_pd15_us, PF3_AFTER_PD15_US,
//                                SEQ_DELAY_MAX_US, 0x2U);
//  run_pd15_after_us = Cmd_Range(cmd->pd15_after_pf3_us, PD15_AFTER_PF3_US,
//                                SEQ_DELAY_MAX_US, 0x4U);
//  ctrl->cmd_bad           = cmd_bad;
//  ctrl->pd14_delay_ns     = run_pd14_delay_ns;
//  ctrl->pf3_after_pd15_us = run_pf3_after_us;
//  ctrl->pd15_after_pf3_us = run_pd15_after_us;
//
//  /* The only thing in the system that clears the safety latch, and it takes a
//     person deliberately setting a flag. */
//  if (playlist_flags & CMD_FLAG_CLEAR_TRIP)
//  {
//    tripped = 0U;
//  }
//
//  /* Either flag overrides the compiled default; NO_EXT_TRIG wins if a confused
//     host sets both, because refusing to wait is the weaker claim and a run
//     that starts is easier to diagnose than one that never does. */
//  run_ext_trig = EXT_TRIG_ENABLE ? 1U : 0U;
//  if (playlist_flags & CMD_FLAG_EXT_TRIG)
//  {
//    run_ext_trig = 1U;
//  }
//  if (playlist_flags & CMD_FLAG_NO_EXT_TRIG)
//  {
//    run_ext_trig = 0U;
//  }
//
//  run_seq = seq;
//  return 1U;
//}
//
///**
//  * @brief One run: PD15 up, every segment, PD15 down.
//  *
//  * A segment boundary is the only place parameters change, and the gate is
//  * closed across it - so nothing in the decision path ever sees a mixture of
//  * two operating points.
//  */
//static void Run_Playlist(void)
//{
//  ctrl->status    = CTRL_STATUS_WAIT_GATE;
//  ctrl->run_seq   = run_seq;
//  ctrl->ext_trig_enabled = run_ext_trig;
//  ctrl->ext_trig_waits   = ext_trig_waits;
//  ctrl->seg_count = playlist_n;
//  ctrl->seg_done  = 0U;
//  ctrl->seg_index = 0U;
//
//  for (uint32_t i = 0U; i < MAX_SEG; i++)
//  {
//    res[i].valid         = SEG_NOT_RUN;
//    res[i].reject_reason = REJ_NONE;
//    res[i].trace_count   = 0U;
//  }
//
//  rt_pd14_delay = ns_to_ticks(run_pd14_delay_ns);
//  rt_pf3_delay  = ns_to_ticks((uint64_t)run_pf3_after_us * 1000ULL);
//  rt_pd15_hold  = ns_to_ticks((uint64_t)run_pd15_after_us * 1000ULL);
//
//  /* ---- wait for the external start, if this run wants one -------------
//     Everything above is preparation and none of it drives anything; the run
//     properly begins on the next line. So this is where the edge belongs: the
//     board is fully armed and the instant PB8 rises, PD15 follows within a poll
//     of main's loop. */
//  if (run_ext_trig)
//  {
//    ctrl->status   = CTRL_STATUS_WAIT_TRIG;
//    ext_trig_fired = 0U;          /* discard anything that arrived earlier */
//    ext_trig_waits++;
//
//    const uint32_t t_wait = TIM2->CNT;
//    const uint32_t limit  = ns_to_ticks((uint64_t)EXT_TRIG_TIMEOUT_MS
//                                        * 1000000ULL);
//    while (!ext_trig_fired)
//    {
//      ctrl->ext_trig_count = ext_trig_count;
//      if (EXT_TRIG_TIMEOUT_MS && ((TIM2->CNT - t_wait) >= limit))
//      {
//        /* Say so rather than sit silent. A run that blocks forever on an edge
//           that never comes looks exactly like a crashed target from the host,
//           and the two want completely different responses. */
//        ctrl->status         = CTRL_STATUS_NO_TRIG;
//        ctrl->ext_trig_count = ext_trig_count;
//        ctrl->seg_done       = 0U;
//        return;
//      }
//    }
//  }
//
//  /* ---- PD15, the startup-circuit enable: up for the WHOLE playlist ----- */
//  const uint32_t t_en1 = TIM2->CNT;
//  EN1_PORT->BSRR = (uint32_t)EN1_PIN;
//
//#if !SAFE_ARM_IN_GATE
//  safe_arm_tick = t_en1;        /* reference first, then arm */
//  safe_arm      = 1U;
//#endif
//
//  /* The trace buffer is handed out as the segments run, so a segment that
//     finishes early gives its unused words back to the ones after it rather
//     than leaving a hole in the buffer. */
//  uint32_t used      = 0U;
//  uint32_t done      = 0U;
//  uint32_t lead_in   = t_en1 + rt_pf3_delay;   /* when segment 0 opens */
//  uint32_t have_lead = 1U;
//
//  for (uint32_t i = 0U; i < playlist_n; i++)
//  {
//    seg_cfg_t c;
//    uint64_t  gate = 0U;
//
//    /* EVERYTHING main does between segments runs with the handler masked.
//       Not for atomicity - for speed. With an interrupt per conversion whose
//       body is longer than the conversion period, main's share of the CPU is
//       not small, it is very close to zero: the core tail-chains from one
//       handler straight into the next and main advances a few instructions a
//       millisecond. Work that reads as "a hundred instructions, call it
//       microseconds" then takes seconds, and a four-segment playlist takes a
//       minute to do eighty milliseconds of switching.
//
//       The handler has nothing to do here anyway: PF3 is low, PB5 is idle low,
//       and the PD14 queue was drained by the settle wait at the end of the
//       previous segment - which is the one stretch that deliberately leaves the
//       handler running, because draining that queue is its job. */
//    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC);
//
//    ctrl->seg_index = i;
//    seg_resolve(&playlist[i], &c);
//
//    const uint32_t rej = seg_check(&c, &gate);
//    if (rej != REJ_NONE)
//    {
//      Seg_Reject(i, &c, rej);
//      __DSB();
//      ADC1->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
//      __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
//      continue;
//    }
//
//#if TRACE_ENABLE
//    const uint32_t left = (playlist_n - i);
//    const uint32_t cap  = (TRACE_SAMPLES - used) / left;
//#else
//    const uint32_t cap  = 0U;
//#endif
//
//    /* ---- the parameter change, with the handler off -------------------- */
//    /* ---- load AND arm, both with the handler off ------------------------
//       Arming is two stores - the deadline, then the state - and the handler
//       MUST NOT run between them. If it does, it sees the new deadline with the
//       OLD state, falls into Seq_Advance()'s default branch and parks the
//       deadline SEQ_NEVER_TICKS - 4.47 seconds - into the future. PF3 then
//       rises 4.47 s late, every segment, and a four-segment playlist takes
//       twenty-two seconds to do fifty milliseconds of work.
//
//       That is not a rare race. main's share of the CPU here is
//       1 - (handler / conversion period), and the handler is LONGER than the
//       conversion period - so a single store from main can take a microsecond
//       to retire, with thousands of handler entries inside the window. Masking
//       is the fix rather than a clever lock-free handshake, because between
//       segments there is nothing for the handler to do anyway. */
//    Seg_Load(&c, gate, used, cap);
//    seq_deadline = have_lead ? lead_in : TIM2->CNT;
//    have_lead    = 0U;
//    seq_state    = SEQ_WAIT_PF3;
//    /* The mask itself overran the converter and left EOC set; clear both so
//       the first interrupt after the unmask is a real conversion and OVR still
//       means "the handler fell behind". */
//    /* A compiler barrier before the unmask. The rt_* parameters and
//       gate_ticks_cfg are plain statics - the handler reads them, so the stores
//       have to exist, but nothing stops the compiler SINKING one past the
//       volatile write that re-enables the interrupt. Then the first handler
//       entry of the segment would run on one field from the previous operating
//       point. The "memory" clobber inside __DSB() forbids that, and the
//       instruction itself costs a handful of cycles once per segment. */
//    __DSB();
//    ADC1->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
//    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
//
//    while ((seq_state == SEQ_WAIT_PF3) || (seq_state == SEQ_GATE))
//    {
//      if (gate_open)
//      {
//        ctrl->status = CTRL_STATUS_RUNNING;
//      }
//      Ctrl_Live();
//    }
//
//    /* PD14's last edge is still in flight in TIM4, one delay behind PB5.
//       Nothing may mask the handler until it has come out. */
//    const uint32_t settle = TIM2->CNT;
//    while ((TIM2->CNT - settle) < ns_to_ticks(4ULL * run_pd14_delay_ns))
//    {
//      /* spin */
//    }
//
//    /* Masked again for the bookkeeping - same reason, and the handler has
//       nothing left to do now the queue has drained. */
//    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC);
//    Seg_Record(i, &c);
//#if TRACE_ENABLE
//    used += trace_n;
//#endif
//    done++;
//    __DSB();
//    ADC1->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
//    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
//
//    if (tripped)
//    {
//      /* The handler has already dropped PF3 and armed PD15's fall. The rest of
//         the playlist is abandoned: whatever made the input leave the safety
//         window is still out there, and the next segment would meet it too. */
//      break;
//    }
//
//    /* The dead time this segment asked for, measured from here - the next
//       iteration's Seg_Load runs inside it. */
//    lead_in   = TIM2->CNT + ns_to_ticks((uint64_t)c.gap_us * 1000ULL);
//    have_lead = 1U;
//  }
//
//  ctrl->seg_done = done;
//
//  if (!tripped)
//  {
//    /* Same two-store arming, same mask. */
//    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC);
//    seq_deadline = TIM2->CNT + rt_pd15_hold;
//    seq_state    = SEQ_POST;
//    /* A compiler barrier before the unmask. The rt_* parameters and
//       gate_ticks_cfg are plain statics - the handler reads them, so the stores
//       have to exist, but nothing stops the compiler SINKING one past the
//       volatile write that re-enables the interrupt. Then the first handler
//       entry of the segment would run on one field from the previous operating
//       point. The "memory" clobber inside __DSB() forbids that, and the
//       instruction itself costs a handful of cycles once per segment. */
//    __DSB();
//    ADC1->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
//    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
//  }
//  while (seq_state != SEQ_DONE)
//  {
//    Ctrl_Live();
//  }
//
//  Ctrl_Publish(gate_meas_ticks);
//#if TRACE_ENABLE
//  ctrl->trace_count = used;     /* the whole buffer, all segments packed */
//#endif
//  ctrl->status = tripped ? CTRL_STATUS_TRIPPED : CTRL_STATUS_DONE;
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
//  Delay_Timer_Config();
//  Ctrl_Init();
//  Cmd_Init();
//  Adc_Config();
//
//  /* The window LUT is NOT built here. It belongs to a segment, and the first
//     segment builds it - so there is exactly one place amplitude, offset and
//     threshold turn into comparison limits. */
//
//  /* PB5 low, PD14 low. Tie them together before anything can move. */
//  last_edge_tick = TIM2->CNT;
//  pb5_high       = 0U;
//  Pd14_Arm();
//
//  /* Park the sequence before the first interrupt can see it. */
//  seq_state    = SEQ_IDLE;
//  seq_deadline = TIM2->CNT + SEQ_NEVER_TICKS;
//
//  /* The converter runs from here and is never stopped again. The handler
//     starts firing immediately and does nothing but service PD14 until a gate
//     opens. It IS briefly masked between segments while the parameters change -
//     see Seg_Load() - but the conversions themselves never stop. */
//  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
//  if (HAL_ADC_Start(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* MEASURE the conversion rate before anything needs it. */
//  Rate_Probe();
//
//  Playlist_Default();
//
//  /* ---- the run loop ----------------------------------------------------
//     One pass is one PD15 window holding the whole playlist. After it, the
//     board waits: for a host to write a new playlist into the command block, or
//     - if the playlist asked to loop - for nothing at all, and goes round
//     again. A board with no host attached therefore runs the compiled-in sweep
//     and then sits still, which is what a demonstration wants. */
//  for (;;)
//  {
//    Run_Playlist();
//
//    /* A latched trip is never looped through. The input left the safety window
//       once; running the same playlist again would only meet it again, and a
//       demonstration that silently restarts after a trip hides the trip. Only
//       a host command carrying CMD_FLAG_CLEAR_TRIP starts anything after
//       this. */
//    const uint32_t may_loop = (playlist_flags & CMD_FLAG_LOOP) && !tripped;
//
//    while (!Cmd_Poll())
//    {
//      if (may_loop)
//      {
//        break;
//      }
//      /* Idle. A busy loop rather than __WFI(): the WFI gates the H7's debug
//         clock, and a host trying to write the command block into a sleeping
//         target gets "unable to read device id from ROM table" instead of a
//         run. The converter and its handler also stay live, so the PD14 queue
//         is always serviced. */
//      Ctrl_Live();
//    }
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
