/*
 * ============================================================================
 * File:        adc_mkrzero.h
 * Author:      <Your Name>
 * Student ID:  <Your ID>
 * Date:        19 November 2026
 * Platform:    Arduino MKR Zero (ATSAMD21G18A, ARM Cortex-M0+, 48 MHz)
 * Toolchain:   PlatformIO / arm-none-eabi-gcc
 *
 * Description:
 *   Public interface of the driver for the SAMD21 on-chip 12-bit
 *   successive-approximation ADC, input A0 (PA02 = AIN[0]), 0 .. 3.3 V.
 *
 *   The six routines describe WHAT the ADC does, not HOW it is reached.
 *   The caller never touches a register, a pin multiplexer or a bus. The
 *   same prototypes could be implemented for an external I2C or SPI ADC
 *   without changing any code that calls them.
 *
 *   Usage rules:
 *     - init_adc() must succeed before any other routine is used. Every
 *       routine called before that returns false (or ADC_READ_ERROR).
 *     - One conversion at a time: after start_adc() the result must be
 *       collected with singleread_adc() or cancelled with stop_adc()
 *       before the next start_adc() or singlewrite_adc().
 *     - Every result is scaled to 16 bits (0 .. 65520 for 0 .. 3.3 V),
 *       whatever averaging is selected, so callers never need to know
 *       the hardware resolution.
 *
 *   The Arduino library (Arduino.h) is NOT used. Only the CMSIS device
 *   header (sam.h) and the standard C library are included.
 * ============================================================================
 */

#ifndef ADC_MKRZERO_H
#define ADC_MKRZERO_H

#include <stdint.h>     /* uint16_t                                       */
#include <stdbool.h>    /* bool, true, false                              */
#include "sam.h"        /* CMSIS device header: ADC, PM, GCLK, PORT regs  */

/* ---------------------------------------------------------------------------
 * Public constants
 * ------------------------------------------------------------------------- */

/** Full-scale input voltage in millivolts (INTVCC1 reference, gain 1/2). */
#define ADC_VREF_MV         (3300UL)

/** Results are always scaled to this many codes (16 bits). */
#define ADC_FULL_SCALE      (65536UL)

/** Largest valid result: 4095 << 4. Any larger value is impossible.     */
#define ADC_RESULT_MAX      (0xFFF0U)

/** Returned by singleread_adc() on failure. Never a valid result.       */
#define ADC_READ_ERROR      (0xFFFFU)

/** Allowed hardware averaging: 1, 2, 4 ... 1024 samples per result.     */
#define ADC_AVG_MIN         (1U)
#define ADC_AVG_MAX         (1024U)

/* ---------------------------------------------------------------------------
 * Driver routines
 * ------------------------------------------------------------------------- */

/**
 * @brief  Configure the ADC from scratch. No reset default is trusted:
 *         clocks, pin function, calibration, reference, gain, input,
 *         sampling time, resolution, averaging (1), interrupts and events
 *         are all written explicitly. The first conversion is discarded.
 * @return true if the ADC is enabled and produced a conversion;
 *         false if A0 is in use by the DAC or the hardware did not respond.
 */
bool init_adc(void);

/**
 * @brief  Start ONE conversion and return at once (asynchronous).
 * @return true if the conversion was started; false if the driver is not
 *         initialised or a previous conversion has not been collected.
 */
bool start_adc(void);

/**
 * @brief  Abort a conversion in progress (flushes the ADC pipeline) and
 *         discard any result. Safe to call when the ADC is idle.
 * @return true if the ADC is now idle; false if not initialised or the
 *         flush did not complete.
 */
bool stop_adc(void);

/**
 * @brief  Ask whether the conversion begun by start_adc() has finished.
 *         Never blocks.
 * @return true if a result is waiting to be collected; false otherwise
 *         (still converting, nothing started, or not initialised).
 */
bool is_done_adc(void);

/**
 * @brief  Return one 16-bit-scaled result. If start_adc() was called, its
 *         result is collected; otherwise a conversion is started here.
 *         Waits (with a timeout) for the conversion to finish.
 * @return 0 .. ADC_RESULT_MAX on success; ADC_READ_ERROR on failure.
 */
uint16_t singleread_adc(void);

/**
 * @brief  Write one configuration word to the ADC: the number of samples
 *         the hardware averages into each result. The value is read back
 *         from the ADC to confirm the write.
 * @param  avg_samples  1, 2, 4, 8, 16, 32, 64, 128, 256, 512 or 1024.
 * @return true if written and verified; false if the value is invalid,
 *         a conversion is pending, the driver is not initialised, or the
 *         read-back did not match (init_adc() must then be called again).
 */
bool singlewrite_adc(uint16_t avg_samples);

#endif /* ADC_MKRZERO_H */