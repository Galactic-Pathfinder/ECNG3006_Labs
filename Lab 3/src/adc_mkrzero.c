/*
 * ============================================================================
 * File:        adc_mkrzero.c
 * Author:      <Your Name>
 * Student ID:  <Your ID>
 * Date:        19 November 2026
 * Platform:    Arduino MKR Zero (ATSAMD21G18A, ARM Cortex-M0+, 48 MHz)
 * Toolchain:   PlatformIO / arm-none-eabi-gcc
 *
 * Description:
 *   Register-level driver for the SAMD21 on-chip ADC (see adc_mkrzero.h).
 *
 *   Configuration written by init_adc():
 *     Input       : A0 = PA02 = AIN[0], single-ended against GND
 *     Range       : 0 .. 3.3 V  (reference INTVCC1 = VDDANA/2 = 1.65 V,
 *                                gain 1/2)
 *     ADC clock   : GCLK0 48 MHz / 32 = 1.5 MHz
 *     Sample time : SAMPLEN = 3 -> (3 + 1) / 2 = 2 ADC clocks, which gives a
 *                   high-impedance source (potentiometer) time to charge the
 *                   sample-and-hold capacitor
 *     Resolution  : 12 bits, hardware averaging 1 (changed by
 *                   singlewrite_adc())
 *     Interrupts, events, window monitor and gain/offset correction: off
 *
 *   Result scaling:
 *     With 2^n averaged samples the hardware returns 12 + n bits (n <= 4)
 *     and right-shifts larger sums back to 16 bits (ADJRES = 0). The driver
 *     shifts every result left by (4 - min(n, 4)) so that it is always on a
 *     16-bit scale: 0 .. 0xFFF0 for 0 .. 3.3 V.
 *
 *   Coding conventions: fixed-width types, no magic numbers, braces on every
 *   block, file-scope data and helpers are static, and every wait on the
 *   hardware is bounded, so a fault returns false instead of hanging.
 *
 * References:
 *   - Microchip SAM D21/DA1 Family Data Sheet (DS40001882), ADC chapter:
 *     initialisation, synchronisation, SWTRIG.FLUSH, AVGCTRL table
 *     "Averaging", NVM Software Calibration Area (BIASCAL, LINEARITY).
 *   - Arduino MKR Zero pinout: A0 = PA02.
 * ============================================================================
 */

#include "adc_mkrzero.h"

/* ---------------------------------------------------------------------------
 * Private constants
 * ------------------------------------------------------------------------- */
#define PORT_GROUP_A            (0U)
#define ADC_PIN                 (2U)          /* PA02 = A0 = AIN[0]          */
#define ADC_PIN_MASK            (1UL << ADC_PIN)
#define PMUX_FUNCTION_B         (1U)          /* peripheral function B = ADC */

#define ADC_SAMPLEN_VALUE       (3U)          /* sampling = 2 ADC clocks     */
#define ADC_MAX_EXTRA_BITS      (4U)          /* 12-bit -> 16-bit at most    */
#define ADC_AVG_MAX_LOG2        (10U)         /* 2^10 = 1024 samples         */
#define ADC_LINEARITY_LOW_BITS  (5U)          /* LINEARITY bits 4:0 in word 0 */

/*
 * Bounded waits. One pass of wait_until() takes well over 4 CPU cycles
 * (function call + peripheral read), i.e. > 83 ns at 48 MHz.
 *   Sync / reset / flush : 10 000 passes    > 0.8 ms  (needs < 1 us)
 *   Conversion          : 1 000 000 passes  > 83 ms   (worst case, 1024
 *                         samples x ~6 us = ~6.2 ms)
 */
#define ADC_SYNC_TIMEOUT_LOOPS  (10000UL)
#define ADC_CONV_TIMEOUT_LOOPS  (1000000UL)

/* Flags that must be cleared before a new conversion is started */
#define ADC_RESULT_FLAGS        ((uint8_t)(ADC_INTFLAG_RESRDY | ADC_INTFLAG_OVERRUN))

/* ---------------------------------------------------------------------------
 * Private types and data
 * ------------------------------------------------------------------------- */
typedef bool (*ready_check_t)(void);          /* condition for wait_until() */

static bool    s_initialised = false;         /* init_adc() succeeded       */
static bool    s_pending     = false;         /* started, not yet collected */
static uint8_t s_shift       = ADC_MAX_EXTRA_BITS;   /* scale to 16 bits    */

/* ---------------------------------------------------------------------------
 * Private function prototypes
 * ------------------------------------------------------------------------- */
static bool wait_until(ready_check_t is_ready, uint32_t max_loops);
static bool gclk_sync_idle(void);
static bool adc_sync_idle(void);
static bool adc_reset_done(void);
static bool adc_start_taken(void);
static bool adc_flush_done(void);
static bool adc_result_ready(void);
static bool dac_owns_a0(void);
static void adc_load_calibration(void);
static bool avg_to_log2(uint16_t avg_samples, uint8_t *p_log2n);
static bool adc_apply_averaging(uint8_t log2n);
static bool adc_convert_and_discard(void);

/* ===========================================================================
 * Private helpers
 * ======================================================================== */

/**
 * @brief  Poll is_ready() until it returns true or max_loops passes elapse.
 * @return true if the condition became true in time.
 */
static bool wait_until(ready_check_t is_ready, uint32_t max_loops)
{
    uint32_t loops = 0UL;

    while ((!is_ready()) && (loops < max_loops))
    {
        loops++;
    }
    return is_ready();
}

static bool gclk_sync_idle(void)
{
    return (GCLK->STATUS.bit.SYNCBUSY == 0U);
}

static bool adc_sync_idle(void)
{
    return (ADC->STATUS.bit.SYNCBUSY == 0U);
}

static bool adc_reset_done(void)
{
    return (ADC->CTRLA.bit.SWRST == 0U) && (ADC->STATUS.bit.SYNCBUSY == 0U);
}

static bool adc_start_taken(void)
{
    return (ADC->SWTRIG.bit.START == 0U);
}

static bool adc_flush_done(void)
{
    return (ADC->SWTRIG.bit.FLUSH == 0U) && (ADC->STATUS.bit.SYNCBUSY == 0U);
}

static bool adc_result_ready(void)
{
    return (ADC->INTFLAG.bit.RESRDY != 0U);
}

/**
 * @brief  A0 (PA02) is also the DAC output VOUT. If the DAC is clocked,
 *         enabled and driving the pin, the ADC must not take the pin.
 *         DAC registers are read only when its bus clock is on.
 */
static bool dac_owns_a0(void)
{
    bool owned = false;

    if ((PM->APBCMASK.reg & PM_APBCMASK_DAC) != 0UL)
    {
        owned = (DAC->CTRLA.bit.ENABLE != 0U) && (DAC->CTRLB.bit.EOEN != 0U);
    }
    return owned;
}

/**
 * @brief  Copy the factory calibration from the NVM Software Calibration
 *         Area into ADC->CALIB. The hardware does NOT do this by itself.
 */
static void adc_load_calibration(void)
{
    uint32_t bias;
    uint32_t linearity;

    bias = (*((const volatile uint32_t *)ADC_FUSES_BIASCAL_ADDR) &
            ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;

    linearity = (*((const volatile uint32_t *)ADC_FUSES_LINEARITY_0_ADDR) &
                 ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
    linearity |= ((*((const volatile uint32_t *)ADC_FUSES_LINEARITY_1_ADDR) &
                   ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos)
                 << ADC_LINEARITY_LOW_BITS;

    ADC->CALIB.reg = (uint16_t)(ADC_CALIB_BIAS_CAL(bias) |
                                ADC_CALIB_LINEARITY_CAL(linearity));
}

/**
 * @brief  Convert a sample count (1, 2, 4 ... 1024) to its log2.
 * @return false if avg_samples is not an allowed power of two.
 */
static bool avg_to_log2(uint16_t avg_samples, uint8_t *p_log2n)
{
    uint8_t n;
    bool    valid = false;

    for (n = 0U; n <= ADC_AVG_MAX_LOG2; n++)
    {
        if (avg_samples == (uint16_t)(1U << n))
        {
            *p_log2n = n;
            valid = true;
        }
    }
    return valid;
}

/**
 * @brief  Program 2^log2n hardware averaging. The ADC is disabled while
 *         CTRLB / AVGCTRL change, the registers are read back, the ADC is
 *         re-enabled and its first conversion is discarded.
 * @return true if every step succeeded.
 */
static bool adc_apply_averaging(uint8_t log2n)
{
    uint8_t  extra_bits;
    uint8_t  avgctrl;
    uint16_t ctrlb;
    bool     ok;

    extra_bits = (log2n > ADC_MAX_EXTRA_BITS) ? ADC_MAX_EXTRA_BITS : log2n;

    /* ADJRES = 0: keep the extra bits that averaging produces */
    avgctrl = (uint8_t)(ADC_AVGCTRL_SAMPLENUM(log2n) | ADC_AVGCTRL_ADJRES(0U));

    /* Single-ended, right-adjusted, single shot, no correction, 1.5 MHz.
     * Averaging needs the 16-bit result mode. */
    ctrlb = (uint16_t)(ADC_CTRLB_PRESCALER_DIV32 |
                       ((log2n == 0U) ? ADC_CTRLB_RESSEL_12BIT
                                      : ADC_CTRLB_RESSEL_16BIT));

    ADC->CTRLA.reg = 0U;                               /* disable          */
    ok = wait_until(adc_sync_idle, ADC_SYNC_TIMEOUT_LOOPS);

    ADC->AVGCTRL.reg = avgctrl;
    ADC->CTRLB.reg   = ctrlb;
    ok = wait_until(adc_sync_idle, ADC_SYNC_TIMEOUT_LOOPS) && ok;

    /* Read back: confirms the write actually reached the ADC */
    ok = ok && (ADC->AVGCTRL.reg == avgctrl) && (ADC->CTRLB.reg == ctrlb);

    ADC->CTRLA.reg = (uint8_t)ADC_CTRLA_ENABLE;        /* enable           */
    ok = wait_until(adc_sync_idle, ADC_SYNC_TIMEOUT_LOOPS) && ok;
    ok = ok && (ADC->CTRLA.bit.ENABLE != 0U);

    if (ok)
    {
        s_shift = (uint8_t)(ADC_MAX_EXTRA_BITS - extra_bits);
        ok = adc_convert_and_discard();     /* first result after enable */
    }
    return ok;
}

/**
 * @brief  Run one conversion and throw the result away (used after the
 *         ADC is enabled or reconfigured, as the datasheet requires).
 */
static bool adc_convert_and_discard(void)
{
    bool ok;

    ADC->INTFLAG.reg = ADC_RESULT_FLAGS;
    ADC->SWTRIG.reg  = (uint8_t)ADC_SWTRIG_START;
    ok = wait_until(adc_sync_idle, ADC_SYNC_TIMEOUT_LOOPS);
    ok = ok && wait_until(adc_result_ready, ADC_CONV_TIMEOUT_LOOPS);
    (void)ADC->RESULT.reg;                  /* reading clears RESRDY      */
    return ok;
}

/* ===========================================================================
 * Public routines
 * ======================================================================== */

bool init_adc(void)
{
    bool ok;

    s_initialised = false;
    s_pending     = false;

    /* 1. A0 is shared with the DAC output - refuse if the DAC drives it */
    if (dac_owns_a0())
    {
        return false;
    }

    /* 2. Pin PA02: input, no pull, digital buffer off, function B (analog) */
    PORT->Group[PORT_GROUP_A].DIRCLR.reg = ADC_PIN_MASK;
    PORT->Group[PORT_GROUP_A].PINCFG[ADC_PIN].reg = (uint8_t)PORT_PINCFG_PMUXEN;
    PORT->Group[PORT_GROUP_A].PMUX[ADC_PIN / 2U].reg = (uint8_t)
        ((PORT->Group[PORT_GROUP_A].PMUX[ADC_PIN / 2U].reg &
          (uint8_t)~PORT_PMUX_PMUXE_Msk) | PORT_PMUX_PMUXE(PMUX_FUNCTION_B));

    /* 3. Clocks: APB bus clock + generic clock GCLK0 (48 MHz) to the ADC */
    PM->APBCMASK.reg |= PM_APBCMASK_ADC;
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_ID_ADC |
                                   GCLK_CLKCTRL_GEN_GCLK0 |
                                   GCLK_CLKCTRL_CLKEN);
    if (!wait_until(gclk_sync_idle, ADC_SYNC_TIMEOUT_LOOPS))
    {
        return false;
    }

    /* 4. Software reset: every ADC register to a known state (disabled) */
    ADC->CTRLA.reg = (uint8_t)ADC_CTRLA_SWRST;
    if (!wait_until(adc_reset_done, ADC_SYNC_TIMEOUT_LOOPS))
    {
        return false;
    }

    /* 5. Factory calibration */
    adc_load_calibration();

    /* 6. Analog front end, written explicitly (no default is trusted) */
    ADC->REFCTRL.reg   = (uint8_t)ADC_REFCTRL_REFSEL_INTVCC1;   /* 1.65 V   */
    ADC->SAMPCTRL.reg  = (uint8_t)ADC_SAMPCTRL_SAMPLEN(ADC_SAMPLEN_VALUE);
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN0 |     /* + : AIN0 = A0   */
                         ADC_INPUTCTRL_MUXNEG_GND |      /* - : GND         */
                         ADC_INPUTCTRL_GAIN_DIV2;        /* no scan/offset  */
    ADC->WINCTRL.reg   = (uint8_t)ADC_WINCTRL_WINMODE_DISABLE;
    ADC->EVCTRL.reg    = 0U;                             /* no events       */
    ADC->INTENCLR.reg  = (uint8_t)ADC_INTENCLR_MASK;     /* polling only    */
    ADC->INTFLAG.reg   = (uint8_t)ADC_INTFLAG_MASK;      /* clear old flags */
    ok = wait_until(adc_sync_idle, ADC_SYNC_TIMEOUT_LOOPS);

    ok = ok && (ADC->REFCTRL.reg == (uint8_t)ADC_REFCTRL_REFSEL_INTVCC1);
    ok = ok && (ADC->INPUTCTRL.reg == (ADC_INPUTCTRL_MUXPOS_PIN0 |
                                       ADC_INPUTCTRL_MUXNEG_GND |
                                       ADC_INPUTCTRL_GAIN_DIV2));

    /* 7. 12-bit, 1.5 MHz, no averaging; enable; discard first result */
    ok = ok && adc_apply_averaging(0U);

    s_initialised = ok;
    return ok;
}

bool start_adc(void)
{
    bool ok = false;

    if (s_initialised && (!s_pending))
    {
        ADC->INTFLAG.reg = ADC_RESULT_FLAGS;           /* no stale result  */
        ADC->SWTRIG.reg  = (uint8_t)ADC_SWTRIG_START;
        ok = wait_until(adc_sync_idle, ADC_SYNC_TIMEOUT_LOOPS);
        s_pending = ok;
    }
    return ok;
}

bool stop_adc(void)
{
    bool ok = false;

    if (s_initialised)
    {
        /* After a flush the ADC "resumes where it left off": a START that
         * has not been taken yet would launch a new conversion. So first
         * let START clear (it clears when the conversion begins, < 1 us). */
        (void)wait_until(adc_start_taken, ADC_SYNC_TIMEOUT_LOOPS);

        /* FLUSH aborts the conversion in progress and restarts the ADC
         * clock; the bit reads 1 until the flush has finished. */
        ADC->SWTRIG.reg = (uint8_t)ADC_SWTRIG_FLUSH;
        ok = wait_until(adc_flush_done, ADC_SYNC_TIMEOUT_LOOPS);

        ADC->INTFLAG.reg = ADC_RESULT_FLAGS;           /* drop any result  */
        s_pending = false;
    }
    return ok;
}

bool is_done_adc(void)
{
    return s_initialised && s_pending && adc_result_ready();
}

uint16_t singleread_adc(void)
{
    uint16_t result = ADC_READ_ERROR;
    uint16_t raw;

    if (s_initialised)
    {
        if (!s_pending)
        {
            (void)start_adc();                 /* blocking use: start here */
        }

        if (s_pending)
        {
            if (wait_until(adc_result_ready, ADC_CONV_TIMEOUT_LOOPS))
            {
                raw       = (uint16_t)ADC->RESULT.reg;   /* clears RESRDY */
                result    = (uint16_t)(raw << s_shift);  /* 16-bit scale  */
                s_pending = false;
            }
            else
            {
                (void)stop_adc();              /* timed out: abort it      */
            }
        }
    }
    return result;
}

bool singlewrite_adc(uint16_t avg_samples)
{
    uint8_t log2n = 0U;
    bool    ok    = false;

    if (s_initialised && (!s_pending) && avg_to_log2(avg_samples, &log2n))
    {
        ok = adc_apply_averaging(log2n);

        /* State unknown after a failed write: force a new init_adc() */
        s_initialised = ok;
    }
    return ok;
}