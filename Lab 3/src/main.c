/*
 * ============================================================================
 * File:        main.c
 * Author:      <Your Name>
 * Student ID:  <Your ID>
 * Date:        19 November 2026
 * Platform:    Arduino MKR Zero (ATSAMD21G18A, ARM Cortex-M0+, 48 MHz)
 * Toolchain:   PlatformIO / arm-none-eabi-gcc, newlib-nano C library
 *
 * Description:
 *   Unit test for the ADC driver (adc_mkrzero.h / adc_mkrzero.c).
 *
 *   Part 1 - calls made BEFORE init_adc() must fail safely.
 *   Part 2 - init_adc() must succeed; bad singlewrite_adc() arguments must
 *            be rejected; the largest averaging (1024) must still read.
 *   Part 3 - forever, for each averaging setting 1, 2, 4 ... 64:
 *              WRITE  singlewrite_adc(avg)                          -> true
 *              READ   singleread_adc()            (blocking)        -> valid
 *              READ   start_adc(), poll is_done_adc(), then
 *                     singleread_adc()            (asynchronous)    -> valid
 *              BUSY   while a conversion is pending, a second
 *                     start_adc() and a singlewrite_adc() are refused
 *              STOP   stop_adc() aborts it, is_done_adc() stays false
 *            One table row is printed per setting, with a PASS/FAIL total
 *            per sweep. The built-in LED toggles after every sweep.
 *
 *   Hardware:
 *     Potentiometer : ends to 3.3 V and GND, wiper to A0 (PA02)
 *     UART          : SERCOM5, pin 14 = TX (PB22), pin 13 = RX (PB23),
 *                     115200 8N1, to a USB-serial adapter (common GND)
 *     LED           : LED_BUILTIN = PB08
 *
 *   The Arduino library (Arduino.h) is NOT used. Only CMSIS device headers
 *   (sam.h), the ADC driver and the standard C library are included.
 * ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "sam.h"
#include "adc_mkrzero.h"

/* ---------------------------------------------------------------------------
 * Configuration constants
 * ------------------------------------------------------------------------- */
#define CPU_CLOCK_HZ          (48000000UL)  /* GCLK0 set to 48 MHz at start-up */
#define UART_BAUD_RATE        (115200UL)
#define UART_OVERSAMPLING     (16ULL)

#define PORT_GROUP_B          (1U)
#define UART_TX_PIN           (22U)         /* PB22 = SERCOM5 PAD2          */
#define UART_RX_PIN           (23U)         /* PB23 = SERCOM5 PAD3          */
#define PMUX_FUNCTION_D       (3U)          /* SERCOM-ALT                   */
#define UART_TXPO_PAD2        (1U)
#define UART_RXPO_PAD3        (3U)

#define LED_PIN               (8U)          /* PB08 = LED_BUILTIN           */
#define LED_MASK              (1UL << LED_PIN)

#define SYSTICK_TICKS_PER_MS  (CPU_CLOCK_HZ / 1000UL)
#define SWEEP_PERIOD_MS       (1000UL)      /* pause between sweeps         */
#define FAULT_BLINK_MS        (100UL)       /* fast blink = init failed     */
#define POLL_LIMIT            (1000000UL)   /* is_done_adc() polls allowed  */

/* Averaging settings written in every sweep (64 x ~6 us fits a 1 ms slot) */
static const uint16_t k_avg_settings[] = { 1U, 2U, 4U, 8U, 16U, 32U, 64U };
#define NUM_AVG_SETTINGS      (sizeof(k_avg_settings) / sizeof(k_avg_settings[0]))

/* ---------------------------------------------------------------------------
 * Test bookkeeping
 * ------------------------------------------------------------------------- */
static uint32_t s_checks = 0UL;
static uint32_t s_passed = 0UL;
static uint32_t s_failed = 0UL;

/* ---------------------------------------------------------------------------
 * Function prototypes
 * ------------------------------------------------------------------------- */
static void     uart_init(void);
static void     uart_putc(char c);
int             _write(int file, char *ptr, int len);   /* newlib hook */
static void     led_init(void);
static void     led_toggle(void);
static void     delay_ms(uint32_t ms);
static void     halt_with_fast_blink(void);
static bool     check(bool condition, const char *p_name);
static uint32_t to_millivolts(uint16_t value);
static void     test_one_setting(uint16_t avg);

/* ===========================================================================
 * Serial output (SERCOM5 USART) and newlib retargeting
 * ======================================================================== */

/**
 * @brief Configure SERCOM5 as an asynchronous USART, 8N1, UART_BAUD_RATE.
 */
static void uart_init(void)
{
    PORT->Group[PORT_GROUP_B].PINCFG[UART_TX_PIN].reg = PORT_PINCFG_PMUXEN;
    PORT->Group[PORT_GROUP_B].PINCFG[UART_RX_PIN].reg =
        PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;
    PORT->Group[PORT_GROUP_B].PMUX[UART_TX_PIN / 2U].reg =
        PORT_PMUX_PMUXE(PMUX_FUNCTION_D) | PORT_PMUX_PMUXO(PMUX_FUNCTION_D);

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY) { /* wait for sync */ }

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST) { /* wait for sync */ }

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
                               SERCOM_USART_CTRLA_DORD |
                               SERCOM_USART_CTRLA_TXPO(UART_TXPO_PAD2) |
                               SERCOM_USART_CTRLA_RXPO(UART_RXPO_PAD3);

    /* Arithmetic baud: BAUD = 65536 * (1 - 16 * f_baud / f_ref) */
    SERCOM5->USART.BAUD.reg = (uint16_t)(65536ULL -
        ((65536ULL * UART_OVERSAMPLING * UART_BAUD_RATE) / CPU_CLOCK_HZ));

    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0U) |
                               SERCOM_USART_CTRLB_TXEN |
                               SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB) { /* wait for sync */ }

    SERCOM5->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE) { /* wait for sync */ }

    (void)setvbuf(stdout, NULL, _IONBF, 0U);
}

static void uart_putc(char c)
{
    while (!SERCOM5->USART.INTFLAG.bit.DRE) { /* wait for empty buffer */ }
    SERCOM5->USART.DATA.reg = (uint16_t)(uint8_t)c;
}

/**
 * @brief newlib system call used by printf(); '\n' is sent as "\r\n".
 */
int _write(int file, char *ptr, int len)
{
    int i;

    (void)file;
    for (i = 0; i < len; i++)
    {
        if (ptr[i] == '\n')
        {
            uart_putc('\r');
        }
        uart_putc(ptr[i]);
    }
    return len;
}

/* ===========================================================================
 * LED and timing
 * ======================================================================== */

static void led_init(void)
{
    PORT->Group[PORT_GROUP_B].PINCFG[LED_PIN].reg = 0U;       /* plain GPIO */
    PORT->Group[PORT_GROUP_B].DIRSET.reg = LED_MASK;
    PORT->Group[PORT_GROUP_B].OUTCLR.reg = LED_MASK;
}

static void led_toggle(void)
{
    PORT->Group[PORT_GROUP_B].OUTTGL.reg = LED_MASK;
}

/**
 * @brief Busy-wait delay using SysTick in polled mode (no interrupt).
 */
static void delay_ms(uint32_t ms)
{
    uint32_t i;

    SysTick->CTRL = 0UL;
    SysTick->LOAD = SYSTICK_TICKS_PER_MS - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    for (i = 0UL; i < ms; i++)
    {
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
        {
            /* wait 1 ms */
        }
    }
    SysTick->CTRL = 0UL;
}

/**
 * @brief Stop here with the LED blinking fast (visible fault indicator).
 */
static void halt_with_fast_blink(void)
{
    while (true)
    {
        led_toggle();
        delay_ms(FAULT_BLINK_MS);
    }
}

/* ===========================================================================
 * Test helpers
 * ======================================================================== */

/**
 * @brief  Count one check. Prints a PASS/FAIL line when p_name != NULL.
 * @return condition (so the caller can use the outcome).
 */
static bool check(bool condition, const char *p_name)
{
    s_checks++;
    if (condition)
    {
        s_passed++;
    }
    else
    {
        s_failed++;
    }

    if (p_name != NULL)
    {
        printf("  [%s] %s\n", condition ? "PASS" : "FAIL", p_name);
    }
    return condition;
}

/**
 * @brief  16-bit-scaled ADC result -> millivolts (0 .. 3300).
 */
static uint32_t to_millivolts(uint16_t value)
{
    return ((uint32_t)value * ADC_VREF_MV) / ADC_FULL_SCALE;
}

/**
 * @brief  Write one averaging setting, then read the ADC two ways and test
 *         the busy and stop behaviour. Prints one table row.
 */
static void test_one_setting(uint16_t avg)
{
    bool     write_ok;
    bool     async_ok    = false;
    bool     busy_ok     = false;
    bool     stop_ok;
    uint16_t blocking_value;
    uint16_t async_value = ADC_READ_ERROR;
    uint32_t polls       = 0UL;

    /* WRITE: one configuration word (hardware averaging) */
    write_ok = check(singlewrite_adc(avg), NULL);

    /* READ 1: blocking - singleread_adc() starts and waits by itself */
    blocking_value = singleread_adc();
    (void)check(blocking_value != ADC_READ_ERROR, NULL);

    /* READ 2: asynchronous - start, poll while the ADC works, collect */
    if (start_adc())
    {
        while ((!is_done_adc()) && (polls < POLL_LIMIT))
        {
            polls++;
        }
        async_value = singleread_adc();
        async_ok = (polls < POLL_LIMIT) && (async_value != ADC_READ_ERROR);
    }
    (void)check(async_ok, NULL);

    /* BUSY: a pending conversion must block a new start and a write */
    if (start_adc())
    {
        busy_ok = (!start_adc()) && (!singlewrite_adc(avg));
    }
    (void)check(busy_ok, NULL);

    /* STOP: abort the pending conversion - no result may appear */
    stop_ok = check(stop_adc() && (!is_done_adc()), NULL);

    printf("  %4u | %-4s  | %5u = %4lu mV   | %5u = %4lu mV %7lu | %-4s | %-4s\n",
           (unsigned int)avg,
           write_ok ? "OK" : "FAIL",
           (unsigned int)blocking_value,
           (unsigned long)to_millivolts(blocking_value),
           (unsigned int)async_value,
           (unsigned long)to_millivolts(async_value),
           (unsigned long)polls,
           busy_ok ? "OK" : "FAIL",
           stop_ok ? "OK" : "FAIL");
}

/* ===========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    uint32_t sweep = 0UL;
    uint32_t failed_before;
    uint32_t i;
    bool     init_ok;

    uart_init();
    led_init();

    printf("\n==== ADC driver unit test - Arduino MKR Zero ====\n");
    printf("Author: <Your Name>   ID: <Your ID>\n\n");

    printf("Part 1: calls before init_adc() must fail safely\n");
    (void)check(!start_adc(),   "start_adc() refused");
    (void)check(!stop_adc(),    "stop_adc() refused");
    (void)check(!is_done_adc(), "is_done_adc() returns false");
    (void)check(singleread_adc() == ADC_READ_ERROR,
                "singleread_adc() returns ADC_READ_ERROR");
    (void)check(!singlewrite_adc(ADC_AVG_MIN), "singlewrite_adc(1) refused");

    printf("\nPart 2: initialisation and argument checks\n");
    init_ok = check(init_adc(), "init_adc() succeeds");
    if (!init_ok)
    {
        printf("init_adc() failed (is the DAC driving A0?). Halting.\n");
        halt_with_fast_blink();
    }
    (void)check(!singlewrite_adc(0U),    "singlewrite_adc(0) rejected");
    (void)check(!singlewrite_adc(3U),    "singlewrite_adc(3) rejected (not 2^n)");
    (void)check(!singlewrite_adc(2048U), "singlewrite_adc(2048) rejected (> 1024)");
    (void)check(singlewrite_adc(ADC_AVG_MAX), "singlewrite_adc(1024) accepted");
    (void)check(singleread_adc() != ADC_READ_ERROR,
                "singleread_adc() completes at 1024x averaging");

    printf("\nPart 3: repeated write + read sweeps (turn the knob on A0)\n");
    printf("Results are 16-bit scaled: 65536 codes = 3300 mV\n");

    while (true)
    {
        sweep++;
        failed_before = s_failed;

        printf("\n--- Sweep %lu ---\n", (unsigned long)sweep);
        printf("   avg | write | blocking read     | async read        polls | busy | stop\n");
        for (i = 0UL; i < NUM_AVG_SETTINGS; i++)
        {
            test_one_setting(k_avg_settings[i]);
        }

        printf("Sweep %lu %s | total: %lu checks, %lu passed, %lu failed\n",
               (unsigned long)sweep,
               (s_failed == failed_before) ? "PASS" : "FAIL",
               (unsigned long)s_checks,
               (unsigned long)s_passed,
               (unsigned long)s_failed);

        led_toggle();
        delay_ms(SWEEP_PERIOD_MS);
    }

    return 0;   /* never reached */
}