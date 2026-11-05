/*

  File:        main.c
  Author:      Saanjive Sookdeo
  Student ID:  816045952
  Date:        26/11/2026
  Platform:    Arduino MKR Zero 
  Toolchain:   PlatformIO / arm-none-eabi-gcc
 
  Description:
    Table-based cyclic executive ported from Williams, "Real-Time Systems
    Development", Ch. 4, Section 4.7 ("Demonstration Cyclic Executive Code").
 
    The original Linux program relied on the operating system for printf(),
    sleep() and times(). On bare hardware these are replaced by:
      - printf() -> newlib-nano printf, retargeted to SERCOM5 USART
                    (MKR Zero pins 14 = TX, 13 = RX) via _write().
      - times()  -> time_ms(): reads a time structure that the TC3
                    hardware-timer ISR increments every 1 ms.
      - sleep()  -> sleep_s()/sleep_ms(): wait on time_ms().
 

 References:
    - Microchip SAM D21/DA1 Family Data Sheet (DS40001882): sections on
      GCLK, PM, PORT, SERCOM USART and TC.
    - Arduino MKR Zero schematic / pinout (Serial1 = PB22 TX, PB23 RX).
    - newlib documentation: system call _write() for retargeting stdio.
 
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "sam.h"


 // Configuration constants

#define CPU_CLOCK_HZ        (48000000UL)  // GCLK0 set to 48 MHz at start-up 
#define UART_BAUD_RATE      (115200UL)
#define UART_OVERSAMPLING   (16ULL)

#define TICKS_PER_SEC       (1000U)       // 1 tick = 1 ms                   
#define MS_PER_SEC          (1000U)
#define TC3_PRESCALER       (64UL)
#define TC3_TOP_VALUE       ((CPU_CLOCK_HZ / TC3_PRESCALER / TICKS_PER_SEC) - 1UL)

#define SLOT_COUNT          (4U)          // SLOTX in the original        
#define CYCLE_COUNT         (5U)          // CYCLEX in the original       
#define SLOT_TIME_MS        (5000U)       // SLOT_T in the original: 5 s  

// MKR Zero Serial1 pins: PB22 = TX (SERCOM5 PAD2), PB23 = RX (SERCOM5 PAD3) 
#define UART_PORT_GROUP     (1U)          // Port B                          
#define UART_TX_PIN         (22U)
#define UART_RX_PIN         (23U)
#define PMUX_FUNCTION_D     (3U)          // SERCOM-ALT peripheral function  
#define UART_TXPO_PAD2      (1U)
#define UART_RXPO_PAD3      (3U)


 // Time structure, updated by the TC3 interrupt service routine

typedef struct
{
    uint32_t seconds;             
    uint16_t milliseconds;   
} sys_time_t;

static volatile sys_time_t g_sys_time = { 0U, 0U };


 //Scheduler state
 
static uint32_t g_slot_start_ms = 0U;   
static bool     g_slot_done     = false; 

// Function prototypes
  
static void     uart_init(void);
static void     uart_putc(char c);
static void     timer_init(void);
uint32_t        time_ms(void);
void            sleep_ms(uint32_t duration_ms);
void            sleep_s(uint32_t duration_s);

static void     one(void);
static void     two(void);
static void     three(void);
static void     four(void);
static void     five(void);
static void     burn(void);

// Task table: each row is one 5 s slot, executed in round-robin order 
static void (* const task_table[SLOT_COUNT][CYCLE_COUNT])(void) =
{
    { one,  two,   burn, burn, burn },
    { one,  three, burn, burn, burn },
    { one,  four,  burn, burn, burn },
    { burn, burn,  burn, burn, burn }
};

/*  Serial output (SERCOM5 USART) and newlib retargeting*/

// @brief Configure SERCOM5 as an asynchronous USART, 8N1, UART_BAUD_RATE.

static void uart_init(void)
{
    // Route PB22/PB23 to SERCOM5 (peripheral function D) 
    PORT->Group[UART_PORT_GROUP].PINCFG[UART_TX_PIN].reg = PORT_PINCFG_PMUXEN;
    PORT->Group[UART_PORT_GROUP].PINCFG[UART_RX_PIN].reg =
        PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;
    PORT->Group[UART_PORT_GROUP].PMUX[UART_TX_PIN / 2U].reg =
        PORT_PMUX_PMUXE(PMUX_FUNCTION_D) | PORT_PMUX_PMUXO(PMUX_FUNCTION_D);

    // Enable the bus clock and feed SERCOM5 from GCLK0 (48 MHz)
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY) { /* wait for sync */ }

    // Reset the peripheral 
    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST) { /* wait for sync */ }

    // Internal clock, LSB first, TX on PAD2, RX on PAD3, 16x oversampling
    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
                               SERCOM_USART_CTRLA_DORD |
                               SERCOM_USART_CTRLA_TXPO(UART_TXPO_PAD2) |
                               SERCOM_USART_CTRLA_RXPO(UART_RXPO_PAD3);

    // Arithmetic baud: BAUD = 65536 * (1 - 16 * f_baud / f_ref)
    SERCOM5->USART.BAUD.reg = (uint16_t)(65536ULL -
        ((65536ULL * UART_OVERSAMPLING * UART_BAUD_RATE) / CPU_CLOCK_HZ));

    // 8 data bits, 1 stop bit, enable transmitter and receiver
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0U) |
                               SERCOM_USART_CTRLB_TXEN |
                               SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB) { /* wait for sync */ }

    SERCOM5->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE) { /* wait for sync */ }

    // No stdio buffering: each printf() is sent immediately
    (void)setvbuf(stdout, NULL, _IONBF, 0U);
}

/**
 * @brief Blocking transmit of one character.
 */
static void uart_putc(char c)
{
    while (!SERCOM5->USART.INTFLAG.bit.DRE) { /* wait for empty buffer */ }
    SERCOM5->USART.DATA.reg = (uint16_t)(uint8_t)c;
}

/**
 * @brief newlib system call used by printf(); sends output to the UART.
 *        '\n' is expanded to "\r\n" for terminal programs.
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
 * Hardware timer (TC3), ISR, time and sleep functions
 * ======================================================================== */

/**
 * @brief Configure TC3 to interrupt every 1 ms (48 MHz / 64 / 750 = 1 kHz).
 */
static void timer_init(void)
{
    // Enable the bus clock and feed TC3 from GCLK0 (48 MHz)
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3 |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY) { /* wait for sync */ }

    /* Reset the timer */
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC3->COUNT16.CTRLA.bit.SWRST) { /* wait for reset */ }

    /* 16-bit counter, match-frequency mode (TOP = CC0), prescaler /64 */
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                             TC_CTRLA_WAVEGEN_MFRQ |
                             TC_CTRLA_PRESCALER_DIV64 |
                             TC_CTRLA_PRESCSYNC_PRESC;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) { /* wait for sync */ }

    TC3->COUNT16.CC[0].reg = (uint16_t)TC3_TOP_VALUE;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) { /* wait for sync */ }

    /* Interrupt on compare match with CC0 */
    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
    NVIC_SetPriority(TC3_IRQn, 0U);
    NVIC_EnableIRQ(TC3_IRQn);

    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) { /* wait for sync */ }
}

/**
 * @brief TC3 interrupt service routine, called every 1 ms.
 *        The name overrides the weak TC3_Handler entry in the vector table.
 */
void TC3_Handler(void)
{
    if (TC3->COUNT16.INTFLAG.bit.MC0)
    {
        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;   /* clear the flag */

        g_sys_time.milliseconds++;
        if (g_sys_time.milliseconds >= MS_PER_SEC)
        {
            g_sys_time.milliseconds = 0U;
            g_sys_time.seconds++;
        }
    }
}

/**
 * @brief  Replacement for times(): milliseconds since start-up.
 *         Interrupts are briefly disabled so the two fields of the
 *         time structure are read consistently.
 */
uint32_t time_ms(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t now;

    __disable_irq();
    now = (g_sys_time.seconds * MS_PER_SEC) + g_sys_time.milliseconds;
    __set_PRIMASK(primask);

    return now;
}

/**
 * @brief Replacement for sleep(): wait for the given number of milliseconds.
 *        Unsigned subtraction keeps this correct when the counter wraps.
 */
void sleep_ms(uint32_t duration_ms)
{
    uint32_t start = time_ms();

    while ((time_ms() - start) < duration_ms)
    {
        __WFI();    /* sleep until the next 1 ms timer interrupt */
    }
}

/**
 * @brief Wait for the given number of seconds (same units as Linux sleep()).
 */
void sleep_s(uint32_t duration_s)
{
    sleep_ms(duration_s * MS_PER_SEC);
}

/*
  Tasks
*/

static void one(void)
{
    printf("[%8lu ms] task 1 running\n", (unsigned long)time_ms());
    sleep_s(1U);
}

static void two(void)
{
    printf("[%8lu ms] task 2 running\n", (unsigned long)time_ms());
    sleep_s(2U);
}

static void three(void)
{
    printf("[%8lu ms] task 3 running\n", (unsigned long)time_ms());
    sleep_s(3U);
}

static void four(void)
{
    printf("[%8lu ms] task 4 running\n", (unsigned long)time_ms());
    sleep_s(4U);
}

static void five(void)
{
    printf("[%8lu ms] task 5 running\n", (unsigned long)time_ms());
    sleep_s(5U);
}

/**
 * @brief Idle until the end of the current 5 s slot, report the idle time,
 *        then end the slot (original code set cycle = CYCLEX for this).
 */
static void burn(void)
{
    uint32_t burn_start = time_ms();
    uint32_t now        = burn_start;

    while ((now - g_slot_start_ms) < SLOT_TIME_MS)
    {
        now = time_ms();    /* burn time here */
    }

    printf("burn time = %lums\n\n", (unsigned long)(now - burn_start));

    g_slot_start_ms = now;
    g_slot_done     = true;
}

/* 
  Main: cyclic executive
  */

int main(void)
{
    uint32_t slot;
    uint32_t cycle;

    uart_init();
    timer_init();

    printf("\nCyclic executive - Arduino MKR Zero\n");
    printf("clock ticks/sec = %u\n\n", TICKS_PER_SEC);

    (void)five;     // Task 5 is defined but not in the book's task table 

    g_slot_start_ms = time_ms();

    while (true)
    {
        for (slot = 0U; slot < SLOT_COUNT; slot++)
        {
            g_slot_done = false;
            for (cycle = 0U; (cycle < CYCLE_COUNT) && !g_slot_done; cycle++)
            {
                task_table[slot][cycle]();
            }
        }
    }

    return 0;
}