/*
 * ============================================================================
 * File:        main.c
 * Author:      <Your Name>
 * Student ID:  <Your ID>
 * Date:        <Date>
 * Platform:    Arduino MKR Zero (ATSAMD21G18A, ARM Cortex-M0+, 48 MHz)
 * Toolchain:   PlatformIO / arm-none-eabi-gcc, FreeRTOS V10.2.1
 *              (FreeRTOS_SAMD21 port, copied into lib/)
 *
 * Description:
 *   FreeRTOS example with three tasks and one mutex.
 *
 *   - vLedOnTask  : takes the GPIO mutex, turns the LED ON, ACTIVELY waits
 *                   (busy loop, CPU kept busy) for 0.5 s, gives the mutex
 *                   back and then yields with taskYIELD().
 *   - vLedOffTask : takes the GPIO mutex, turns the LED OFF, gives the
 *                   mutex back and task-delays (blocks) for 1 s.
 *   - vStatusTask : prints a status message on the serial UART and
 *                   task-delays for 1 s.
 *
 *   The LED pin is shared by the first two tasks, so every write to it is
 *   protected by a FreeRTOS mutex. By default it is header pin D6 (PA20)
 *   with an external LED + 220-330 ohm resistor to GND, so the pin can be
 *   probed with an oscilloscope. Set LED_ON_HEADER_D6 to 0 to use the
 *   built-in LED (PB08) instead.
 *
 *   Priorities (each task has its own level, set by a const int):
 *     vLedOnTask = 1 (lowest), vLedOffTask = 2, vStatusTask = 3 (highest).
 *   vLedOnTask never blocks (it busy-waits), so it MUST have the lowest
 *   priority: if it were higher, the other two tasks would never run.
 *   The other two tasks block in vTaskDelay(), which lets vLedOnTask run.
 *   When vLedOffTask wakes up while vLedOnTask holds the mutex, the mutex's
 *   priority inheritance raises vLedOnTask to priority 2 until it gives
 *   the mutex back.
 *
 *   Serial output: SERCOM5 USART, 115200 8N1, MKR Zero pins
 *   14 = TX (PB22) and 13 = RX (PB23).
 *
 *   The Arduino library (Arduino.h) is NOT used. Only CMSIS device headers
 *   (sam.h), the FreeRTOS kernel and the standard C library are included.
 *
 * References:
 *   - https://www.freertos.org/implementing-a-FreeRTOS-task.html
 *   - https://freertos.org/a00019.html       (xTaskCreate)
 *   - https://freertos.org/a00122.html       (xSemaphoreCreateMutex)
 *   - Microchip SAM D21 Family Data Sheet (DS40001882): PORT, GCLK, SERCOM
 * ============================================================================
 */

#include <stdint.h>
#include <stdio.h>
#include "sam.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ---------------------------------------------------------------------------
 * Task priorities - one level per task, each set by a const int
 * ------------------------------------------------------------------------- */
static const int LED_ON_TASK_PRIORITY  = 1;   /* lowest: busy-waits      */
static const int LED_OFF_TASK_PRIORITY = 2;
static const int STATUS_TASK_PRIORITY  = 3;   /* highest                 */

/* ---------------------------------------------------------------------------
 * Timing and stack sizes
 * ------------------------------------------------------------------------- */
#define LED_ON_TIME_MS          (500U)    /* active wait with LED on     */
#define LED_OFF_DELAY_MS        (1000U)   /* task-delay after LED off    */
#define STATUS_DELAY_MS         (1000U)   /* task-delay after printing   */

#define LED_TASK_STACK_WORDS    (configMINIMAL_STACK_SIZE)
#define STATUS_TASK_STACK_WORDS (256U)    /* snprintf() needs more stack */

/* ---------------------------------------------------------------------------
 * Hardware definitions
 * ------------------------------------------------------------------------- */
#define CPU_CLOCK_HZ            (48000000UL)
#define UART_BAUD_RATE          (115200UL)
#define UART_OVERSAMPLING       (16ULL)

/* Shared LED pin: 1 = header pin D6 (PA20) with an external LED +
 * 220-330 ohm resistor (can be probed with a scope),
 * 0 = built-in LED, MKR Zero pin 32 = PB08 (not on a header pin). */
#define LED_ON_HEADER_D6        1
#if LED_ON_HEADER_D6
#define LED_PORT_GROUP          (0U)      /* Port A */
#define LED_PIN                 (20U)     /* PA20 = MKR pin D6 */
#else
#define LED_PORT_GROUP          (1U)      /* Port B */
#define LED_PIN                 (8U)      /* PB08 = built-in LED */
#endif
#define LED_PIN_MASK            (1UL << LED_PIN)

/* Serial1 pins: PB22 = TX (SERCOM5 PAD2), PB23 = RX (SERCOM5 PAD3) */
#define UART_PORT_GROUP         (1U)      /* Port B */
#define UART_TX_PIN             (22U)
#define UART_RX_PIN             (23U)
#define PMUX_FUNCTION_D         (3U)      /* SERCOM-ALT function     */
#define UART_TXPO_PAD2          (1U)
#define UART_RXPO_PAD3          (3U)

#define STATUS_BUFFER_SIZE      (96U)

/* ---------------------------------------------------------------------------
 * Shared objects
 * ------------------------------------------------------------------------- */
static SemaphoreHandle_t xLedMutex = NULL;     /* protects the LED GPIO pin */

/* Counters written only while the mutex is held, read by the status task */
static volatile uint32_t ulLedOnCount  = 0U;
static volatile uint32_t ulLedOffCount = 0U;

/* ---------------------------------------------------------------------------
 * Function prototypes
 * ------------------------------------------------------------------------- */
static void led_init(void);
static void led_on(void);
static void led_off(void);
static void uart_init(void);
static void uart_putc(char c);
static void uart_puts(const char *str);

static void vLedOnTask(void *pvParameters);
static void vLedOffTask(void *pvParameters);
static void vStatusTask(void *pvParameters);

void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

/* ===========================================================================
 * GPIO (LED)
 * ======================================================================== */

/**
 * @brief Configure the LED pin as a digital output, LED initially off.
 */
static void led_init(void)
{
    PORT->Group[LED_PORT_GROUP].DIRSET.reg = LED_PIN_MASK;
    PORT->Group[LED_PORT_GROUP].OUTCLR.reg = LED_PIN_MASK;
}

/** @brief Drive the LED pin high. Caller must hold xLedMutex. */
static void led_on(void)
{
    PORT->Group[LED_PORT_GROUP].OUTSET.reg = LED_PIN_MASK;
}

/** @brief Drive the LED pin low. Caller must hold xLedMutex. */
static void led_off(void)
{
    PORT->Group[LED_PORT_GROUP].OUTCLR.reg = LED_PIN_MASK;
}

/* ===========================================================================
 * Serial output (SERCOM5 USART, polled)
 * ======================================================================== */

/**
 * @brief Configure SERCOM5 as an asynchronous USART, 8N1, UART_BAUD_RATE.
 */
static void uart_init(void)
{
    /* Route PB22/PB23 to SERCOM5 (peripheral function D) */
    PORT->Group[UART_PORT_GROUP].PINCFG[UART_TX_PIN].reg = PORT_PINCFG_PMUXEN;
    PORT->Group[UART_PORT_GROUP].PINCFG[UART_RX_PIN].reg =
        PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;
    PORT->Group[UART_PORT_GROUP].PMUX[UART_TX_PIN / 2U].reg =
        PORT_PMUX_PMUXE(PMUX_FUNCTION_D) | PORT_PMUX_PMUXO(PMUX_FUNCTION_D);

    /* Enable the bus clock and feed SERCOM5 from GCLK0 (48 MHz) */
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY) { /* wait for sync */ }

    /* Reset the peripheral */
    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST) { /* wait for sync */ }

    /* Internal clock, LSB first, TX on PAD2, RX on PAD3, 16x oversampling */
    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
                               SERCOM_USART_CTRLA_DORD |
                               SERCOM_USART_CTRLA_TXPO(UART_TXPO_PAD2) |
                               SERCOM_USART_CTRLA_RXPO(UART_RXPO_PAD3);

    /* Arithmetic baud: BAUD = 65536 * (1 - 16 * f_baud / f_ref) */
    SERCOM5->USART.BAUD.reg = (uint16_t)(65536ULL -
        ((65536ULL * UART_OVERSAMPLING * UART_BAUD_RATE) / CPU_CLOCK_HZ));

    /* 8 data bits, 1 stop bit, enable transmitter and receiver */
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0U) |
                               SERCOM_USART_CTRLB_TXEN |
                               SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB) { /* wait for sync */ }

    SERCOM5->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE) { /* wait for sync */ }
}

/** @brief Blocking transmit of one character. */
static void uart_putc(char c)
{
    while (!SERCOM5->USART.INTFLAG.bit.DRE) { /* wait for empty buffer */ }
    SERCOM5->USART.DATA.reg = (uint16_t)(uint8_t)c;
}

/** @brief Transmit a string, expanding '\n' to "\r\n" for terminals. */
static void uart_puts(const char *str)
{
    while (*str != '\0')
    {
        if (*str == '\n')
        {
            uart_putc('\r');
        }
        uart_putc(*str);
        str++;
    }
}

/* ===========================================================================
 * Tasks
 * ======================================================================== */

/**
 * @brief Task 1: LED on, actively wait 0.5 s, then yield.
 *        The mutex is held for the whole 0.5 s so the other task cannot
 *        change the pin while it is supposed to be on.
 */
static void vLedOnTask(void *pvParameters)
{
    const TickType_t xOnTime = pdMS_TO_TICKS(LED_ON_TIME_MS);
    TickType_t xStart;

    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE)
        {
            led_on();
            ulLedOnCount++;

            /* Active (busy) wait: the task stays Running and uses the CPU;
             * it does not block like vTaskDelay() would. */
            xStart = xTaskGetTickCount();
            while ((xTaskGetTickCount() - xStart) < xOnTime)
            {
                /* spin */
            }

            (void)xSemaphoreGive(xLedMutex);
        }

        taskYIELD();
    }
}

/**
 * @brief Task 2: LED off, then task-delay (block) for 1 s.
 */
static void vLedOffTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE)
        {
            led_off();
            ulLedOffCount++;
            (void)xSemaphoreGive(xLedMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(LED_OFF_DELAY_MS));
    }
}

/**
 * @brief Task 3: print a status message on the UART, then task-delay 1 s.
 *        Only this task uses the UART, so it needs no mutex.
 */
static void vStatusTask(void *pvParameters)
{
    char acBuffer[STATUS_BUFFER_SIZE];

    (void)pvParameters;

    for (;;)
    {
        (void)snprintf(acBuffer, sizeof(acBuffer),
                       "[%7lu ms] status: LED on-count=%lu off-count=%lu\n",
                       (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                       (unsigned long)ulLedOnCount,
                       (unsigned long)ulLedOffCount);
        uart_puts(acBuffer);

        vTaskDelay(pdMS_TO_TICKS(STATUS_DELAY_MS));
    }
}

/* ===========================================================================
 * FreeRTOS hook functions (enabled in FreeRTOSConfig.h)
 * ======================================================================== */

/** @brief Called if pvPortMalloc() fails: stop with the LED on. */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    led_on();
    for (;;) { }
}

/** @brief Called if a task overflows its stack: stop with the LED on. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    led_on();
    for (;;) { }
}

/* ===========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    BaseType_t xOk = pdPASS;

    led_init();
    uart_init();
    uart_puts("\nFreeRTOS 3-task mutex demo - Arduino MKR Zero\n");

    /* Create the mutex that manages the LED GPIO pin */
    xLedMutex = xSemaphoreCreateMutex();
    configASSERT(xLedMutex != NULL);

    /* Create the three tasks, each with its own priority */
    xOk &= xTaskCreate(vLedOnTask,  "LedOn",  LED_TASK_STACK_WORDS, NULL,
                       (UBaseType_t)LED_ON_TASK_PRIORITY,  NULL);
    xOk &= xTaskCreate(vLedOffTask, "LedOff", LED_TASK_STACK_WORDS, NULL,
                       (UBaseType_t)LED_OFF_TASK_PRIORITY, NULL);
    xOk &= xTaskCreate(vStatusTask, "Status", STATUS_TASK_STACK_WORDS, NULL,
                       (UBaseType_t)STATUS_TASK_PRIORITY,  NULL);
    configASSERT(xOk == pdPASS);

    /* Start the scheduler - does not return while the RTOS is running */
    vTaskStartScheduler();

    /* Only reached if there was not enough heap for the idle task */
    for (;;) { }

    return 0;
}