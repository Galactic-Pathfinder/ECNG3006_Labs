/*
 * ============================================================================
 * File:        main.c
 * Author:      Saanjive Sookdeo
 * Student ID:  816045952
 * Date:        16/11/26
 * Platform:    Arduino MKR Zero (ATSAMD21G18A, ARM Cortex-M0+, 48 MHz)
 * Toolchain:   PlatformIO / arm-none-eabi-gcc, FreeRTOS V10.2.1
 *              (FreeRTOS_SAMD21 library via lib_deps; the application's
 *              include/FreeRTOSConfig.h is force-included, see that file) *
 * Description:
 *   Question 3 - relative task performance, using the "LED owns the lock"
 *   design that gives a visible blink: LED ON 0.5 s, OFF 1 s (1.5 s cycle).
 *
 *     Task 1 (vTask1LedOff) : take lock, LED OFF, task-delay 1 s while
 *                             still holding the lock, give lock, then
 *                             sleep (2 ticks at a time) until Task 3 has
 *                             taken the lock.
 *     Task 2 (vTask2Status) : print a status line on the UART,
 *                             task-delay 1 s. (does not use the lock)
 *     Task 3 (vTask3LedOn)  : take lock, LED ON, ACTIVELY wait 0.5 s,
 *                             give lock, yield.
 *
 *   Why this numbering: Task 3 busy-waits. If it were the highest
 *   priority it would give the lock, yield to nobody and take the lock
 *   straight back, so Tasks 1 and 2 would never run in cases (a)-(c).
 *   With Task 3 lowest, Task 1 (high) and Task 3 (low) share the lock and
 *   Task 2 (medium) does not: the classic priority-inversion set-up.
 *
 *   Experiment (EXPERIMENT, one PlatformIO environment each):
 *     0 same priority, run to completion  : co-operative, mutex
 *     1 (a) T1>T2>T3, run to completion   : co-operative, mutex
 *     2 (b) T1>T2>T3, no inheritance      : preemptive, BINARY SEMAPHORE
 *     3 (c) T1>T2>T3, inheritance         : preemptive, MUTEX
 *   FreeRTOS mutexes always use priority inheritance; a binary semaphore
 *   is the same lock without it, so case (b) uses a binary semaphore.
 *
 *   The active wait burns a calibrated amount of CPU work (0.5 s when it
 *   runs uninterrupted). If Task 3 is preempted while the LED is on, the
 *   work takes longer, so the LED stays on longer than 500 ms - that is
 *   how priority inversion shows up in the measurements.
 *
 * Evidence produced:
 *   - UART status line every second (115200 8N1, pin 14 = TX):
 *       on_ms / off_ms     : last measured LED on / off time
 *       on_max             : longest LED on time (> 500 = Task 3 was
 *                            preempted while holding the lock)
 *       t2_lat / t2_max    : Task 2 lateness: wake-up -> start of print
 *   - Trace pins, HIGH while that task is running (oscilloscope):
 *       D0 (PA22) = Task 1,  D1 (PA23) = Task 2,  D2 (PA10) = Task 3
 *   - Shared LED pin: D6 (PA20) + external LED and 220-330 ohm resistor
 *     (LED_ON_HEADER_D6 = 0 uses the built-in LED PB08 instead).
 *
 *   The Arduino library (Arduino.h) is NOT used.
 *   Works as main.c (C) or main.cpp (C++): the FreeRTOS hook functions are
 *   wrapped in extern "C" so the kernel can find them in either case.
 * ============================================================================
 */

#include <stdint.h>
#include "sam.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#ifndef EXPERIMENT
#define EXPERIMENT              3
#endif

/* Stop with a clear message if the application FreeRTOSConfig.h is not the
 * one in use. Checks the settings this program needs (task tags for the
 * trace pins, and the scheduler mode chosen by EXPERIMENT); the library's
 * own config fails both, any version of the application config passes. */
#if (configUSE_APPLICATION_TASK_TAG != 1)
#error "Wrong FreeRTOSConfig.h in use (task tags off): include/FreeRTOSConfig.h must hold the application config, and platformio.ini must have -include include/FreeRTOSConfig.h"
#endif
#if (((EXPERIMENT == 0) || (EXPERIMENT == 1)) != (configUSE_PREEMPTION == 0))
#error "configUSE_PREEMPTION does not match EXPERIMENT: use the application include/FreeRTOSConfig.h (it sets preemption from EXPERIMENT)"
#endif

/* ---------------------------------------------------------------------------
 * Task priorities - one const int per task, set by the experiment
 * ------------------------------------------------------------------------- */
#if (EXPERIMENT == 0)
static const int TASK1_PRIORITY = 1;      /* all the SAME priority       */
static const int TASK2_PRIORITY = 1;
static const int TASK3_PRIORITY = 1;
#else
static const int TASK1_PRIORITY = 3;      /* Task 1 highest              */
static const int TASK2_PRIORITY = 2;      /* then Task 2                 */
static const int TASK3_PRIORITY = 1;      /* then Task 3                 */
#endif

#if (EXPERIMENT == 2)
#define USE_BINARY_SEMAPHORE    1         /* lock WITHOUT inheritance    */
#else
#define USE_BINARY_SEMAPHORE    0         /* mutex: WITH inheritance     */
#endif

static const char * const EXPERIMENT_NAME[] =
{
    "SAME priority, run to completion (co-operative, mutex)",
    "(a) T1>T2>T3, run to completion (co-operative, mutex)",
    "(b) T1>T2>T3, preemptive, NO inheritance (binary semaphore)",
    "(c) T1>T2>T3, preemptive, priority inheritance (mutex)"
};

/* ---------------------------------------------------------------------------
 * Timing and stack sizes
 * ------------------------------------------------------------------------- */
#define LED_ON_TIME_MS          (500U)    /* Task 3 active wait          */
#define LED_OFF_TIME_MS         (1000U)   /* Task 1 task-delay           */
#define HANDOFF_TICKS           (2U)      /* Task 1 sleep after giving   */
#define STATUS_DELAY_MS         (1000U)   /* Task 2 task-delay           */
#define TASK_STACK_WORDS        (configMINIMAL_STACK_SIZE)
#define CALIBRATION_LOOPS       (20000UL)

/* ---------------------------------------------------------------------------
 * Hardware definitions
 * ------------------------------------------------------------------------- */
#define CPU_CLOCK_HZ            (48000000UL)
#define UART_BAUD_RATE          (115200UL) /* lower it (e.g. 9600) to make
                                              Task 2 use more CPU time    */
#define UART_OVERSAMPLING       (16ULL)

#define LED_ON_HEADER_D6        1
#if LED_ON_HEADER_D6
#define LED_PORT_GROUP          (0U)      /* Port A */
#define LED_PIN                 (20U)     /* PA20 = MKR pin D6 */
#else
#define LED_PORT_GROUP          (1U)      /* Port B */
#define LED_PIN                 (8U)      /* PB08 = built-in LED */
#endif
#define LED_PIN_MASK            (1UL << LED_PIN)

#define TRACE_PORT_GROUP        (0U)           /* Port A */
#define TRACE_TASK1_MASK        (1UL << 22U)   /* D0 */
#define TRACE_TASK2_MASK        (1UL << 23U)   /* D1 */
#define TRACE_TASK3_MASK        (1UL << 10U)   /* D2 */
#define TRACE_ALL_MASK          (TRACE_TASK1_MASK | TRACE_TASK2_MASK | \
                                 TRACE_TASK3_MASK)

#define UART_PORT_GROUP         (1U)      /* PB22 = TX, PB23 = RX */
#define UART_TX_PIN             (22U)
#define UART_RX_PIN             (23U)
#define PMUX_FUNCTION_D         (3U)
#define UART_TXPO_PAD2          (1U)
#define UART_RXPO_PAD3          (3U)

/* ---------------------------------------------------------------------------
 * Shared objects and measurements (1 tick = 1 ms)
 * ------------------------------------------------------------------------- */
static SemaphoreHandle_t xLedLock = NULL;   /* the "right to own the LED" */
static volatile uint32_t ulLoopsPerMs = 1U;

static volatile uint32_t onStartTick  = 0U; /* last OFF->ON edge          */
static volatile uint32_t offStartTick = 0U; /* last ON->OFF edge          */
static volatile uint32_t onMs   = 0U;       /* last LED on time           */
static volatile uint32_t offMs  = 0U;       /* last LED off time          */
static volatile uint32_t onMax  = 0U;       /* longest LED on time        */
static volatile uint32_t t2Lat  = 0U;       /* Task 2 last lateness       */
static volatile uint32_t t2Max  = 0U;       /* Task 2 worst lateness      */

/* ---------------------------------------------------------------------------
 * Function prototypes
 * ------------------------------------------------------------------------- */
static void gpio_init(void);
static void led_on(void);
static void led_off(void);
static void uart_init(void);
static void uart_puts(const char *s);
static void uart_putu(uint32_t v);
/* noinline + aligned: calibration and the task must run the SAME machine
 * code. If the compiler inlines a separate copy of the loop into each
 * caller, the copies sit at different flash addresses and can run at
 * different speeds (flash wait states), making the active wait wrong. */
static void burn_loops(uint32_t loops) __attribute__((noinline, aligned(16)));
static void calibrate_active_wait(void);

static void vTask1LedOff(void *pvParameters);
static void vTask2Status(void *pvParameters);
static void vTask3LedOn(void *pvParameters);

#ifdef __cplusplus
extern "C" {
#endif
void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
#ifdef __cplusplus
}
#endif

/* ===========================================================================
 * GPIO
 * ======================================================================== */

static void gpio_init(void)
{
    PORT->Group[LED_PORT_GROUP].DIRSET.reg   = LED_PIN_MASK;
    PORT->Group[LED_PORT_GROUP].OUTCLR.reg   = LED_PIN_MASK;
    PORT->Group[TRACE_PORT_GROUP].DIRSET.reg = TRACE_ALL_MASK;
    PORT->Group[TRACE_PORT_GROUP].OUTCLR.reg = TRACE_ALL_MASK;
}

/** @brief LED pin high. Caller must hold xLedLock. */
static void led_on(void)
{
    PORT->Group[LED_PORT_GROUP].OUTSET.reg = LED_PIN_MASK;
}

/** @brief LED pin low. Caller must hold xLedLock. */
static void led_off(void)
{
    PORT->Group[LED_PORT_GROUP].OUTCLR.reg = LED_PIN_MASK;
}

/* ===========================================================================
 * UART (SERCOM5, polled, 8N1)
 * ======================================================================== */

static void uart_init(void)
{
    PORT->Group[UART_PORT_GROUP].PINCFG[UART_TX_PIN].reg = PORT_PINCFG_PMUXEN;
    PORT->Group[UART_PORT_GROUP].PINCFG[UART_RX_PIN].reg =
        PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;
    PORT->Group[UART_PORT_GROUP].PMUX[UART_TX_PIN / 2U].reg =
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

    SERCOM5->USART.BAUD.reg = (uint16_t)(65536ULL -
        ((65536ULL * UART_OVERSAMPLING * UART_BAUD_RATE) / CPU_CLOCK_HZ));

    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0U) |
                               SERCOM_USART_CTRLB_TXEN |
                               SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB) { /* wait for sync */ }

    SERCOM5->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE) { /* wait for sync */ }
}

static void uart_puts(const char *s)
{
    while (*s != '\0')
    {
        while (!SERCOM5->USART.INTFLAG.bit.DRE) { /* wait for empty buffer */ }
        SERCOM5->USART.DATA.reg = (uint16_t)(uint8_t)*s;
        s++;
    }
}

/** @brief Print an unsigned number in decimal (no printf needed). */
static void uart_putu(uint32_t v)
{
    char buf[11];
    int  i = 10;

    buf[i] = '\0';
    do
    {
        buf[--i] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);
    uart_puts(&buf[i]);
}

/* ===========================================================================
 * Active wait: calibrated CPU work
 * ======================================================================== */

static void burn_loops(uint32_t loops)
{
    volatile uint32_t i;

    for (i = loops; i > 0U; i--)
    {
        /* burn CPU time */
    }
}

/** @brief Count loops per ms with SysTick, before the scheduler starts. */
static void calibrate_active_wait(void)
{
    uint32_t start;
    uint32_t end;
    uint32_t cycles;

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0x00FFFFFFUL;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    start = SysTick->VAL;
    burn_loops(CALIBRATION_LOOPS);
    end   = SysTick->VAL;
    SysTick->CTRL = 0U;

    cycles = (start - end) & 0x00FFFFFFUL;     /* SysTick counts down */
    ulLoopsPerMs = (uint32_t)(((uint64_t)CALIBRATION_LOOPS *
                               (CPU_CLOCK_HZ / 1000UL)) / cycles);
}

/* ===========================================================================
 * Tasks
 * ======================================================================== */

/**
 * @brief Task 1: owns the LED while it is OFF for 1 s (task delay).
 */
static void vTask1LedOff(void *pvParameters)
{
    TickType_t now;

    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(xLedLock, portMAX_DELAY) == pdTRUE)
        {
            now = xTaskGetTickCount();
            onMs = now - onStartTick;           /* how long it was ON  */
            if ((onStartTick != 0U) && (onMs > onMax))
            {
                onMax = onMs;
            }
            offStartTick = now;
            led_off();

            /* Sleep 1 s while still holding the lock: the CPU is free,
             * but Task 3 cannot turn the LED back on early. */
            vTaskDelay(pdMS_TO_TICKS(LED_OFF_TIME_MS));

            (void)xSemaphoreGive(xLedLock);
        }

        /* Hand-off: sleep until Task 3 has actually taken the lock (count
         * drops from 1 to 0); otherwise this task would take it straight
         * back. Checking the count, instead of sleeping a fixed 2 ticks,
         * stops Task 3 missing its turn when Task 2 happens to be printing
         * during the hand-off (seen once as on_ms=1410 in a case (b) log). */
        do
        {
            vTaskDelay(HANDOFF_TICKS);
        } while (uxSemaphoreGetCount(xLedLock) != 0U);
    }
}

/**
 * @brief Task 2: print a status line, then task-delay 1 s.
 *        Lateness = wake-up time -> time it actually started printing.
 */
static void vTask2Status(void *pvParameters)
{
    TickType_t xRelease = xTaskGetTickCount();
    TickType_t xStart;
    uint32_t   count = 0U;

    (void)pvParameters;

    for (;;)
    {
        xStart = xTaskGetTickCount();
        t2Lat  = (uint32_t)(xStart - xRelease);
        if (t2Lat > t2Max)
        {
            t2Max = t2Lat;
        }

        uart_puts("status ");      uart_putu(count++);
        uart_puts(" tick=");       uart_putu((uint32_t)xStart);
        uart_puts(" on_ms=");      uart_putu(onMs);
        uart_puts(" off_ms=");     uart_putu(offMs);
        uart_puts(" on_max=");     uart_putu(onMax);
        uart_puts(" t2_lat=");     uart_putu(t2Lat);
        uart_puts(" t2_max=");     uart_putu(t2Max);
        uart_puts("\r\n");

        xRelease = xTaskGetTickCount() + pdMS_TO_TICKS(STATUS_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(STATUS_DELAY_MS));
    }
}

/**
 * @brief Task 3: owns the LED while it is ON for 0.5 s (active wait),
 *        then yields.
 */
static void vTask3LedOn(void *pvParameters)
{
    TickType_t now;

    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(xLedLock, portMAX_DELAY) == pdTRUE)
        {
            now = xTaskGetTickCount();
            offMs = now - offStartTick;         /* how long it was OFF */
            onStartTick = now;
            led_on();

            burn_loops(ulLoopsPerMs * LED_ON_TIME_MS);  /* active wait */

            (void)xSemaphoreGive(xLedLock);
        }

        taskYIELD();
    }
}

/* ===========================================================================
 * FreeRTOS hook functions (C linkage, also when compiled as C++)
 * ======================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

#ifdef __cplusplus
}
#endif

/* ===========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    TaskHandle_t xTask1 = NULL;
    TaskHandle_t xTask2 = NULL;
    TaskHandle_t xTask3 = NULL;
    BaseType_t   xOk    = pdPASS;

    /* Clocks are already set to 48 MHz by the start-up code (SystemInit) */
    gpio_init();
    uart_init();
    calibrate_active_wait();

    uart_puts("\r\nQ3 experiment ");
    uart_putu((uint32_t)EXPERIMENT);
    uart_puts(": ");
    uart_puts(EXPERIMENT_NAME[EXPERIMENT]);
    uart_puts("\r\npriorities T1/T2/T3 = ");
    uart_putu((uint32_t)TASK1_PRIORITY); uart_puts("/");
    uart_putu((uint32_t)TASK2_PRIORITY); uart_puts("/");
    uart_putu((uint32_t)TASK3_PRIORITY);
    uart_puts("  loops/ms=");
    uart_putu(ulLoopsPerMs);
    uart_puts("\r\nbuild: " __DATE__ " " __TIME__ "  (fw v3: noinline wait + hand-off check)\r\n");

#if USE_BINARY_SEMAPHORE
    xLedLock = xSemaphoreCreateBinary();
    configASSERT(xLedLock != NULL);
    (void)xSemaphoreGive(xLedLock);      /* binary semaphores start empty */
#else
    xLedLock = xSemaphoreCreateMutex();
    configASSERT(xLedLock != NULL);
#endif

    xOk &= xTaskCreate(vTask1LedOff, "T1_LedOff", TASK_STACK_WORDS, NULL,
                       (UBaseType_t)TASK1_PRIORITY, &xTask1);
    xOk &= xTaskCreate(vTask2Status, "T2_Status", TASK_STACK_WORDS, NULL,
                       (UBaseType_t)TASK2_PRIORITY, &xTask2);
    xOk &= xTaskCreate(vTask3LedOn,  "T3_LedOn",  TASK_STACK_WORDS, NULL,
                       (UBaseType_t)TASK3_PRIORITY, &xTask3);
    configASSERT(xOk == pdPASS);

    vTaskSetApplicationTaskTag(xTask1, (TaskHookFunction_t)TRACE_TASK1_MASK);
    vTaskSetApplicationTaskTag(xTask2, (TaskHookFunction_t)TRACE_TASK2_MASK);
    vTaskSetApplicationTaskTag(xTask3, (TaskHookFunction_t)TRACE_TASK3_MASK);

    vTaskStartScheduler();

    for (;;) { }

    return 0;
}
