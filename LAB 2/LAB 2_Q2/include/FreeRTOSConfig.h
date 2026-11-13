/*
 * ============================================================================
 * File:        FreeRTOSConfig.h
 * Author:      <Your Name>
 * Platform:    Arduino MKR Zero (ATSAMD21G18A, ARM Cortex-M0+, 48 MHz)
 *
 * Description:
 *   Application-specific FreeRTOS configuration for the "three tasks + one
 *   mutex" LED example. It replaces the FreeRTOSConfig.h that ships inside
 *   the FreeRTOS_SAMD21 library (that copy was deleted from lib/ so this
 *   one, in the application's include/ folder, is the only one used).
 *
 *   Changes from the library's default configuration:
 *     - configCPU_CLOCK_HZ fixed at 48 MHz (no dependency on Arduino F_CPU).
 *     - configMAX_PRIORITIES = 4: idle (0) + three application priorities.
 *     - configUSE_MUTEXES = 1 (required by xSemaphoreCreateMutex()).
 *     - Recursive mutexes, counting semaphores, queue sets, software
 *       timers and run-time statistics turned OFF (not used).
 *     - Idle hook OFF (the library's hook called the Arduino loop()).
 *     - configASSERT() no longer calls the Arduino Serial error hooks.
 *     - configPRIO_BITS = 2, the real number of NVIC priority bits on the
 *       SAMD21, so the kernel (SysTick/PendSV) runs at the lowest priority.
 *     - Heap reduced to 8 KB (3 small tasks + idle task + 1 mutex).
 *
 *   Reference: https://www.freertos.org/a00110.html
 * ============================================================================
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ---------------------------------------------------------------------------
 * Scheduler
 * ------------------------------------------------------------------------- */
#define configUSE_PREEMPTION                1
#define configUSE_TIME_SLICING              1
#define configCPU_CLOCK_HZ                  ( 48000000UL )
#define configTICK_RATE_HZ                  ( ( TickType_t ) 1000 )  /* 1 ms tick */
#define configMAX_PRIORITIES                ( 4 )
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 128 ) /* words */
#define configMAX_TASK_NAME_LEN             ( 16 )
#define configUSE_16_BIT_TICKS              0
#define configIDLE_SHOULD_YIELD             1

/* ---------------------------------------------------------------------------
 * Memory allocation (heap_4bis.c in the library)
 * ------------------------------------------------------------------------- */
#define configSUPPORT_STATIC_ALLOCATION     0
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 8 * 1024 ) )

/* ---------------------------------------------------------------------------
 * Synchronisation objects
 * ------------------------------------------------------------------------- */
#define configUSE_MUTEXES                   1   /* needed for the GPIO mutex */
#define configUSE_RECURSIVE_MUTEXES         0
#define configUSE_COUNTING_SEMAPHORES       0
#define configUSE_QUEUE_SETS                0
#define configQUEUE_REGISTRY_SIZE           0

/* ---------------------------------------------------------------------------
 * Hook functions (implemented in main.c)
 * ------------------------------------------------------------------------- */
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configUSE_MALLOC_FAILED_HOOK        1   /* vApplicationMallocFailedHook  */
#define configCHECK_FOR_STACK_OVERFLOW      2   /* vApplicationStackOverflowHook */

/* ---------------------------------------------------------------------------
 * Features not used by this application
 * ------------------------------------------------------------------------- */
#define configUSE_TRACE_FACILITY            1   /* must stay 1: the library's
                                                   tasks.c adds functions that
                                                   use the trace TCB number  */
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configGENERATE_RUN_TIME_STATS       0
#define configUSE_APPLICATION_TASK_TAG      0
#define configUSE_CO_ROUTINES               0
#define configMAX_CO_ROUTINE_PRIORITIES     ( 1 )
#define configUSE_TIMERS                    0

/* ---------------------------------------------------------------------------
 * Optional API functions (1 = include, 0 = exclude)
 * ------------------------------------------------------------------------- */
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_vTaskDelayUntil             0
#define INCLUDE_vTaskPrioritySet            0
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_xTaskGetCurrentTaskHandle   0
#define INCLUDE_uxTaskGetStackHighWaterMark 0
#define INCLUDE_xTaskGetIdleTaskHandle      0
#define INCLUDE_eTaskGetState               0

/* ---------------------------------------------------------------------------
 * Assert: stop everything so the fault can be found with a debugger
 * ------------------------------------------------------------------------- */
#define configASSERT( x )                                   \
    if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ) { } }

/* ---------------------------------------------------------------------------
 * Cortex-M0+ interrupt priorities
 * SAMD21 NVIC implements 2 priority bits (4 levels, 0 = highest).
 * ------------------------------------------------------------------------- */
#define configPRIO_BITS                                 2
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0x03
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    0x01

#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* Map the FreeRTOS port handlers onto the CMSIS vector-table names.
 * SysTick is not mapped: the startup code's SysTick_Handler calls
 * sysTickHook(), which the library's port.c uses to run the RTOS tick. */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler

#endif /* FREERTOS_CONFIG_H */
