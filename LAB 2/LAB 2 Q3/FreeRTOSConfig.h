/*
 * ============================================================================
 * File:        FreeRTOSConfig.h   (application copy, in include/)
 * Author:      <Your Name>
 * Platform:    Arduino MKR Zero (ATSAMD21G18A, ARM Cortex-M0+, 48 MHz)
 *
 * Description:
 *   Application-specific FreeRTOS configuration for the Question 3
 *   scheduling experiments. The FreeRTOS_SAMD21 library stays where
 *   PlatformIO downloads it (lib_deps -> .pio/libdeps/<env>/).
 *
 *   How this file replaces the library's own FreeRTOSConfig.h:
 *   platformio.ini passes  -include include/FreeRTOSConfig.h  so GCC reads
 *   this file FIRST in every source file. It defines the guard
 *   FREERTOS_CONFIG_H, so when FreeRTOS.h later includes the library's
 *   copy, that copy is skipped. Kernel, port and application all see the
 *   same settings. This file therefore contains only #define lines (no C
 *   declarations), so it is safe in C, C++ and assembler files.
 *
 *   EXPERIMENT is set per PlatformIO environment (platformio.ini):
 *     0 = same priority, run to completion   -> co-operative scheduler
 *     1 = (a) T1>T2>T3, run to completion    -> co-operative scheduler
 *     2 = (b) T1>T2>T3, no prio inheritance  -> preemptive scheduler
 *     3 = (c) T1>T2>T3, prio inheritance     -> preemptive scheduler
 *   (the lock type, mutex or binary semaphore, is chosen in main.c)
 *
 *   Changes from the library's default configuration:
 *     - configUSE_PREEMPTION chosen by EXPERIMENT (0 = run to completion).
 *     - configUSE_APPLICATION_TASK_TAG = 1 and trace hooks: each task
 *       drives its own pin while it runs (oscilloscope evidence).
 *     - configCPU_CLOCK_HZ fixed at 48 MHz.
 *     - configMAX_PRIORITIES = 4: idle (0) + three application priorities.
 *     - Idle hook, run-time statistics, software timers, recursive mutexes,
 *       counting semaphores and queue sets turned OFF (not used). This also
 *       removes the library's "implicit declaration" warnings.
 *     - configASSERT() simply stops (no Arduino Serial error printing).
 *     - configPRIO_BITS = 2, the real NVIC priority bits on the SAMD21.
 *     - Heap reduced to 8 KB.
 *
 *   Reference: https://www.freertos.org/a00110.html
 * ============================================================================
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Marker checked by main.c: proves this application config is the one in
 * use (and not the library's copy in .pio/libdeps). */
#define APP_FREERTOS_CONFIG                 1

/* ---------------------------------------------------------------------------
 * Scheduler
 * ------------------------------------------------------------------------- */
#ifndef EXPERIMENT
#define EXPERIMENT                          3
#endif

/* "Run to completion": a task keeps the CPU until it blocks or yields.
 * In FreeRTOS this is the co-operative scheduler (configUSE_PREEMPTION 0). */
#if ( EXPERIMENT == 0 ) || ( EXPERIMENT == 1 )
#define configUSE_PREEMPTION                0
#else
#define configUSE_PREEMPTION                1
#endif
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
#define configUSE_APPLICATION_TASK_TAG      1   /* tag = trace pin mask   */
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
 * Trace hooks: drive one header pin HIGH while each task is running.
 * Each task's tag (set in main.c) holds the PORT A bit mask of its pin,
 * so an oscilloscope / logic analyser shows exactly which task has the CPU.
 * Raw register addresses are used so this header does not need sam.h:
 *   PORT base 0x41004400, Group 0 (Port A): OUTCLR +0x14, OUTSET +0x18.
 * ------------------------------------------------------------------------- */
#define TRACE_PORTA_OUTCLR  ( *( volatile unsigned long * ) 0x41004414UL )
#define TRACE_PORTA_OUTSET  ( *( volatile unsigned long * ) 0x41004418UL )

#define traceTASK_SWITCHED_IN() \
    ( TRACE_PORTA_OUTSET = ( unsigned long ) pxCurrentTCB->pxTaskTag )
#define traceTASK_SWITCHED_OUT() \
    ( TRACE_PORTA_OUTCLR = ( unsigned long ) pxCurrentTCB->pxTaskTag )

/* ---------------------------------------------------------------------------
 * Assert: stop everything so the fault can be found with a debugger
 * ------------------------------------------------------------------------- */
#define configASSERT( x )                                   \
    if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ) { } }

/* ---------------------------------------------------------------------------
 * Needed only so the library's (unused) error_hooks.cpp still compiles
 * ------------------------------------------------------------------------- */
#define configCAL_FACTOR    ( 48000000UL / 6000UL )

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
