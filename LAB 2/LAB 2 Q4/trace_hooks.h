// Q4: FreeRTOS trace macros.
// platformio.ini adds this file to every file it compiles (-include), so the
// FreeRTOS_SAMD21 library's tasks.c uses these macros instead of its empty
// defaults. The kernel runs them on every context switch.
//
// Switched out and still in its ready list -> preempted / yielded
// Switched out, not ready, waiting on the mutex -> blocked on the mutex
// Switched out, not ready, not waiting on it    -> blocked in vTaskDelay()
#ifndef TRACE_HOOKS_H
#define TRACE_HOOKS_H

#ifndef __ASSEMBLER__

#ifdef __cplusplus
extern "C" {
#endif
void taskSwitchedOut(unsigned long num, long stillReady,
                     long waitingOnMutex, unsigned long tick);
void taskSwitchedIn(unsigned long num, unsigned long tick);
#ifdef __cplusplus
}
#endif

#define traceTASK_SWITCHED_OUT()                                       \
    taskSwitchedOut(pxCurrentTCB->uxTaskNumber,                        \
        listIS_CONTAINED_WITHIN(                                       \
            &pxReadyTasksLists[pxCurrentTCB->uxPriority],              \
            &pxCurrentTCB->xStateListItem),                            \
        listLIST_ITEM_CONTAINER(&pxCurrentTCB->xEventListItem) != NULL,\
        xTickCount)

#define traceTASK_SWITCHED_IN()                                        \
    taskSwitchedIn(pxCurrentTCB->uxTaskNumber, xTickCount)

#endif /* __ASSEMBLER__ */

#endif /* TRACE_HOOKS_H */
