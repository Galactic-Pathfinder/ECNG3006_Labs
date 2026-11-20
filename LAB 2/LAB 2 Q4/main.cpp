#include <sam.h>

#include <FreeRTOS_SAMD21.h>

// =====================================================================
// LED pattern: ON for 0.5 s, OFF for 1 s, repeating (1.5 s cycle)
//
// The mutex is the "right to own the LED". Each LED task keeps it for
// the WHOLE time it wants the pin in its state:
//
// ON task  : take mutex -> pin ON  -> busy-wait 500 ms  -> give -> yield
// OFF task : take mutex -> pin OFF -> vTaskDelay 1000 ms -> give -> yield
//
// While one task holds the mutex the other is blocked on it, so neither
// can change the pin early.
//
// Q4: FreeRTOS TRACE MACROS track every time a task is blocked.
// traceTASK_SWITCHED_OUT / traceTASK_SWITCHED_IN are defined with -D in
// platformio.ini (so the library's tasks.c sees them) and call
// taskSwitchedOut() / taskSwitchedIn() below on every context switch.
// For each task they count:
//   preempted    - switched out but still Ready (preempted or yielded)
//   delay_blocks - blocked in vTaskDelay()
//   mutex_blocks - blocked waiting for the LED mutex
//   blocked_ms   - total time the task was off the CPU after blocking
// The Status task prints the counts once per second.
// =====================================================================

// ---------- Q4: choose the case to run ----------
// 1 = all three tasks the SAME priority
// 0 = DISTINCT priorities (ON 1 < OFF 2 < STATUS 3)
#define SAME_PRIORITIES 0

// ---------- Task priorities (higher number = higher priority) ----------
#if SAME_PRIORITIES
static const int PRIO_LED_ON  = 1;
static const int PRIO_LED_OFF = 1;
static const int PRIO_STATUS  = 1;
#else
static const int PRIO_LED_ON  = 1;  // busy-waiting task lowest
static const int PRIO_LED_OFF = 2;
static const int PRIO_STATUS  = 3;  // short prints, so it always runs on time
#endif

static const int TASK_STACK_WORDS = 128;

// ---------- Mutex that guards the shared LED pin (PB08) ----------
static SemaphoreHandle_t ledMutex;

// ---------- Measured LED timing (1 tick = 1 ms at 1000 Hz) ----------
static volatile uint32_t onStartTick = 0;  // tick of the last OFF->ON edge
static volatile uint32_t offStartTick = 0; // tick of the last ON->OFF edge
static volatile uint32_t onMs = 0;         // how long the LED was last ON
static volatile uint32_t offMs = 0;        // how long the LED was last OFF

// ---------- Q4: blocking counters, written by the trace macros ----------
// Index = task number set in main(): 1 = LedOn, 2 = LedOff, 3 = Status.
// The kernel's own tasks (Idle, Timer) keep number 0 and are ignored.
struct TaskStats {
    uint32_t preempted;
    uint32_t delayBlocks;
    uint32_t mutexBlocks;
    uint32_t blockedMs;
    uint32_t blockedAt;     // tick when the current block started
    bool     isBlocked;
};
static TaskStats stats[4];
static const char *const taskName[4] = { "", "LedOn ", "LedOff", "Status" };

// Called by traceTASK_SWITCHED_OUT (inside the kernel, interrupts off).
// The macro (platformio.ini) passes values only the kernel can see:
//   num            = pxCurrentTCB->uxTaskNumber
//   stillReady     = task is still in its ready list (preempted/yielded)
//   waitingOnMutex = task's event list item is in use (waiting on the mutex)
//   tick           = xTickCount
extern "C" void taskSwitchedOut(unsigned long num, long stillReady,
                                long waitingOnMutex, unsigned long tick) {
    if (num < 1 || num > 3) return;
    TaskStats *s = &stats[num];

    if (stillReady) {               // not blocked, just lost the CPU
        s->preempted++;
        return;
    }
    if (waitingOnMutex) s->mutexBlocks++;   // BLOCKED on the LED mutex
    else                s->delayBlocks++;   // BLOCKED in vTaskDelay()
    s->isBlocked = true;
    s->blockedAt = tick;
}

// Called by traceTASK_SWITCHED_IN (inside the kernel, interrupts off)
extern "C" void taskSwitchedIn(unsigned long num, unsigned long tick) {
    if (num < 1 || num > 3) return;
    TaskStats *s = &stats[num];

    if (s->isBlocked) {             // running again after a block
        s->blockedMs += tick - s->blockedAt;
        s->isBlocked = false;
    }
}

// ---------- Raw SERCOM5 UART (MKR Zero: pin 14 = TX/PB22, pin 13 = RX/PB23) ----------
static void uart_init(void) {
    // Route PB22/PB23 to SERCOM5 (peripheral function D = 3)
    PORT->Group[1].PINCFG[22].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PINCFG[23].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[1].PMUX[11].reg = PORT_PMUX_PMUXE(3) | PORT_PMUX_PMUXO(3);

    // Power and clock SERCOM5 from GCLK0 (48 MHz)
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM5_CORE;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Reset, then configure: internal clock, TX on PAD2, RX on PAD3, LSB first
    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST);

    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |
                               SERCOM_USART_CTRLA_MODE(1) |
                               SERCOM_USART_CTRLA_RXPO(3) |
                               SERCOM_USART_CTRLA_TXPO(1);
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
    while (SERCOM5->USART.SYNCBUSY.bit.CTRLB);

    SERCOM5->USART.BAUD.reg = 63019; // 115200 baud @ 48 MHz
    SERCOM5->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE);
}

static void uart_puts(const char *s) {
    while (*s) {
        while (!SERCOM5->USART.INTFLAG.bit.DRE);
        SERCOM5->USART.DATA.reg = (uint8_t)*s++;
    }
}

static void uart_putu(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do { buf[--i] = '0' + (v % 10); v /= 10; } while (v);
    uart_puts(&buf[i]);
}


// Task 1: owns the LED while it is ON for 0.5 s (active wait), then yields
void vTaskLedOn(void *pvParameters) {
    while (1) {
        // 1. Take the mutex (blocked here while the OFF task owns the LED)
        if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {

            // 2. Drive the pin high and note when it happened
            TickType_t now = xTaskGetTickCount();
            offMs = now - offStartTick; // how long it was OFF
            onStartTick = now;
            PORT->Group[1].OUTSET.reg = PORT_PB08; // LED ON

            // 3. Actively (busy) wait 500 ms - still holding the mutex
            while ((xTaskGetTickCount() - now) < pdMS_TO_TICKS(500)) {
                // spin
                __NOP();
            }

            // 4. Give the mutex back so the OFF task can take over
            xSemaphoreGive(ledMutex);
        }

        // 5. Yield before looping (the lab's "actively wait, then yield")
        taskYIELD();
    }
}

// Task 2: owns the LED while it is OFF for 1 s (task delay), then hands over
void vTaskLedOff(void *pvParameters) {
    while (1) {
        // 1. Take the mutex (blocked here while the ON task owns the LED)
        if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {

            // 2. Drive the pin low and note when it happened
            TickType_t now = xTaskGetTickCount();
            onMs = now - onStartTick; // how long it was ON
            offStartTick = now;
            PORT->Group[1].OUTCLR.reg = PORT_PB08; // LED OFF

            // 3. Sleep 1 s while still holding the mutex (task is blocked,
            // not spinning, so the CPU is free for other tasks)
            vTaskDelay(pdMS_TO_TICKS(1000)); // 1000ms delay

            // 4. Give the mutex back so the ON task can take over
            xSemaphoreGive(ledMutex);
        }

        // 5. Let the ON task run and take the mutex. Without this the OFF
        // task would re-take it immediately and ON would starve. 2 ticks
        // (not 1) guarantees at least 1 full ms.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Task 3: prints a status message + the Q4 blocking counts, then sleeps 1 s
void vTaskStatus(void *pvParameters) {
    uint32_t count = 0;

    // Boot banner: if you do not see this line, an OLD firmware is running
    uart_puts(SAME_PRIORITIES ? "boot: Q4 SAME priorities (1/1/1)"
                              : "boot: Q4 DISTINCT priorities (ON 1 < OFF 2 < STATUS 3)");
    uart_puts(" tick_hz=");
    uart_putu((uint32_t)configTICK_RATE_HZ);
    uart_puts("\r\n");

    while (1) {
        // 1. Read the pin state. A single register read is atomic, so no
        // mutex here - taking it would block this task for up to 1 s.
        const char *state = (PORT->Group[1].OUT.reg & PORT_PB08) ? "ON" : "OFF";

        // 2. Print the status line
        uart_puts("status ");
        uart_putu(count++);
        uart_puts(" tick=");
        uart_putu((uint32_t)xTaskGetTickCount());
        uart_puts(" LED=");
        uart_puts(state);
        uart_puts(" on_ms=");
        uart_putu(onMs);
        uart_puts(" off_ms=");
        uart_putu(offMs);
        uart_puts("\r\n");

        // 3. Q4: print the blocking counts for each task
        for (int i = 1; i <= 3; i++) {
            taskENTER_CRITICAL();           // consistent copy of the counters
            TaskStats s = stats[i];
            taskEXIT_CRITICAL();

            uart_puts("  ");
            uart_puts(taskName[i]);
            uart_puts(": preempted=");
            uart_putu(s.preempted);
            uart_puts(" delay_blocks=");
            uart_putu(s.delayBlocks);
            uart_puts(" mutex_blocks=");
            uart_putu(s.mutexBlocks);
            uart_puts(" blocked_ms=");
            uart_putu(s.blockedMs);
            uart_puts("\r\n");
        }

        // 4. Keep the task asleep for 1 s non-blocking
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1000ms delay
    }
}

int main(void) {
    TaskHandle_t hOn, hOff, hStatus;

    // Initialize standard microcontroller system clocks (48MHz layout)
    SystemInit();
    uart_init();

    // Configure Pin PB08 (Built-in LED) as a digital OUTPUT
    PORT->Group[1].DIRSET.reg = PORT_PB08;

    // Create the mutex that guards the shared LED pin
    ledMutex = xSemaphoreCreateMutex();
    configASSERT(ledMutex != NULL);

    // Create the three tasks, each with its own priority constant
    xTaskCreate(vTaskLedOn, "LedOn", TASK_STACK_WORDS, NULL, PRIO_LED_ON, &hOn);
    xTaskCreate(vTaskLedOff, "LedOff", TASK_STACK_WORDS, NULL, PRIO_LED_OFF, &hOff);
    xTaskCreate(vTaskStatus, "Status", TASK_STACK_WORDS, NULL, PRIO_STATUS, &hStatus);

    // Q4: number each task so the trace macros know which counters to use
    vTaskSetTaskNumber(hOn, 1);
    vTaskSetTaskNumber(hOff, 2);
    vTaskSetTaskNumber(hStatus, 3);

    // Boot the FreeRTOS Scheduler engine execution path
    vTaskStartScheduler();

    while (1) {
        __NOP(); // Safety lock trap loop
    }
    return 0;
}

// Retain underlying framework linkages to keep compilation targets happy
extern "C" {
void setup() {
    // Left completely blank
}
void loop() {
    // Left completely blank
}
}