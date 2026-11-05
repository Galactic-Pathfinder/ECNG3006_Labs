/*
 * Test-suite entry point. Runs the Unity tests repeatedly.
 * Results are printed on SERCOM5 TX (D14), 115200 8N1.
 * LED: 2 slow blinks = all tests passed, 10 fast blinks = failures.
 */
extern "C" {
#include "NotUart.h"
int run_tests(void);
}

extern "C" void unity_putc(int c)
{
    if (c == '\n') {
        uart_putc('\r');
    }
    uart_putc((char)c);
}

static void blink(int n, unsigned int ms)
{
    while (n--) {
        led_set(1);
        delay_ms(ms);
        led_set(0);
        delay_ms(ms);
    }
}

int main(void)
{
    uart_init();
    led_init();

    for (;;) {
        int failures = run_tests();

        if (failures == 0) {
            blink(2, 500);
        } else {
            blink(10, 80);
        }
        delay_ms(3000);
    }
}
