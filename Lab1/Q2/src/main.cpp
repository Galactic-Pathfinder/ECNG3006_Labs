/*
 * Main program: demonstrates the new magnitude() function.
 * Arduino.h is not used; this file provides its own main(), which the
 * start-up code calls after setting the CPU clock to 48 MHz.
 * Output: SERCOM5 TX on D14, 115200 8N1 (USB-to-serial adapter).
 */
extern "C" {
#include "calc.h"

}

int main(void)
{

    led_init();

    for (;;) {


        led_set(1);
        delay_ms(500);
        led_set(0);
        delay_ms(1500);
    }
}
