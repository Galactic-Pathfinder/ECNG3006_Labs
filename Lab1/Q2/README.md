# ECNG3006 – Unity Unit Testing on the Arduino MKR Zero

Unit tests for the `calc` library (from [kasramp/c-unit-testing](https://github.com/kasramp/c-unit-testing)),
built with PlatformIO and run on an Arduino MKR Zero (SAMD21G18A, ARM Cortex-M0+, 48 MHz).
`Arduino.h` is **not** used: the Arduino framework only supplies the start-up code, linker
script and 48 MHz clock set-up. All I/O is register-level code using `sam.h`.

## Project layout

```
platformio.ini          two environments: app and CalcUnityTest
src/
  calc.c, calc.h        calc library (+ new magnitude() function)
  NotUart.c, NotUart.h  register-level UART (SERCOM5, D14 TX) and LED (PA20) drivers
  main.cpp              main program, demonstrates magnitude()
CalcUnityTest/
  Testcalc.c            Unity test suite (17 tests)
  Test_main.cpp         test entry point, prints results over UART + LED status
  unity_config.h        routes Unity output to the UART
unity/                  Unity test framework (ThrowTheSwitch)
binaries/               main.bin and suite.bin
proof/                  screenshots / video of both programs running
```

## Build and upload

```
pio run -e app                        # main program
pio run -e CalcUnityTest              # test suite
pio run -e app -t upload              # double-tap RESET first
pio run -e CalcUnityTest -t upload    # double-tap RESET first
```

The firmware does not use the native USB port, so the board must be put into its
bootloader (double-tap RESET) before each upload. Output is read on D14 (TX) with a
3.3 V USB-to-serial adapter at 115200 8N1.

- Main program prints `magnitude(-42) = 42` and `magnitude(17) = 17`.
- Test suite prints Unity results ending in `17 Tests 0 Failures 0 Ignored OK`.
  LED: 2 slow blinks = all passed, 10 fast blinks = failures.

## New function: `magnitude`

`unsigned int magnitude(int x)` returns the unsigned magnitude of `x`. It negates in unsigned
arithmetic, so `magnitude(INT_MIN)` is also correct and well defined.

`test_magnitude` verifies that `int` is a **signed** type: it checks that `(int)-1 < 0`, and that
negative inputs (`-5`, `-1`, `INT_MIN`) return the correct positive magnitude. If `int` were
unsigned, `-5` would be stored as a very large positive number and `magnitude(-5)` would not
return 5, so the test would fail.

## `test_add`: catching an 8-bit `int`

`test_add` was extended with:

```c
TEST_ASSERT_EQUAL_INT(200, add(100, 100));     /* wraps to -56 on 8-bit int */
TEST_ASSERT_EQUAL_INT(-200, add(-100, -100));  /* wraps to 56 on 8-bit int  */
TEST_ASSERT_TRUE(sizeof(int) >= 2);
```

Reasoning: a signed 8-bit integer only holds -128 to 127. The sums 200 and -200 are outside
that range, so on an 8-bit `int` they overflow and wrap around to -56 and 56, and the
assertions fail. A 16-bit signed integer holds -32768 to 32767, so the same sums are correct
and the assertions pass. The original test (`10 + 20 = 30`) fits in 8 bits and could never
detect the problem. The `sizeof(int) >= 2` check catches it directly. It uses `>=` rather than
`== 2` because the MKR Zero's ARM compiler defines `int` as 32 bits.

## Reflection

_Add your own reflection here: one challenge you faced and how it could be avoided in future._
