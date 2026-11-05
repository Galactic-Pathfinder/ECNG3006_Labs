/* Unity configuration: send each output character to the UART */
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

void unity_putc(int c);

#ifdef __cplusplus
}
#endif

#define UNITY_OUTPUT_CHAR(c) unity_putc(c)

#endif /* UNITY_CONFIG_H */
