# STM32H7S3 Errata

## Erratum 2.2.15

ST documents the following STM32H7R/S device erratum:

- [I/O compensation could alter the duty cycle of a high-frequency output signal](https://www.st.com/resource/en/errata_sheet/es0596-stm32h7rxx7sxx-device-errata-stmicroelectronics.pdf)

The recommended workaround has already been implemented in the boot-side code:

- [`sbs.c`](../../STM32CubeIDE/MX25UW25645GXDI00_STM32H7S3Z8T/Boot/Core/Src/sbs.c)
