# STM32CubeIDE Projects

This STM32CubeIDE workspace contains the firmware projects for the JUMBLEQ hardware.
All projects target the **STM32H7S3Z8T6** on the [STM32H7S3 Development Board](https://github.com/YamamotoWorksDev/STM32H7S3_Dev_Board).

For the debug probe settings used with these projects, see [Debug Configuration for ST-LINK](docs/Debug_Configuration-STLINK.md) and [Debug Configuration for J-Link](docs/Debug_Configuration-JLINK.md).

## Projects

### `JUMBLEQ`

The main firmware for JUMBLEQ.

- Runs USB, audio, LED, ADC, and OLED processing on FreeRTOS
- Provides USB Audio Class 2.0 and USB MIDI using TinyUSB
- Controls the AK4619 audio codec and ADAU1466 DSP
- Provides the user interface through the OLED, LEDs, and hardware controls
- Saves settings to EEPROM and restores them at startup

The buildable STM32CubeIDE project is located in `JUMBLEQ/Appli`.
The application is configured to run from external flash at `0x90010000`. The post-build step generates BIN and UF2 files.

### `LED_BLINK_APP`

A minimal test application for verifying basic board operation and application startup from external flash.
It turns LED1 and LED2 on and off in sequence at 500 ms intervals.

The buildable STM32CubeIDE project is located in `LED_BLINK_APP/Appli`.
Like `JUMBLEQ`, the application is configured to run from external flash at `0x90010000`. The post-build step generates a UF2 file.

### `MX25UW25645GXDI00_STM32H7S3Z8T`

The bootloader and external memory loader for the **MX25UW25645GXDI00** external flash device.

- `Boot`: Bootloader that writes UF2 files over USB Mass Storage Class
  - Enters UF2 update mode when started while SW3 is held down
  - Jumps to the application in external flash during a normal startup
- `ExtMemLoader`: Loader that allows STM32CubeProgrammer and STM32CubeIDE to access the external flash
- `uf2conv.py`: Tool that converts BIN files to UF2 format

#### Boot Build Configuration

The `Boot` project must be built with compiler optimization enabled to fit in the internal flash of the STM32H7S3Z8T6. The `Debug` configuration uses `-O0`, which produces an image that is too large and causes the build to fail. Use the `Release` configuration, which uses `-Os` to optimize the image for size.

In STM32CubeIDE, select `Project > Build Configurations > Set Active > Release` before building the `Boot` project.

For bootloader details, see [`docs/UF2_BOOTLOADER_SPEC.md`](MX25UW25645GXDI00_STM32H7S3Z8T/docs/UF2_BOOTLOADER_SPEC.md).

## Project Relationships

`MX25UW25645GXDI00_STM32H7S3Z8T/Boot` handles startup and firmware updates, then runs either `JUMBLEQ/Appli` or `LED_BLINK_APP/Appli` from external flash.

