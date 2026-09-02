# JUMBLEQ

<img width="517" height="126" alt="JUMBLEQ logo" src="https://github.com/user-attachments/assets/0fec933e-3c29-45de-b90b-8d6eba264ed7" />

[Official Website](https://jumbleq.io/) · [日本語](https://jumbleq.io/ja/)

<img width="900" alt="JUMBLEQ beta prototype" src="https://github.com/user-attachments/assets/296e7e9c-62b2-4ea7-a199-6dd3ea355abf" />

“JUMBLEQ” is short for “jumbled equipment.”

JUMBLEQ is a faderless stereo digital audio mixer that replaces the traditional channel faders found on DJ mixers with magnetic switches. The switches behave like faders while enabling a compact and unconventional control layout.

Each of the two analog stereo inputs and two USB stereo inputs can be routed to Channel Fader A, Channel Fader B, or THRU. The USB audio I/O can also be used as a send/return path, allowing a USB host device to operate as an external effects processor.

As a USB Audio Class 2.0 interface, JUMBLEQ provides 4-in/4-out audio at 24-bit, 48 kHz or 96 kHz. It is USB class-compliant and can be used with Windows, macOS, iPhone, iPad, and Android devices without a device-specific driver.

## Key Features

- Faderless mixing with 10 magnetic switches and 12 potentiometers
- Flexible stereo input routing: assign each of the two analog and two USB inputs to Channel Fader A, Channel Fader B, or THRU
- 4-in/4-out USB Audio Class 2.0 at 24-bit, 48 kHz or 96 kHz
- USB audio send/return routing for external effects processing
- USB MIDI control and the browser-based JUMBLEQ Configurator
- Built-in UF2 bootloader for drag-and-drop firmware updates

## Documentation

See the [JUMBLEQ Documentation](./Docs/README.md) for the complete documentation index.

### User Guide

- [Operation Guide](./Docs/user-guide/operation-guide.md)
- [Firmware update](./Docs/user-guide/firmware-update.md)
- [JUMBLEQ Configurator](./Docs/user-guide/configurator.md)
- [Notes for Windows users](./Docs/user-guide/windows.md)

### Technical Reference

- [Audio specifications](./Docs/reference/audio-spec.md)
- [Hardware and PCB modules](./Docs/reference/hardware.md)
- [Block diagram and audio signal flow](./Docs/reference/signal-flow.md)
- [MIDI reference](./Docs/reference/midi/README.md)

### Development

- [Development environment](./Docs/development/environment.md)
- [STM32CubeIDE projects](./STM32CubeIDE/README.md)
- [Build and UF2 generation](./Docs/development/build-and-uf2.md)
- [ST-LINK debug configuration](./STM32CubeIDE/docs/Debug_Configuration-STLINK.md)
- [J-Link debug configuration](./STM32CubeIDE/docs/Debug_Configuration-JLINK.md)
- [STM32H7S3 errata](./Docs/development/errata.md)

## Public Website

Visit the [official JUMBLEQ website](https://jumbleq.io/) for an overview of the product, its features, controls, and specifications.

The Astro source is maintained in [`Site`](./Site/) and deployed to GitHub Pages by [the Pages workflow](./.github/workflows/pages.yml).

## Repository Contents

- [`Docs`](./Docs/) — user guides, technical reference, and development notes
- [`Site`](./Site/) — Astro source for the public JUMBLEQ website
- [`STM32CubeIDE`](./STM32CubeIDE/) — MCU application, bootloader, and external memory loader projects
- [`SigmaDSP`](./SigmaDSP/) — ADAU1466 DSP project and exported code
- [`Max`](./Max/) — alternative Max Standalone and Max for Live versions of JUMBLEQ Configurator
- [`Plasticity`](./Plasticity/) — enclosure CAD data and printable parts
- [`Logo`](./Logo/) — JUMBLEQ logo and icon artwork
- [`UI_Design`](./UI_Design/) — panel and control-layout artwork

## Related Repositories

- [JUMBLEQ Configurator](https://github.com/yamamo2shun1/JUMBLEQ-Configurator) — browser-based JUMBLEQ Configurator

## Gallery

- [Beta Prototype](./Gallery/beta_prototype.md)
- [Alpha Prototype](./Gallery/alpha_prototype.md)
- [Functional Prototype](./Gallery/functional_prototype.md)
- [Early Prototype](./Gallery/early_prototype.md)
- [Proof of Concept](./Gallery/proof_of_concept.md)

## License

Unless otherwise noted below, this project is licensed under the [MIT License](./LICENSE.md).

- Hardware design sources in [`Plasticity`](./Plasticity/) are licensed under the [CERN Open Hardware Licence Version 2 – Permissive](./Plasticity/LICENSE) (`CERN-OHL-P-2.0`).
- Panel and control-layout design sources in [`UI_Design`](./UI_Design/) are licensed under the [CERN Open Hardware Licence Version 2 – Permissive](./UI_Design/LICENSE) (`CERN-OHL-P-2.0`).
