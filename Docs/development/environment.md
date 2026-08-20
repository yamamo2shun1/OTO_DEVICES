# Development Environment

The project currently uses the following development tools and packages:

- IDE: [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 2.2.0
- Code generation and configuration: [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) 6.18.1
- Toolchain: GNU Tools for STM32 14.3.rel1.20251027-0700 (GCC 14.3.1)
- UF2 conversion: [Python](https://www.python.org/) 3.13.14
- MCU firmware package: STM32CubeH7RS v1.3.0
- DSP development: [SigmaStudio+](https://www.analog.com/en/resources/evaluation-hardware-and-software/embedded-development-software/sigmastudio-plus.html) 3.4.0
- Web Configurator runtime: [Node.js](https://nodejs.org/) 22.13.0 or later
- Web Configurator framework: [Next.js](https://nextjs.org/) 16.3.1 and [React](https://react.dev/) 19.2.6
- Web Configurator development: [TypeScript](https://www.typescriptlang.org/) 5.9.3 and [Tailwind CSS](https://tailwindcss.com/) 4.3.3
- Web Configurator validation: [ESLint](https://eslint.org/) 9.39.4 and [Playwright](https://playwright.dev/) 1.62.1 or later
- Legacy Configurator development: [Max](https://cycling74.com/products/max-9) 9.1.5

See the [STM32CubeIDE project overview](../../STM32CubeIDE/README.md) for the firmware projects included in this repository. For Web Configurator setup, development, testing, and deployment, see the [JUMBLEQ Configurator README](../../WebConfigurator/README.md).
