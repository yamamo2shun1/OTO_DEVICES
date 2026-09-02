# JUMBLEQ SigmaStudio+ Project

This directory contains the SigmaStudio+ project and generated files for the JUMBLEQ ADAU1466 DSP.

## Updating the DSP Project

After modifying [`JUMBLEQ_DSP.ssprj`](./JUMBLEQ_DSP.ssprj), perform the following operations in SigmaStudio+:

1. Run **Link Compile**.
2. Run **Export System Files**.

The exported files are written to [`exported_code`](./exported_code/).

## Integrating Changes into JUMBLEQ

To include the updated DSP project in JUMBLEQ, copy the following generated header files from [`exported_code`](./exported_code/) to [`STM32CubeIDE/JUMBLEQ/Appli/Core/Inc`](../STM32CubeIDE/JUMBLEQ/Appli/Core/Inc/), replacing the existing files:

- `JUMBLEQ_DSP_ADAU146xSchematic_1.h`
- `JUMBLEQ_DSP_ADAU146xSchematic_1_Defines.h`
- `JUMBLEQ_DSP_ADAU146xSchematic_1_PARAM.h`
